/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

namespace
{
constexpr int32_t kDuctedFanAirframe = 16;
constexpr int32_t kDuctedFanTailsitterVtolAirframe = 17;
constexpr unsigned kRateIndiRcChannel = 11; // RC12
constexpr hrt_abstime kRcSignalTimeout = 500_ms;

bool rcChannelEnabled(const rc_channels_s &rc_channels, unsigned channel)
{
	return (rc_channels.channel_count > channel)
	       && !rc_channels.signal_lost
	       && (hrt_elapsed_time(&rc_channels.timestamp) < kRcSignalTimeout)
	       && (rc_channels.channels[channel] >= 0.f);
}

bool indiAllocationFeedbackSupported(int32_t airframe)
{
	return airframe == 0 || airframe == 9 || airframe == kDuctedFanAirframe
	       || airframe == kDuctedFanTailsitterVtolAirframe;
}

uint8_t torqueAllocationInstance(int32_t airframe)
{
	return (airframe == kDuctedFanAirframe || airframe == kDuctedFanTailsitterVtolAirframe) ? 1 : 0;
}

bool allocationFeedbackValid(const allocation_value_s &allocation_value)
{
	if (allocation_value.timestamp == 0 || hrt_elapsed_time(&allocation_value.timestamp) >= 100_ms) {
		return false;
	}

	return Vector3f(allocation_value.allocated_torque).isAllFinite()
	       && Vector3f(allocation_value.torque_setpoint_scale).isAllFinite();
}
} // namespace

ModuleBase::Descriptor MulticopterRateControl::desc{task_spawn, custom_command, print_usage};

MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_thrust_setpoint_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (_route_torque_to_instance1) {
		// Reserve instance 0 first, then verify that the CA16 torque publisher owns
		// instance 1. The allocator callback remains on instance 0.
		if (!_vehicle_torque_setpoint_pub.advertise()
		    || _vehicle_torque_setpoint1_pub.get_instance() != 1) {
			PX4_ERR("failed to configure CA16 torque routing");
			return false;
		}
	}

	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());

	_indi_control.setParams(Vector3f(_param_mc_indi_roll_p.get(), _param_mc_indi_pitch_p.get(),
					 _param_mc_indi_yaw_p.get()),
				Vector3f(_param_mc_j_x.get(), _param_mc_j_y.get(), _param_mc_j_z.get()));
	const int32_t airframe = _param_ca_airframe.get();
	_torque_allocation_instance = torqueAllocationInstance(airframe);
	_route_torque_to_instance1 = airframe == kDuctedFanAirframe;
	_indi_capable = indiAllocationFeedbackSupported(airframe) && _indi_control.paramsValid();
}

void
MulticopterRateControl::publishTorqueSetpoint(const vehicle_torque_setpoint_s &vehicle_torque_setpoint)
{
	if (_route_torque_to_instance1) {
		// CA16 matrix 0 produces force only. Publish the complete MC torque and
		// optional PCA priority split on matrix 1 before instance 0 triggers the
		// allocator, matching the explicit routing used by CA17.
		_vehicle_torque_setpoint1_pub.publish(vehicle_torque_setpoint);

		vehicle_torque_setpoint_s force_matrix_torque_setpoint{};
		force_matrix_torque_setpoint.timestamp_sample = vehicle_torque_setpoint.timestamp_sample;
		force_matrix_torque_setpoint.timestamp = vehicle_torque_setpoint.timestamp;
		_vehicle_torque_setpoint_pub.publish(force_matrix_torque_setpoint);

	} else {
		_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);
	}
}

bool
MulticopterRateControl::computeIndiTorqueSetpoint(const Vector3f &rates, const Vector3f &rates_setpoint,
		const Vector3f &angular_accel, hrt_abstime reference_timestamp, Vector3f &torque_setpoint,
		Vector3f &indi_feedback)
{
	Vector3f allocated_torque;
	Vector3f output_scale;

	if (!getDelayedAllocatedTorque(reference_timestamp, allocated_torque, output_scale)) {
		return false;
	}

	const IndiControl::Output physical_output =
		_indi_control.update(rates, rates_setpoint, angular_accel, allocated_torque);

	indi_feedback = output_scale.emult(physical_output.feedback_torque);
	torque_setpoint = output_scale.emult(physical_output.rate_error_torque + physical_output.feedback_torque);
	return true;
}

void
MulticopterRateControl::updateAllocatedTorqueHistory()
{
	allocation_value_s allocation_value;

	if (!_allocation_value_sub.copy(&allocation_value) || !allocationFeedbackValid(allocation_value)
	    || allocation_value.timestamp <= _indi_torque_history_last_timestamp) {
		return;
	}

	if (_indi_torque_history_last_timestamp != 0
	    && allocation_value.timestamp > _indi_torque_history_last_timestamp + 100_ms) {
		_indi_torque_history.reset();
	}

	const Vector3f torque_setpoint_scale(allocation_value.torque_setpoint_scale);

	// Do not interpolate feedback across an effectiveness/allocation-scale change.
	if (!_indi_torque_history.empty()
	    && (torque_setpoint_scale - _indi_torque_history.get_newest().torque_setpoint_scale).norm_squared() > 1e-8f) {
		_indi_torque_history.reset();
	}

	IndiTorqueSample sample{};
	sample.time_us = allocation_value.timestamp;
	sample.allocated_torque = Vector3f(allocation_value.allocated_torque);
	sample.torque_setpoint_scale = torque_setpoint_scale;
	_indi_torque_history.push(sample);
	_indi_torque_history_last_timestamp = sample.time_us;
}

bool
MulticopterRateControl::getDelayedAllocatedTorque(hrt_abstime reference_timestamp, Vector3f &allocated_torque,
		Vector3f &torque_setpoint_scale)
{
	if (_indi_torque_history.empty()
	    || hrt_elapsed_time(&_indi_torque_history.get_newest().time_us) >= 100_ms) {
		return false;
	}

	const float delay_s = PX4_ISFINITE(_param_mc_indi_torque_delay.get())
			      ? math::constrain(_param_mc_indi_torque_delay.get(), 0.f, 0.1f) : 0.f;
	const hrt_abstime delay_us = static_cast<hrt_abstime>(delay_s * 1e6f);
	const hrt_abstime target_timestamp = reference_timestamp > delay_us ? reference_timestamp - delay_us : 0;
	const IndiTorqueSample &newest = _indi_torque_history.get_newest();

	if (target_timestamp >= newest.time_us) {
		allocated_torque = newest.allocated_torque;
		torque_setpoint_scale = newest.torque_setpoint_scale;
		return allocated_torque.isAllFinite() && torque_setpoint_scale.isAllFinite();
	}

	IndiTorqueSample older{};
	IndiTorqueSample newer{};
	bool older_valid = false;
	bool newer_valid = false;
	uint8_t index = _indi_torque_history.get_oldest_index();

	for (int entry = 0; entry < _indi_torque_history.entries(); entry++) {
		const IndiTorqueSample &sample = _indi_torque_history[index];

		if (sample.time_us <= target_timestamp) {
			older = sample;
			older_valid = true;
		}

		if (sample.time_us >= target_timestamp) {
			newer = sample;
			newer_valid = true;
			break;
		}

		index = _indi_torque_history.next(index);
	}

	if (!older_valid) {
		return false;
	}

	allocated_torque = older.allocated_torque;
	torque_setpoint_scale = older.torque_setpoint_scale;

	if (newer_valid && newer.time_us > older.time_us) {
		const float interpolation = static_cast<float>(target_timestamp - older.time_us)
					    / static_cast<float>(newer.time_us - older.time_us);
		allocated_torque += (newer.allocated_torque - older.allocated_torque) * interpolation;
		torque_setpoint_scale += (newer.torque_setpoint_scale - older.torque_setpoint_scale) * interpolation;
	}

	return allocated_torque.isAllFinite() && torque_setpoint_scale.isAllFinite();
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		/* check for updates in other topics */
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);
		_rc_channels_sub.update(&_rc_channels);
		// Keep the filtered allocated-torque history warm even while PID is active,
		// so a requested delayed sample is immediately available on INDI entry.
		updateAllocatedTorqueHistory();

		// use rates setpoint topic
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
				_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
				_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

				// publish rate setpoint
				vehicle_rates_setpoint.roll = _rates_setpoint(0);
				vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				vehicle_rates_setpoint.yaw = _rates_setpoint(2);
				_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				vehicle_rates_setpoint.timestamp = hrt_absolute_time();

				_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
			}

		} else if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
			_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
			_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
			_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
		}

		// run the rate controller
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			// reset integral if disarmed
			if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// update saturation status from control allocation feedback
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_subs[_torque_allocation_instance].update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			// Keep the original PID controller running as a hot standby.
			Vector3f torque_setpoint =
				_rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);
			Vector3f indi_feedback{};
			const bool indi_flight_enabled = _vehicle_control_mode.flag_armed
						 && _vehicle_status.takeoff_time != 0 && !_landed;
			const bool indi_requested = (_param_mc_indi_rate_en.get() == 1)
						    || rcChannelEnabled(_rc_channels, kRateIndiRcChannel);
			const bool indi_active = _indi_capable && indi_requested && indi_flight_enabled
						 && computeIndiTorqueSetpoint(rates, _rates_setpoint, angular_accel, now,
							 torque_setpoint, indi_feedback);
			const Vector3f torque_setpoint_before_output_processing = torque_setpoint;

			// apply low-pass filtering on yaw axis to reduce high frequency torque caused by rotor acceleration
			torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.indi_active = indi_active;
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;

			// scale setpoints by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			if (indi_active) {
				vehicle_torque_setpoint.xyz_indi_feedback_valid = true;

				for (int axis = 0; axis < 3; axis++) {
					const float total = torque_setpoint_before_output_processing(axis);
					vehicle_torque_setpoint.xyz_indi_feedback[axis] = fabsf(total) > FLT_EPSILON ?
							indi_feedback(axis) * vehicle_torque_setpoint.xyz[axis] / total : 0.f;
				}
			}

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = hrt_absolute_time();
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = hrt_absolute_time();
			publishTorqueSetpoint(vehicle_torque_setpoint);

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);

		}
	}

	perf_end(_loop_perf);
}

void MulticopterRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
		float dt)
{
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += vehicle_torque_setpoint.xyz[i] * vehicle_torque_setpoint.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = vehicle_torque_setpoint.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return ModuleBase::main(MulticopterRateControl::desc, argc, argv);
}
