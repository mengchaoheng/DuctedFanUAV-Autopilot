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

/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

#include "DfHoverRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>
#include <inttypes.h>
#include <time.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

namespace
{
hrt_abstime runtimeMeasurementTimeUs()
{
#if defined(__PX4_POSIX)
	timespec ts {};
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return static_cast<hrt_abstime>(ts.tv_sec) * 1000000ULL + static_cast<hrt_abstime>(ts.tv_nsec) / 1000ULL;
#else
	return hrt_absolute_time();
#endif
}

bool hasTorqueAuthority(const allocation_value_s &allocation_value)
{
	if (!allocation_value.feedback_valid) {
		return false;
	}

	for (int row = 0; row < 3; row++) {
		bool axis_has_authority = false;

		for (unsigned actuator = 0; actuator < allocation_value.u_dim; actuator++) {
			const float b = allocation_value.b[row * allocation_value_s::MAX_U + actuator];

			if (PX4_ISFINITE(b) && fabsf(b) > 1.e-4f) {
				axis_has_authority = true;
				break;
			}
		}

		if (!axis_has_authority) {
			return false;
		}
	}

	return true;
}

bool isRecentAllocationValue(const allocation_value_s &allocation_value)
{
	return allocation_value.timestamp != 0 && hrt_elapsed_time(&allocation_value.timestamp) < 500_ms;
}

bool isRecentControlAllocatorStatus(const control_allocator_status_s &status)
{
	return status.timestamp != 0 && hrt_elapsed_time(&status.timestamp) < 500_ms;
}

const char *rateControlMethodName(uint8_t method)
{
	switch (method) {
	case rate_ctrl_status_s::METHOD_INDI:
		return "INDI";

	case rate_ctrl_status_s::METHOD_RATE_CONTROL:
	default:
		return "RateControl";
	}
}
} // namespace

ModuleBase::Descriptor DfHoverRateControl::desc{task_spawn, custom_command, print_usage};

DfHoverRateControl::DfHoverRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_vehicle_thrust_setpoint0_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_thrust_setpoint1_pub(vtol ? ORB_ID(vehicle_thrust_setpoint_virtual_mc) : ORB_ID(vehicle_thrust_setpoint)),
	_vehicle_torque_setpoint0_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_vehicle_torque_setpoint1_pub(vtol ? ORB_ID(vehicle_torque_setpoint_virtual_mc) : ORB_ID(vehicle_torque_setpoint)),
	_vtol(vtol),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
	_indi_control_status_pub.advertise();
	_vehicle_thrust_setpoint0_pub.advertise();
	_vehicle_torque_setpoint0_pub.advertise();

	if (!_vtol) {
		_vehicle_thrust_setpoint1_pub.advertise();
		_vehicle_torque_setpoint1_pub.advertise();
	}
}

DfHoverRateControl::~DfHoverRateControl()
{
	perf_free(_loop_perf);
}

bool
DfHoverRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
DfHoverRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_df_rollrate_k.get(), _param_df_pitchrate_k.get(), _param_df_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_df_rollrate_p.get(), _param_df_pitchrate_p.get(), _param_df_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_df_rollrate_i.get(), _param_df_pitchrate_i.get(), _param_df_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_df_rollrate_d.get(), _param_df_pitchrate_d.get(), _param_df_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_df_rr_int_lim.get(), _param_df_pr_int_lim.get(), _param_df_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_df_rollrate_ff.get(), _param_df_pitchrate_ff.get(), _param_df_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_df_acro_r_max.get()), radians(_param_df_acro_p_max.get()),
				  radians(_param_df_acro_y_max.get()));

	_output_lpf_yaw.setCutoffFreq(_param_df_yaw_tq_cutoff.get());

	_indi_control.setParams(Vector3f(_param_df_indi_roll_p.get(), _param_df_indi_pitch_p.get(),
					 _param_df_indi_yaw_p.get()));
	_use_indi = _param_df_use_indi.get() == 1;
	_use_tau_i = _param_df_use_tau_i.get() == 1;
	_use_u = _param_df_use_u.get() == 1;
	_indi_waiting = false;
}

void
DfHoverRateControl::Run()
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

		// use rates setpoint topic
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};

		if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.roll, _param_df_acro_expo.get(), _param_df_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.pitch, _param_df_acro_expo.get(), _param_df_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.yaw, _param_df_acro_expo_y.get(), _param_df_acro_supexpoy.get())};

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
			if (_vehicle_rates_setpoint_sub.copy(&vehicle_rates_setpoint)) {
				_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
				_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
				_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
				_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
			}
		}

		// run the rate controller
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			// reset integral if disarmed
			if (!_vehicle_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// run rate controller
			_allocation_value_sub.update(&_allocation_value);
			const bool use_allocation_value = hasTorqueAuthority(_allocation_value) && isRecentAllocationValue(_allocation_value);

			// update saturation status from the torque allocation instance
			_control_allocator_status_sub.update(&_control_allocator_status);
			const control_allocator_status_s *control_allocator_status = nullptr;

			if (use_allocation_value && isRecentControlAllocatorStatus(_control_allocator_status)) {
				control_allocator_status = &_control_allocator_status;
			}

			if (control_allocator_status != nullptr) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status->torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status->unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status->unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			Vector3f indi_fb;
			Vector3f error_fb;
			Vector3f torque_setpoint;
			Vector3f torque_setpoint_indi_physical;
			Vector3f torque_setpoint_rate_error_feedback;
			Vector3f torque_setpoint_indi_feedback;
			matrix::Vector<float, allocation_value_s::MAX_Y> indi_output_scale;
			indi_fb.setZero();
			error_fb.setZero();
			torque_setpoint.setZero();
			torque_setpoint_indi_physical.setZero();
			torque_setpoint_rate_error_feedback.setZero();
			torque_setpoint_indi_feedback.setZero();
			indi_output_scale.setAll(1.f);
			float indi_dt = 0.f;
			bool control_flag = false;
			uint8_t rate_control_method = rate_ctrl_status_s::METHOD_RATE_CONTROL;

			const hrt_abstime timestamp_controller_start = runtimeMeasurementTimeUs();
			const bool allocation_value_valid = hasTorqueAuthority(_allocation_value) && isRecentAllocationValue(_allocation_value);

			if (_use_indi && allocation_value_valid) {
				const hrt_abstime now_indi = hrt_absolute_time();

				if (_last_indi_run != 0) {
					indi_dt = math::constrain((now_indi - _last_indi_run) * 1e-6f, 0.0001f, 0.02f);
				}

				_last_indi_run = now_indi;
				_rate_control.resetIntegral();
				error_fb = _indi_control.update(rates, _rates_setpoint, angular_accel, _allocation_value, indi_fb,
								_maybe_landed || _landed, _use_u, _use_tau_i);
				torque_setpoint_indi_physical = error_fb + indi_fb;
				torque_setpoint = torque_setpoint_indi_physical;

				for (int axis = 0; axis < allocation_value_s::MAX_Y; axis++) {
					if (PX4_ISFINITE(_allocation_value.control_allocation_scale[axis])
					    && fabsf(_allocation_value.control_allocation_scale[axis]) > FLT_EPSILON) {
						indi_output_scale(axis) = _allocation_value.control_allocation_scale[axis];
					}
				}

				// Keep INDI in physical coordinates, then apply the same row scaling
				// used by control allocation: v_norm = D * v_phys.
				const Vector3f indi_torque_output_scale(indi_output_scale(0), indi_output_scale(1), indi_output_scale(2));
				torque_setpoint_rate_error_feedback = indi_torque_output_scale.emult(error_fb);
				torque_setpoint_indi_feedback = indi_torque_output_scale.emult(indi_fb);
				torque_setpoint = torque_setpoint_rate_error_feedback + torque_setpoint_indi_feedback;
				_last_indi_torque_physical = torque_setpoint_indi_physical;
				_last_indi_torque_normalized = torque_setpoint;
				_last_indi_status_valid = true;
				control_flag = true;
				rate_control_method = rate_ctrl_status_s::METHOD_INDI;
				_indi_waiting = false;

			} else {
				_indi_waiting = _use_indi && !allocation_value_valid;
				torque_setpoint = _rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);
				torque_setpoint_indi_physical = torque_setpoint;
				_last_indi_run = 0;
			}

			_rate_control_running_time_us = runtimeMeasurementTimeUs() - timestamp_controller_start;

			if (rate_control_method != _rate_control_method) {
				_rate_control_running_time_avg_us = 0.f;
				_rate_control_running_time_count = 0;
				_rate_control_method = rate_control_method;
			}

			if (_rate_control_running_time_count < UINT32_MAX) {
				_rate_control_running_time_count++;
				_rate_control_running_time_avg_us += (static_cast<float>(_rate_control_running_time_us) -
								       _rate_control_running_time_avg_us) /
								      static_cast<float>(_rate_control_running_time_count);
			}

			// apply low-pass filtering on yaw axis to reduce high frequency torque caused by rotor acceleration
			if (!control_flag) {
				torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);
			}

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			rate_ctrl_status.method = _rate_control_method;
			rate_ctrl_status.rate_control_running_time = static_cast<float>(_rate_control_running_time_us);
			rate_ctrl_status.rate_control_running_time_avg = _rate_control_running_time_avg_us;
			rate_ctrl_status.rate_control_running_time_samples = _rate_control_running_time_count;
			_controller_status_pub.publish(rate_ctrl_status);

			// publish thrust and torque setpoints
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;
			vehicle_torque_setpoint.xyz_split_valid = control_flag;

			for (int i = 0; i < 3; i++) {
				vehicle_torque_setpoint.xyz_rate_error_feedback[i] =
					(control_flag && PX4_ISFINITE(torque_setpoint_rate_error_feedback(i))) ?
					torque_setpoint_rate_error_feedback(i) : 0.f;
				vehicle_torque_setpoint.xyz_indi_feedback[i] =
					(control_flag && PX4_ISFINITE(torque_setpoint_indi_feedback(i))) ?
					torque_setpoint_indi_feedback(i) : 0.f;
			}

			indi_control_status_s indi_control_status{};
			indi_control_status.timestamp_sample = angular_velocity.timestamp_sample;
			indi_control_status.control_flag = control_flag;
			indi_control_status.enabled = _use_indi;
			indi_control_status.allocation_valid = allocation_value_valid;
			indi_control_status.u_dim = _allocation_value.u_dim;
			indi_control_status.indi_dt = indi_dt;

			for (int i = 0; i < 3; i++) {
				indi_control_status.error_fb[i] = PX4_ISFINITE(error_fb(i)) ? error_fb(i) : 0.f;
				indi_control_status.indi_fb[i] = PX4_ISFINITE(indi_fb(i)) ? indi_fb(i) : 0.f;
				indi_control_status.output[i] = PX4_ISFINITE(torque_setpoint_indi_physical(i)) ? torque_setpoint_indi_physical(i) : 0.f;
				indi_control_status.output_normalized[i] = PX4_ISFINITE(torque_setpoint(i)) ? torque_setpoint(i) : 0.f;
				indi_control_status.control_allocation_scale[i] = indi_output_scale(i);
			}

			// scale setpoints by battery status if enabled
			if (_param_df_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						const float torque_scaled = vehicle_torque_setpoint.xyz[i] * _battery_status_scale;
						const float torque_limited = math::constrain(torque_scaled, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] = torque_limited;

						if (vehicle_torque_setpoint.xyz_split_valid) {
							const float rate_error_scaled = vehicle_torque_setpoint.xyz_rate_error_feedback[i] * _battery_status_scale;
							const float indi_scaled = vehicle_torque_setpoint.xyz_indi_feedback[i] * _battery_status_scale;
							const float split_sum = rate_error_scaled + indi_scaled;
							const float split_limit_scale = (fabsf(split_sum) > FLT_EPSILON) ? torque_limited / split_sum : 1.f;

							vehicle_torque_setpoint.xyz_rate_error_feedback[i] = rate_error_scaled * split_limit_scale;
							vehicle_torque_setpoint.xyz_indi_feedback[i] = indi_scaled * split_limit_scale;
						}
					}
				}
			}

			const hrt_abstime setpoint_timestamp = hrt_absolute_time();
			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = setpoint_timestamp;
			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = setpoint_timestamp;

			if (_vtol) {
				_vehicle_thrust_setpoint0_pub.publish(vehicle_thrust_setpoint);
				_vehicle_torque_setpoint0_pub.publish(vehicle_torque_setpoint);

			} else {
				vehicle_thrust_setpoint_s vehicle_thrust_setpoint1{};
				vehicle_torque_setpoint_s vehicle_torque_setpoint0{};

				vehicle_thrust_setpoint1.timestamp_sample = angular_velocity.timestamp_sample;
				vehicle_thrust_setpoint1.timestamp = setpoint_timestamp;
				vehicle_torque_setpoint0.timestamp_sample = angular_velocity.timestamp_sample;
				vehicle_torque_setpoint0.timestamp = setpoint_timestamp;

				_vehicle_thrust_setpoint0_pub.publish(vehicle_thrust_setpoint);
				_vehicle_thrust_setpoint1_pub.publish(vehicle_thrust_setpoint1);
				_vehicle_torque_setpoint1_pub.publish(vehicle_torque_setpoint);
				_vehicle_torque_setpoint0_pub.publish(vehicle_torque_setpoint0);
			}

			indi_control_status.timestamp = vehicle_torque_setpoint.timestamp;
			_indi_control_status_pub.publish(indi_control_status);

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);

		}
	}

	perf_end(_loop_perf);
}

void DfHoverRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint,
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

int DfHoverRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	DfHoverRateControl *instance = new DfHoverRateControl(vtol);

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

int DfHoverRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DfHoverRateControl::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("INDI rate control: enabled=%d active=%d waiting=%d use_u=%d use_tau_i=%d allocation_valid=%d u_dim=%u",
		 (int)_use_indi, (int)_last_indi_status_valid, (int)_indi_waiting, (int)_use_u, (int)_use_tau_i,
		 (int)(hasTorqueAuthority(_allocation_value) && isRecentAllocationValue(_allocation_value)), (unsigned)_allocation_value.u_dim);
	PX4_INFO("Rate control method: %s time_us=%" PRIu64 " avg_us=%.2f samples=%" PRIu32,
		 rateControlMethodName(_rate_control_method), _rate_control_running_time_us,
		 (double)_rate_control_running_time_avg_us, _rate_control_running_time_count);

	if (_allocation_value.u_dim > 0) {
		PX4_INFO("INDI allocation: D_torque=%.4g %.4g %.4g D_thrust=%.4g %.4g %.4g U=%.4g %.4g %.4g",
			 (double)_allocation_value.control_allocation_scale[0],
			 (double)_allocation_value.control_allocation_scale[1],
			 (double)_allocation_value.control_allocation_scale[2],
			 (double)_allocation_value.control_allocation_scale[3],
			 (double)_allocation_value.control_allocation_scale[4],
			 (double)_allocation_value.control_allocation_scale[5],
			 (double)_allocation_value.actuator_scale[0],
			 (double)(_allocation_value.u_dim > 1 ? _allocation_value.actuator_scale[1] : 0.f),
			 (double)(_allocation_value.u_dim > 2 ? _allocation_value.actuator_scale[2] : 0.f));
	}

	if (_last_indi_status_valid) {
		PX4_INFO("INDI last output: v_phys=%.4g %.4g %.4g v_norm=%.4g %.4g %.4g",
			 (double)_last_indi_torque_physical(0), (double)_last_indi_torque_physical(1),
			 (double)_last_indi_torque_physical(2),
			 (double)_last_indi_torque_normalized(0), (double)_last_indi_torque_normalized(1),
			 (double)_last_indi_torque_normalized(2));
	}

	perf_print_counter(_loop_perf);
	return 0;
}

int DfHoverRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the ducted fan hover rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has the standard PID loop and an optional INDI loop for angular rate control.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("df_hover_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int df_hover_rate_control_main(int argc, char *argv[])
{
	return ModuleBase::main(DfHoverRateControl::desc, argc, argv);
}
