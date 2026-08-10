/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "MulticopterPositionControl.hpp"

#include <geo/geo.h>

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>
#include "PositionControl/ControlMath.hpp"

using namespace matrix;

namespace
{
constexpr unsigned kAccelerationIndiRcChannel = 10; // RC11
constexpr hrt_abstime kRcSignalTimeout = 500_ms;
constexpr uint8_t kIndiAccelerationVelocityValid = 1u << 0;
constexpr uint8_t kIndiAccelerationEkfValid = 1u << 1;
constexpr uint8_t kIndiAccelerationImuValid = 1u << 2;
constexpr hrt_abstime kIndiAccelerationSampleMaxAge = 50_ms;

bool rcChannelEnabled(const rc_channels_s &rc_channels, unsigned channel)
{
	return (rc_channels.channel_count > channel)
	       && !rc_channels.signal_lost
	       && (hrt_elapsed_time(&rc_channels.timestamp) < kRcSignalTimeout)
	       && (rc_channels.channels[channel] >= 0.f);
}

bool indiAllocationFeedbackSupported(int32_t airframe)
{
	return airframe == 0 || airframe == 9 || airframe == 16 || airframe == 17;
}

bool allocationForceValid(const allocation_value_s &allocation_value)
{
	if (allocation_value.timestamp == 0 || hrt_elapsed_time(&allocation_value.timestamp) >= 100_ms) {
		return false;
	}

	return Vector3f(allocation_value.allocated_force).isAllFinite();
}
} // namespace

ModuleBase::Descriptor MulticopterPositionControl::desc{task_spawn, custom_command, print_usage};

MulticopterPositionControl::MulticopterPositionControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint))
{
	_sample_interval_s.update(0.01f); // 100 Hz default
	parameters_update(true);
	_indi_hover_thrust = _param_mpc_thr_hover.get();
	_indi_hover_thrust_target = _indi_hover_thrust;
	_tilt_limit_slew_rate.setSlewRate(.2f);
	_takeoff_status_pub.advertise();
}

MulticopterPositionControl::~MulticopterPositionControl()
{
	perf_free(_cycle_perf);
}

bool MulticopterPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void MulticopterPositionControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		const float mass = _param_mpc_mass.get();
		const bool mass_valid = PX4_ISFINITE(mass) && mass > FLT_EPSILON;
		const bool allocation_feedback_supported = indiAllocationFeedbackSupported(_param_ca_airframe.get());
		const bool normalized_force_feedback = _param_mpc_indi_f_src.get() == 1;
		_indi_inverse_mass = mass_valid ? 1.f / mass : 0.f;
		_indi_capable = allocation_feedback_supported && (normalized_force_feedback || mass_valid);

		float sample_freq_hz = 1.f / _sample_interval_s.mean();

		// velocity notch filter
		if ((_param_mpc_vel_nf_frq.get() > 0.f) && (_param_mpc_vel_nf_bw.get() > 0.f)) {
			_vel_xy_notch_filter.setParameters(sample_freq_hz, _param_mpc_vel_nf_frq.get(), _param_mpc_vel_nf_bw.get());
			_vel_z_notch_filter.setParameters(sample_freq_hz, _param_mpc_vel_nf_frq.get(), _param_mpc_vel_nf_bw.get());

		} else {
			_vel_xy_notch_filter.disable();
			_vel_z_notch_filter.disable();
		}

		// velocity xy/z low pass filter
		if (_param_mpc_vel_lp.get() > 0.f) {
			_vel_xy_lp_filter.set_cutoff_frequency(sample_freq_hz, _param_mpc_vel_lp.get());
			_vel_z_lp_filter.set_cutoff_frequency(sample_freq_hz, _param_mpc_vel_lp.get());
			_vel_xy_lp_filter.reset(_vel_xy_filtered);
			_vel_z_lp_filter.reset(_vel_z_filtered);

		} else {
			// disable filtering
			_vel_xy_lp_filter.disable();
			_vel_z_lp_filter.disable();
		}

		// velocity derivative xy/z low pass filter
		if (_param_mpc_veld_lp.get() > 0.f) {
			_vel_deriv_xy_lp_filter.set_cutoff_frequency(sample_freq_hz, _param_mpc_veld_lp.get());
			_vel_deriv_z_lp_filter.set_cutoff_frequency(sample_freq_hz, _param_mpc_veld_lp.get());
			_vel_deriv_xy_lp_filter.reset(_vel_deriv_xy_filtered);
			_vel_deriv_z_lp_filter.reset(_vel_deriv_z_filtered);

		} else {
			// disable filtering
			_vel_deriv_xy_lp_filter.disable();
			_vel_deriv_z_lp_filter.disable();
		}

		// The direct EKF and IMU acceleration candidates use identical independent
		// filters so they can be compared and selected without sharing filter state.
		if (_param_mpc_indi_a_lp.get() > 0.f) {
			_indi_acceleration_ekf_lp_filter.set_cutoff_frequency(sample_freq_hz, _param_mpc_indi_a_lp.get());
			_indi_acceleration_imu_lp_filter.set_cutoff_frequency(sample_freq_hz, _param_mpc_indi_a_lp.get());

		} else {
			_indi_acceleration_ekf_lp_filter.disable();
			_indi_acceleration_imu_lp_filter.disable();
		}

		_indi_acceleration_ekf_filter_initialized = false;
		_indi_acceleration_imu_filter_initialized = false;



		int num_changed = 0;

		if (_param_sys_vehicle_resp.get() >= 0.f) {
			// make it less sensitive at the lower end
			float responsiveness = _param_sys_vehicle_resp.get() * _param_sys_vehicle_resp.get();

			num_changed += _param_mpc_acc_hor.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_hor_max.commit_no_notification(math::lerp(2.f, 15.f, responsiveness));
			num_changed += _param_mpc_man_y_max.commit_no_notification(math::lerp(80.f, 450.f, responsiveness));

			if (responsiveness > 0.6f) {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(0.f);

			} else {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(math::lerp(0.5f, 0.f, responsiveness / 0.6f));
			}

			if (responsiveness < 0.5f) {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(45.f);

			} else {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(math::min(MAX_SAFE_TILT_DEG, math::lerp(45.f, 70.f,
						(responsiveness - 0.5f) * 2.f)));
			}

			num_changed += _param_mpc_acc_down_max.commit_no_notification(math::lerp(0.8f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_up_max.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_jerk_max.commit_no_notification(math::lerp(2.f, 50.f, responsiveness));
			num_changed += _param_mpc_jerk_auto.commit_no_notification(math::lerp(1.f, 25.f, responsiveness));
		}

		if (_param_mpc_xy_vel_all.get() >= 0.f) {
			float xy_vel = _param_mpc_xy_vel_all.get();
			num_changed += _param_mpc_vel_manual.commit_no_notification(xy_vel);
			num_changed += _param_mpc_vel_man_back.commit_no_notification(-1.f);
			num_changed += _param_mpc_vel_man_side.commit_no_notification(-1.f);
			num_changed += _param_mpc_xy_cruise.commit_no_notification(xy_vel);
			num_changed += _param_mpc_xy_vel_max.commit_no_notification(xy_vel);
		}

		if (_param_mpc_z_vel_all.get() >= 0.f) {
			float z_vel = _param_mpc_z_vel_all.get();
			num_changed += _param_mpc_z_v_auto_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_vel_max_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_v_auto_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_z_vel_max_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_tko_speed.commit_no_notification(z_vel * 0.6f);
			num_changed += _param_mpc_land_speed.commit_no_notification(z_vel * 0.5f);
		}

		if (num_changed > 0) {
			param_notify_changes();
		}

		if (_param_mpc_tiltmax_air.get() > MAX_SAFE_TILT_DEG) {
			_param_mpc_tiltmax_air.set(MAX_SAFE_TILT_DEG);
			_param_mpc_tiltmax_air.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Tilt constrained to safe value\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_AIR</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_tilt_set"), events::Log::Warning,
					    "Maximum tilt limit has been constrained to a safe value", MAX_SAFE_TILT_DEG);
		}

		if (_param_mpc_tiltmax_lnd.get() > _param_mpc_tiltmax_air.get()) {
			_param_mpc_tiltmax_lnd.set(_param_mpc_tiltmax_air.get());
			_param_mpc_tiltmax_lnd.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Land tilt has been constrained by max tilt\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_LND</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_land_tilt_set"), events::Log::Warning,
					    "Land tilt limit has been constrained by maximum tilt", _param_mpc_tiltmax_air.get());
		}

		_control.setPositionGains(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get()));
		_control.setVelocityGains(
			Vector3f(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get()),
			Vector3f(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get()),
			Vector3f(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get()));
		_control.setHorizontalThrustMargin(_param_mpc_thr_xy_marg.get());
		_control.setAccelerationIndiTransitionTime(_param_mpc_indi_tr_t.get());
		_control.decoupleHorizontalAndVecticalAcceleration(_param_mpc_acc_decouple.get());
		_goto_control.setParamMpcAccHor(_param_mpc_acc_hor.get());
		_goto_control.setParamMpcAccDownMax(_param_mpc_acc_down_max.get());
		_goto_control.setParamMpcAccUpMax(_param_mpc_acc_up_max.get());
		_goto_control.setParamMpcJerkAuto(_param_mpc_jerk_auto.get());
		_goto_control.setParamMpcXyCruise(_param_mpc_xy_cruise.get());
		_goto_control.setParamMpcXyErrMax(_param_mpc_xy_err_max.get());
		_goto_control.setParamMpcXyVelMax(_param_mpc_xy_vel_max.get());
		_goto_control.setParamMpcYawrautoMax(_param_mpc_yawrauto_max.get());
		_goto_control.setParamMpcYawrautoAcc(_param_mpc_yawrauto_acc.get());
		_goto_control.setParamMpcZVAutoDn(_param_mpc_z_v_auto_dn.get());
		_goto_control.setParamMpcZVAutoUp(_param_mpc_z_v_auto_up.get());

		// Check that the design parameters are inside the absolute maximum constraints
		if (_param_mpc_xy_cruise.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_xy_cruise.set(_param_mpc_xy_vel_max.get());
			_param_mpc_xy_cruise.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_XY_CRUISE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_cruise_set"), events::Log::Warning,
					    "Cruise speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		if (_param_mpc_vel_manual.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_vel_manual.set(_param_mpc_xy_vel_max.get());
			_param_mpc_vel_manual.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MANUAL</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_set"), events::Log::Warning,
					    "Manual speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		if (_param_mpc_vel_man_back.get() > _param_mpc_vel_manual.get()) {
			_param_mpc_vel_man_back.set(_param_mpc_vel_manual.get());
			_param_mpc_vel_man_back.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual backward speed has been constrained by forward speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MAN_BACK</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_back_set"), events::Log::Warning,
					    "Manual backward speed has been constrained by forward speed", _param_mpc_vel_manual.get());
		}

		if (_param_mpc_vel_man_side.get() > _param_mpc_vel_manual.get()) {
			_param_mpc_vel_man_side.set(_param_mpc_vel_manual.get());
			_param_mpc_vel_man_side.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual sideways speed has been constrained by forward speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MAN_SIDE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_side_set"), events::Log::Warning,
					    "Manual sideways speed has been constrained by forward speed", _param_mpc_vel_manual.get());
		}

		if (_param_mpc_z_v_auto_up.get() > _param_mpc_z_vel_max_up.get()) {
			_param_mpc_z_v_auto_up.set(_param_mpc_z_vel_max_up.get());
			_param_mpc_z_v_auto_up.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Ascent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_UP</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_up_vel_set"), events::Log::Warning,
					    "Ascent speed has been constrained by max speed", _param_mpc_z_vel_max_up.get());
		}

		if (_param_mpc_z_v_auto_dn.get() > _param_mpc_z_vel_max_dn.get()) {
			_param_mpc_z_v_auto_dn.set(_param_mpc_z_vel_max_dn.get());
			_param_mpc_z_v_auto_dn.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Descent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_DN</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_down_vel_set"), events::Log::Warning,
					    "Descent speed has been constrained by max speed", _param_mpc_z_vel_max_dn.get());
		}

		if (_param_mpc_thr_hover.get() > _param_mpc_thr_max.get() ||
		    _param_mpc_thr_hover.get() < _param_mpc_thr_min.get()) {
			_param_mpc_thr_hover.set(math::constrain(_param_mpc_thr_hover.get(), _param_mpc_thr_min.get(),
						 _param_mpc_thr_max.get()));
			_param_mpc_thr_hover.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max\t");
			/* EVENT
			 * @description <param>MPC_THR_HOVER</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_hover_thrust_set"), events::Log::Warning,
					    "Hover thrust has been constrained by min/max thrust", _param_mpc_thr_hover.get());
		}

		if (!_hover_thrust_initialized) {
			_control.setHoverThrust(_param_mpc_thr_hover.get());
			_hover_thrust_initialized = true;
		}

		// initialize vectors from params and enforce constraints
		_param_mpc_tko_speed.set(math::min(_param_mpc_tko_speed.get(), _param_mpc_z_vel_max_up.get()));
		_param_mpc_land_speed.set(math::min(_param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get()));

		_takeoff.setSpoolupTime(_param_com_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_mpc_z_vel_p_acc.get());
	}
}

PositionControlStates MulticopterPositionControl::set_vehicle_states(const vehicle_local_position_s
		&vehicle_local_position, const float dt_s)
{
	PositionControlStates states;

	const Vector2f position_xy(vehicle_local_position.x, vehicle_local_position.y);

	// only set position states if valid and finite
	if (vehicle_local_position.xy_valid && position_xy.isAllFinite()) {
		states.position.xy() = position_xy;

	} else {
		states.position(0) = states.position(1) = NAN;
	}

	if (PX4_ISFINITE(vehicle_local_position.z) && vehicle_local_position.z_valid) {
		states.position(2) = vehicle_local_position.z;

	} else {
		states.position(2) = NAN;
	}

	const Vector2f velocity_xy(vehicle_local_position.vx, vehicle_local_position.vy);

	if (vehicle_local_position.v_xy_valid && velocity_xy.isAllFinite()) {
		const Vector2f vel_xy_prev = _vel_xy_filtered;

		// vel xy notch filter, then low pass filter
		_vel_xy_filtered = _vel_xy_lp_filter.apply(_vel_xy_notch_filter.apply(velocity_xy));
		states.velocity.xy() = _vel_xy_filtered;

		// vel xy derivative low pass filter
		_vel_deriv_xy_filtered = _vel_deriv_xy_lp_filter.apply((_vel_xy_filtered - vel_xy_prev) / dt_s);
		states.acceleration.xy() = _vel_deriv_xy_filtered;

	} else {
		states.velocity(0) = states.velocity(1) = NAN;
		states.acceleration(0) = states.acceleration(1) = NAN;

		// reset filters to prevent acceleration spikes when regaining velocity
		_vel_xy_lp_filter.reset({});
		_vel_xy_filtered = {};
		_vel_xy_notch_filter.reset();
		_vel_deriv_xy_lp_filter.reset({});
		_vel_deriv_xy_filtered = {};
	}

	if (PX4_ISFINITE(vehicle_local_position.vz) && vehicle_local_position.v_z_valid) {

		const float vel_z_prev = _vel_z_filtered;

		// vel z notch filter, then low pass filter
		_vel_z_filtered = _vel_z_lp_filter.apply(_vel_z_notch_filter.apply(vehicle_local_position.vz));
		states.velocity(2) = _vel_z_filtered;

		// vel z derivative low pass filter
		_vel_deriv_z_filtered = _vel_deriv_z_lp_filter.apply((_vel_z_filtered - vel_z_prev) / dt_s);
		states.acceleration(2) = _vel_deriv_z_filtered;

	} else {
		states.velocity(2) = NAN;
		states.acceleration(2) = NAN;

		// reset filters to prevent acceleration spikes when regaining velocity
		_vel_z_lp_filter.reset({});
		_vel_z_filtered = 0.f;
		_vel_z_notch_filter.reset();
		_vel_deriv_z_lp_filter.reset({});
		_vel_deriv_z_filtered = 0.f;
	}

	states.yaw = vehicle_local_position.heading;

	return states;
}

void MulticopterPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	// reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update(false);
	_rc_channels_sub.update(&_rc_channels);

	perf_begin(_cycle_perf);
	vehicle_local_position_s vehicle_local_position;

	if (_local_pos_sub.update(&vehicle_local_position)) {
		const float dt =
			math::constrain(((vehicle_local_position.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = vehicle_local_position.timestamp_sample;

		_sample_interval_s.update(dt);

		if (_vehicle_control_mode_sub.updated()) {
			const bool previous_position_control_enabled = _vehicle_control_mode.flag_multicopter_position_control_enabled;

			if (_vehicle_control_mode_sub.copy(&_vehicle_control_mode)) {
				if (!previous_position_control_enabled && _vehicle_control_mode.flag_multicopter_position_control_enabled) {
					_time_position_control_enabled = _vehicle_control_mode.timestamp;

				} else if (previous_position_control_enabled && !_vehicle_control_mode.flag_multicopter_position_control_enabled) {
					// clear existing setpoint when controller is no longer active
					_setpoint = PositionControl::empty_trajectory_setpoint;
					_control.setInputSetpoint(_setpoint);
				}
			}
		}

		_vehicle_land_detected_sub.update(&_vehicle_land_detected);
		_vehicle_attitude_sub.update(&_vehicle_attitude);
		_vehicle_acceleration_sub.update(&_vehicle_acceleration);
		updateAllocatedForceHistory();

		bool indi_hover_thrust_changed = false;
		if (_hover_thrust_estimate_sub.updated()) {
			hover_thrust_estimate_s hte;

			if (_hover_thrust_estimate_sub.copy(&hte) && hte.valid && PX4_ISFINITE(hte.hover_thrust)) {
				_indi_hover_thrust_target = hte.hover_thrust;
				_indi_hte_valid = true;

			} else {
				_indi_hte_valid = false;
			}
		}

		PositionControlStates states{set_vehicle_states(vehicle_local_position, dt)};
		_indi_acceleration_source_valid = 0;
		_indi_acceleration_velocity_derivative = states.acceleration;

		if (_indi_acceleration_velocity_derivative.isAllFinite()) {
			_indi_acceleration_source_valid |= kIndiAccelerationVelocityValid;
		}

		const Vector3f acceleration_ekf_raw(vehicle_local_position.ax, vehicle_local_position.ay,
				vehicle_local_position.az);

		if (acceleration_ekf_raw.isAllFinite()) {
			if (!_indi_acceleration_ekf_filter_initialized) {
				_indi_acceleration_ekf_lp_filter.reset(acceleration_ekf_raw);
				_indi_acceleration_ekf_filter_initialized = true;
			}

			_indi_acceleration_ekf = _indi_acceleration_ekf_lp_filter.apply(acceleration_ekf_raw);
			_indi_acceleration_source_valid |= kIndiAccelerationEkfValid;

		} else {
			_indi_acceleration_ekf = Vector3f{NAN, NAN, NAN};
			_indi_acceleration_ekf_filter_initialized = false;
		}

		const bool imu_recent = _vehicle_acceleration.timestamp_sample > 0
					&& vehicle_local_position.timestamp_sample >= _vehicle_acceleration.timestamp_sample
					&& vehicle_local_position.timestamp_sample - _vehicle_acceleration.timestamp_sample
					<= kIndiAccelerationSampleMaxAge;
		const bool attitude_recent = _vehicle_attitude.timestamp_sample > 0
					     && vehicle_local_position.timestamp_sample >= _vehicle_attitude.timestamp_sample
					     && vehicle_local_position.timestamp_sample - _vehicle_attitude.timestamp_sample
					     <= kIndiAccelerationSampleMaxAge;
		const Vector3f acceleration_imu_body(_vehicle_acceleration.xyz);
		const Quatf attitude(_vehicle_attitude.q);

		if (imu_recent && attitude_recent && acceleration_imu_body.isAllFinite() && attitude.isAllFinite()) {
			Vector3f acceleration_imu_raw = Dcmf(attitude) * acceleration_imu_body;
			acceleration_imu_raw(2) += CONSTANTS_ONE_G;

			if (!_indi_acceleration_imu_filter_initialized) {
				_indi_acceleration_imu_lp_filter.reset(acceleration_imu_raw);
				_indi_acceleration_imu_filter_initialized = true;
			}

			_indi_acceleration_imu = _indi_acceleration_imu_lp_filter.apply(acceleration_imu_raw);
			_indi_acceleration_source_valid |= kIndiAccelerationImuValid;

		} else {
			_indi_acceleration_imu = Vector3f{NAN, NAN, NAN};
			_indi_acceleration_imu_filter_initialized = false;
		}

		// If a goto setpoint is available this publishes a trajectory setpoint to go there
		// If trajectory_setpoint is published elsewhere, do not use the goto setpoint
		const bool goto_setpoint_enable = _vehicle_control_mode.flag_multicopter_position_control_enabled
						  && !_trajectory_setpoint_sub.updated();

		if (_goto_control.checkForSetpoint(vehicle_local_position.timestamp_sample, goto_setpoint_enable)) {
			_goto_control.update(dt, states.position, states.velocity, states.acceleration, states.yaw);
		}

		_trajectory_setpoint_sub.update(&_setpoint);

		adjustSetpointForEKFResets(vehicle_local_position, _setpoint);

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {
			// set failsafe setpoint if there hasn't been a new
			// trajectory setpoint since position control started
			if ((_setpoint.timestamp < _time_position_control_enabled)
			    && (vehicle_local_position.timestamp_sample > _time_position_control_enabled)) {

				_setpoint = generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, false);
			}
		}

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled
		    && (_setpoint.timestamp >= _time_position_control_enabled)) {

			// update vehicle constraints and handle smooth takeoff
			_vehicle_constraints_sub.update(&_vehicle_constraints);

			// fix to prevent the takeoff ramp to ramp to a too high value or get stuck because of NAN
			// TODO: this should get obsolete once the takeoff limiting moves into the flight tasks
			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
			}

			if (_vehicle_control_mode.flag_control_offboard_enabled) {

				const bool want_takeoff = _vehicle_control_mode.flag_armed
							  && (vehicle_local_position.timestamp_sample < _setpoint.timestamp + 1_s);

				if (want_takeoff && PX4_ISFINITE(_setpoint.position[2])
				    && (_setpoint.position[2] < states.position(2))) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.velocity[2])
					   && (_setpoint.velocity[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.acceleration[2])
					   && (_setpoint.acceleration[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else {
					_vehicle_constraints.want_takeoff = false;
				}

				// override with defaults
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
				_vehicle_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
			}

			bool skip_takeoff = _param_com_throw_en.get();
			// handle smooth takeoff
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
						    _vehicle_constraints.want_takeoff,
						    _vehicle_constraints.speed_up, skip_takeoff, vehicle_local_position.timestamp_sample);

			const bool not_taken_off             = (_takeoff.getTakeoffState() < TakeoffState::rampup);
			const bool flying                    = (_takeoff.getTakeoffState() >= TakeoffState::flight);
			const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

			if (!flying) {
				indi_hover_thrust_changed = fabsf(_indi_hover_thrust - _param_mpc_thr_hover.get()) > FLT_EPSILON;
				_control.setHoverThrust(_param_mpc_thr_hover.get());
				_indi_hover_thrust = _param_mpc_thr_hover.get();
				_indi_hover_thrust_target = _indi_hover_thrust;
				_indi_hte_valid = false;
			}

			// make sure takeoff ramp is not amended by acceleration feed-forward
			if (_takeoff.getTakeoffState() == TakeoffState::rampup && PX4_ISFINITE(_setpoint.velocity[2])) {
				_setpoint.acceleration[2] = NAN;
			}

			if (not_taken_off || flying_but_ground_contact) {
				// we are not flying yet and need to avoid any corrections
				_setpoint = PositionControl::empty_trajectory_setpoint;
				_setpoint.timestamp = vehicle_local_position.timestamp_sample;
				Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration); // High downwards acceleration to make sure there's no thrust

				// prevent any integrator windup
				_control.resetIntegral();
			}

			// limit tilt during takeoff ramupup
			const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
						     ? _param_mpc_tiltmax_lnd.get() : _param_mpc_tiltmax_air.get();
			_control.setTiltLimit(_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt));

			const float speed_up = _takeoff.updateRamp(dt,
					       PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get());
			const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :
						 _param_mpc_z_vel_max_dn.get();

			// Allow ramping from zero thrust on takeoff
			const float minimum_thrust = flying ? _param_mpc_thr_min.get() : 0.f;
			_control.setThrustLimits(minimum_thrust, _param_mpc_thr_max.get());

			float max_speed_xy = _param_mpc_xy_vel_max.get();

			if (PX4_ISFINITE(vehicle_local_position.vxy_max)) {
				max_speed_xy = math::min(max_speed_xy, vehicle_local_position.vxy_max);
			}

			_control.setVelocityLimits(
				max_speed_xy,
				math::min(speed_up, _param_mpc_z_vel_max_up.get()), // takeoff ramp starts with negative velocity limit
				math::max(speed_down, 0.f));

			_control.setInputSetpoint(_setpoint);

			// update states
			if (!PX4_ISFINITE(_setpoint.position[2])
			    && PX4_ISFINITE(_setpoint.velocity[2]) && (fabsf(_setpoint.velocity[2]) > FLT_EPSILON)
			    && PX4_ISFINITE(vehicle_local_position.z_deriv) && vehicle_local_position.z_valid && vehicle_local_position.v_z_valid) {
				// A change in velocity is demanded and the altitude is not controlled.
				// Set velocity to the derivative of position
				// because it has less bias but blend it in across the landing speed range
				//  <  MPC_LAND_SPEED: ramp up using altitude derivative without a step
				//  >= MPC_LAND_SPEED: use altitude derivative
				float weighting = fminf(fabsf(_setpoint.velocity[2]) / _param_mpc_land_speed.get(), 1.f);
				states.velocity(2) = vehicle_local_position.z_deriv * weighting + vehicle_local_position.vz * (1.f - weighting);
			}

			if ((!PX4_ISFINITE(_setpoint.velocity[0]) || !PX4_ISFINITE(_setpoint.velocity[1]))
			    && (!PX4_ISFINITE(_setpoint.position[0]) || !PX4_ISFINITE(_setpoint.position[1]))) {
				// Horizontal velocity is not controlled, reset the integrators to avoid
				// over-corrections when starting again.
				_control.resetIntegralXY();
			}

			const bool indi_requested = (_param_mpc_indi_acc_en.get() == 1)
						    || rcChannelEnabled(_rc_channels, kAccelerationIndiRcChannel);
			const bool use_indi = _indi_capable && indi_requested;
			const int32_t acceleration_source = math::constrain<int32_t>(_param_mpc_indi_a_src.get(), 0, 2);
			Vector3f selected_acceleration = _indi_acceleration_velocity_derivative;
			uint8_t selected_acceleration_valid_bit = kIndiAccelerationVelocityValid;

			if (acceleration_source == 1) {
				selected_acceleration = _indi_acceleration_ekf;
				selected_acceleration_valid_bit = kIndiAccelerationEkfValid;

			} else if (acceleration_source == 2) {
				selected_acceleration = _indi_acceleration_imu;
				selected_acceleration_valid_bit = kIndiAccelerationImuValid;
			}

			const bool selected_acceleration_valid =
				((_indi_acceleration_source_valid & selected_acceleration_valid_bit) != 0)
				&& selected_acceleration.isAllFinite();

			// Apply every valid HTE output immediately. PID preserves its output with
			// updateHoverThrust(); INDI applies the same hover thrust to its output map
			// and selected feedback conversion in this cycle.
			if (_indi_hte_valid) {
				const float hover_thrust_previous = _indi_hover_thrust;
				_indi_hover_thrust = _indi_hover_thrust_target;
				indi_hover_thrust_changed = fabsf(_indi_hover_thrust - hover_thrust_previous) > FLT_EPSILON;
			}

			// Convert the delayed force only after applying the current hover-thrust
			// estimate because the normalized-force path depends on that value.
			const Vector3f delayed_allocated_thrust_acceleration =
				getDelayedAllocatedThrustAcceleration(vehicle_local_position.timestamp_sample);
			const bool acceleration_indi_feedback_available = selected_acceleration_valid
					&& delayed_allocated_thrust_acceleration.isAllFinite();
			// This readiness flag is not an EKF-health decision. Invalid base position
			// or velocity states are handled by PositionControl's normal failsafe path.
			const bool acceleration_indi_ready = use_indi && acceleration_indi_feedback_available;

			if (acceleration_indi_ready) {
				// Switch a_0 and F_0 into PositionControl as one complete INDI feedback pair.
				// Otherwise leave states.acceleration on the velocity-derivative source used
				// by the conventional PID controller.
				states.acceleration = selected_acceleration;
				states.allocated_thrust_acceleration = delayed_allocated_thrust_acceleration;

			} else {
				states.allocated_thrust_acceleration = Vector3f{NAN, NAN, NAN};
			}

			if (acceleration_indi_ready) {
				// INDI uses the hover-thrust estimate directly with the selected force
				// conversion. Do not inject a compensation into the PID integrator.
				_control.setHoverThrust(_indi_hover_thrust);

			} else if (indi_hover_thrust_changed) {
				// Conventional PID hot standby keeps its original bumpless hover-thrust update.
				_control.updateHoverThrust(_indi_hover_thrust);
			}

			_control.setState(states);

			const hrt_abstime now = hrt_absolute_time();

			// Run position control
			if (_control.update(dt)) {

				// Valid control update - store for fallback
				_last_valid_setpoint = _setpoint;

			} else {

				// Initial update failed - Try fallback if within timeout
				if (now < _last_valid_setpoint.timestamp + 200_ms) {
					// Use last valid setpoint
					adjustSetpointForEKFResets(vehicle_local_position, _last_valid_setpoint);
					_control.setInputSetpoint(_last_valid_setpoint);
				}

				// Still failing / not within timeout - Go to failsafe
				if (!_control.update(dt)) {

					_vehicle_constraints = {0, NAN, NAN, false, {}}; // reset constraints

					_control.setInputSetpoint(generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, true));
					_control.setVelocityLimits(_param_mpc_xy_vel_max.get(), _param_mpc_z_vel_max_up.get(), _param_mpc_z_vel_max_dn.get());

					_control.update(dt);
				}
			}

			// Publish internal position control setpoints
			// on top of the input/feed-forward setpoints these containt the PID corrections
			// This message is used by other modules (such as Landdetector) to determine vehicle intention.
			vehicle_local_position_setpoint_s local_pos_sp{};
			_control.getLocalPositionSetpoint(local_pos_sp);
			local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp_pub.publish(local_pos_sp);

			acceleration_indi_status_s indi_status{};
			indi_status.timestamp_sample = vehicle_local_position.timestamp_sample;
			indi_status.force_timestamp = _indi_delayed_force_timestamp;
			states.acceleration.copyTo(indi_status.acceleration_feedback);
			_indi_acceleration_velocity_derivative.copyTo(indi_status.acceleration_velocity_derivative);
			_indi_acceleration_ekf.copyTo(indi_status.acceleration_ekf);
			_indi_acceleration_imu.copyTo(indi_status.acceleration_imu);
			delayed_allocated_thrust_acceleration.copyTo(indi_status.allocated_thrust_acceleration);
			_indi_undelayed_thrust_acceleration.copyTo(indi_status.allocated_thrust_acceleration_undelayed);
			_control.getAccelerationIndiRawThrustSetpoint().copyTo(indi_status.raw_thrust_setpoint);
			indi_status.hover_thrust = _indi_hover_thrust;
			indi_status.controller_transition_progress = _control.getAccelerationIndiTransitionProgress();
			indi_status.acceleration_source = math::constrain<int32_t>(_param_mpc_indi_a_src.get(), 0, 2);
			indi_status.acceleration_source_valid = _indi_acceleration_source_valid;
			indi_status.feedback_valid = acceleration_indi_feedback_available;
			indi_status.controller_transition_active = _control.accelerationIndiTransitionActive();
			indi_status.timestamp = hrt_absolute_time();
			_acceleration_indi_status_pub.publish(indi_status);

			// Publish attitude setpoint output
			vehicle_attitude_setpoint_s attitude_setpoint{};
			_control.getAttitudeSetpoint(attitude_setpoint);
			attitude_setpoint.timestamp = hrt_absolute_time();
			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

		} else {
			// an update is necessary here because otherwise the takeoff state doesn't get skipped with non-altitude-controlled modes
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f, true,
						    vehicle_local_position.timestamp_sample);
			_control.resetIntegral();
		}

		// Publish takeoff status
		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

		if (takeoff_state != _takeoff_status_pub.get().takeoff_state
		    || !isEqualF(_tilt_limit_slew_rate.getState(), _takeoff_status_pub.get().tilt_limit)) {
			_takeoff_status_pub.get().takeoff_state = takeoff_state;
			_takeoff_status_pub.get().tilt_limit = _tilt_limit_slew_rate.getState();
			_takeoff_status_pub.get().timestamp = hrt_absolute_time();
			_takeoff_status_pub.update();
		}
	}

	perf_end(_cycle_perf);
}

void MulticopterPositionControl::updateAllocatedForceHistory()
{
	allocation_value_s allocation_value;

	if (_vehicle_attitude.timestamp == 0 || hrt_elapsed_time(&_vehicle_attitude.timestamp) >= 100_ms
	    || !_allocation_value_sub.copy(&allocation_value) || !allocationForceValid(allocation_value)) {
		return;
	}

	if (allocation_value.timestamp == 0 || allocation_value.timestamp <= _indi_force_history_last_timestamp) {
		return;
	}

	if (_indi_force_history_last_timestamp != 0
	    && allocation_value.timestamp > _indi_force_history_last_timestamp + 100_ms) {
		_indi_force_history.reset();
	}

	const Vector3f force_body(allocation_value.allocated_force);
	const Vector3f force_setpoint_scale(allocation_value.force_setpoint_scale);
	Dcmf R_to_ned(Quatf(_vehicle_attitude.q));

	IndiForceSample sample{};
	sample.time_us = allocation_value.timestamp;
	sample.physical_force_ned = R_to_ned * force_body;
	sample.normalized_force_ned = force_setpoint_scale.isAllFinite()
				      ? R_to_ned * force_body.emult(force_setpoint_scale)
				      : Vector3f{NAN, NAN, NAN};
	_indi_force_history.push(sample);
	_indi_force_history_last_timestamp = sample.time_us;
}

Vector3f MulticopterPositionControl::allocationFeedbackToThrustAcceleration(const IndiForceSample &sample) const
{
	if (_param_mpc_indi_f_src.get() == 1) {
		if (!PX4_ISFINITE(_indi_hover_thrust) || _indi_hover_thrust <= FLT_EPSILON) {
			return Vector3f{NAN, NAN, NAN};
		}

		return sample.normalized_force_ned * (CONSTANTS_ONE_G / _indi_hover_thrust);
	}

	// Preserve the original physical acceleration-INDI feedback. This path uses
	// the allocation model's force directly and has no hover-thrust correction.
	return sample.physical_force_ned * _indi_inverse_mass;
}

Vector3f MulticopterPositionControl::getDelayedAllocatedThrustAcceleration(hrt_abstime reference_timestamp)
{
	_indi_delayed_force_timestamp = 0;
	_indi_undelayed_thrust_acceleration = Vector3f{NAN, NAN, NAN};

	if (_indi_force_history.empty()) {
		return Vector3f{NAN, NAN, NAN};
	}

	if (hrt_elapsed_time(&_indi_force_history.get_newest().time_us) >= 100_ms) {
		return Vector3f{NAN, NAN, NAN};
	}

	const float delay_s = PX4_ISFINITE(_param_mpc_indi_f_dly.get())
			      ? math::constrain(_param_mpc_indi_f_dly.get(), 0.f, 0.1f) : 0.f;
	const hrt_abstime delay_us = static_cast<hrt_abstime>(delay_s * 1e6f);
	const hrt_abstime target_timestamp = reference_timestamp > delay_us ? reference_timestamp - delay_us : 0;
	const IndiForceSample &newest = _indi_force_history.get_newest();
	_indi_undelayed_thrust_acceleration = allocationFeedbackToThrustAcceleration(newest);

	if (!_indi_undelayed_thrust_acceleration.isAllFinite()) {
		return Vector3f{NAN, NAN, NAN};
	}

	if (target_timestamp >= newest.time_us) {
		_indi_delayed_force_timestamp = newest.time_us;
		return _indi_undelayed_thrust_acceleration;
	}

	IndiForceSample older{};
	IndiForceSample newer{};
	bool older_valid = false;
	bool newer_valid = false;
	uint8_t index = _indi_force_history.get_oldest_index();

	for (int entry = 0; entry < _indi_force_history.entries(); entry++) {
		const IndiForceSample &sample = _indi_force_history[index];

		if (sample.time_us <= target_timestamp) {
			older = sample;
			older_valid = true;
		}

		if (sample.time_us >= target_timestamp) {
			newer = sample;
			newer_valid = true;
			break;
		}

		index = _indi_force_history.next(index);
	}

	if (!older_valid) {
		// The configured delay is not available yet. Keep the conventional PID
		// path active until the force history covers the requested timestamp.
		return Vector3f{NAN, NAN, NAN};
	}

	IndiForceSample delayed_sample = older;
	_indi_delayed_force_timestamp = older.time_us;

	if (newer_valid && newer.time_us > older.time_us) {
		const float interpolation = static_cast<float>(target_timestamp - older.time_us)
					    / static_cast<float>(newer.time_us - older.time_us);
		delayed_sample.physical_force_ned +=
			(newer.physical_force_ned - older.physical_force_ned) * interpolation;
		delayed_sample.normalized_force_ned +=
			(newer.normalized_force_ned - older.normalized_force_ned) * interpolation;
		_indi_delayed_force_timestamp = target_timestamp;
	}

	const Vector3f delayed_acceleration = allocationFeedbackToThrustAcceleration(delayed_sample);

	if (!delayed_acceleration.isAllFinite()) {
		return Vector3f{NAN, NAN, NAN};
	}

	return delayed_acceleration;
}

trajectory_setpoint_s MulticopterPositionControl::generateFailsafeSetpoint(const hrt_abstime &now,
		const PositionControlStates &states, bool warn)
{
	// rate limit the warnings
	warn = warn && (now - _last_warn) > 2_s;

	if (warn) {
		PX4_WARN("invalid setpoints");
		_last_warn = now;
	}

	trajectory_setpoint_s failsafe_setpoint = PositionControl::empty_trajectory_setpoint;
	failsafe_setpoint.timestamp = now;

	if (Vector2f(states.velocity).isAllFinite()) {
		// don't move along xy
		failsafe_setpoint.velocity[0] = failsafe_setpoint.velocity[1] = 0.f;

		if (warn) {
			PX4_WARN("Failsafe: stop and wait");
		}

	} else {
		// descend with land speed since we can't stop
		failsafe_setpoint.acceleration[0] = failsafe_setpoint.acceleration[1] = 0.f;
		failsafe_setpoint.velocity[2] = _param_mpc_land_speed.get();

		if (warn) {
			PX4_WARN("Failsafe: blind land");
		}
	}

	if (PX4_ISFINITE(states.velocity(2))) {
		// don't move along z if we can stop in all dimensions
		if (!PX4_ISFINITE(failsafe_setpoint.velocity[2])) {
			failsafe_setpoint.velocity[2] = 0.f;
		}

	} else {
		// emergency descend with a bit below hover thrust
		failsafe_setpoint.velocity[2] = NAN;
		failsafe_setpoint.acceleration[2] = .3f;

		if (warn) {
			PX4_WARN("Failsafe: blind descent");
		}
	}

	return failsafe_setpoint;
}

void MulticopterPositionControl::adjustSetpointForEKFResets(const vehicle_local_position_s &vehicle_local_position,
		trajectory_setpoint_s &setpoint)
{
	if ((setpoint.timestamp != 0) && (setpoint.timestamp < vehicle_local_position.timestamp)) {
		if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
			setpoint.velocity[0] += vehicle_local_position.delta_vxy[0];
			setpoint.velocity[1] += vehicle_local_position.delta_vxy[1];
		}

		if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
			setpoint.velocity[2] += vehicle_local_position.delta_vz;
		}

		if (vehicle_local_position.xy_reset_counter != _xy_reset_counter) {
			setpoint.position[0] += vehicle_local_position.delta_xy[0];
			setpoint.position[1] += vehicle_local_position.delta_xy[1];
		}

		if (vehicle_local_position.z_reset_counter != _z_reset_counter) {
			setpoint.position[2] += vehicle_local_position.delta_z;
		}

		if (vehicle_local_position.heading_reset_counter != _heading_reset_counter) {
			setpoint.yaw = wrap_pi(setpoint.yaw + vehicle_local_position.delta_heading);
		}
	}

	if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
		_vel_xy_filtered += Vector2f(vehicle_local_position.delta_vxy);
		_vel_xy_lp_filter.reset(_vel_xy_filtered);
		_vel_xy_notch_filter.reset();
	}

	if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
		_vel_z_filtered += vehicle_local_position.delta_vz;
		_vel_z_lp_filter.reset(_vel_z_filtered);
		_vel_z_notch_filter.reset();
	}

	// save latest reset counters
	_vxy_reset_counter = vehicle_local_position.vxy_reset_counter;
	_vz_reset_counter = vehicle_local_position.vz_reset_counter;
	_xy_reset_counter = vehicle_local_position.xy_reset_counter;
	_z_reset_counter = vehicle_local_position.z_reset_counter;
	_heading_reset_counter = vehicle_local_position.heading_reset_counter;
}

int MulticopterPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterPositionControl *instance = new MulticopterPositionControl(vtol);

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

int MulticopterPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[])
{
	return ModuleBase::main(MulticopterPositionControl::desc, argc, argv);
}
