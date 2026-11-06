/**
 * Copyright 2026 Chaoheng Meng
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "McOmMpcAccIndi.hpp"

#include <drivers/drv_hrt.h>
#include <geo/geo.h>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/log.h>

using namespace matrix;

namespace
{
constexpr float kSampleRateChangeThreshold = 0.1f;

bool allocationFeedbackValid(const allocation_value_s &allocation)
{
	return allocation.timestamp != 0
	       && Vector3f(allocation.raw_allocated_force).isAllFinite()
	       && Vector3f(allocation.force_setpoint_scale).isAllFinite();
}

Vector3f constrainDisturbance(const Vector3f &raw, float limit)
{
	Vector3f disturbance = raw;
	const float norm = disturbance.norm();

	if (limit > FLT_EPSILON && PX4_ISFINITE(norm) && norm > limit) {
		disturbance *= limit / norm;
	}

	return disturbance;
}
} // namespace

ModuleBase::Descriptor McOmMpcAccIndi::desc{task_spawn, custom_command, print_usage};

McOmMpcAccIndi::McOmMpcAccIndi() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	parametersUpdated();
}

McOmMpcAccIndi::~McOmMpcAccIndi()
{
	_vehicle_attitude_sub.unregisterCallback();
	perf_free(_loop_perf);
}

bool McOmMpcAccIndi::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("vehicle_attitude callback registration failed");
		return false;
	}

	return true;
}

void McOmMpcAccIndi::parametersUpdated()
{
	updateParams();
	int32_t onboard_enabled = 0;
	param_get(_onboard_enable_handle, &onboard_enabled);
	_onboard_enabled = onboard_enabled == 1;
}

void McOmMpcAccIndi::updateEnableState(bool middle_layer_enabled)
{
	if (!_middle_layer_enabled_previous && middle_layer_enabled) {
		// Rebuild matched histories together after re-enabling. Nominal MPC
		// retains control until fresh acceleration and delayed force are valid.
		_ekf_acceleration_filter = AccelerationFilterState{};
		_imu_acceleration_filter = AccelerationFilterState{};
		_force_filter = AccelerationFilterState{};
		_force_history.reset();
		_force_history_last_timestamp = 0;
		_force_feedback_timestamp = 0;
		resetTransition();
	}

	_middle_layer_enabled_previous = middle_layer_enabled;
}

void McOmMpcAccIndi::updateAccelerationFilter(const Vector3f &raw, uint64_t timestamp,
		AccelerationFilterState &state)
{
	if (timestamp == 0 || timestamp <= state.timestamp) {
		return;
	}

	if (state.timestamp > 0) {
		const float sample_dt = (timestamp - state.timestamp) * 1e-6f;

		if (sample_dt > 0.f && sample_dt < 0.1f) {
			state.sample_interval_s.update(sample_dt);

		} else {
			state.sample_interval_s.reset();
			state.initialized = false;
		}
	}

	state.timestamp = timestamp;

	if (!raw.isAllFinite()) {
		state.value = Vector3f{NAN, NAN, NAN};
		state.initialized = false;
		return;
	}

	const float cutoff_hz = math::max(_param_acceleration_cutoff.get(), 0.f);

	if (cutoff_hz <= FLT_EPSILON) {
		state.filter.disable();
		state.value = raw;
		state.initialized = false;
		state.cutoff_hz = cutoff_hz;
		return;
	}

	if (!state.sample_interval_s.valid()) {
		state.value = raw;
		return;
	}

	const float sample_hz = 1.f / state.sample_interval_s.mean();
	const bool sample_rate_changed = state.initialized
		&& fabsf(sample_hz - state.sample_hz) > kSampleRateChangeThreshold * state.sample_hz;
	const bool configuration_changed = !state.initialized || sample_rate_changed
		|| fabsf(cutoff_hz - state.cutoff_hz) > FLT_EPSILON;

	if (configuration_changed) {
		const Vector3f reset_value = state.value.isAllFinite() ? state.value : raw;
		state.filter.set_cutoff_frequency(sample_hz, cutoff_hz);
		state.filter.reset(reset_value);
		state.initialized = true;
		state.sample_hz = sample_hz;
		state.cutoff_hz = cutoff_hz;
	}

	state.value = state.filter.apply(raw);
}

Dcmf McOmMpcAccIndi::attitudeAt(uint64_t timestamp, const Dcmf &attitude) const
{
	if (timestamp == 0 || _vehicle_attitude.timestamp_sample == 0) {
		return attitude;
	}

	const int64_t delta_us = static_cast<int64_t>(timestamp)
			- static_cast<int64_t>(_vehicle_attitude.timestamp_sample);
	const float dt = delta_us * 1e-6f;
	const Vector3f omega(_vehicle_angular_velocity.xyz);

	// The EKF attitude is the anchor. Short gyro propagation only removes the
	// asynchronous sample offset; it is never allowed to become an estimator.
	if (fabsf(dt) <= 0.01f && omega.isAllFinite()
	    && _vehicle_angular_velocity.timestamp_sample > 0) {
		return attitude * Dcmf(AxisAnglef(omega * dt));
	}

	return attitude;
}

void McOmMpcAccIndi::updateSensorInputs()
{
	_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity);

	vehicle_local_position_s local_position;

	if (_vehicle_local_position_sub.update(&local_position)) {
		updateAccelerationFilter(Vector3f(local_position.ax, local_position.ay, local_position.az),
			local_position.timestamp_sample, _ekf_acceleration_filter);
	}

	vehicle_acceleration_s acceleration;

	if (_vehicle_acceleration_sub.update(&acceleration)) {
		const Vector3f acceleration_body(acceleration.xyz);
		const int64_t attitude_delta = static_cast<int64_t>(acceleration.timestamp_sample)
				- static_cast<int64_t>(_vehicle_attitude.timestamp_sample);

		if (acceleration_body.isAllFinite()
		    && attitude_delta >= -static_cast<int64_t>(10_ms)
		    && attitude_delta <= static_cast<int64_t>(10_ms)) {
			Vector3f acceleration_ned = attitudeAt(
				acceleration.timestamp_sample, Dcmf(Quatf(_vehicle_attitude.q))) * acceleration_body;
			acceleration_ned(2) += CONSTANTS_ONE_G;
			updateAccelerationFilter(acceleration_ned, acceleration.timestamp_sample,
				_imu_acceleration_filter);
		}
	}
}

void McOmMpcAccIndi::updateForceHistory()
{
	allocation_value_s allocation;

	if (!_allocation_value_sub.copy(&allocation) || !allocationFeedbackValid(allocation)
	    || allocation.timestamp <= _force_history_last_timestamp) {
		return;
	}

	if (_force_history_last_timestamp != 0
	    && allocation.timestamp > _force_history_last_timestamp + 100_ms) {
		_force_history.reset();
		_force_filter = AccelerationFilterState{};
	}

	ForceSample sample{};
	// Match main.m: H[R(t) F_body(t)] and H[a_ned(t)] must share
	// their inertial-frame expansion point. R(t) H[F_body(t)] is NOT
	// equivalent during a fast rotation; it produces a fictitious disturbance
	// even for constant thrust with perfect acceleration measurements.
	// Start from the unfiltered allocation so this is exactly one OM filter,
	// independent of the body-frame CA_FORCE_CUTOFF used by other controllers.
	const Vector3f force_body(allocation.raw_allocated_force);
	const Vector3f force_scale(allocation.force_setpoint_scale);
	sample.time_us = allocation.timestamp;
	// This is allocated command feedback, not measured motor thrust. Remaining
	// actuator/estimator delay must be evaluated separately in flight logs.
	const Dcmf force_attitude = attitudeAt(
		allocation.timestamp, Dcmf(Quatf(_vehicle_attitude.q)));
	updateAccelerationFilter(force_attitude * force_body.emult(force_scale),
		allocation.timestamp, _force_filter);
	sample.allocated_force_ned = _force_filter.value;
	_force_history.push(sample);
	_force_history_last_timestamp = sample.time_us;
}

bool McOmMpcAccIndi::getDelayedAllocatedForce(uint64_t reference_timestamp,
		Vector3f &allocated_force_ned)
{
	_force_feedback_timestamp = 0;
	if (_force_history.empty()
	    || hrt_elapsed_time(&_force_history.get_newest().time_us) >= 100_ms) {
		return false;
	}

	const float delay_s = PX4_ISFINITE(_param_force_delay.get())
			? math::constrain(_param_force_delay.get(), 0.f, 0.1f) : 0.f;
	const uint64_t delay_us = static_cast<uint64_t>(delay_s * 1e6f);
	const uint64_t target = reference_timestamp > delay_us ? reference_timestamp - delay_us : 0;
	const ForceSample &newest = _force_history.get_newest();

	if (target >= newest.time_us) {
		allocated_force_ned = newest.allocated_force_ned;
		_force_feedback_timestamp = newest.time_us;
		return allocated_force_ned.isAllFinite();
	}

	ForceSample older{};
	ForceSample newer{};
	bool older_valid = false;
	bool newer_valid = false;
	uint8_t index = _force_history.get_oldest_index();

	for (int entry = 0; entry < _force_history.entries(); ++entry) {
		const ForceSample &sample = _force_history[index];

		if (sample.time_us <= target && sample.allocated_force_ned.isAllFinite()) {
			older = sample;
			older_valid = true;
		}

		if (sample.time_us >= target && sample.allocated_force_ned.isAllFinite()) {
			newer = sample;
			newer_valid = true;
			break;
		}

		index = _force_history.next(index);
	}

	if (!older_valid) {
		return false;
	}

	allocated_force_ned = older.allocated_force_ned;
	_force_feedback_timestamp = older.time_us;

	if (newer_valid && newer.time_us > older.time_us) {
		const float alpha = static_cast<float>(target - older.time_us)
				    / static_cast<float>(newer.time_us - older.time_us);
		allocated_force_ned += (newer.allocated_force_ned - older.allocated_force_ned) * alpha;
		_force_feedback_timestamp = target;
	}

	return allocated_force_ned.isAllFinite();
}

void McOmMpcAccIndi::resetTransition()
{
	_acceleration_active_previous = false;
	_transition_elapsed = 0.f;
	_transition_progress = 0.f;
}

void McOmMpcAccIndi::publishDisturbance(uint64_t now, uint64_t acceleration_timestamp,
		bool valid, const Vector3f &disturbance)
{
	om_mpc_disturbance_s feedback{};
	feedback.timestamp = now;
	feedback.timestamp_sample = acceleration_timestamp;
	disturbance.copyTo(feedback.acceleration_disturbance);
	feedback.valid = valid;
	_disturbance_pub.publish(feedback);
}

void McOmMpcAccIndi::Run()
{
	perf_begin(_loop_perf);

	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup(desc);
		perf_end(_loop_perf);
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s update;
		_parameter_update_sub.copy(&update);
		parametersUpdated();
	}

	_rc_channels_sub.update(&_rc_channels);
	const uint64_t now = hrt_absolute_time();
	const bool middle_layer_enabled = om_mpc::middleLayerRequested(_param_enable.get() == 1, _rc_channels, now);
	updateEnableState(middle_layer_enabled);

	// Match mc_att_control: compute once for each new attitude message.
	const bool attitude_updated = _vehicle_attitude_sub.update(&_vehicle_attitude);
	if (_vehicle_attitude.timestamp == 0
	    || hrt_elapsed_time(&_vehicle_attitude.timestamp) >= 100_ms) {
		publishDisturbance(now, 0, false, Vector3f{NAN, NAN, NAN});
		perf_end(_loop_perf);
		return;
	}

	// A queued callback may have already been consumed by the preceding run.
	// Keep still-valid feedback when there is no unread attitude sample.
	if (!attitude_updated) {
		perf_end(_loop_perf);
		return;
	}

	// Use the same sample-time clock and dt bounds as mc_att_control. Keep this
	// clock running while disabled so enabling INDI does not restart its timebase.
	const float dt = math::constrain((_vehicle_attitude.timestamp_sample - _last_run_timestamp) * 1e-6f,
					0.0002f, 0.02f);
	_last_run_timestamp = _vehicle_attitude.timestamp_sample;

	const Dcmf attitude(Quatf(_vehicle_attitude.q));

	_vehicle_control_mode_sub.update(&_vehicle_control_mode);
	_vehicle_land_detected_sub.update(&_vehicle_land_detected);
	_vehicle_status_sub.update(&_vehicle_status);
	_onboard_command_sub.update(&_onboard_command);
	const float hover_thrust = _onboard_command.hover_thrust;
	_control.setParams(
		Vector3f(_param_kphi_roll.get(), _param_kphi_pitch.get(), _param_kphi_yaw.get()),
		hover_thrust, CONSTANTS_ONE_G);
	const bool requested = om_mpc::mpcRequested(_onboard_enabled, _rc_channels, now)
			&& _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ORBIT
			&& _onboard_command.valid && _onboard_command.timestamp_valid != 0
			&& hrt_elapsed_time(&_onboard_command.timestamp_valid) < 60_ms
			&& PX4_ISFINITE(hover_thrust) && hover_thrust > FLT_EPSILON
			&& Vector3f(_onboard_command.thrust_body).isAllFinite()
			&& Vector3f(_onboard_command.rates).isAllFinite()
			&& Vector3f(_onboard_command.mpc_disturbance).isAllFinite();
	const int32_t source = math::constrain<int32_t>(_param_acceleration_source.get(), 1, 2);

	// Nominal MPC commands go straight to mc_att_control. This module only
	// produces optional corrections and disturbance feedback while enabled.
	if (!requested || !middle_layer_enabled) {
		resetTransition();
		publishDisturbance(now, 0, false, Vector3f{NAN, NAN, NAN});
		perf_end(_loop_perf);
		return;
	}

	updateSensorInputs();
	updateForceHistory();

	const AccelerationFilterState &selected = source == 2
			? _imu_acceleration_filter : _ekf_acceleration_filter;

	// A held acceleration sample must not stall the algebraic correction. Run
	// on each new attitude, like mc_att_control; acceleration and force filters
	// advance only on their own new sample timestamps.

	const bool flight_enabled = _vehicle_control_mode.flag_armed
			&& _vehicle_status.takeoff_time != 0 && !_vehicle_land_detected.landed;
	const bool enabled = requested && middle_layer_enabled && flight_enabled
			&& _control.paramsValid() && attitude.isAllFinite();

	const bool acceleration_fresh = selected.timestamp > 0
			&& hrt_elapsed_time(&selected.timestamp) < 100_ms && selected.value.isAllFinite();
	const Vector3f acceleration_ned = selected.value;
	Vector3f allocated_force_ned{NAN, NAN, NAN};
	Vector3f allocated_force_raw{NAN, NAN, NAN};
	bool force_valid = acceleration_fresh
			&& getDelayedAllocatedForce(selected.timestamp, allocated_force_raw);

	if (force_valid) {
		allocated_force_ned = allocated_force_raw;
		force_valid = allocated_force_ned.isAllFinite();
	}
	const bool feedback_valid = enabled && acceleration_fresh && force_valid;
	Vector3f disturbance{NAN, NAN, NAN};
	// Unavailable corrections must not look like valid zero-valued diagnostics.
	OmMpcAccIndiControl::Output corrected{
		Vector3f{NAN, NAN, NAN}, Vector3f{NAN, NAN, NAN},
		Vector3f{NAN, NAN, NAN}, Vector3f{NAN, NAN, NAN}
	};
	bool corrected_valid = false;
	hrt_abstime control_start{0};

	const Vector3f nominal_rates(_onboard_command.rates);
	const Vector3f nominal_thrust{0.f, 0.f,
		-math::constrain(-_onboard_command.thrust_body[2], 0.f, 1.f)};
	// Carry the disturbance used by this QP, not the newest estimate, so INDI
	// compensates only the residual disturbance instead of compensating twice.
	const Vector3f mpc_disturbance(_onboard_command.mpc_disturbance);

	if (feedback_valid && hover_thrust > FLT_EPSILON) {
		control_start = hrt_absolute_time();
		const Vector3f disturbance_raw = acceleration_ned - Vector3f{0.f, 0.f, CONSTANTS_ONE_G}
				- allocated_force_ned * (CONSTANTS_ONE_G / hover_thrust);
		disturbance = constrainDisturbance(disturbance_raw,
				math::max(_param_disturbance_limit.get(), 0.f));
		// The callback already supplies the newest estimated attitude. Do not
		// extrapolate it to scheduler wall time: main.m and the native PX4 control
		// modules use the estimator sample itself for the current control state.
		corrected_valid = _control.update(attitude, nominal_rates, nominal_thrust,
			mpc_disturbance, acceleration_ned, allocated_force_ned, corrected);
	}

	const bool acceleration_active = enabled && corrected_valid;

	if (acceleration_active != _acceleration_active_previous) {
		_transition_elapsed = 0.f;
		_transition_progress = 0.f;
		_acceleration_active_previous = acceleration_active;
	}

	Vector3f output_rates = nominal_rates;
	Vector3f output_thrust = nominal_thrust;

	if (acceleration_active) {
		const float transition_time = math::max(_param_transition_time.get(), 0.f);
		const float ratio = transition_time > FLT_EPSILON
				? math::constrain(_transition_elapsed / transition_time, 0.f, 1.f) : 1.f;
		_transition_progress = ratio * ratio * (3.f - 2.f * ratio);
		output_rates += _transition_progress * (corrected.rates_setpoint - output_rates);
		output_thrust += _transition_progress * (corrected.thrust_body - output_thrust);
		_transition_elapsed += dt;

	} else {
		_transition_progress = 0.f;
	}

	const uint64_t publication_time = hrt_absolute_time();
	om_mpc_acc_indi_command_s correction{};
	correction.timestamp = publication_time;
	correction.timestamp_nominal = _onboard_command.timestamp_valid;
	correction.timestamp_sample = selected.timestamp;
	correction.timestamp_force = allocated_force_ned.isAllFinite() ? _force_feedback_timestamp : 0;
	output_rates.copyTo(correction.omega_cmd);
	output_thrust.copyTo(correction.thrust_body);
	correction.valid = acceleration_active;
	correction.compute_us = acceleration_active ? static_cast<uint32_t>(publication_time - control_start) : 0;
	acceleration_ned.copyTo(correction.a_f);
	(allocated_force_ned * (CONSTANTS_ONE_G / hover_thrust)).copyTo(correction.beta_f);
	corrected.attitude_error.copyTo(correction.e_phi);
	_onboard_correction_pub.publish(correction);

	// DFB reuses the enabled translational INDI path's matched measurements.
	const bool disturbance_valid = om_mpc::disturbanceRequested(_param_disturbance_enable.get() == 1, _rc_channels, now)
			&& feedback_valid && disturbance.isAllFinite();
	publishDisturbance(publication_time, selected.timestamp, disturbance_valid, disturbance);
	perf_end(_loop_perf);
}

int McOmMpcAccIndi::task_spawn(int argc, char *argv[])
{
	McOmMpcAccIndi *instance = new McOmMpcAccIndi();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int McOmMpcAccIndi::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int McOmMpcAccIndi::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
OM-MPC acceleration-INDI middle layer. It uses estimator/IMU acceleration and
allocator feedback to correct onboard OmMpcCommand. Corrections return through
mc_att_control, which owns the final VehicleRatesSetpoint publication.
Disturbance feedback reuses the enabled translational INDI measurements.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("mc_om_mpc_acc_indi", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mc_om_mpc_acc_indi_main(int argc, char *argv[])
{
	return ModuleBase::main(McOmMpcAccIndi::desc, argc, argv);
}
