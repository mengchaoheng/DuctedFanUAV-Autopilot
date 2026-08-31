/**
 * Copyright 2026 Chaoheng Meng
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "McOmMpcIndi.hpp"

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
	       && Vector3f(allocation.allocated_force).isAllFinite()
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

ModuleBase::Descriptor McOmMpcIndi::desc{task_spawn, custom_command, print_usage};

McOmMpcIndi::McOmMpcIndi() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	parametersUpdated();
}

McOmMpcIndi::~McOmMpcIndi()
{
	_vehicle_attitude_sub.unregisterCallback();
	perf_free(_loop_perf);
}

bool McOmMpcIndi::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("vehicle_attitude callback registration failed");
		return false;
	}

	return true;
}

void McOmMpcIndi::parametersUpdated()
{
	updateParams();
	_control.setParams(
		Vector3f(_param_kphi_roll.get(), _param_kphi_pitch.get(), _param_kphi_yaw.get()),
		Vector3f(_param_rate_roll_max.get(), _param_rate_pitch_max.get(), _param_rate_yaw_max.get()),
		_param_hover_thrust.get(), CONSTANTS_ONE_G);
}

void McOmMpcIndi::updateAccelerationFilter(const Vector3f &raw, uint64_t timestamp,
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

Dcmf McOmMpcIndi::attitudeAt(uint64_t timestamp, const Dcmf &attitude) const
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

void McOmMpcIndi::updateSensorInputs(const Dcmf &attitude)
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
			Vector3f acceleration_ned = attitudeAt(acceleration.timestamp_sample, attitude)
						   * acceleration_body;
			acceleration_ned(2) += CONSTANTS_ONE_G;
			updateAccelerationFilter(acceleration_ned, acceleration.timestamp_sample,
				_imu_acceleration_filter);
		}
	}
}

void McOmMpcIndi::updateForceHistory(const Dcmf &attitude)
{
	allocation_value_s allocation;

	if (!_allocation_value_sub.copy(&allocation) || !allocationFeedbackValid(allocation)
	    || allocation.timestamp <= _force_history_last_timestamp) {
		return;
	}

	if (_force_history_last_timestamp != 0
	    && allocation.timestamp > _force_history_last_timestamp + 100_ms) {
		_force_history.reset();
	}

	ForceSample sample{};
	sample.time_us = allocation.timestamp;
	const Vector3f force_body(allocation.allocated_force);
	const Vector3f force_scale(allocation.force_setpoint_scale);
	sample.allocated_force_ned = attitudeAt(allocation.timestamp, attitude)
				     * force_body.emult(force_scale);
	_force_history.push(sample);
	_force_history_last_timestamp = sample.time_us;
}

bool McOmMpcIndi::getDelayedAllocatedForce(uint64_t reference_timestamp,
		Vector3f &allocated_force_ned)
{
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

	if (newer_valid && newer.time_us > older.time_us) {
		const float alpha = static_cast<float>(target - older.time_us)
				    / static_cast<float>(newer.time_us - older.time_us);
		allocated_force_ned += (newer.allocated_force_ned - older.allocated_force_ned) * alpha;
	}

	return allocated_force_ned.isAllFinite();
}

void McOmMpcIndi::resetTransition()
{
	_acceleration_active_previous = false;
	_transition_elapsed = 0.f;
	_transition_progress = 0.f;
}

void McOmMpcIndi::publishStatus(uint64_t now, uint64_t acceleration_timestamp,
		bool acceleration_active, bool disturbance_valid, const Vector3f &disturbance)
{
	if (_last_status_publish != 0 && now < _last_status_publish + 10_ms) {
		return;
	}

	_rate_ctrl_status_sub.update(&_rate_ctrl_status);
	om_mpc_indi_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.timestamp_sample = acceleration_timestamp > 0 ? acceleration_timestamp : now;
	status.acceleration_active = acceleration_active;
	status.rate_active = _rate_ctrl_status.indi_active;
	status.disturbance_valid = disturbance_valid;

	if (disturbance_valid) {
		disturbance.copyTo(status.acceleration_disturbance);
	}

	_status_pub.publish(status);
	const uint64_t next_scheduled_publish = _last_status_publish + 10_ms;
	_last_status_publish = (_last_status_publish == 0 || now > next_scheduled_publish + 10_ms)
			       ? now : next_scheduled_publish;
}

void McOmMpcIndi::Run()
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

	_vehicle_attitude_sub.update(&_vehicle_attitude);

	if (_vehicle_attitude.timestamp == 0
	    || hrt_elapsed_time(&_vehicle_attitude.timestamp) >= 100_ms) {
		perf_end(_loop_perf);
		return;
	}

	const uint64_t now = hrt_absolute_time();
	const Dcmf attitude(Quatf(_vehicle_attitude.q));

	const bool nominal_setpoint_updated = _om_mpc_setpoint_sub.update(&_nominal_setpoint);
	_vehicle_control_mode_sub.update(&_vehicle_control_mode);
	_vehicle_land_detected_sub.update(&_vehicle_land_detected);
	_vehicle_status_sub.update(&_vehicle_status);
	const bool requested = _nominal_setpoint.timestamp != 0
			&& hrt_elapsed_time(&_nominal_setpoint.timestamp) < 100_ms
			&& PX4_ISFINITE(_nominal_setpoint.thrust_acceleration)
			&& Vector3f(_nominal_setpoint.rates).isAllFinite()
			&& Vector3f(_nominal_setpoint.mpc_disturbance).isAllFinite();
	const int32_t source = math::constrain<int32_t>(_param_acceleration_source.get(), 1, 2);

	if (!requested) {
		resetTransition();
		_last_status_publish = 0;
		perf_end(_loop_perf);
		return;
	}

	// With the acceleration middle layer disabled this module is a literal
	// message adapter: publish each 100 Hz MPC command once. Sensor-rate
	// execution is only needed when acceleration INDI is enabled.
	if (_param_acceleration_enable.get() != 1 && !nominal_setpoint_updated) {
		perf_end(_loop_perf);
		return;
	}

	updateSensorInputs(attitude);
	updateForceHistory(attitude);

	const bool acceleration_requested = _param_acceleration_enable.get() == 1;
	const AccelerationFilterState &selected = source == 2
			? _imu_acceleration_filter : _ekf_acceleration_filter;

	if (acceleration_requested) {
		// The attitude callback only wakes this work item. Run the controller once
		// for each new sample from the selected acceleration source.
		const bool acceleration_sample_updated = selected.timestamp > 0
				&& (source != _last_control_acceleration_source
				    || selected.timestamp != _last_control_acceleration_timestamp);

		if (!acceleration_sample_updated) {
			perf_end(_loop_perf);
			return;
		}

		const float rate_max_hz = math::constrain(_param_rate_max.get(), 50.f, 500.f);
		const uint64_t minimum_interval_us = static_cast<uint64_t>(1e6f / rate_max_hz);

		if (_last_run_timestamp > 0 && now < _last_run_timestamp + minimum_interval_us) {
			perf_end(_loop_perf);
			return;
		}

		_last_control_acceleration_timestamp = selected.timestamp;
		_last_control_acceleration_source = source;
	}

	const float dt = _last_run_timestamp > 0 && now > _last_run_timestamp
			 ? math::constrain((now - _last_run_timestamp) * 1e-6f, 0.001f, 0.02f) : 0.004f;
	_last_run_timestamp = now;

	const bool flight_enabled = _vehicle_control_mode.flag_armed
			&& _vehicle_status.takeoff_time != 0 && !_vehicle_land_detected.landed;
	const bool enabled = requested && _param_enable.get() == 1 && flight_enabled
			&& _control.paramsValid() && attitude.isAllFinite();

	const bool acceleration_fresh = selected.timestamp > 0
			&& hrt_elapsed_time(&selected.timestamp) < 100_ms && selected.value.isAllFinite();
	Vector3f allocated_force_ned{NAN, NAN, NAN};
	const bool force_valid = acceleration_fresh
			&& getDelayedAllocatedForce(selected.timestamp, allocated_force_ned);
	const bool feedback_valid = enabled && acceleration_fresh && force_valid;
	Vector3f disturbance{NAN, NAN, NAN};
	OmMpcIndiControl::Output corrected{};
	bool corrected_valid = false;

	const Vector3f nominal_rates(_nominal_setpoint.rates);
	const Vector3f nominal_thrust{0.f, 0.f,
		-math::constrain(_nominal_setpoint.thrust_acceleration
				 * _param_hover_thrust.get() / CONSTANTS_ONE_G, 0.f, 1.f)};
	const Vector3f mpc_disturbance(_nominal_setpoint.mpc_disturbance);

	if (feedback_valid && _param_hover_thrust.get() > FLT_EPSILON) {
		const Vector3f disturbance_raw = selected.value - Vector3f{0.f, 0.f, CONSTANTS_ONE_G}
				- allocated_force_ned * (CONSTANTS_ONE_G / _param_hover_thrust.get());
		disturbance = constrainDisturbance(disturbance_raw,
				math::max(_param_disturbance_limit.get(), 0.f));
		const Dcmf control_attitude = attitudeAt(now, attitude);
		corrected_valid = _control.update(control_attitude, nominal_rates, nominal_thrust,
			mpc_disturbance, selected.value, allocated_force_ned, corrected);
	}

	const bool acceleration_active = enabled && acceleration_requested
			&& corrected_valid;

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

	vehicle_rates_setpoint_s output{};
	output.timestamp = hrt_absolute_time();
	output.roll = output_rates(0);
	output.pitch = output_rates(1);
	output.yaw = output_rates(2);
	output_thrust.copyTo(output.thrust_body);
	const bool disturbance_valid = feedback_valid && disturbance.isAllFinite()
			&& _param_disturbance_enable.get() == 1
			&& (_param_acceleration_enable.get() != 1 || _transition_progress >= 1.f);

	_setpoint_pub.publish(output);
	publishStatus(now, selected.timestamp, acceleration_active,
		disturbance_valid, disturbance);
	perf_end(_loop_perf);
}

int McOmMpcIndi::task_spawn(int argc, char *argv[])
{
	McOmMpcIndi *instance = new McOmMpcIndi();

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

int McOmMpcIndi::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int McOmMpcIndi::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
OM-MPC acceleration-INDI middle layer. It consumes the external nominal
OmMpcSetpoint, estimator/IMU acceleration and allocator feedback, then publishes
the standard VehicleRatesSetpoint consumed by mc_rate_control. Other controllers
remain on their existing setpoint paths.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("mc_om_mpc_indi", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mc_om_mpc_indi_main(int argc, char *argv[])
{
	return ModuleBase::main(McOmMpcIndi::desc, argc, argv);
}
