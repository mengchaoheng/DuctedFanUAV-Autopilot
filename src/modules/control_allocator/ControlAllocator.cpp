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
 * @file ControlAllocator.cpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocator.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <inttypes.h>
#include <cstdio>
#include <time.h>

using namespace matrix;
using namespace time_literals;

namespace
{
constexpr float kAllocationFeedbackSampleRateChangeThreshold = 0.1f;
constexpr hrt_abstime kAllocationFeedbackFilterStatusPublishInterval = 1_s;

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

const char *allocationAxisName(int axis)
{
	switch (axis) {
	case ControlAllocation::ROLL:
		return "Mx";

	case ControlAllocation::PITCH:
		return "My";

	case ControlAllocation::YAW:
		return "Mz";

	case ControlAllocation::THRUST_X:
		return "Fx";

	case ControlAllocation::THRUST_Y:
		return "Fy";

	case ControlAllocation::THRUST_Z:
		return "Fz";
	}

	return "?";
}

uint8_t allocationActiveAxesMask(const ControlAllocation &allocation)
{
	const auto &effectiveness = allocation.getEffectivenessMatrix();
	uint8_t axes_mask = 0;

	for (int axis = 0; axis < ControlAllocation::NUM_AXES; axis++) {
		for (int actuator = 0; actuator < allocation.numConfiguredActuators(); actuator++) {
			if (fabsf(effectiveness(axis, actuator)) > FLT_EPSILON) {
				axes_mask |= static_cast<uint8_t>(1u << axis);
				break;
			}
		}
	}

	return axes_mask;
}

const char *allocationSolverStatusName(int8_t status)
{
	switch (status) {
	case control_allocator_status_s::SOLVER_STATUS_OK:
		return "ok";

	case control_allocator_status_s::SOLVER_STATUS_UNAVAILABLE:
		return "unavailable";

	case control_allocator_status_s::SOLVER_STATUS_FAILED:
		return "failed";

	case control_allocator_status_s::SOLVER_STATUS_NOT_RUN:
	default:
		return "not_run";
	}
}

const char *allocationMethodName(AllocationMethod method)
{
	switch (method) {
	case AllocationMethod::NONE:
		return "None";

	case AllocationMethod::PSEUDO_INVERSE:
		return "Pseudo-inverse";

	case AllocationMethod::SEQUENTIAL_DESATURATION:
		return "Sequential desaturation";

	case AllocationMethod::AUTO:
		return "Auto";

	case AllocationMethod::INV:
		return "INV";

	case AllocationMethod::DP_LPCA:
		return "DP_LPCA";

	case AllocationMethod::DPSCALED_LPCA:
		return "DPscaled_LPCA";

	case AllocationMethod::PCA:
		return "PCA";

	case AllocationMethod::WLS:
		return "WLS";
	}

	return "Unknown";
}

void formatAllocationAxesMask(uint8_t axes_mask, char *buffer, size_t buffer_len)
{
	if (buffer_len == 0) {
		return;
	}

	size_t offset = 0;
	bool first = true;
	buffer[0] = '\0';

	for (int axis = 0; axis < ControlAllocation::NUM_AXES; axis++) {
		if ((axes_mask & (1u << axis)) == 0) {
			continue;
		}

		const int written = snprintf(buffer + offset, buffer_len - offset, "%s%s", first ? "" : "/",
					     allocationAxisName(axis));

		if (written < 0) {
			buffer[0] = '\0';
			return;
		}

		if (static_cast<size_t>(written) >= buffer_len - offset) {
			buffer[buffer_len - 1] = '\0';
			return;
		}

		offset += static_cast<size_t>(written);
		first = false;
	}

	if (first) {
		snprintf(buffer, buffer_len, "none");
	}
}

void setPrioritySetpoint(ControlAllocation &allocator,
			 const matrix::Vector3f &torque_indi_feedback)
{
	matrix::Vector<float, ControlAllocation::NUM_AXES> higher;
	higher.setZero();

	higher(ControlAllocation::ROLL) = torque_indi_feedback(0);
	higher(ControlAllocation::PITCH) = torque_indi_feedback(1);
	higher(ControlAllocation::YAW) = torque_indi_feedback(2);

	allocator.setControlSetpointPriorityHigher(higher);
}
}

ModuleBase::Descriptor ControlAllocator::desc{task_spawn, custom_command, print_usage};

ControlAllocator::ControlAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_control_allocator_status_pub[0].advertise();
	_control_allocator_status_pub[1].advertise();

	_actuator_motors_pub.advertise();
	_actuator_servos_pub.advertise();
	_actuator_servos_trim_pub.advertise();
	_allocation_value_pub.advertise();

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_R%u_SLEW", i);
		_param_handles.slew_rate_motors[i] = param_find(buffer);
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV%u_SLEW", i);
		_param_handles.slew_rate_servos[i] = param_find(buffer);
	}

	reset_allocation_feedback_state(allocation_feedback_filter_status_s::FILTER_EVENT_INITIAL);
	parameters_updated();
}

ControlAllocator::~ControlAllocator()
{
	for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
		delete _control_allocation[i];
	}

	delete _actuator_effectiveness;

	perf_free(_loop_perf);
}

bool
ControlAllocator::init()
{
	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	ScheduleDelayed(50_ms);
#endif

	return true;
}

void
ControlAllocator::parameters_updated()
{
	const float torque_cutoff = math::max(_param_ca_torque_cutoff.get(), 0.f);
	const float force_cutoff = math::max(_param_ca_force_cutoff.get(), 0.f);

	if (fabsf(torque_cutoff - _allocation_feedback_torque_cutoff) > FLT_EPSILON
	    || fabsf(force_cutoff - _allocation_feedback_force_cutoff) > FLT_EPSILON) {
		_allocation_feedback_torque_cutoff = torque_cutoff;
		_allocation_feedback_force_cutoff = force_cutoff;
		_allocation_feedback_filters_configured = false;
		_allocation_feedback_filter_event_reason = allocation_feedback_filter_status_s::FILTER_EVENT_PARAMETER;
		_allocation_feedback_filter_event_timestamp = hrt_absolute_time();
	}

	_has_slew_rate = false;

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		param_get(_param_handles.slew_rate_motors[i], &_params.slew_rate_motors[i]);
		_has_slew_rate |= _params.slew_rate_motors[i] > FLT_EPSILON;
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		param_get(_param_handles.slew_rate_servos[i], &_params.slew_rate_servos[i]);
		_has_slew_rate |= _params.slew_rate_servos[i] > FLT_EPSILON;
	}

	// Allocation method & effectiveness source
	// Do this first: in case a new method is loaded, it will be configured below
	bool updated = update_effectiveness_source();
	update_allocation_method(updated); // must be called after update_effectiveness_source()

	if (_num_control_allocation == 0) {
		return;
	}

	for (int i = 0; i < _num_control_allocation; ++i) {
		if (_allocation_methods[i] == AllocationMethod::WLS) {
			matrix::Vector<float, NUM_AXES> control_weights;
			control_weights(0) = _param_ca_wls_roll_weight.get();
			control_weights(1) = _param_ca_wls_pitch_weight.get();
			control_weights(2) = _param_ca_wls_yaw_weight.get();
			control_weights(3) = _param_ca_wls_force_x_weight.get();
			control_weights(4) = _param_ca_wls_force_y_weight.get();
			control_weights(5) = _param_ca_wls_force_z_weight.get();
			static_cast<ControlAllocationWLS *>(_control_allocation[i])->setWeights(control_weights,
					_param_ca_wls_actuator_weight.get(), _param_ca_wls_gamma.get());
		}

		_control_allocation[i]->updateParameters();
	}

	update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::CONFIGURATION_UPDATE);
}

void
ControlAllocator::update_allocation_method(bool force)
{
	AllocationMethod configured_method = (AllocationMethod)_param_ca_method.get();

	if (!_actuator_effectiveness) {
		PX4_ERR("_actuator_effectiveness null");
		return;
	}

	if (_allocation_method_id != configured_method || force) {

		ActuatorVector actuator_sp[ActuatorEffectiveness::MAX_NUM_MATRICES];

		// Cleanup first
		for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
			// Save current state
			if (_control_allocation[i] != nullptr) {
				actuator_sp[i] = _control_allocation[i]->getActuatorSetpoint();
			}

			delete _control_allocation[i];
			_control_allocation[i] = nullptr;
			_allocation_methods[i] = AllocationMethod::NONE;
			_allocation_running_time_us[i] = 0;
			_allocation_running_time_avg_us[i] = 0.f;
			_allocation_running_time_count[i] = 0;
		}

		_num_control_allocation = _actuator_effectiveness->numMatrices();

		AllocationMethod desired_methods[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getDesiredAllocationMethod(desired_methods);

		bool normalize_rpy[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getNormalizeRPY(normalize_rpy);

		for (int i = 0; i < _num_control_allocation; ++i) {
			AllocationMethod method = configured_method;

			if (configured_method == AllocationMethod::AUTO) {
				method = desired_methods[i];
			}

			// CA16/17 matrix 0 contains only the force-producing rotor. LPCA/PCA
			// require the multi-axis surface matrix and would otherwise fall back
			// every cycle. Keep the requested method on matrix 1 only.
			const bool ducted_fan_force_matrix = i == 0
					&& (_effectiveness_source_id == EffectivenessSource::DUCTED_FAN
					    || _effectiveness_source_id == EffectivenessSource::DUCTED_FAN_TAILSITTER_VTOL);
			const bool requires_surface_matrix = method == AllocationMethod::DP_LPCA
					|| method == AllocationMethod::DPSCALED_LPCA
					|| method == AllocationMethod::PCA;

			if (ducted_fan_force_matrix && requires_surface_matrix) {
				method = AllocationMethod::INV;
			}

			_allocation_methods[i] = method;

			switch (method) {
			case AllocationMethod::PSEUDO_INVERSE:
				_control_allocation[i] = new ControlAllocationPseudoInverse();
				break;

			case AllocationMethod::SEQUENTIAL_DESATURATION:
				_control_allocation[i] = new ControlAllocationSequentialDesaturation();
				break;

			case AllocationMethod::INV:
				_control_allocation[i] = new ControlAllocationInv();
				break;

			case AllocationMethod::DP_LPCA:
				_control_allocation[i] = new ControlAllocationDPLPCA();
				break;

			case AllocationMethod::DPSCALED_LPCA:
				_control_allocation[i] = new ControlAllocationDPscaledLPCA();
				break;

			case AllocationMethod::PCA:
				_control_allocation[i] = new ControlAllocationPCA();
				break;

			case AllocationMethod::WLS:
				_control_allocation[i] = new ControlAllocationWLS();
				break;

			default:
				PX4_ERR("Unknown allocation method");
				break;
			}

			if (_control_allocation[i] == nullptr) {
				PX4_ERR("alloc failed");
				_num_control_allocation = 0;

			} else {
				_control_allocation[i]->setNormalizeRPY(normalize_rpy[i]);
				_control_allocation[i]->setActuatorSetpoint(actuator_sp[i]);
			}
		}

		_allocation_method_id = configured_method;
	}
}

bool
ControlAllocator::update_effectiveness_source()
{
	const EffectivenessSource source = (EffectivenessSource)_param_ca_airframe.get();

	if (_effectiveness_source_id != source) {

		// try to instanciate new effectiveness source
		ActuatorEffectiveness *tmp = nullptr;

		switch (source) {
		case EffectivenessSource::NONE:
		case EffectivenessSource::MULTIROTOR:
			tmp = new ActuatorEffectivenessMultirotor(this);
			break;

		case EffectivenessSource::STANDARD_VTOL:
			tmp = new ActuatorEffectivenessStandardVTOL(this);
			break;

		case EffectivenessSource::TILTROTOR_VTOL:
			tmp = new ActuatorEffectivenessTiltrotorVTOL(this);
			break;

		case EffectivenessSource::TAILSITTER_VTOL:
			tmp = new ActuatorEffectivenessTailsitterVTOL(this);
			break;

		case EffectivenessSource::FIXED_WING:
			tmp = new ActuatorEffectivenessFixedWing(this);
			break;

		case EffectivenessSource::MOTORS_6DOF: // just a different UI from MULTIROTOR
			tmp = new ActuatorEffectivenessUUV(this);
			break;

		case EffectivenessSource::MULTIROTOR_WITH_TILT:
			tmp = new ActuatorEffectivenessMCTilt(this);
			break;

		case EffectivenessSource::CUSTOM:
			tmp = new ActuatorEffectivenessCustom(this);
			break;

		case EffectivenessSource::DUCTED_FAN:
			tmp = new ActuatorEffectivenessDuctedFan(this);
			break;

		case EffectivenessSource::DUCTED_FAN_TAILSITTER_VTOL:
			tmp = new ActuatorEffectivenessDuctedFanTailsitterVTOL(this);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_ESC:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::MOTORS);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_SERVO:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::SERVOS);
			break;

		case EffectivenessSource::HELICOPTER_COAXIAL:
			tmp = new ActuatorEffectivenessHelicopterCoaxial(this);
			break;

		case EffectivenessSource::SPACECRAFT_2D:
			tmp = new ActuatorEffectivenessSpacecraft(this);
			break;

		case EffectivenessSource::ROVER_ACKERMANN: // Unreachable: Rover startup scripts don't load control_allocator. Controllers publish actuator_outputs directly.
		case EffectivenessSource::ROVER_DIFFERENTIAL:
		case EffectivenessSource::ROVER_MECANUM:
		default:
			PX4_ERR("Unknown airframe");
			break;
		}

		// Replace previous source with new one
		if (tmp == nullptr) {
			// It did not work, forget about it
			PX4_ERR("Actuator effectiveness init failed");
			_param_ca_airframe.set((int)_effectiveness_source_id);

		} else {
			// Swap effectiveness sources
			delete _actuator_effectiveness;
			_actuator_effectiveness = tmp;

			// Save source id
			_effectiveness_source_id = source;
		}

		return true;
	}

	return false;
}

void
ControlAllocator::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	// Push backup schedule
	ScheduleDelayed(50_ms);
#endif

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		if (_handled_motor_failure_bitmask == 0) {
			// We don't update the geometry after an actuator failure, as it could lead to unexpected results
			// (e.g. a user could add/remove motors, such that the bitmask isn't correct anymore)
			updateParams();
			parameters_updated();
		}
	}

	if (_num_control_allocation == 0 || _actuator_effectiveness == nullptr) {
		return;
	}

	{
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.update(&vehicle_status)) {

			_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
			_is_vtol = vehicle_status.is_vtol;

			ActuatorEffectiveness::FlightPhase flight_phase{ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT};

			// Check if the current flight phase is HOVER or FIXED_WING
			if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				flight_phase = ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT;

			} else {
				flight_phase = ActuatorEffectiveness::FlightPhase::FORWARD_FLIGHT;
			}

			// Special cases for VTOL in transition
			if (vehicle_status.is_vtol && vehicle_status.in_transition_mode) {
				if (vehicle_status.in_transition_to_fw) {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_HF_TO_FF;

				} else {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_FF_TO_HF;
				}
			}

			// Forward to effectiveness source
			const ActuatorEffectiveness::FlightPhase previous_flight_phase = _actuator_effectiveness->getFlightPhase();
			_actuator_effectiveness->setFlightPhase(flight_phase);

			if (_actuator_effectiveness->effectivenessDependsOnFlightPhase()
			    && previous_flight_phase != _actuator_effectiveness->getFlightPhase()) {
				update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::CONFIGURATION_UPDATE);
			}
		}
	}

	{
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) {
			const bool publish_controls_prev = _publish_controls;
			_publish_controls = vehicle_control_mode.flag_control_allocation_enabled;

			if (publish_controls_prev && !_publish_controls) {
				reset_allocation_feedback_state(allocation_feedback_filter_status_s::FILTER_EVENT_GAP);
			}
		}
	}

	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	const hrt_abstime now = hrt_absolute_time();
	const bool previous_run_valid = _last_run != 0;
	const float dt_unconstrained = static_cast<float>(now - _last_run) * 1e-6f;
	const float dt = math::constrain(dt_unconstrained, 0.0002f, 0.02f);
	const bool allocation_feedback_gap = previous_run_valid && dt_unconstrained > 0.1f;

	_actuator_group_preflight_check.handleCommand(now, _effectiveness_source_id == EffectivenessSource::TILTROTOR_VTOL);
	_actuator_group_preflight_check.updateState(now);

	bool do_update = false;
	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	// Run allocator on torque changes
	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);
		_torque_sp_indi_feedback = matrix::Vector3f(vehicle_torque_setpoint.xyz_indi_feedback);
		_torque_sp_indi_feedback_valid = vehicle_torque_setpoint.xyz_indi_feedback_valid;

		do_update = true;
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;

	}

	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);

		_actuator_effectiveness->stopMotorsBasedOnThrustSetpoint(_thrust_sp);
		_thrust_sp.nanToZero();
	}

	if (do_update) {
		_last_run = now;

		check_for_motor_failures();

		update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::NO_EXTERNAL_UPDATE);

		// Set control setpoint vector(s)
		matrix::Vector<float, NUM_AXES> c[ActuatorEffectiveness::MAX_NUM_MATRICES];
		matrix::Vector3f torque_sp_indi_feedback[ActuatorEffectiveness::MAX_NUM_MATRICES] {};
		bool torque_sp_indi_feedback_valid[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

		c[0](0) = _torque_sp(0);
		c[0](1) = _torque_sp(1);
		c[0](2) = _torque_sp(2);
		c[0](3) = _thrust_sp(0);
		c[0](4) = _thrust_sp(1);
		c[0](5) = _thrust_sp(2);
		torque_sp_indi_feedback[0] = _torque_sp_indi_feedback;
		torque_sp_indi_feedback_valid[0] = _torque_sp_indi_feedback_valid;

		if (_num_control_allocation > 1) {
			if (_vehicle_torque_setpoint1_sub.copy(&vehicle_torque_setpoint)) {
				c[1](0) = vehicle_torque_setpoint.xyz[0];
				c[1](1) = vehicle_torque_setpoint.xyz[1];
				c[1](2) = vehicle_torque_setpoint.xyz[2];
				torque_sp_indi_feedback[1] = matrix::Vector3f(vehicle_torque_setpoint.xyz_indi_feedback);
				torque_sp_indi_feedback_valid[1] = vehicle_torque_setpoint.xyz_indi_feedback_valid;
			}

			if (_vehicle_thrust_setpoint1_sub.copy(&vehicle_thrust_setpoint)) {
				c[1](3) = vehicle_thrust_setpoint.xyz[0];
				c[1](4) = vehicle_thrust_setpoint.xyz[1];
				c[1](5) = vehicle_thrust_setpoint.xyz[2];
			}
		}

		_actuator_group_preflight_check.applyOverrides(c, _is_vtol, *_actuator_effectiveness);

		for (int i = 0; i < _num_control_allocation; ++i) {

			_control_allocation[i]->setControlSetpoint(c[i]);

			if (_allocation_methods[i] == AllocationMethod::PCA && torque_sp_indi_feedback_valid[i]) {
				setPrioritySetpoint(*_control_allocation[i], torque_sp_indi_feedback[i]);
			}

			// Do allocation
			const hrt_abstime allocation_start = runtimeMeasurementTimeUs();
			_control_allocation[i]->allocate();
			const hrt_abstime allocation_running_time_us = runtimeMeasurementTimeUs() - allocation_start;
			_allocation_running_time_us[i] = allocation_running_time_us;

			if (_allocation_running_time_count[i] < UINT32_MAX) {
				_allocation_running_time_count[i]++;
				_allocation_running_time_avg_us[i] += (static_cast<float>(allocation_running_time_us) -
								       _allocation_running_time_avg_us[i]) /
								      static_cast<float>(_allocation_running_time_count[i]);
			}

			_actuator_effectiveness->allocateAuxilaryControls(dt, i, _control_allocation[i]->_actuator_sp); //flaps and spoilers
			_actuator_effectiveness->updateSetpoint(c[i], i, _control_allocation[i]->_actuator_sp,
								_control_allocation[i]->getActuatorMin(), _control_allocation[i]->getActuatorMax());

			if (i == 0) {
				// The motors are always in allocation 0
				handle_stopped_motors(now);
			}

			if (_has_slew_rate) {
				_control_allocation[i]->applySlewRateLimit(dt);
			}

			_control_allocation[i]->clipActuatorSetpoint();
		}

		// Form one physical INDI feedback sample from the final post-processed
		// setpoints. CA0/9 use matrix 0; CA16/17 use force matrix 0 and torque matrix 1.
		if (_publish_controls && indi_feedback_supported()) {
			if (allocation_feedback_gap) {
				reset_allocation_feedback_state(allocation_feedback_filter_status_s::FILTER_EVENT_GAP);
			}

			publish_allocation_value(previous_run_valid && !allocation_feedback_gap ? dt : NAN);
		}
	}

	// Publish actuator setpoint and allocator status
	publish_actuator_controls();

	// Publish status at limited rate, as it's somewhat expensive and we use it for slower dynamics
	// (i.e. anti-integrator windup)
	if (now - _last_status_pub >= 5_ms) {
		publish_control_allocator_status(0);

		if (_num_control_allocation > 1) {
			publish_control_allocator_status(1);
		}

		_last_status_pub = now;
	}

	perf_end(_loop_perf);
}

void
ControlAllocator::update_effectiveness_matrix_if_needed(EffectivenessUpdateReason reason)
{
	ActuatorEffectiveness::Configuration config{};

	if (reason == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE
	    && hrt_elapsed_time(&_last_effectiveness_update) < 100_ms) { // rate-limit updates
		return;
	}

	if (_actuator_effectiveness->getEffectivenessMatrix(config, reason)) {
		_last_effectiveness_update = hrt_absolute_time();

		memcpy(_control_allocation_selection_indexes, config.matrix_selection_indexes,
		       sizeof(_control_allocation_selection_indexes));

		// Get the minimum and maximum depending on type and configuration
		ActuatorEffectiveness::ActuatorVector minimum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector maximum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector slew_rate[ActuatorEffectiveness::MAX_NUM_MATRICES];
		int actuator_idx = 0;
		int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

		actuator_servos_trim_s trims{};
		static_assert(actuator_servos_trim_s::NUM_CONTROLS == actuator_servos_s::NUM_CONTROLS, "size mismatch");

		for (int actuator_type = 0; actuator_type < (int)ActuatorType::COUNT; ++actuator_type) {
			_num_actuators[actuator_type] = config.num_actuators[actuator_type];

			for (int actuator_type_idx = 0; actuator_type_idx < config.num_actuators[actuator_type]; ++actuator_type_idx) {
				if (actuator_idx >= NUM_ACTUATORS) {
					_num_actuators[actuator_type] = 0;
					PX4_ERR("Too many actuators");
					break;
				}

				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if ((ActuatorType)actuator_type == ActuatorType::MOTORS) {
					if (actuator_type_idx >= MAX_NUM_MOTORS) {
						PX4_ERR("Too many motors");
						_num_actuators[actuator_type] = 0;
						break;
					}

					if (_param_r_rev.get() & (1u << actuator_type_idx)) {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;

					} else {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 0.f;
					}

					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_motors[actuator_type_idx];

				} else if ((ActuatorType)actuator_type == ActuatorType::SERVOS) {
					if (actuator_type_idx >= MAX_NUM_SERVOS) {
						PX4_ERR("Too many servos");
						_num_actuators[actuator_type] = 0;
						break;
					}

					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_servos[actuator_type_idx];
					trims.trim[actuator_type_idx] = config.trim[selected_matrix](actuator_idx_matrix[selected_matrix]);

				} else {
					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
				}

				maximum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 1.f;

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		// Handle failed actuators
		if (_handled_motor_failure_bitmask) {
			actuator_idx = 0;
			memset(&actuator_idx_matrix, 0, sizeof(actuator_idx_matrix));

			for (int motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if (_handled_motor_failure_bitmask & (1 << motors_idx)) {
					ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[selected_matrix];

					for (int i = 0; i < NUM_AXES; i++) {
						matrix(i, actuator_idx_matrix[selected_matrix]) = 0.0f;
					}
				}

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		for (int i = 0; i < _num_control_allocation; ++i) {
			_control_allocation[i]->setActuatorMin(minimum[i]);
			_control_allocation[i]->setActuatorMax(maximum[i]);
			_control_allocation[i]->setSlewRateLimit(slew_rate[i]);

			// Set all the elements of a row to 0 if that row has weak authority.
			// That ensures that the algorithm doesn't try to control axes with only marginal control authority,
			// which in turn would degrade the control of the main axes that actually should and can be controlled.

			// Preserve physical B*U for feedback before the original weak-row suppression.
			_allocation_effectiveness_bu[i] = config.effectiveness_matrices[i];
			ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[i];

			for (int n = 0; n < NUM_AXES; n++) {
				bool all_entries_small = true;

				for (int m = 0; m < config.num_actuators_matrix[i]; m++) {
					if (fabsf(matrix(n, m)) > 0.05f) {
						all_entries_small = false;
					}
				}

				if (all_entries_small) {
					matrix.row(n) = 0.f;
				}
			}

			// Assign control effectiveness matrix
			int total_num_actuators = config.num_actuators_matrix[i];
			_control_allocation[i]->setEffectivenessMatrix(config.effectiveness_matrices[i], config.trim[i],
					config.linearization_point[i], total_num_actuators, reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE);
		}

		trims.timestamp = hrt_absolute_time();
		_actuator_servos_trim_pub.publish(trims);
	}
}


void
ControlAllocator::handle_stopped_motors(const hrt_abstime now)
{
	const ActuatorBitmask stopped_motors_due_to_effectiveness = _actuator_effectiveness->getStoppedMotors();

	const ActuatorBitmask stopped_motors = stopped_motors_due_to_effectiveness
					       | _handled_motor_failure_bitmask
					       | _motor_stop_mask;
	_stopped_motors = stopped_motors;

	// Handle stopped motors by setting NaN
	const unsigned int allocation_index = 0;  // Motors always in allocation 0
	_control_allocation[allocation_index]->applyNanToActuators(stopped_motors);

	// Apply ice shedding, which applies _only_ to stopped motors
	const bool any_stopped_motor_failed = 0 != (stopped_motors_due_to_effectiveness & (_handled_motor_failure_bitmask | _motor_stop_mask));
	const float ice_shedding_output = get_ice_shedding_output(now);

	if (ice_shedding_output > FLT_EPSILON && !any_stopped_motor_failed) {
		for (int motors_idx = 0; motors_idx < _num_actuators[allocation_index] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
			if (stopped_motors & 1u << motors_idx) {
				_control_allocation[allocation_index]->_actuator_sp(motors_idx) = ice_shedding_output;
			}
		}
	}
}

void
ControlAllocator::publish_control_allocator_status(int matrix_index)
{
	control_allocator_status_s control_allocator_status{};
	control_allocator_status.timestamp = hrt_absolute_time();
	control_allocator_status.actuator_group_preflight_check_active = _actuator_group_preflight_check.isActive();

	const ControlAllocation *allocation = _control_allocation[matrix_index];
	const ControlAllocation::Diagnostics &diagnostics = allocation->getDiagnostics();
	control_allocator_status.fallback = allocation->usedFallback();
	control_allocator_status.solver_status = diagnostics.solver_status;
	control_allocator_status.solver_err = diagnostics.solver_err;
	control_allocator_status.full_row_rank = diagnostics.full_row_rank;
	control_allocator_status.solver_rho = diagnostics.solver_rho;
	control_allocator_status.solver_prepare_time = diagnostics.solver_prepare_time;
	control_allocator_status.solver_core_time = diagnostics.solver_core_time;
	control_allocator_status.solver_post_time = diagnostics.solver_post_time;
	// TODO: disabled motors (?)

	// Allocated control
	const matrix::Vector<float, NUM_AXES> &allocated_control = _control_allocation[matrix_index]->getAllocatedControl();

	// Unallocated control
	const matrix::Vector<float, NUM_AXES> unallocated_control = _control_allocation[matrix_index]->getControlSetpoint() -
			allocated_control;
	control_allocator_status.unallocated_torque[0] = unallocated_control(0);
	control_allocator_status.unallocated_torque[1] = unallocated_control(1);
	control_allocator_status.unallocated_torque[2] = unallocated_control(2);
	control_allocator_status.unallocated_thrust[0] = unallocated_control(3);
	control_allocator_status.unallocated_thrust[1] = unallocated_control(4);
	control_allocator_status.unallocated_thrust[2] = unallocated_control(5);

	// override control_allocator_status in customized saturation logic for certain effectiveness types
	_actuator_effectiveness->getUnallocatedControl(matrix_index, control_allocator_status);

	// Allocation success flags
	control_allocator_status.torque_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_torque[0],
			control_allocator_status.unallocated_torque[1],
			control_allocator_status.unallocated_torque[2]).norm_squared() < 1e-6f);
	control_allocator_status.thrust_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_thrust[0],
			control_allocator_status.unallocated_thrust[1],
			control_allocator_status.unallocated_thrust[2]).norm_squared() < 1e-6f);

	// Actuator saturation
	const ActuatorVector &actuator_sp = _control_allocation[matrix_index]->getActuatorSetpoint();
	const ActuatorVector &actuator_min = _control_allocation[matrix_index]->getActuatorMin();
	const ActuatorVector &actuator_max = _control_allocation[matrix_index]->getActuatorMax();

	for (int i = 0; i < NUM_ACTUATORS; i++) {
		if (actuator_sp(i) > (actuator_max(i) - FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_UPPER;

		} else if (actuator_sp(i) < (actuator_min(i) + FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_LOWER;
		}
	}

	// Handled motor failures
	control_allocator_status.handled_motor_failure_mask = _handled_motor_failure_bitmask;
	control_allocator_status.motor_stop_mask = _motor_stop_mask;
	control_allocator_status.allocation_running_time = static_cast<float>(_allocation_running_time_us[matrix_index]);
	control_allocator_status.allocation_running_time_avg = _allocation_running_time_avg_us[matrix_index];
	control_allocator_status.allocation_running_time_samples = _allocation_running_time_count[matrix_index];

	_control_allocator_status_pub[matrix_index].publish(control_allocator_status);
}

float
ControlAllocator::get_ice_shedding_output(hrt_abstime now)
{
	const float period_sec = _param_ice_shedding_period.get();

	const bool feature_disabled_by_param = period_sec <= FLT_EPSILON;
	const bool in_forward_flight = _actuator_effectiveness->getFlightPhase() == ActuatorEffectiveness::FlightPhase::FORWARD_FLIGHT;

	// If any stopped motor has failed, the feature will create much more
	// torque than in the nominal case, and becomes pointless anyway as we
	// cannot go back to multicopter
	const bool apply_shedding = _is_vtol && in_forward_flight;

	if (feature_disabled_by_param || !apply_shedding) {
		return 0.0f;

	} else {
		// Square wave output
		const float elapsed_in_period = fmodf(static_cast<float>(now) / 1_s, period_sec);
		const float ice_shedding_output = elapsed_in_period < ICE_SHEDDING_ON_SEC ? ICE_SHEDDING_OUTPUT : 0.0f;

		return ice_shedding_output;
	}
}

void
ControlAllocator::publish_actuator_controls()
{
	if (!_publish_controls) {
		return;
	}

	actuator_motors_s actuator_motors;
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _timestamp_sample;

	actuator_servos_s actuator_servos;
	actuator_servos.timestamp = actuator_motors.timestamp;
	actuator_servos.timestamp_sample = _timestamp_sample;

	actuator_motors.reversible_flags = _param_r_rev.get();

	int actuator_idx = 0;
	int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

	// motors
	int motors_idx;

	for (motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
		int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
		float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
		actuator_motors.control[motors_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
		++actuator_idx_matrix[selected_matrix];
		++actuator_idx;
	}

	for (int i = motors_idx; i < actuator_motors_s::NUM_CONTROLS; i++) {
		actuator_motors.control[i] = NAN;
	}

	_actuator_motors_pub.publish(actuator_motors);

	// servos
	if (_num_actuators[1] > 0) {
		int servos_idx;

		for (servos_idx = 0; servos_idx < _num_actuators[1] && servos_idx < actuator_servos_s::NUM_CONTROLS; servos_idx++) {
			int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
			float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
			actuator_servos.control[servos_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
			++actuator_idx_matrix[selected_matrix];
			++actuator_idx;
		}

		for (int i = servos_idx; i < actuator_servos_s::NUM_CONTROLS; i++) {
			actuator_servos.control[i] = NAN;
		}

		_actuator_servos_pub.publish(actuator_servos);
	}
}

void
ControlAllocator::reset_allocation_feedback_state(uint8_t event_flag)
{
	_allocation_feedback_sample_interval.reset();
	_allocation_feedback_filter_sample_freq = NAN;
	_allocation_feedback_filters_configured = false;
	_allocation_wrench_feedback.setAll(NAN);

	for (int axis = 0; axis < NUM_AXES; axis++) {
		_allocation_wrench_feedback_filter[axis].reset(0.f);
	}

	_allocation_feedback_filter_reset_count++;
	_allocation_feedback_filter_event_reason = event_flag;
	_allocation_feedback_filter_event_timestamp = hrt_absolute_time();
}

void
ControlAllocator::update_allocation_feedback_sample_rate(float sample_interval_s)
{
	if (!PX4_ISFINITE(sample_interval_s) || sample_interval_s <= 0.f) {
		return;
	}

	_allocation_feedback_sample_interval.update(sample_interval_s);

	if (!_allocation_feedback_sample_interval.valid()) {
		return;
	}

	const float sample_freq = 1.f / _allocation_feedback_sample_interval.mean();
	const bool sample_rate_changed = !PX4_ISFINITE(_allocation_feedback_filter_sample_freq)
					 || fabsf(sample_freq - _allocation_feedback_filter_sample_freq)
					 > kAllocationFeedbackSampleRateChangeThreshold
					 * _allocation_feedback_filter_sample_freq;

	if (!_allocation_feedback_filters_configured || sample_rate_changed) {
		const uint8_t event_flag = sample_rate_changed && PX4_ISFINITE(_allocation_feedback_filter_sample_freq)
					   ? allocation_feedback_filter_status_s::FILTER_EVENT_SAMPLE_RATE : 0;
		update_allocation_feedback_filter_config(sample_freq, event_flag);
	}
}

void
ControlAllocator::update_allocation_feedback_filter_config(float sample_freq, uint8_t event_flag)
{
	for (int axis = 0; axis < NUM_AXES; axis++) {
		const float cutoff = axis < 3 ? _allocation_feedback_torque_cutoff : _allocation_feedback_force_cutoff;

		if (cutoff > FLT_EPSILON) {
			_allocation_wrench_feedback_filter[axis].set_cutoff_frequency(sample_freq, cutoff);

		} else {
			_allocation_wrench_feedback_filter[axis].disable();
		}

		// set_cutoff_frequency() clears the delay state. Preserve the current value.
		const float reset_value = PX4_ISFINITE(_allocation_wrench_feedback(axis)) ?
					  _allocation_wrench_feedback(axis) : 0.f;
		_allocation_wrench_feedback_filter[axis].reset(reset_value);
	}

	_allocation_feedback_filter_sample_freq = sample_freq;
	_allocation_feedback_filters_configured = true;
	_allocation_feedback_filter_reconfigure_count++;

	if (event_flag != 0) {
		_allocation_feedback_filter_event_reason = event_flag;
		_allocation_feedback_filter_event_timestamp = hrt_absolute_time();
	}
}

void
ControlAllocator::filter_allocation_wrench(const matrix::Vector<float, NUM_AXES> &raw_wrench,
		float sample_interval_s, matrix::Vector<float, NUM_AXES> &filtered_wrench)
{
	update_allocation_feedback_sample_rate(sample_interval_s);
	bool axis_reinitialized = false;

	for (int axis = 0; axis < NUM_AXES; axis++) {
		const float cutoff = axis < 3 ? _allocation_feedback_torque_cutoff : _allocation_feedback_force_cutoff;

		if (!PX4_ISFINITE(raw_wrench(axis))) {
			_allocation_wrench_feedback(axis) = NAN;
			filtered_wrench(axis) = NAN;
			continue;
		}

		if (cutoff > FLT_EPSILON && _allocation_feedback_filters_configured) {
			if (!PX4_ISFINITE(_allocation_wrench_feedback(axis))) {
				_allocation_wrench_feedback_filter[axis].reset(raw_wrench(axis));
				axis_reinitialized = true;
			}

			_allocation_wrench_feedback(axis) = _allocation_wrench_feedback_filter[axis].apply(raw_wrench(axis));

		} else {
			_allocation_wrench_feedback(axis) = raw_wrench(axis);
		}

		filtered_wrench(axis) = _allocation_wrench_feedback(axis);
	}

	if (axis_reinitialized) {
		_allocation_feedback_filter_reset_count++;
		_allocation_feedback_filter_event_reason = allocation_feedback_filter_status_s::FILTER_EVENT_AXIS_RECOVERY;
		_allocation_feedback_filter_event_timestamp = hrt_absolute_time();
	}
}

bool
ControlAllocator::calculate_allocation_wrench(int matrix_index,
		matrix::Vector<float, NUM_AXES> &wrench) const
{
	const ControlAllocation *allocation = _control_allocation[matrix_index];
	ControlAllocation::ActuatorVector actuator_delta;
	actuator_delta.setZero();

	for (int actuator = 0; actuator < allocation->numConfiguredActuators(); actuator++) {
		const float actuator_sp = allocation->getActuatorSetpoint()(actuator);
		const float actuator_trim = allocation->_actuator_trim(actuator);

		if (PX4_ISFINITE(actuator_sp)) {
			actuator_delta(actuator) = actuator_sp - actuator_trim;

		} else if (matrix_index == 0 && (_stopped_motors & (ActuatorBitmask{1} << actuator))) {
			// Supported layouts keep stopped motors in matrix 0. They are published
			// as NaN but physically produce zero thrust.
			actuator_delta(actuator) = -actuator_trim;

		} else {
			return false;
		}
	}

	wrench = _allocation_effectiveness_bu[matrix_index] * actuator_delta;
	return true;
}

bool
ControlAllocator::indi_feedback_supported() const
{
	switch (_effectiveness_source_id) {
	case EffectivenessSource::MULTIROTOR:
	case EffectivenessSource::CUSTOM:
		return _num_control_allocation == 1;

	case EffectivenessSource::DUCTED_FAN:
	case EffectivenessSource::DUCTED_FAN_TAILSITTER_VTOL:
		return _num_control_allocation == 2;

	default:
		return false;
	}
}

int
ControlAllocator::indi_torque_matrix() const
{
	return _effectiveness_source_id == EffectivenessSource::DUCTED_FAN
	       || _effectiveness_source_id == EffectivenessSource::DUCTED_FAN_TAILSITTER_VTOL ? 1 : 0;
}

void
ControlAllocator::publish_allocation_value(float sample_interval_s)
{
	const hrt_abstime now = hrt_absolute_time();
	matrix::Vector<float, NUM_AXES> force_matrix_wrench;
	matrix::Vector<float, NUM_AXES> torque_matrix_wrench;
	matrix::Vector<float, NUM_AXES> raw_wrench;
	raw_wrench.setAll(NAN);
	const int torque_matrix = indi_torque_matrix();

	const bool force_valid = calculate_allocation_wrench(0, force_matrix_wrench);
	const bool torque_valid = torque_matrix == 0 ? force_valid :
				  calculate_allocation_wrench(torque_matrix, torque_matrix_wrench);

	if (force_valid && torque_valid) {
		if (torque_matrix == 0) {
			torque_matrix_wrench = force_matrix_wrench;
		}

		raw_wrench(0) = torque_matrix_wrench(0);
		raw_wrench(1) = torque_matrix_wrench(1);
		raw_wrench(2) = torque_matrix_wrench(2);
		raw_wrench(3) = force_matrix_wrench(3);
		raw_wrench(4) = force_matrix_wrench(4);
		raw_wrench(5) = force_matrix_wrench(5);
	}

	matrix::Vector<float, NUM_AXES> filtered_wrench;
	filter_allocation_wrench(raw_wrench, sample_interval_s, filtered_wrench);

	allocation_value_s msg{};
	msg.timestamp = now;
	msg.timestamp_sample = _timestamp_sample;
	const matrix::Vector<float, NUM_AXES> &torque_scale = _control_allocation[torque_matrix]->_control_allocation_scale;
	const matrix::Vector<float, NUM_AXES> &force_scale = _control_allocation[0]->_control_allocation_scale;

	for (int axis = 0; axis < 3; axis++) {
		msg.raw_allocated_torque[axis] = raw_wrench(axis);
		msg.raw_allocated_force[axis] = raw_wrench(axis + 3);
		msg.allocated_torque[axis] = filtered_wrench(axis);
		msg.allocated_force[axis] = filtered_wrench(axis + 3);
		msg.torque_setpoint_scale[axis] = torque_scale(axis);
		msg.force_setpoint_scale[axis] = force_scale(axis + 3);
	}

	_allocation_value_pub.publish(msg);
	publish_allocation_feedback_filter_status(now);
}

void
ControlAllocator::publish_allocation_feedback_filter_status(hrt_abstime now)
{
	if (_allocation_feedback_status_last_publish == 0) {
		_allocation_feedback_status_last_publish = now;
		return;
	}

	if (now - _allocation_feedback_status_last_publish < kAllocationFeedbackFilterStatusPublishInterval) {
		return;
	}

	allocation_feedback_filter_status_s status{};
	status.timestamp = now;
	status.sample_interval = _allocation_feedback_sample_interval.valid()
				 ? _allocation_feedback_sample_interval.mean() : NAN;
	status.filter_sample_rate_hz = _allocation_feedback_filter_sample_freq;
	status.torque_filter_cutoff_hz = _allocation_feedback_filters_configured
					 && _allocation_feedback_torque_cutoff > FLT_EPSILON
					 ? _allocation_wrench_feedback_filter[0].get_cutoff_freq() : 0.f;
	status.force_filter_cutoff_hz = _allocation_feedback_filters_configured
					&& _allocation_feedback_force_cutoff > FLT_EPSILON
					? _allocation_wrench_feedback_filter[3].get_cutoff_freq() : 0.f;
	status.filter_reset_count = _allocation_feedback_filter_reset_count;
	status.filter_reconfigure_count = _allocation_feedback_filter_reconfigure_count;
	status.filter_event_timestamp = _allocation_feedback_filter_event_timestamp;
	status.filter_event_reason = _allocation_feedback_filter_event_reason;
	status.filters_configured = _allocation_feedback_filters_configured;

	_allocation_feedback_filter_status_pub.publish(status);

	_allocation_feedback_status_last_publish = now;
}

void
ControlAllocator::check_for_motor_failures()
{
	failure_detector_status_s failure_detector_status;

	if ((FailureMode)_param_ca_failure_mode.get() > FailureMode::IGNORE
	    && _failure_detector_status_sub.update(&failure_detector_status)) {

		if (_motor_stop_mask != failure_detector_status.motor_stop_mask) {
			_motor_stop_mask = failure_detector_status.motor_stop_mask;
			PX4_WARN("Stopping motors (%d)", _motor_stop_mask);
		}

		if (failure_detector_status.fd_motor) {
			if (_handled_motor_failure_bitmask != failure_detector_status.motor_failure_mask) {
				// motor failure bitmask changed
				switch ((FailureMode)_param_ca_failure_mode.get()) {
				case FailureMode::REMOVE_FIRST_FAILING_MOTOR: {
						// Count number of failed motors
						const int num_motors_failed = math::countSetBits(failure_detector_status.motor_failure_mask);

						// Only handle if it is the first failure
						if (_handled_motor_failure_bitmask == 0 && num_motors_failed == 1) {
							_handled_motor_failure_bitmask = failure_detector_status.motor_failure_mask;
							PX4_WARN("Removing motor from allocation (0x%x)", _handled_motor_failure_bitmask);

							for (int i = 0; i < _num_control_allocation; ++i) {
								_control_allocation[i]->setHadActuatorFailure(true);
							}

							update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
						}
					}
					break;

				default:
					break;
				}

			}

		} else if (_handled_motor_failure_bitmask != 0) {
			// Clear bitmask completely
			PX4_INFO("Restoring all motors");
			_handled_motor_failure_bitmask = 0;

			for (int i = 0; i < _num_control_allocation; ++i) {
				_control_allocation[i]->setHadActuatorFailure(false);
			}

			update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
		}
	}
}

int ControlAllocator::task_spawn(int argc, char *argv[])
{
	ControlAllocator *instance = new ControlAllocator();

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

int ControlAllocator::print_status()
{
	PX4_INFO("Running");

	// Print current allocation method
	switch (_allocation_method_id) {
	case AllocationMethod::NONE:
		PX4_INFO("Method: None");
		break;

	case AllocationMethod::PSEUDO_INVERSE:
		PX4_INFO("Method: Pseudo-inverse");
		break;

	case AllocationMethod::SEQUENTIAL_DESATURATION:
		PX4_INFO("Method: Sequential desaturation");
		break;

	case AllocationMethod::AUTO:
		PX4_INFO("Method: Auto");
		break;

	case AllocationMethod::INV:
	case AllocationMethod::DP_LPCA:
	case AllocationMethod::DPSCALED_LPCA:
	case AllocationMethod::PCA:
	case AllocationMethod::WLS:
		PX4_INFO("Method: %s", allocationMethodName(_allocation_method_id));
		break;
	}

	// Print current airframe
	if (_actuator_effectiveness != nullptr) {
		PX4_INFO("Effectiveness Source: %s", _actuator_effectiveness->name());
	}

	PX4_INFO("Allocator input/output: normalized wrench / normalized actuator input");
	PX4_INFO("Allocation wrench feedback: torque cutoff=%.2f Hz, force cutoff=%.2f Hz (no actuator dynamics)",
		 (double)_allocation_feedback_torque_cutoff, (double)_allocation_feedback_force_cutoff);

	// Print current effectiveness matrix
	for (int i = 0; i < _num_control_allocation; ++i) {
		const ActuatorEffectiveness::EffectivenessMatrix &effectiveness = _control_allocation[i]->getEffectivenessMatrix();

		if (_num_control_allocation > 1) {
			PX4_INFO("Instance: %i", i);
		}

		PX4_INFO("  Method: %s", allocationMethodName(_allocation_methods[i]));

		PX4_INFO("  Effectiveness =");
		int num_configured = _control_allocation[i]->numConfiguredActuators();

		// print column numbering
		if (num_configured > 1) {
			printf("  ");

			for (int col = 0; col < num_configured; col++) {
				printf("|%2u      ", col);
			}

			printf("\n");
		}

		// Print effectiveness matrix with row labels
		const char *row_labels[] = {"Mx", "My", "Mz", "Fx", "Fy", "Fz"};

		for (int row = 0; row < 6; row++) {
			printf("%2s|", row_labels[row]);

			for (int col = 0; col < num_configured; col++) {
				double d = static_cast<double>(effectiveness(row, col));

				// avoid -0.0 for display
				if (fabs(d - 0.0) < 1e-9) {
					// print fixed width zero
					printf(" 0       ");

				} else if ((fabs(d) < 1e-4) || (fabs(d) >= 10.0)) {
					printf("% .1e ", d);

				} else {
					printf("% 6.5f ", d);
				}
			}

			printf("\n");
		}

		PX4_INFO("  minimum =");

		// print column numbering
		if (num_configured > 1) {
			printf("  ");

			for (int col = 0; col < num_configured; col++) {
				printf("|%2u      ", col);
			}

			printf("\n");
		}

		printf("  |");

		for (int col = 0; col < num_configured; col++) {
			double d = static_cast<double>(_control_allocation[i]->getActuatorMin()(col));

			// avoid -0.0 for display
			if (fabs(d - 0.0) < 1e-9) {
				// print fixed width zero
				printf(" 0       ");

			} else if ((fabs(d) < 1e-4) || (fabs(d) >= 10.0)) {
				printf("% .1e ", d);

			} else {
				printf("% 6.5f ", d);
			}
		}

		printf("\n");
		PX4_INFO("  maximum =");

		// print column numbering
		if (num_configured > 1) {
			printf("  ");

			for (int col = 0; col < num_configured; col++) {
				printf("|%2u      ", col);
			}

			printf("\n");
		}

		printf("  |");

		for (int col = 0; col < num_configured; col++) {
			double d = static_cast<double>(_control_allocation[i]->getActuatorMax()(col));

			// avoid -0.0 for display
			if (fabs(d - 0.0) < 1e-9) {
				// print fixed width zero
				printf(" 0       ");

			} else if ((fabs(d) < 1e-4) || (fabs(d) >= 10.0)) {
				printf("% .1e ", d);

			} else {
				printf("% 6.5f ", d);
			}
		}

		printf("\n");
		PX4_INFO("  Configured actuators: %i", num_configured);

		const ControlAllocation::Diagnostics &diagnostics = _control_allocation[i]->getDiagnostics();
		char axes[32];
		formatAllocationAxesMask(allocationActiveAxesMask(*_control_allocation[i]), axes, sizeof(axes));
		PX4_INFO("  diagnostics: solver=%s(%d) err=%d fallback=%d full_rank=%d rows=%u axes=%s rho=%.4g",
			 allocationSolverStatusName(diagnostics.solver_status), diagnostics.solver_status,
			 diagnostics.solver_err, (int)_control_allocation[i]->usedFallback(), (int)diagnostics.full_row_rank,
			 (unsigned)diagnostics.active_rows, axes, (double)diagnostics.solver_rho);
		PX4_INFO("  timing_us: prep=%.1f core=%.1f post=%.1f alloc=%" PRIu64 " avg=%.2f samples=%" PRIu32,
			 (double)diagnostics.solver_prepare_time, (double)diagnostics.solver_core_time,
			 (double)diagnostics.solver_post_time, _allocation_running_time_us[i],
			 (double)_allocation_running_time_avg_us[i], _allocation_running_time_count[i]);
	}

	if (_handled_motor_failure_bitmask) {
		PX4_INFO("Failed motors: %i (0x%x)", math::countSetBits(_handled_motor_failure_bitmask),
			 _handled_motor_failure_bitmask);
	}

	// Print perf
	perf_print_counter(_loop_perf);

	return 0;
}

int ControlAllocator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ControlAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements control allocation. It takes torque and thrust setpoints
as inputs and outputs actuator setpoint messages.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("control_allocator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Control Allocator app start / stop handling function
 */
extern "C" __EXPORT int control_allocator_main(int argc, char *argv[]);

int control_allocator_main(int argc, char *argv[])
{
	return ModuleBase::main(ControlAllocator::desc, argc, argv);
}
