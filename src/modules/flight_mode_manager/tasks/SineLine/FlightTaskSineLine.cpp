/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "FlightTaskSineLine.hpp"

#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

FlightTaskSineLine::FlightTaskSineLine()
{
	_sticks_data_required = false;
}

bool FlightTaskSineLine::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskManualAltitudeSmoothVel::activate(last_setpoint);

	_use_parameterized_geometry = _param_mc_orbit_source.get() == 1;
	_radius = _use_parameterized_geometry
		  ? math::constrain(_param_mc_orbit_rad.get(), kRadiusMin, _param_mc_orbit_rad_max.get())
		  : kInitialRadius;
	_trajectory_velocity = _param_mc_orbit_vel.get();
	_sanitizeParameters(_radius, _trajectory_velocity);
	_started_positive = _trajectory_velocity >= 0.f;
	_center = _position;
	_phase = 0.f;
	_initial_heading = _yaw;
	_yaw_behaviour = _param_mc_orbit_yaw_mod.get();
	_in_approach = true;

	_heading_smoothing.reset(PX4_ISFINITE(last_setpoint.yaw) ? last_setpoint.yaw : _yaw,
				 PX4_ISFINITE(last_setpoint.yawspeed) ? last_setpoint.yawspeed : 0.f);
	_slew_rate_velocity.setForcedValue(0.f);
	_slew_rate_velocity.setSlewRate(_param_mpc_acc_hor.get());
	_resetApproachSmoothing();

	ret = ret && _position.isAllFinite() && _velocity.isAllFinite() && PX4_ISFINITE(_yaw);
	return ret;
}

bool FlightTaskSineLine::applyCommandParameters(const vehicle_command_s &command, bool &success)
{
	if (command.command != vehicle_command_s::VEHICLE_CMD_DO_ORBIT) {
		return false;
	}

	success = true;

	if (_use_parameterized_geometry) {
		return true;
	}

	float new_radius = _radius;
	float new_absolute_velocity = fabsf(_trajectory_velocity);
	bool new_positive = _trajectory_velocity >= 0.f;

	if (PX4_ISFINITE(command.param1)) {
		new_positive = command.param1 > 0.f;
		new_radius = fabsf(command.param1);
	}

	if (PX4_ISFINITE(command.param2)) {
		new_absolute_velocity = fabsf(command.param2);
	}

	if (!math::isInRange(new_radius, kRadiusMin, _param_mc_orbit_rad_max.get())) {
		success = false;
		return true;
	}

	float new_velocity = signFromBool(new_positive) * new_absolute_velocity;
	_sanitizeParameters(new_radius, new_velocity);
	_radius = new_radius;
	_trajectory_velocity = new_velocity;
	_started_positive = new_positive;

	if (PX4_ISFINITE(command.param3)) {
		if (static_cast<uint8_t>(lroundf(command.param3)) == vehicle_command_s::ORBIT_YAW_BEHAVIOUR_UNCHANGED) {
			_yaw_behaviour = _param_mc_orbit_yaw_mod.get();

		} else {
			_yaw_behaviour = static_cast<int>(lroundf(command.param3));
		}
	}

	_initial_heading = _yaw;

	if (PX4_ISFINITE(command.param5) && PX4_ISFINITE(command.param6)) {
		if (_geo_projection.isInitialized()) {
			_center.xy() = _geo_projection.project(command.param5, command.param6);

		} else {
			success = false;
		}
	}

	if (PX4_ISFINITE(command.param7)) {
		if (_geo_projection.isInitialized()) {
			_center(2) = _global_local_alt0 - command.param7;

		} else {
			success = false;
		}
	}

	if (success) {
		_findClosestPhase();
		_in_approach = true;
		_slew_rate_velocity.setForcedValue(0.f);
		_resetApproachSmoothing();
	}

	return true;
}

bool FlightTaskSineLine::update()
{
	bool ret = true;
	_configurePositionSmoothing();

	if (_in_approach && _isAtApproachPoint()) {
		_in_approach = false;
		_slew_rate_velocity.setForcedValue(0.f);
		FlightTaskManualAltitudeSmoothVel::_smoothing.reset(
			PX4_ISFINITE(_acceleration_setpoint(2)) ? _acceleration_setpoint(2) : _acceleration(2),
			PX4_ISFINITE(_velocity_setpoint(2)) ? _velocity_setpoint(2) : _velocity(2),
			PX4_ISFINITE(_position_setpoint(2)) ? _position_setpoint(2) : _position(2));
	}

	if (_in_approach) {
		ret = FlightTask::update();
		_generateApproachSetpoints();

	} else {
		ret = FlightTaskManualAltitudeSmoothVel::update();
		_adjustParametersByStick();
		_generateTrackingSetpoints();
	}

	if (PX4_ISFINITE(_yaw_setpoint)) {
		_heading_smoothing.setMaxHeadingRate(math::radians(_param_mpc_yawrauto_max.get()));
		_heading_smoothing.setMaxHeadingAccel(math::radians(_param_mpc_yawrauto_acc.get()));
		_heading_smoothing.update(_yaw_setpoint, _deltatime);
		_yaw_setpoint = _heading_smoothing.getSmoothedHeading();
		_yawspeed_setpoint = _heading_smoothing.getSmoothedHeadingRate();
	}

	_sendTelemetry();
	return ret;
}

void FlightTaskSineLine::_sanitizeParameters(float &radius, float &velocity) const
{
	radius = math::constrain(radius, kRadiusMin, _param_mc_orbit_rad_max.get());
	velocity = math::constrain(velocity, -fabsf(_param_mpc_xy_vel_max.get()), fabsf(_param_mpc_xy_vel_max.get()));

	const float maximum_velocity = sqrtf(math::max(_param_mpc_acc_hor.get(), 0.f) * radius);

	if (fabsf(velocity) > maximum_velocity) {
		velocity = math::signNoZero(velocity) * maximum_velocity;
	}
}

void FlightTaskSineLine::_adjustParametersByStick()
{
	float radius = _radius;
	float velocity = _trajectory_velocity;

	switch (_yaw_behaviour) {
	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE:
		radius -= signFromBool(_started_positive) * _sticks.getRollExpo() * _deltatime * _param_mpc_xy_cruise.get();
		velocity += signFromBool(_started_positive) * _sticks.getPitchExpo() * _deltatime * _param_mpc_acc_hor.get();
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING:
	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_UNCONTROLLED:
	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED:
	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER:
	default:
		radius -= _sticks.getPitchExpo() * _deltatime * _param_mpc_xy_cruise.get();
		velocity += _sticks.getRollExpo() * _deltatime * _param_mpc_acc_hor.get();
		break;
	}

	_sanitizeParameters(radius, velocity);
	_radius = radius;
	_trajectory_velocity = velocity;
}

void FlightTaskSineLine::_configurePositionSmoothing()
{
	_position_smoothing.setMaxAllowedHorizontalError(_param_mpc_xy_err_max.get());
	_position_smoothing.setVerticalAcceptanceRadius(_param_nav_mc_alt_rad.get());
	_position_smoothing.setCruiseSpeed(_param_mpc_xy_cruise.get());
	_position_smoothing.setHorizontalTrajectoryGain(_param_mpc_xy_traj_p.get());
	_position_smoothing.setTargetAcceptanceRadius(kHorizontalAcceptanceRadius);
	_position_smoothing.setMaxAccelerationXY(_param_mpc_acc_hor.get());
	_position_smoothing.setMaxVelocityXY(_param_mpc_xy_vel_max.get());
	_position_smoothing.setMaxJerk(_param_mpc_jerk_auto.get());

	if (_velocity_setpoint(2) < 0.f) {
		_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_up.get());
		_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_up_max.get());

	} else {
		_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_down_max.get());
		_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_dn.get());
	}
}

void FlightTaskSineLine::_findClosestPhase()
{
	float minimum_distance_squared = INFINITY;

	for (int i = 0; i < kClosestPointSamples; ++i) {
		const float phase = 2.f * M_PI_F * static_cast<float>(i) / static_cast<float>(kClosestPointSamples);
		const float distance_squared = (_curvePosition(phase) - _position.xy()).norm_squared();

		if (distance_squared < minimum_distance_squared) {
			minimum_distance_squared = distance_squared;
			_phase = phase;
		}
	}
}

void FlightTaskSineLine::_generateApproachSetpoints()
{
	const Vector2f target_xy = _curvePosition(_phase);
	const Vector3f target{target_xy(0), target_xy(1), _center(2)};
	PositionSmoothing::PositionSmoothingSetpoints out_setpoints;
	_position_smoothing.generateSetpoints(_position, target, Vector3f{}, _deltatime, false, out_setpoints);

	const Vector2f position_to_center = _center.xy() - _position.xy();
	_yaw_setpoint = position_to_center.norm() > FLT_EPSILON
			? atan2f(position_to_center(1), position_to_center(0))
			: _initial_heading;

	_position_setpoint = out_setpoints.position;
	_velocity_setpoint = out_setpoints.velocity;
	_acceleration_setpoint = out_setpoints.acceleration;
	_jerk_setpoint = out_setpoints.jerk;
}

void FlightTaskSineLine::_generateTrackingSetpoints()
{
	/*
	 * p(phi) = center + [0, R sin(phi)]^T
	 * phi_dot = V / R
	 *
	 * With V allowed to pass through the entry slew limiter:
	 *
	 * p_dot  = [0, V cos(phi)]^T
	 * p_ddot = [0, V_dot cos(phi) - V^2 / R sin(phi)]^T
	 */
	const float previous_velocity = _slew_rate_velocity.getState();
	const float velocity = _slew_rate_velocity.update(_trajectory_velocity, _deltatime);
	const float velocity_derivative = _deltatime > FLT_EPSILON
					  ? (velocity - previous_velocity) / _deltatime : 0.f;
	const float phase_rate = velocity / _radius;
	const float phase_acceleration = velocity_derivative / _radius;
	const Vector2f first_derivative = _curveFirstDerivative(_phase);
	const Vector2f second_derivative = _curveSecondDerivative(_phase);
	const Vector2f feedforward_velocity = first_derivative * phase_rate;
	const Vector2f feedforward_acceleration = second_derivative * phase_rate * phase_rate
						 + first_derivative * phase_acceleration;

	_position_setpoint.xy() = _curvePosition(_phase);
	_velocity_setpoint.xy() = feedforward_velocity;
	_acceleration_setpoint.xy() = feedforward_acceleration;
	_jerk_setpoint(0) = NAN;
	_jerk_setpoint(1) = NAN;

	_generateYawSetpoint(feedforward_velocity);
	_phase = wrap_2pi(_phase + phase_rate * _deltatime);
}

void FlightTaskSineLine::_generateYawSetpoint(const Vector2f &velocity)
{
	Vector2f tangent = velocity;

	if (tangent.norm() < 0.01f) {
		const float next_motion = -sinf(_phase) * _trajectory_velocity * _trajectory_velocity / _radius;
		tangent = {0.f, fabsf(next_motion) > FLT_EPSILON ? next_motion : _trajectory_velocity};
	}

	switch (_yaw_behaviour) {
	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING:
		_yaw_setpoint = _initial_heading;
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_UNCONTROLLED:
		_yaw_setpoint = NAN;
		_yawspeed_setpoint = 0.f;
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED:
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER: {
			const Vector2f position_to_center = _center.xy() - _position.xy();
			_yaw_setpoint = position_to_center.norm() > kHorizontalAcceptanceRadius
					? atan2f(position_to_center(1), position_to_center(0))
					: atan2f(tangent(1), tangent(0));
		}
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE:
	default:
		_yaw_setpoint = atan2f(tangent(1), tangent(0));
		break;
	}
}

void FlightTaskSineLine::_resetApproachSmoothing()
{
	Vector3f position = _position;
	Vector3f velocity = _velocity;
	Vector3f acceleration = _acceleration;

	for (int i = 0; i < 3; ++i) {
		if (PX4_ISFINITE(_position_setpoint(i))) {
			position(i) = _position_setpoint(i);
		}

		if (PX4_ISFINITE(_velocity_setpoint(i))) {
			velocity(i) = _velocity_setpoint(i);
		}

		if (PX4_ISFINITE(_acceleration_setpoint(i))) {
			acceleration(i) = _acceleration_setpoint(i);
		}
	}

	_position_smoothing.reset(acceleration, velocity, position);
}

bool FlightTaskSineLine::_isAtApproachPoint() const
{
	const Vector2f target_xy = _curvePosition(_phase);
	return (target_xy - _position.xy()).norm() < kHorizontalAcceptanceRadius
	       && fabsf(_position(2) - _center(2)) < _param_nav_mc_alt_rad.get();
}

Vector2f FlightTaskSineLine::_curvePosition(float phase) const
{
	return _center.xy() + Vector2f{0.f, _radius * sinf(phase)};
}

Vector2f FlightTaskSineLine::_curveFirstDerivative(float phase) const
{
	return {0.f, _radius * cosf(phase)};
}

Vector2f FlightTaskSineLine::_curveSecondDerivative(float phase) const
{
	return {0.f, -_radius * sinf(phase)};
}

void FlightTaskSineLine::_ekfResetHandlerPositionXY(const Vector2f &delta_xy)
{
	_center.xy() += delta_xy;
	Vector3f smoothed_position = _position_smoothing.getCurrentPosition();
	smoothed_position.xy() += delta_xy;
	_position_smoothing.forceSetPosition(smoothed_position);
}

void FlightTaskSineLine::_ekfResetHandlerPositionZ(float delta_z)
{
	FlightTaskManualAltitudeSmoothVel::_ekfResetHandlerPositionZ(delta_z);
	_center(2) += delta_z;
	Vector3f smoothed_position = _position_smoothing.getCurrentPosition();
	smoothed_position(2) += delta_z;
	_position_smoothing.forceSetPosition(smoothed_position);
}

void FlightTaskSineLine::_ekfResetHandlerHeading(float delta_psi)
{
	FlightTaskManualAltitudeSmoothVel::_ekfResetHandlerHeading(delta_psi);
	_initial_heading = wrap_pi(_initial_heading + delta_psi);
}

bool FlightTaskSineLine::_sendTelemetry()
{
	if (!_geo_projection.isInitialized()) {
		return false;
	}

	orbit_status_s orbit_status{};
	orbit_status.radius = math::signNoZero(_trajectory_velocity) * _radius;
	orbit_status.frame = 0;
	orbit_status.yaw_behaviour = _yaw_behaviour;
	_geo_projection.reproject(_center(0), _center(1), orbit_status.x, orbit_status.y);
	const float local_altitude = PX4_ISFINITE(_position_setpoint(2)) ? _position_setpoint(2) : _position(2);
	orbit_status.z = _global_local_alt0 - local_altitude;
	orbit_status.timestamp = hrt_absolute_time();
	_orbit_status_pub.publish(orbit_status);
	return true;
}
