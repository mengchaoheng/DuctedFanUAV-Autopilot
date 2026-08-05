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

#include "FlightTaskFigureEight.hpp"

#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

FlightTaskFigureEight::FlightTaskFigureEight()
{
	_sticks_data_required = false;
}

bool FlightTaskFigureEight::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskManualAltitudeSmoothVel::activate(last_setpoint);

	_use_parameterized_geometry = _param_mc_orbit_source.get() == 1;
	_major_radius = _use_parameterized_geometry
			? math::constrain(fabsf(_param_mc_f8_x_amplitude.get()), 0.f, _param_mc_orbit_rad_max.get())
			: kInitialMajorRadius;
	_minor_radius = _use_parameterized_geometry
			? math::constrain(fabsf(_param_mc_f8_y_amplitude.get()), 0.f, _param_mc_orbit_rad_max.get())
			: kInitialMajorRadius * 0.5f;
	_vertical_amplitude = _use_parameterized_geometry
			? math::constrain(fabsf(_param_mc_f8_z_amplitude.get()), 0.f, _param_mc_orbit_rad_max.get())
			: 0.f;
	_use_angular_frequency = _use_parameterized_geometry;
	_trajectory_omega = _param_mc_f8_omega.get();
	if (_use_parameterized_geometry && _param_mc_f8_acceleration.get() > FLT_EPSILON) {
		// Conservative bound for ||q_theta_theta|| over the complete figure:
		// sqrt(Ax^2 + 16 Ay^2 + 16 Az^2). This keeps the effective Omega
		// constant instead of changing it with the current phase.
		const float second_derivative_bound = sqrtf(_major_radius * _major_radius
									 + 16.f * _minor_radius * _minor_radius
									 + 16.f * _vertical_amplitude * _vertical_amplitude);
		if (second_derivative_bound > FLT_EPSILON) {
			const float omega_limit = sqrtf(_param_mc_f8_acceleration.get() / second_derivative_bound);
			_trajectory_omega = math::constrain(_trajectory_omega, -omega_limit, omega_limit);
		}
	}
	_hold_parameterized_origin = _use_parameterized_geometry
			&& (fabsf(_trajectory_omega) <= FLT_EPSILON
			    || (_major_radius <= FLT_EPSILON && _minor_radius <= FLT_EPSILON));
	_trajectory_velocity = _use_parameterized_geometry ? _trajectory_omega * math::max(_major_radius, _minor_radius) : 1.f;
	if (!_use_parameterized_geometry) {
		_sanitizeParameters(_major_radius, _trajectory_velocity);
	}
	_started_clockwise = _trajectory_velocity > 0.f;
	_center = _position;
	_phase = 0.f;
	_setTrajectoryHeadingFromParameter();
	_initial_heading = _yaw;
	_yaw_behaviour = _param_mc_orbit_yaw_mod.get();

	_heading_smoothing.reset(PX4_ISFINITE(last_setpoint.yaw) ? last_setpoint.yaw : _yaw,
				 PX4_ISFINITE(last_setpoint.yawspeed) ? last_setpoint.yawspeed : 0.f);
	_slew_rate_velocity.setForcedValue(0.f);
	_slew_rate_velocity.setSlewRate(_param_mpc_acc_hor.get());
	_slew_rate_phase.setForcedValue(_use_parameterized_geometry ? _trajectory_omega : 0.f);
	_slew_rate_phase.setSlewRate(_param_mc_f8_acceleration.get() / math::max(_major_radius, 1.f));
	_resetApproachSmoothing();
	if (_use_parameterized_geometry && !_hold_parameterized_origin) {
		_findClosestPhase();
	}
	_in_approach = !_hold_parameterized_origin;

	ret = ret && _position.isAllFinite() && _velocity.isAllFinite() && PX4_ISFINITE(_yaw);
	return ret;
}

bool FlightTaskFigureEight::applyCommandParameters(const vehicle_command_s &command, bool &success)
{
	if (command.command != vehicle_command_s::VEHICLE_CMD_DO_ORBIT) {
		return false;
	}

	success = true;

	if (_use_parameterized_geometry) {
		return true;
	}

	float new_radius = _major_radius;
	float new_velocity = fabsf(_trajectory_velocity);
	float direction = math::signNoZero(_trajectory_velocity);

	if (PX4_ISFINITE(command.param1)) {
		direction = math::signNoZero(command.param1);
		new_radius = fabsf(command.param1);
	}

	if (PX4_ISFINITE(command.param2)) {
		new_velocity = fabsf(command.param2);
	}

	if (!math::isInRange(new_radius, kRadiusMin, _param_mc_orbit_rad_max.get())) {
		success = false;
		return true;
	}

	_sanitizeParameters(new_radius, new_velocity);
	_major_radius = new_radius;
	_minor_radius = _major_radius * 0.5f;
	_vertical_amplitude = 0.f;
	_use_angular_frequency = false;
	_trajectory_omega = 0.f;
	_trajectory_velocity = direction * new_velocity;
	_started_clockwise = direction > 0.f;

	if (PX4_ISFINITE(command.param3)) {
		if (static_cast<uint8_t>(lroundf(command.param3)) == vehicle_command_s::ORBIT_YAW_BEHAVIOUR_UNCHANGED) {
			_yaw_behaviour = _param_mc_orbit_yaw_mod.get();

		} else {
			_yaw_behaviour = static_cast<int>(lroundf(command.param3));
		}
	}

	_initial_heading = _yaw;
	_setTrajectoryHeadingFromParameter();

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
		_slew_rate_phase.setForcedValue(0.f);
		_resetApproachSmoothing();
	}

	return true;
}

bool FlightTaskFigureEight::update()
{
	bool ret = true;
	_configurePositionSmoothing();

	if (_hold_parameterized_origin) {
		ret = ret && FlightTaskManualAltitudeSmoothVel::update();
		_position_setpoint = _center;
		_velocity_setpoint.setAll(0.f);
		_acceleration_setpoint.setAll(0.f);
		_jerk_setpoint.setAll(0.f);
		_yaw_setpoint = _initial_heading;
		_yawspeed_setpoint = NAN;
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

	if (_in_approach && _isAtApproachPoint()) {
		_in_approach = false;
		_slew_rate_velocity.setForcedValue(0.f);
		_slew_rate_phase.setForcedValue(0.f);
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

void FlightTaskFigureEight::_sanitizeParameters(float &radius, float &velocity) const
{
	radius = math::constrain(radius, kRadiusMin, _param_mc_orbit_rad_max.get());
	velocity = math::constrain(velocity, -fabsf(_param_mpc_xy_vel_max.get()), fabsf(_param_mpc_xy_vel_max.get()));
}

void FlightTaskFigureEight::_adjustParametersByStick()
{
	// Parameter/RC-triggered trajectories keep their configured geometry and
	// speed. Stick adjustment is reserved for MAVLink-commanded trajectories.
	if (_use_parameterized_geometry) {
		return;
	}

	float radius = _major_radius;
	float velocity = _trajectory_velocity;

	switch (_yaw_behaviour) {
	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE:
		// Match Orbit stick semantics when the vehicle nose follows the path:
		// roll changes the lateral path size and pitch changes tangential speed.
		radius -= signFromBool(_started_clockwise) * _sticks.getRollExpo() * _deltatime * _param_mpc_xy_cruise.get();
		velocity += signFromBool(_started_clockwise) * _sticks.getPitchExpo() * _deltatime * _param_mpc_acc_hor.get();
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

	const float direction = math::signNoZero(velocity);
	float absolute_velocity = fabsf(velocity);

	_sanitizeParameters(radius, absolute_velocity);
	_major_radius = radius;
	_minor_radius = _major_radius * 0.5f;
	_trajectory_velocity = direction * absolute_velocity;
}

void FlightTaskFigureEight::_configurePositionSmoothing()
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

void FlightTaskFigureEight::_findClosestPhase()
{
	float minimum_distance_squared = INFINITY;
	float closest_phase = 0.f;

	for (int i = 0; i < kClosestPointSamples; ++i) {
		const float phase = 2.f * M_PI_F * static_cast<float>(i) / static_cast<float>(kClosestPointSamples);
		const float distance_squared = (_curvePosition(phase) - _position.xy()).norm_squared();

		if (distance_squared < minimum_distance_squared) {
			minimum_distance_squared = distance_squared;
			closest_phase = phase;
		}
	}

	_phase = closest_phase;
}

void FlightTaskFigureEight::_generateApproachSetpoints()
{
	const Vector2f target_xy = _curvePosition(_phase);
	const Vector3f target{target_xy(0), target_xy(1), _center(2) + _curveVerticalPosition(_phase)};
	PositionSmoothing::PositionSmoothingSetpoints out_setpoints;
	_position_smoothing.generateSetpoints(_position, target, Vector3f{}, _deltatime, false, out_setpoints);

	const Vector2f position_to_target = target_xy - _position.xy();
	_yaw_setpoint = position_to_target.norm() > FLT_EPSILON
			? atan2f(position_to_target(1), position_to_target(0))
			: _initial_heading;

	_position_setpoint = out_setpoints.position;
	_velocity_setpoint = out_setpoints.velocity;
	_acceleration_setpoint = out_setpoints.acceleration;
	_jerk_setpoint = out_setpoints.jerk;
}

void FlightTaskFigureEight::_generateTrackingSetpoints()
{
	/*
	 * With q(theta) = [A sin(theta), B sin(2 theta)]^T and
	 * p(theta) = center + R(psi) q(theta), the derivatives are
	 *
	 *   p_theta       = R(psi) [ A cos(theta),  2 B cos(2 theta)]^T
	 *   p_theta_theta = R(psi) [-A sin(theta), -4 B sin(2 theta)]^T
	 *
	 * The phase rate is chosen from the requested signed ground speed v:
	 *
	 *   theta_dot = v / ||p_theta||
	 *
	 * including tangential acceleration a_t from the speed slew limiter:
	 *
	 *   theta_ddot = a_t / ||p_theta||
	 *                 - (p_theta dot p_theta_theta) / ||p_theta||^2 * theta_dot^2
	 *
	 * Therefore the exact kinematic references are
	 *
	 *   position_sp     = p(theta)
	 *   velocity_ff     = p_theta theta_dot
	 *   acceleration_ff = p_theta_theta theta_dot^2 + p_theta theta_ddot
	 *
	 * The finite position reference enables the normal multicopter position
	 * outer loop, while velocity and acceleration are feed-forward terms only.
	 * MC_F8_ACC is applied once at activation using a conservative bound on
	 * ||p_theta_theta||, so the effective phase rate remains constant during
	 * the trajectory.
	 */
	const Vector2f path_position = _curvePosition(_phase);
	const Vector2f first_derivative = _curveFirstDerivative(_phase);
	const Vector2f second_derivative = _curveSecondDerivative(_phase);
	const float derivative_norm = math::max(first_derivative.norm(), 0.01f);
	const float curvature_speed_limit = _param_mpc_xy_vel_max.get();

	float phase_rate = 0.f;
	float phase_acceleration = 0.f;

	if (_use_angular_frequency) {
		// The parameterized harmonic form uses the same phase convention as
		// the benchmark reference: theta_dot is the configured omega. Limit
		// The activation-time global limit keeps this configured omega constant.
		phase_rate = _trajectory_omega;
		phase_acceleration = 0.f;

	} else {
		const float target_speed = math::signNoZero(_trajectory_velocity)
				   * math::min(fabsf(_trajectory_velocity), curvature_speed_limit);
		const float previous_speed = _slew_rate_velocity.getState();
		const float smoothed_speed = _slew_rate_velocity.update(target_speed, _deltatime);
		const float tangential_acceleration = _deltatime > FLT_EPSILON
					 ? (smoothed_speed - previous_speed) / _deltatime : 0.f;
		phase_rate = smoothed_speed / derivative_norm;
		phase_acceleration = tangential_acceleration / derivative_norm
				       - first_derivative.dot(second_derivative)
				       / (derivative_norm * derivative_norm) * phase_rate * phase_rate;
	}

	const Vector2f feedforward_velocity = first_derivative * phase_rate;
	const Vector2f feedforward_acceleration = second_derivative * phase_rate * phase_rate
						 + first_derivative * phase_acceleration;

	_position_setpoint.xy() = path_position;
	_velocity_setpoint.xy() = feedforward_velocity;
	_acceleration_setpoint.xy() = feedforward_acceleration;
	_position_setpoint(2) = _center(2) + _curveVerticalPosition(_phase);
	_velocity_setpoint(2) = _curveVerticalFirstDerivative(_phase) * phase_rate;
	_acceleration_setpoint(2) = _curveVerticalSecondDerivative(_phase) * phase_rate * phase_rate
					+ _curveVerticalFirstDerivative(_phase) * phase_acceleration;
	_jerk_setpoint(0) = NAN;
	_jerk_setpoint(1) = NAN;

	_generateYawSetpoint(feedforward_velocity);
	_phase = wrap_2pi(_phase + phase_rate * _deltatime);
}

void FlightTaskFigureEight::_generateYawSetpoint(const Vector2f &tangent_velocity)
{
	Vector2f tangent = tangent_velocity;

	if (tangent.norm() < 0.01f) {
		tangent = _curveFirstDerivative(_phase) * math::signNoZero(_trajectory_velocity);
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
		// Keep the yaw setpoint generated by FlightTaskManualAltitudeSmoothVel.
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

void FlightTaskFigureEight::_resetApproachSmoothing()
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

void FlightTaskFigureEight::_setTrajectoryHeadingFromParameter()
{
	// MC_F8_HDG directly defines the major-axis orientation of the shape in
	// the local NED frame. It is intentionally independent of vehicle yaw.
	_trajectory_heading = wrap_pi(math::radians(_param_mc_f8_heading.get()));
}

bool FlightTaskFigureEight::_isAtApproachPoint() const
{
	const Vector2f target_xy = _curvePosition(_phase);
	return (target_xy - _position.xy()).norm() < kHorizontalAcceptanceRadius
	       && fabsf(_position(2) - (_center(2) + _curveVerticalPosition(_phase))) < _param_nav_mc_alt_rad.get();
}

Vector2f FlightTaskFigureEight::_curvePosition(float phase) const
{
	const auto curve = motion_planning::HarmonicTrajectory2D::figureEight(_major_radius, _minor_radius);
	return _center.xy() + _rotateToLocalFrame(curve.evaluate(phase, 0));
}

Vector2f FlightTaskFigureEight::_curveFirstDerivative(float phase) const
{
	const auto curve = motion_planning::HarmonicTrajectory2D::figureEight(_major_radius, _minor_radius);
	return _rotateToLocalFrame(curve.evaluate(phase, 1));
}

Vector2f FlightTaskFigureEight::_curveSecondDerivative(float phase) const
{
	const auto curve = motion_planning::HarmonicTrajectory2D::figureEight(_major_radius, _minor_radius);
	return _rotateToLocalFrame(curve.evaluate(phase, 2));
}

float FlightTaskFigureEight::_curveVerticalPosition(float phase) const
{
	return _vertical_amplitude * sinf(2.f * phase);
}

float FlightTaskFigureEight::_curveVerticalFirstDerivative(float phase) const
{
	return 2.f * _vertical_amplitude * cosf(2.f * phase);
}

float FlightTaskFigureEight::_curveVerticalSecondDerivative(float phase) const
{
	return -4.f * _vertical_amplitude * sinf(2.f * phase);
}

Vector2f FlightTaskFigureEight::_rotateToLocalFrame(const Vector2f &vector) const
{
	const float cosine = cosf(_trajectory_heading);
	const float sine = sinf(_trajectory_heading);
	return {cosine * vector(0) - sine * vector(1), sine * vector(0) + cosine * vector(1)};
}

void FlightTaskFigureEight::_ekfResetHandlerPositionXY(const Vector2f &delta_xy)
{
	_center.xy() += delta_xy;
	Vector3f smoothed_position = _position_smoothing.getCurrentPosition();
	smoothed_position.xy() += delta_xy;
	_position_smoothing.forceSetPosition(smoothed_position);
}

void FlightTaskFigureEight::_ekfResetHandlerPositionZ(float delta_z)
{
	FlightTaskManualAltitudeSmoothVel::_ekfResetHandlerPositionZ(delta_z);
	_center(2) += delta_z;
	Vector3f smoothed_position = _position_smoothing.getCurrentPosition();
	smoothed_position(2) += delta_z;
	_position_smoothing.forceSetPosition(smoothed_position);
}

void FlightTaskFigureEight::_ekfResetHandlerHeading(float delta_psi)
{
	FlightTaskManualAltitudeSmoothVel::_ekfResetHandlerHeading(delta_psi);
	// The shape orientation is defined in the local NED frame and therefore
	// must not follow a vehicle heading estimate reset.
	_initial_heading = wrap_pi(_initial_heading + delta_psi);
}

bool FlightTaskFigureEight::_sendTelemetry()
{
	if (!_geo_projection.isInitialized()) {
		return false;
	}

	orbit_status_s orbit_status{};
	orbit_status.radius = math::signNoZero(_trajectory_velocity) * _major_radius;
	orbit_status.frame = 0;
	orbit_status.yaw_behaviour = _yaw_behaviour;
	_geo_projection.reproject(_center(0), _center(1), orbit_status.x, orbit_status.y);
	const float local_altitude = PX4_ISFINITE(_position_setpoint(2)) ? _position_setpoint(2) : _position(2);
	orbit_status.z = _global_local_alt0 - local_altitude;
	orbit_status.timestamp = hrt_absolute_time();
	_orbit_status_pub.publish(orbit_status);
	return true;
}
