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

	// MAV_CMD_DO_ORBIT supplies the actual major radius before setpoint generation.
	// Keep a valid internal value while the task and command are being connected.
	_major_radius = kInitialMajorRadius;
	_trajectory_velocity = _param_mc_f8_vel.get();
	_sanitizeParameters(_major_radius, _trajectory_velocity);
	_minor_ratio = _param_mc_f8_ratio.get();
	_minor_radius = _major_radius * _minor_ratio;
	_heading_offset = math::radians(_param_mc_f8_heading.get());
	_center = _position;
	_phase = 0.f;
	_setTrajectoryHeadingFromYaw();
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

bool FlightTaskFigureEight::applyCommandParameters(const vehicle_command_s &command, bool &success)
{
	if (command.command != vehicle_command_s::VEHICLE_CMD_DO_ORBIT) {
		return false;
	}

	success = true;
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
	_minor_radius = _major_radius * _minor_ratio;
	_trajectory_velocity = direction * new_velocity;

	if (PX4_ISFINITE(command.param3)) {
		if (static_cast<uint8_t>(lroundf(command.param3)) == vehicle_command_s::ORBIT_YAW_BEHAVIOUR_UNCHANGED) {
			_yaw_behaviour = _param_mc_orbit_yaw_mod.get();

		} else {
			_yaw_behaviour = static_cast<int>(lroundf(command.param3));
		}
	}

	_initial_heading = _yaw;
	_setTrajectoryHeadingFromYaw();

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

bool FlightTaskFigureEight::update()
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

void FlightTaskFigureEight::_sanitizeParameters(float &radius, float &velocity) const
{
	radius = math::constrain(radius, kRadiusMin, _param_mc_orbit_rad_max.get());
	velocity = math::constrain(fabsf(velocity), 0.f, fabsf(_param_mpc_xy_vel_max.get()));
}

void FlightTaskFigureEight::_adjustParametersByStick()
{
	float radius = _major_radius - _sticks.getPitchExpo() * _deltatime * _param_mpc_xy_cruise.get();
	float velocity = _trajectory_velocity + _sticks.getRollExpo() * _deltatime * _param_mpc_acc_hor.get();
	const float direction = math::signNoZero(velocity);
	float absolute_velocity = fabsf(velocity);

	_sanitizeParameters(radius, absolute_velocity);
	_major_radius = radius;
	_minor_radius = _major_radius * _minor_ratio;
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
	const Vector3f target{target_xy(0), target_xy(1), _center(2)};
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
	 * Therefore the exact kinematic feed-forward references are
	 *
	 *   velocity_ff     = p_theta theta_dot
	 *   acceleration_ff = p_theta_theta theta_dot^2 + p_theta theta_ddot
	 *
	 * and the commanded horizontal velocity additionally closes path error:
	 *
	 *   velocity_sp = velocity_ff + MPC_XY_TRAJ_P (p(theta) - position)
	 *
	 * Horizontal position_sp stays NAN in tracking so that velocity and
	 * acceleration feed-forward are not combined with an additional outer
	 * position loop. MC_F8_ACC limits v through the local curvature kappa:
	 *
	 *   kappa = |p_theta x p_theta_theta| / ||p_theta||^3
	 *   |v| <= sqrt(MC_F8_ACC / kappa)
	 */
	const Vector2f path_position = _curvePosition(_phase);
	const Vector2f first_derivative = _curveFirstDerivative(_phase);
	const Vector2f second_derivative = _curveSecondDerivative(_phase);
	const float derivative_norm = math::max(first_derivative.norm(), 0.01f);
	const float cross_product = first_derivative(0) * second_derivative(1)
				    - first_derivative(1) * second_derivative(0);
	const float curvature = fabsf(cross_product) / (derivative_norm * derivative_norm * derivative_norm);
	const float curvature_speed_limit = curvature > FLT_EPSILON ? sqrtf(_param_mc_f8_acceleration.get() / curvature)
					  : _param_mpc_xy_vel_max.get();
	const float target_speed = math::signNoZero(_trajectory_velocity)
				   * math::min(fabsf(_trajectory_velocity), curvature_speed_limit);

	const float previous_speed = _slew_rate_velocity.getState();
	const float smoothed_speed = _slew_rate_velocity.update(target_speed, _deltatime);
	const float tangential_acceleration = _deltatime > FLT_EPSILON
					 ? (smoothed_speed - previous_speed) / _deltatime : 0.f;
	const float phase_rate = smoothed_speed / derivative_norm;
	const float phase_acceleration = tangential_acceleration / derivative_norm
				       - first_derivative.dot(second_derivative)
				       / (derivative_norm * derivative_norm) * phase_rate * phase_rate;

	const Vector2f feedforward_velocity = first_derivative * phase_rate;
	const Vector2f feedforward_acceleration = second_derivative * phase_rate * phase_rate
						 + first_derivative * phase_acceleration;
	Vector2f correction_velocity = (path_position - _position.xy()) * _param_mpc_xy_traj_p.get();
	const float correction_limit = math::min(_param_mpc_xy_cruise.get(), _param_mpc_xy_vel_max.get());

	if (correction_velocity.norm() > correction_limit) {
		correction_velocity = correction_velocity.unit_or_zero() * correction_limit;
	}

	Vector2f velocity_setpoint = feedforward_velocity + correction_velocity;

	if (velocity_setpoint.norm() > _param_mpc_xy_vel_max.get()) {
		velocity_setpoint = velocity_setpoint.unit_or_zero() * _param_mpc_xy_vel_max.get();
	}

	_position_setpoint(0) = NAN;
	_position_setpoint(1) = NAN;
	_velocity_setpoint.xy() = velocity_setpoint;
	_acceleration_setpoint.xy() = feedforward_acceleration;
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

void FlightTaskFigureEight::_setTrajectoryHeadingFromYaw()
{
	// At phase zero the unrotated curve tangent is [A, 2B]. Rotate the
	// curve so that an RC-only entry starts moving along the current heading.
	const float tangent_heading = atan2f(2.f * _minor_radius, _major_radius);
	_trajectory_heading = wrap_pi(_yaw + _heading_offset - tangent_heading);
}

bool FlightTaskFigureEight::_isAtApproachPoint() const
{
	const Vector2f target_xy = _curvePosition(_phase);
	return (target_xy - _position.xy()).norm() < kHorizontalAcceptanceRadius
	       && fabsf(_position(2) - _center(2)) < _param_nav_mc_alt_rad.get();
}

Vector2f FlightTaskFigureEight::_curvePosition(float phase) const
{
	const Vector2f curve{_major_radius * sinf(phase), _minor_radius * sinf(2.f * phase)};
	return _center.xy() + _rotateToLocalFrame(curve);
}

Vector2f FlightTaskFigureEight::_curveFirstDerivative(float phase) const
{
	return _rotateToLocalFrame({_major_radius * cosf(phase), 2.f * _minor_radius * cosf(2.f * phase)});
}

Vector2f FlightTaskFigureEight::_curveSecondDerivative(float phase) const
{
	return _rotateToLocalFrame({-_major_radius * sinf(phase), -4.f * _minor_radius * sinf(2.f * phase)});
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
	_trajectory_heading = wrap_pi(_trajectory_heading + delta_psi);
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
