/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskOrbit.cpp
 */

#include "FlightTaskOrbit.hpp"

#include <lib/systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/events.h>
#include <lib/geo/geo.h>

using namespace matrix;

FlightTaskOrbit::FlightTaskOrbit()
{
	_sticks_data_required = false;
}

bool FlightTaskOrbit::applyCommandParameters(const vehicle_command_s &command, bool &success)
{
	if (command.command != vehicle_command_s::VEHICLE_CMD_DO_ORBIT) {
		return false;
	}

	success = true;

	if (_use_parameterized_geometry) {
		return true;
	}

	// save previous velocity and rotation direction
	bool new_is_clockwise = _orbit_velocity > 0;
	float new_radius = _orbit_radius;
	float new_absolute_velocity = fabsf(_orbit_velocity);

	if (PX4_ISFINITE(command.param1)) {
		new_is_clockwise = command.param1 > 0.f;
		new_radius = fabsf(command.param1);
	}

	// commanded velocity, take sign of radius as rotation direction
	if (PX4_ISFINITE(command.param2)) {
		new_absolute_velocity = command.param2;
	}

	float new_velocity = signFromBool(new_is_clockwise) * new_absolute_velocity;

	if (math::isInRange(new_radius, _radius_min, _param_mc_orbit_rad_max.get())) {
		_started_clockwise = new_is_clockwise;
		_sanitizeParams(new_radius, new_velocity);
		_orbit_radius = new_radius;
		_orbit_velocity = new_velocity;
		_radii = {new_radius, new_radius};
		_vertical_amplitude = 0.f;
		_use_angular_frequency = false;

	} else {
		mavlink_log_critical(&_mavlink_log_pub, "Orbit radius limit exceeded\t");
		events::send(events::ID("orbit_radius_exceeded"), events::Log::Alert, "Orbit radius limit exceeded");
		success = false;
	}

	// commanded heading behaviour
	if (PX4_ISFINITE(command.param3)) {
		if (static_cast<uint8_t>(lround(command.param3)) == vehicle_command_s::ORBIT_YAW_BEHAVIOUR_UNCHANGED) {
			if (!_currently_orbiting) {	// only change the yaw behaviour if we are not actively orbiting
				_yaw_behaviour = _param_mc_orbit_yaw_mod.get();
			}

		} else {
			_yaw_behaviour = command.param3;
		}
	}

	// save current yaw estimate for ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING
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
		_setOrbitPhaseFromPosition();
	}

	// perpendicularly approach the orbit circle again when new parameters get commanded
	if (!_is_position_on_circle() && !_in_circle_approach) {
		_in_circle_approach = true;
		_position_smoothing.reset(_acceleration_setpoint, _velocity_setpoint, _position);
	}

	return true;
}

bool FlightTaskOrbit::sendTelemetry()
{
	orbit_status_s orbit_status{};
	orbit_status.radius = math::signNoZero(_orbit_velocity) * _orbit_radius;
	orbit_status.frame = 0; // MAV_FRAME::MAV_FRAME_GLOBAL
	orbit_status.yaw_behaviour = _yaw_behaviour;

	if (_geo_projection.isInitialized()) {
		// While chainging altitude by stick _position_setpoint(2) is not set (NAN)
		float local_altitude = PX4_ISFINITE(_position_setpoint(2)) ? _position_setpoint(2) : _position(2);
		// local -> global
		_geo_projection.reproject(_center(0), _center(1), orbit_status.x, orbit_status.y);
		orbit_status.z = _global_local_alt0 - local_altitude;

	} else {
		return false; // don't send the message if the transformation failed
	}

	orbit_status.timestamp = hrt_absolute_time();
	_orbit_status_pub.publish(orbit_status);

	return true;
}

void FlightTaskOrbit::_sanitizeParams(float &radius, float &velocity) const
{
	// clip the radius to be within range
	radius = math::constrain(radius, _radius_min, _param_mc_orbit_rad_max.get());
	velocity = math::constrain(velocity, -fabsf(_param_mpc_xy_vel_max.get()), fabsf(_param_mpc_xy_vel_max.get()));

	bool exceeds_maximum_acceleration = (velocity * velocity) >= _trajectory_acceleration_max * radius;

	// value combination is not valid. Reduce velocity instead of
	// radius, as small radius + low velocity is better for safety
	if (exceeds_maximum_acceleration) {
		velocity = sign(velocity) * sqrtf(_trajectory_acceleration_max * radius);
	}
}

bool FlightTaskOrbit::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTaskManualAltitudeSmoothVel::activate(last_setpoint);
	_currently_orbiting = false;
	_use_parameterized_geometry = _param_mc_orbit_source.get() == 1;
	_trajectory_acceleration_max = _use_parameterized_geometry
			? math::max(_param_mc_circle_acceleration.get(), 0.f)
			: _default_acceleration_max;

	_radii = {_param_mc_circle_x_amplitude.get(), _param_mc_circle_y_amplitude.get()};
	_radii(0) = math::constrain(fabsf(_radii(0)), 0.f, _param_mc_orbit_rad_max.get());
	_radii(1) = math::constrain(fabsf(_radii(1)), 0.f, _param_mc_orbit_rad_max.get());
	_vertical_amplitude = math::constrain(fabsf(_param_mc_circle_z_amplitude.get()), 0.f,
						_param_mc_orbit_rad_max.get());
	_orbit_radius = math::max(_radii(0), _radii(1));
	_orbit_radius = _use_parameterized_geometry ? _orbit_radius : _radius_min;
	_use_angular_frequency = _use_parameterized_geometry;
	_trajectory_omega = _param_mc_circle_omega.get();
	if (_use_parameterized_geometry && _trajectory_acceleration_max > FLT_EPSILON) {
		const float second_derivative_bound = sqrtf(_radii(0) * _radii(0) + _radii(1) * _radii(1)
									 + _vertical_amplitude * _vertical_amplitude);
		if (second_derivative_bound > FLT_EPSILON) {
			const float omega_limit = sqrtf(_trajectory_acceleration_max / second_derivative_bound);
			_trajectory_omega = math::constrain(_trajectory_omega, -omega_limit, omega_limit);
		}
	}
	_hold_parameterized_origin = _use_parameterized_geometry
			&& (fabsf(_trajectory_omega) <= FLT_EPSILON || _orbit_radius <= FLT_EPSILON);
	// Command mode keeps the original MAVLink radius/speed interface. It does
	// not use the parameterized axes until a command supplies its geometry.
	_orbit_velocity = _use_parameterized_geometry ? _trajectory_omega * _orbit_radius : 0.f;
	if (!_use_parameterized_geometry) {
		_radii = {_radius_min, _radius_min};
		_vertical_amplitude = 0.f;
	}
	if (!_use_parameterized_geometry && _orbit_radius > FLT_EPSILON) {
		_sanitizeParams(_orbit_radius, _orbit_velocity);
	}
	_started_clockwise = _orbit_velocity >= 0.f;
	_center = _position;
	_orbit_phase = 0.f;

	if (_use_parameterized_geometry) {
		// Parameter and RC entry do not carry MAV_CMD_DO_ORBIT param3. Use the
		// existing Orbit default without changing command-driven yaw semantics.
		_yaw_behaviour = _param_mc_orbit_yaw_mod.get();

	} else {
		// Keep the placeholder circle reference at the current position until
		// MAV_CMD_DO_ORBIT supplies the geometry.
		_center(0) -= _orbit_radius;
	}

	_initial_heading = _yaw;
	if (_use_parameterized_geometry && !_hold_parameterized_origin) {
		_setOrbitPhaseFromPosition();
	}
	_heading_smoothing.reset(PX4_ISFINITE(last_setpoint.yaw) ? last_setpoint.yaw : _yaw,
				 PX4_ISFINITE(last_setpoint.yawspeed) ? last_setpoint.yawspeed : 0.f);
	_slew_rate_velocity.setSlewRate(_param_mpc_acc_hor.get());

	// need a valid position and velocity
	ret = ret && _position.isAllFinite() && _velocity.isAllFinite();

	Vector3f pos_prev{last_setpoint.position};
	Vector3f vel_prev{last_setpoint.velocity};
	Vector3f accel_prev{last_setpoint.acceleration};

	for (int i = 0; i < 3; i++) {
		// If the position setpoint is unknown, set to the current position
		if (!PX4_ISFINITE(pos_prev(i))) { pos_prev(i) = _position(i); }

		// If the velocity setpoint is unknown, set to the current velocity
		if (!PX4_ISFINITE(vel_prev(i))) { vel_prev(i) = _velocity(i); }

		// If accel setpoint unknown, set to the current accel
		if (!PX4_ISFINITE(accel_prev(i))) { accel_prev(i) = _acceleration(i); }
	}

	_position_smoothing.reset(accel_prev, vel_prev, pos_prev);
	_in_circle_approach = !_hold_parameterized_origin;

	return ret;
}

bool FlightTaskOrbit::update()
{
	bool ret = true;
	_currently_orbiting = true;
	_updateTrajectoryBoundaries();
	_adjustParametersByStick();

	if (_hold_parameterized_origin) {
		// A zero configured phase rate is an explicit stationary trajectory. Do
		// not enter the legacy altitude/velocity mode or advance an old phase.
		ret = ret && FlightTaskManualAltitudeSmoothVel::update();
		_position_setpoint = _center;
		_velocity_setpoint.setAll(0.f);
		_acceleration_setpoint.setAll(0.f);
		_jerk_setpoint.setAll(0.f);
		_yaw_setpoint = _initial_heading;
		_yawspeed_setpoint = NAN;
		_heading_smoothing.setMaxHeadingRate(math::radians(_param_mpc_yawrauto_max.get()));
		_heading_smoothing.setMaxHeadingAccel(math::radians(_param_mpc_yawrauto_acc.get()));
		_heading_smoothing.update(_yaw_setpoint, _deltatime);
		_yaw_setpoint = _heading_smoothing.getSmoothedHeading();
		_yawspeed_setpoint = _heading_smoothing.getSmoothedHeadingRate();
		sendTelemetry();
		return ret;
	}

	if (_is_position_on_circle()) {
		if (_in_circle_approach) {
			_in_circle_approach = false;
			_setOrbitPhaseFromPosition();
			_slew_rate_velocity.setForcedValue(0.f); // reset the slew rate when moving between orbits.
			FlightTaskManualAltitudeSmoothVel::_smoothing.reset(
				PX4_ISFINITE(_acceleration_setpoint(2)) ? _acceleration_setpoint(2) : _acceleration(2),
				PX4_ISFINITE(_velocity_setpoint(2)) ? _velocity_setpoint(2) : _velocity(2),
				PX4_ISFINITE(_position_setpoint(2)) ? _position_setpoint(2) : _position(2));
		}
	}

	if (_in_circle_approach) {
		_generate_circle_approach_setpoints();

	} else {
		// update altitude
		ret = ret && FlightTaskManualAltitudeSmoothVel::update();

		// this generates x, y and yaw setpoints
		_generate_circle_setpoints();
		_generate_circle_yaw_setpoints();
	}

	// Apply yaw smoothing
	_heading_smoothing.setMaxHeadingRate(math::radians(_param_mpc_yawrauto_max.get()));
	_heading_smoothing.setMaxHeadingAccel(math::radians(_param_mpc_yawrauto_acc.get()));
	_heading_smoothing.update(_yaw_setpoint, _deltatime);
	_yaw_setpoint = _heading_smoothing.getSmoothedHeading();

	if (_in_circle_approach) { // don't override feed-forward which is already calculated for circling
		_yawspeed_setpoint = _heading_smoothing.getSmoothedHeadingRate();
	}

	// publish information to UI
	sendTelemetry();

	return ret;
}

void FlightTaskOrbit::_updateTrajectoryBoundaries()
{
	// update params of the position smoothing
	_position_smoothing.setMaxAllowedHorizontalError(_param_mpc_xy_err_max.get());
	_position_smoothing.setVerticalAcceptanceRadius(_param_nav_mc_alt_rad.get());
	_position_smoothing.setCruiseSpeed(_param_mpc_xy_cruise.get());
	_position_smoothing.setHorizontalTrajectoryGain(_param_mpc_xy_traj_p.get());
	_position_smoothing.setTargetAcceptanceRadius(_horizontal_acceptance_radius);

	// Update the constraints of the trajectories
	_position_smoothing.setMaxAccelerationXY(_param_mpc_acc_hor.get()); // TODO : Should be computed using heading
	_position_smoothing.setMaxVelocityXY(_param_mpc_xy_vel_max.get());
	_position_smoothing.setMaxJerk(_param_mpc_jerk_auto.get()); // TODO : Should be computed using heading

	if (_velocity_setpoint(2) < 0.f) { // up
		_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_up.get());
		_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_up_max.get());

	} else { // down
		_position_smoothing.setMaxAccelerationZ(_param_mpc_acc_down_max.get());
		_position_smoothing.setMaxVelocityZ(_param_mpc_z_v_auto_dn.get());
	}

}

bool FlightTaskOrbit::_is_position_on_circle() const
{
	if (_use_parameterized_geometry) {
		const motion_planning::HarmonicTrajectory3D curve{
			{_radii(0), _radii(1), _vertical_amplitude},
			{1.f, 1.f, 1.f},
			{M_PI_2_F, 0.f, 0.f}};
		return (_center + curve.evaluate(_orbit_phase, 0) - _position).norm() < _horizontal_acceptance_radius;
	}

	return (fabsf(Vector2f(_position - _center).length() - _orbit_radius) < _horizontal_acceptance_radius)
	       && fabsf(_position(2) - _center(2)) < _param_nav_mc_alt_rad.get();

}

void FlightTaskOrbit::_adjustParametersByStick()
{
	// Parameter/RC-triggered Orbit uses latched geometry and speed. Only a
	// MAVLink DO_ORBIT command is allowed to use the stick-based adjustments.
	if (_use_parameterized_geometry) {
		return;
	}

	float radius = _orbit_radius;
	float velocity = _orbit_velocity;

	switch (_yaw_behaviour) {
	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE:
		radius -= signFromBool(_started_clockwise) * _sticks.getRollExpo() * _deltatime * _param_mpc_xy_cruise.get();
		velocity += signFromBool(_started_clockwise) * _sticks.getPitchExpo() * _deltatime * _param_mpc_acc_hor.get();
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING:
	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_UNCONTROLLED:
	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED:
	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER:
	default:
		// stick input adjusts parameters within a fixed time frame
		radius -= _sticks.getPitchExpo() * _deltatime * _param_mpc_xy_cruise.get();
		velocity += _sticks.getRollExpo() * _deltatime * _param_mpc_acc_hor.get();
		break;
	}

	_sanitizeParams(radius, velocity);
	_orbit_radius = radius;
	_orbit_velocity = velocity;
}

void FlightTaskOrbit::_generate_circle_approach_setpoints()
{
	if (_use_parameterized_geometry) {
		const motion_planning::HarmonicTrajectory3D curve{
			{_radii(0), _radii(1), _vertical_amplitude},
			{1.f, 1.f, 1.f},
			{M_PI_2_F, 0.f, 0.f}};
		const Vector3f target = _center + curve.evaluate(_orbit_phase, 0);
		PositionSmoothing::PositionSmoothingSetpoints out_setpoints;
		_position_smoothing.generateSetpoints(_position, target, Vector3f{}, _deltatime, false, out_setpoints);
		_position_setpoint = out_setpoints.position;
		_velocity_setpoint = out_setpoints.velocity;
		_acceleration_setpoint = out_setpoints.acceleration;
		_jerk_setpoint = out_setpoints.jerk;
		_yaw_setpoint = atan2f(target(1) - _position(1), target(0) - _position(0));
		return;
	}

	const Vector2f center2d = Vector2f(_center);
	const Vector2f position_to_center_xy = center2d - Vector2f(_position);
	const Vector2f center_to_position_xy = -position_to_center_xy;
	const Vector2f radial_direction = center_to_position_xy.norm() > FLT_EPSILON
					  ? center_to_position_xy.unit_or_zero()
					  : Vector2f{cosf(_orbit_phase), sinf(_orbit_phase)};
	const Vector2f closest_point_on_circle = center2d + radial_direction * _orbit_radius;

	const Vector3f target_circle_point{closest_point_on_circle(0), closest_point_on_circle(1), _center(2)};

	PositionSmoothing::PositionSmoothingSetpoints out_setpoints;
	_position_smoothing.generateSetpoints(_position, target_circle_point,
	{0.f, 0.f, 0.f}, _deltatime, false, out_setpoints);

	_yaw_setpoint = atan2f(position_to_center_xy(1), position_to_center_xy(0));

	_position_setpoint = out_setpoints.position;
	_velocity_setpoint = out_setpoints.velocity;
	_acceleration_setpoint = out_setpoints.acceleration;
	_jerk_setpoint = out_setpoints.jerk;
}

void FlightTaskOrbit::_generate_circle_setpoints()
{
	/*
	 * The ideal horizontal circle is parameterized by
	 *
	 *   p(phi) = center + R [cos(phi), sin(phi)]^T
	 *   v(phi) = v_t [-sin(phi), cos(phi)]^T
	 *   a(phi) = -(v_t^2 / R) [cos(phi), sin(phi)]^T
	 *            + a_t [-sin(phi), cos(phi)]^T
	 *   phi_dot = v_t / R
	 *
	 * A finite position setpoint enables the multicopter position outer loop.
	 * Velocity and acceleration remain the exact kinematic feed-forward terms;
	 * path-error feedback is therefore not duplicated in this flight task.
	 */
	const motion_planning::HarmonicTrajectory3D circle{
		{_radii(0), _radii(1), _vertical_amplitude},
		{1.f, 1.f, 1.f},
		{M_PI_2_F, 0.f, 0.f}};
	const Vector3f position_offset = circle.evaluate(_orbit_phase, 0);
	const Vector3f phase_derivative = circle.evaluate(_orbit_phase, 1);
	const Vector3f phase_second_derivative = circle.evaluate(_orbit_phase, 2);

	// Slew rate reduces tangential jerk when entering an orbit or changing speed.
	const float previous_velocity = _slew_rate_velocity.getState();
	const float tangential_velocity = _use_angular_frequency
			? _orbit_velocity
			: _slew_rate_velocity.update(_orbit_velocity, _deltatime);
	const float tangential_acceleration = _deltatime > FLT_EPSILON
					      ? (tangential_velocity - previous_velocity) / _deltatime : 0.f;

	float phase_rate = _use_angular_frequency ? _trajectory_omega
							: tangential_velocity / math::max(_orbit_radius, FLT_EPSILON);
	const float phase_acceleration = _use_angular_frequency ? 0.f
							 : tangential_acceleration / math::max(_orbit_radius, FLT_EPSILON);
	_position_setpoint = _center + position_offset;
	_velocity_setpoint = phase_derivative * phase_rate;
	_acceleration_setpoint = phase_second_derivative * phase_rate * phase_rate + phase_derivative * phase_acceleration;
	_orbit_phase = wrap_2pi(_orbit_phase + phase_rate * _deltatime);
}

void FlightTaskOrbit::_setOrbitPhaseFromPosition()
{
	if (_use_parameterized_geometry && (_radii(0) > FLT_EPSILON || _radii(1) > FLT_EPSILON)) {
		const Vector2f delta = (_position - _center).xy();
		_orbit_phase = wrap_2pi(atan2f(delta(1) / math::max(_radii(1), FLT_EPSILON),
						 delta(0) / math::max(_radii(0), FLT_EPSILON)));
		return;
	}

	const Vector2f center_to_position = (_position - _center).xy();

	if (center_to_position.norm() > FLT_EPSILON) {
		_orbit_phase = wrap_2pi(atan2f(center_to_position(1), center_to_position(0)));
	}
}

void FlightTaskOrbit::_ekfResetHandlerPositionXY(const Vector2f &delta_xy)
{
	_center.xy() += delta_xy;
	Vector3f smoothed_position = _position_smoothing.getCurrentPosition();
	smoothed_position.xy() += delta_xy;
	_position_smoothing.forceSetPosition(smoothed_position);
}

void FlightTaskOrbit::_ekfResetHandlerPositionZ(float delta_z)
{
	FlightTaskManualAltitudeSmoothVel::_ekfResetHandlerPositionZ(delta_z);
	_center(2) += delta_z;
	Vector3f smoothed_position = _position_smoothing.getCurrentPosition();
	smoothed_position(2) += delta_z;
	_position_smoothing.forceSetPosition(smoothed_position);
}

void FlightTaskOrbit::_generate_circle_yaw_setpoints()
{
	Vector3f center_to_position = _position - _center;

	switch (_yaw_behaviour) {
	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING:
		// make vehicle keep the same heading as when the orbit was commanded
		_yaw_setpoint = _initial_heading;
		_yawspeed_setpoint = NAN;
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_UNCONTROLLED:
		// no yaw setpoint
		_yaw_setpoint = NAN;
		_yawspeed_setpoint = 0.f; // No yaw setpoint is invalid -> just brake to 0°/s
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE:
		_yaw_setpoint = atan2f(signFromBool(_started_clockwise) * center_to_position(0),
				       -signFromBool(_started_clockwise) * center_to_position(1));
		_yawspeed_setpoint = _orbit_velocity / _orbit_radius;
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED:
		// inherit setpoint from altitude flight task
		break;

	case orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER:
	default:
		_yaw_setpoint = atan2f(-center_to_position(1), -center_to_position(0));
		// yawspeed feed-forward because we know the necessary angular rate
		_yawspeed_setpoint = _orbit_velocity / _orbit_radius;
		break;
	}
}
