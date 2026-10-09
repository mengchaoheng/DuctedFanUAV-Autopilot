/**
 * Copyright 2026 Chaoheng Meng
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "OmMpcIndiControl.hpp"

#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

void OmMpcIndiControl::setParams(const Vector3f &attitude_gain, float acceleration_gain,
		float hover_thrust, float gravity)
{
	_attitude_gain = attitude_gain;
	_acceleration_gain = acceleration_gain;
	_hover_thrust = hover_thrust;
	_gravity = gravity;
}

bool OmMpcIndiControl::paramsValid() const
{
	return _attitude_gain.isAllFinite()
	       && PX4_ISFINITE(_acceleration_gain) && _acceleration_gain >= 0.f && _acceleration_gain <= 1.f
	       && PX4_ISFINITE(_hover_thrust) && _hover_thrust > FLT_EPSILON
	       && PX4_ISFINITE(_gravity) && _gravity > FLT_EPSILON;
}

bool OmMpcIndiControl::update(const Dcmf &R_to_ned, const Vector3f &nominal_rates,
		const Vector3f &nominal_thrust_body, const Vector3f &mpc_disturbance_ned,
		const Vector3f &acceleration_ned, const Vector3f &allocated_thrust_ned,
		Output &output) const
{
	if (!paramsValid() || !R_to_ned.isAllFinite() || !nominal_rates.isAllFinite()
	    || !nominal_thrust_body.isAllFinite() || !mpc_disturbance_ned.isAllFinite()
	    || !acceleration_ned.isAllFinite() || !allocated_thrust_ned.isAllFinite()) {
		return false;
	}

	const Vector3f nominal_thrust_ned = R_to_ned * nominal_thrust_body;
	const Vector3f gravity_ned{0.f, 0.f, _gravity};
	const Vector3f acceleration_target = gravity_ned
			+ nominal_thrust_ned * (_gravity / _hover_thrust) + mpc_disturbance_ned;
	Vector3f corrected_thrust_ned = allocated_thrust_ned
			+ _acceleration_gain * (acceleration_target - acceleration_ned) * (_hover_thrust / _gravity);
	float corrected_thrust = corrected_thrust_ned.norm();
	const float nominal_thrust = nominal_thrust_body.norm();

	if (!PX4_ISFINITE(corrected_thrust)) {
		return false;
	}

	const Vector3f b3 = R_to_ned.col(2);
	Vector3f attitude_error{};

	// Keep the full angle for diagnostics only. Zero thrust has no direction.
	if (corrected_thrust > 1e-6f) {
		const Vector3f b3_command = -corrected_thrust_ned / corrected_thrust;
		const Vector3f cross_axis = b3.cross(b3_command);
		const float sine = cross_axis.norm();
		const float cosine = math::constrain(b3.dot(b3_command), -1.f, 1.f);
		if (sine < 1e-6f) {
			attitude_error = cosine >= 0.f ? Vector3f{} : Vector3f{M_PI_F, 0.f, 0.f};
		} else {
			attitude_error = R_to_ned.transpose() * cross_axis * (atan2f(sine, cosine) / sine);
		}
	}

	// Local thrust-direction correction: equal thrust magnitudes recover the
	// small-angle gain, while either vanishing vector removes its directional
	// authority. Unlike the principal Log, this error is continuous across the
	// antipode and cannot choose an arbitrary 180-degree recovery rotation.
	const float force_squared_sum = nominal_thrust * nominal_thrust
			+ corrected_thrust * corrected_thrust;
	Vector3f tilt_error{};
	if (force_squared_sum > 1e-12f) {
		tilt_error = R_to_ned.transpose() * nominal_thrust_ned.cross(corrected_thrust_ned)
				* (2.f / force_squared_sum);
	}

	output.attitude_error = attitude_error;
	output.rates_setpoint = nominal_rates + _attitude_gain.emult(tilt_error);
	output.thrust_ned = corrected_thrust_ned;

	output.thrust_body = Vector3f{0.f, 0.f, -math::constrain(corrected_thrust, 0.f, 1.f)};
	return output.rates_setpoint.isAllFinite() && output.thrust_body.isAllFinite();
}
