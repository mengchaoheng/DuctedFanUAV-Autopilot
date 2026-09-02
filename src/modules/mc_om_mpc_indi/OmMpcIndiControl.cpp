/**
 * Copyright 2026 Chaoheng Meng
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "OmMpcIndiControl.hpp"

#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

void OmMpcIndiControl::setParams(const Vector3f &attitude_gain, float hover_thrust, float gravity)
{
	_attitude_gain = attitude_gain;
	_hover_thrust = hover_thrust;
	_gravity = gravity;
}

bool OmMpcIndiControl::paramsValid() const
{
	return _attitude_gain.isAllFinite()
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
			+ (acceleration_target - acceleration_ned) * (_hover_thrust / _gravity);
	float corrected_thrust = corrected_thrust_ned.norm();
	const float nominal_thrust = nominal_thrust_body.norm();

	if (!PX4_ISFINITE(corrected_thrust)) {
		return false;
	}

	// Match main.m at zero corrected thrust: keep the nominal current-attitude
	// direction with only a numerical epsilon. Do not insert gain scheduling or
	// an amplitude limiter into the middle-layer control law.
	if (corrected_thrust < 1e-6f) {
		corrected_thrust = math::max(nominal_thrust, 1e-6f);
		corrected_thrust_ned = Vector3f(R_to_ned.col(2)) * -corrected_thrust;
	}

	const Vector3f b3 = R_to_ned.col(2);
	const Vector3f b3_command = -corrected_thrust_ned / corrected_thrust;
	const Vector3f cross_axis = b3.cross(b3_command);
	const float sine = cross_axis.norm();
	const float cosine = math::constrain(b3.dot(b3_command), -1.f, 1.f);
	Vector3f attitude_error;

	if (sine < 1e-6f) {
		// Match main.m's deterministic minimum-rotation branch. At the exact
		// antipode any transverse axis is valid; select current body x. The
		// sign ambiguity into a sample-to-sample axis jump.
		attitude_error = cosine >= 0.f ? Vector3f{} : Vector3f{M_PI_F, 0.f, 0.f};

	} else {
		const Vector3f rotation_vector_ned = cross_axis * (atan2f(sine, cosine) / sine);
		attitude_error = R_to_ned.transpose() * rotation_vector_ned;
	}

	output.attitude_error = attitude_error;
	output.rates_setpoint = nominal_rates + _attitude_gain.emult(output.attitude_error);
	output.thrust_ned = corrected_thrust_ned;

	output.thrust_body = Vector3f{0.f, 0.f, -math::constrain(corrected_thrust, 0.f, 1.f)};
	return output.rates_setpoint.isAllFinite() && output.thrust_body.isAllFinite();
}
