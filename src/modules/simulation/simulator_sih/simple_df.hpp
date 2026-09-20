/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once
#include "classic_models.hpp"

namespace sih_classic
{
// Equivalent whole-wing model in body FRD (forward-flight X = body -Z).
// One shared static lift/drag curve plus its aerodynamic rate derivatives.
// Derivatives are computed analytically from the same curves, not fitted knobs.
inline float curve_slope(float alpha, float slope, float stall, float post, bool lift)
{
	const float raw = coefficient(alpha, slope, stall, post, lift);
	const float derivative = fabsf(alpha) > stall ? post : slope;
	if (lift) { return fabsf(alpha) > stall && fabsf(raw) < 1e-8f ? 0.f : derivative; }
	return raw > 0.f ? derivative : raw < 0.f ? -derivative : 0.f;
}

inline Wrench whole_wing(const Vector3f &velocity, const Vector3f &rates,
			float left_command, float right_command, float lift_scale, float drag_scale, float control_scale)
{
	Wrench result;
	const Vector3f flow(velocity(0), 0.f, velocity(2));
	const float speed_sq = flow.norm_squared();
	if (flow(2) >= 0.f || speed_sq <= 0.0001f) { return result; }

	const float speed = sqrtf(speed_sq);
	const Vector3f tangent = flow / speed;
	const Vector3f span(0.f, 1.f, 0.f);
	const Vector3f lift_direction = span.cross(tangent);
	const Vector3f drag_direction = -tangent;
	float alpha = 0.05984281113f + atan2f(flow(0), -flow(2));
	while (fabsf(alpha) > M_PI_2_F) { alpha += alpha > 0.f ? -M_PI_F : M_PI_F; }

	const float left = 0.6981f * math::constrain(left_command, -1.f, 1.f);
	const float right = 0.6981f * math::constrain(right_command, -1.f, 1.f);
	const float cl_delta = -0.3f * control_scale;
	const float cl = lift_scale * coefficient(alpha, 2.5f, 0.6391428111f, -2.7f, true)
			 + cl_delta * 0.5f * (left + right);
	const float cd = drag_scale * fabsf(coefficient(alpha, 0.4f, 0.6391428111f, -0.85f, false));
	const float cl_alpha = lift_scale * curve_slope(alpha, 2.5f, 0.6391428111f, -2.7f, true);
	const float cd_alpha = drag_scale * curve_slope(alpha, 0.4f, 0.6391428111f, -0.85f, false);
	constexpr float pressure_area = 0.5f * 1.2041f * 0.23f; // rho*S/2, total wing area
	constexpr float lever = 0.3145f;
	const float q_area = pressure_area * speed_sq;
	const Vector3f force_coefficient = cl * lift_direction + cd * drag_direction;
	result.force = q_area * force_coefficient;

	// Differential elevon moment; the symmetric part already contributes to CL.
	result.moment = Vector3f(0.f, -lever, 0.f).cross(
			0.5f * q_area * cl_delta * (left - right) * lift_direction);

	// Equivalent aerodynamic rate moment: l^2 e_y x (dF/dv)(omega x e_y).
	// At alpha=0 this includes M_Bz ~= -rho*S_half*CL_alpha*l^2*V*r_B.
	// This is the standard roll-damping derivative, essential for forward flight.
	const Vector3f dv = rates.cross(span);
	const Vector3f dtangent = (dv - tangent * tangent.dot(dv)) / speed;
	const float dalpha = (-flow(2) * dv(0) + flow(0) * dv(2)) / speed_sq;
	const Vector3f derivative = 2.f * pressure_area * flow.dot(dv) * force_coefficient
			+ q_area * (dalpha * (cl_alpha * lift_direction + cd_alpha * drag_direction)
				    + cl * span.cross(dtangent) - cd * dtangent);
	result.moment += lever * lever * span.cross(derivative);
	return result;
}

// Paper-style shared vane model. Velocity is vehicle relative to air in FRD.
// The axial slipstream is added to that relative velocity (body -Z).
// No per-vane rotational flow, duct tables, or independently tuned surfaces.
inline Wrench simple_df(const Vector3f &velocity, const float *commands, bool wing,
			float thrust_max, float wash_max, float kv, float radius, float arm, float angle_max,
			float wing_lift = 1.f, float wing_drag = 1.f, float wing_control = 1.f,
			const Vector3f &rates = Vector3f())
{
	Wrench result;
	const float motor = math::constrain(commands[0], 0.f, 1.f);
	result.force(2) = -thrust_max * motor * motor;
	const Vector3f air = velocity + Vector3f(0.f, 0.f, -wash_max * motor);
	const int count = wing ? 6 : 4;

	for (int i = 0; i < count; ++i) {
		// Channel order is clockwise looking down the thrust axis.
		const float theta = (wing ? -2.f : 2.f) * M_PI_F * float(i) / float(count);
		const Vector3f radial(cosf(theta), sinf(theta), 0.f);
		const Vector3f cp = radius * radial + Vector3f(0.f, 0.f, arm);
		const Vector3f span = radial;
		const Vector3f in_plane = air - span * span.dot(air);
		// Preserve the Classic forward-flow envelope. Reverse flow has no vane authority.
		if (air(2) >= 0.f || in_plane.norm_squared() < 1e-8f) { continue; }

		const Vector3f lift = span.cross(in_plane).unit_or_zero();
		const float delta = angle_max * math::constrain(commands[i + 1], -1.f, 1.f);
		const Vector3f force = kv * in_plane.norm_squared() * delta * lift;
		result.force += force;
		result.moment += cp.cross(force);
	}

	if (wing) {
		const Wrench aero = whole_wing(velocity, rates, commands[7], commands[8], wing_lift, wing_drag, wing_control);
		result.force += aero.force;
		result.moment += aero.moment;
	}

	return result;
}
}
