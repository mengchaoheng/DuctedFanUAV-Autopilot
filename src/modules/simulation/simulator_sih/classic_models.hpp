/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

// Body FRD throughout. Plant geometry is independent of allocator tuning.
// Equations follow this repository's Classic LiftDragPlugin/DuctedFanPlugin.
namespace sih_classic
{
using matrix::Vector3f;
struct Wrench { Vector3f force{}; Vector3f moment{}; };
}
#include "classic_model_data.hpp"
namespace sih_classic
{
inline float lag(float state, float command, float dt, float tau)
{
	return tau > 0.f ? command + (state - command) * expf(-dt / tau) : command;
}

inline float coefficient(float alpha, float slope, float stall, float post_stall, bool signed_lift)
{
	if (alpha > stall) {
		const float value = slope * stall + post_stall * (alpha - stall);
		return signed_lift ? math::max(0.f, value) : value;
	}

	if (alpha < -stall) {
		const float value = -slope * stall + post_stall * (alpha + stall);
		return signed_lift ? math::min(0.f, value) : value;
	}

	return slope * alpha;
}

template<size_t N>
inline float spline(const float (&segments)[N][6], float angle)
{
	const float x = math::constrain(angle, segments[0][0], segments[N - 1][1]);
	const float *row = segments[N - 1];

	for (const auto &segment : segments) {
		if (x >= segment[0] && x <= segment[1]) { row = segment; break; }
	}

	const float dx = x - row[0];
	return ((row[2] * dx + row[3]) * dx + row[4]) * dx + row[5];
}

inline Wrench shc09(const Vector3f &velocity, const Vector3f &rates, const float *commands)
{
	// SHC09.sdf DuctedFanPlugin coefficients; CSV data above are generated.
	const float u = velocity(0), v = velocity(1), w = velocity(2);
	const float speed = velocity.norm(), vxy = sqrtf(u * u + v * v), vxz = sqrtf(u * u + w * w);
	const float x_dir = vxy > 1e-12f ? u / vxy : 1.f;
	const float y_dir = vxy > 1e-12f ? v / vxy : 0.f;
	const float cos_aoa = speed > 1e-6f ? math::constrain(-w / speed, -1.f, 1.f) : 0.f;
	const float sin_aoa = speed > 1e-6f ? vxy / speed : 1.f;
	const float duct_aoa = speed > 1e-6f ? acosf(cos_aoa) : M_PI_2_F;
	const float wing_aoa = vxz > 1.f ? acosf(math::constrain(-w / vxz, -1.f, 1.f)) : M_PI_2_F;
	const float omega = 1200.f * math::constrain(commands[0], 0.f, 1.f);
	const float thrust = fabsf(2.937612e-5f * omega * omega
				   + speed * omega * (-3.7614e-4f - 2.6733e-4f * cos_aoa)
				   + speed * speed * spline(duct_spline_dt, duct_aoa));
	const float side = speed * omega * 5.0699e-4f * sin_aoa + speed * speed * spline(duct_spline_dn, duct_aoa);
	float pitch = -0.15f * side;

	if (omega > 1e-6f) { pitch += thrust * 2.6619f * speed / omega * sin_aoa; }

	const float exit_speed = -0.5f * w + sqrtf(0.25f * w * w + thrust / (0.7f * 1.225f * 0.0408f));
	const float exit_squared = exit_speed * exit_speed;
	const float p_roll[6] = {-1.f, -0.5f, 0.5f, 1.f, 0.5f, -0.5f};
	const float p_pitch[6] = {0.f, 0.8660254f, 0.8660254f, 0.f, -0.8660254f, -0.8660254f};
	Vector3f control;

	for (int i = 0; i < 6; ++i) {
		const float force = 0.0032f * exit_squared * 0.6981f * math::constrain(commands[i + 1], -1.f, 1.f);
		control += force * Vector3f(0.267f * p_roll[i], 0.267f * p_pitch[i], 0.066f);
	}

	Wrench result;
	result.force = Vector3f(-side * x_dir - vxz * vxz * spline(wing_spline_wl, wing_aoa),
				-side * y_dir, -thrust - vxz * vxz * spline(wing_spline_wd, wing_aoa));
	result.moment = control + Vector3f(-pitch * y_dir - 3.7e-5f * omega * rates(1),
					   pitch * x_dir + vxz * vxz * spline(wing_spline_wm, wing_aoa)
					   - rates(1) * (0.0198f + 0.0015f * vxz) + 3.7e-5f * omega * rates(0),
					   -5.7792e-7f * omega * omega + 5.3688e-4f * exit_squared);
	return result;
}
} // namespace sih_classic
