// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "OmMpcModel.hpp"

namespace om_mpc
{
// Deterministic, bounded tracking errors around a moving figure eight.
// Synthetic state creation is outside the timed build/update/solve region.
inline bool bench_state(float time, Path &path, Vector3f &position, Vector3f &velocity, Dcmf &rotation)
{
	path = {};
	path.center = {0.f, 0.f, -3.f};
	path.amplitude = {4.5f, 4.5f, 1.f};
	path.harmonic = {1.f, 2.f, 1.f};
	path.omega = .3f;
	path.phase = path.omega * time;
	path.rotation = 1.57079632679f;
	Knot reference_state;

	if (!reference(path, 0.f, reference_state)) { return false; }

	position = reference_state.position + Vector3f{.15f * cosf(.4f * time), .12f * sinf(.5f * time), .05f * sinf(.3f * time)};
	velocity = reference_state.velocity + Vector3f{-.06f * sinf(.4f * time), .06f * cosf(.5f * time), .015f * cosf(.3f * time)};
	rotation = reference_state.rotation * Dcmf(matrix::Eulerf(.02f * sinf(.7f * time), -.03f * cosf(.6f * time), .01f * sinf(.5f * time)));
	return true;
}
}
