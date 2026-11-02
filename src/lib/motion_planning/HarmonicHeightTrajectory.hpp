// Copyright 2026 PX4 Development Team. SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <matrix/matrix/math.hpp>

namespace motion_planning
{
// NED displacement about the entry altitude, encoded as bias + A sin(n theta + phase).
// The same coefficients can be published directly to the analytic MPC reference.
struct HarmonicHeightTrajectory {
	float amplitude{0.f};
	float harmonic{2.f};
	float phase{0.f};
	float bias{0.f};

	static HarmonicHeightTrajectory fromHeight(float height_amplitude, int function, int n)
	{
		HarmonicHeightTrajectory curve;
		curve.amplitude = -height_amplitude;
		curve.harmonic = static_cast<float>(n < 1 ? 1 : n > 10 ? 10 : n);

		if (function == 1) { // cos(n theta)
			curve.phase = M_PI_2_F;

		} else if (function == 2) { // 1 - cos(n theta)
			curve.amplitude = height_amplitude;
			curve.phase = M_PI_2_F;
			curve.bias = -height_amplitude;
		}

		return curve;
	}

	float evaluate(float theta, int derivative) const
	{
		const float argument = harmonic * theta + phase;

		switch (derivative) {
		case 0:
			return bias + amplitude * sinf(argument);

		case 1:
			return amplitude * harmonic * cosf(argument);

		case 2:
			return -amplitude * harmonic * harmonic * sinf(argument);

		default:
			return 0.f;
		}
	}
};
}
