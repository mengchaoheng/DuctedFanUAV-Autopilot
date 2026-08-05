/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <matrix/matrix/math.hpp>

/**
 * Small, allocation-free generator for the planar harmonic trajectories used
 * by the Orbit flight tasks.
 *
 * Each coordinate is represented as
 *
 *   q_i(theta) = A_i sin(k_i theta + phi_i).
 *
 * Circle, ellipse/sine-line and figure-eight references are therefore the
 * same object with different amplitudes, harmonics and phases.  Keeping the
 * first two phase derivatives here avoids three task implementations slowly
 * drifting apart (and makes the velocity/acceleration feed-forward terms
 * analytically consistent with the position reference).
 */
namespace motion_planning
{
struct HarmonicTrajectory2D
{
	using Vector2f = matrix::Vector2f;

	Vector2f amplitude{};
	Vector2f harmonic{1.f, 1.f};
	Vector2f phase{};

	Vector2f evaluate(float theta, int derivative) const
	{
		Vector2f result;

		for (int i = 0; i < 2; ++i) {
			const float argument = harmonic(i) * theta + phase(i);
			const float k = harmonic(i);

			switch (derivative) {
			case 0:
				result(i) = amplitude(i) * sinf(argument);
				break;

			case 1:
				result(i) = amplitude(i) * k * cosf(argument);
				break;

			case 2:
				result(i) = -amplitude(i) * k * k * sinf(argument);
				break;

			default:
				result(i) = 0.f;
				break;
			}
		}

		return result;
	}

	static HarmonicTrajectory2D circle(float radius)
	{
		return {{radius, radius}, {1.f, 1.f}, {M_PI_2_F, 0.f}};
	}

	static HarmonicTrajectory2D ellipse(float x_radius, float y_radius)
	{
		return {{x_radius, y_radius}, {1.f, 1.f}, {M_PI_2_F, 0.f}};
	}

	static HarmonicTrajectory2D figureEight(float major_radius, float minor_radius)
	{
		return {{major_radius, minor_radius}, {1.f, 2.f}, {0.f, 0.f}};
	}
};

/**
 * Three-dimensional harmonic reference.  The same phase is shared by all
 * coordinates, while each coordinate has its own amplitude, harmonic and
 * phase.  This is intentionally small and header-only so flight tasks can
 * use the exact position/velocity/acceleration derivatives without a second
 * trajectory implementation.
 */
struct HarmonicTrajectory3D
{
	using Vector3f = matrix::Vector3f;

	Vector3f amplitude{};
	Vector3f harmonic{1.f, 1.f, 1.f};
	Vector3f phase{};

	Vector3f evaluate(float theta, int derivative) const
	{
		Vector3f result;

		for (int i = 0; i < 3; ++i) {
			const float argument = harmonic(i) * theta + phase(i);
			const float k = harmonic(i);

			switch (derivative) {
			case 0:
				result(i) = amplitude(i) * sinf(argument);
				break;

			case 1:
				result(i) = amplitude(i) * k * cosf(argument);
				break;

			case 2:
				result(i) = -amplitude(i) * k * k * sinf(argument);
				break;

			default:
				result(i) = 0.f;
				break;
			}
		}

		return result;
	}
};
}
