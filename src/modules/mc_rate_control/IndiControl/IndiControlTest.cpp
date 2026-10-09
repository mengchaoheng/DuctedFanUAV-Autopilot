/**
 * Copyright 2026 Chaoheng Meng
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include "IndiControl.hpp"

using matrix::Vector3f;

TEST(IndiControl, DefaultPreservesOriginalLaw)
{
	IndiControl control;
	const Vector3f gain{10.f, 12.f, 8.f};
	const Vector3f inertia{.01149f, .01153f, .00487f};
	const Vector3f rate{.2f, -.1f, .3f};
	const Vector3f target{.5f, .2f, -.1f};
	const Vector3f acceleration{1.f, -2.f, 3.f};
	const Vector3f allocated{.1f, -.2f, .05f};
	control.setParams(gain, inertia);
	ASSERT_TRUE(control.paramsValid());
	const auto output = control.update(rate, target, acceleration, allocated);
	const Vector3f expected_low = inertia.emult(gain.emult(target - rate));
	const Vector3f expected_high = allocated - inertia.emult(acceleration);

	for (int axis = 0; axis < 3; ++axis) {
		EXPECT_FLOAT_EQ(output.rate_error_torque(axis), expected_low(axis));
		EXPECT_FLOAT_EQ(output.feedback_torque(axis), expected_high(axis));
	}
}

TEST(IndiControl, VaneInverseTracksRotorWashAndAxialMotion)
{
	const Vector3f inertia{.01149f, .01153f, .00487f};
	for (float axial_velocity : {-10.f, 0.f, 10.f, 40.f}) {
		IndiControl control;
		control.setParams(Vector3f{10.f, 10.f, 10.f}, inertia);
		control.setVaneParams(1.f, 20.f, .03f, 0.f);
		ASSERT_TRUE(control.paramsValid());
		const float flow = (-0.5f * axial_velocity + hypotf(0.5f * axial_velocity, 30.f)) / 20.f;
		const float actual_gain = flow * flow;
		Vector3f allocated{};

		for (int step = 0; step < 40; ++step) {
			const Vector3f measured = allocated.edivide(inertia) * actual_gain;
			const auto output = control.update(Vector3f{}, Vector3f{1.f, 1.f, 1.f}, measured,
							 allocated, 2.25f, axial_velocity, .004f);
			allocated = output.rate_error_torque + output.feedback_torque;
		}

		const Vector3f achieved = allocated.edivide(inertia) * actual_gain;

		for (int axis = 0; axis < 3; ++axis) {
			EXPECT_NEAR(achieved(axis), 10.f, 1e-4f) << axial_velocity;
		}
	}
}

TEST(IndiControl, ChangingVaneGainUsesPhysicalFilteredExpansionPoint)
{
	IndiControl control;
	const Vector3f inertia{.01f, .01f, .01f};
	control.setParams(Vector3f{10.f, 10.f, 10.f}, inertia);
	control.setVaneParams(1.f, 20.f, 0.f, 10.f);
	// Previously applied physical torque and measured acceleration are steady.
	// A fourfold gain change requires one-quarter nominal torque immediately,
	// even when the requested acceleration equals the measured acceleration.
	const Vector3f target{1.f, 1.f, 1.f};
	const Vector3f alpha{10.f, 10.f, 10.f};
	control.update(Vector3f{}, target, alpha, Vector3f{.1f, .1f, .1f}, 1.f, 0.f, .004f);
	const auto output = control.update(Vector3f{}, target, alpha, Vector3f{.025f, .025f, .025f},
					 4.f, 0.f, .004f);
	for (int axis = 0; axis < 3; ++axis) {
		EXPECT_NEAR(output.rate_error_torque(axis) + output.feedback_torque(axis), .025f, 1e-6f);
	}
}

TEST(IndiControl, DisablingVaneModelRestoresFixedEffectivenessAfterUse)
{
	IndiControl control;
	const Vector3f inertia{.01f, .012f, .005f};
	const Vector3f gain{10.f, 10.f, 10.f};
	control.setParams(gain, inertia);
	control.setVaneParams(1.f, 20.f, .03f, 10.f);
	control.update(Vector3f{}, Vector3f{1.f, 1.f, 1.f}, Vector3f{}, Vector3f{.1f, .1f, .1f},
		       4.f, -10.f, .004f);
	// MC_INDI_V_EN=0 passes zero active wash, retaining saved calibration.
	control.setVaneParams(0.f, 0.f, .03f, 10.f);
	ASSERT_TRUE(control.paramsValid());
	const Vector3f alpha{1.f, -2.f, 3.f};
	const Vector3f allocated{.1f, -.2f, .03f};
	const auto output = control.update(Vector3f{}, Vector3f{1.f, 1.f, 1.f}, alpha, allocated,
					 4.f, -10.f, .004f);
	const Vector3f expected = allocated + inertia.emult(gain - alpha);
	for (int axis = 0; axis < 3; ++axis) {
		EXPECT_NEAR(output.rate_error_torque(axis) + output.feedback_torque(axis), expected(axis), 1e-6f);
	}
}
