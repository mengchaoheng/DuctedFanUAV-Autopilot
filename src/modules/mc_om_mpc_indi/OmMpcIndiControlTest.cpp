/**
 * Copyright 2026 Chaoheng Meng
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>

#include "OmMpcIndiControl.hpp"
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

using namespace matrix;

namespace
{
constexpr float kGravity = 9.80665f;
constexpr float kHoverThrust = 0.216f;
constexpr float kTolerance = 1e-5f;

OmMpcIndiControl makeControl(const Vector3f &gain = Vector3f{1.5f, 1.5f, 0.f},
		float acceleration_gain = 1.f)
{
	OmMpcIndiControl control;
	control.setParams(gain, acceleration_gain, kHoverThrust, kGravity);
	return control;
}

void expectVectorNear(const Vector3f &actual, const Vector3f &expected, float tolerance = kTolerance)
{
	for (int axis = 0; axis < 3; ++axis) {
		EXPECT_NEAR(actual(axis), expected(axis), tolerance);
	}
}
} // namespace

TEST(OmMpcIndiControl, HoverIsAnExactFixedPoint)
{
	const auto control = makeControl();
	OmMpcIndiControl::Output output{};
	const Vector3f nominal_rates{0.2f, -0.1f, 0.05f};
	const Vector3f nominal_thrust_body{0.f, 0.f, -kHoverThrust};

	ASSERT_TRUE(control.update(Dcmf{}, nominal_rates, nominal_thrust_body, Vector3f{},
		Vector3f{}, nominal_thrust_body, output));
	expectVectorNear(output.rates_setpoint, nominal_rates);
	expectVectorNear(output.thrust_body, nominal_thrust_body);
	expectVectorNear(output.attitude_error, Vector3f{});
}

TEST(OmMpcIndiControl, MatchedDisturbanceDoesNotChangeNominalCommand)
{
	const auto control = makeControl();
	OmMpcIndiControl::Output output{};
	const Vector3f disturbance{1.2f, -0.7f, 0.4f};
	const Vector3f allocated{0.f, 0.f, -kHoverThrust};

	ASSERT_TRUE(control.update(Dcmf{}, Vector3f{}, allocated, disturbance,
		disturbance, allocated, output));
	expectVectorNear(output.thrust_ned, allocated);
	expectVectorNear(output.thrust_body, allocated);
	expectVectorNear(output.attitude_error, Vector3f{});
}

TEST(OmMpcIndiControl, TranslationalIncrementHasMainMUnitsAndSign)
{
	const auto control = makeControl(Vector3f{});
	OmMpcIndiControl::Output output{};
	const Vector3f hover{0.f, 0.f, -kHoverThrust};
	const Vector3f measured_acceleration{-1.f, 0.f, 0.f};
	const Vector3f expected_force{kHoverThrust / kGravity, 0.f, -kHoverThrust};

	ASSERT_TRUE(control.update(Dcmf{}, Vector3f{}, hover, Vector3f{},
		measured_acceleration, hover, output));
	expectVectorNear(output.thrust_ned, expected_force);
	EXPECT_NEAR(output.thrust_body(2), -expected_force.norm(), kTolerance);
}

TEST(OmMpcIndiControl, TranslationalIncrementGainScalesOnlyTheIncrement)
{
	const auto control = makeControl(Vector3f{}, 0.25f);
	OmMpcIndiControl::Output output{};
	const Vector3f hover{0.f, 0.f, -kHoverThrust};
	const Vector3f measured_acceleration{-1.f, 0.f, 0.f};
	const Vector3f expected_force{0.25f * kHoverThrust / kGravity, 0.f, -kHoverThrust};

	ASSERT_TRUE(control.update(Dcmf{}, Vector3f{}, hover, Vector3f{},
		measured_acceleration, hover, output));
	expectVectorNear(output.thrust_ned, expected_force);
}

TEST(OmMpcIndiControl, AntipodeDoesNotOverrideNominalRotation)
{
	const Vector3f gain{0.5f, 0.25f, 0.f};
	const auto control = makeControl(gain);
	OmMpcIndiControl::Output output{};
	const Vector3f nominal_thrust_body{0.f, 0.f, kHoverThrust};
	const Vector3f measured_acceleration{0.f, 0.f, 2.f * kGravity};
	const Vector3f allocated{0.f, 0.f, kHoverThrust};

	ASSERT_TRUE(control.update(Dcmf{}, Vector3f{}, nominal_thrust_body, Vector3f{},
		measured_acceleration, allocated, output));
	expectVectorNear(output.attitude_error, Vector3f{M_PI_F, 0.f, 0.f});
	expectVectorNear(output.rates_setpoint, Vector3f{});
}

TEST(OmMpcIndiControl, ZeroCorrectedThrustRemainsFinite)
{
	const auto control = makeControl();
	OmMpcIndiControl::Output output{};

	ASSERT_TRUE(control.update(Dcmf{}, Vector3f{}, Vector3f{}, Vector3f{},
		Vector3f{0.f, 0.f, kGravity}, Vector3f{}, output));
	EXPECT_TRUE(output.rates_setpoint.isAllFinite());
	EXPECT_TRUE(output.thrust_body.isAllFinite());
	EXPECT_LE(output.thrust_body.norm(), 1e-5f);
}

TEST(OmMpcIndiControl, MatchedInertialFiltersPreserveNominalDuringFastRotation)
{
	const auto control = makeControl();
	math::LowPassFilter2p<Vector3f> force_filter(250.f, 8.f);
	math::LowPassFilter2p<Vector3f> acceleration_filter(250.f, 8.f);
	const Vector3f gravity{0.f, 0.f, kGravity};
	const Vector3f thrust_body{0.f, 0.f, -0.45f};
	force_filter.reset(thrust_body);
	acceleration_filter.reset(gravity + thrust_body * (kGravity / kHoverThrust));
	float old_path_max_error = 0.f;
	for (int k = 0; k < 1000; ++k) {
		const Dcmf rotation(AxisAnglef(Vector3f{1.f, 0.f, 0.f}, 4.f * k / 250.f));
		const Vector3f force_ned = rotation * thrust_body;
		const Vector3f acceleration = acceleration_filter.apply(
			gravity + force_ned * (kGravity / kHoverThrust));
		const Vector3f force = force_filter.apply(force_ned);
		OmMpcIndiControl::Output output{};
		ASSERT_TRUE(control.update(rotation, Vector3f{4.f, 0.f, 0.f}, thrust_body,
			Vector3f{}, acceleration, force, output));
		expectVectorNear(output.thrust_ned, force_ned, 5e-5f);
		expectVectorNear(output.rates_setpoint, Vector3f{4.f, 0.f, 0.f}, 2e-4f);
		// The previous R*H[F_body] path fails this physical fixed point even
		// though thrust magnitude is constant and sensors have no noise.
		old_path_max_error = fmaxf(old_path_max_error,
			(acceleration - gravity - force_ned * (kGravity / kHoverThrust)).norm());
	}
	EXPECT_GT(old_path_max_error, 1.f);
}

TEST(OmMpcIndiControl, ZeroCorrectedThrustKeepsNominalRatesWithoutRestoringThrust)
{
	const auto control = makeControl();
	OmMpcIndiControl::Output output{};
	const Vector3f nominal{0.f, 0.f, -kHoverThrust};
	const Vector3f rates{-3.f, 0.f, 0.f};
	ASSERT_TRUE(control.update(Dcmf{}, rates, nominal, Vector3f{},
		Vector3f{0.f, 0.f, -kGravity}, nominal, output));
	expectVectorNear(output.rates_setpoint, rates);
	expectVectorNear(output.thrust_body, Vector3f{});
}

TEST(OmMpcIndiControl, SmallAngleRetainsGainAndAntipodeIsContinuous)
{
	const Vector3f gain{0.5f, 0.5f, 0.f};
	const auto control = makeControl(gain);
	const Vector3f nominal{0.f, 0.f, -kHoverThrust};
	const Vector3f rates{-3.f, 0.f, 0.f};
	for (float angle : {1e-3f, M_PI_F - 1e-3f, M_PI_F + 1e-3f}) {
		const Vector3f corrected{0.f, kHoverThrust * sinf(angle), -kHoverThrust * cosf(angle)};
		const Vector3f acceleration = (nominal - corrected) * (kGravity / kHoverThrust);
		OmMpcIndiControl::Output output{};
		ASSERT_TRUE(control.update(Dcmf{}, rates, nominal, Vector3f{}, acceleration, nominal, output));
		EXPECT_NEAR(output.rates_setpoint(0), rates(0) + gain(0) * sinf(angle), kTolerance);
		EXPECT_LT((output.rates_setpoint - rates).norm(), 1e-3f);
	}
}

TEST(OmMpcIndiControl, VanishingCorrectedForceHasVanishingDirectionCorrection)
{
	const auto control = makeControl();
	const Vector3f nominal{0.f, 0.f, -kHoverThrust};
	const Vector3f rates{-3.f, 0.f, 0.f};
	for (float magnitude : {1e-3f, 1e-4f, 1e-5f}) {
		const Vector3f corrected{magnitude, 0.f, 0.f};
		OmMpcIndiControl::Output output{};
		ASSERT_TRUE(control.update(Dcmf{}, rates, nominal, Vector3f{},
			(nominal - corrected) * (kGravity / kHoverThrust), nominal, output));
		EXPECT_LE((output.rates_setpoint - rates).norm(), 3.f * magnitude / kHoverThrust + kTolerance);
	}
}

TEST(OmMpcIndiControl, TiltCorrectionIsInvariantToWorldFrameRotation)
{
	const auto control = makeControl();
	const Vector3f nominal{0.f, 0.f, -kHoverThrust};
	const Vector3f rates{-3.f, 0.2f, 0.f};
	const Vector3f corrected{0.1f, 0.2f, -0.3f};
	const Dcmf rotated{Eulerf{0.4f, -0.6f, 0.9f}};
	OmMpcIndiControl::Output original{}, transformed{};
	ASSERT_TRUE(control.update(Dcmf{}, rates, nominal, Vector3f{},
		Vector3f{0.f, 0.f, kGravity} + (nominal - corrected) * (kGravity / kHoverThrust), nominal, original));
	ASSERT_TRUE(control.update(rotated, rates, nominal, Vector3f{},
		Vector3f{0.f, 0.f, kGravity} + rotated * (nominal - corrected) * (kGravity / kHoverThrust),
		rotated * nominal, transformed));
	expectVectorNear(original.rates_setpoint, transformed.rates_setpoint);
	expectVectorNear(original.thrust_body, transformed.thrust_body);
}
