/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include <ControlAllocationWLS.hpp>

#include <cmath>

namespace
{
using ActuatorVector = ControlAllocation::ActuatorVector;
using ControlVector = matrix::Vector<float, ControlAllocation::NUM_AXES>;
using EffectivenessMatrix = matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS>;

void configure(ControlAllocationWLS &allocator, const EffectivenessMatrix &effectiveness, int actuators,
	       const ActuatorVector &trim = ActuatorVector())
{
	ActuatorVector minimum;
	ActuatorVector maximum;
	ActuatorVector linearization_point;
	minimum.setAll(-1.f);
	maximum.setAll(1.f);
	linearization_point.setZero();
	allocator.setActuatorMin(minimum);
	allocator.setActuatorMax(maximum);
	allocator.setEffectivenessMatrix(effectiveness, trim, linearization_point, actuators, false);
}
} // namespace

TEST(ControlAllocationWLSTest, UnconstrainedSolutionTracksNormalizedWrench)
{
	ControlAllocationWLS allocator;
	EffectivenessMatrix effectiveness;
	effectiveness.setZero();
	effectiveness(ControlAllocation::ROLL, 0) = 1.f;
	effectiveness(ControlAllocation::PITCH, 1) = 1.f;
	effectiveness(ControlAllocation::YAW, 2) = 1.f;
	configure(allocator, effectiveness, 3);

	ControlVector command;
	command.setZero();
	command(ControlAllocation::ROLL) = 0.25f;
	command(ControlAllocation::PITCH) = -0.4f;
	command(ControlAllocation::YAW) = 0.1f;
	allocator.setControlSetpoint(command);
	allocator.allocate();

	EXPECT_FALSE(allocator.usedFallback());
	EXPECT_EQ(allocator.getDiagnostics().solver_status, 1);
	EXPECT_EQ(allocator.getDiagnostics().solver_err, 0);
	EXPECT_EQ(allocator.getDiagnostics().active_rows, 3);
	EXPECT_TRUE(allocator.getDiagnostics().full_row_rank);
	EXPECT_NEAR(allocator.getActuatorSetpoint()(0), 0.25f, 2e-5f);
	EXPECT_NEAR(allocator.getActuatorSetpoint()(1), -0.4f, 2e-5f);
	EXPECT_NEAR(allocator.getActuatorSetpoint()(2), 0.1f, 2e-5f);
}

TEST(ControlAllocationWLSTest, SaturationAddsAndLaterRemovesActiveConstraints)
{
	ControlAllocationWLS allocator;
	EffectivenessMatrix effectiveness;
	effectiveness.setZero();
	effectiveness(ControlAllocation::ROLL, 0) = 1.f;
	effectiveness(ControlAllocation::ROLL, 1) = 1.f;

	ActuatorVector minimum;
	ActuatorVector maximum;
	ActuatorVector trim;
	ActuatorVector linearization_point;
	minimum.setZero();
	maximum.setAll(0.4f);
	trim.setZero();
	linearization_point.setZero();
	allocator.setActuatorMin(minimum);
	allocator.setActuatorMax(maximum);
	allocator.setEffectivenessMatrix(effectiveness, trim, linearization_point, 2, false);

	ControlVector command;
	command.setZero();
	command(ControlAllocation::ROLL) = 1.f;
	allocator.setControlSetpoint(command);
	allocator.allocate();
	EXPECT_NEAR(allocator.getActuatorSetpoint()(0), 0.4f, 1e-6f);
	EXPECT_NEAR(allocator.getActuatorSetpoint()(1), 0.4f, 1e-6f);
	EXPECT_FALSE(allocator.usedFallback());

	command.setZero();
	allocator.setControlSetpoint(command);
	allocator.allocate();
	EXPECT_NEAR(allocator.getActuatorSetpoint()(0), 0.f, 1e-5f);
	EXPECT_NEAR(allocator.getActuatorSetpoint()(1), 0.f, 1e-5f);
	EXPECT_FALSE(allocator.usedFallback());
}

TEST(ControlAllocationWLSTest, SolvesActuatorDeltaAroundTrim)
{
	ControlAllocationWLS allocator;
	EffectivenessMatrix effectiveness;
	effectiveness.setZero();
	effectiveness(ControlAllocation::ROLL, 0) = 1.f;

	ActuatorVector trim;
	trim.setZero();
	trim(0) = 0.3f;
	configure(allocator, effectiveness, 1, trim);

	ControlVector command;
	command.setZero();
	allocator.setControlSetpoint(command);
	allocator.allocate();
	EXPECT_NEAR(allocator.getActuatorSetpoint()(0), 0.3f, 1e-6f);

	command(ControlAllocation::ROLL) = 0.2f;
	allocator.setControlSetpoint(command);
	allocator.allocate();
	EXPECT_NEAR(allocator.getActuatorSetpoint()(0), 0.5f, 2e-5f);
}

TEST(ControlAllocationWLSTest, RankDeficientEffectivenessRemainsRegularized)
{
	ControlAllocationWLS allocator;
	EffectivenessMatrix effectiveness;
	effectiveness.setZero();

	for (int actuator = 0; actuator < 4; ++actuator) {
		effectiveness(ControlAllocation::ROLL, actuator) = 1.f;
		effectiveness(ControlAllocation::PITCH, actuator) = 2.f;
	}

	configure(allocator, effectiveness, 4);
	ControlVector command;
	command.setZero();
	command(ControlAllocation::ROLL) = 0.2f;
	command(ControlAllocation::PITCH) = 0.4f;
	allocator.setControlSetpoint(command);
	allocator.allocate();

	EXPECT_FALSE(allocator.usedFallback());
	EXPECT_FALSE(allocator.getDiagnostics().full_row_rank);
	EXPECT_EQ(allocator.getDiagnostics().solver_status, 1);

	for (int actuator = 0; actuator < 4; ++actuator) {
		EXPECT_TRUE(std::isfinite(allocator.getActuatorSetpoint()(actuator)));
	}
}

TEST(ControlAllocationWLSTest, InvalidCommandFallsBackSafelyToTrim)
{
	ControlAllocationWLS allocator;
	EffectivenessMatrix effectiveness;
	effectiveness.setZero();
	effectiveness(ControlAllocation::ROLL, 0) = 1.f;
	configure(allocator, effectiveness, 1);

	ControlVector command;
	command.setZero();
	command(ControlAllocation::ROLL) = NAN;
	allocator.setControlSetpoint(command);
	allocator.allocate();

	EXPECT_TRUE(allocator.usedFallback());
	EXPECT_EQ(allocator.getDiagnostics().solver_status, -2);
	EXPECT_EQ(allocator.getDiagnostics().solver_err, 1);
	EXPECT_TRUE(std::isfinite(allocator.getActuatorSetpoint()(0)));
	EXPECT_NEAR(allocator.getActuatorSetpoint()(0), 0.f, 1e-6f);
}

TEST(ControlAllocationWLSTest, ActualVehicleDimensionsSatisfyBoundedOptimalityConditions)
{
	// Physical DF4/SHC09 surface matrices and the Iris X rotor geometry.
	const float df4[3][4] = {
		{-0.166227f, 0.f, 0.166227f, 0.f},
		{0.f, -0.166227f, 0.f, 0.166227f},
		{0.068681f, 0.068681f, 0.068681f, 0.068681f}
	};
	const float shc09[3][6] = {
		{-0.343572f, -0.171786f, 0.171786f, 0.343572f, 0.171786f, -0.171786f},
		{0.f, 0.297542f, 0.297542f, 0.f, -0.297542f, -0.297542f},
		{0.084928f, 0.084928f, 0.084928f, 0.084928f, 0.084928f, 0.084928f}
	};
	const float iris_x[4] = {0.125f, -0.1385f, 0.125f, -0.1385f};
	const float iris_y[4] = {0.22f, -0.206f, -0.223f, 0.2025f};
	const float iris_km[4] = {0.0157f, 0.0157f, -0.0157f, -0.0157f};

	for (int model = 0; model < 3; ++model) {
		SCOPED_TRACE(model);
		ControlAllocationWLS allocator;
		EffectivenessMatrix effectiveness;
		const int actuators = model == 1 ? 6 : 4;

		for (int actuator = 0; actuator < actuators; ++actuator) {
			if (model < 2) {
				for (int axis = 0; axis < 3; ++axis) {
					effectiveness(axis, actuator) = model == 0 ? df4[axis][actuator] : shc09[axis][actuator];
				}

			} else {
				effectiveness(0, actuator) = -iris_y[actuator] * 8.5f;
				effectiveness(1, actuator) = iris_x[actuator] * 8.5f;
				effectiveness(2, actuator) = -iris_km[actuator] * 8.5f;
				effectiveness(5, actuator) = -8.5f;
			}
		}

		ActuatorVector trim;
		trim.setAll(0.05f);
		configure(allocator, effectiveness, actuators, trim);
		ActuatorVector minimum;
		minimum.setAll(model == 2 ? 0.f : -1.f);
		allocator.setActuatorMin(minimum);
		ControlVector weights;
		weights.setAll(1.f);
		weights(0) = 0.7f;
		weights(1) = 0.8f;
		constexpr float gamma = 1000.f;
		allocator.setWeights(weights, 1.f, gamma);

		for (int sample = 0; sample < 80; ++sample) {
			SCOPED_TRACE(sample);
			ControlVector command;

			for (int axis = 0; axis < 3; ++axis) {
				command(axis) = (sample % 3 == 0 ? 2.f : 0.1f) * sinf(0.7f * sample + axis);
			}

			if (model == 2) {
				command(5) = -16.f - 20.f * sinf(0.5f * sample);
			}

			allocator.setControlSetpoint(command);
			allocator.allocate();
			ASSERT_FALSE(allocator.usedFallback());
			const ActuatorVector output = allocator.getActuatorSetpoint();

			// Independently check KKT conditions of the original weighted box
			// problem in double precision, including the actuator penalty/trim.
			for (int actuator = 0; actuator < actuators; ++actuator) {
				EXPECT_GE(output(actuator), minimum(actuator) - 1e-6f);
				EXPECT_LE(output(actuator), 1.f + 1e-6f);
				double gradient = static_cast<double>(output(actuator) - trim(actuator));
				double scale = 1.;

				for (int axis = 0; axis < 6; ++axis) {
					double residual = -static_cast<double>(command(axis));

					for (int j = 0; j < actuators; ++j) {
						residual += static_cast<double>(effectiveness(axis, j)) *
							    (static_cast<double>(output(j)) - static_cast<double>(trim(j)));
					}

					const double coefficient = static_cast<double>(gamma) * static_cast<double>(weights(axis)) *
							   static_cast<double>(weights(axis)) * static_cast<double>(effectiveness(axis, actuator));
					gradient += coefficient * residual;
					scale += fabs(coefficient) * (1. + fabs(static_cast<double>(command(axis))));
				}

				const double tolerance = 2e-5 * scale;

				if (output(actuator) <= minimum(actuator) + 1e-5f) {
					EXPECT_GE(gradient, -tolerance);

				} else if (output(actuator) >= 1.f - 1e-5f) {
					EXPECT_LE(gradient, tolerance);

				} else {
					EXPECT_NEAR(gradient, 0., tolerance);
				}
			}
		}
	}
}

TEST(ControlAllocationWLSTest, AllVariablesFixedThenReleasedAndGeometryChanged)
{
	ControlAllocationWLS allocator;
	EffectivenessMatrix effectiveness;
	for (int actuator = 0; actuator < 6; ++actuator) {
		effectiveness(actuator % 3, actuator) = 1.f;
	}
	configure(allocator, effectiveness, 6);
	ActuatorVector fixed;
	fixed.setAll(0.2f);
	allocator.setActuatorMin(fixed);
	allocator.setActuatorMax(fixed);
	ControlVector command;
	command(0) = 0.5f;
	allocator.setControlSetpoint(command);
	allocator.allocate();
	EXPECT_FALSE(allocator.usedFallback());
	for (int actuator = 0; actuator < 6; ++actuator) {
		EXPECT_NEAR(allocator.getActuatorSetpoint()(actuator), 0.2f, 1e-6f);
	}

	// Reconfigure to four thrust-only actuators; the compact row map and
	// the warm active set must not retain the previous torque geometry.
	effectiveness.setZero();
	for (int actuator = 0; actuator < 4; ++actuator) {
		effectiveness(5, actuator) = -1.f;
	}
	configure(allocator, effectiveness, 4);
	command.setZero();
	command(5) = -2.f;
	allocator.setControlSetpoint(command);
	allocator.allocate();
	EXPECT_FALSE(allocator.usedFallback());
	for (int actuator = 0; actuator < 4; ++actuator) {
		EXPECT_NEAR(allocator.getActuatorSetpoint()(actuator), 0.5f, 1e-4f);
	}
}

TEST(ControlAllocationWLSTest, SmallNonzeroAxisIsNotDroppedByRankTolerance)
{
	ControlAllocationWLS allocator;
	EffectivenessMatrix effectiveness;
	effectiveness(5, 0) = 1e-7f;
	configure(allocator, effectiveness, 1);
	ControlVector weights;
	weights.setAll(1.f);
	allocator.setWeights(weights, 1.f, 1e12f);
	ControlVector command;
	command(5) = 1e-6f;
	allocator.setControlSetpoint(command);
	allocator.allocate();
	EXPECT_FALSE(allocator.usedFallback());
	EXPECT_NEAR(allocator.getActuatorSetpoint()(0), 0.1f / 1.01f, 1e-6f);
}

TEST(ControlAllocationWLSTest, SupportsEveryActuatorCountAndReleasesSaturation)
{
	for (int actuators = 0; actuators <= ControlAllocation::NUM_ACTUATORS; ++actuators) {
		SCOPED_TRACE(actuators);
		ControlAllocationWLS allocator;
		EffectivenessMatrix effectiveness;
		int counts[6] {};

		for (int actuator = 0; actuator < actuators; ++actuator) {
			effectiveness(actuator % 6, actuator) = 1.f;
			++counts[actuator % 6];
		}

		configure(allocator, effectiveness, actuators);
		ControlVector weights;
		weights.setAll(1.f);
		allocator.setWeights(weights, 1.f, 1000.f);
		ControlVector command;
		command.setAll(10.f);
		allocator.setControlSetpoint(command);
		allocator.allocate();
		ASSERT_FALSE(allocator.usedFallback());

		for (int actuator = 0; actuator < actuators; ++actuator) {
			EXPECT_NEAR(allocator.getActuatorSetpoint()(actuator), 1.f, 1e-5f);
		}

		command.setAll(0.2f);
		allocator.setControlSetpoint(command);
		allocator.allocate();
		ASSERT_FALSE(allocator.usedFallback());

		for (int actuator = 0; actuator < actuators; ++actuator) {
			const float expected = 200.f / (1.f + 1000.f * counts[actuator % 6]);
			EXPECT_NEAR(allocator.getActuatorSetpoint()(actuator), expected, 5e-5f);
		}
	}
}
