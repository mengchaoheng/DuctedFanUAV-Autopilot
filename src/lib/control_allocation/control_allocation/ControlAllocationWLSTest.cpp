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
