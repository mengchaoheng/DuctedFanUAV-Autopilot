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
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ControlAllocationLPCATest.cpp
 *
 * Numerical coverage for the supported LPCA template dimensions.
 */

#include <gtest/gtest.h>

#include <ControlAllocationLPCA.hpp>

#include <cmath>

using namespace matrix;

namespace
{
using EffectivenessMatrix = matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS>;
using ControlVector = matrix::Vector<float, ControlAllocation::NUM_AXES>;
using ActuatorVector = ControlAllocation::ActuatorVector;

EffectivenessMatrix makeEffectiveness(int rows, int actuators)
{
	EffectivenessMatrix effectiveness;
	effectiveness.setZero();

	for (int row = 0; row < rows; ++row) {
		const int axis = row < 3 ? row : ControlAllocation::THRUST_Z;
		effectiveness(axis, row) = 1.f;
	}

	for (int actuator = rows; actuator < actuators; ++actuator) {
		for (int row = 0; row < rows; ++row) {
			const int axis = row < 3 ? row : ControlAllocation::THRUST_Z;
			const float sign = ((actuator + row) & 1) ? -1.f : 1.f;
			effectiveness(axis, actuator) = sign * (0.1f + 0.02f * row);
		}
	}

	return effectiveness;
}

void configure(ControlAllocationLPCA &allocator, const EffectivenessMatrix &effectiveness, int actuators,
	       bool metric_allocation = false, float actuator_minimum = -1.f)
{
	ActuatorVector actuator_min;
	ActuatorVector actuator_max;
	ActuatorVector actuator_trim;
	ActuatorVector linearization_point;
	actuator_min.setAll(actuator_minimum);
	actuator_max.setAll(1.f);
	actuator_trim.setZero();
	linearization_point.setZero();
	allocator.setMetricAllocation(metric_allocation);
	allocator.setActuatorMin(actuator_min);
	allocator.setActuatorMax(actuator_max);
	allocator.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, actuators, false);
}

ControlVector makeControlSetpoint(int rows, float scale = 1.f)
{
	ControlVector control;
	control.setZero();
	control(ControlAllocation::ROLL) = 0.22f * scale;
	control(ControlAllocation::PITCH) = -0.17f * scale;
	control(ControlAllocation::YAW) = 0.11f * scale;

	if (rows == 4) {
		control(ControlAllocation::THRUST_Z) = -0.31f * scale;
	}

	return control;
}

void expectFiniteAndBounded(const ControlAllocation &allocator, int actuators)
{
	const ActuatorVector &actuator_sp = allocator.getActuatorSetpoint();

	for (int actuator = 0; actuator < actuators; ++actuator) {
		EXPECT_TRUE(std::isfinite(actuator_sp(actuator)));
		EXPECT_GE(actuator_sp(actuator), -1.f);
		EXPECT_LE(actuator_sp(actuator), 1.f);
	}
}

void expectSupportedProblem(ControlAllocationLPCA::Method method, int rows, int actuators)
{
	ControlAllocationLPCA allocator(method);
	configure(allocator, makeEffectiveness(rows, actuators), actuators);
	allocator.setControlSetpoint(makeControlSetpoint(rows));
	allocator.allocate();

	EXPECT_FALSE(allocator.usedFallback()) << "rows=" << rows << ", actuators=" << actuators;
	EXPECT_EQ(allocator.getDiagnostics().solver_status, 1) << "rows=" << rows << ", actuators=" << actuators;
	EXPECT_EQ(allocator.getDiagnostics().solver_err, 0) << "rows=" << rows << ", actuators=" << actuators;
	EXPECT_EQ(allocator.getDiagnostics().active_rows, rows);
	EXPECT_TRUE(allocator.getDiagnostics().full_row_rank);
	expectFiniteAndBounded(allocator, actuators);
}
} // namespace

TEST(ControlAllocationLPCATest, SupportedDimensions)
{
	for (const ControlAllocationLPCA::Method method : {ControlAllocationLPCA::Method::DPLPCA,
			ControlAllocationLPCA::Method::DPscaledLPCA}) {
		expectSupportedProblem(method, 3, 4);
		expectSupportedProblem(method, 3, 6);
	}
}

TEST(ControlAllocationLPCATest, DPscaledCleansDegenerateArtificialBasis)
{
	ControlAllocationDPscaledLPCA allocator;
	EffectivenessMatrix effectiveness;
	effectiveness.setZero();

	// Phase I ends with two zero-valued artificial variables in the basis.
	// Replacing them with original columns must use the complete phase-I matrix.
	const float b[3][6] = {
		{ 0.f,  0.f,  1.f,  1.f, 0.f, 0.f},
		{ 0.f,  0.f,  1.f, -1.f, 0.f, 0.f},
		{-1.f, -1.f, -1.f,  1.f, 0.f, 0.f},
	};

	for (int row = 0; row < 3; ++row) {
		for (int actuator = 0; actuator < 6; ++actuator) {
			effectiveness(row, actuator) = b[row][actuator];
		}
	}

	configure(allocator, effectiveness, 6, true, 0.f);

	ControlVector setpoint;
	setpoint.setZero();
	setpoint(ControlAllocation::ROLL) = 1.f;
	allocator.setControlSetpoint(setpoint);
	allocator.allocate();

	EXPECT_FALSE(allocator.usedFallback());
	EXPECT_EQ(allocator.getDiagnostics().solver_status, 1);
	EXPECT_EQ(allocator.getDiagnostics().solver_err, 0);
	EXPECT_NEAR(allocator.getDiagnostics().solver_rho, 2.f, 1e-5f);

	const ActuatorVector &output = allocator.getActuatorSetpoint();
	EXPECT_NEAR(output(0), 0.f, 1e-5f);
	EXPECT_NEAR(output(1), 0.f, 1e-5f);
	EXPECT_NEAR(output(2), 0.5f, 1e-5f);
	EXPECT_NEAR(output(3), 0.5f, 1e-5f);
	EXPECT_NEAR(output(4), 0.f, 1e-5f);
	EXPECT_NEAR(output(5), 0.f, 1e-5f);
}

TEST(ControlAllocationLPCATest, PriorityAllocation)
{
	ControlAllocationLPCA allocator(ControlAllocationLPCA::Method::PCA);
	configure(allocator, makeEffectiveness(3, 6), 6);

	const ControlVector control = makeControlSetpoint(3);
	ControlVector higher;
	higher.setZero();
	higher(ControlAllocation::ROLL) = 0.08f;
	higher(ControlAllocation::PITCH) = -0.04f;
	higher(ControlAllocation::YAW) = 0.03f;

	allocator.setControlSetpoint(control);
	allocator.setControlSetpointPriorityHigher(higher);
	allocator.allocate();

	EXPECT_FALSE(allocator.usedFallback());
	EXPECT_EQ(allocator.getDiagnostics().solver_status, 1);
	EXPECT_EQ(allocator.getDiagnostics().solver_err, 0);
	expectFiniteAndBounded(allocator, 6);
}

TEST(ControlAllocationLPCATest, PriorityRequiresExplicitSplit)
{
	ControlAllocationLPCA priority(ControlAllocationLPCA::Method::PCA);
	configure(priority, makeEffectiveness(3, 6), 6);

	priority.setControlSetpoint(makeControlSetpoint(3));
	priority.allocate();

	EXPECT_TRUE(priority.usedFallback());
	EXPECT_EQ(priority.getDiagnostics().solver_status, -1);
	EXPECT_EQ(priority.getDiagnostics().solver_err, 5);
	expectFiniteAndBounded(priority, 6);
}

TEST(ControlAllocationLPCATest, PriorityAcceptsZeroHigherCommand)
{
	ControlAllocationLPCA priority(ControlAllocationLPCA::Method::PCA);
	configure(priority, makeEffectiveness(3, 6), 6);
	ControlVector higher;
	higher.setZero();

	priority.setControlSetpoint(makeControlSetpoint(3));
	priority.setControlSetpointPriorityHigher(higher);
	priority.allocate();

	EXPECT_FALSE(priority.usedFallback());
	EXPECT_EQ(priority.getDiagnostics().solver_status, 1);
	EXPECT_EQ(priority.getDiagnostics().solver_err, 0);
	expectFiniteAndBounded(priority, 6);
}

TEST(ControlAllocationLPCATest, SustainedSaturationRemainsBounded)
{
	ControlAllocationLPCA allocator(ControlAllocationLPCA::Method::DPscaledLPCA);
	configure(allocator, makeEffectiveness(3, 6), 6);

	for (int iteration = 0; iteration < 250; ++iteration) {
		const float sign = (iteration & 1) ? -1.f : 1.f;
		allocator.setControlSetpoint(makeControlSetpoint(3, 20.f * sign));
		allocator.allocate();
		expectFiniteAndBounded(allocator, 6);
	}
}

TEST(ControlAllocationLPCATest, RankDeficiencyFallsBackToInverse)
{
	ControlAllocationLPCA allocator(ControlAllocationLPCA::Method::DPLPCA);
	EffectivenessMatrix effectiveness;
	effectiveness.setZero();

	for (int actuator = 0; actuator < 6; ++actuator) {
		effectiveness(ControlAllocation::ROLL, actuator) = 1.f;
		effectiveness(ControlAllocation::PITCH, actuator) = 2.f;
		effectiveness(ControlAllocation::YAW, actuator) = -1.f;
	}

	configure(allocator, effectiveness, 6);
	allocator.setControlSetpoint(makeControlSetpoint(3));
	allocator.allocate();

	EXPECT_TRUE(allocator.usedFallback());
	EXPECT_EQ(allocator.getDiagnostics().solver_status, -1);
	EXPECT_EQ(allocator.getDiagnostics().solver_err, 1);
	expectFiniteAndBounded(allocator, 6);
}

TEST(ControlAllocationLPCATest, UnsupportedPcaAxesFallBackToInverse)
{
	ControlAllocationLPCA allocator(ControlAllocationLPCA::Method::PCA);
	EffectivenessMatrix effectiveness;
	effectiveness.setZero();
	effectiveness(ControlAllocation::ROLL, 0) = 1.f;
	effectiveness(ControlAllocation::PITCH, 1) = 1.f;
	effectiveness(ControlAllocation::THRUST_Z, 2) = 1.f;
	effectiveness(ControlAllocation::ROLL, 3) = 0.2f;

	configure(allocator, effectiveness, 6);
	ControlVector control;
	control.setZero();
	control(ControlAllocation::ROLL) = 0.2f;
	control(ControlAllocation::PITCH) = -0.1f;
	control(ControlAllocation::THRUST_Z) = -0.3f;
	ControlVector higher;
	higher.setZero();
	higher(ControlAllocation::ROLL) = 0.05f;
	allocator.setControlSetpoint(control);
	allocator.setControlSetpointPriorityHigher(higher);
	allocator.allocate();

	EXPECT_TRUE(allocator.usedFallback());
	EXPECT_EQ(allocator.getDiagnostics().solver_status, -1);
	EXPECT_EQ(allocator.getDiagnostics().solver_err, 4);
	expectFiniteAndBounded(allocator, 6);
}

TEST(ControlAllocationLPCATest, UnsupportedDimensionsFallBackToInverse)
{
	ControlAllocationLPCA unsupported_actuator_count(ControlAllocationLPCA::Method::DPLPCA);
	configure(unsupported_actuator_count, makeEffectiveness(3, 5), 5);
	unsupported_actuator_count.setControlSetpoint(makeControlSetpoint(3));
	unsupported_actuator_count.allocate();
	EXPECT_TRUE(unsupported_actuator_count.usedFallback());
	EXPECT_EQ(unsupported_actuator_count.getDiagnostics().solver_status, -1);
	EXPECT_EQ(unsupported_actuator_count.getDiagnostics().solver_err, 3);
	expectFiniteAndBounded(unsupported_actuator_count, 5);

	ControlAllocationLPCA too_many_rows(ControlAllocationLPCA::Method::DPLPCA);
	configure(too_many_rows, makeEffectiveness(4, 6), 6);
	too_many_rows.setControlSetpoint(makeControlSetpoint(4));
	too_many_rows.allocate();
	EXPECT_TRUE(too_many_rows.usedFallback());
	EXPECT_EQ(too_many_rows.getDiagnostics().solver_status, -1);
	EXPECT_EQ(too_many_rows.getDiagnostics().solver_err, 2);
	expectFiniteAndBounded(too_many_rows, 6);
}
