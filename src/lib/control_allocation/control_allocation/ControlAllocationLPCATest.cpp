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
 * Numerical coverage for the LPCA adapter and its supported template sizes.
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

void configure(ControlAllocationLPCA &allocator, const EffectivenessMatrix &effectiveness, int actuators)
{
	ActuatorVector actuator_min;
	ActuatorVector actuator_max;
	ActuatorVector actuator_trim;
	ActuatorVector linearization_point;
	actuator_min.setAll(-1.f);
	actuator_max.setAll(1.f);
	actuator_trim.setZero();
	linearization_point.setZero();
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

TEST(ControlAllocationLPCATest, AllSupportedDimensions)
{
	for (const ControlAllocationLPCA::Method method : {ControlAllocationLPCA::Method::DPLPCA,
			ControlAllocationLPCA::Method::DPscaledLPCA}) {
		for (int rows = 3; rows <= 4; ++rows) {
			for (int actuators = 4; actuators <= 9; ++actuators) {
				expectSupportedProblem(method, rows, actuators);
			}
		}
	}
}

TEST(ControlAllocationLPCATest, PriorityAllocation)
{
	ControlAllocationLPCA allocator(ControlAllocationLPCA::Method::PCA);
	configure(allocator, makeEffectiveness(3, 9), 9);

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
	expectFiniteAndBounded(allocator, 9);
}

TEST(ControlAllocationLPCATest, PriorityRequiresExplicitSplit)
{
	ControlAllocationLPCA priority(ControlAllocationLPCA::Method::PCA);
	configure(priority, makeEffectiveness(3, 9), 9);

	priority.setControlSetpoint(makeControlSetpoint(3));
	priority.allocate();

	EXPECT_TRUE(priority.usedFallback());
	EXPECT_EQ(priority.getDiagnostics().solver_status, -1);
	EXPECT_EQ(priority.getDiagnostics().solver_err, 5);
	expectFiniteAndBounded(priority, 9);
}

TEST(ControlAllocationLPCATest, PriorityAcceptsZeroHigherCommand)
{
	ControlAllocationLPCA priority(ControlAllocationLPCA::Method::PCA);
	configure(priority, makeEffectiveness(3, 9), 9);
	ControlVector higher;
	higher.setZero();

	priority.setControlSetpoint(makeControlSetpoint(3));
	priority.setControlSetpointPriorityHigher(higher);
	priority.allocate();

	EXPECT_FALSE(priority.usedFallback());
	EXPECT_EQ(priority.getDiagnostics().solver_status, 1);
	EXPECT_EQ(priority.getDiagnostics().solver_err, 0);
	expectFiniteAndBounded(priority, 9);
}

TEST(ControlAllocationLPCATest, SustainedSaturationRemainsBounded)
{
	ControlAllocationLPCA allocator(ControlAllocationLPCA::Method::DPscaledLPCA);
	configure(allocator, makeEffectiveness(4, 9), 9);

	for (int iteration = 0; iteration < 250; ++iteration) {
		const float sign = (iteration & 1) ? -1.f : 1.f;
		allocator.setControlSetpoint(makeControlSetpoint(4, 20.f * sign));
		allocator.allocate();
		expectFiniteAndBounded(allocator, 9);
	}
}

TEST(ControlAllocationLPCATest, RankDeficiencyFallsBackToInverse)
{
	ControlAllocationLPCA allocator(ControlAllocationLPCA::Method::DPLPCA);
	EffectivenessMatrix effectiveness;
	effectiveness.setZero();

	for (int actuator = 0; actuator < 4; ++actuator) {
		effectiveness(ControlAllocation::ROLL, actuator) = 1.f;
		effectiveness(ControlAllocation::PITCH, actuator) = 2.f;
		effectiveness(ControlAllocation::YAW, actuator) = -1.f;
	}

	configure(allocator, effectiveness, 4);
	allocator.setControlSetpoint(makeControlSetpoint(3));
	allocator.allocate();

	EXPECT_TRUE(allocator.usedFallback());
	EXPECT_EQ(allocator.getDiagnostics().solver_status, -1);
	EXPECT_EQ(allocator.getDiagnostics().solver_err, 1);
	expectFiniteAndBounded(allocator, 4);
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

	configure(allocator, effectiveness, 4);
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
	expectFiniteAndBounded(allocator, 4);
}

TEST(ControlAllocationLPCATest, UnsupportedDimensionsFallBackToInverse)
{
	ControlAllocationLPCA too_few_actuators(ControlAllocationLPCA::Method::DPLPCA);
	configure(too_few_actuators, makeEffectiveness(3, 3), 3);
	too_few_actuators.setControlSetpoint(makeControlSetpoint(3));
	too_few_actuators.allocate();
	EXPECT_TRUE(too_few_actuators.usedFallback());
	EXPECT_EQ(too_few_actuators.getDiagnostics().solver_status, -1);
	EXPECT_EQ(too_few_actuators.getDiagnostics().solver_err, 3);
	expectFiniteAndBounded(too_few_actuators, 3);

	ControlAllocationLPCA too_many_rows(ControlAllocationLPCA::Method::DPLPCA);
	EffectivenessMatrix effectiveness;
	effectiveness.setZero();

	for (int axis = 0; axis < 5; ++axis) {
		effectiveness(axis, axis) = 1.f;
	}

	configure(too_many_rows, effectiveness, 5);
	ControlVector control;
	control.setAll(0.1f);
	too_many_rows.setControlSetpoint(control);
	too_many_rows.allocate();
	EXPECT_TRUE(too_many_rows.usedFallback());
	EXPECT_EQ(too_many_rows.getDiagnostics().solver_status, -1);
	EXPECT_EQ(too_many_rows.getDiagnostics().solver_err, 2);
	expectFiniteAndBounded(too_many_rows, 5);
}
