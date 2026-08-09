/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *   Copyright (c) 2004, Ola Harkegard. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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

/**
 * @file ControlAllocationWLS.hpp
 *
 * Fixed-size PX4 adapter for Ola Harkegard's QCAT weighted least-squares
 * active-set control allocator.
 */

#pragma once

#include "ControlAllocation.hpp"

class ControlAllocationWLS : public ControlAllocation
{
public:
	ControlAllocationWLS();

	void allocate() override;
	void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
				    const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point,
				    int num_actuators, bool update_normalization_scale) override;

	bool usedFallback() const override { return _used_fallback; }

	/**
	 * Configure the diagonal WLS weights. Axis weights use PX4's wrench order:
	 * [roll, pitch, yaw, thrust_x, thrust_y, thrust_z].
	 */
	void setWeights(const matrix::Vector<float, NUM_AXES> &control_weights, float actuator_weight, float gamma);

private:
	static constexpr int AUGMENTED_ROWS = NUM_AXES + NUM_ACTUATORS;
	static constexpr int MAX_ITERATIONS = 64;

	using EffectivenessMatrix = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>;
	using MixMatrix = matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES>;
	using AugmentedMatrix = matrix::Matrix<float, AUGMENTED_ROWS, NUM_ACTUATORS>;
	using AugmentedVector = matrix::Vector<float, AUGMENTED_ROWS>;

	enum class SolveResult {
		Success,
		InvalidInput,
		LeastSquaresFailure,
		IterationLimit,
	};

	void updateStandardProblem();
	void updateControlAllocationMatrixScale(const MixMatrix &mix);
	void buildUnitEffectiveness();
	void buildAugmentedProblem();
	void buildActuatorDeltaLimits();
	void prepareWarmStart(ActuatorVector &actuator_delta);

	SolveResult solveActiveSet(ActuatorVector &actuator_delta, int &iterations);
	bool solveReducedLeastSquares(const AugmentedVector &residual, ActuatorVector &step);
	bool allocateFallback(ActuatorVector &actuator_delta) const;
	void computeResidual(const ActuatorVector &actuator_delta, AugmentedVector &residual) const;
	bool isFiniteProblem() const;
	void updateRankDiagnostics();

	EffectivenessMatrix _effectiveness_unit;
	MixMatrix _mix_fallback;
	AugmentedMatrix _augmented_matrix;
	AugmentedMatrix _reduced_matrix;
	AugmentedVector _augmented_rhs;

	matrix::Vector<float, NUM_AXES> _control_weights;
	ActuatorVector _actuator_delta_min;
	ActuatorVector _actuator_delta_max;
	int8_t _working_set[NUM_ACTUATORS] {};

	float _actuator_weight{1.f};
	float _gamma{1e6f};
	bool _standard_problem_update_needed{true};
	bool _normalization_needs_update{false};
	bool _warm_start_valid{false};
	bool _used_fallback{false};
};
