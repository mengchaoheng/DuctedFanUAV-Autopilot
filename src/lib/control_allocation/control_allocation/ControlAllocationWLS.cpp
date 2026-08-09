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
 * @file ControlAllocationWLS.cpp
 *
 * This is a fixed-size adaptation of the active-set algorithm in QCAT's
 * wlsc_alloc.c. The MEX gateway, dynamic allocation, BLAS and LAPACK calls are
 * intentionally not carried over. Reduced least-squares systems are solved by
 * PX4 Matrix's Householder QR solver.
 */

#include "ControlAllocationWLS.hpp"

#include <cmath>
#include <cstring>
#include <matrix/matrix/LeastSquaresSolver.hpp>

namespace
{
constexpr float kBoundTolerance = 1e-6f;
constexpr float kDirectionTolerance = 1e-8f;
constexpr float kLagrangeTolerance = 1e-5f;
constexpr float kMatrixZeroTolerance = 1e-6f;
constexpr float kRankRelativeTolerance = 1e-5f;

constexpr int8_t kWorkingSetFree = 0;
constexpr int8_t kWorkingSetLower = -1;
constexpr int8_t kWorkingSetUpper = 1;
constexpr int8_t kWorkingSetFixed = 2;

constexpr int8_t kSolverInvalidInput = 1;
constexpr int8_t kSolverLeastSquaresFailure = 2;
constexpr int8_t kSolverIterationLimit = 3;

float constrainFloat(float value, float lower, float upper)
{
	return fminf(fmaxf(value, lower), upper);
}

bool nearlyEqual(float lhs, float rhs)
{
	return fabsf(lhs - rhs) <= kBoundTolerance;
}
} // namespace

ControlAllocationWLS::ControlAllocationWLS()
{
	_control_weights.setAll(1.f);
}

void
ControlAllocationWLS::setWeights(const matrix::Vector<float, NUM_AXES> &control_weights, float actuator_weight,
				 float gamma)
{
	bool changed = fabsf(_actuator_weight - actuator_weight) > FLT_EPSILON
		       || fabsf(_gamma - gamma) > FLT_EPSILON;

	for (int axis = 0; axis < NUM_AXES; ++axis) {
		changed = changed || fabsf(_control_weights(axis) - control_weights(axis)) > FLT_EPSILON;
	}

	if (changed) {
		_control_weights = control_weights;
		_actuator_weight = actuator_weight;
		_gamma = gamma;
		_standard_problem_update_needed = true;
		_warm_start_valid = false;
		memset(_working_set, 0, sizeof(_working_set));
	}
}

void
ControlAllocationWLS::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
	bool update_normalization_scale)
{
	ControlAllocation::setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, num_actuators,
			update_normalization_scale);
	_standard_problem_update_needed = true;
	_normalization_needs_update = update_normalization_scale;
	_warm_start_valid = false;
	memset(_working_set, 0, sizeof(_working_set));
}

void
ControlAllocationWLS::allocate()
{
	updateStandardProblem();
	_prev_actuator_sp = _actuator_sp;

	ActuatorVector actuator_delta;
	prepareWarmStart(actuator_delta);

	_diagnostics = {};
	updateRankDiagnostics();
	_used_fallback = false;

	int iterations = 0;
	const SolveResult result = solveActiveSet(actuator_delta, iterations);

	if (result == SolveResult::Success) {
		_diagnostics.solver_status = 1;
		_warm_start_valid = true;

	} else {
		_diagnostics.solver_status = -2;

		switch (result) {
		case SolveResult::InvalidInput:
			_diagnostics.solver_err = kSolverInvalidInput;
			break;

		case SolveResult::LeastSquaresFailure:
			_diagnostics.solver_err = kSolverLeastSquaresFailure;
			break;

		case SolveResult::IterationLimit:
			_diagnostics.solver_err = kSolverIterationLimit;
			break;

		case SolveResult::Success:
			break;
		}

		_used_fallback = true;
		_warm_start_valid = false;
		memset(_working_set, 0, sizeof(_working_set));

		if (!allocateFallback(actuator_delta)) {
			actuator_delta.setZero();
		}
	}

	_actuator_sp = _actuator_trim + actuator_delta;
	clipActuatorSetpoint();
}

void
ControlAllocationWLS::updateStandardProblem()
{
	if (!_standard_problem_update_needed) {
		buildActuatorDeltaLimits();
		return;
	}

	MixMatrix mix_raw;
	matrix::geninv(_effectiveness, mix_raw);

	if (_normalization_needs_update && !_had_actuator_failure) {
		updateControlAllocationMatrixScale(mix_raw);
		_normalization_needs_update = false;
	}

	buildUnitEffectiveness();
	buildAugmentedProblem();
	buildActuatorDeltaLimits();
	matrix::geninv(_effectiveness_unit, _mix_fallback);
	_standard_problem_update_needed = false;
}

void
ControlAllocationWLS::updateControlAllocationMatrixScale(const MixMatrix &mix)
{
	if (_normalize_rpy) {
		int num_non_zero_roll_torque = 0;
		int num_non_zero_pitch_torque = 0;

		for (int actuator = 0; actuator < _num_actuators; ++actuator) {
			num_non_zero_roll_torque += fabsf(mix(actuator, ROLL)) > 1e-3f;
			num_non_zero_pitch_torque += fabsf(mix(actuator, PITCH)) > 1e-3f;
		}

		float roll_norm_scale = 1.f;
		float pitch_norm_scale = 1.f;

		if (num_non_zero_roll_torque > 0) {
			roll_norm_scale = sqrtf(mix.col(ROLL).norm_squared() / (num_non_zero_roll_torque / 2.f));
		}

		if (num_non_zero_pitch_torque > 0) {
			pitch_norm_scale = sqrtf(mix.col(PITCH).norm_squared() / (num_non_zero_pitch_torque / 2.f));
		}

		_control_allocation_scale(ROLL) = fmaxf(roll_norm_scale, pitch_norm_scale);
		_control_allocation_scale(PITCH) = _control_allocation_scale(ROLL);
		_control_allocation_scale(YAW) = mix.col(YAW).max();

	} else {
		_control_allocation_scale(ROLL) = 1.f;
		_control_allocation_scale(PITCH) = 1.f;
		_control_allocation_scale(YAW) = 1.f;
	}

	_control_allocation_scale(THRUST_Z) = 1.f;

	for (int axis_idx = 2; axis_idx >= 0; --axis_idx) {
		int num_non_zero_thrust = 0;
		float norm_sum = 0.f;

		for (int actuator = 0; actuator < _num_actuators; ++actuator) {
			const float norm = fabsf(mix(actuator, 3 + axis_idx));
			norm_sum += norm;
			num_non_zero_thrust += norm > FLT_EPSILON;
		}

		if (num_non_zero_thrust > 0) {
			_control_allocation_scale(3 + axis_idx) = norm_sum / num_non_zero_thrust;

		} else {
			_control_allocation_scale(3 + axis_idx) = _control_allocation_scale(THRUST_Z);
		}
	}
}

void
ControlAllocationWLS::buildUnitEffectiveness()
{
	_effectiveness_unit.setZero();

	for (int axis = 0; axis < NUM_AXES; ++axis) {
		for (int actuator = 0; actuator < _num_actuators; ++actuator) {
			_effectiveness_unit(axis, actuator) = _control_allocation_scale(axis) * _effectiveness(axis, actuator);
		}
	}
}

void
ControlAllocationWLS::buildAugmentedProblem()
{
	_augmented_matrix.setZero();
	_augmented_rhs.setZero();
	const float control_scale = sqrtf(_gamma);

	for (int axis = 0; axis < NUM_AXES; ++axis) {
		for (int actuator = 0; actuator < _num_actuators; ++actuator) {
			_augmented_matrix(axis, actuator) = control_scale * _control_weights(axis)
							    * _effectiveness_unit(axis, actuator);
		}
	}

	for (int actuator = 0; actuator < _num_actuators; ++actuator) {
		_augmented_matrix(NUM_AXES + actuator, actuator) = _actuator_weight;
	}
}

void
ControlAllocationWLS::buildActuatorDeltaLimits()
{
	_actuator_delta_min.setZero();
	_actuator_delta_max.setZero();

	for (int actuator = 0; actuator < _num_actuators; ++actuator) {
		if (std::isfinite(_actuator_min(actuator)) && std::isfinite(_actuator_max(actuator))
		    && _actuator_max(actuator) >= _actuator_min(actuator)) {
			_actuator_delta_min(actuator) = _actuator_min(actuator) - _actuator_trim(actuator);
			_actuator_delta_max(actuator) = _actuator_max(actuator) - _actuator_trim(actuator);
		}
	}
}

void
ControlAllocationWLS::prepareWarmStart(ActuatorVector &actuator_delta)
{
	actuator_delta.setZero();

	for (int actuator = 0; actuator < _num_actuators; ++actuator) {
		float previous_delta = _actuator_sp(actuator) - _actuator_trim(actuator);

		if (!std::isfinite(previous_delta)) {
			previous_delta = 0.f;
		}

		actuator_delta(actuator) = constrainFloat(previous_delta, _actuator_delta_min(actuator),
					     _actuator_delta_max(actuator));

		if (_actuator_delta_max(actuator) - _actuator_delta_min(actuator) <= kBoundTolerance) {
			actuator_delta(actuator) = _actuator_delta_min(actuator);
			_working_set[actuator] = kWorkingSetFixed;
			continue;
		}

		if (!_warm_start_valid) {
			_working_set[actuator] = kWorkingSetFree;

		} else if (_working_set[actuator] == kWorkingSetLower) {
			if (nearlyEqual(actuator_delta(actuator), _actuator_delta_min(actuator))) {
				actuator_delta(actuator) = _actuator_delta_min(actuator);

			} else {
				_working_set[actuator] = kWorkingSetFree;
			}

		} else if (_working_set[actuator] == kWorkingSetUpper) {
			if (nearlyEqual(actuator_delta(actuator), _actuator_delta_max(actuator))) {
				actuator_delta(actuator) = _actuator_delta_max(actuator);

			} else {
				_working_set[actuator] = kWorkingSetFree;
			}

		} else {
			_working_set[actuator] = kWorkingSetFree;
		}
	}
}

ControlAllocationWLS::SolveResult
ControlAllocationWLS::solveActiveSet(ActuatorVector &actuator_delta, int &iterations)
{
	if (!isFiniteProblem()) {
		return SolveResult::InvalidInput;
	}

	const float control_scale = sqrtf(_gamma);

	for (int axis = 0; axis < NUM_AXES; ++axis) {
		_augmented_rhs(axis) = control_scale * _control_weights(axis) * (_control_sp(axis) - _control_trim(axis));
	}

	for (int actuator = 0; actuator < NUM_ACTUATORS; ++actuator) {
		_augmented_rhs(NUM_AXES + actuator) = 0.f;
	}

	AugmentedVector residual;
	computeResidual(actuator_delta, residual);

	for (iterations = 1; iterations <= MAX_ITERATIONS; ++iterations) {
		ActuatorVector step;

		if (!solveReducedLeastSquares(residual, step)) {
			return SolveResult::LeastSquaresFailure;
		}

		ActuatorVector candidate = actuator_delta + step;
		bool feasible = true;

		for (int actuator = 0; actuator < _num_actuators; ++actuator) {
			if (_working_set[actuator] == kWorkingSetFree
			    && (candidate(actuator) < _actuator_delta_min(actuator) - kBoundTolerance
				|| candidate(actuator) > _actuator_delta_max(actuator) + kBoundTolerance)) {
				feasible = false;
				break;
			}
		}

		if (feasible) {
			for (int actuator = 0; actuator < _num_actuators; ++actuator) {
				if (_working_set[actuator] == kWorkingSetFree) {
					actuator_delta(actuator) = constrainFloat(candidate(actuator), _actuator_delta_min(actuator),
									  _actuator_delta_max(actuator));
				}
			}

			computeResidual(actuator_delta, residual);
			int constraint_to_remove = -1;
			float minimum_multiplier = -kLagrangeTolerance;

			for (int actuator = 0; actuator < _num_actuators; ++actuator) {
				if (_working_set[actuator] != kWorkingSetLower
				    && _working_set[actuator] != kWorkingSetUpper) {
					continue;
				}

				float unsigned_multiplier = 0.f;

				for (int row = 0; row < AUGMENTED_ROWS; ++row) {
					unsigned_multiplier += _augmented_matrix(row, actuator) * residual(row);
				}

				const float multiplier = _working_set[actuator] * unsigned_multiplier;

				if (multiplier < minimum_multiplier) {
					minimum_multiplier = multiplier;
					constraint_to_remove = actuator;
				}
			}

			if (constraint_to_remove < 0) {
				return SolveResult::Success;
			}

			_working_set[constraint_to_remove] = kWorkingSetFree;

		} else {
			float alpha = 1.f;
			int blocking_actuator = -1;

			for (int actuator = 0; actuator < _num_actuators; ++actuator) {
				if (_working_set[actuator] != kWorkingSetFree) {
					continue;
				}

				float actuator_alpha = 2.f;

				if (step(actuator) > kDirectionTolerance) {
					actuator_alpha = (_actuator_delta_max(actuator) - actuator_delta(actuator)) / step(actuator);

				} else if (step(actuator) < -kDirectionTolerance) {
					actuator_alpha = (_actuator_delta_min(actuator) - actuator_delta(actuator)) / step(actuator);
				}

				if (actuator_alpha < alpha) {
					alpha = actuator_alpha;
					blocking_actuator = actuator;
				}
			}

			if (blocking_actuator < 0 || !std::isfinite(alpha)) {
				return SolveResult::LeastSquaresFailure;
			}

			alpha = constrainFloat(alpha, 0.f, 1.f);

			for (int actuator = 0; actuator < _num_actuators; ++actuator) {
				if (_working_set[actuator] == kWorkingSetFree) {
					actuator_delta(actuator) += alpha * step(actuator);
					actuator_delta(actuator) = constrainFloat(actuator_delta(actuator),
										 _actuator_delta_min(actuator), _actuator_delta_max(actuator));
				}
			}

			if (step(blocking_actuator) > 0.f) {
				actuator_delta(blocking_actuator) = _actuator_delta_max(blocking_actuator);
				_working_set[blocking_actuator] = kWorkingSetUpper;

			} else {
				actuator_delta(blocking_actuator) = _actuator_delta_min(blocking_actuator);
				_working_set[blocking_actuator] = kWorkingSetLower;
			}

			computeResidual(actuator_delta, residual);
		}
	}

	return SolveResult::IterationLimit;
}

bool
ControlAllocationWLS::solveReducedLeastSquares(const AugmentedVector &residual, ActuatorVector &step)
{
	_reduced_matrix.setZero();
	step.setZero();
	int free_count = 0;
	int excluded_actuators[NUM_ACTUATORS] {};
	int excluded_count = 0;

	for (int actuator = 0; actuator < NUM_ACTUATORS; ++actuator) {
		if (actuator < _num_actuators && _working_set[actuator] == kWorkingSetFree) {
			for (int row = 0; row < AUGMENTED_ROWS; ++row) {
				_reduced_matrix(row, free_count) = _augmented_matrix(row, actuator);
			}

			++free_count;

		} else {
			excluded_actuators[excluded_count++] = actuator;
		}
	}

	// LeastSquaresSolver has compile-time dimensions. Complete the packed free
	// columns with mutually orthogonal dummy columns in actuator-penalty rows.
	// They cannot change the solution of the real free-variable subsystem.
	for (int column = free_count; column < NUM_ACTUATORS; ++column) {
		_reduced_matrix(NUM_AXES + excluded_actuators[column - free_count], column) = 1.f;
	}

	matrix::LeastSquaresSolver<float, AUGMENTED_ROWS, NUM_ACTUATORS> solver(_reduced_matrix);
	const ActuatorVector packed_step = solver.solve(residual);
	int free_index = 0;

	for (int actuator = 0; actuator < _num_actuators; ++actuator) {
		if (_working_set[actuator] == kWorkingSetFree) {
			if (!std::isfinite(packed_step(free_index))) {
				return false;
			}

			step(actuator) = packed_step(free_index++);
		}
	}

	return true;
}

bool
ControlAllocationWLS::allocateFallback(ActuatorVector &actuator_delta) const
{
	actuator_delta = _mix_fallback * (_control_sp - _control_trim);
	bool valid = true;

	for (int actuator = 0; actuator < _num_actuators; ++actuator) {
		if (!std::isfinite(actuator_delta(actuator))) {
			actuator_delta(actuator) = 0.f;
			valid = false;
		}

		actuator_delta(actuator) = constrainFloat(actuator_delta(actuator), _actuator_delta_min(actuator),
						     _actuator_delta_max(actuator));
	}

	return valid;
}

void
ControlAllocationWLS::computeResidual(const ActuatorVector &actuator_delta, AugmentedVector &residual) const
{
	residual = _augmented_rhs - _augmented_matrix * actuator_delta;
}

bool
ControlAllocationWLS::isFiniteProblem() const
{
	if (_num_actuators < 0 || _num_actuators > NUM_ACTUATORS || !std::isfinite(_gamma) || _gamma <= 0.f
	    || !std::isfinite(_actuator_weight) || _actuator_weight <= 0.f) {
		return false;
	}

	for (int axis = 0; axis < NUM_AXES; ++axis) {
		if (!std::isfinite(_control_sp(axis)) || !std::isfinite(_control_trim(axis))
		    || !std::isfinite(_control_weights(axis)) || _control_weights(axis) <= 0.f) {
			return false;
		}
	}

	for (int actuator = 0; actuator < _num_actuators; ++actuator) {
		if (!std::isfinite(_actuator_delta_min(actuator)) || !std::isfinite(_actuator_delta_max(actuator))
		    || _actuator_delta_min(actuator) > _actuator_delta_max(actuator)) {
			return false;
		}

		for (int axis = 0; axis < NUM_AXES; ++axis) {
			if (!std::isfinite(_effectiveness_unit(axis, actuator))) {
				return false;
			}
		}
	}

	return true;
}

void
ControlAllocationWLS::updateRankDiagnostics()
{
	float work[NUM_AXES][NUM_ACTUATORS] {};
	int active_rows[NUM_AXES] {};
	int rows = 0;
	float max_abs = 0.f;

	for (int axis = 0; axis < NUM_AXES; ++axis) {
		bool active = false;

		for (int actuator = 0; actuator < _num_actuators; ++actuator) {
			active = active || fabsf(_effectiveness_unit(axis, actuator)) > kMatrixZeroTolerance;
		}

		if (active) {
			active_rows[rows++] = axis;
		}
	}

	for (int row = 0; row < rows; ++row) {
		for (int actuator = 0; actuator < _num_actuators; ++actuator) {
			work[row][actuator] = _effectiveness_unit(active_rows[row], actuator);
			max_abs = fmaxf(max_abs, fabsf(work[row][actuator]));
		}
	}

	const float tolerance = fmaxf(kMatrixZeroTolerance, max_abs * kRankRelativeTolerance);
	int rank = 0;

	for (int column = 0; column < _num_actuators && rank < rows; ++column) {
		int pivot_row = rank;
		float pivot_abs = fabsf(work[pivot_row][column]);

		for (int row = rank + 1; row < rows; ++row) {
			if (fabsf(work[row][column]) > pivot_abs) {
				pivot_abs = fabsf(work[row][column]);
				pivot_row = row;
			}
		}

		if (pivot_abs <= tolerance) {
			continue;
		}

		if (pivot_row != rank) {
			for (int actuator = column; actuator < _num_actuators; ++actuator) {
				const float temporary = work[rank][actuator];
				work[rank][actuator] = work[pivot_row][actuator];
				work[pivot_row][actuator] = temporary;
			}
		}

		const float pivot = work[rank][column];

		for (int row = rank + 1; row < rows; ++row) {
			const float factor = work[row][column] / pivot;

			for (int actuator = column; actuator < _num_actuators; ++actuator) {
				work[row][actuator] -= factor * work[rank][actuator];
			}
		}

		++rank;
	}

	_diagnostics.active_rows = static_cast<uint8_t>(rows);
	_diagnostics.full_row_rank = rank == rows;
}
