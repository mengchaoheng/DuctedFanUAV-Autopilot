/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

/**
 * @file ControlAllocationLPCA.cpp
 *
 * PX4 adapter for normalized INV and LPCA control allocation algorithms.
 */

#include "ControlAllocationLPCA.hpp"

#include <drivers/drv_hrt.h>
#include <float.h>
#include <cmath>
#include <cstring>
#include <time.h>

#if defined(__clang__)
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Warray-bounds"
# pragma clang diagnostic ignored "-Wdouble-promotion"
# pragma clang diagnostic ignored "-Wfloat-equal"
# pragma clang diagnostic ignored "-Wreorder"
# pragma clang diagnostic ignored "-Wshadow"
# pragma clang diagnostic ignored "-Wuninitialized"
# pragma clang diagnostic ignored "-Wunused-variable"
#elif defined(__GNUC__)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Warray-bounds"
# pragma GCC diagnostic ignored "-Wdouble-promotion"
# pragma GCC diagnostic ignored "-Wfloat-equal"
# pragma GCC diagnostic ignored "-Wreorder"
# pragma GCC diagnostic ignored "-Wshadow"
# pragma GCC diagnostic ignored "-Wuninitialized"
# pragma GCC diagnostic ignored "-Wunused-variable"
#endif

#include "pca/ControlAllocation.h"

namespace
{
constexpr float kMatrixZeroTolerance = 1e-6f;
constexpr float kRankRelativeTolerance = 1e-5f;
constexpr int8_t kLPCAUnavailableRank = 1;
constexpr int8_t kLPCAUnavailableRows = 2;
constexpr int8_t kLPCAUnavailableActuators = 3;

enum class LPCAMethod {
	Dir,
	DPScaled,
	Priority,
};

struct LPCADiagnostics {
	int err{0};
	float rho{0.f};
	float prepare_time_us{0.f};
	float core_time_us{0.f};
	float post_time_us{0.f};
	hrt_abstime prepare_start{0};
};

hrt_abstime runtimeMeasurementTimeUs()
{
#if defined(__PX4_POSIX)
	timespec ts {};
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return static_cast<hrt_abstime>(ts.tv_sec) * 1000000ULL + static_cast<hrt_abstime>(ts.tv_nsec) / 1000ULL;
#else
	return hrt_absolute_time();
#endif
}

float constrainFloat(float value, float lower, float upper)
{
	if (value < lower) {
		return lower;
	}

	if (value > upper) {
		return upper;
	}

	return value;
}

bool isFinite(float value)
{
	return std::isfinite(value);
}

template<int Rows, int Cols>
bool runLPCA(LPCAMethod method, const float b_par[ControlAllocation::NUM_AXES][ControlAllocation::NUM_ACTUATORS],
			  const float y_par[ControlAllocation::NUM_AXES],
			  const float y_higher_par[ControlAllocation::NUM_AXES],
			  const float y_lower_par[ControlAllocation::NUM_AXES],
			  bool priority_split_valid,
			  const float actuator_min[ControlAllocation::NUM_ACTUATORS],
			  const float actuator_max[ControlAllocation::NUM_ACTUATORS],
			  float output[ControlAllocation::NUM_ACTUATORS],
			  LPCADiagnostics &diagnostics)
{
	float b[Rows][Cols];
	float input[Rows];
	float input_higher[Rows];
	float input_lower[Rows];
	float lower[Cols];
	float upper[Cols];
	const bool use_priority = method == LPCAMethod::Priority;

	for (int i = 0; i < Rows; ++i) {
		input[i] = y_par[i];

		if (use_priority) {
			input_lower[i] = priority_split_valid ? y_lower_par[i] : input[i];
			input_higher[i] = priority_split_valid ? y_higher_par[i] : 0.f;
		}

		memcpy(b[i], b_par[i], Cols * sizeof(b[i][0]));
	}

	memcpy(lower, actuator_min, Cols * sizeof(lower[0]));
	memcpy(upper, actuator_max, Cols * sizeof(upper[0]));

	const hrt_abstime core_start = runtimeMeasurementTimeUs();
	diagnostics.prepare_time_us = static_cast<float>(core_start - diagnostics.prepare_start);

	Aircraft<Rows, Cols> aircraft(b, upper, lower);
	DP_LP_ControlAllocator<Rows, Cols> allocator(aircraft);
	float raw[Cols];
	float restored[Cols];
	int err = 0;
	float rho = 0.f;

	if (method == LPCAMethod::Dir) {
		allocator.DP_LPCA(input, raw, err, rho);

	} else if (method == LPCAMethod::DPScaled) {
		allocator.DPscaled_LPCA(input, raw, err, rho);

	} else {
		allocator.DP_LPCA_prio(input_higher, input_lower, raw, err, rho);
	}

	diagnostics.err = err;
	diagnostics.rho = rho;

	allocator.restoring(raw, restored);

	const hrt_abstime post_start = runtimeMeasurementTimeUs();
	diagnostics.core_time_us = static_cast<float>(post_start - core_start);

	for (int i = 0; i < Cols; ++i) {
		if (!isFinite(restored[i])) {
			diagnostics.post_time_us = static_cast<float>(runtimeMeasurementTimeUs() - post_start);
			return false;
		}

		output[i] = constrainFloat(restored[i], lower[i], upper[i]);
	}

	diagnostics.post_time_us = static_cast<float>(runtimeMeasurementTimeUs() - post_start);
	return err == 0;
}

template<int Rows>
bool dispatchLPCAColumns(int cols, LPCAMethod method,
				      const float b_par[ControlAllocation::NUM_AXES][ControlAllocation::NUM_ACTUATORS],
				      const float y_par[ControlAllocation::NUM_AXES],
				      const float y_higher_par[ControlAllocation::NUM_AXES],
				      const float y_lower_par[ControlAllocation::NUM_AXES],
				      bool priority_split_valid,
				      const float actuator_min[ControlAllocation::NUM_ACTUATORS],
				      const float actuator_max[ControlAllocation::NUM_ACTUATORS],
				      float output[ControlAllocation::NUM_ACTUATORS],
			      LPCADiagnostics &diagnostics)
{
	switch (cols) {
	// case 2:
	// 	if constexpr (Rows <= 2) { return runLPCA<Rows, 2>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
	// 	break;

	// case 3:
	// 	if constexpr (Rows <= 3) { return runLPCA<Rows, 3>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
	// 	break;

	case 4:
		if constexpr (Rows <= 4) { return runLPCA<Rows, 4>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
		break;

	case 5:
		if constexpr (Rows <= 5) { return runLPCA<Rows, 5>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
		break;

	case 6:
		if constexpr (Rows <= 6) { return runLPCA<Rows, 6>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
		break;

	case 7:
		if constexpr (Rows <= 7) { return runLPCA<Rows, 7>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
		break;

	case 8:
		if constexpr (Rows <= 8) { return runLPCA<Rows, 8>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
		break;

	case 9:
		if constexpr (Rows <= 9) { return runLPCA<Rows, 9>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
		break;

	// case 10:
	// 	if constexpr (Rows <= 10) { return runLPCA<Rows, 10>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
	// 	break;

	// case 11:
	// 	if constexpr (Rows <= 11) { return runLPCA<Rows, 11>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
	// 	break;

	// case 12:
	// 	if constexpr (Rows <= 12) { return runLPCA<Rows, 12>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
	// 	break;

	// case 13:
	// 	if constexpr (Rows <= 13) { return runLPCA<Rows, 13>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
	// 	break;

	// case 14:
	// 	if constexpr (Rows <= 14) { return runLPCA<Rows, 14>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
	// 	break;

	// case 15:
	// 	if constexpr (Rows <= 15) { return runLPCA<Rows, 15>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
	// 	break;

	// case 16:
	// 	if constexpr (Rows <= 16) { return runLPCA<Rows, 16>(method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output, diagnostics); }
	// 	break;
	}

	return false;
}

bool dispatchLPCA(int rows, int cols, LPCAMethod method,
			       const float b_par[ControlAllocation::NUM_AXES][ControlAllocation::NUM_ACTUATORS],
			       const float y_par[ControlAllocation::NUM_AXES],
			       const float y_higher_par[ControlAllocation::NUM_AXES],
			       const float y_lower_par[ControlAllocation::NUM_AXES],
			       bool priority_split_valid,
			       const float actuator_min[ControlAllocation::NUM_ACTUATORS],
			       const float actuator_max[ControlAllocation::NUM_ACTUATORS],
			       float output[ControlAllocation::NUM_ACTUATORS],
		       LPCADiagnostics &diagnostics)
{
	switch (rows) {
	// case 2:
	// 	return dispatchLPCAColumns<2>(cols, method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output,
	// 						   diagnostics);

	case 3:
		return dispatchLPCAColumns<3>(cols, method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output,
							   diagnostics);

	case 4:
		return dispatchLPCAColumns<4>(cols, method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output,
							   diagnostics);

	// case 5:
	// 	return dispatchLPCAColumns<5>(cols, method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output,
	// 						   diagnostics);

	// case 6:
	// 	return dispatchLPCAColumns<6>(cols, method, b_par, y_par, y_higher_par, y_lower_par, priority_split_valid, actuator_min, actuator_max, output,
	// 						   diagnostics);
	}

	return false;
}

} // namespace

void
ControlAllocationLPCA::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
	bool update_normalization_scale)
{
	ControlAllocation::setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, num_actuators,
			update_normalization_scale);
	_standard_problem_update_needed = true;
	_normalization_needs_update = update_normalization_scale;

	if (_metric_allocation && update_normalization_scale) {
		_normalization_needs_update = false;
	}
}

void
ControlAllocationLPCA::setActuatorMin(const ActuatorVector &actuator_min)
{
	ControlAllocation::setActuatorMin(actuator_min);
	_standard_problem_update_needed = true;
}

void
ControlAllocationLPCA::setActuatorMax(const ActuatorVector &actuator_max)
{
	ControlAllocation::setActuatorMax(actuator_max);
	_standard_problem_update_needed = true;
}

void
ControlAllocationLPCA::allocate()
{
	updateStandardProblem();

	_prev_actuator_sp = _actuator_sp;

	ActuatorVector actuator_delta;
	actuator_delta.setZero();

	_diagnostics = {};
	_diagnostics.active_rows = static_cast<uint8_t>(_num_active_rows);
	_diagnostics.full_row_rank = _full_row_rank;

	for (int i = 0; i < _num_active_rows; ++i) {
		_diagnostics.active_axes_mask |= static_cast<uint8_t>(1u << _active_rows[i]);
	}

	bool allocated = false;
	_used_fallback = false;

	if (_method == Method::Inv) {
		allocated = allocateInv(actuator_delta);

	} else if (_lpca_unavailable_reason != 0) {
		_diagnostics.solver_status = -1;
		_diagnostics.solver_err = _lpca_unavailable_reason;
		_used_fallback = true;
		allocated = allocateInv(actuator_delta);

	} else {
		const bool priority_split_valid = _method == Method::PCA && _control_sp_priority_split_valid;
		_diagnostics.priority_split_valid = priority_split_valid;
		_lpca_nonzero_command = false;

		for (int i = 0; i < _num_active_rows; ++i) {
			const int axis = _active_rows[i];
			_y_par[i] = _control_sp(axis) - _control_trim(axis);
			_lpca_nonzero_command = _lpca_nonzero_command || fabsf(_y_par[i]) > kMatrixZeroTolerance;

			if (priority_split_valid) {
				_y_higher_par[i] = _control_sp_priority_higher(axis);
				_y_lower_par[i] = _control_sp_priority_lower(axis) - _control_trim(axis);
			}
		}

		allocated = allocateLPCA(actuator_delta);

		if (!allocated) {
			_used_fallback = true;
			allocated = allocateInv(actuator_delta);
		}
	}

	if (!allocated) {
		actuator_delta.setZero();
	}

	_actuator_sp = _actuator_trim + actuator_delta;
	clipActuatorSetpoint();
}

void
ControlAllocationLPCA::updateStandardProblem()
{
	if (!_standard_problem_update_needed) {
		return;
	}

	MixMatrix mix_raw;
	matrix::geninv(_effectiveness, mix_raw);

	if (!_metric_allocation) {
		if (_normalization_needs_update && !_had_actuator_failure) {
			updateControlAllocationMatrixScale(mix_raw);
			_normalization_needs_update = false;
		}
	}

	buildUnitEffectiveness();
	buildActiveRows();
	buildActuatorDeltaLimits();
	_full_row_rank = hasFullRowRank();
	_lpca_unavailable_reason = lpcaUnavailableReason();

	matrix::geninv(_effectiveness_unit, _mix_inv);

	_standard_problem_update_needed = false;
}

void
ControlAllocationLPCA::updateControlAllocationMatrixScale(const MixMatrix &mix)
{
	if (_normalize_rpy) {
		int num_non_zero_roll_torque = 0;
		int num_non_zero_pitch_torque = 0;

		for (int i = 0; i < _num_actuators; i++) {
			if (fabsf(mix(i, ROLL)) > 1e-3f) {
				++num_non_zero_roll_torque;
			}

			if (fabsf(mix(i, PITCH)) > 1e-3f) {
				++num_non_zero_pitch_torque;
			}
		}

		float roll_norm_scale = 1.f;

		if (num_non_zero_roll_torque > 0) {
			roll_norm_scale = sqrtf(mix.col(ROLL).norm_squared() / (num_non_zero_roll_torque / 2.f));
		}

		float pitch_norm_scale = 1.f;

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

		for (int i = 0; i < _num_actuators; i++) {
			float norm = fabsf(mix(i, 3 + axis_idx));
			norm_sum += norm;

			if (norm > FLT_EPSILON) {
				++num_non_zero_thrust;
			}
		}

		if (num_non_zero_thrust > 0) {
			_control_allocation_scale(3 + axis_idx) = norm_sum / num_non_zero_thrust;

		} else {
			_control_allocation_scale(3 + axis_idx) = _control_allocation_scale(THRUST_Z);
		}
	}
}

void
ControlAllocationLPCA::buildUnitEffectiveness()
{
	_effectiveness_unit.setZero();

	for (int axis = 0; axis < NUM_AXES; ++axis) {
		for (int actuator = 0; actuator < _num_actuators; ++actuator) {
			_effectiveness_unit(axis, actuator) = _metric_allocation ? _effectiveness(axis, actuator) :
							       _control_allocation_scale(axis) * _effectiveness(axis, actuator);
		}
	}
}

void
ControlAllocationLPCA::buildActiveRows()
{
	_num_active_rows = 0;
	memset(_active_rows, 0, sizeof(_active_rows));
	memset(_b_par, 0, sizeof(_b_par));

	for (int axis = 0; axis < NUM_AXES; ++axis) {
		bool active = false;

		for (int actuator = 0; actuator < _num_actuators; ++actuator) {
			if (fabsf(_effectiveness_unit(axis, actuator)) > kMatrixZeroTolerance) {
				active = true;
				break;
			}
		}

		if (!active) {
			continue;
		}

		_active_rows[_num_active_rows] = axis;

		for (int actuator = 0; actuator < _num_actuators; ++actuator) {
			_b_par[_num_active_rows][actuator] = _effectiveness_unit(axis, actuator);
		}

		++_num_active_rows;
	}
}

void
ControlAllocationLPCA::buildActuatorDeltaLimits()
{
	for (int actuator = 0; actuator < NUM_ACTUATORS; ++actuator) {
		_actuator_delta_min[actuator] = 0.f;
		_actuator_delta_max[actuator] = 0.f;
	}

	for (int actuator = 0; actuator < _num_actuators; ++actuator) {
		if (_actuator_max(actuator) >= _actuator_min(actuator)) {
			_actuator_delta_min[actuator] = _actuator_min(actuator) - _actuator_trim(actuator);
			_actuator_delta_max[actuator] = _actuator_max(actuator) - _actuator_trim(actuator);
		}
	}
}

bool
ControlAllocationLPCA::allocateInv(ActuatorVector &actuator_delta)
{
	if (!_used_fallback) {
		_diagnostics.solver_status = 0;
	}

	actuator_delta.setZero();

	for (int actuator = 0; actuator < _num_actuators; ++actuator) {
		for (int row = 0; row < _num_active_rows; ++row) {
			const int axis = _active_rows[row];
			actuator_delta(actuator) += _mix_inv(actuator, axis) * (_control_sp(axis) - _control_trim(axis));
		}

		if (!isFinite(actuator_delta(actuator))) {
			return false;
		}

		actuator_delta(actuator) = constrainFloat(actuator_delta(actuator), _actuator_delta_min[actuator],
					  _actuator_delta_max[actuator]);
	}

	return true;
}

bool
ControlAllocationLPCA::allocateLPCA(ActuatorVector &actuator_delta)
{
	LPCADiagnostics lpca_diagnostics;
	lpca_diagnostics.prepare_start = runtimeMeasurementTimeUs();

	if (!_lpca_nonzero_command) {
		_diagnostics.solver_status = 0;
		actuator_delta.setZero();
		return true;
	}

	float output[NUM_ACTUATORS];
	LPCAMethod lpca_method = LPCAMethod::Priority;

	if (_method == Method::DPLPCA) {
		lpca_method = LPCAMethod::Dir;

	} else if (_method == Method::DPscaledLPCA) {
		lpca_method = LPCAMethod::DPScaled;
	}

	const bool lpca_ok = dispatchLPCA(_num_active_rows, _num_actuators, lpca_method, _b_par, _y_par, _y_higher_par,
					  _y_lower_par, _diagnostics.priority_split_valid, _actuator_delta_min, _actuator_delta_max, output,
					  lpca_diagnostics);

	if (!lpca_ok) {
		_diagnostics.solver_status = -2;
		_diagnostics.solver_err = static_cast<int8_t>(lpca_diagnostics.err);
		_diagnostics.solver_rho = lpca_diagnostics.rho;
		_diagnostics.solver_prepare_time = lpca_diagnostics.prepare_time_us;
		_diagnostics.solver_core_time = lpca_diagnostics.core_time_us;
		_diagnostics.solver_post_time = lpca_diagnostics.post_time_us;
		return false;
	}

	_diagnostics.solver_status = 1;
	_diagnostics.solver_err = static_cast<int8_t>(lpca_diagnostics.err);
	_diagnostics.solver_rho = lpca_diagnostics.rho;
	_diagnostics.solver_prepare_time = lpca_diagnostics.prepare_time_us;
	_diagnostics.solver_core_time = lpca_diagnostics.core_time_us;
	_diagnostics.solver_post_time = lpca_diagnostics.post_time_us;

	actuator_delta.setZero();

	for (int actuator = 0; actuator < _num_actuators; ++actuator) {
		actuator_delta(actuator) = output[actuator];
	}

	return true;
}

int8_t
ControlAllocationLPCA::lpcaUnavailableReason() const
{
	if (!_full_row_rank) {
		return kLPCAUnavailableRank;
	}

	if (_num_active_rows < 2) {
		return kLPCAUnavailableRows;
	}

	if (_num_actuators < _num_active_rows) {
		return kLPCAUnavailableActuators;
	}

	return 0;
}

bool
ControlAllocationLPCA::hasFullRowRank() const
{
	float b[NUM_AXES][NUM_ACTUATORS] {};

	for (int row = 0; row < _num_active_rows; ++row) {
		for (int col = 0; col < _num_actuators; ++col) {
			b[row][col] = _b_par[row][col];
		}
	}

	return computeRowRank(b, _num_active_rows, _num_actuators) == _num_active_rows;
}

int
ControlAllocationLPCA::computeRowRank(float matrix[NUM_AXES][NUM_ACTUATORS], int rows, int cols)
{
	float max_abs = 0.f;

	for (int row = 0; row < rows; ++row) {
		for (int col = 0; col < cols; ++col) {
			max_abs = fmaxf(max_abs, fabsf(matrix[row][col]));
		}
	}

	const float tolerance = fmaxf(kMatrixZeroTolerance, max_abs * kRankRelativeTolerance);
	int rank = 0;

	for (int col = 0; col < cols && rank < rows; ++col) {
		int pivot_row = rank;
		float pivot_abs = fabsf(matrix[pivot_row][col]);

		for (int row = rank + 1; row < rows; ++row) {
			const float candidate = fabsf(matrix[row][col]);

			if (candidate > pivot_abs) {
				pivot_abs = candidate;
				pivot_row = row;
			}
		}

		if (pivot_abs <= tolerance) {
			continue;
		}

		if (pivot_row != rank) {
			for (int c = col; c < cols; ++c) {
				const float tmp = matrix[rank][c];
				matrix[rank][c] = matrix[pivot_row][c];
				matrix[pivot_row][c] = tmp;
			}
		}

		const float pivot = matrix[rank][col];

		for (int row = 0; row < rows; ++row) {
			if (row == rank) {
				continue;
			}

			const float factor = matrix[row][col] / pivot;

			if (fabsf(factor) <= FLT_EPSILON) {
				continue;
			}

			for (int c = col; c < cols; ++c) {
				matrix[row][c] -= factor * matrix[rank][c];
			}
		}

		++rank;
	}

	return rank;
}

#if defined(__clang__)
# pragma clang diagnostic pop
#elif defined(__GNUC__)
# pragma GCC diagnostic pop
#endif
