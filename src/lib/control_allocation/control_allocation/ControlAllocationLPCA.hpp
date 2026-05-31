/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

/**
 * @file ControlAllocationLPCA.hpp
 *
 * PX4 adapter for normalized INV and LPCA control allocation algorithms.
 */

#pragma once

#include "ControlAllocation.hpp"

class ControlAllocationLPCA : public ControlAllocation
{
public:
	enum class Method {
		Inv,
		DPLPCA,
		DPscaledLPCA,
		PCA,
	};

	explicit ControlAllocationLPCA(Method method) : _method(method) {}
	~ControlAllocationLPCA() override = default;

	void allocate() override;
	void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
				    const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
				    bool update_normalization_scale) override;
	bool usedFallback() const override { return _used_fallback; }

private:
	using EffectivenessMatrix = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>;

	void updateStandardProblem();
	void buildUnitEffectiveness();
	void buildActiveRows();
	void buildActuatorDeltaLimits();

	bool allocateInv(ActuatorVector &actuator_delta) const;
	bool allocateLPCA(ActuatorVector &actuator_delta) const;
	bool hasFullRowRank() const;

	static int computeRowRank(float matrix[NUM_AXES][NUM_ACTUATORS], int rows, int cols);

	Method _method;
	EffectivenessMatrix _effectiveness_unit;

	float _b_par[NUM_AXES][NUM_ACTUATORS] {};
	float _y_par[NUM_AXES] {};
	float _actuator_delta_min[NUM_ACTUATORS] {};
	float _actuator_delta_max[NUM_ACTUATORS] {};
	int _active_rows[NUM_AXES] {};
	int _num_active_rows{0};

	bool _standard_problem_update_needed{false};
	bool _normalization_needs_update{false};
	bool _full_row_rank{false};
	bool _used_fallback{false};
	mutable bool _debug_inv_printed{false};
	mutable bool _debug_lpca_entry_printed{false};
	mutable bool _debug_lpca_output_printed{false};
	mutable bool _debug_lpca_fallback_printed{false};
	mutable bool _debug_lpca_solver_fail_printed{false};
	mutable bool _debug_lpca_solver_error_accepted_printed{false};
};

class ControlAllocationInv : public ControlAllocationLPCA
{
public:
	ControlAllocationInv() : ControlAllocationLPCA(Method::Inv) {}
};

class ControlAllocationDPLPCA : public ControlAllocationLPCA
{
public:
	ControlAllocationDPLPCA() : ControlAllocationLPCA(Method::DPLPCA) {}
};

class ControlAllocationDPscaledLPCA : public ControlAllocationLPCA
{
public:
	ControlAllocationDPscaledLPCA() : ControlAllocationLPCA(Method::DPscaledLPCA) {}
};

class ControlAllocationPCA : public ControlAllocationLPCA
{
public:
	ControlAllocationPCA() : ControlAllocationLPCA(Method::PCA) {}
};
