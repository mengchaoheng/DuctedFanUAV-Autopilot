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

	void allocate() override;
	void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
				    const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
				    bool update_normalization_scale) override;
	void setActuatorMin(const ActuatorVector &actuator_min) override;
	void setActuatorMax(const ActuatorVector &actuator_max) override;
	bool usedFallback() const override { return _used_fallback; }
	void setMetricAllocation(bool metric_allocation) { _metric_allocation = metric_allocation; }

private:
	using EffectivenessMatrix = matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS>;
	using MixMatrix = matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES>;

	void updateStandardProblem();
	void updateControlAllocationMatrixScale(const MixMatrix &mix);
	void buildUnitEffectiveness();
	void buildActiveRows();
	void buildActuatorDeltaLimits();

	bool allocateInv(ActuatorVector &actuator_delta);
	bool allocateLPCA(ActuatorVector &actuator_delta);
	int8_t lpcaUnavailableReason() const;
	bool hasFullRowRank() const;

	static int computeRowRank(float matrix[NUM_AXES][NUM_ACTUATORS], int rows, int cols);

	Method _method;
	EffectivenessMatrix _effectiveness_unit;
	MixMatrix _mix_inv;

	float _b_par[NUM_AXES][NUM_ACTUATORS] {};
	float _y_par[NUM_AXES] {};
	float _y_higher_par[NUM_AXES] {};
	float _y_lower_par[NUM_AXES] {};
	float _actuator_delta_min[NUM_ACTUATORS] {};
	float _actuator_delta_max[NUM_ACTUATORS] {};
	int _active_rows[NUM_AXES] {};
	int _num_active_rows{0};
	int8_t _lpca_unavailable_reason{0};

	bool _lpca_nonzero_command{false};
	bool _standard_problem_update_needed{false};
	bool _normalization_needs_update{false};
	bool _full_row_rank{false};
	bool _used_fallback{false};
	bool _metric_allocation{false};
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
