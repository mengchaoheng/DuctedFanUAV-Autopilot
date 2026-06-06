/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessControlSurfaces.hpp"

class ActuatorEffectivenessDuctedFan : public ModuleParams, public ActuatorEffectiveness
{
public:
	explicit ActuatorEffectivenessDuctedFan(ModuleParams *parent);
	~ActuatorEffectivenessDuctedFan() override = default;

	// The current ducted-fan airframes use two allocation instances because
	// their force and torque allocation are physically decoupled: motors provide
	// force, and control surfaces provide torque. This also lets PCA consume the
	// existing torque-only INDI priority split on instance 1 while instance 0
	// handles force without a priority split.
	//
	// If a future ducted-fan geometry has motors or surfaces that both produce
	// force and torque, this split is no longer valid. Use a single allocation
	// matrix for that coupled geometry and extend PCA priority semantics first
	// (for example force > INDI torque > rate-error torque).
	int numMatrices() const override { return 2; }

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
			    const ActuatorVector &actuator_max) override;

	const char *name() const override { return "Ducted Fan"; }

private:
	ActuatorEffectivenessRotors _motors;
	ActuatorEffectivenessControlSurfaces _torque;

	uint32_t _motors_mask{};
};
