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
