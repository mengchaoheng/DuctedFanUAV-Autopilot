/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

#pragma once

#include "ActuatorEffectivenessCustom.hpp"

class ActuatorEffectivenessDuctedFan : public ActuatorEffectivenessCustom
{
public:
	explicit ActuatorEffectivenessDuctedFan(ModuleParams *parent) : ActuatorEffectivenessCustom(parent) {}
	~ActuatorEffectivenessDuctedFan() override = default;

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		normalize[0] = true;
	}

	const char *name() const override { return "Ducted Fan"; }
};
