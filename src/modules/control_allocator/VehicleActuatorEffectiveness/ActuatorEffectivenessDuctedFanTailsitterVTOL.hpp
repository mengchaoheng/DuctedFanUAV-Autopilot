/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

#pragma once

#include "ActuatorEffectivenessTailsitterVTOL.hpp"

class ActuatorEffectivenessDuctedFanTailsitterVTOL : public ActuatorEffectivenessTailsitterVTOL
{
public:
	explicit ActuatorEffectivenessDuctedFanTailsitterVTOL(ModuleParams *parent);
	~ActuatorEffectivenessDuctedFanTailsitterVTOL() override = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;
	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override;
	bool effectivenessDependsOnFlightPhase() const override { return true; }

	const char *name() const override { return "Ducted Fan Tailsitter VTOL"; }

private:
	void updateSurfaceMatrixForFlightPhase(Configuration &configuration) const;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::DF_FW_CS_GAIN>) _param_df_fw_cs_gain
	)
};
