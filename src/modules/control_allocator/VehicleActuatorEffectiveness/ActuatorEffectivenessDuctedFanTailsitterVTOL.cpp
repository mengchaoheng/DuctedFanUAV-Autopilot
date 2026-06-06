/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

#include "ActuatorEffectivenessDuctedFanTailsitterVTOL.hpp"

ActuatorEffectivenessDuctedFanTailsitterVTOL::ActuatorEffectivenessDuctedFanTailsitterVTOL(ModuleParams *parent)
	: ActuatorEffectivenessTailsitterVTOL(parent)
{
}

bool
ActuatorEffectivenessDuctedFanTailsitterVTOL::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (!ActuatorEffectivenessTailsitterVTOL::getEffectivenessMatrix(configuration, external_update)) {
		return false;
	}

	updateSurfaceMatrixForFlightPhase(configuration);

	return true;
}

void ActuatorEffectivenessDuctedFanTailsitterVTOL::getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const
{
	normalize[0] = true;
	normalize[1] = true;
}

void
ActuatorEffectivenessDuctedFanTailsitterVTOL::updateSurfaceMatrixForFlightPhase(Configuration &configuration) const
{
	constexpr int surface_matrix_index = 1;
	const int surface_count = configuration.num_actuators_matrix[surface_matrix_index] - _first_control_surface_idx;

	if (surface_count <= 0) {
		return;
	}

	EffectivenessMatrix &surface_matrix = configuration.effectiveness_matrices[surface_matrix_index];
	const bool hover_phase = getFlightPhase() == FlightPhase::HOVER_FLIGHT;

	if (hover_phase) {
		// The wing elevons are not in the propeller slipstream hover model, so do not use them in hover allocation.
		for (int actuator = 6; actuator < 8 && actuator < surface_count; ++actuator) {
			for (int axis = 0; axis < NUM_AXES; ++axis) {
				surface_matrix(axis, _first_control_surface_idx + actuator) = 0.f;
			}
		}

	} else {
		// The duct tail surfaces see a higher dynamic pressure in transition and forward flight.
		const float forward_surface_gain = _param_df_fw_cs_gain.get();

		for (int actuator = 0; actuator < 6 && actuator < surface_count; ++actuator) {
			for (int axis = 0; axis < 3; ++axis) {
				surface_matrix(axis, _first_control_surface_idx + actuator) *= forward_surface_gain;
			}
		}
	}
}
