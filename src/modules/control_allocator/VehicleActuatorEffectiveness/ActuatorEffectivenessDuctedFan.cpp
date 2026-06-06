/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

#include "ActuatorEffectivenessDuctedFan.hpp"

using namespace matrix;

ActuatorEffectivenessDuctedFan::ActuatorEffectivenessDuctedFan(ModuleParams *parent)
	: ModuleParams(parent), _motors(this), _torque(this)
{
}

bool
ActuatorEffectivenessDuctedFan::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// The current ducted-fan airframes use independent motor and surface allocation matrices.
	configuration.selected_matrix = 0;
	_motors.enablePropellerTorque(false);
	const bool motors_added_successfully = _motors.addActuators(configuration);
	_motors_mask = _motors.getMotors();

	configuration.selected_matrix = 1;
	const bool torque_added_successfully = _torque.addActuators(configuration);

	return motors_added_successfully && torque_added_successfully;
}

void ActuatorEffectivenessDuctedFan::getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const
{
	normalize[0] = true;
	normalize[1] = true;
}

void ActuatorEffectivenessDuctedFan::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	if (matrix_index == 0) {
		stopMaskedMotorsWithZeroThrust(_motors_mask, actuator_sp);
	}
}
