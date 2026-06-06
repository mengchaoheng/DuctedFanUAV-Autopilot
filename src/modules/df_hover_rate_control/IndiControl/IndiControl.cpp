/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

/**
 * @file IndiControl.cpp
 */

#include "IndiControl.hpp"

#include <px4_platform_common/defines.h>

using namespace matrix;

void IndiControl::setParams(const Vector3f &P)
{
	_gain_p = P;
}

Vector3f IndiControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const allocation_value_s &allocation_value, Vector3f &indi_fb, bool landed, bool use_u,
			     bool use_tau_i)
{
	const Vector3f rate_error = rate_sp - rate;

	if (landed || !use_tau_i) {
		indi_fb.setZero();

	} else {
		Vector3f Bu;
		Bu.setZero();

		if (use_u && allocation_value.feedback_valid) {

			for (unsigned row = 0; row < 3; row++) {
				for (unsigned actuator = 0; actuator < allocation_value.u_dim; actuator++) {
					const float actuator_feedback = allocation_value.u_ultimate_phys[actuator];
					Bu(row) += allocation_value.b[row * allocation_value_s::MAX_U + actuator] *
						   actuator_feedback;
				}
			}
		}

		indi_fb = Bu - angular_accel;
	}

	const Vector3f error_fb = _gain_p.emult(rate_error);
	return error_fb;
}
