/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

/**
 * @file IndiControl.cpp
 */

#include "IndiControl.hpp"

#include <px4_platform_common/defines.h>

using namespace matrix;

void IndiControl::setParams(const Vector3f &P, const Vector3f &inertia)
{
	_gain_p = P;
	_inertia = inertia;
}

bool IndiControl::paramsValid() const
{
	return _gain_p.isAllFinite() && _inertia.isAllFinite()
	       && _inertia(0) > FLT_EPSILON && _inertia(1) > FLT_EPSILON && _inertia(2) > FLT_EPSILON;
}

IndiControl::Output IndiControl::update(const Vector3f &rate, const Vector3f &rate_sp,
		const Vector3f &angular_accel, const Vector3f &allocated_torque) const
{
	const Vector3f rate_error = rate_sp - rate;
	const Vector3f angular_accel_sp = _gain_p.emult(rate_error);

	// tau_c = tau_0 + J * (alpha_c - alpha_0)
	Output output;
	output.rate_error_torque = _inertia.emult(angular_accel_sp);
	output.feedback_torque = allocated_torque - _inertia.emult(angular_accel);
	return output;
}
