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

Vector3f IndiControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const Vector3f &allocated_torque, Vector3f &indi_fb, bool landed, bool use_u, bool use_tau_i)
{
	const Vector3f rate_error = rate_sp - rate;
	const Vector3f angular_accel_sp = _gain_p.emult(rate_error);
	const Vector3f error_fb = _inertia.emult(angular_accel_sp);
	indi_fb.setZero();

	if (!landed) {
		// tau_c = tau_0 + J * (alpha_c - alpha_0). Keep the incremental
		// feedback and commanded-acceleration terms separate for allocator priority.
		if (use_u) {
			indi_fb += allocated_torque;
		}

		if (use_tau_i) {
			indi_fb -= _inertia.emult(angular_accel);
		}
	}

	return error_fb;
}
