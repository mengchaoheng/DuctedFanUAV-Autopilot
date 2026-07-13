/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

/**
 * @file IndiControl.hpp
 */

#pragma once

#include <matrix/matrix/math.hpp>

class IndiControl
{
public:
	IndiControl() = default;
	~IndiControl() = default;

	void setParams(const matrix::Vector3f &P, const matrix::Vector3f &inertia);
	bool paramsValid() const;

	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				const matrix::Vector3f &angular_accel, const matrix::Vector3f &allocated_torque,
				matrix::Vector3f &indi_fb, bool landed, bool use_u, bool use_tau_i);

private:
	matrix::Vector3f _gain_p;
	matrix::Vector3f _inertia;
};
