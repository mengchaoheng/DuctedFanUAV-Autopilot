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
	struct Output {
		matrix::Vector3f rate_error_torque;
		matrix::Vector3f feedback_torque;
	};

	IndiControl() = default;
	~IndiControl() = default;

	void setParams(const matrix::Vector3f &P, const matrix::Vector3f &inertia);
	bool paramsValid() const;

	Output update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
		      const matrix::Vector3f &angular_accel, const matrix::Vector3f &allocated_torque) const;

private:
	matrix::Vector3f _gain_p;
	matrix::Vector3f _inertia;
};
