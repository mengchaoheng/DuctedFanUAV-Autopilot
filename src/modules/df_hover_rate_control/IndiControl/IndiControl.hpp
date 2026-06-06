/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

/**
 * @file IndiControl.hpp
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <uORB/topics/allocation_value.h>

class IndiControl
{
public:
	IndiControl() = default;
	~IndiControl() = default;

	void setParams(const matrix::Vector3f &P);

	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
				const matrix::Vector3f &angular_accel, const allocation_value_s &allocation_value, matrix::Vector3f &indi_fb,
				bool landed, bool use_u, bool use_tau_i);

private:
	matrix::Vector3f _gain_p;
};
