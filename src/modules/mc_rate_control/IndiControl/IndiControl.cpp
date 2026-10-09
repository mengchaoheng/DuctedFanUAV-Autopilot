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
	       && PX4_ISFINITE(_hover_wash) && _hover_wash >= 0.f
	       && (_hover_wash <= 0.f || (PX4_ISFINITE(_motor_tau) && _motor_tau >= 0.f
					 && PX4_ISFINITE(_hover_thrust) && _hover_thrust > FLT_EPSILON
					 && PX4_ISFINITE(_torque_cutoff) && _torque_cutoff >= 0.f))
	       && _inertia(0) > FLT_EPSILON && _inertia(1) > FLT_EPSILON && _inertia(2) > FLT_EPSILON;
}

void IndiControl::setVaneParams(float hover_thrust, float hover_wash, float motor_tau, float torque_cutoff)
{
	if (fabsf(hover_thrust - _hover_thrust) > FLT_EPSILON
	    || fabsf(hover_wash - _hover_wash) > FLT_EPSILON
	    || fabsf(motor_tau - _motor_tau) > FLT_EPSILON
	    || fabsf(torque_cutoff - _torque_cutoff) > FLT_EPSILON) {
		_wash_initialized = false;
		_torque_initialized = false;
	}

	_hover_thrust = hover_thrust;
	_hover_wash = hover_wash;
	_motor_tau = motor_tau;
	_torque_cutoff = torque_cutoff;
}

Vector3f IndiControl::filterPhysicalTorque(const Vector3f &input, float dt)
{
	if (!_torque_initialized || dt <= 0.f || dt > .1f || _torque_cutoff <= 0.f) {
		_torque_input1 = _torque_input2 = _torque_output1 = _torque_output2 = input;
		_torque_initialized = true;
		return input;
	}

	const float k = tanf(M_PI_F * fminf(_torque_cutoff * dt, .45f));
	const float norm = 1.f / (1.f + sqrtf(2.f) * k + k * k);
	const float b0 = k * k * norm;
	const float a1 = 2.f * (k * k - 1.f) * norm;
	const float a2 = (1.f - sqrtf(2.f) * k + k * k) * norm;
	const Vector3f output = (input + 2.f * _torque_input1 + _torque_input2) * b0
			       - _torque_output1 * a1 - _torque_output2 * a2;
	_torque_input2 = _torque_input1;
	_torque_input1 = input;
	_torque_output2 = _torque_output1;
	_torque_output1 = output;
	return output;
}

IndiControl::Output IndiControl::update(const Vector3f &rate, const Vector3f &rate_sp,
		const Vector3f &angular_accel, const Vector3f &allocated_torque,
		float thrust, float axial_velocity, float dt)
{
	const Vector3f rate_error = rate_sp - rate;
	const Vector3f angular_accel_sp = _gain_p.emult(rate_error);
	float inverse_effectiveness = 1.f;
	Vector3f physical_torque = allocated_torque;

	if (_hover_wash > 0.f) {
		const float target = _hover_wash * sqrtf(fmaxf(0.f, thrust) / _hover_thrust);

		if (!_wash_initialized) {
			_wash_estimate = target;
			_wash_initialized = true;

		} else {
			const float weight = _motor_tau > 0.f ? -expm1f(-fmaxf(0.f, dt) / _motor_tau) : 1.f;
			_wash_estimate += weight * (target - _wash_estimate);
		}

		// Same powered axial-momentum branch as the DF4/SHC09 plant.
		const float outlet = -0.5f * axial_velocity + hypotf(0.5f * axial_velocity, _wash_estimate);
		const float flow = outlet / _hover_wash;
		inverse_effectiveness = 1.f / fmaxf(.1f, flow * flow);
		physical_torque = filterPhysicalTorque(allocated_torque * (flow * flow), dt);
	}

	// Filter physical G*u before inversion: H[G*u] != G_now*H[u].
	// The disabled-model path retains the original fixed-effectiveness law.
	Output output;
	output.rate_error_torque = _inertia.emult(angular_accel_sp) * inverse_effectiveness;
	output.feedback_torque = (physical_torque - _inertia.emult(angular_accel))
				 * inverse_effectiveness;
	return output;
}
