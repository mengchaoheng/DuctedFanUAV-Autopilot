/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file IndiControl.cpp
 */

#include <IndiControl.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

void IndiControl::setParams(const Vector3f &P, const float k_cv, const float k_v)
{
	_gain_p = P;
	_k_cv = k_cv;
	_k_v = k_v;//MC_OMEGA_2_WIND
}

void IndiControl::init()
{
	_H_1.setZero();
	_H_1(0, 0) = 0.3491f * _k_cv*_k_v*_k_v*2.f*_L_1/_I_x;
	_H_1(1, 1) = 0.3491f * _k_cv*_k_v*_k_v*2.f*_L_1/_I_y;
	_H_1(2, 2) = 0.3491f * _k_cv*_k_v*_k_v*4.f*_L_2/_I_z;

	_H_inv.setZero();
	_H_inv(0, 0) = 1.f/_H_1(0, 0);
	_H_inv(1, 1) = 1.f/_H_1(1, 1);
	_H_inv(2, 2) = 1.f/_H_1(2, 2);

	_B.setZero();
	_B(0, 0)=-0.5f;
	_B(0, 2)=0.5f;
	_B(1, 1)=-0.5f;
	_B(1, 3)=0.5f;
	_B(2, 0)=0.25f;
	_B(2, 1)=0.25f;
	_B(2, 2)=0.25f;
	_B(2, 3)=0.25f;

	_H_3(2) = _I_prop/_I_z;
}

Vector3f IndiControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const actuator_outputs_value_s &actuator_outputs_value, Vector3f &Nu_i, const bool landed)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	if (landed) {
		Nu_i.setZero();
		// PX4_INFO("Nu_i of INDI is zero");
	}
	else
	{
		Matrix<float, 4, 1> delta_0 (actuator_outputs_value.delta);

		Vector3f H_2{-_I_prop*rate(2)/_I_x, _I_prop*rate(1)/_I_y, 0.f};

		Vector3f T = (2.f * _H_1 * _B * delta_0 + H_2) * (actuator_outputs_value.propeller_omega_d - actuator_outputs_value.propeller_omega_0) + _H_3*(actuator_outputs_value.dpropeller_omega_d - actuator_outputs_value.dpropeller_omega_0);

		Nu_i = _B * delta_0 - _H_inv / (actuator_outputs_value.propeller_omega_0 * actuator_outputs_value.propeller_omega_0) * (angular_accel + T);
		// Nu_i(0) = math::constrain(Nu_i(0), -1.f, 1.f);
		// Nu_i(1) = math::constrain(Nu_i(1), -1.f, 1.f);
		// Nu_i(2) = math::constrain(Nu_i(2), -1.f, 1.f);
		// PX4_INFO("Nu_i of INDI is: roll: %f, pitch: %f, yaw: %f \n", (double) Nu_i(0), (double) Nu_i(1), (double) Nu_i(2));
		// PX4_INFO("T: roll: %f, pitch: %f, yaw: %f \n", (double) T(0), (double) T(1), (double) T(2));
		// PX4_INFO("angular_accel+T: roll: %f, pitch: %f, yaw: %f \n", (double) (angular_accel(0)+T(0)), (double) (angular_accel(1)+T(1)), (double) (angular_accel(2)+T(2)));
	}
	Vector3f K = _H_inv / (actuator_outputs_value.propeller_omega_0 * actuator_outputs_value.propeller_omega_0) * _gain_p;
	// PX4_INFO("propeller_omega_0: %f", (double) actuator_outputs_value.propeller_omega_0);
	// PX4_INFO("_H_inv: roll: %f, pitch: %f, yaw: %f \n", (double) _H_inv(0, 0), (double) _H_inv(1, 1), (double) _H_inv(2, 2));
	// 18617.958984, pitch: 18682.771484, yaw: 8847.666992
	// K(0) = PX4_ISFINITE(K(0)) ? K(0) : 0.0f;
	// K(1) = PX4_ISFINITE(K(1)) ? K(1) : 0.0f;
	// K(2) = PX4_ISFINITE(K(2)) ? K(2) : 0.0f;
	Vector3f Nu_f= K.emult(rate_error);
	// PX4_INFO("K of INDI is: roll: %f, pitch: %f, yaw: %f \n", (double) K(0), (double) K(1), (double) K(2));
	// Nu_f(0) = math::constrain(Nu_f(0), -1.f, 1.f);
	// Nu_f(1) = math::constrain(Nu_f(1), -1.f, 1.f);
	// Nu_f(2) = math::constrain(Nu_f(2), -1.f, 1.f);
	return Nu_f;
}
