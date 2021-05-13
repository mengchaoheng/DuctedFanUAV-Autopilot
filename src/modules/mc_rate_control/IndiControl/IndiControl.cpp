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
	_I.setZero();
	_I(0, 0) = _I_x;
	_I(1, 1) = _I_y;
	_I(2, 2) = _I_z;
	_I_inv.setZero();
	_I_inv(0, 0) = 1/_I_x;
	_I_inv(1, 1) = 1/_I_y;
	_I_inv(2, 2) = 1/_I_z;


	_L.setZero();
	_L(0, 0)=-_L_1;
	_L(0, 2)=_L_1;
	_L(1, 1)=-_L_1;
	_L(1, 3)=_L_1;
	_L(2, 0)=_L_2;
	_L(2, 1)=_L_2;
	_L(2, 2)=_L_2;
	_L(2, 3)=_L_2;

	_H_3(2) = _I_prop/_I_z;
}

Vector3f IndiControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const actuator_outputs_value_s &actuator_outputs_value, Vector3f &Nu_i, const bool landed)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	if (landed) {
		Nu_i.setZero();
		// PX4_INFO("now is INDI");
	}
	else
	{
		Matrix<float, 4, 1> delta_0 (actuator_outputs_value.delta);

		Vector3f H_2{-_I_prop*rate(2)/_I_x, _I_prop*rate(1)/_I_y, 0.f};

		Matrix<float, 3, 4> Bdelta_0=_L * _k_cv*_k_v*_k_v*actuator_outputs_value.propeller_omega_0*actuator_outputs_value.propeller_omega_0;
		Vector3f T = (_I_inv * (_L * 2*_k_cv*_k_v*_k_v*actuator_outputs_value.propeller_omega_0) * delta_0 + H_2) * (actuator_outputs_value.propeller_omega_d-actuator_outputs_value.propeller_omega_0) + _H_3*(actuator_outputs_value.dpropeller_omega_d-actuator_outputs_value.dpropeller_omega_0);

		Nu_i=Bdelta_0*delta_0 - _I*(angular_accel+T);
	}

	Vector3f Nu_f=_I*_gain_p.emult(rate_error);

	return Nu_f+Nu_i;
}
