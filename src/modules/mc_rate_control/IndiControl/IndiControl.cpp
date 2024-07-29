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
	// _k=k;  // k =_k_cv*_k_v*_k_v

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
	// l1=0.167;l2=0.0698;k=3; % k*delta=F on cs
	// I_x=0.00967;I_y=0.0097;I_z=0.00448;
	// I=diag([I_x;I_y;I_z]);
	// B=I\[-l1 0 l1 0;0 -l1 0 l1;l2 l2 l2 l2]*k;
	// % B=[-l1*k/I_x 0 l1*k/I_x 0;0 -l1*k/I_y 0 l1*k/I_y;l2*k/I_z l2*k/I_z l2*k/I_z l2*k/I_z];
	// % B=I\diag([2*l1;2*l1;4*l2])*k*[-0.5 0 0.5 0;0 -0.5 0 0.5;0.25 0.25 0.25 0.25];
	// % [-0.5 0 0.5 0;0 -0.5 0 0.5;0.25 0.25 0.25 0.25]= piv([-1 0 1;0 -1 1;1 0 1;0 1 1])
	// % I\[2*l1 0 0;0 2*l1 0;0 0 4*l2]*k is the different of gain, that is diag([92.4509;92.1649;186.9643])
	_B.setZero(); //= { {-46.2254,0.0,46.2254,0.0}, {0.0,-46.0825,0.0,46.0825},{46.7411,46.7411,46.7411,46.7411}};
	_B(0, 0)=-_L_1*_k/_I_x; // the same as angular_accel
	_B(0, 2)=_L_1*_k/_I_x;
	_B(1, 1)=-_L_1*_k/_I_y;
	_B(1, 3)=_L_1*_k/_I_y;
	_B(2, 0)=_L_2*_k/_I_z;
	_B(2, 1)=_L_2*_k/_I_z;
	_B(2, 2)=_L_2*_k/_I_z;
	_B(2, 3)=_L_2*_k/_I_z;
	// PX4_INFO("_B");
	// _B.print();

	_H_3(2) = _I_prop/_I_z; // _I_z ???
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
		Nu_i = _B * delta_0 - angular_accel; // estimation of -f(x) and other disturbance d
	}
	Vector3f K =  _gain_p; // by diag([92.4509;92.1649;186.9643]), using the same as PID param
	Vector3f Nu_f= K.emult(rate_error);
	return Nu_f;
}
