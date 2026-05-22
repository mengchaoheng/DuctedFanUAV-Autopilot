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

void IndiControl::setParams(const Vector3f &P, const float k)
{
	_gain_p = P;
	_k=k;  // k_omega2force =_k_cv*_k_v*_k_v
	_B.setZero();

	_B(0, 0)= -_L_1*_k/_I_x;
	_B(0, 1)= -0.5f*_L_1*_k/_I_x;
	_B(0, 2)= 0.5f*_L_1*_k/_I_x;
	_B(0, 3)= _L_1*_k/_I_x;
	_B(0, 4)= 0.5f*_L_1*_k/_I_x;
	_B(0, 5)= -0.5f*_L_1*_k/_I_x;

	_B(1, 1)= sqrt(3)*0.5*_L_1*_k/_I_y;
	_B(1, 2)= sqrt(3)*0.5*_L_1*_k/_I_y;
	_B(1, 4)= -sqrt(3)*0.5*_L_1*_k/_I_y;
	_B(1, 5)= -sqrt(3)*0.5*_L_1*_k/_I_y;

	_B(2, 0)= _L_2*_k/_I_z;
	_B(2, 1)= _L_2*_k/_I_z;
	_B(2, 2)= _L_2*_k/_I_z;
	_B(2, 3)= _L_2*_k/_I_z;
	_B(2, 4)= _L_2*_k/_I_z;
	_B(2, 5)= _L_2*_k/_I_z;
	// PX4_INFO("INDI is updated");

}

void IndiControl::init()
{
	// l1 = 0.292166;
	// l2 = 0.073699;
	// k_omega2force = 1.93;
	// I_x = 0.0438;
	// I_y = 0.0436;
	// I_z = 0.005006;
	// d = 60*pi/180;
	// I = diag([I_x, I_y, I_z]);
	// B = I \ [-l1, -l1*cos(d),  l1*cos(d),  l1,  l1*cos(d), -l1*cos(d);
        //       0,   l1*sin(d),  l1*sin(d),  0,  -l1*sin(d), -l1*sin(d);
        //       l2,  l2,         l2,         l2,  l2,         l2] * k_omega2force;

	_B.setZero();
	_B(0, 0)= -_L_1*_k/_I_x;
	_B(0, 1)= -0.5f*_L_1*_k/_I_x;
	_B(0, 2)= 0.5f*_L_1*_k/_I_x;
	_B(0, 3)= _L_1*_k/_I_x;
	_B(0, 4)= 0.5f*_L_1*_k/_I_x;
	_B(0, 5)= -0.5f*_L_1*_k/_I_x;

	_B(1, 1)= sqrt(3)*0.5*_L_1*_k/_I_y;
	_B(1, 2)= sqrt(3)*0.5*_L_1*_k/_I_y;
	_B(1, 4)= -sqrt(3)*0.5*_L_1*_k/_I_y;
	_B(1, 5)= -sqrt(3)*0.5*_L_1*_k/_I_y;

	_B(2, 0)= _L_2*_k/_I_z;
	_B(2, 1)= _L_2*_k/_I_z;
	_B(2, 2)= _L_2*_k/_I_z;
	_B(2, 3)= _L_2*_k/_I_z;
	_B(2, 4)= _L_2*_k/_I_z;
	_B(2, 5)= _L_2*_k/_I_z;
	// PX4_INFO("_B");
	// _B.print();
}

Vector3f IndiControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const actuator_outputs_value_s &actuator_outputs_value, Vector3f &Nu_i, const bool landed, bool use_u, bool use_tau_i)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	if (landed || !use_tau_i) {
		Nu_i.setZero();
		// PX4_INFO("Nu_i of INDI is zero");
	}
	else
	{
		Matrix<float, 6, 1> delta_0 (actuator_outputs_value.delta);
		if(use_u) {
			Nu_i = _B * delta_0 - angular_accel;
		}
		else {
			Nu_i =  - angular_accel;
		}

	}
	Vector3f K =  _gain_p; // by diag([92.4509;92.1649;186.9643]), using the same as PID param
	Vector3f Nu_f= K.emult(rate_error);
	return Nu_f;
}
