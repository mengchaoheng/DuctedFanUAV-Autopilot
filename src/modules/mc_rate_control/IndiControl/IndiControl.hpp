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
 * @file IndiControl.hpp
 *
 * PID 3 axis angular rate / angular velocity control.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/filter/LowPassFilter2pVector3f.hpp>

#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/actuator_outputs_value.h>
#include <uORB/topics/actuator_outputs.h>

class IndiControl
{
public:
	IndiControl() = default;
	~IndiControl() = default;

	/**
	 * Set the rate control gains
	 * @param P 3D vector of proportional gains for body x,y,z axis
	 * @param k_cv
	 * @param k_v
	 */
	void setParams(const matrix::Vector3f &P, const float k_cv, const float k_v);

	void init();

	void updateOmega(float Omega_0, float Omega_d);

	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt desired vehicle angular rate setpoint
	 * @param actuator_outputs current commander value of actuators
	 * @param actuator_outputs_value current value of actuators
	 * @param Nu_i second term of virtual control
	 * @return [-1,1] normalized torque vector to apply to the vehicle //This is not a value between -1 and +1
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp, const matrix::Vector3f &angular_accel,
			     const float dt, const actuator_outputs_s &actuator_outputs,
			     const actuator_outputs_value_s &actuator_outputs_value, matrix::Vector3f &Nu_i, const bool landed);

private:

	// Gains
	matrix::Vector3f _gain_p; ///< rate control proportional gain for all axes x, y, z
	//I of prop
	float _I_prop{0.000029f};
	float _I_x{0.025504f};
	float _I_y{0.025504f};
	float _I_z{0.00562f};
	matrix::Matrix<float, 3, 3> _I;
	matrix::Matrix<float, 3, 3> _I_inv;

	float _L_1{0.17078793f};
	float _L_2{0.06647954f};
	matrix::Matrix<float, 3, 4> _L;

	float _k_cv{0.015f};
	float _k_v{0.0117f};	//MC_OMEGA_2_WIND

	matrix::Vector3f _H_3{0.f, 0.f, 0.f};

	float _dOmega_d{0.f};
	float _dOmega_0{0.f};

	float _Omega_d_prev{0.f};
	float _Omega_0_prev{0.f};
	bool _first_update{true};
};
