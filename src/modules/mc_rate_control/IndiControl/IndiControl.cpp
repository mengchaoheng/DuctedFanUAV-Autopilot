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
	_k = k;
}

void IndiControl::init()
{
}

Vector3f IndiControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const allocation_value_s &allocation_value, Vector3f &Nu_i, const bool landed, bool use_u, bool use_tau_i)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	if (landed || !use_tau_i) {
		Nu_i.setZero();
		// PX4_INFO("Nu_i of INDI is zero");
	}
	else
	{
		if (use_u && allocation_value.y_dim >= 3 && allocation_value.u_dim > 0 &&
		    allocation_value.u_dim <= allocation_value_s::MAX_U) {
			Vector3f Bu;
			Bu.setZero();

			for (unsigned row = 0; row < 3; row++) {
				for (unsigned actuator = 0; actuator < allocation_value.u_dim; actuator++) {
					Bu(row) += allocation_value.b[row * allocation_value_s::MAX_U + actuator] *
						   allocation_value.u_ultimate[actuator];
				}
			}

			Nu_i = Bu - angular_accel;
		}
		else {
			Nu_i =  - angular_accel;
		}

	}
	Vector3f K =  _gain_p;
	Vector3f Nu_f= K.emult(rate_error);
	return Nu_f;
}
