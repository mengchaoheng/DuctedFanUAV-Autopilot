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
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// compensate for the effect of the yaw weight rescaling the output
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}
}

matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	Quatf qd = _attitude_setpoint_q;

	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		// full attitude control anyways generates no yaw input and directly takes the combination of
		// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
		qd_red = qd;

	} else {
		// transform rotation from current to desired thrust vector into a world frame reduced desired attitude
		qd_red *= q;
	}

	// mix full and reduced desired attitude
	Quatf q_mix = qd_red.inversed() * qd;
	// float temp=acosf(q_mix(0))-asinf(q_mix(3));
	// PX4_INFO("===================");
	// PX4_INFO("acosf:%8.6f\n", (double) acosf(q_mix(0)));
	// PX4_INFO("asinf:%8.6f\n", (double) asinf(q_mix(3)));
	// PX4_INFO("===================");
	// PX4_INFO("temp:%8.6f\n", (double) temp);
	// PX4_INFO("===================");
	Dcmf R_sp{qd};
	Dcmf R{q};
	// const Eulerf euler{R_sp};
	// PX4_INFO("psi:%8.6f\n", (double) euler.psi());
	// PX4_INFO("===================");
	q_mix.canonicalize();

	// catch numerical problems with the domain of acosf and asinf
	q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
	q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
	// qd = qd_red * Quatf(cosf(_yaw_w * acosf(q_mix(0))), 0, 0, sinf(_yaw_w * asinf(q_mix(3))));


	// quaternion attitude control law, qe is rotation from q to qd
	// const Quatf qe = q.inversed() * qd;
	// AxisAngle<float> rot{qe};
	// Vector<float, 3> rot_e = rot.axis() * sin(rot.angle());
	// Vector<float, 3> rot_e_h = 2.0f*rot.axis() * sin(rot.angle()/2.0f); // close to rot_e

	Dcmf R_e = (R.transpose()*R_sp - R_sp.transpose()*R);
	Vector<float, 3> e_R =R_e.vee()/2; //e_R = rot_e
	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	// const Vector3f eq = 2.f * qe.canonical().imag(); // eq =  sign(q(0))*rot_e_h =  sign(q(0)) * 2.0f*rot.axis() * sin(rot.angle()/2.0f)
	// const Vector3f eq = 2.f * qe.imag()*qe(0); //eq=rot_e= q(0)*rot_e_h


	// PX4_INFO("===================");
	// PX4_INFO("qe1:%8.6f\t qe2:%8.6f\t qe3:%8.6f\t", (double) qe(1), (double) qe(2), (double) qe(3));
	// PX4_INFO("===================");
	// PX4_INFO("rot_e_h0:%8.6f\t rot_e_h1:%8.6f\t rot_e_h2:%8.6f\t", (double) rot_e_h(0), (double) rot_e_h(1), (double) rot_e_h(2));
	// PX4_INFO("===================");
	// PX4_INFO("rot_e0:%8.6f\t rot_e1:%8.6f\t rot_e2:%8.6f\t", (double) rot_e(0), (double) rot_e(1), (double) rot_e(2));
	// PX4_INFO("===================");
	// PX4_INFO("e0:%8.6f\t e1:%8.6f\t e2:%8.6f\t", (double) (rot_e(0)-rot_e_h(0)), (double) (rot_e(1)-rot_e_h(1)), (double) (rot_e(2)-rot_e_h(2)));
	// PX4_INFO("===================");
	// PX4_INFO("rot0:%8.6f\t rot1:%8.6f\t rot2:%8.6f\t", (double) rot(0), (double) rot(1), (double) rot(2));
	// PX4_INFO("===================");
	// PX4_INFO("e0:%8.6f\t e1:%8.6f\t e2:%8.6f\t", (double) (rot(0)-rot_e(0)), (double) (rot(1)-rot_e(1)), (double) (rot(2)-rot_e(2)));
	// PX4_INFO("===================");
	// PX4_INFO("e_R0:%8.6f\t e_R1:%8.6f\t e_R2:%8.6f\t", (double) e_R(0), (double) e_R(1), (double) e_R(2));
	// PX4_INFO("===================");
	// PX4_INFO("e0:%8.6f\t e1:%8.6f\t e2:%8.6f\t", (double) (e_R(0)-rot_e(0)), (double) (e_R(1)-rot_e(1)), (double) (e_R(2)-rot_e(2)));
	// PX4_INFO("===================");
	// PX4_INFO("eq0:%8.6f\t eq1:%8.6f\t eq2:%8.6f\t", (double) eq(0), (double) eq(1), (double) eq(2));
	// PX4_INFO("===================");
	// PX4_INFO("e0:%8.6f\t e1:%8.6f\t e2:%8.6f\t", (double) (eq(0)-rot_e_h(0)), (double) (eq(1)-rot_e_h(1)), (double) (eq(2)-rot_e_h(2)));
	// calculate angular rates setpoint
	matrix::Vector3f rate_setpoint = e_R.emult(_proportional_gain);

	// Feed forward the yaw setpoint rate.
	// yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
	// but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	// Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	// and multiply it by the yaw setpoint rate (yawspeed_setpoint).
	// This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	// such that it can be added to the rates setpoint.
	if (is_finite(_yawspeed_setpoint)) {
		rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
