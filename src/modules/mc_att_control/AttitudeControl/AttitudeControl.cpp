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

#ifndef MODULE_NAME
#define MODULE_NAME "mc_att_control"
#endif

#include <px4_platform_common/log.h>

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
	// Ezra Tal
	// const float yaw_ref = Eulerf(qd).psi();
	// Tal & Karaman (2021) Eq. (22)-(26): compute xi_c = \bar{xi}_c \circ xi_psi
	// const Vector3f i_z{0.f, 0.f, 1.f};
	// const Vector3f minus_bz_c = -qd.dcm_z();
	// const Vector3f minus_bz_c_b = Dcmf(q.inversed()) * minus_bz_c;

	// const float dot_iz = i_z.dot(minus_bz_c_b);
	// const Vector3f cross_iz = i_z.cross(minus_bz_c_b);

	// Quatf bar_xi_c{1.f - dot_iz, -cross_iz(0), -cross_iz(1), -cross_iz(2)};
	// bar_xi_c.normalize();

	// const Vector3f n_psi_ref{sinf(yaw_ref), -cosf(yaw_ref), 0.f};
	// const Quatf q_tilt = q * bar_xi_c; // qd_red
	// const Vector3f bar_n_psi_ref = Dcmf(q_tilt.inversed()) * n_psi_ref;

	// const float n1 = bar_n_psi_ref(0);
	// const float n2 = bar_n_psi_ref(1);
	// const float kappa = (fabsf(n2) > 1e-6f) ? (-n1 / n2) : 0.f;

	// Quatf xi_psi{1.f, 0.f, 0.f, kappa/(1+sqrtf(1.f + kappa * kappa))};
	// xi_psi.normalize();

	// const Quatf xi_c = bar_xi_c * xi_psi; // = qe

	// PX4_INFO("xi_c: [%.3f %.3f %.3f %.3f]", (double)xi_c(0), (double)xi_c(1), (double)xi_c(2), (double)xi_c(3));



	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf q_e_red_I(e_z, e_z_d);
	Quatf qd_red;

	if (fabsf(q_e_red_I(1)) > (1.f - 1e-5f) || fabsf(q_e_red_I(2)) > (1.f - 1e-5f)) {
		// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		// full attitude control anyways generates no yaw input and directly takes the combination of
		// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
		qd_red = qd;

	} else {
		// Transform rotation from current to desired thrust vector into a world frame reduced desired attitude.
		// This is a right multiplication as the tilt error quaternion is obtained from two Z vectors expressed in the world frame.
		Quatf q_e_red = q.inversed() * q_e_red_I * q; // bar_xi_c ?
		qd_red =q * q_e_red; // xi * bar_xi_c = q_tilt = q * bar_xi_c
	}

	// With a full desired attitude given by: qd = qd_red * qd_dyaw, extract the delta yaw component.
	// By definition, the delta yaw quaternion has the form (cos(angle/2), 0, 0, sin(angle/2))
	// const Quatf q_e_red_cmp = q.inversed() * qd_red;

	// PX4_INFO("q_e_red-bar_xi_c: [%.6f %.6f %.6f %.6f]",
	// 	 (double)(q_e_red_cmp(0) - bar_xi_c(0)),
	// 	 (double)(q_e_red_cmp(1) - bar_xi_c(1)),
	// 	 (double)(q_e_red_cmp(2) - bar_xi_c(2)),
	// 	 (double)(q_e_red_cmp(3) - bar_xi_c(3)));
	// PX4_INFO("qd_red-q_tilt: [%.6f %.6f %.6f %.6f]",
	// 	 (double)(qd_red(0) - q_tilt(0)),
	// 	 (double)(qd_red(1) - q_tilt(1)),
	// 	 (double)(qd_red(2) - q_tilt(2)),
	// 	 (double)(qd_red(3) - q_tilt(3)));

	Quatf qd_dyaw = qd_red.inversed() * qd;
	qd_dyaw.canonicalize();

	// const Quatf q_xi_psi_delta = xi_psi.inversed() * qd_dyaw;
	// const float angle_diff_xi_psi_qd_dyaw = 2.f * atan2f(q_xi_psi_delta.imag().norm(), fabsf(q_xi_psi_delta(0)));
	// PX4_INFO("xi_psi-qd_dyaw: [%.6f %.6f %.6f %.6f], angle_diff: %.6f rad",
	// 	 (double)(xi_psi(0) - qd_dyaw(0)),
	// 	 (double)(xi_psi(1) - qd_dyaw(1)),
	// 	 (double)(xi_psi(2) - qd_dyaw(2)),
	// 	 (double)(xi_psi(3) - qd_dyaw(3)),
	// 	 (double)angle_diff_xi_psi_qd_dyaw);


	// catch numerical problems with the domain of acosf and asinf
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	// scale the delta yaw angle and re-combine the desired attitude
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = q.inversed() * qd;

	// qe = q_e_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3)))) = xi_c;

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	const Vector3f eq = 2.f * qe.canonical().imag(); //由于q和-q都是表示同一个姿态。canonical是规范化，使得实部为正, 同时还考虑了实部很小时认为是0（参考文献中sgn(qe_0)=1），然后将寻找第一个不为零的数，据此调整符号（使其为正）, 确保四元数虚部唯一。这时候直接使用虚部即可作姿态误差对应的参考角速度。

	// calculate angular rates setpoint
	Vector3f rate_setpoint = eq.emult(_proportional_gain);

	// Feed forward the yaw setpoint rate.
	// yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
	// but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	// Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	// and multiply it by the yaw setpoint rate (yawspeed_setpoint).
	// This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	// such that it can be added to the rates setpoint.
	if (std::isfinite(_yawspeed_setpoint)) {
		rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
