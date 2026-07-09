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

namespace
{

matrix::Dcmf expMapSO3(const matrix::Vector3f &v)
{
	const float theta = v.norm();
	const matrix::Dcmf K(v.hat());

	if (theta < 1e-6f) {
		return matrix::Dcmf(matrix::Dcmf() + K + 0.5f * K * K);
	}

	return matrix::Dcmf(matrix::Dcmf()
			    + (sinf(theta) / theta) * K
			    + ((1.f - cosf(theta)) / (theta * theta)) * K * K);
}

matrix::Vector3f logMapSO3(const matrix::Dcmf &R)
{
	const float cos_phi = math::constrain((R(0, 0) + R(1, 1) + R(2, 2) - 1.f) * 0.5f, -1.f, 1.f);
	const float phi = acosf(cos_phi);

	const matrix::Dcmf R_skew = R - R.transpose();
	const matrix::Vector3f vee = R_skew.vee();

	if (phi < 1e-5f) {
		return 0.5f * vee;
	}

	const float sin_phi = sinf(phi);

	if (fabsf(sin_phi) > 1e-5f) {
		return (phi / (2.f * sin_phi)) * vee;
	}

	matrix::Vector3f axis;

	if (R(0, 0) >= R(1, 1) && R(0, 0) >= R(2, 2)) {
		axis(0) = sqrtf(fmaxf((R(0, 0) + 1.f) * 0.5f, 0.f));

		if (axis(0) > 1e-5f) {
			axis(1) = (R(0, 1) + R(1, 0)) / (4.f * axis(0));
			axis(2) = (R(0, 2) + R(2, 0)) / (4.f * axis(0));

		} else {
			axis = matrix::Vector3f(1.f, 0.f, 0.f);
		}

	} else if (R(1, 1) >= R(2, 2)) {
		axis(1) = sqrtf(fmaxf((R(1, 1) + 1.f) * 0.5f, 0.f));

		if (axis(1) > 1e-5f) {
			axis(0) = (R(0, 1) + R(1, 0)) / (4.f * axis(1));
			axis(2) = (R(1, 2) + R(2, 1)) / (4.f * axis(1));

		} else {
			axis = matrix::Vector3f(0.f, 1.f, 0.f);
		}

	} else {
		axis(2) = sqrtf(fmaxf((R(2, 2) + 1.f) * 0.5f, 0.f));

		if (axis(2) > 1e-5f) {
			axis(0) = (R(0, 2) + R(2, 0)) / (4.f * axis(2));
			axis(1) = (R(1, 2) + R(2, 1)) / (4.f * axis(2));

		} else {
			axis = matrix::Vector3f(0.f, 0.f, 1.f);
		}
	}

	if (axis.norm() > 1e-5f) {
		axis.normalize();
		return phi * axis;
	}

	return matrix::Vector3f(0.f, 0.f, 0.f);
}

} // namespace

void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// compensate for the effect of the yaw weight rescaling the output
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}
}

void AttitudeControl::setAttitudeErrorMode(const int mode)
{
	_attitude_error_mode = math::constrain(mode,
					       static_cast<int>(ATTITUDE_ERROR_DEFAULT),
					       static_cast<int>(ATTITUDE_ERROR_TILT_TORSION));
}

matrix::Vector3f AttitudeControl::calculateAttitudeErrorDefault(const Quatf &q, Quatf qd) const
{
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
		Quatf q_e_red = q.inversed() * q_e_red_I * q; // bar_xi_c
		qd_red =q * q_e_red; // xi * bar_xi_c = q_tilt = q * bar_xi_c
	}

	// With a full desired attitude given by: qd = qd_red * qd_dyaw, extract the delta yaw component.
	// By definition, the delta yaw quaternion has the form (cos(angle/2), 0, 0, sin(angle/2))
	Quatf qd_dyaw = qd_red.inversed() * qd;
	qd_dyaw.canonicalize();
	// catch numerical problems with the domain of acosf and asinf
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	// scale the delta yaw angle and re-combine the desired attitude
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = q.inversed() * qd;

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	const Vector3f eq = 2.f * qe.canonical().imag(); // Since q and -q represent the same orientation. Canonical is a normalization process that ensures the real part is positive, and also considers the real part as zero when it is very small (as referenced by sgn(qe_0)=1). Then, the first non-zero number is found to adjust the sign (making it positive), ensuring the uniqueness of the quaternion's imaginary part. At this point, the imaginary part can be directly used as the reference angular velocity corresponding to the orientation error.

	return eq;
}

matrix::Vector3f AttitudeControl::calculateAttitudeErrorQuaternionImag(const Quatf &q, const Quatf &qd) const
{

	const Quatf qe = q.inversed() * qd;

	return 2.f * qe.canonical().imag();
}

matrix::Vector3f AttitudeControl::calculateAttitudeErrorQuaternionLog(const Quatf &q, const Quatf &qd) const
{
	// Quaternion attitude error, rotation from current attitude q to desired attitude qd.
	const Quatf qe = (q.inversed() * qd).canonical();

	// PX4 AxisAnglef(Quatf) already returns angle-axis vector: angle * axis.
	return Vector3f(AxisAnglef(qe));
}

matrix::Vector3f AttitudeControl::calculateAttitudeErrorDcmLog(const Quatf &q, const Quatf &qd) const
{
	const Dcmf R(q);
	const Dcmf Rd(qd);

	// Rotation error from current attitude to desired attitude, expressed in current body frame.
	const Dcmf Re = R.transpose() * Rd;

	const float cos_theta = math::constrain((Re(0, 0) + Re(1, 1) + Re(2, 2) - 1.f) * 0.5f, -1.f, 1.f);
	const float theta = acosf(cos_theta);

	const Dcmf Re_skew = Re - Re.transpose();
	const Vector3f vee = Re_skew.vee();

	if (theta < 1e-5f) {
		// Log(R) ~= 0.5 * (R - R^T) for small angle.
		return 0.5f * vee;
	}

	const float sin_theta = sinf(theta);

	if (fabsf(sin_theta) > 1e-5f) {
		// Log(R) = theta / (2 sin(theta)) * (R - R^T).
		return (theta / (2.f * sin_theta)) * vee;
	}

	// Near pi, theta / sin(theta) is ill-conditioned.
	// Extract the rotation axis from R = 2 * axis * axis^T - I.
	Vector3f axis;

	if (Re(0, 0) >= Re(1, 1) && Re(0, 0) >= Re(2, 2)) {
		axis(0) = sqrtf(math::max((Re(0, 0) + 1.f) * 0.5f, 0.f));

		if (axis(0) > 1e-5f) {
			axis(1) = (Re(0, 1) + Re(1, 0)) / (4.f * axis(0));
			axis(2) = (Re(0, 2) + Re(2, 0)) / (4.f * axis(0));

		} else {
			axis = Vector3f(1.f, 0.f, 0.f);
		}

	} else if (Re(1, 1) >= Re(2, 2)) {
		axis(1) = sqrtf(math::max((Re(1, 1) + 1.f) * 0.5f, 0.f));

		if (axis(1) > 1e-5f) {
			axis(0) = (Re(0, 1) + Re(1, 0)) / (4.f * axis(1));
			axis(2) = (Re(1, 2) + Re(2, 1)) / (4.f * axis(1));

		} else {
			axis = Vector3f(0.f, 1.f, 0.f);
		}

	} else {
		axis(2) = sqrtf(math::max((Re(2, 2) + 1.f) * 0.5f, 0.f));

		if (axis(2) > 1e-5f) {
			axis(0) = (Re(0, 2) + Re(2, 0)) / (4.f * axis(2));
			axis(1) = (Re(1, 2) + Re(2, 1)) / (4.f * axis(2));

		} else {
			axis = Vector3f(0.f, 0.f, 1.f);
		}
	}

	if (axis.norm() > 1e-5f) {
		axis.normalize();
		return theta * axis;
	}

	return Vector3f(0.f, 0.f, 0.f);
}

matrix::Vector3f AttitudeControl::calculateAttitudeErrorDcmVee(const Quatf &q, const Quatf &qd) const
{
	const Dcmf R(q);
	const Dcmf Rd(qd);

	const Dcmf e_R = R.transpose() * Rd - Rd.transpose() * R;

	return 0.5f * e_R.vee();
}

matrix::Vector3f AttitudeControl::calculateAttitudeErrorEzraTal(const Quatf &q, const Quatf &qd) const
{
	// Tal & Karaman (2021)[4], Eq. (22)-(27)
	const float yaw_ref = Eulerf(qd).psi();
	const Vector3f i_z{0.f, 0.f, 1.f};
	const Vector3f minus_bz_c = -qd.dcm_z();
	const Vector3f minus_bz_c_b = Dcmf(q.inversed()) * minus_bz_c;

	const float dot_iz = math::constrain(i_z.dot(minus_bz_c_b), -1.f, 1.f);
	const Vector3f cross_iz = i_z.cross(minus_bz_c_b);

	Quatf bar_xi_c{1.f - dot_iz, -cross_iz(0), -cross_iz(1), -cross_iz(2)};
	const float bar_xi_c_norm = sqrtf(bar_xi_c(0) * bar_xi_c(0) + bar_xi_c(1) * bar_xi_c(1)
					      + bar_xi_c(2) * bar_xi_c(2) + bar_xi_c(3) * bar_xi_c(3));

	if (bar_xi_c_norm > 1e-6f) {
		bar_xi_c.normalize();

	} else {
		// Singular 180 degree case in Eq. (23): select one valid tilt rotation direction.
		bar_xi_c = Quatf{0.f, 1.f, 0.f, 0.f};
	}

	const Vector3f n_psi_ref{sinf(yaw_ref), -cosf(yaw_ref), 0.f};
	const Quatf q_tilt = q * bar_xi_c;
	const Vector3f bar_n_psi_ref = Dcmf(q_tilt.inversed()) * n_psi_ref;

	const float n1 = bar_n_psi_ref(0);
	const float n2 = bar_n_psi_ref(1);
	const float kappa = (fabsf(n2) > 1e-6f) ? (-n1 / n2) : 0.f;

	Quatf xi_psi{1.f, 0.f, 0.f, kappa / (1.f + sqrtf(1.f + kappa * kappa))}; // fix the error in Tal & Karaman (2021)[4].

	xi_psi.normalize();

	const Quatf xi_c = (bar_xi_c * xi_psi).canonical();
	const Vector3f xi_c_imag = xi_c.imag();
	const float xi_c_imag_norm = xi_c_imag.norm();

	if (xi_c_imag_norm > 1e-6f) {
		const float angle = 2.f * acosf(math::constrain(xi_c(0), -1.f, 1.f));
		return xi_c_imag * (angle / xi_c_imag_norm);
	}

	return Vector3f(0.f, 0.f, 0.f);
}

matrix::Vector3f AttitudeControl::calculateAttitudeErrorTiltPrioritized(const Quatf &q, const Quatf &qd) const
{
	// Brescianini & D'Andrea [6] / Sun et al. [7], adapted to PX4 convention:
	// qe = q^{-1} * qd.
	Quatf qe = q.inversed() * qd;
	qe.normalize();

	const float q0 = qe(0);
	const float q1 = qe(1);
	const float q2 = qe(2);
	const float q3 = qe(3);

	const float q02_q32 = q0 * q0 + q3 * q3;
	const float denom = sqrtf(q02_q32);

	if (denom < 1e-6f) {
		// Reduced-attitude singularity: desired thrust direction is opposite to current thrust direction.
		return 2.f * qe.canonical().imag();
	}

	const Vector3f qe_red{
		(q0 * q1 - q2 * q3) / denom,
		(q0 * q2 + q1 * q3) / denom,
		0.f
	};

	const Vector3f qe_yaw{
		0.f,
		0.f,
		q3 / denom
	};

	const float q0_sign = (q0 >= 0.f) ? 1.f : -1.f;

	// Factor 2 keeps small-angle scaling consistent with PX4's 2 * imag(qe).
	return 2.f * (qe_red + _yaw_w * q0_sign * qe_yaw);
}

matrix::Vector3f AttitudeControl::calculateAttitudeErrorTiltTorsion(const Quatf &q, const Quatf &qd) const
{
	const Dcmf R(q);
	const Dcmf R_sp(qd);

	// PX4 body-frame equivalent of Yu's relative rotation [2].
	// This matches the tested sign convention:
	// Dcmf R_e = R.transpose() * R_sp;
	const Dcmf R_e = R.transpose() * R_sp;

	const Vector3f z_c{0.f, 0.f, 1.f};
	const Vector3f z_t = R_e * z_c;

	const float dot_z = math::constrain(z_c.dot(z_t), -1.f, 1.f);
	Vector3f tilt_rot_axis = z_c.cross(z_t);
	const float tilt_rot_axis_norm = tilt_rot_axis.norm();

	Dcmf R_tilt;

	if (tilt_rot_axis_norm > 1e-6f) {
		const float tilt_angle = atan2f(tilt_rot_axis_norm, dot_z);
		tilt_rot_axis *= tilt_angle / tilt_rot_axis_norm;
		R_tilt = expMapSO3(tilt_rot_axis);

	} else if (dot_z > 0.f) {
		R_tilt = Dcmf();

	} else {
		R_tilt = expMapSO3(Vector3f(acosf(-1.f), 0.f, 0.f));
	}

	// Yu: R_e = R_torsion * R_tilt.
	const Dcmf R_torsion = R_e * R_tilt.transpose();

	const Vector3f e_tilt = logMapSO3(R_tilt);

	// R_torsion should be a rotation around the intermediate/target z-axis.
	// The projection keeps the torsion component clean under numerical error.
	const Vector3f z_intermediate = R_tilt * z_c;
	const Vector3f e_torsion_raw = logMapSO3(R_torsion);
	const Vector3f e_torsion = z_intermediate * e_torsion_raw.dot(z_intermediate);

	return e_tilt + _yaw_w * e_torsion;
}


matrix::Vector3f AttitudeControl::calculateAttitudeError(const Quatf &q, const Quatf &qd) const
{
	// Reference:
	// [1] D. Brescianini, M. Hehn, and R. D’Andrea, “Nonlinear Quadrocopter Attitude Control: Technical Report,” ETH Zurich, 2013. doi: 10.3929/ETHZ-A-009970340.
	// [2] Y. Yu, S. Yang, M. Wang, C. Li, and Z. Li, “High performance full attitude control of a quadrotor on SO (3),” in 2015 IEEE International Conference on Robotics and Automation (ICRA), Seattle, WA, USA: IEEE, 2015, pp. 1698–1703. doi: 10.1109/icra.2015.7139416.
	// [3] T. Lee, M. Leok, and N. H. McClamroch, “Geometric Tracking Control of a Quadrotor UAV on SE(3),” Mar. 10, 2010, arXiv: arXiv:1003.2005. doi: 10.48550/arXiv.1003.2005.
	// [4] E. Tal and S. Karaman, “Accurate Tracking of Aggressive Quadrotor Trajectories Using Incremental Nonlinear Dynamic Inversion and Differential Flatness,” IEEE Trans. Contr. Syst. Technol., vol. 29, no. 3, pp. 1203–1218, May 2021, doi: 10.1109/tcst.2020.3001117.
	// [5] J. Johnson and R. Beard, “Globally-Attractive Logarithmic Geometric Control of a Quadrotor for Aggressive Trajectory Tracking,” Dec. 01, 2021, arXiv: arXiv:2109.07025. doi: 10.48550/arXiv.2109.07025.
	// [6] D. Brescianini and R. D’Andrea, “Tilt-Prioritized Quadrocopter Attitude Control,” IEEE Transactions on Control Systems Technology, vol. 28, no. 2, pp. 376–387, Mar. 2020, doi: 10.1109/TCST.2018.2873224.
	// [7] S. Sun, A. Romero, P. Foehn, E. Kaufmann, and D. Scaramuzza, “A Comparative Study of Nonlinear MPC and Differential-Flatness-Based Control for Quadrotor Agile Flight,” Feb. 23, 2022, arXiv: arXiv:2109.01365. Accessed: May 27, 2022. [Online]. Available: http://arxiv.org/abs/2109.01365
	// [8] J. Sola, “Quaternion kinematics for the error-state Kalman filter,” arXiv:1711.02508 [cs], Nov. 2017, Accessed: Sep. 26, 2020. [Online]. Available: http://arxiv.org/abs/1711.02508
	// [9] J. Sola, J. Deray, and D. Atchuthan, “A micro Lie theory for state estimation in robotics,” Dec. 08, 2021, arXiv: arXiv:1812.01537. doi: 10.48550/arXiv.1812.01537.

	// Error in ref:


	switch (_attitude_error_mode) {
		// The equivalent relationship between the different attitude error representations can be found in the references [8][9].
	case ATTITUDE_ERROR_QUATERNION_IMAG:
		return calculateAttitudeErrorQuaternionImag(q, qd); // Equivalent: 2*n*sin(theta/2). It can be found in [1], here is the original version without separate handling of yaw.

	case ATTITUDE_ERROR_QUATERNION_LOG:
		return calculateAttitudeErrorQuaternionLog(q, qd); // Equivalent: n*theta. more robustly than the DCM log map.

	case ATTITUDE_ERROR_DCM_LOG:
		return calculateAttitudeErrorDcmLog(q, qd); // Equivalent: n*theta. It can be found in [5], which handles the situation theta = pi.

	case ATTITUDE_ERROR_DCM_VEE:
		return calculateAttitudeErrorDcmVee(q, qd); // Equivalent: n*sin(theta). It can be found in [3], is equivalent to 2*q_0*q_v in quaternion version.

	case ATTITUDE_ERROR_EZRA_TAL:
		return calculateAttitudeErrorEzraTal(q, qd); // Equivalent: n*theta. It can be found in [4], where there are some singularities.

	case ATTITUDE_ERROR_TILT_PRIORITIZED:
		return calculateAttitudeErrorTiltPrioritized(q, qd); // Equivalent: 2*n*sin(theta/2). It can be found in [6][7].

	case ATTITUDE_ERROR_TILT_TORSION:
		return calculateAttitudeErrorTiltTorsion(q, qd); // Equivalent: n*theta. It can be found in [2].

	case ATTITUDE_ERROR_DEFAULT:
	default:
		return calculateAttitudeErrorDefault(q, qd); // Equivalent: 2*n*sin(theta/2). It can be found in [1], here is the version with separate handling of yaw.
	}
}

matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	const Quatf qd = _attitude_setpoint_q;
	const Vector3f eq = calculateAttitudeError(q, qd);

	// calculate angular rates setpoint
	Vector3f rate_setpoint = eq.emult(_proportional_gain);

	// Feed forward the yaw setpoint rate.
	// yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
	// but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	// Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transpose (== q.inversed)
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
