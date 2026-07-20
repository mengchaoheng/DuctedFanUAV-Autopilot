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

static __attribute__((noinline)) Quatf qmul(const Quatf &a, const Quatf &b) { return a * b; }
static __attribute__((noinline)) Quatf qinv(const Quatf &q) { return q.inversed(); }
static __attribute__((noinline)) Vector3f qzaxis(const Quatf &q) { return q.dcm_z(); }

namespace
{
static constexpr bool kDebugAttitudeErrorMode = false; // Set to true to print the attitude error mode when it changes.

void debugPrintAttitudeErrorModeIfChanged(const int mode, const char *branch)
{
	if (!kDebugAttitudeErrorMode) {
		return;
	}

	static int last_mode = -999;
	static const char *last_branch = nullptr;

	if (mode != last_mode || branch != last_branch) {
		PX4_INFO("MC_ATT_ERR_MODE=%d -> %s", mode, branch);

		last_mode = mode;
		last_branch = branch;
	}
}

/*
 * SO(3) helper references used in this file:
 *
 * [R1] Blanco Claraco, "A tutorial on SE(3) transformation parameterizations
 *      and on-manifold optimization", Sec. 9.4.1:
 *      - Rodrigues Exp map.
 *      - Matrix Log baseline:
 *            Log(R)^vee = theta / (2 * sin(theta)) * (R - R^T)^vee
 *      - Around-pi axis extraction with:
 *            S = R + R^T + (1 - tr(R)) * I
 *
 * [R2] Sola, "Quaternion kinematics for the error-state Kalman filter",
 *      Sec. 2.3-2.4:
 *      - Rotation matrix and quaternion represent the same SO(3) rotation.
 *      - Quaternion Exp and Log:
 *            q = [cos(theta/2), u * sin(theta/2)]
 *            Log(q) = theta * u
 *            theta = 2 * atan2(||qv||, qw)
 *
 * [R3] Sola, Deray, Atchuthan, "A micro Lie theory for state estimation
 *      in robotics", Appendix B:
 *      - S3 and SO(3) share the same tangent vector space R^3.
 *      - q and -q encode the same SO(3) rotation.
 *      - Use positive real part before Log to select the principal branch.
 *
 * [R4] Nurlanov, "Exploring SO(3) logarithmic map: degeneracies and derivatives",
 *      Sec. 2 and Sec. 3.2.4:
 *      - Matrix baseline Log is standard.
 *      - Matrix baseline becomes ill-conditioned near theta = pi.
 *      - Through-quaternion Log is the robust candidate.
 *      - Stable quaternion Log small-angle branch:
 *            (2 / qw - 2 * ||qv||^2 / (3 * qw^3)) * qv
 *
 * [R5] manif: https://github.com/artivis/manif, SO3Base::log():
 *      - Implements SO(3) Log through quaternion coefficients.
 *      - Uses two_angle = 2 * atan2(||qv||, qw).
 *      - Uses a sign branch for the principal angle-axis representative.
 *
 * [R6] PX4 matrix::Quatf::canonical():
 *      - Selects a deterministic representative from q and -q.
 *      - For regular cases, this gives qw >= 0.
 *      - For theta ~= pi, it fixes the axis sign using the first significant component.
 */

/**
 * SO(3) exponential map.
 *
 * Formula:
 *     theta = ||phi||
 *     Exp(phi) = I
 *              + sin(theta) / theta * hat(phi)
 *              + (1 - cos(theta)) / theta^2 * hat(phi)^2
 *
 * Sources:
 *     [R1] Rodrigues Exp map.
 *     [R3] SO(3) Exp map in Appendix B.
 *
 * Small-angle branch:
 *     sin(theta) / theta = 1 - theta^2 / 6 + theta^4 / 120 + O(theta^6)
 *     (1 - cos(theta)) / theta^2 = 1/2 - theta^2 / 24 + theta^4 / 720 + O(theta^6)
 *
 * Use:
 *     Needed by SO(3)-level methods such as Yu tilt-torsion construction.
 */
matrix::Dcmf expMapSO3(const matrix::Vector3f &phi)
{
	const float theta_sq = phi.dot(phi);
	const matrix::Dcmf W(phi.hat());
	const matrix::Dcmf W2(W * W);
	const matrix::Dcmf I;

	if (theta_sq < 1e-8f) {
		const float theta_4 = theta_sq * theta_sq;
		const float A = 1.f - theta_sq / 6.f + theta_4 / 120.f;
		const float B = 0.5f - theta_sq / 24.f + theta_4 / 720.f;

		return matrix::Dcmf(I + A * W + B * W2);
	}

	const float theta = sqrtf(theta_sq);
	const float A = sinf(theta) / theta;
	const float B = (1.f - cosf(theta)) / theta_sq;

	return matrix::Dcmf(I + A * W + B * W2);
}

/**
 * Principal SO(3) logarithm through a unit quaternion.
 *
 * Mathematical target:
 *     e = Log(R)^vee = theta * u, with theta in [0, pi].
 *
 * Quaternion representation:
 *     q = [qw, qv] = [cos(theta/2), u * sin(theta/2)]
 *
 * Principal quaternion Log:
 *     theta = 2 * atan2(||qv||, qw)
 *     u = qv / ||qv||
 *     Log(q) = theta * u
 *
 * Regular branch implemented below:
 *     Log(q) = 2 * atan2(||qv||, qw) * qv / ||qv||
 *
 * Small-angle branch implemented below:
 *     Log(q) ~= (2 / qw - 2 * ||qv||^2 / (3 * qw^3)) * qv
 *
 * Why quaternion route:
 *     Direct matrix baseline:
 *         Log(R)^vee = theta / (2 * sin(theta)) * (R - R^T)^vee
 *     is the standard formula [R1], [R4].
 *
 *     Near theta = pi, sin(theta) approaches zero and the matrix baseline
 *     becomes ill-conditioned [R4]. The through-quaternion route gives a
 *     robust principal Log and matches manif::SO3Base::log() [R5].
 *
 * Branch selection:
 *     q and -q represent the same SO(3) element [R3].
 *     q_in.canonical() selects one deterministic representative [R6].
 *     For regular cases this is equivalent to qw >= 0.
 *     The resulting angle from atan2 is the principal angle in [0, pi].
 *
 * Unit input assumption:
 *     q_in is expected to be unit length. In the attitude controller,
 *     qe = q^-1 * qd is unit length when q and qd are attitude quaternions.
 */
matrix::Vector3f logMapSO3FromUnitQuat(const matrix::Quatf &q_in)
{
	const matrix::Quatf q = q_in.canonical();

	const float qw = math::constrain(q(0), 0.f, 1.f);
	const matrix::Vector3f qv = q.imag();

	const float qv_norm_sq = qv.dot(qv);

	if (qv_norm_sq < 1e-12f) {
		const float qw_sq = qw * qw;

		if (qw > 1e-6f) {
			const float coeff = 2.f / qw - (2.f / 3.f) * qv_norm_sq / (qw * qw_sq);
			return coeff * qv;
		}

		// Defensive fallback for degenerate floating-point input.
		return 2.f * qv;
	}

	const float qv_norm = sqrtf(qv_norm_sq);

	// theta = 2 * atan2(||qv||, qw), theta in [0, pi] after canonical().
	const float angle = 2.f * atan2f(qv_norm, qw);

	// theta * u = theta * qv / ||qv||.
	return (angle / qv_norm) * qv;
}

/**
 * SO(3) logarithm with a DCM input.
 *
 * Mathematical definition:
 *     Log(R)^vee
 *
 * Reference matrix formula:
 *     theta = acos((tr(R) - 1) / 2)
 *     Log(R)^vee = theta / (2 * sin(theta)) * (R - R^T)^vee
 *
 * Around-pi matrix reference:
 *     S = R + R^T + (1 - tr(R)) * I
 *     n_j * n_k = S_jk / (3 - tr(R))
 *
 * Implementation:
 *     Convert R to a unit quaternion and call logMapSO3FromUnitQuat().
 *
 * Reason:
 *     This preserves the SO(3) mathematical definition while using the
 *     through-quaternion robust route recommended by [R4] and used by [R5].
 */
matrix::Vector3f logMapSO3(const matrix::Dcmf &R)
{
	const matrix::Quatf q(R);

	return logMapSO3FromUnitQuat(q);
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

void AttitudeControl::setRefModelFrequency(float omega_n)
{
	_omega_n = math::max(omega_n, 0.1f);
	_kq      = _omega_n * _omega_n;
}

void AttitudeControl::setAttitudeSetpoint(const Quatf &qd, const float yawspeed_setpoint, const float dt)
{
	Quatf qd_normalized = qd;
	qd_normalized.normalize();

	if (_ref_initialized && dt > 0.f) {
		propagateReferenceModel(qd_normalized, yawspeed_setpoint, dt);

	} else {
		// First call (or dt out of range): snap reference to the current setpoint.
		_q_ref = qd_normalized;
		_omega_correction.zero();
		_omega_command.zero();
		_ref_initialized = true;
	}
}

void AttitudeControl::propagateReferenceModel(const Quatf &qd, const float yawspeed_setpoint, const float dt)
{
	// 2nd-order critically damped ref model with exact (ZOH) discretisation.
	// Repeated eigenvalue at s = -_omega_n; unconditionally stable for any dt.

	// Tangent-space inputs: rotate the analytical yaw rate into q_ref's body
	//    frame, and form the small-angle error vector from q_ref to q_d.
	const Quatf q_ref_inv = qinv(_q_ref);
	const Vector3f yaw_axis_body = qzaxis(q_ref_inv); // world yaw axis expressed in q_ref's body frame
	const Vector3f omega_command = PX4_ISFINITE(yawspeed_setpoint)
				       ? yaw_axis_body * yawspeed_setpoint
				       : Vector3f{};

	Quatf q_err = qmul(q_ref_inv, qd);
	q_err.canonicalize();
	const Vector3f e = 2.f * q_err.imag();

	// Entries of exp(A*dt) for A = [0 -1; _kq -2*_omega_n]. A has the repeated eigenvalue lambda = -_omega_n.
	// The matrix N = A - lambda*I is then nilpotent (a matrix is nilpotent when
	// N^k = 0 for some k, and here N^2 = 0). Writing exp(A*dt) = e^(lambda*dt) * exp(N*dt) and expanding the
	// series for exp(N*dt) = I + N*dt + (N*dt)^2/2! + ... , every term from (N*dt)^2
	// onward vanishes, so the exponential truncates to the exact closed form
	//     exp(A*dt) = e^(-_omega_n*dt) * [ (1 + _omega_n*dt) I + dt*A ]
	//               = emt * [ a  -b ;  gamma  delta ].
	const float w_dt  = _omega_n * dt;
	const float emt   = expf(-w_dt);
	const float a     = (1.f + w_dt) * emt;
	const float b     = dt * emt;
	const float gamma = _kq * dt * emt;
	const float delta = (1.f - w_dt) * emt;

	// Propagate the error-driven correction in tangent space (the 2nd-order state). delta_phi is the integral
	//    of omega over [0, dt]; the correction part collapses to e(0) - e(dt) since e_dot = -correction.
	const Vector3f delta_phi = (1.f - a) * e + b * _omega_correction + omega_command * dt;
	_omega_correction = gamma * e + delta * _omega_correction;

	// Yaw-rate command: the heading setpoint just follows the measured yaw, so feeding the error-driven
	// rate forward closes a positive-feedback loop. Keep only the commanded rate (omega_command) on the yaw axis.
	if (PX4_ISFINITE(yawspeed_setpoint) && (fabsf(yawspeed_setpoint) > FLT_EPSILON)) {
		_omega_correction -= _omega_correction.dot(yaw_axis_body) * yaw_axis_body;
	}

	// Commanded (analytical) reference rate, kept separate so update() can exempt it from the feedforward limit.
	_omega_command = omega_command;

	_q_ref     = qmul(_q_ref, Quatf(AxisAnglef(delta_phi)));
	_q_ref.normalize();
}

void AttitudeControl::adaptAttitudeSetpoint(const Quatf &q_delta)
{
	// Apply the world-frame delta to the reference attitude. _omega_correction and _omega_command are
	// in the reference body frame and physically invariant under a world relabeling.
	_q_ref = qmul(q_delta, _q_ref);
	_q_ref.normalize();
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
		qd_red = q * q_e_red; // xi * bar_xi_c = q_tilt = q * bar_xi_c
	}

	// With a full desired attitude given by: qd = qd_red * qd_dyaw, extract the delta yaw component.
	// By definition, the delta yaw quaternion has the form (cos(angle/2), 0, 0, sin(angle/2))
	Quatf qd_dyaw = qmul(qinv(qd_red), qd);
	qd_dyaw.canonicalize();
	// catch numerical problems with the domain of acosf and asinf
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	// scale the delta yaw angle and re-combine the desired attitude
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = qmul(qinv(q), qd);

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
	// PX4 body-frame attitude error:
	//     qe = q^-1 * qd
	//
	// Same mathematical error as:
	//     Re = R^T * Rd
	//     eq = Log(Re)^vee
	//
	// Quaternion principal Log:
	//     qe = [qw, qv] = [cos(theta/2), u * sin(theta/2)]
	//     eq = 2 * atan2(||qv||, qw) * qv / ||qv||
	const Quatf qe = q.inversed() * qd;

	return logMapSO3FromUnitQuat(qe);
}

matrix::Vector3f AttitudeControl::calculateAttitudeErrorDcmLog(const Quatf &q, const Quatf &qd) const
{
	const Dcmf R(q);
	const Dcmf Rd(qd);

	// PX4 body-frame attitude error:
	//     Re = R^T * Rd
	//
	// This is the DCM form of:
	//     qe = q^-1 * qd
	//
	// Desired error vector:
	//     eq = Log(Re)^vee
	//
	// Direct R formula for reference:
	//     theta = acos((tr(Re) - 1) / 2)
	//     eq = theta / (2 * sin(theta)) * (Re - Re^T)^vee
	//
	// Small-angle matrix reference:
	//     eq ~= 0.5 * (1 + theta^2 / 6 + 7 * theta^4 / 360)
	//           * (Re - Re^T)^vee
	//
	// Around-pi matrix reference:
	//     S = Re + Re^T + (1 - tr(Re)) * I
	//     n_j * n_k = S_jk / (3 - tr(Re))
	//
	// Compiled implementation:
	//     logMapSO3(Re), internally using quaternion-based principal Log.
	const Dcmf Re = R.transpose() * Rd;

	return logMapSO3(Re);
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
		debugPrintAttitudeErrorModeIfChanged(_attitude_error_mode, "ATTITUDE_ERROR_QUATERNION_IMAG");
		return calculateAttitudeErrorQuaternionImag(q, qd); // Equivalent: 2*n*sin(theta/2). It can be found in [1], here is the original version without separate handling of yaw. At theta=0 it returns 0; at theta=pi it returns 2*n_canonical, with canonical() selecting the q/-q representative.

	case ATTITUDE_ERROR_QUATERNION_LOG:
		debugPrintAttitudeErrorModeIfChanged(_attitude_error_mode, "ATTITUDE_ERROR_QUATERNION_LOG");
		return calculateAttitudeErrorQuaternionLog(q, qd); // Equivalent: n*theta, more robust than the DCM log map. At theta=0 it returns 0 by the quaternion Log Taylor limit; at theta=pi it returns pi*n_canonical through the principal quaternion Log.

	case ATTITUDE_ERROR_DCM_LOG:
		debugPrintAttitudeErrorModeIfChanged(_attitude_error_mode, "ATTITUDE_ERROR_DCM_LOG");
		return calculateAttitudeErrorDcmLog(q, qd); // Equivalent: n*theta. It can be found in [5], which handles the situation theta = pi. The direct matrix formula is 0/0 at theta=pi, and our implementation corrects it to pi*n_canonical through quaternion-based principal Log.

	case ATTITUDE_ERROR_DCM_VEE:
		debugPrintAttitudeErrorModeIfChanged(_attitude_error_mode, "ATTITUDE_ERROR_DCM_VEE");
		return calculateAttitudeErrorDcmVee(q, qd); // Equivalent: n*sin(theta). It can be found in [3], and is equivalent to 2*q_0*q_v in quaternion version. At theta=0 it returns 0; at theta=pi it also returns 0, which is the known pi critical point of this vee error.

	case ATTITUDE_ERROR_EZRA_TAL:
		debugPrintAttitudeErrorModeIfChanged(_attitude_error_mode, "ATTITUDE_ERROR_EZRA_TAL");
		return calculateAttitudeErrorEzraTal(q, qd); // Equivalent: n*theta. It can be found in [4], where there are some singularities. At theta=0 it returns 0; at theta=pi the nonsingular xi_c branch returns pi*n_canonical, and the antipodal-thrust singularity is corrected by selecting bar_xi_c = [0, 1, 0, 0].

	case ATTITUDE_ERROR_TILT_PRIORITIZED:
		debugPrintAttitudeErrorModeIfChanged(_attitude_error_mode, "ATTITUDE_ERROR_TILT_PRIORITIZED");
		return calculateAttitudeErrorTiltPrioritized(q, qd); // Equivalent: 2*n*sin(theta/2). It can be found in [6][7]. At theta=0 it returns 0; at theta=pi the split is finite when sqrt(q0^2+q3^2)>0, and the opposite-thrust singularity q0=q3=0 is corrected by falling back to 2*imag(qe.canonical()) = 2*n_canonical.

	case ATTITUDE_ERROR_TILT_TORSION:
		debugPrintAttitudeErrorModeIfChanged(_attitude_error_mode, "ATTITUDE_ERROR_TILT_TORSION");
		return calculateAttitudeErrorTiltTorsion(q, qd); // Equivalent: n*theta. It can be found in [2]. At theta=0 it returns 0; at theta=pi it is evaluated by the tilt/torsion split, with antipodal tilt corrected by selecting R_tilt=Exp(pi*e_x), while pure torsion gives yaw_w*pi*e_z.

	case ATTITUDE_ERROR_DEFAULT:
	default:
		debugPrintAttitudeErrorModeIfChanged(_attitude_error_mode, "ATTITUDE_ERROR_DEFAULT");
		return calculateAttitudeErrorDefault(q, qd); // Equivalent: 2*n*sin(theta/2). It can be found in [1], here is the version with separate handling of yaw. At zero final mixed error it returns 0; at pi final mixed error it returns 2*n_canonical, and the opposite-thrust corner keeps the original PX4 full-attitude fallback before yaw recombination.
	}
}

matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	// The P controller always tracks the reference-model attitude, while the selected
	// error mode only changes how the error between q and _q_ref is represented.
	const Vector3f eq = calculateAttitudeError(q, _q_ref);

	// calculate angular rates setpoint
	Vector3f rate_setpoint = eq.emult(_proportional_gain);

	// Map reference-frame rates into the current body frame.
	const Quatf q_rel = qmul(qinv(q), _q_ref);

	// The commanded reference rate (e.g. manual/auto yaw rate) is a setpoint, not a model prediction, so it
	// bypasses the reference model: it is fed forward at unity regardless of the feedforward gain and limit.
	rate_setpoint += q_rel.rotateVector(_omega_command);

	// the gain scales and the limit caps the model's error-driven anticipation (zero at gain 0)
	Vector3f omega_ff = _ff_gain * q_rel.rotateVector(_omega_correction);

	if (_ff_max > 0.f) {
		for (int i = 0; i < 3; i++) {
			omega_ff(i) = math::constrain(omega_ff(i), -_ff_max, _ff_max);
		}
	}

	rate_setpoint += omega_ff;

	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
