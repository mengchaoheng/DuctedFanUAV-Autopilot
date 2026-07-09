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
 * @file AttitudeControl.hpp
 *
 * A quaternion based attitude controller.
 *
 * @author Matthias Grob	<maetugr@gmail.com>
 *
 * Publication documenting the implemented Quaternion Attitude Control:
 * Nonlinear Quadrocopter Attitude Control (2013)
 * by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
 * Institute for Dynamic Systems and Control (IDSC), ETH Zurich
 *
 * https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

class AttitudeControl
{
public:
	AttitudeControl() = default;
	~AttitudeControl() = default;

	/**
	 * Set proportional attitude control gain
	 * @param proportional_gain 3D vector containing gains for roll, pitch, yaw
	 * @param yaw_weight A fraction [0,1] deprioritizing yaw compared to roll and pitch
	 */
	void setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight);

	/**
	 * Select attitude error calculation mode
	 *
	 * 0: PX4 default reduced attitude with yaw weighting, Brescianini et al. 2013 style,
	 *    error scale 2 * n * sin(theta / 2).
	 * 1: Full quaternion imaginary-vector error,
	 *    error scale 2 * n * sin(theta / 2).
	 * 2: Full quaternion logarithm-map error,
	 *    error scale n * theta.
	 * 3: Full DCM logarithm-map error,
	 *    error scale n * theta.
	 * 4: Full DCM vee-map error, Lee et al. 2010 style,
	 *    error scale n * sin(theta).
	 * 5: Tal and Karaman incremental attitude command error,
	 *    error scale n * theta.
	 * 6: Tilt-prioritized quaternion error, Brescianini and D'Andrea 2020 / Sun et al. 2022 style,
	 *    error scale 2 * n * sin(theta / 2).
	 * 7: Tilt-torsion SO(3) logarithm-map error, Yu et al. 2015 style,
	 *    error scale n * theta.
	 */
	void setAttitudeErrorMode(int mode);

	/**
	 * Set hard limit for output rate setpoints
	 * @param rate_limit [rad/s] 3D vector containing limits for roll, pitch, yaw
	 */
	void setRateLimit(const matrix::Vector3f &rate_limit) { _rate_limit = rate_limit; }

	/**
	 * Set a new attitude setpoint replacing the one tracked before
	 * @param qd desired vehicle attitude setpoint
	 * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
	 */
	void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint)
	{
		_attitude_setpoint_q = qd;
		_attitude_setpoint_q.normalize();
		_yawspeed_setpoint = yawspeed_setpoint;
	}

	/**
	 * Adjust last known attitude setpoint by a delta rotation
	 * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
	 * @param q_delta delta rotation to apply
	 */
	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
	{
		_attitude_setpoint_q = q_delta * _attitude_setpoint_q;
		_attitude_setpoint_q.normalize();
	}

	/**
	 * Run one control loop cycle calculation
	 * @param q estimation of the current vehicle attitude unit quaternion
	 * @return [rad/s] body frame 3D angular rate setpoint vector to be executed by the rate controller
	 */
	matrix::Vector3f update(const matrix::Quatf &q) const;

private:
	enum AttitudeErrorMode {
		ATTITUDE_ERROR_DEFAULT = 0,
		ATTITUDE_ERROR_QUATERNION_IMAG = 1,
		ATTITUDE_ERROR_QUATERNION_LOG = 2,
		ATTITUDE_ERROR_DCM_LOG = 3,
		ATTITUDE_ERROR_DCM_VEE = 4,
		ATTITUDE_ERROR_EZRA_TAL = 5,
		ATTITUDE_ERROR_TILT_PRIORITIZED = 6,
		ATTITUDE_ERROR_TILT_TORSION = 7,
	};

	matrix::Vector3f calculateAttitudeErrorDefault(const matrix::Quatf &q, matrix::Quatf qd) const;
	matrix::Vector3f calculateAttitudeErrorQuaternionImag(const matrix::Quatf &q, const matrix::Quatf &qd) const;
	matrix::Vector3f calculateAttitudeErrorQuaternionLog(const matrix::Quatf &q, const matrix::Quatf &qd) const;
	matrix::Vector3f calculateAttitudeErrorDcmLog(const matrix::Quatf &q, const matrix::Quatf &qd) const;
	matrix::Vector3f calculateAttitudeErrorDcmVee(const matrix::Quatf &q, const matrix::Quatf &qd) const;
	matrix::Vector3f calculateAttitudeErrorEzraTal(const matrix::Quatf &q, const matrix::Quatf &qd) const;
	matrix::Vector3f calculateAttitudeErrorTiltPrioritized(const matrix::Quatf &q, const matrix::Quatf &qd) const;
	matrix::Vector3f calculateAttitudeErrorTiltTorsion(const matrix::Quatf &q, const matrix::Quatf &qd) const;
	matrix::Vector3f calculateAttitudeError(const matrix::Quatf &q, const matrix::Quatf &qd) const;

	matrix::Vector3f _proportional_gain;
	matrix::Vector3f _rate_limit;
	float _yaw_w{0.f}; ///< yaw weight [0,1] to deprioritize compared to roll and pitch
	int _attitude_error_mode{ATTITUDE_ERROR_DEFAULT}; ///< selected attitude error calculation mode

	matrix::Quatf _attitude_setpoint_q; ///< latest known attitude setpoint e.g. from position control
	float _yawspeed_setpoint{0.f}; ///< latest known yawspeed feed-forward setpoint
};
