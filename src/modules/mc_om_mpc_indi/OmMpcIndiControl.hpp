/**
 * Copyright 2026 Chaoheng Meng
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include <matrix/matrix/math.hpp>

class OmMpcIndiControl
{
public:
	struct Output {
		matrix::Vector3f rates_setpoint{};
		matrix::Vector3f thrust_body{};
		matrix::Vector3f attitude_error{};
	};

	void setParams(const matrix::Vector3f &attitude_gain, const matrix::Vector3f &rate_limit,
		       float hover_thrust, float gravity);
	bool paramsValid() const;

	bool update(const matrix::Dcmf &R_to_ned, const matrix::Vector3f &nominal_rates,
		    const matrix::Vector3f &nominal_thrust_body,
		    const matrix::Vector3f &mpc_disturbance_ned,
		    const matrix::Vector3f &acceleration_ned,
		    const matrix::Vector3f &allocated_thrust_ned, Output &output) const;

private:
	matrix::Vector3f _attitude_gain{};
	matrix::Vector3f _rate_limit{};
	float _hover_thrust{0.f};
	float _gravity{9.80665f};
};

