/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

/**
 * @file IndiControl.hpp
 */

#pragma once

#include <matrix/matrix/math.hpp>

class IndiControl
{
public:
	struct Output {
		matrix::Vector3f rate_error_torque;
		matrix::Vector3f feedback_torque;
	};

	IndiControl() = default;
	~IndiControl() = default;

	void setParams(const matrix::Vector3f &P, const matrix::Vector3f &inertia);
	bool paramsValid() const;
	void setVaneParams(float hover_thrust, float hover_wash, float motor_tau, float torque_cutoff = 10.f);

	// Vane model: raw nominal B*u input. Disabled model: existing filtered B*u.
	Output update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
		      const matrix::Vector3f &angular_accel, const matrix::Vector3f &allocated_torque,
		      float thrust = 0.f, float axial_velocity = 0.f, float dt = 0.f);

private:
	matrix::Vector3f _gain_p;
	matrix::Vector3f _inertia;
	float _hover_thrust{1.f};
	float _hover_wash{0.f};
	float _motor_tau{0.03f};
	float _wash_estimate{0.f};
	bool _wash_initialized{false};
	float _torque_cutoff{10.f};
	matrix::Vector3f _torque_input1{}, _torque_input2{}, _torque_output1{}, _torque_output2{};
	bool _torque_initialized{false};
	matrix::Vector3f filterPhysicalTorque(const matrix::Vector3f &input, float dt);
};
