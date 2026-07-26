/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "Spline.hpp"

#include <gz/math/Vector3.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/msgs/wind.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/transport/Node.hh>

#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace px4
{

class DuctedFanPlugin :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity,
		       const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm,
		       gz::sim::EventManager &event_mgr) override;

	void PreUpdate(const gz::sim::UpdateInfo &info,
		       gz::sim::EntityComponentManager &ecm) override;

private:
	template<typename T>
	class FirstOrderFilter
	{
	public:
		FirstOrderFilter(double time_constant_up, double time_constant_down, T initial_state) :
			_time_constant_up(time_constant_up),
			_time_constant_down(time_constant_down),
			_previous_state(initial_state)
		{
		}

		T Update(T input_state, double sampling_time)
		{
			const double time_constant = input_state > _previous_state ? _time_constant_up : _time_constant_down;
			const double alpha = std::exp(-sampling_time / time_constant);
			_previous_state = alpha * _previous_state + (1.0 - alpha) * input_state;
			return _previous_state;
		}

	private:
		double _time_constant_up{0.0125};
		double _time_constant_down{0.025};
		T _previous_state{};
	};

	void ResolveEntities(gz::sim::EntityComponentManager &ecm);
	bool EnsureComponents(gz::sim::EntityComponentManager &ecm);
	void UpdateForcesAndMoments(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm);
	void ReadControlJointAngles(gz::sim::EntityComponentManager &ecm);
	void LoadControlJoints(const std::shared_ptr<const sdf::Element> &sdf);
	void LoadControlEffectivenessMatrix(const std::shared_ptr<const sdf::Element> &sdf);
	void ActuatorCallback(const gz::msgs::Actuators &msg);
	void MotorFailureCallback(const gz::msgs::Int32 &msg);
	void WindCallback(const gz::msgs::Wind &msg);
	void SubscribeActuatorTopics(const std::shared_ptr<const sdf::Element> &sdf, gz::sim::EntityComponentManager &ecm);
	void SubscribeWindTopic(const std::shared_ptr<const sdf::Element> &sdf, gz::sim::EntityComponentManager &ecm);
	void SubscribeFailureTopics(const std::shared_ptr<const sdf::Element> &sdf, gz::sim::EntityComponentManager &ecm);

	gz::sim::Model _model{gz::sim::kNullEntity};
	gz::sim::Entity _joint_entity{gz::sim::kNullEntity};
	gz::sim::Entity _rotor_link_entity{gz::sim::kNullEntity};
	gz::sim::Entity _base_link_entity{gz::sim::kNullEntity};
	gz::sim::Link _base_link{gz::sim::kNullEntity};

	std::string _model_name;
	std::string _joint_name;
	std::string _link_name;
	std::string _command_sub_topic{"command/motor_speed"};

	static constexpr int kControlJointCount = 6;
	std::array<std::string, kControlJointCount> _control_joint_names{};
	std::array<gz::sim::Entity, kControlJointCount> _control_joint_entities{};
	std::array<double, kControlJointCount> _control_joint_angles{};
	std::array<std::array<double, kControlJointCount>, 3> _b_cs{};

	gz::transport::Node _node;
	gz::math::Vector3d _wind_velocity{0.0, 0.0, 0.0};
	std::mutex _command_mutex;
	double _ref_motor_rot_vel{0.0};
	int _motor_failure_number{0};

	int _motor_number{0};
	int _turning_direction{1};
	bool _reversible{false};

	double _sampling_time{0.01};
	double _max_rot_velocity{838.0};
	double _motor_constant{8.54858e-06};
	double _moment_constant{0.016};
	double _time_constant_up{1.0 / 80.0};
	double _time_constant_down{1.0 / 40.0};
	double _rotor_velocity_slowdown_sim{10.0};

	double _k_th{0.0};
	double _k_ts{0.0};
	double _k_ns{0.0};
	double _l_cpz{0.0};
	double _k_cpx{0.0};
	double _duct_sd{0.7};
	double _duct_s{0.0408};
	double _air_density{1.225};
	double _k_my0{0.0};
	double _k_myv{0.0};
	double _k_sta{0.0};
	double _fan_inertia{0.0};
	double _d_cs{0.0};
	double _l1_cs{0.0};
	double _l2_cs{0.0};

	std::vector<SplineSegment> _spline_dt;
	std::vector<SplineSegment> _spline_dn;
	std::vector<SplineSegment> _wing_spline_wl;
	std::vector<SplineSegment> _wing_spline_wd;
	std::vector<SplineSegment> _wing_spline_wm;
	std::unique_ptr<FirstOrderFilter<double>> _rotor_velocity_filter;
};

} // namespace px4
