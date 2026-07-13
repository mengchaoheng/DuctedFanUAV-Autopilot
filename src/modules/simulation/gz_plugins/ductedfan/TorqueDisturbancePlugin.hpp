/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/transport/Node.hh>

#include <array>
#include <random>
#include <string>

namespace px4
{

class TorqueDisturbancePlugin :
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
	enum class Mode {
		Constant,
		Sine,
		GaussMarkov
	};

	enum class Frame {
		BodyFlu,
		BodyFrd,
		WorldEnu,
		WorldNed
	};

	gz::sim::Model _model{gz::sim::kNullEntity};
	gz::sim::Entity _link_entity{gz::sim::kNullEntity};
	gz::sim::Link _link{gz::sim::kNullEntity};
	gz::transport::Node _node;
	gz::transport::Node::Publisher _wrench_publisher;

	std::string _link_name;
	Mode _mode{Mode::Sine};
	Frame _torque_frame{Frame::BodyFlu};
	Frame _force_frame{Frame::BodyFlu};
	double _amplitude_x{0.0};
	double _amplitude_y{0.0};
	double _amplitude_z{0.0};
	double _frequency_x{1.0};
	double _frequency_y{1.0};
	double _frequency_z{1.0};
	double _phase_x{0.0};
	double _phase_y{0.0};
	double _phase_z{0.0};
	double _bias_x{0.0};
	double _bias_y{0.0};
	double _bias_z{0.0};
	double _force_amplitude_x{0.0};
	double _force_amplitude_y{0.0};
	double _force_amplitude_z{0.0};
	double _force_frequency_x{1.0};
	double _force_frequency_y{1.0};
	double _force_frequency_z{1.0};
	double _force_phase_x{0.0};
	double _force_phase_y{0.0};
	double _force_phase_z{0.0};
	double _force_bias_x{0.0};
	double _force_bias_y{0.0};
	double _force_bias_z{0.0};
	double _start_time_sec{0.0};
	double _running_time_sec{0.1};
	int _repeat_count{1};
	double _repeat_interval_sec{0.0};
	int _last_repeat_index{-1};
	unsigned int _seed{0};
	std::array<double, 3> _torque_correlation_time_sec{{1.0, 1.0, 1.0}};
	std::array<double, 3> _force_correlation_time_sec{{1.0, 1.0, 1.0}};
	std::mt19937 _random_engine{0};
	std::normal_distribution<double> _standard_normal{0.0, 1.0};
	std::array<double, 3> _torque_gauss_markov_state{};
	std::array<double, 3> _force_gauss_markov_state{};
	bool _gauss_markov_initialized{false};
	std::string _wrench_topic;
	double _publish_rate_hz{50.0};
	double _last_publish_time_sec{0.0};
	bool _has_last_publish_time{false};
	double _reference_time{0.0};
	bool _has_reference_time{false};
};

} // namespace px4
