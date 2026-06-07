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
	gz::sim::Model _model{gz::sim::kNullEntity};
	gz::sim::Entity _link_entity{gz::sim::kNullEntity};
	gz::sim::Link _link{gz::sim::kNullEntity};

	std::string _link_name;
	double _amplitude_x{0.0};
	double _amplitude_y{0.0};
	double _amplitude_z{0.0};
	double _frequency_x{1.0};
	double _frequency_y{1.0};
	double _frequency_z{1.0};
	double _bias_x{0.0};
	double _bias_y{0.0};
	double _bias_z{0.0};
	double _start_time_sec{0.0};
	double _running_time_sec{0.1};
	double _reference_time{0.0};
	bool _has_reference_time{false};
};

} // namespace px4
