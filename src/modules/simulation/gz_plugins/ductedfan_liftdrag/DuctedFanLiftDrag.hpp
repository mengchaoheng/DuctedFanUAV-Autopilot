/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <gz/math/Helpers.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/wind.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/transport/Node.hh>

#include <memory>
#include <string>
#include <vector>

namespace px4
{

class DuctedFanLiftDrag :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity,
		       const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm,
		       gz::sim::EventManager &eventMgr) override;

	void PreUpdate(const gz::sim::UpdateInfo &info,
		       gz::sim::EntityComponentManager &ecm) override;

private:
	struct PropellerWind {
		std::string joint_name;
		gz::sim::Entity joint_entity{gz::sim::kNullEntity};
		double k_v{0.0};
		double rotor_velocity_slowdown_sim{1.0};
	};

	void WindCallback(const gz::msgs::Wind &msg);
	void ResolveEntities(gz::sim::EntityComponentManager &ecm);
	bool EnsureComponents(gz::sim::EntityComponentManager &ecm);
	gz::math::Vector3d PropellerWashVelocity(gz::sim::EntityComponentManager &ecm,
			const gz::math::Quaterniond &link_rot) const;

	gz::sim::Model _model{gz::sim::kNullEntity};
	gz::sim::Entity _link_entity{gz::sim::kNullEntity};
	gz::sim::Link _link{gz::sim::kNullEntity};
	gz::sim::Entity _control_joint_entity{gz::sim::kNullEntity};

	std::string _link_name;
	std::string _control_joint_name;
	std::vector<PropellerWind> _propeller_wind;

	gz::transport::Node _node;
	gz::math::Vector3d _wind_velocity{0.0, 0.0, 0.0};

	double _cla{1.0};
	double _cda{0.01};
	double _cma{0.0};
	double _alpha_stall{0.5 * GZ_PI};
	double _cla_stall{0.0};
	double _cda_stall{1.0};
	double _cma_stall{0.0};
	double _cm_delta{0.0};
	double _rho{1.2041};
	double _area{1.0};
	double _alpha0{0.0};
	double _control_joint_rad_to_cl{4.0};
	bool _radial_symmetry{false};
	bool _wash_only{false};

	gz::math::Vector3d _cp{0.0, 0.0, 0.0};
	gz::math::Vector3d _forward{1.0, 0.0, 0.0};
	gz::math::Vector3d _upward{0.0, 0.0, 1.0};
};

} // namespace px4
