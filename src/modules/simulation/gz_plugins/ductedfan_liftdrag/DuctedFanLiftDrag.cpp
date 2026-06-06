/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "DuctedFanLiftDrag.hpp"

#include <gz/math/Helpers.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>

#include <algorithm>
#include <cmath>
#include <stdexcept>

using namespace px4;

namespace
{

template<typename T>
void ReadIfPresent(const std::shared_ptr<const sdf::Element> &sdf, const std::string &name, T &value)
{
	if (sdf->HasElement(name)) {
		value = sdf->Get<T>(name);
	}
}

} // namespace

void DuctedFanLiftDrag::Configure(const gz::sim::Entity &entity,
				  const std::shared_ptr<const sdf::Element> &sdf,
				  gz::sim::EntityComponentManager &ecm,
				  gz::sim::EventManager &)
{
	_model = gz::sim::Model(entity);

	if (!_model.Valid(ecm)) {
		gzerr << "DuctedFanLiftDrag must be attached to a model entity.\n";
		return;
	}

	const auto sdf_clone = sdf->Clone();

	ReadIfPresent(sdf_clone, "radial_symmetry", _radial_symmetry);
	ReadIfPresent(sdf_clone, "a0", _alpha0);
	ReadIfPresent(sdf_clone, "cla", _cla);
	ReadIfPresent(sdf_clone, "cda", _cda);
	ReadIfPresent(sdf_clone, "cma", _cma);
	ReadIfPresent(sdf_clone, "alpha_stall", _alpha_stall);
	ReadIfPresent(sdf_clone, "cla_stall", _cla_stall);
	ReadIfPresent(sdf_clone, "cda_stall", _cda_stall);
	ReadIfPresent(sdf_clone, "cma_stall", _cma_stall);
	ReadIfPresent(sdf_clone, "cm_delta", _cm_delta);
	ReadIfPresent(sdf_clone, "cp", _cp);
	ReadIfPresent(sdf_clone, "forward", _forward);
	ReadIfPresent(sdf_clone, "upward", _upward);
	ReadIfPresent(sdf_clone, "area", _area);
	ReadIfPresent(sdf_clone, "air_density", _rho);
	ReadIfPresent(sdf_clone, "link_name", _link_name);
	ReadIfPresent(sdf_clone, "control_joint_name", _control_joint_name);
	ReadIfPresent(sdf_clone, "control_joint_rad_to_cl", _control_joint_rad_to_cl);

	_forward.Normalize();
	_upward.Normalize();

	if (sdf_clone->HasElement("propeller_wind_map")) {
		auto propeller_map = sdf_clone->GetElement("propeller_wind_map");
		ReadIfPresent(propeller_map, "wash_only", _wash_only);

		auto propeller = propeller_map->GetElement("propeller");

		while (propeller) {
			PropellerWind wind;
			wind.joint_name = propeller->Get<std::string>("joint");
			wind.k_v = propeller->Get<double>("k_v");
			wind.rotor_velocity_slowdown_sim = propeller->Get<double>("rotorVelocitySlowdownSim");
			_propeller_wind.push_back(wind);
			propeller = propeller->GetNextElement("propeller");
		}
	}

	ResolveEntities(ecm);

	const auto world_entity = gz::sim::worldEntity(ecm);
	const auto world = gz::sim::World(world_entity);
	const std::string world_name = world.Name(ecm).value_or("default");
	std::string wind_topic = "/world/" + world_name + "/wind_info";

	if (sdf_clone->HasElement("windSubTopic")) {
		const std::string configured_topic = sdf_clone->Get<std::string>("windSubTopic");

		if (!configured_topic.empty() && configured_topic.front() == '/') {
			wind_topic = configured_topic;
		}
	}

	_node.Subscribe(wind_topic, &DuctedFanLiftDrag::WindCallback, this);
}

void DuctedFanLiftDrag::PreUpdate(const gz::sim::UpdateInfo &info,
				  gz::sim::EntityComponentManager &ecm)
{
	if (info.paused) {
		return;
	}

	ResolveEntities(ecm);

	if (!EnsureComponents(ecm)) {
		return;
	}

	const auto pose = _link.WorldPose(ecm);
	const auto linear_velocity = _link.WorldLinearVelocity(ecm);
	const auto angular_velocity = _link.WorldAngularVelocity(ecm);

	if (!pose.has_value() || !linear_velocity.has_value() || !angular_velocity.has_value()) {
		return;
	}

	const gz::math::Vector3d cp_world = pose->Rot().RotateVector(_cp);
	const gz::math::Vector3d link_velocity_world = *linear_velocity + angular_velocity->Cross(cp_world);
	const gz::math::Vector3d wash_velocity_world = PropellerWashVelocity(ecm, pose->Rot());
	const gz::math::Vector3d air_velocity = _wash_only ? -wash_velocity_world :
						link_velocity_world - _wind_velocity - wash_velocity_world;

	if (air_velocity.Length() <= 0.01) {
		return;
	}

	gz::math::Vector3d vel_i = air_velocity;
	vel_i.Normalize();

	gz::math::Vector3d forward_i = pose->Rot().RotateVector(_forward);

	if (forward_i.Dot(air_velocity) <= 0.0) {
		return;
	}

	gz::math::Vector3d upward_i;

	if (_radial_symmetry) {
		const gz::math::Vector3d tmp = forward_i.Cross(vel_i);
		upward_i = forward_i.Cross(tmp);
		upward_i.Normalize();

	} else {
		upward_i = pose->Rot().RotateVector(_upward);
	}

	gz::math::Vector3d spanwise_i = forward_i.Cross(upward_i);
	spanwise_i.Normalize();

	const double min_ratio = -1.0;
	const double max_ratio = 1.0;
	double sweep = std::asin(gz::math::clamp(spanwise_i.Dot(vel_i), min_ratio, max_ratio));

	while (std::fabs(sweep) > 0.5 * GZ_PI) {
		sweep = sweep > 0.0 ? sweep - GZ_PI : sweep + GZ_PI;
	}

	const double cos_sweep_angle = 1.0;
	const gz::math::Vector3d vel_in_ld_plane = air_velocity - air_velocity.Dot(spanwise_i) * spanwise_i;

	gz::math::Vector3d drag_direction = -vel_in_ld_plane;
	drag_direction.Normalize();

	gz::math::Vector3d lift_i = spanwise_i.Cross(vel_in_ld_plane);
	lift_i.Normalize();

	const gz::math::Vector3d moment_direction = spanwise_i;
	const double cos_alpha = gz::math::clamp(lift_i.Dot(upward_i), min_ratio, max_ratio);
	double alpha = lift_i.Dot(forward_i) >= 0.0 ? _alpha0 + std::acos(cos_alpha) :
		       _alpha0 - std::acos(cos_alpha);

	while (std::fabs(alpha) > 0.5 * GZ_PI) {
		alpha = alpha > 0.0 ? alpha - GZ_PI : alpha + GZ_PI;
	}

	const double speed_in_ld_plane = vel_in_ld_plane.Length();
	const double q = 0.5 * _rho * speed_in_ld_plane * speed_in_ld_plane;

	double cl = 0.0;

	if (alpha > _alpha_stall) {
		cl = (_cla * _alpha_stall + _cla_stall * (alpha - _alpha_stall)) * cos_sweep_angle;
		cl = std::max(0.0, cl);

	} else if (alpha < -_alpha_stall) {
		cl = (-_cla * _alpha_stall + _cla_stall * (alpha + _alpha_stall)) * cos_sweep_angle;
		cl = std::min(0.0, cl);

	} else {
		cl = _cla * alpha * cos_sweep_angle;
	}

	double control_angle = 0.0;

	if (_control_joint_entity != gz::sim::kNullEntity) {
		const auto joint_position = ecm.Component<gz::sim::components::JointPosition>(_control_joint_entity);

		if (joint_position && !joint_position->Data().empty()) {
			control_angle = joint_position->Data()[0];
		}
	}

	cl += _control_joint_rad_to_cl * control_angle;
	const gz::math::Vector3d lift = cl * q * _area * lift_i;

	double cd = 0.0;

	if (alpha > _alpha_stall) {
		cd = (_cda * _alpha_stall + _cda_stall * (alpha - _alpha_stall)) * cos_sweep_angle;

	} else if (alpha < -_alpha_stall) {
		cd = (-_cda * _alpha_stall + _cda_stall * (alpha + _alpha_stall)) * cos_sweep_angle;

	} else {
		cd = _cda * alpha * cos_sweep_angle;
	}

	cd = std::fabs(cd);
	const gz::math::Vector3d drag = cd * q * _area * drag_direction;

	double cm = 0.0;

	if (alpha > _alpha_stall) {
		cm = (_cma * _alpha_stall + _cma_stall * (alpha - _alpha_stall)) * cos_sweep_angle;
		cm = std::max(0.0, cm);

	} else if (alpha < -_alpha_stall) {
		cm = (-_cma * _alpha_stall + _cma_stall * (alpha + _alpha_stall)) * cos_sweep_angle;
		cm = std::min(0.0, cm);

	} else {
		cm = _cma * alpha * cos_sweep_angle;
	}

	cm += _cm_delta * control_angle;

	gz::math::Vector3d force = lift + drag;
	gz::math::Vector3d moment = cm * q * _area * moment_direction;
	force.Correct();
	moment.Correct();

	_link.AddWorldWrench(ecm, force, moment, _cp);
}

void DuctedFanLiftDrag::ResolveEntities(gz::sim::EntityComponentManager &ecm)
{
	if (_link_entity == gz::sim::kNullEntity && !_link_name.empty()) {
		_link_entity = _model.LinkByName(ecm, _link_name);

		if (_link_entity != gz::sim::kNullEntity) {
			_link = gz::sim::Link(_link_entity);
			_link.EnableVelocityChecks(ecm, true);
		}
	}

	if (_control_joint_entity == gz::sim::kNullEntity && !_control_joint_name.empty()) {
		_control_joint_entity = _model.JointByName(ecm, _control_joint_name);
	}

	for (auto &propeller : _propeller_wind) {
		if (propeller.joint_entity == gz::sim::kNullEntity) {
			propeller.joint_entity = _model.JointByName(ecm, propeller.joint_name);
		}
	}
}

bool DuctedFanLiftDrag::EnsureComponents(gz::sim::EntityComponentManager &ecm)
{
	bool ready = true;

	if (_link_entity == gz::sim::kNullEntity) {
		return false;
	}

	if (!ecm.Component<gz::sim::components::WorldPose>(_link_entity)) {
		ecm.CreateComponent(_link_entity, gz::sim::components::WorldPose());
		ready = false;
	}

	if (!ecm.Component<gz::sim::components::WorldLinearVelocity>(_link_entity)) {
		ecm.CreateComponent(_link_entity, gz::sim::components::WorldLinearVelocity());
		ready = false;
	}

	if (!ecm.Component<gz::sim::components::WorldAngularVelocity>(_link_entity)) {
		ecm.CreateComponent(_link_entity, gz::sim::components::WorldAngularVelocity());
		ready = false;
	}

	if (_control_joint_entity != gz::sim::kNullEntity &&
	    !ecm.Component<gz::sim::components::JointPosition>(_control_joint_entity)) {
		ecm.CreateComponent(_control_joint_entity, gz::sim::components::JointPosition());
		ready = false;
	}

	for (const auto &propeller : _propeller_wind) {
		if (propeller.joint_entity == gz::sim::kNullEntity) {
			ready = false;
			continue;
		}

		if (!ecm.Component<gz::sim::components::JointVelocity>(propeller.joint_entity)) {
			ecm.CreateComponent(propeller.joint_entity, gz::sim::components::JointVelocity());
			ready = false;
		}

		if (!ecm.Component<gz::sim::components::JointAxis>(propeller.joint_entity)) {
			ready = false;
		}
	}

	return ready;
}

gz::math::Vector3d DuctedFanLiftDrag::PropellerWashVelocity(gz::sim::EntityComponentManager &ecm,
		const gz::math::Quaterniond &link_rot) const
{
	gz::math::Vector3d wash_velocity{0.0, 0.0, 0.0};

	for (const auto &propeller : _propeller_wind) {
		if (propeller.joint_entity == gz::sim::kNullEntity) {
			continue;
		}

		const auto joint_velocity = ecm.Component<gz::sim::components::JointVelocity>(propeller.joint_entity);
		const auto joint_axis = ecm.Component<gz::sim::components::JointAxis>(propeller.joint_entity);

		if (!joint_velocity || joint_velocity->Data().empty() || !joint_axis) {
			continue;
		}

		const double omega = joint_velocity->Data()[0] * propeller.rotor_velocity_slowdown_sim;
		const double speed_mag = propeller.k_v * std::fabs(omega);
		gz::math::Vector3d axis = link_rot.RotateVector(joint_axis->Data().Xyz());
		axis.Normalize();
		wash_velocity += -axis * speed_mag;
	}

	return wash_velocity;
}

void DuctedFanLiftDrag::WindCallback(const gz::msgs::Wind &msg)
{
	_wind_velocity.Set(msg.linear_velocity().x(), msg.linear_velocity().y(), msg.linear_velocity().z());
}

GZ_ADD_PLUGIN(
	DuctedFanLiftDrag,
	gz::sim::System,
	DuctedFanLiftDrag::ISystemConfigure,
	DuctedFanLiftDrag::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(DuctedFanLiftDrag, "px4::DuctedFanLiftDrag")
