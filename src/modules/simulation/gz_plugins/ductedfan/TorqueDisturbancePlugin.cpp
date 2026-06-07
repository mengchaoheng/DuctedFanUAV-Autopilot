/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "TorqueDisturbancePlugin.hpp"

#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Pose.hh>

#include <chrono>
#include <cmath>

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

void TorqueDisturbancePlugin::Configure(const gz::sim::Entity &entity,
					const std::shared_ptr<const sdf::Element> &sdf,
					gz::sim::EntityComponentManager &ecm,
					gz::sim::EventManager &)
{
	_model = gz::sim::Model(entity);

	if (!_model.Valid(ecm)) {
		gzerr << "[TorqueDisturbancePlugin] Plugin must be attached to a model entity.\n";
		return;
	}

	ReadIfPresent(sdf, "link_name", _link_name);
	ReadIfPresent(sdf, "amplitude_x", _amplitude_x);
	ReadIfPresent(sdf, "amplitude_y", _amplitude_y);
	ReadIfPresent(sdf, "amplitude_z", _amplitude_z);
	ReadIfPresent(sdf, "frequency_x", _frequency_x);
	ReadIfPresent(sdf, "frequency_y", _frequency_y);
	ReadIfPresent(sdf, "frequency_z", _frequency_z);
	ReadIfPresent(sdf, "bias_x", _bias_x);
	ReadIfPresent(sdf, "bias_y", _bias_y);
	ReadIfPresent(sdf, "bias_z", _bias_z);
	ReadIfPresent(sdf, "start_time", _start_time_sec);
	ReadIfPresent(sdf, "running_time", _running_time_sec);

	_link_entity = _model.LinkByName(ecm, _link_name);

	if (_link_entity == gz::sim::kNullEntity) {
		gzerr << "[TorqueDisturbancePlugin] Link " << _link_name << " not found.\n";
		return;
	}

	_link = gz::sim::Link(_link_entity);
}

void TorqueDisturbancePlugin::PreUpdate(const gz::sim::UpdateInfo &info,
					gz::sim::EntityComponentManager &ecm)
{
	if (info.paused || _link_entity == gz::sim::kNullEntity) {
		return;
	}

	if (!ecm.Component<gz::sim::components::WorldPose>(_link_entity)) {
		ecm.CreateComponent(_link_entity, gz::sim::components::WorldPose());
		return;
	}

	const double sim_time = std::chrono::duration<double>(info.simTime).count();

	if (!_has_reference_time) {
		_reference_time = sim_time;
		_has_reference_time = true;
	}

	const double t = sim_time - _reference_time;

	if (t < _start_time_sec || t > _start_time_sec + _running_time_sec) {
		return;
	}

	const double tau_x = _bias_x + _amplitude_x * std::sin(2.0 * GZ_PI * _frequency_x * (t - _start_time_sec));
	const double tau_y = _bias_y + _amplitude_y * std::sin(2.0 * GZ_PI * _frequency_y * (t - _start_time_sec));
	const double tau_z = _bias_z + _amplitude_z * std::sin(2.0 * GZ_PI * _frequency_z * (t - _start_time_sec));

	const auto pose = _link.WorldPose(ecm);

	if (!pose.has_value()) {
		return;
	}

	const gz::math::Vector3d torque_world = pose->Rot().RotateVector({tau_x, tau_y, tau_z});
	_link.AddWorldWrench(ecm, {0.0, 0.0, 0.0}, torque_world);
}

GZ_ADD_PLUGIN(
	TorqueDisturbancePlugin,
	gz::sim::System,
	TorqueDisturbancePlugin::ISystemConfigure,
	TorqueDisturbancePlugin::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(TorqueDisturbancePlugin, "px4::TorqueDisturbancePlugin")
