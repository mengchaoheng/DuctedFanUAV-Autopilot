/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "DuctedFanPlugin.hpp"

#include <gz/math/Helpers.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

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

gz::math::Vector3d FluToFrd(const gz::math::Vector3d &v)
{
	return {v.X(), -v.Y(), -v.Z()};
}

gz::math::Vector3d FrdToFlu(const gz::math::Vector3d &v)
{
	return {v.X(), -v.Y(), -v.Z()};
}

std::string NormalizeCommandSubTopic(std::string topic)
{
	while (!topic.empty() && topic.front() == '/') {
		topic.erase(topic.begin());
	}

	const std::string classic_prefix = "gazebo/";

	if (topic.rfind(classic_prefix, 0) == 0) {
		topic = topic.substr(classic_prefix.size());
	}

	if (topic.empty()) {
		topic = "command/motor_speed";
	}

	return topic;
}

} // namespace

void DuctedFanPlugin::Configure(const gz::sim::Entity &entity,
				const std::shared_ptr<const sdf::Element> &sdf,
				gz::sim::EntityComponentManager &ecm,
				gz::sim::EventManager &)
{
	_model = gz::sim::Model(entity);

	if (!_model.Valid(ecm)) {
		gzerr << "[DuctedFanPlugin] Plugin must be attached to a model entity.\n";
		return;
	}

	_model_name = _model.Name(ecm);
	const auto sdf_clone = sdf->Clone();

	ReadIfPresent(sdf_clone, "jointName", _joint_name);
	ReadIfPresent(sdf_clone, "linkName", _link_name);
	ReadIfPresent(sdf_clone, "motorNumber", _motor_number);
	ReadIfPresent(sdf_clone, "reversible", _reversible);
	ReadIfPresent(sdf_clone, "maxRotVelocity", _max_rot_velocity);
	ReadIfPresent(sdf_clone, "motorConstant", _motor_constant);
	ReadIfPresent(sdf_clone, "momentConstant", _moment_constant);
	ReadIfPresent(sdf_clone, "timeConstantUp", _time_constant_up);
	ReadIfPresent(sdf_clone, "timeConstantDown", _time_constant_down);
	ReadIfPresent(sdf_clone, "rotorVelocitySlowdownSim", _rotor_velocity_slowdown_sim);
	ReadIfPresent(sdf_clone, "thrustVelocityCoupling", _k_th);
	ReadIfPresent(sdf_clone, "thrustAoaCoupling", _k_ts);
	ReadIfPresent(sdf_clone, "sideForceVelocityCoupling", _k_ns);
	ReadIfPresent(sdf_clone, "sideForceArmZ", _l_cpz);
	ReadIfPresent(sdf_clone, "thrustCenterOffsetCoeff", _k_cpx);
	ReadIfPresent(sdf_clone, "ductExpansionRatio", _duct_sd);
	ReadIfPresent(sdf_clone, "ductDiskArea", _duct_s);
	ReadIfPresent(sdf_clone, "airDensity", _air_density);
	ReadIfPresent(sdf_clone, "pitchDampingConstant", _k_my0);
	ReadIfPresent(sdf_clone, "pitchDampingVelocityCoeff", _k_myv);
	ReadIfPresent(sdf_clone, "ductTorqueCoeff", _k_sta);
	ReadIfPresent(sdf_clone, "fanInertia", _fan_inertia);
	ReadIfPresent(sdf_clone, "control_surface_force_coeff", _d_cs);
	ReadIfPresent(sdf_clone, "control_surface_arm_l1", _l1_cs);
	ReadIfPresent(sdf_clone, "control_surface_arm_l2", _l2_cs);

	if (sdf_clone->HasElement("turningDirection")) {
		const std::string direction = sdf_clone->Get<std::string>("turningDirection");
		_turning_direction = direction == "cw" ? -1 : 1;
	}

	if (sdf_clone->HasElement("commandSubTopic")) {
		_command_sub_topic = NormalizeCommandSubTopic(sdf_clone->Get<std::string>("commandSubTopic"));
	}

	if (sdf_clone->HasElement("ductSplineDtFile")) {
		LoadSplineFromCsv(sdf_clone->Get<std::string>("ductSplineDtFile"), _spline_dt);
	}

	if (sdf_clone->HasElement("ductSplineDnFile")) {
		LoadSplineFromCsv(sdf_clone->Get<std::string>("ductSplineDnFile"), _spline_dn);
	}

	if (sdf_clone->HasElement("wingSplineWlFile")) {
		LoadSplineFromCsv(sdf_clone->Get<std::string>("wingSplineWlFile"), _wing_spline_wl);
	}

	if (sdf_clone->HasElement("wingSplineWdFile")) {
		LoadSplineFromCsv(sdf_clone->Get<std::string>("wingSplineWdFile"), _wing_spline_wd);
	}

	if (sdf_clone->HasElement("wingSplineWmFile")) {
		LoadSplineFromCsv(sdf_clone->Get<std::string>("wingSplineWmFile"), _wing_spline_wm);
	}

	LoadControlJoints(sdf_clone);
	LoadControlEffectivenessMatrix(sdf_clone);
	SubscribeActuatorTopics(sdf_clone, ecm);
	SubscribeWindTopic(sdf_clone, ecm);
	SubscribeFailureTopics(sdf_clone, ecm);

	_rotor_velocity_filter = std::make_unique<FirstOrderFilter<double>>(
					 _time_constant_up, _time_constant_down, _ref_motor_rot_vel);

	ResolveEntities(ecm);
}

void DuctedFanPlugin::SubscribeActuatorTopics(const std::shared_ptr<const sdf::Element> &, gz::sim::EntityComponentManager &)
{
	const std::array<std::string, 2> topics{
		"/" + _model_name + "/" + _command_sub_topic,
		"/model/" + _model_name + "/" + _command_sub_topic
	};

	for (const auto &topic : topics) {
		_node.Subscribe(topic, &DuctedFanPlugin::ActuatorCallback, this);
		gzmsg << "[DuctedFanPlugin] Subscribed to " << topic << "\n";
	}
}

void DuctedFanPlugin::SubscribeWindTopic(const std::shared_ptr<const sdf::Element> &sdf,
		gz::sim::EntityComponentManager &ecm)
{
	const auto world_entity = gz::sim::worldEntity(ecm);
	const auto world = gz::sim::World(world_entity);
	const std::string world_name = world.Name(ecm).value_or("default");
	std::string wind_topic = "/world/" + world_name + "/wind_info";

	if (sdf->HasElement("windSubTopic")) {
		const std::string configured_topic = sdf->Get<std::string>("windSubTopic");

		if (!configured_topic.empty() && configured_topic.front() == '/') {
			wind_topic = configured_topic;
		}
	}

	_node.Subscribe(wind_topic, &DuctedFanPlugin::WindCallback, this);
}

void DuctedFanPlugin::SubscribeFailureTopics(const std::shared_ptr<const sdf::Element> &sdf,
		gz::sim::EntityComponentManager &)
{
	std::string failure_topic = "/model/" + _model_name + "/motor_failure/motor_number";

	if (sdf->HasElement("MotorFailureTopic")) {
		failure_topic = sdf->Get<std::string>("MotorFailureTopic");
	}

	_node.Subscribe(failure_topic, &DuctedFanPlugin::MotorFailureCallback, this);
	_node.Subscribe("/gazebo/motor_failure_num", &DuctedFanPlugin::MotorFailureCallback, this);
}

void DuctedFanPlugin::ResolveEntities(gz::sim::EntityComponentManager &ecm)
{
	if (_joint_entity == gz::sim::kNullEntity && !_joint_name.empty()) {
		_joint_entity = _model.JointByName(ecm, _joint_name);
	}

	if (_rotor_link_entity == gz::sim::kNullEntity && !_link_name.empty()) {
		_rotor_link_entity = _model.LinkByName(ecm, _link_name);
	}

	if (_base_link_entity == gz::sim::kNullEntity) {
		_base_link_entity = _model.LinkByName(ecm, "base_link");

		if (_base_link_entity != gz::sim::kNullEntity) {
			_base_link = gz::sim::Link(_base_link_entity);
			_base_link.EnableVelocityChecks(ecm, true);
		}
	}

	for (int i = 0; i < kControlJointCount; ++i) {
		if (_control_joint_entities[i] == gz::sim::kNullEntity && !_control_joint_names[i].empty()) {
			_control_joint_entities[i] = _model.JointByName(ecm, _control_joint_names[i]);
		}
	}
}

bool DuctedFanPlugin::EnsureComponents(gz::sim::EntityComponentManager &ecm)
{
	bool ready = true;

	if (_joint_entity == gz::sim::kNullEntity || _base_link_entity == gz::sim::kNullEntity) {
		return false;
	}

	if (!ecm.Component<gz::sim::components::JointVelocity>(_joint_entity)) {
		ecm.CreateComponent(_joint_entity, gz::sim::components::JointVelocity());
		ready = false;
	}

	if (!ecm.Component<gz::sim::components::JointVelocityCmd>(_joint_entity)) {
		ecm.CreateComponent(_joint_entity, gz::sim::components::JointVelocityCmd({0.0}));
		ready = false;
	}

	if (!ecm.Component<gz::sim::components::WorldPose>(_base_link_entity)) {
		ecm.CreateComponent(_base_link_entity, gz::sim::components::WorldPose());
		ready = false;
	}

	if (!ecm.Component<gz::sim::components::WorldLinearVelocity>(_base_link_entity)) {
		ecm.CreateComponent(_base_link_entity, gz::sim::components::WorldLinearVelocity());
		ready = false;
	}

	if (!ecm.Component<gz::sim::components::WorldAngularVelocity>(_base_link_entity)) {
		ecm.CreateComponent(_base_link_entity, gz::sim::components::WorldAngularVelocity());
		ready = false;
	}

	for (const auto &joint_entity : _control_joint_entities) {
		if (joint_entity != gz::sim::kNullEntity &&
		    !ecm.Component<gz::sim::components::JointPosition>(joint_entity)) {
			ecm.CreateComponent(joint_entity, gz::sim::components::JointPosition());
			ready = false;
		}
	}

	return ready;
}

void DuctedFanPlugin::PreUpdate(const gz::sim::UpdateInfo &info,
				gz::sim::EntityComponentManager &ecm)
{
	if (info.paused) {
		return;
	}

	ResolveEntities(ecm);

	if (!EnsureComponents(ecm)) {
		return;
	}

	_sampling_time = std::chrono::duration<double>(info.dt).count();

	if (_sampling_time <= 0.0) {
		return;
	}

	UpdateForcesAndMoments(info, ecm);
}

void DuctedFanPlugin::LoadControlJoints(const std::shared_ptr<const sdf::Element> &sdf)
{
	for (int i = 0; i < kControlJointCount; ++i) {
		const std::string tag = "control_joint_name_" + std::to_string(i + 1);

		if (sdf->HasElement(tag)) {
			_control_joint_names[i] = sdf->Get<std::string>(tag);
		}
	}
}

void DuctedFanPlugin::LoadControlEffectivenessMatrix(const std::shared_ptr<const sdf::Element> &sdf)
{
	for (auto &row : _b_cs) {
		row.fill(0.0);
	}

	if (!sdf->HasElement("control_effectiveness_matrix")) {
		return;
	}

	std::stringstream stream(sdf->Get<std::string>("control_effectiveness_matrix"));
	std::vector<double> values;
	double value = 0.0;

	while (stream >> value) {
		values.push_back(value);
	}

	if (values.size() != 18) {
		gzerr << "[DuctedFanPlugin] control_effectiveness_matrix expected 18 values, got "
		      << values.size() << "\n";
		return;
	}

	int index = 0;

	for (auto &row : _b_cs) {
		for (double &cell : row) {
			cell = values[index++];
		}
	}
}

void DuctedFanPlugin::ReadControlJointAngles(gz::sim::EntityComponentManager &ecm)
{
	for (int i = 0; i < kControlJointCount; ++i) {
		_control_joint_angles[i] = 0.0;

		if (_control_joint_entities[i] == gz::sim::kNullEntity) {
			continue;
		}

		const auto joint_position = ecm.Component<gz::sim::components::JointPosition>(_control_joint_entities[i]);

		if (joint_position && !joint_position->Data().empty()) {
			_control_joint_angles[i] = joint_position->Data()[0];
		}
	}
}

void DuctedFanPlugin::UpdateForcesAndMoments(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &ecm)
{
	const auto joint_velocity = ecm.Component<gz::sim::components::JointVelocity>(_joint_entity);

	if (!joint_velocity || joint_velocity->Data().empty()) {
		return;
	}

	const auto pose = _base_link.WorldPose(ecm);
	const auto linear_velocity = _base_link.WorldLinearVelocity(ecm);
	const auto angular_velocity = _base_link.WorldAngularVelocity(ecm);

	if (!pose.has_value() || !linear_velocity.has_value() || !angular_velocity.has_value()) {
		return;
	}

	const double motor_rot_vel = joint_velocity->Data()[0];
	const double real_motor_velocity = motor_rot_vel * _rotor_velocity_slowdown_sim;
	const double omega = std::abs(real_motor_velocity);

	if (_sampling_time > 0.0 && omega / (2.0 * GZ_PI) > 1.0 / (2.0 * _sampling_time)) {
		gzwarn << "[DuctedFanPlugin] Aliasing on motor " << _motor_number
		       << ". Consider smaller simulation time steps or a larger rotorVelocitySlowdownSim.\n";
	}

	ReadControlJointAngles(ecm);

	const auto airspeed_frd = FluToFrd(pose->Rot().RotateVectorReverse(*linear_velocity - _wind_velocity));
	const auto rates_frd = FluToFrd(pose->Rot().RotateVectorReverse(*angular_velocity));
	const double u = airspeed_frd.X();
	const double v = airspeed_frd.Y();
	const double w = airspeed_frd.Z();
	const double p = rates_frd.X();
	const double q = rates_frd.Y();

	const double v_xz = std::sqrt(u * u + w * w);
	const double v_xy = std::sqrt(u * u + v * v);
	const double speed = std::sqrt(u * u + v * v + w * w);
	const double inv_v_xy = v_xy > 1e-12 ? 1.0 / v_xy : 0.0;
	const double x_dir = v_xy > 1e-12 ? u * inv_v_xy : 1.0;
	const double y_dir = v_xy > 1e-12 ? v * inv_v_xy : 0.0;

	double alpha = 0.0;

	if (v_xz <= 1.0) {
		alpha = 0.5 * GZ_PI;

	} else {
		alpha = std::acos(std::clamp(-w / v_xz, -1.0, 1.0));
	}

	const double duct_cos_aoa = speed > 1e-6 ? std::clamp(-w / speed, -1.0, 1.0) : 0.0;
	const double duct_sin_aoa = speed > 1e-6 ? v_xy / speed : 1.0;
	const double duct_aoa = speed > 1e-6 ? std::acos(duct_cos_aoa) : 0.5 * GZ_PI;

	const double dt = _spline_dt.empty() ? 0.0 : PpvalSpline(_spline_dt, duct_aoa, true);
	const double dn = _spline_dn.empty() ? 0.0 : PpvalSpline(_spline_dn, duct_aoa, true);
	const double omega2 = omega * omega;

	double duct_thrust = _motor_constant * omega2
			     + speed * omega * (_k_th + _k_ts * duct_cos_aoa)
			     + speed * speed * dt;

	if (!_reversible) {
		duct_thrust = std::abs(duct_thrust);
	}

	const double duct_side_force = speed * omega * _k_ns * duct_sin_aoa + speed * speed * dn;
	double duct_pitch_moment = duct_side_force * _l_cpz;

	if (omega > 1e-6) {
		duct_pitch_moment += duct_thrust * _k_cpx * speed / omega * duct_sin_aoa;
	}

	const double wl = _wing_spline_wl.empty() ? 0.0 : PpvalSpline(_wing_spline_wl, alpha, true);
	const double wd = _wing_spline_wd.empty() ? 0.0 : PpvalSpline(_wing_spline_wd, alpha, true);
	const double wm = _wing_spline_wm.empty() ? 0.0 : PpvalSpline(_wing_spline_wm, alpha, true);
	const double v_xz2 = v_xz * v_xz;
	const double wing_force_x = v_xz2 * wl;
	const double wing_force_z = v_xz2 * wd;
	const double wing_pitch_moment = v_xz2 * wm;
	const double ve_root = std::max(0.0, 0.25 * w * w + duct_thrust / (_duct_sd * _air_density * _duct_s));
	const double ve = -0.5 * w + std::sqrt(ve_root);
	const double rotor_torque = _moment_constant * omega2;

	const gz::math::Vector3d force_frd(
		-duct_side_force * x_dir - wing_force_x,
		-duct_side_force * y_dir,
		-duct_thrust - wing_force_z);

	std::array<double, 3> m_cs{0.0, 0.0, 0.0};
	const std::array<double, 3> lever{_l1_cs, _l1_cs, _l2_cs};

	for (int row = 0; row < 3; ++row) {
		for (int col = 0; col < kControlJointCount; ++col) {
			m_cs[row] += _b_cs[row][col] * _control_joint_angles[col];
		}

		m_cs[row] *= _d_cs * ve * ve * lever[row];
	}

	const gz::math::Vector3d torque_frd(
		-duct_pitch_moment * y_dir - _turning_direction * _fan_inertia * omega * q + m_cs[0],
		duct_pitch_moment * x_dir + wing_pitch_moment - q * (_k_my0 + _k_myv * v_xz)
		+ _turning_direction * _fan_inertia * omega * p + m_cs[1],
		-_turning_direction * rotor_torque + _turning_direction * _k_sta * ve * ve + m_cs[2]);

	const auto force_world = pose->Rot().RotateVector(FrdToFlu(force_frd));
	const auto torque_world = pose->Rot().RotateVector(FrdToFlu(torque_frd));
	_base_link.AddWorldWrench(ecm, force_world, torque_world);

	double ref_motor_rot_vel = 0.0;

	{
		std::lock_guard<std::mutex> lock(_command_mutex);
		ref_motor_rot_vel = _ref_motor_rot_vel;
	}

	ref_motor_rot_vel = _rotor_velocity_filter->Update(ref_motor_rot_vel, _sampling_time);

	if (_motor_failure_number == _motor_number + 1) {
		ref_motor_rot_vel = 0.0;
	}

	const auto joint_velocity_cmd = ecm.Component<gz::sim::components::JointVelocityCmd>(_joint_entity);

	if (joint_velocity_cmd) {
		*joint_velocity_cmd = gz::sim::components::JointVelocityCmd({
			_turning_direction * ref_motor_rot_vel / _rotor_velocity_slowdown_sim
		});
	}
}

void DuctedFanPlugin::ActuatorCallback(const gz::msgs::Actuators &msg)
{
	if (_motor_number > msg.velocity_size() - 1) {
		return;
	}

	std::lock_guard<std::mutex> lock(_command_mutex);
	_ref_motor_rot_vel = std::min(static_cast<double>(msg.velocity(_motor_number)), _max_rot_velocity);
}

void DuctedFanPlugin::MotorFailureCallback(const gz::msgs::Int32 &msg)
{
	_motor_failure_number = msg.data();
}

void DuctedFanPlugin::WindCallback(const gz::msgs::Wind &msg)
{
	_wind_velocity.Set(msg.linear_velocity().x(), msg.linear_velocity().y(), msg.linear_velocity().z());
}

GZ_ADD_PLUGIN(
	DuctedFanPlugin,
	gz::sim::System,
	DuctedFanPlugin::ISystemConfigure,
	DuctedFanPlugin::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(DuctedFanPlugin, "px4::DuctedFanPlugin")
