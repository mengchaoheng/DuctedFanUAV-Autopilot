/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "TorqueDisturbancePlugin.hpp"

#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Pose.hh>

#include <algorithm>
#include <array>
#include <cctype>
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

std::string ToLower(std::string value)
{
	std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char character) {
		return static_cast<char>(std::tolower(character));
	});
	return value;
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

	std::string mode{"sine"};
	ReadIfPresent(sdf, "mode", mode);
	mode = ToLower(mode);

	if (mode == "gauss_markov" || mode == "random") {
		_mode = Mode::GaussMarkov;

	} else if (mode == "constant") {
		_mode = Mode::Constant;

	} else if (mode != "sine") {
		gzwarn << "[TorqueDisturbancePlugin] Unknown mode '" << mode
		       << "'; using sine.\n";
	}

	std::string frame{"body_flu"};
	ReadIfPresent(sdf, "frame", frame);

	const auto parse_frame = [](std::string value, const Frame fallback, const char *element_name) {
		value = ToLower(value);

		if (value == "body_flu") {
			return Frame::BodyFlu;

		} else if (value == "body_frd") {
			return Frame::BodyFrd;

		} else if (value == "world" || value == "world_enu") {
			return Frame::WorldEnu;

		} else if (value == "world_ned") {
			return Frame::WorldNed;
		}

		gzwarn << "[TorqueDisturbancePlugin] Unknown " << element_name << " '" << value
		       << "'; using the legacy frame setting.\n";
		return fallback;
	};

	const Frame common_frame = parse_frame(frame, Frame::BodyFlu, "frame");
	_torque_frame = common_frame;
	_force_frame = common_frame;

	std::string torque_frame{frame};
	std::string force_frame{frame};
	ReadIfPresent(sdf, "torque_frame", torque_frame);
	ReadIfPresent(sdf, "force_frame", force_frame);
	_torque_frame = parse_frame(torque_frame, common_frame, "torque_frame");
	_force_frame = parse_frame(force_frame, common_frame, "force_frame");

	ReadIfPresent(sdf, "amplitude_x", _amplitude_x);
	ReadIfPresent(sdf, "amplitude_y", _amplitude_y);
	ReadIfPresent(sdf, "amplitude_z", _amplitude_z);
	ReadIfPresent(sdf, "frequency_x", _frequency_x);
	ReadIfPresent(sdf, "frequency_y", _frequency_y);
	ReadIfPresent(sdf, "frequency_z", _frequency_z);
	ReadIfPresent(sdf, "phase_x", _phase_x);
	ReadIfPresent(sdf, "phase_y", _phase_y);
	ReadIfPresent(sdf, "phase_z", _phase_z);
	ReadIfPresent(sdf, "bias_x", _bias_x);
	ReadIfPresent(sdf, "bias_y", _bias_y);
	ReadIfPresent(sdf, "bias_z", _bias_z);
	ReadIfPresent(sdf, "force_amplitude_x", _force_amplitude_x);
	ReadIfPresent(sdf, "force_amplitude_y", _force_amplitude_y);
	ReadIfPresent(sdf, "force_amplitude_z", _force_amplitude_z);
	ReadIfPresent(sdf, "force_frequency_x", _force_frequency_x);
	ReadIfPresent(sdf, "force_frequency_y", _force_frequency_y);
	ReadIfPresent(sdf, "force_frequency_z", _force_frequency_z);
	ReadIfPresent(sdf, "force_phase_x", _force_phase_x);
	ReadIfPresent(sdf, "force_phase_y", _force_phase_y);
	ReadIfPresent(sdf, "force_phase_z", _force_phase_z);
	ReadIfPresent(sdf, "force_bias_x", _force_bias_x);
	ReadIfPresent(sdf, "force_bias_y", _force_bias_y);
	ReadIfPresent(sdf, "force_bias_z", _force_bias_z);
	ReadIfPresent(sdf, "start_time", _start_time_sec);
	ReadIfPresent(sdf, "running_time", _running_time_sec);
	ReadIfPresent(sdf, "repeat_count", _repeat_count);
	ReadIfPresent(sdf, "repeat_interval", _repeat_interval_sec);
	ReadIfPresent(sdf, "seed", _seed);

	double common_correlation_time{1.0};
	ReadIfPresent(sdf, "correlation_time", common_correlation_time);
	_torque_correlation_time_sec.fill(common_correlation_time);
	_force_correlation_time_sec.fill(common_correlation_time);

	std::array<double, 3> per_axis_correlation_time{{common_correlation_time, common_correlation_time,
			common_correlation_time}};
	ReadIfPresent(sdf, "correlation_time_x", per_axis_correlation_time[0]);
	ReadIfPresent(sdf, "correlation_time_y", per_axis_correlation_time[1]);
	ReadIfPresent(sdf, "correlation_time_z", per_axis_correlation_time[2]);
	_torque_correlation_time_sec = per_axis_correlation_time;
	_force_correlation_time_sec = per_axis_correlation_time;

	ReadIfPresent(sdf, "torque_correlation_time_x", _torque_correlation_time_sec[0]);
	ReadIfPresent(sdf, "torque_correlation_time_y", _torque_correlation_time_sec[1]);
	ReadIfPresent(sdf, "torque_correlation_time_z", _torque_correlation_time_sec[2]);
	ReadIfPresent(sdf, "moment_correlation_time_x", _torque_correlation_time_sec[0]);
	ReadIfPresent(sdf, "moment_correlation_time_y", _torque_correlation_time_sec[1]);
	ReadIfPresent(sdf, "moment_correlation_time_z", _torque_correlation_time_sec[2]);
	ReadIfPresent(sdf, "force_correlation_time_x", _force_correlation_time_sec[0]);
	ReadIfPresent(sdf, "force_correlation_time_y", _force_correlation_time_sec[1]);
	ReadIfPresent(sdf, "force_correlation_time_z", _force_correlation_time_sec[2]);
	ReadIfPresent(sdf, "wrench_topic", _wrench_topic);
	ReadIfPresent(sdf, "publish_rate", _publish_rate_hz);

	_repeat_count = std::max(_repeat_count, 1);
	_repeat_interval_sec = std::max(_repeat_interval_sec, 0.0);
	_random_engine.seed(_seed);
	_standard_normal.reset();

	for (std::size_t axis = 0; axis < 3; ++axis) {
		if (_torque_correlation_time_sec[axis] <= 0.0) {
			gzwarn << "[TorqueDisturbancePlugin] torque correlation time for axis " << axis
			       << " must be positive; using 1 s.\n";
			_torque_correlation_time_sec[axis] = 1.0;
		}

		if (_force_correlation_time_sec[axis] <= 0.0) {
			gzwarn << "[TorqueDisturbancePlugin] force correlation time for axis " << axis
			       << " must be positive; using 1 s.\n";
			_force_correlation_time_sec[axis] = 1.0;
		}
	}

	if (_publish_rate_hz < 0.0) {
		gzwarn << "[TorqueDisturbancePlugin] publish_rate must be non-negative; "
		       << "using 0 (publish every active simulation step).\n";
		_publish_rate_hz = 0.0;
	}

	_link_entity = _model.LinkByName(ecm, _link_name);

	if (_link_entity == gz::sim::kNullEntity) {
		gzerr << "[TorqueDisturbancePlugin] Link " << _link_name << " not found.\n";
		return;
	}

	_link = gz::sim::Link(_link_entity);

	if (_wrench_topic.empty()) {
		_wrench_topic = "/model/" + _model.Name(ecm) + "/disturbance_wrench";
	}

	_wrench_publisher = _node.Advertise<gz::msgs::Wrench>(_wrench_topic);

	if (!_wrench_publisher.Valid()) {
		gzwarn << "[TorqueDisturbancePlugin] Failed to advertise gz.msgs.Wrench on "
		       << _wrench_topic << ". The disturbance will still be applied.\n";
	}
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

	if (_running_time_sec <= 0.0 || t < _start_time_sec) {
		return;
	}

	const double elapsed_time = t - _start_time_sec;
	const double cycle_time = _running_time_sec + _repeat_interval_sec;

	if (cycle_time <= 0.0) {
		return;
	}

	const int repeat_index = static_cast<int>(elapsed_time / cycle_time);

	if (repeat_index >= _repeat_count) {
		return;
	}

	const double disturbance_time = elapsed_time - repeat_index * cycle_time;

	if (disturbance_time > _running_time_sec) {
		return;
	}

	if (repeat_index != _last_repeat_index) {
		_last_repeat_index = repeat_index;
		_gauss_markov_initialized = false;
		gzmsg << "[TorqueDisturbancePlugin] Disturbance repeat "
		      << repeat_index + 1 << " / " << _repeat_count
		      << ", sim time: " << t
		      << ", disturbance local time: " << disturbance_time << " s\n";
	}

	gz::math::Vector3d torque;
	gz::math::Vector3d force;

	if (_mode == Mode::GaussMarkov) {
		// Reuse the sine amplitudes as stationary standard deviations in this mode.
		const std::array<double, 3> torque_standard_deviation{
			std::abs(_amplitude_x), std::abs(_amplitude_y), std::abs(_amplitude_z)
		};
		const std::array<double, 3> force_standard_deviation{
			std::abs(_force_amplitude_x), std::abs(_force_amplitude_y), std::abs(_force_amplitude_z)
		};

		if (!_gauss_markov_initialized) {
			for (std::size_t axis = 0; axis < 3; ++axis) {
				_torque_gauss_markov_state[axis] = torque_standard_deviation[axis] * _standard_normal(_random_engine);
				_force_gauss_markov_state[axis] = force_standard_deviation[axis] * _standard_normal(_random_engine);
			}

			_gauss_markov_initialized = true;

		} else {
			const double dt = std::chrono::duration<double>(info.dt).count();

			if (dt > 0.0) {
				for (std::size_t axis = 0; axis < 3; ++axis) {
					const double torque_state_transition = std::exp(-dt / _torque_correlation_time_sec[axis]);
					const double torque_innovation_scale = std::sqrt(std::max(0.0,
							1.0 - torque_state_transition * torque_state_transition));
					const double force_state_transition = std::exp(-dt / _force_correlation_time_sec[axis]);
					const double force_innovation_scale = std::sqrt(std::max(0.0,
							1.0 - force_state_transition * force_state_transition));

					_torque_gauss_markov_state[axis] = torque_state_transition * _torque_gauss_markov_state[axis]
						+ torque_standard_deviation[axis] * torque_innovation_scale * _standard_normal(_random_engine);
					_force_gauss_markov_state[axis] = force_state_transition * _force_gauss_markov_state[axis]
						+ force_standard_deviation[axis] * force_innovation_scale * _standard_normal(_random_engine);
				}
			}
		}

		torque.Set(_bias_x + _torque_gauss_markov_state[0],
			   _bias_y + _torque_gauss_markov_state[1],
			   _bias_z + _torque_gauss_markov_state[2]);
		force.Set(_force_bias_x + _force_gauss_markov_state[0],
			  _force_bias_y + _force_gauss_markov_state[1],
			  _force_bias_z + _force_gauss_markov_state[2]);

	} else if (_mode == Mode::Constant) {
		torque.Set(_bias_x + _amplitude_x, _bias_y + _amplitude_y, _bias_z + _amplitude_z);
		force.Set(_force_bias_x + _force_amplitude_x,
			  _force_bias_y + _force_amplitude_y,
			  _force_bias_z + _force_amplitude_z);

	} else {
		torque.Set(_bias_x + _amplitude_x * std::sin(2.0 * GZ_PI * _frequency_x * disturbance_time + _phase_x),
			   _bias_y + _amplitude_y * std::sin(2.0 * GZ_PI * _frequency_y * disturbance_time + _phase_y),
			   _bias_z + _amplitude_z * std::sin(2.0 * GZ_PI * _frequency_z * disturbance_time + _phase_z));
		force.Set(_force_bias_x
			  + _force_amplitude_x * std::sin(2.0 * GZ_PI * _force_frequency_x * disturbance_time + _force_phase_x),
			  _force_bias_y
			  + _force_amplitude_y * std::sin(2.0 * GZ_PI * _force_frequency_y * disturbance_time + _force_phase_y),
			  _force_bias_z
			  + _force_amplitude_z * std::sin(2.0 * GZ_PI * _force_frequency_z * disturbance_time + _force_phase_z));
	}

	const auto pose = _link.WorldPose(ecm);

	if (!pose.has_value()) {
		return;
	}

	const auto to_world_enu = [&pose](gz::math::Vector3d vector, const Frame frame) {
		if (frame == Frame::WorldEnu) {
			return vector;

		} else if (frame == Frame::WorldNed) {
			// Gazebo world is ENU, while PX4 and the benchmark use NED.
			return gz::math::Vector3d(vector.Y(), vector.X(), -vector.Z());

		} else if (frame == Frame::BodyFrd) {
			// Gazebo body is FLU, while PX4 body is FRD.
			vector.Set(vector.X(), -vector.Y(), -vector.Z());
		}

		return pose->Rot().RotateVector(vector);
	};

	const gz::math::Vector3d force_world = to_world_enu(force, _force_frame);
	const gz::math::Vector3d torque_world = to_world_enu(torque, _torque_frame);

	_link.AddWorldForce(ecm, force_world);
	_link.AddWorldWrench(ecm, gz::math::Vector3d::Zero, torque_world);

	const bool publish_due = !_has_last_publish_time
				 || sim_time < _last_publish_time_sec
				 || _publish_rate_hz <= 0.0
				 || sim_time - _last_publish_time_sec >= 1.0 / _publish_rate_hz;

	if (_wrench_publisher.Valid() && publish_due) {
		// Gazebo receives a world-frame wrench, so publish that exact representation.
		gz::msgs::Wrench wrench;
		gz::msgs::Set(wrench.mutable_header()->mutable_stamp(), info.simTime);
		wrench.mutable_force()->set_x(force_world.X());
		wrench.mutable_force()->set_y(force_world.Y());
		wrench.mutable_force()->set_z(force_world.Z());
		wrench.mutable_torque()->set_x(torque_world.X());
		wrench.mutable_torque()->set_y(torque_world.Y());
		wrench.mutable_torque()->set_z(torque_world.Z());
		_wrench_publisher.Publish(wrench);
		_last_publish_time_sec = sim_time;
		_has_last_publish_time = true;
	}
}

GZ_ADD_PLUGIN(
	TorqueDisturbancePlugin,
	gz::sim::System,
	TorqueDisturbancePlugin::ISystemConfigure,
	TorqueDisturbancePlugin::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(TorqueDisturbancePlugin, "px4::TorqueDisturbancePlugin")
