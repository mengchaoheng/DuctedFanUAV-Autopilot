/**
 * Copyright 2026 Chaoheng Meng
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "OmMpcAccIndiControl.hpp"
#include <lib/controllib/OmMpcEnable.hpp>

#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/mathlib/math/WelfordMean.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/ringbuffer/TimestampedRingBuffer.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/allocation_value.h>
#include <uORB/topics/om_mpc_disturbance.h>
#include <uORB/topics/om_mpc_command.h>
#include <uORB/topics/om_mpc_acc_indi_command.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class McOmMpcAccIndi final : public ModuleBase, public ModuleParams, public px4::WorkItem
{
public:
	static Descriptor desc;

	McOmMpcAccIndi();
	~McOmMpcAccIndi() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	void Run() override;

private:
	struct AccelerationFilterState {
		matrix::Vector3f value{NAN, NAN, NAN};
		uint64_t timestamp{0};
		bool initialized{false};
		float sample_hz{0.f};
		float cutoff_hz{0.f};
		math::WelfordMean<float> sample_interval_s{};
		math::LowPassFilter2p<matrix::Vector3f> filter{};
	};

	struct ForceSample {
		uint64_t time_us{0};
		matrix::Vector3f allocated_force_ned{NAN, NAN, NAN};
	};

	static constexpr size_t kForceHistoryLength{128};

	void parametersUpdated();
	void updateEnableState(bool middle_layer_enabled);
	void updateAccelerationFilter(const matrix::Vector3f &raw, uint64_t timestamp,
		AccelerationFilterState &state);
	void updateSensorInputs();
	void updateForceHistory();
	bool getDelayedAllocatedForce(uint64_t reference_timestamp,
		matrix::Vector3f &allocated_force_ned);
	matrix::Dcmf attitudeAt(uint64_t timestamp, const matrix::Dcmf &attitude) const;
	void resetTransition();
	void publishDisturbance(uint64_t now, uint64_t acceleration_timestamp, bool valid,
		const matrix::Vector3f &disturbance);

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::Subscription _onboard_command_sub{ORB_ID(om_mpc_command)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _allocation_value_sub{ORB_ID(allocation_value)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
	rc_channels_s _rc_channels{};

	uORB::Publication<om_mpc_disturbance_s> _disturbance_pub{ORB_ID(om_mpc_disturbance)};
	uORB::Publication<om_mpc_acc_indi_command_s> _onboard_correction_pub{ORB_ID(om_mpc_acc_indi_command)};

	vehicle_attitude_s _vehicle_attitude{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	om_mpc_command_s _onboard_command{};
	bool _onboard_enabled{false};
	bool _middle_layer_enabled_previous{false};
	const param_t _onboard_enable_handle{param_find("MPC_EN")};
	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_land_detected_s _vehicle_land_detected{};
	vehicle_status_s _vehicle_status{};

	AccelerationFilterState _ekf_acceleration_filter{};
	AccelerationFilterState _imu_acceleration_filter{};
	AccelerationFilterState _force_filter{};
	TimestampedRingBuffer<ForceSample, kForceHistoryLength> _force_history{};
	uint64_t _force_history_last_timestamp{0};
	uint64_t _force_feedback_timestamp{0};

	OmMpcAccIndiControl _control{};
	bool _acceleration_active_previous{false};
	float _transition_elapsed{0.f};
	float _transition_progress{0.f};
	uint64_t _last_run_timestamp{0};

	perf_counter_t _loop_perf{nullptr};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MPC_ACCINDI_EN>) _param_enable,
		(ParamInt<px4::params::MPC_DFB_EN>) _param_disturbance_enable,
		(ParamInt<px4::params::MPC_ACCINDI_SRC>) _param_acceleration_source,
		(ParamFloat<px4::params::MPC_ACCINDI_LPF>) _param_acceleration_cutoff,
		(ParamFloat<px4::params::MPC_DFB_LIM>) _param_disturbance_limit,
		(ParamFloat<px4::params::MPC_ACCINDI_FDLY>) _param_force_delay,
		(ParamFloat<px4::params::MPC_ACCINDI_TR_T>) _param_transition_time,
		(ParamFloat<px4::params::MPC_ACCINDI_K_R>) _param_kphi_roll,
		(ParamFloat<px4::params::MPC_ACCINDI_K_P>) _param_kphi_pitch,
		(ParamFloat<px4::params::MPC_ACCINDI_K_Y>) _param_kphi_yaw
	)
};
