/**
 * Copyright 2026 Chaoheng Meng
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "OmMpcIndiControl.hpp"

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
#include <uORB/topics/om_mpc_indi_status.h>
#include <uORB/topics/om_mpc_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class McOmMpcIndi final : public ModuleBase, public ModuleParams, public px4::WorkItem
{
public:
	static Descriptor desc;

	McOmMpcIndi();
	~McOmMpcIndi() override;

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
	void updateAccelerationFilter(const matrix::Vector3f &raw, uint64_t timestamp,
		AccelerationFilterState &state);
	void updateSensorInputs();
	void updateForceHistory();
	bool getDelayedAllocatedForce(uint64_t reference_timestamp,
		matrix::Vector3f &allocated_force_ned);
	matrix::Dcmf attitudeAt(uint64_t timestamp, const matrix::Dcmf &attitude) const;
	void resetTransition();
	void publishStatus(uint64_t now, uint64_t acceleration_timestamp,
		bool acceleration_active, bool disturbance_valid,
		const matrix::Vector3f &disturbance, const matrix::Vector3f &acceleration_ned,
		const matrix::Vector3f &allocated_force_ned, const matrix::Vector3f &nominal_rates,
		const OmMpcIndiControl::Output &corrected);

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::Subscription _om_mpc_setpoint_sub{ORB_ID(om_mpc_setpoint)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _allocation_value_sub{ORB_ID(allocation_value)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _rate_ctrl_status_sub{ORB_ID(rate_ctrl_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::Publication<vehicle_rates_setpoint_s> _setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<om_mpc_indi_status_s> _status_pub{ORB_ID(om_mpc_indi_status)};

	vehicle_attitude_s _vehicle_attitude{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	om_mpc_setpoint_s _nominal_setpoint{};
	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_land_detected_s _vehicle_land_detected{};
	vehicle_status_s _vehicle_status{};
	rate_ctrl_status_s _rate_ctrl_status{};

	AccelerationFilterState _ekf_acceleration_filter{};
	AccelerationFilterState _imu_acceleration_filter{};
	AccelerationFilterState _allocated_force_filter{};
	TimestampedRingBuffer<ForceSample, kForceHistoryLength> _force_history{};
	uint64_t _force_history_last_timestamp{0};

	OmMpcIndiControl _control{};
	bool _acceleration_active_previous{false};
	float _transition_elapsed{0.f};
	float _transition_progress{0.f};
	uint64_t _last_run_timestamp{0};
	uint64_t _last_status_publish{0};
	uint64_t _last_control_acceleration_timestamp{0};
	int32_t _last_control_acceleration_source{0};

	perf_counter_t _loop_perf{nullptr};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MC_OM_INDI_EN>) _param_enable,
		(ParamInt<px4::params::MC_OM_ACC_EN>) _param_acceleration_enable,
		(ParamInt<px4::params::MC_OM_DIST_EN>) _param_disturbance_enable,
		(ParamFloat<px4::params::MC_OM_RATE_MAX>) _param_rate_max,
		(ParamInt<px4::params::MC_OM_A_SRC>) _param_acceleration_source,
		(ParamFloat<px4::params::MC_OM_A_LP>) _param_acceleration_cutoff,
		(ParamFloat<px4::params::MC_OM_D_LIM>) _param_disturbance_limit,
		(ParamFloat<px4::params::MC_OM_F_DLY>) _param_force_delay,
		(ParamFloat<px4::params::MC_OM_TR_T>) _param_transition_time,
		(ParamFloat<px4::params::MC_OM_KPHI_R>) _param_kphi_roll,
		(ParamFloat<px4::params::MC_OM_KPHI_P>) _param_kphi_pitch,
		(ParamFloat<px4::params::MC_OM_KPHI_Y>) _param_kphi_yaw,
		(ParamFloat<px4::params::MC_OM_HOVER>) _param_hover_thrust
	)
};
