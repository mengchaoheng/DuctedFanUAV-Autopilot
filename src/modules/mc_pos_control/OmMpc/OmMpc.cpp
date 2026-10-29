// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
#include "OmMpc.hpp"
#include "OmMpcModel.hpp"
#include "OmSolverRuntime.hpp"
#include <px4_platform_common/log.h>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/om_mpc_reference.h>
#include <uORB/topics/om_mpc_command.h>
#include <uORB/topics/om_mpc_status.h>
#include <uORB/topics/om_mpc_indi_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <lib/controllib/OmMpcEnable.hpp>
#include <time.h>
#include <unistd.h>
using matrix::Vector3f;
namespace
{
#ifdef OM_MPC_OFFLINE_BENCHMARK
#ifndef __PX4_POSIX
#error "Offline MPC timing is only available for POSIX simulation"
#endif
// Explicit, non-default research build: lockstep waits for the optimization.
// Convergence, feasibility, state validity and command ownership stay checked.
static constexpr bool enforce_wall_deadline = false;
#else
static constexpr bool enforce_wall_deadline = true;
#endif
float parameter(param_t handle, float fallback)
{
	float v = fallback;
	param_get(handle, &v);
	return v;
}

int setting(param_t handle)
{
	int32_t v = 0;
	param_get(handle, &v);
	return v;
}

// Status prints are infrequent; the worker uses handles resolved once.
int setting(const char *name)
{
	return setting(param_find(name));
}

uint64_t wall_us()
{
#ifdef __PX4_NUTTX
	return hrt_absolute_time();
#else
	timespec t {};
	clock_gettime(CLOCK_MONOTONIC, &t);
	return uint64_t(t.tv_sec) * 1000000 + t.tv_nsec / 1000;
#endif
}

}

// This private queue never serializes a QP with another controller's Run().
// NuttX priority 236: below every default EKF queue (241..238) and
// hp_default (237), above logger (230) and lp_default/autosave (205).
// The same -19 offset is within Darwin's narrower FIFO range as well.
static constexpr int8_t om_priority = -19;
static_assert(om_priority < px4::wq_configurations::INS0.relative_priority
	      && om_priority < px4::wq_configurations::INS1.relative_priority
	      && om_priority < px4::wq_configurations::INS2.relative_priority
	      && om_priority < px4::wq_configurations::INS3.relative_priority,
	      "OM-MPC must run below all EKF work queues");
static constexpr px4::wq_config_t om_queue{"wq:om_mpc", 8192, om_priority};
struct OmMpc::Runtime {
	// Resolve handles once; cache model settings on parameter_update.
	// Enable/rate remain inexpensive live reads on every release.
	const param_t h_mpc_om_rate = param_find("MPC_UPD_MODE");
	const param_t h_mpc_om_en = param_find("MPC_EN");
	const param_t h_mpc_thr_hover = param_find("MPC_THR_HOVER");
	const param_t h_mpc_thr_max = param_find("MPC_THR_MAX");
	const param_t h_mc_rollrate_max = param_find("MC_ROLLRATE_MAX");
	const param_t h_mc_pitchrate_max = param_find("MC_PITCHRATE_MAX");
	const param_t h_mc_yawrate_max = param_find("MC_YAWRATE_MAX");
	const param_t h_mpc_om_ep = param_find("MPC_ENTRY_POS");
	const param_t h_mpc_om_ev = param_find("MPC_ENTRY_VEL");
	const param_t h_mpc_om_er = param_find("MPC_ENTRY_ROT");
	const param_t h_mpc_om_n = param_find("MPC_N");
	const param_t h_mpc_om_dt = param_find("MPC_DT");
	const param_t h_mpc_om_iter = param_find("MPC_QP_ITMAX");
	const param_t h_mpc_om_d_gain = param_find("MPC_DFB_GAIN");
	const param_t h_cost[6] {param_find("MPC_QPOS"), param_find("MPC_QVEL"), param_find("MPC_QROT"),
		param_find("MPC_RTHR"), param_find("MPC_ROMG"), param_find("MPC_TERM_SCALE")};
	float cost[6] {324.f, 10.f, 4.32f, .25f, .8f, 1.f};
	uORB::Subscription parameter_sub{ORB_ID(parameter_update)};
	uORB::Subscription rc_sub{ORB_ID(rc_channels)};
	rc_channels_s rc{};
	uORB::Subscription hover_sub{ORB_ID(hover_thrust_estimate)};
	bool config_valid{false}, model_initialized{false}, horizon_supported{true};
	int prediction_horizon{15};
	float prediction_dt{.04f}, disturbance_gain{0.f};
	float hover{.2163f}, configured_hover{.2163f}, max_normalized{1.f}, max_thrust{0.f};
	Vector3f rate_limits{}, entry_limits{};
	int iteration_limit{100};
	void update_config()
	{
		if (!config_valid || parameter_sub.updated()) {
			parameter_update_s update{};
			parameter_sub.copy(&update);
			const int configured_n = setting(h_mpc_om_n);
#ifdef __PX4_POSIX
			horizon_supported = configured_n >= 1 && configured_n <= OmMaxHorizon;
#else
			horizon_supported = configured_n == 10 || configured_n == 15 || configured_n == 20;
#endif
			const int next_n = horizon_supported ? configured_n : 15;
			const float next_dt = parameter(h_mpc_om_dt, .04f);
			float next_cost[6];
			bool cost_valid = true;

			for (int i = 0; i < 6; ++i) {
				next_cost[i] = parameter(h_cost[i], cost[i]);
				cost_valid &= std::isfinite(next_cost[i]) && next_cost[i] > 0.f;
			}

			horizon_supported &= cost_valid;

			if (config_valid && (next_n != prediction_horizon || memcmp(&next_dt, &prediction_dt, sizeof(next_dt)) != 0
					     || memcmp(next_cost, cost, sizeof(cost)) != 0)) {
				om_float_close();
				workspace_ready = false;
				engaged = false;
				model_initialized = false;
			}

			memcpy(cost, next_cost, sizeof(cost));
			prediction_horizon = next_n;
			prediction_dt = next_dt;
			disturbance_gain = fminf(fmaxf(parameter(h_mpc_om_d_gain, 0.f), 0.f), 1.f);
			configured_hover = parameter(h_mpc_thr_hover, .2163f);
			max_normalized = parameter(h_mpc_thr_max, 1.f);
			rate_limits = {parameter(h_mc_rollrate_max, 1145.9156f) *.01745329252f,
				       parameter(h_mc_pitchrate_max, 1145.9156f) *.01745329252f,
				       parameter(h_mc_yawrate_max, 458.3662f) *.01745329252f
				      };
			entry_limits = {parameter(h_mpc_om_ep, 1.f), parameter(h_mpc_om_ev, 2.f),
					parameter(h_mpc_om_er, 35.f) *.01745329252f
				       };
			iteration_limit = setting(h_mpc_om_iter);

			if (iteration_limit < 1 || iteration_limit > 100) {
				iteration_limit = 100;
			}

			config_valid = true;
		}

		// Follow valid HTE updates without rebuilding the QP workspace. Bounds
		// are updated by build(); a stopped/invalid estimator uses the parameter.
		hover_thrust_estimate_s estimate{};
		hover = hover_sub.copy(&estimate) && estimate.valid && estimate.timestamp != 0
			&& hrt_elapsed_time(&estimate.timestamp) < 1000000
			&& std::isfinite(estimate.hover_thrust) && estimate.hover_thrust > .01f
			&& estimate.hover_thrust <= 1.f ? estimate.hover_thrust : configured_hover;
		max_thrust = hover > .01f ? 9.80665f * max_normalized / hover : 0.f;
	}

	OmCpuMonitor cpu_monitor;
	uORB::Subscription ref_sub {ORB_ID(om_mpc_reference)}, pos_sub {ORB_ID(vehicle_local_position)}, att_sub {ORB_ID(vehicle_attitude)},
	     disturbance_sub{ORB_ID(om_mpc_indi_status)},
	     status_sub{ORB_ID(vehicle_status)}, mode_sub{ORB_ID(vehicle_control_mode)}, land_sub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<om_mpc_command_s> publication{ORB_ID(om_mpc_command)};
	// Runtime is heap-allocated; the problem never occupies the 8 KiB worker stack.
	OmFloatProblem problem{};
	bool workspace_ready = false;
	bool engaged = false;
	uint64_t resets = 0;
	bool had_resets = false;
	uint64_t last_position_timestamp = 0;
	unsigned last_generation = 0, pending_updates = 0, divider = 2;
	bool generation_valid = false;
	float state_period_us = 10000.f;
};

OmMpc::OmMpc() : ScheduledWorkItem("om_mpc", om_queue), _runtime(new Runtime{})
{
	// Construct subscriptions/problem storage before mc_pos_control joins its
	// queue. State notifications must not allocate, solve, or create tasks.
}

OmMpc::~OmMpc()
{
	_stop.store(true);
	ScheduleClear();
#ifdef __PX4_POSIX

	// The owner may be destroyed on nav_and_controllers. Its lockstep
	// component must not sleep on simulation time while waiting for this
	// queue: simulation cannot advance until both Runs have returned.
	// ScheduleClear synchronizes queue removal before checking in-flight work.
	while (_run_in_progress.load()) {
		usleep(100);
	}

#endif
	// Deinit detaches after in-flight work has finished; solver storage stays valid.
	Deinit();
	// Run may have scheduled once while shutdown was waiting: cancel again.
	ScheduleClear();

	if (_runtime) {
		if (_runtime->workspace_ready) {
			om_float_close();
		}

		om_mpc_command_s inactive{};
		inactive.timestamp = hrt_absolute_time();
		_runtime->publication.publish(inactive);
		delete _runtime;
	}

	pthread_mutex_destroy(&_mutex);
}

void OmMpc::onLocalPositionUpdate()
{
	if (_stop.load() || !_runtime) {
		return;
	}

	_started.store(true);
	ScheduleNow();
}

void OmMpc::schedule_next()
{
	if (_stop.load()) {
		return;
	}

	// Only new publications release a QP. The timer is a stale-state watchdog.
	_runtime->rc_sub.update(&_runtime->rc);
	if (om_mpc::mpcRequested(setting(_runtime->h_mpc_om_en) == 1, _runtime->rc, hrt_absolute_time())
	    && _runtime->pos_sub.updated()) {
		ScheduleNow();

	} else {
		ScheduleDelayed(100000);
	}
}

void OmMpc::Run()
{
	if (_stop.load() || !_runtime) {
		return;
	}
	const uint64_t cycle_begin = wall_us();

	auto &h_mpc_om_rate = _runtime->h_mpc_om_rate;
	auto &h_mpc_om_en = _runtime->h_mpc_om_en;
	auto &ref_sub = _runtime->ref_sub;
	auto &pos_sub = _runtime->pos_sub;
	auto &att_sub = _runtime->att_sub;
	auto &status_sub = _runtime->status_sub;
	auto &mode_sub = _runtime->mode_sub;
	auto &land_sub = _runtime->land_sub;
	auto &publication = _runtime->publication;
	auto &problem = _runtime->problem;
	auto &workspace_ready = _runtime->workspace_ready;
	auto &engaged = _runtime->engaged;
	auto &resets = _runtime->resets;
	auto &had_resets = _runtime->had_resets;
	uint64_t published_cpu = UINT64_MAX;
	_runtime->update_config();
	const unsigned divider = setting(h_mpc_om_rate) == 100 ? 1 : 2;

	if (divider != _runtime->divider) {
		_runtime->pending_updates = 0;
		_runtime->divider = divider;
	}

	_runtime->rc_sub.update(&_runtime->rc);
	const bool enabled = om_mpc::mpcRequested(setting(h_mpc_om_en) == 1, _runtime->rc, hrt_absolute_time());

	if (!enabled) {
		// Publications while disabled are not missed MPC releases. Start a new
		// group on re-enable instead of counting the inactive interval as backlog.
		_runtime->generation_valid = false;
		_runtime->pending_updates = 0;
		_runtime->last_position_timestamp = 0;
	}

	vehicle_local_position_s pos{};
	const bool have_position = enabled && pos_sub.update(&pos);
	const uint64_t wake = hrt_absolute_time();
	unsigned due = 0;

	if (have_position) {
		const unsigned generation = pos_sub.get_last_generation();
		const unsigned updates = _runtime->generation_valid ? generation - _runtime->last_generation : 1;

		if (_runtime->generation_valid && updates && pos.timestamp > _runtime->last_position_timestamp) {
			const float sample_period = float(pos.timestamp - _runtime->last_position_timestamp) / updates;

			if (sample_period >= 1000.f && sample_period <= 100000.f) {
				// Publication time, not timestamp_sample: several new estimates may
				// legitimately share one sensor sampling timestamp.
				_runtime->state_period_us += .1f * (sample_period - _runtime->state_period_us);
			}
		}

		_runtime->generation_valid = true;
		_runtime->last_generation = generation;
		_runtime->last_position_timestamp = pos.timestamp;
		_runtime->pending_updates += updates;
		due = _runtime->pending_updates / divider;
		_runtime->pending_updates %= divider;
	}

	if (enabled && !due && _runtime->last_position_timestamp != 0
	    && wake < _runtime->last_position_timestamp + 100000) {
		// Wait for the next group of messages without invalidating the command.
		// Coalesced messages never cause a burst of historical solves.
		schedule_next();
		return;
	}

	const unsigned period_us = unsigned(lroundf(_runtime->state_period_us * divider));
	const unsigned budget_us = period_us < 20000 ? period_us : 20000;
	const uint64_t lateness = have_position && wake > pos.timestamp ? wake - pos.timestamp : 0;
	const uint64_t warm_deadline = enforce_wall_deadline ?
		wall_us() + (lateness < budget_us ? budget_us - lateness : 0) : UINT64_MAX;
	pthread_mutex_lock(&_mutex);
	_stats.period_us = period_us;

	if (_runtime->engaged) {
		_stats.skipped += due > 1 ? due - 1 : 0;

		if (lateness > _stats.wake_max_us) {
			_stats.wake_max_us = lateness;
		}
	}
	pthread_mutex_unlock(&_mutex);

	if (enforce_wall_deadline && due && lateness >= budget_us) {
		_ready.store(false);
		om_mpc_command_s inactive{};
		inactive.timestamp = wake;
		inactive.deadline_missed = true;
		inactive.cpu_us = UINT32_MAX;
		publication.publish(inactive);
		pthread_mutex_lock(&_mutex);
		_stats.deadline_misses += engaged;
		pthread_mutex_unlock(&_mutex);
		schedule_next();
		return;
	}

	om_mpc_reference_s ref{};
	vehicle_attitude_s att{};
	vehicle_status_s status{};
	vehicle_control_mode_s mode{};
	vehicle_land_detected_s land{};
	om_mpc_command_s output{};
	output.timestamp = hrt_absolute_time();
	output.cpu_us = UINT32_MAX;
	// The same explicit trajectory/state checks apply in flight and simulation.
	// SYS_HITL describes the sensor source, not controller output ownership.
	const bool eligible = due && have_position && _runtime->horizon_supported && enabled
			      && ref_sub.copy(&ref) && ref.valid && hrt_elapsed_time(&ref.timestamp) < 100000
			      && pos.xy_valid && pos.z_valid && pos.v_xy_valid && pos.v_z_valid && hrt_elapsed_time(&pos.timestamp) < 100000
			      && att_sub.copy(&att) && hrt_elapsed_time(&att.timestamp) < 100000
			      && status_sub.copy(&status) && status.nav_state == vehicle_status_s::NAVIGATION_STATE_ORBIT && status.takeoff_time != 0
			      && mode_sub.copy(&mode) && mode.flag_armed && mode.flag_control_position_enabled
			      && land_sub.copy(&land) && !land.landed && !land.ground_contact;

	// 1. Validate the current flight/reference before touching solver state.
	if (eligible) {
		const uint64_t begin = wall_us();
		const uint64_t current_resets = uint64_t(pos.xy_reset_counter) | (uint64_t(pos.z_reset_counter) << 8)
						| (uint64_t(pos.vxy_reset_counter) << 16) | (uint64_t(pos.vz_reset_counter) << 24) | (uint64_t(att.quat_reset_counter) << 32);

		if (had_resets && resets != current_resets) {
			om_float_close();
			workspace_ready = false;
			engaged = false;
		}

		had_resets = true;
		resets = current_resets;
		const bool cold = !workspace_ready;
		// 2. Construct the nominal error-state QP from fresh state and analytic reference.
		const uint64_t cpu_begin = om_qp_cpu_time_us();
		om_mpc::Path path;
		path.center = Vector3f(ref.center);
		path.amplitude = Vector3f(ref.amplitude);
		path.harmonic = Vector3f(ref.harmonic);
		path.offset = Vector3f(ref.phase_offset);
		path.phase = ref.phase;
		path.omega = ref.phase_rate;
		path.omega_dot = ref.phase_acceleration;
		path.ramp_time = ref.ramp_remaining;
		path.rotation = ref.heading_rotation;
		path.yaw = ref.yaw;
		const float hover = _runtime->hover, max_normalized = _runtime->max_normalized;
		output.hover_thrust = hover;
		// Flight tasks publish geometry/phase at their own cadence (~50 Hz).
		// Evaluate it at this fresh state release and at every prediction knot;
		// MPC does not need a new reference message for each state publication.
		const float elapsed = hrt_elapsed_time(&ref.timestamp) * 1e-6f;
		// Use only a fresh matched estimate from the already active INDI bridge.
		// With gain zero, stale feedback, or a disabled publisher, stay nominal.
		Vector3f predicted_disturbance{};
		om_mpc_indi_status_s feedback{};
		if (_runtime->disturbance_gain > 0.f && _runtime->disturbance_sub.copy(&feedback)
		    && feedback.disturbance_valid && feedback.timestamp != 0 && feedback.timestamp_sample != 0
		    && hrt_elapsed_time(&feedback.timestamp) < 100000
		    && hrt_elapsed_time(&feedback.timestamp_sample) < 100000
		    && Vector3f(feedback.acceleration_disturbance).isAllFinite()) {
			predicted_disturbance = Vector3f(feedback.acceleration_disturbance) * _runtime->disturbance_gain;
		}
		const bool built = hover > .01f && om_mpc::build(path, elapsed, Vector3f{pos.x, pos.y, pos.z}, Vector3f{pos.vx, pos.vy, pos.vz},
				   matrix::Dcmf(matrix::Quatf(att.q)), _runtime->max_thrust, _runtime->rate_limits, problem, _runtime->model_initialized,
				   _runtime->prediction_dt, _runtime->prediction_horizon, predicted_disturbance);
		if (built && !_runtime->model_initialized) {
			// A weight change resets the workspace under PID ownership. Warm
			// cycles reuse these diagonal costs and the solver factorization.
			for (int i = 0; i < 9; ++i) {
				problem.q[i] = _runtime->cost[i / 3];
				problem.p[i] = problem.q[i] * _runtime->cost[5];
			}

			for (int i = 0; i < 4; ++i) {
				problem.r[i] = _runtime->cost[i == 0 ? 3 : 4];
			}

			PX4_INFO("OM-MPC applied QP: N=%d dt=%.6f H=%.6f Q=[%.6f %.6f %.6f] R=[%.6f %.6f] P/Q=%.6f",
				 (int)problem.horizon, (double)problem.dt, (double)(problem.horizon * problem.dt),
				 (double)problem.q[0], (double)problem.q[3], (double)problem.q[6],
				 (double)problem.r[0], (double)problem.r[1], (double)(problem.p[0] / problem.q[0]));
		}

		_runtime->model_initialized |= built;
		const uint64_t frontend_end = wall_us();
		// Enter the local error model only after standby PID has captured the
		// moving reference. A deadline rejection does not restart this capture.
		EntryWait entry;

		if (!engaged) {
			entry.limit[0] = _runtime->entry_limits(0);
			entry.limit[1] = _runtime->entry_limits(1);
			entry.limit[2] = _runtime->entry_limits(2);

			if (built) {
				entry.error[0] = Vector3f(problem.x0).norm();
				entry.error[1] = Vector3f(problem.x0 + 3).norm();
				entry.error[2] = Vector3f(problem.x0 + 6).norm();

				for (int i = 0; i < 3; i++) {
					if (!(entry.error[i] <= entry.limit[i])) {
						entry.blocked |= 2u << i;
					}
				}

			} else {
				entry.blocked = 1;
			}
		}

		const bool captured = engaged || (built && entry.blocked == 0);

		// 3. Update/solve with the existing cold or warm deadline.
		if (captured) {
			const bool ready = built && om_float_update(problem, OmQpTolerance, OmQpLayout::OmErrorState);
			const uint64_t update_end = wall_us();
			OmFloatResult result{};
			result.status = 7;
			const uint64_t result_deadline = !enforce_wall_deadline ? UINT64_MAX : cold ? begin + 20000 : warm_deadline;
			om_qp_set_deadline(result_deadline);
			const int iteration_limit = _runtime->iteration_limit;

			if (ready) {
				result = om_float_solve(problem, cold ? 100 : iteration_limit);
				workspace_ready = true;
			}

			om_qp_set_deadline(0);
			const uint64_t cost = wall_us() - begin;
			output.solver_status = result.status;
			output.iterations = result.iterations;
			output.primal_residual = result.primal_residual;
			output.solve_us = cost;
			output.frontend_us = frontend_end - begin;
			output.update_us = update_end - frontend_end;
			output.solver_us = begin + cost - update_end;
			output.initializing = cold;
			output.timestamp_sample = pos.timestamp_sample;
			output.reference_timestamp = ref.timestamp;
			const uint64_t cpu_end = om_qp_cpu_time_us();
			const uint64_t cpu = (cpu_begin != UINT64_MAX && cpu_end != UINT64_MAX && cpu_end >= cpu_begin) ? cpu_end - cpu_begin : 0;
			const bool solved = ready && result.status == 1 && result.finite && result.primal_residual <= 2e-5f;
			output.deadline_missed = begin + cost > result_deadline;
			if (cpu_begin != UINT64_MAX && cpu_end != UINT64_MAX && cpu_end >= cpu_begin) {
				output.cpu_us = cpu;
			}
			const bool valid = solved && !output.deadline_missed;
			// Cold setup always belongs to standby PID (both update rates).
			// Its separate 20 ms/100-iteration budget prepares a workspace;
			// only a fresh warm solve can acquire control output ownership.
			_runtime->rc_sub.update(&_runtime->rc);
			if (valid && !cold
			    && om_mpc::mpcRequested(setting(h_mpc_om_en) == 1, _runtime->rc, hrt_absolute_time())
			    && status_sub.copy(&status)
			    && status.nav_state == vehicle_status_s::NAVIGATION_STATE_ORBIT) {
				output.active = true;
				predicted_disturbance.copyTo(output.mpc_disturbance);
				published_cpu = cpu_begin != UINT64_MAX && cpu_end != UINT64_MAX && cpu_end >= cpu_begin ? cpu : UINT64_MAX;
				engaged = true;

				for (int i = 0; i < 3; i++) {
					output.rates[i] = problem.reference_input[1 + i] + result.inputs[1 + i];
					output.position_error[i] = problem.x0[i];
				}

				const float thrust = problem.reference_input[0] + result.inputs[0];
				output.thrust_body[2] = -fminf(fmaxf(thrust * hover / 9.80665f, 0.f), max_normalized);

			} else if (!solved && !result.resumable) {
				om_float_close();
				workspace_ready = false;
			}

			// A converged but late command is rejected; retain its valid workspace
			// for the next warm solve instead of forcing another cold initialization.
			pthread_mutex_lock(&_mutex);
			++_stats.calls;
			_stats.recovery_attempts += result.recovery_attempted;
			_stats.recovery_successes += result.recovery_attempted && result.recovery_succeeded && solved;

			if (result.arena_used > _stats.arena_peak) {
				_stats.arena_peak = result.arena_used;
			}

			_stats.cold_calls += cold;
			_stats.cold_failures += cold && !valid;
			_stats.cold_over_10ms += cold && cost > 10000;
			_stats.warm_over_10ms += !cold && cost > 10000;

			if (cold && cost > _stats.cold_max) {
				_stats.cold_max = cost;
			}

			if (!cold && cost > _stats.warm_max) {
				_stats.warm_max = cost;
			}

			_stats.resumed_limits += result.resumable;
			_stats.late_valid += solved && !valid;

			if (!valid) {
				_stats.failure = {_stats.calls, frontend_end - begin, update_end - frontend_end,
						  begin + cost - update_end, cost, cpu, result.status, result.iterations, cold};
			}

			_stats.failures += !valid;
			_stats.deadline_misses += !cold && output.deadline_missed;
			_stats.sum_us += cost;

			if (cpu_begin != UINT64_MAX && cpu_end != UINT64_MAX && cpu_end >= cpu_begin) {
				_stats.cpu_sum += cpu;
				++_stats.cpu_samples;
			}

			_stats.frontend_sum += frontend_end - begin;
			_stats.update_sum += update_end - frontend_end;
			_stats.solve_sum += begin + cost - update_end;

			if (cost > _stats.max_us) {
				_stats.max_us = cost;
			}

			_stats.overruns += cost > 20000;
			_stats.last_status = result.status;
			_stats.last_iterations = result.iterations;
			_stats.residual = result.primal_residual;
			_ready.store(output.active);
			pthread_mutex_unlock(&_mutex);

		} else {
			pthread_mutex_lock(&_mutex);
			++_stats.entry_waits;
			entry.timestamp = hrt_absolute_time();
			_stats.entry_wait = entry;
			pthread_mutex_unlock(&_mutex);
		}

	} else {
		if (workspace_ready) {
			om_float_close();
		}

		workspace_ready = false;
		engaged = false;
		had_resets = false;
		_runtime->pending_updates = 0;
	}

	if (!output.active) {
		_ready.store(false);
	}

	// 4. Publish one command. Inactive commands preserve the original PID path.
	output.timestamp = hrt_absolute_time();
	publication.publish(output);

	if (output.active) {
		const uint64_t publication_end = wall_us();
		const uint64_t elapsed = publication_end - cycle_begin;
		pthread_mutex_lock(&_mutex);
		++_stats.tracking.calls;
		_stats.tracking.sum += elapsed;

		if (elapsed > _stats.tracking.maximum) {
			_stats.tracking.maximum = elapsed;
		}

		if (published_cpu != UINT64_MAX) {
			++_stats.tracking.cpu_samples;
			_stats.tracking.cpu_sum += published_cpu;

			if (published_cpu > _stats.tracking.cpu_max) {
				_stats.tracking.cpu_max = published_cpu;
			}
		}

		pthread_mutex_unlock(&_mutex);
	}

	if (have_position) {
		schedule_next();

	} else {
		ScheduleDelayed(100000);
	}
}

void OmMpc::print_status()
{
	pthread_mutex_lock(&_mutex);
	const Statistics stats = _stats;
	pthread_mutex_unlock(&_mutex);

	uORB::Subscription output_sub{ORB_ID(om_mpc_status)};
	om_mpc_status_s output{};
	const bool applied = output_sub.copy(&output) && output.active && hrt_elapsed_time(&output.timestamp) < 300000;
	const int n = setting("MPC_N");
	const unsigned nominal_rate = setting("MPC_UPD_MODE") == 100 ? 100 : 50;
	PX4_INFO("OM-MPC: worker=%s ready=%s active=%s state/%u (nominal %uHz) N%d dt=%.3f",
		 _started.load() ? "started" : "off", _ready.load() ? "yes" : "no", applied ? "yes" : "no",
		 nominal_rate == 100 ? 1u : 2u, nominal_rate, n, (double)parameter(param_find("MPC_DT"), .04f));
#ifdef __PX4_POSIX
	if (n < 1 || n > OmMaxHorizon) {
		PX4_WARN("OM-MPC requires N1..%d on POSIX; PID retains control", OmMaxHorizon);
	}
#else
	if (n != 10 && n != 15 && n != 20) {
		PX4_WARN("OM-MPC requires N10/15/20; PID retains control");
	}
#endif
	if (!enforce_wall_deadline) {
		PX4_WARN("OM-MPC offline benchmark build: wall-clock deadlines disabled");
	}

	const auto &tracking = stats.tracking;
	PX4_INFO("OM-MPC published warm cycles=%llu elapsed mean=%.2f max=%llu us (Run entry through publication)",
		 (unsigned long long)tracking.calls, tracking.calls ? double(tracking.sum) / tracking.calls : 0.,
		 (unsigned long long)tracking.maximum);
	PX4_INFO("OM-MPC published QP CPU: samples=%llu mean=%.2f max=%llu us",
		 (unsigned long long)tracking.cpu_samples, tracking.cpu_samples ? double(tracking.cpu_sum) / tracking.cpu_samples : 0.,
		 (unsigned long long)tracking.cpu_max);
	PX4_INFO("OM-MPC attempts: calls=%llu failures=%llu mean=%.2f max=%llu us",
		 (unsigned long long)stats.calls, (unsigned long long)stats.failures, stats.calls ? double(stats.sum_us) / stats.calls : 0.,
		 (unsigned long long)stats.max_us);
	PX4_INFO("OM-MPC mean frontend=%.2f update=%.2f solve/checks=%.2f us; CPU=%.2f us",
		 stats.calls ? double(stats.frontend_sum) / stats.calls : 0., stats.calls ? double(stats.update_sum) / stats.calls : 0.,
		 stats.calls ? double(stats.solve_sum) / stats.calls : 0., stats.cpu_samples ? double(stats.cpu_sum) / stats.cpu_samples : 0.);
	PX4_INFO("OM-MPC estimated period=%u us, deadline misses=%llu skipped=%llu wake max=%llu us",
		 stats.period_us, (unsigned long long)stats.deadline_misses, (unsigned long long)stats.skipped, (unsigned long long)stats.wake_max_us);
	// Cold setup belongs to PID and is intentionally separated from published commands.
	PX4_INFO("OM-MPC cold: calls=%llu failures=%llu max=%llu us; warm max=%llu us",
		 (unsigned long long)stats.cold_calls, (unsigned long long)stats.cold_failures,
		 (unsigned long long)stats.cold_max, (unsigned long long)stats.warm_max);
	PX4_INFO("OM-MPC compute>10ms cold=%llu warm=%llu >20ms=%llu late-converged=%llu",
		 (unsigned long long)stats.cold_over_10ms, (unsigned long long)stats.warm_over_10ms,
		 (unsigned long long)stats.overruns, (unsigned long long)stats.late_valid);
	PX4_INFO("OM-MPC entry waits=%llu retained workspaces=%llu recoveries=%llu/%llu",
		 (unsigned long long)stats.entry_waits, (unsigned long long)stats.resumed_limits,
		 (unsigned long long)stats.recovery_successes, (unsigned long long)stats.recovery_attempts);

	const auto &entry = stats.entry_wait;
	if (entry.timestamp) {
		PX4_INFO("OM-MPC last entry wait: blocked=%u pos=%.3f/%.3f m vel=%.3f/%.3f m/s rot=%.1f/%.1f deg",
			 entry.blocked, (double)entry.error[0], (double)entry.limit[0], (double)entry.error[1], (double)entry.limit[1],
			 (double)(entry.error[2] * 57.295779513f), (double)(entry.limit[2] * 57.295779513f));
	}
	const auto &failure = stats.failure;
	if (failure.call) {
		PX4_INFO("OM-MPC last failure: call=%llu %s status=%d iter=%d total=%llu CPU=%llu us",
			 (unsigned long long)failure.call, failure.cold ? "cold" : "warm", failure.status, failure.iterations,
			 (unsigned long long)failure.total, (unsigned long long)failure.cpu);
		PX4_INFO("OM-MPC failure frontend=%llu update=%llu solve/checks=%llu us",
			 (unsigned long long)failure.frontend, (unsigned long long)failure.update, (unsigned long long)failure.solve);
	}
	PX4_INFO("OM-MPC solver=qpDUNES float32 iteration cap=%d last status=%d iter=%d residual=%.8f",
		 setting("MPC_QP_ITMAX"), stats.last_status, stats.last_iterations, (double)stats.residual);
	PX4_INFO("OM-MPC memory: arena peak=%u capacity=%u problem=%u stack=%u bytes",
		 (unsigned)stats.arena_peak, om_arena_capacity(n), (unsigned)sizeof(OmFloatProblem), (unsigned)om_queue.stacksize);
}
