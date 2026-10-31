// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
#include "OmMpcBench.hpp"
#include "OmMpcBenchCase.hpp"
#include "OmSolverRuntime.hpp"
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/log.h>

namespace
{
template<typename T> void read_parameter(const char *name, T &value)
{
	param_get(param_find(name), &value);
}
}

OmMpcBench::OmMpcBench(unsigned period_us, unsigned seconds) :
	_seconds(seconds), _period(period_us)
{
	read_parameter("MPC_N", _horizon);
	read_parameter("MPC_DT", _dt);
	read_parameter("MPC_QP_ITMAX", _iteration_limit);
	const char *names[] {"MPC_QPOS", "MPC_QVEL", "MPC_QROT", "MPC_RTHR", "MPC_ROMG", "MPC_TERM_SCALE"};
	bool costs_valid = true;

	for (unsigned i = 0; i < 6; ++i) {
		read_parameter(names[i], _cost[i]);
		costs_valid &= std::isfinite(_cost[i]) && _cost[i] > 0.f;
	}

	float hover = .51f, maximum = 1.f;
	read_parameter("MPC_THR_HOVER", hover);
	read_parameter("MPC_THR_MAX", maximum);
	const char *rate_names[] {"MC_ROLLRATE_MAX", "MC_PITCHRATE_MAX", "MC_YAWRATE_MAX"};

	for (unsigned i = 0; i < 3; ++i) {
		float degrees = _rate_limit[i] * 57.2957795f;
		read_parameter(rate_names[i], degrees);
		_rate_limit[i] = degrees * .01745329252f;
		costs_valid &= std::isfinite(_rate_limit[i]) && _rate_limit[i] > 0.f;
	}

	_max_thrust = 9.80665f * maximum / hover;
	_valid = period_us >= 2000 && period_us <= 1000000 && seconds >= 1 && seconds <= 10 && costs_valid
		 && (_horizon == 10 || _horizon == 15 || _horizon == 20) && _horizon <= OmMaxHorizon
		 && std::isfinite(_dt) && _dt >= .005f && _dt <= .1f
		 && _iteration_limit >= 1 && _iteration_limit <= 100
		 && std::isfinite(hover) && hover > .01f && hover <= 1.f
		 && std::isfinite(maximum) && maximum > 0.f && maximum <= 1.f && disarmed();
}

bool OmMpcBench::disarmed()
{
	vehicle_status_s status{};
	return _status_sub.copy(&status) && status.timestamp && hrt_elapsed_time(&status.timestamp) < 2000000
	       && status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED;
}

void OmMpcBench::sample_load(Load &load)
{
	cpuload_s sample{};

	if (_load_sub.update(&sample) && sample.timestamp && hrt_elapsed_time(&sample.timestamp) < 1500000
	    && std::isfinite(sample.load) && sample.load >= 0.f && sample.load <= 1.f) {
		load.add(sample.load);
	}
}

unsigned OmMpcBench::finish(const char *reason)
{
	om_qp_set_deadline(0);
	om_float_close();
	_finished = true;
	_measure_end = hrt_absolute_time();
	PX4_INFO("MPC bench %s (no control output)", reason);
	print_status();
	return 0;
}

unsigned OmMpcBench::run(OmFloatProblem &problem)
{
	if (_cancelled || !disarmed()) { return finish(_cancelled ? "stopped" : "aborted: armed/stale status"); }

	const uint64_t now = hrt_absolute_time();

	if (!_started) {
		_started = true;
		_begin = _next = now;
		om_float_close();
		PX4_INFO("MPC bench: N=%d dt=%.4f period=%u us target=%.2f Hz duration=%u s", (int)_horizon, (double)_dt,
			 _period, 1e6 / _period, _seconds);
		PX4_INFO("Q=%.1f/%.1f/%.1f R=%.3f/%.1f terminal=%.2f", (double)_cost[0], (double)_cost[1],
			 (double)_cost[2], (double)_cost[3], (double)_cost[4], (double)_cost[5]);
		PX4_INFO("Synthetic figure eight; first cold solve excluded from warm statistics");
		sample_load(_baseline_load);
	}

	if (now - _begin >= uint64_t(_seconds) * 1000000) { return finish("complete"); }

	if (!_measure_begin && _workspace_ready && now >= _next) {
		_measure_begin = now;
		cpuload_s discarded{};
		_load_sub.update(&discarded); // discard the sample spanning the cold setup
	}

	if (_measure_begin) {
		sample_load(_test_load);
	}

	if (now < _next) { return unsigned(_next - now); }

	const uint64_t release = _next;
	_next += _period;
	om_mpc::Path path;
	matrix::Vector3f position, velocity;
	matrix::Dcmf rotation;

	// Independent of solver progress: identical reference phase vs elapsed time
	// at both rates; no constant-QP shortcut or integrated pseudo-plant.
	if (!om_mpc::bench_state(float(now - _begin) * 1e-6f, path, position, velocity, rotation)) {
		return finish("aborted: invalid synthetic state");
	}

	const bool cold = !_workspace_ready;
	const uint64_t cpu_begin = om_qp_cpu_time_us();
	const uint64_t begin = hrt_absolute_time();
	const bool built = om_mpc::build(path, 0.f, position, velocity, rotation, _max_thrust, matrix::Vector3f(_rate_limit),
					 problem, _model_ready, _dt, _horizon);

	if (built && !_model_ready) {
		for (unsigned i = 0; i < 9; ++i) {
			problem.q[i] = _cost[i / 3];
			problem.p[i] = problem.q[i] * _cost[5];
		}

		for (unsigned i = 0; i < 4; ++i) { problem.r[i] = _cost[i ? 4 : 3]; }

		_model_ready = true;
	}

	const uint64_t built_at = hrt_absolute_time();
	const bool updated = built && om_float_update(problem, OmQpTolerance, OmQpLayout::OmErrorState);
	const uint64_t updated_at = hrt_absolute_time();
	const uint64_t cycle_deadline = cold ? begin + 20000 : release + _period;
	const uint64_t test_deadline = _begin + uint64_t(_seconds) * 1000000;
	const uint64_t deadline = cycle_deadline < test_deadline ? cycle_deadline : test_deadline;
	om_qp_set_deadline(deadline);
	OmFloatResult result{};

	if (updated) {
		result = om_float_solve(problem, cold ? 100 : _iteration_limit);
		_workspace_ready = true;
	}

	om_qp_set_deadline(0);
	const uint64_t end = hrt_absolute_time();
	const uint64_t cpu_end = om_qp_cpu_time_us();
	const bool solved = updated && result.status == 1 && result.finite && result.primal_residual <= 2e-5f;

	if (result.arena_used > _arena_peak) { _arena_peak = result.arena_used; }

	_cold += cold;

	if (cold) { _cold_wall.add(end - begin); _cold_failed += !solved; }

	if (_measure_begin && !cold) {
		_failed += !solved;
		_late += end > deadline;
		_wall.add(end - begin);
		_frontend.add(built_at - begin);
		_update.add(updated_at - built_at);
		_solve.add(end - updated_at);
		_iterations.add(result.iterations > 0 ? unsigned(result.iterations) : 0);

		if (cpu_begin != UINT64_MAX && cpu_end != UINT64_MAX && cpu_end >= cpu_begin) { _cpu.add(cpu_end - cpu_begin); }
	}

	if (!solved && !result.resumable) {
		om_float_close();
		_workspace_ready = false;
	}

	// Skip missed releases, never run a burst to catch up.
	if (end >= _next) {
		const unsigned skipped = unsigned((end - _next) / _period + 1);
		_next += uint64_t(skipped) * _period;

		if (_measure_begin) { _skipped += skipped; }
	}

	return unsigned(_next - end);
}

void OmMpcBench::print_status() const
{
	const uint64_t end = _finished ? _measure_end : hrt_absolute_time();
	const double elapsed = _measure_begin ? double(end - _measure_begin) * 1e-6 : 0.;
	PX4_INFO("Bench %s: N=%d dt=%.4f period=%u us target=%.2f Hz", _finished ? "finished" : "running",
		 (int)_horizon, (double)_dt, _period, 1e6 / _period);
	PX4_INFO("Measured %.2f s: calls=%u achieved=%.2f Hz failed=%u late=%u skipped=%u cold=%u", elapsed,
		 _wall.count, elapsed > 0. ? _wall.count / elapsed : 0., _failed, _late, _skipped, _cold);
	PX4_INFO("Total wall us mean/max=%.1f/%llu", _wall.mean(), (unsigned long long)_wall.maximum);
	PX4_INFO("Cold wall us mean/max=%.1f/%llu failed=%u", _cold_wall.mean(), (unsigned long long)_cold_wall.maximum, _cold_failed);
	PX4_INFO("Build/update/solver wall us mean=%.1f/%.1f/%.1f", _frontend.mean(), _update.mean(), _solve.mean());
	PX4_INFO("Core thread CPU us mean/max=%.1f/%llu samples=%u", _cpu.mean(), (unsigned long long)_cpu.maximum, _cpu.count);
	PX4_INFO("Core CPU contribution=%.2f%% iterations mean/max=%.2f/%llu", elapsed > 0. ? double(_cpu.sum) / (elapsed * 10000.) : 0.,
		 _iterations.mean(), (unsigned long long)_iterations.maximum);
	PX4_INFO("System CPU pre-test/test/peak=%.2f/%.2f/%.2f%% samples=%u/%u", _baseline_load.percent(),
		 _test_load.percent(), (double)_test_load.maximum * 100., _baseline_load.count, _test_load.count);

	if (_baseline_load.count && _test_load.count) {
		PX4_INFO("System CPU delta=%+.2f percentage points", _test_load.percent() - _baseline_load.percent());

	} else { PX4_WARN("System CPU delta unavailable: insufficient cpuload samples"); }

	PX4_INFO("QP arena used/reserved=%u/%u B problem=%u B bench=%u B", _arena_peak, om_arena_capacity(_horizon),
		 (unsigned)sizeof(OmFloatProblem), (unsigned)sizeof(*this));
}
