// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "OmQpSolver.hpp"
#include "OmSolverRuntime.hpp"
#include <uORB/Subscription.hpp>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/vehicle_status.h>

// Ground-only synthetic load. Called exclusively by the existing MPC worker,
// never publishes commands and never creates a second solver workspace.
class OmMpcBench
{
public:

	OmMpcBench(unsigned period_us, unsigned seconds);
	bool valid() const { return _valid; }

	bool started() const { return _started; }

	void cancel() { _cancelled = true; }

	// Zero means finished; otherwise delay until the next test release (us).
	unsigned run(OmFloatProblem &problem);
	void print_status() const;

private:

	struct Timing {
		uint64_t sum{0}, maximum{0};
		unsigned count{0};
		void add(uint64_t value) { sum += value; ++count; if (value > maximum) { maximum = value; } }

		double mean() const { return count ? double(sum) / count : 0.; }
	};

	struct Load {
		float sum{0.f}, maximum{0.f};
		unsigned count{0};
		void add(float value) { sum += value; ++count; if (value > maximum) { maximum = value; } }

		double percent() const { return count ? double(sum) * 100. / count : 0.; }
	};

	unsigned finish(const char *reason);
	void sample_load(Load &load);
	bool disarmed();
	const unsigned _seconds, _period;
	bool _valid{false}, _started{false}, _cancelled{false}, _finished{false};
	bool _model_ready{false}, _workspace_ready{false};
	int32_t _horizon{20}, _iteration_limit{100};
	float _dt{.05f}, _cost[6] {550.f, 100.f, 500.f, .25f, 350.f, 1.f};
	float _max_thrust{19.22873f}, _rate_limit[3] {20.f, 20.f, 8.f};
	uint64_t _begin{0}, _measure_begin{0}, _next{0}, _measure_end{0};
	unsigned _failed{0}, _late{0}, _skipped{0}, _cold{0}, _arena_peak{0};
	unsigned _cold_failed{0};
	Timing _wall, _cpu, _frontend, _update, _solve, _iterations, _cold_wall;
	Load _baseline_load, _test_load;
	OmCpuMonitor _cpu_monitor;
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)}, _load_sub{ORB_ID(cpuload)};
};
