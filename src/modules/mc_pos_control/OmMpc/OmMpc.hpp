// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <pthread.h>
#include <stdint.h>

// mc_pos_control owns this worker; only its private queue touches the QP.
// hasValidCommand() means a command is available. Actual output ownership is
// reported separately by mc_att_control through om_mpc_status.active.
class OmMpc : public px4::ScheduledWorkItem
{
public:
	OmMpc();
	~OmMpc();
	// Called by the owner's existing local-position callback; never solves here.
	void onLocalPositionUpdate();
	void print_status();
	bool hasValidCommand() const { return _ready.load(); }

private:
	void Run() override;
	void schedule_next();
	struct Runtime;
	Runtime *_runtime{nullptr};
	px4::atomic_bool _started{false}, _stop{false}, _ready{false};

	struct TrackingTiming {
		uint64_t calls{0}, sum{0}, maximum{0}, cpu_samples{0}, cpu_sum{0}, cpu_max{0};
	};
	struct Failure {
		uint64_t call{0}, frontend{0}, update{0}, solve{0}, total{0}, cpu{0};
		int status{0}, iterations{0};
		bool cold{false};
	};
	struct EntryWait {
		uint64_t timestamp{0};
		float error[3]{}, limit[3]{};
		unsigned blocked{0}; // 1=frontend, 2=position, 4=velocity, 8=rotation
	};
	struct Statistics {
		TrackingTiming tracking; // Successfully published warm commands only.
		Failure failure; // Most recent rejection; detailed per-cycle data is in ULog.
		EntryWait entry_wait;
		uint64_t calls{0}, failures{0}, sum_us{0}, max_us{0}, overruns{0};
		uint64_t deadline_misses{0}, skipped{0}, wake_max_us{0};
		uint64_t cpu_sum{0}, cpu_samples{0};
		uint64_t frontend_sum{0}, update_sum{0}, solve_sum{0};
		uint64_t cold_calls{0}, cold_failures{0}, cold_max{0}, warm_max{0};
		uint64_t cold_over_10ms{0}, warm_over_10ms{0}, late_valid{0};
		uint64_t entry_waits{0}, resumed_limits{0};
		uint64_t recovery_attempts{0}, recovery_successes{0};
		uint32_t arena_peak{0};
		unsigned period_us{20000};
		int last_status{0}, last_iterations{0};
		float residual{0.f};
	} _stats{};
	pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER; // Only statistics, never solver execution or console I/O.
};
