// Private worker deadline and CPU diagnostics; not shared with control work queues.
#include "OmSolverRuntime.hpp"
#include <drivers/drv_hrt.h>
#include <time.h>
#include <unistd.h>
#ifdef __PX4_NUTTX
#include <px4_platform/cpuload.h>
#include <nuttx/sched.h>
#include <nuttx/irq.h>
#endif
namespace
{
uint64_t solve_deadline;
uint64_t timing_us()
{
#ifdef __PX4_POSIX
	// SIH lockstep hrt time is simulated; use actual host elapsed time here.
	timespec stamp{};
	clock_gettime(CLOCK_MONOTONIC, &stamp);
	return static_cast<uint64_t>(stamp.tv_sec) * 1000000ULL + stamp.tv_nsec / 1000;
#else
	return hrt_absolute_time();
#endif
}
uint64_t cpu_time_us()
{
#ifdef __PX4_NUTTX
#ifdef CONFIG_SCHED_INSTRUMENTATION
	const pid_t tid = gettid();
	const irqstate_t flags = up_irq_save();
	uint64_t result = UINT64_MAX;
	for (const auto &task : system_load.tasks) {
		if (task.valid && task.tcb && task.tcb->pid == tid && task.curr_start_time) {
			result = task.total_runtime + hrt_absolute_time() - task.curr_start_time;
			break;
		}
	}
	up_irq_restore(flags);
	return result;
#else
	return UINT64_MAX;
#endif
#else
	timespec stamp {};
	if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &stamp) != 0) {
		return UINT64_MAX;
	}
	return static_cast<uint64_t>(stamp.tv_sec) * 1000000ULL + stamp.tv_nsec / 1000;
#endif
}

}

uint64_t om_qp_cpu_time_us()
{
	return cpu_time_us();
}
OmCpuMonitor::OmCpuMonitor()
{
#if defined(__PX4_NUTTX) && defined(CONFIG_SCHED_INSTRUMENTATION)
	cpuload_monitor_start();
#endif
}
OmCpuMonitor::~OmCpuMonitor()
{
#if defined(__PX4_NUTTX) && defined(CONFIG_SCHED_INSTRUMENTATION)
	cpuload_monitor_stop();
#endif
}
void om_qp_set_deadline(uint64_t deadline)
{
	solve_deadline = deadline;
}
extern "C" int om_float_om_qp_deadline_expired(void)
{
	return solve_deadline && timing_us() >= solve_deadline;
}
