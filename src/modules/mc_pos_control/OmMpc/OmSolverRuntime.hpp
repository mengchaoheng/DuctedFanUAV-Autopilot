#pragma once

#include <stdint.h>
void om_qp_set_deadline(uint64_t deadline);

#ifdef MC_POS_OM_MPC_BENCH
uint64_t om_qp_cpu_time_us();
// Keep NuttX scheduler runtime accounting enabled during the ground benchmark.
// The reference-counted monitor composes with top and other users.
class OmCpuMonitor
{
public:
	OmCpuMonitor() = default;
	~OmCpuMonitor();
	OmCpuMonitor(const OmCpuMonitor &) = delete;
	OmCpuMonitor &operator=(const OmCpuMonitor &) = delete;
	void start();
	void stop();

private:
	bool _running{false};
};
#endif
