#pragma once

#include <stdint.h>
void om_qp_set_deadline(uint64_t deadline);
uint64_t om_qp_cpu_time_us();
// Keep NuttX scheduler runtime accounting enabled for the worker lifetime.
// The reference-counted monitor composes with top and other users.
class OmCpuMonitor
{
public:
	OmCpuMonitor();
	~OmCpuMonitor();
	OmCpuMonitor(const OmCpuMonitor &) = delete;
	OmCpuMonitor &operator=(const OmCpuMonitor &) = delete;
};
