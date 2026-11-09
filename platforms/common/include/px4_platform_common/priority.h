// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#ifdef __PX4_DARWIN
#include <sched.h>
#endif

// NuttX-sized offsets can exceed Darwin's narrower scheduler range. Darwin's
// pthread attributes accept negative priorities, which can become priority 63
// instead of background work. Check the native range before creating a thread;
// preserve the requested priorities on every other platform.
static inline int px4_sched_priority_clamp(int priority)
{
#ifdef __PX4_DARWIN
	const int minimum = sched_get_priority_min(SCHED_FIFO);
	const int maximum = sched_get_priority_max(SCHED_FIFO);
	priority = priority < minimum ? minimum : priority;
	priority = priority > maximum ? maximum : priority;
#endif
	return priority;
}
