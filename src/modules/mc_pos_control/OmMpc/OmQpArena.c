// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
#include "OmQpArena.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Exclusive to the OM-MPC worker. All solver allocations belong to one
// checked arena allocated at entry and released on disable/reset/shutdown.
// Warm updates and solves do not allocate or free heap memory.
static unsigned char *arena;
static size_t arena_capacity;
static size_t arena_used;
#ifdef __PX4_NUTTX
// ARM GCC builtins avoid optional NuttX libc setjmp support.
static void *allocation_failure[5];
#define ARENA_SETJMP() __builtin_setjmp(allocation_failure)
#define ARENA_LONGJMP() __builtin_longjmp(allocation_failure, 1)
#else
#include <setjmp.h>
static jmp_buf allocation_failure;
#define ARENA_SETJMP() setjmp(allocation_failure)
#define ARENA_LONGJMP() longjmp(allocation_failure, 1)
#endif

int om_qp_arena_open(size_t capacity)
{
	arena = malloc(capacity);
	arena_capacity = arena ? capacity : 0;
	arena_used = 0;
	return arena != NULL;
}
void om_qp_arena_close(void)
{
	free(arena);
	arena = NULL;
	arena_capacity = arena_used = 0;
}
size_t om_qp_arena_used(void) { return arena_used; }

void *om_qp_calloc(size_t count, size_t size)
{
	const size_t alignment = 16;
	if (size && count > SIZE_MAX / size) { ARENA_LONGJMP(); }
	const size_t bytes = count * size;
	const size_t padded = (bytes ? bytes : 1) + alignment - 1;
	if (padded < bytes) { ARENA_LONGJMP(); }
	const size_t reserved = padded & ~(alignment - 1);
	if (!arena || reserved > arena_capacity - arena_used) { ARENA_LONGJMP(); }
	void *result = arena + arena_used;
	arena_used += reserved;
	memset(result, 0, reserved);
	return result;
}
void om_qp_free(void *ptr) { (void)ptr; }

return_t om_qp_setup_horizon_checked(qpData_t *qp, qpOptions_t *options, int horizon)
{
	if (ARENA_SETJMP()) {
		// Only C frames occur between setjmp and allocator. No partial-tree
		// cleanup is needed: releasing the arena disposes every allocation.
		memset(qp, 0, sizeof(*qp));
		return QPDUNES_ERR_INVALID_ARGUMENT;
	}
	return qpDUNES_setup(qp, horizon, 9, 4, NULL, options);
}

