// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
#pragma once
// Legacy qpDUNES headers contain non-prototype C declarations.
#ifndef __cplusplus
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-prototypes"
#endif
#include <qpDUNES.h>
#ifndef __cplusplus
#pragma GCC diagnostic pop
#endif
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
void *om_qp_calloc(size_t count, size_t size);
void om_qp_free(void *ptr);
int om_qp_arena_open(size_t capacity);
void om_qp_arena_close(void);
size_t om_qp_arena_used(void);
return_t om_qp_setup_horizon_checked(qpData_t *qp, qpOptions_t *options, int horizon);
int om_qp_deadline_expired(void);
#ifdef __cplusplus
}
#endif
