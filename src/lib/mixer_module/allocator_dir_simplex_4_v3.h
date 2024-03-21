/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: allocator_dir_simplex_4_v3.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2024-03-21 09:21:52
 */

#ifndef ALLOCATOR_DIR_SIMPLEX_4_V3_H
#define ALLOCATOR_DIR_SIMPLEX_4_V3_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void allocator_dir_simplex_4_v3(const float v[3], const float umin[4],
                                       const float umax[4], float u[4],
                                       float *z, unsigned long *iters);

extern void allocator_dir_simplex_4_v3_initialize(void);

extern void allocator_dir_simplex_4_v3_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for allocator_dir_simplex_4_v3.h
 *
 * [EOF]
 */
