/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: allocator_dir_LPwrap_4.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2024-03-24 20:52:32
 */

#ifndef ALLOCATOR_DIR_LPWRAP_4_H
#define ALLOCATOR_DIR_LPWRAP_4_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void allocator_dir_LPwrap_4(const float B[12], const float v[3],
                                   const float umin[4], const float umax[4],
                                   float u[4], float *z, unsigned int *iters);

extern void allocator_dir_LPwrap_4_initialize(void);

extern void allocator_dir_LPwrap_4_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for allocator_dir_LPwrap_4.h
 *
 * [EOF]
 */
