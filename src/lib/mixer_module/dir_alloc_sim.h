/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: dir_alloc_sim.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2024-03-11 20:38:05
 */

#ifndef DIR_ALLOC_SIM_H
#define DIR_ALLOC_SIM_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void dir_alloc_sim(const float v[3], const float umin[4],
                          const float umax[4], const float B[12], float u[4],
                          float *z, float *iters);

extern void dir_alloc_sim_initialize(void);

extern void dir_alloc_sim_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for dir_alloc_sim.h
 *
 * [EOF]
 */
