/*
 * File: wls_alloc_gen.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2025-06-16 11:03:22
 */

#ifndef WLS_ALLOC_GEN_H
#define WLS_ALLOC_GEN_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void wls_alloc_gen(const float B[12], const float v[3],
                          const float umin[4], const float umax[4],
                          const float Wv[9], const float Wu[16],
                          const float ud[4], float gam, float u[4], float W[4],
                          float imax, float m);

extern void wls_alloc_gen_initialize(void);

extern void wls_alloc_gen_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for wls_alloc_gen.h
 *
 * [EOF]
 */
