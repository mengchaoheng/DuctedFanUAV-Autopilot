/*
 * File: dir_alloc_six.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 03-Jun-2022 18:26:31
 */

#ifndef DIR_ALLOC_SIX_H
#define DIR_ALLOC_SIX_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void dir_alloc_six(const float umin[6], const float umax[6],
                          const float v[3], const float B[18], float u[6],
                          float *z, short *iters);

extern void dir_alloc_six_initialize(void);

extern void dir_alloc_six_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for dir_alloc_six.h
 *
 * [EOF]
 */
