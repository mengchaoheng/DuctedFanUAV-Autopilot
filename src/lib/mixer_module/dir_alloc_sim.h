/*
 * File: dir_alloc_sim.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 25-Apr-2021 15:48:18
 */

#ifndef DIR_ALLOC_SIM_H
#define DIR_ALLOC_SIM_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "dir_alloc_sim_types.h"

/* Function Declarations */
extern void dir_alloc_sim(const double v[3], const double umin[4], const double
  umax[4], double u[4], double *z, double *iters);
extern void dir_alloc_sim_initialize(void);
extern void dir_alloc_sim_terminate(void);

#endif

/*
 * File trailer for dir_alloc_sim.h
 *
 * [EOF]
 */
