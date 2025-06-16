/*
 * File: wls_alloc_gen_types.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2025-06-16 11:03:22
 */

#ifndef WLS_ALLOC_GEN_TYPES_H
#define WLS_ALLOC_GEN_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef struct_emxArray_real32_T
#define struct_emxArray_real32_T
struct emxArray_real32_T {
  float *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};
#endif /* struct_emxArray_real32_T */
#ifndef typedef_emxArray_real32_T
#define typedef_emxArray_real32_T
typedef struct emxArray_real32_T emxArray_real32_T;
#endif /* typedef_emxArray_real32_T */

#endif
/*
 * File trailer for wls_alloc_gen_types.h
 *
 * [EOF]
 */
