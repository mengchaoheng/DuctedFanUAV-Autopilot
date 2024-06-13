/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: allocator_dir_LPwrap_4.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2024-03-24 20:52:32
 */

/* Include Files */
#include "allocator_dir_LPwrap_4.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static int do_vectors(double b, unsigned char c_data[], int c_size[2],
                      int ia_data[], int *ib_size);

static float maximum(const float x[3], int *idx);

static float minimum(const float x[4], int *idx);

static bool simplxuprevsol_C(float A[8], float ct[4], float b[2],
                             unsigned char inB[2], const float h[4],
                             signed char e[4], unsigned int *itlim,
                             float b_y0[2]);

/* Function Definitions */
/*
 * Arguments    : double b
 *                unsigned char c_data[]
 *                int c_size[2]
 *                int ia_data[]
 *                int *ib_size
 * Return Type  : int
 */
static int do_vectors(double b, unsigned char c_data[], int c_size[2],
                      int ia_data[], int *ib_size)
{
  int b_ialast;
  int ia_size;
  int iafirst;
  int ialast;
  int iblast;
  int nc;
  int nia;
  c_size[0] = 1;
  *ib_size = 0;
  nc = 0;
  nia = -1;
  iafirst = 0;
  ialast = 1;
  iblast = 1;
  while ((ialast <= 3) && (iblast <= 1)) {
    unsigned char ak;
    b_ialast = ialast;
    ak = (unsigned char)ialast;
    while ((b_ialast < 3) && (b_ialast == ialast - 1)) {
      b_ialast++;
    }
    ialast = b_ialast;
    // if(ak == b)
    if (fabs(ak - b) <= (double) 0.00000001f) {
      ialast = b_ialast + 1;
      iafirst = b_ialast;
      iblast = 2;
    } else if (ak < b) {
      nc++;
      nia++;
      c_data[nc - 1] = ak;
      ia_data[nia] = iafirst + 1;
      ialast = b_ialast + 1;
      iafirst = b_ialast;
    } else {
      iblast = 2;
    }
  }
  while (ialast <= 3) {
    b_ialast = ialast;
    while ((b_ialast < 3) && (b_ialast == ialast - 1)) {
      b_ialast++;
    }
    nc++;
    nia++;
    c_data[nc - 1] = (unsigned char)ialast;
    ia_data[nia] = iafirst + 1;
    ialast = b_ialast + 1;
    iafirst = b_ialast;
  }
  if (nia + 1 < 1) {
    nia = -1;
  }
  ia_size = nia + 1;
  if (nc < 1) {
    c_size[1] = 0;
  } else {
    c_size[1] = nc;
  }
  return ia_size;
}

/*
 * Arguments    : const float x[3]
 *                int *idx
 * Return Type  : float
 */
static float maximum(const float x[3], int *idx)
{
  float ex;
  int k;
  if (!rtIsNaNF(x[0])) {
    *idx = 1;
  } else {
    bool exitg1;
    *idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!rtIsNaNF(x[k - 1])) {
        *idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (*idx == 0) {
    ex = x[0];
    *idx = 1;
  } else {
    int i;
    ex = x[*idx - 1];
    i = *idx + 1;
    for (k = i; k < 4; k++) {
      float f;
      f = x[k - 1];
      if (ex < f) {
        ex = f;
        *idx = k;
      }
    }
  }
  return ex;
}

/*
 * Arguments    : const float x[4]
 *                int *idx
 * Return Type  : float
 */
static float minimum(const float x[4], int *idx)
{
  float ex;
  int k;
  if (!rtIsNaNF(x[0])) {
    *idx = 1;
  } else {
    bool exitg1;
    *idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 5)) {
      if (!rtIsNaNF(x[k - 1])) {
        *idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (*idx == 0) {
    ex = x[0];
    *idx = 1;
  } else {
    int i;
    ex = x[*idx - 1];
    i = *idx + 1;
    for (k = i; k < 5; k++) {
      float f;
      f = x[k - 1];
      if (ex > f) {
        ex = f;
        *idx = k;
      }
    }
  }
  return ex;
}

/*
 * Bounded Revised Simplex
 *
 * function [yout, inBout,bout, itout,errout] =
 * simplxuprevsol(A,ct,b,inB,inD,h,e,m,n,itlim)
 *
 *    Solves the linear program:
 *           minimize c'y
 *           subject to
 *           Ay = b
 *           0<= y <= h
 *
 *   Inputs:
 *           A [m,n]   = lhs Matrix of equaltity constraints
 *           ct [1,n]  = transpose of cost vector
 *           b [m,1]   = rhs vector for equality constraint
 *           inB [m]   = Vector of indices of unknowns in the initial basic set
 *           inD [n-m] = Vector of indices of unknowns not in the initial basic
 * set h[n,1]    = Upper Bound for unknowns e[n,1]    = Sign for unknown
 * variables (+ lower bound, - upper bound) Optional inputs: m,n       = number
 * of constraints, unknowns (Opposite standard CA convention
 *           itlim     = Upper bound on the allowed iterations\
 *
 *  Outputs:
 *          yout[n,1]  = Optimal output variable
 *          inBout     = indices of Basic vectors in output
 *          eout       = sign associate with output unknowns
 *          itout      = number of iterations remaining out of itlim
 *          errout     = Flag (=true) if unbounded is set
 *
 *  Modification History
 *    2002      Roger Beck  Original
 *    8/2014    Roger Beck  Update for use
 *    9/2014    Roger Beck  Added anti-cycling rule
 *
 * Arguments    : float A[8]
 *                float ct[4]
 *                float b[2]
 *                unsigned char inB[2]
 *                const float h[4]
 *                signed char e[4]
 *                unsigned int *itlim
 *                float b_y0[2]
 * Return Type  : bool
 */
static bool simplxuprevsol_C(float A[8], float ct[4], float b[2],
                             unsigned char inB[2], const float h[4],
                             signed char e[4], unsigned int *itlim,
                             float b_y0[2])
{
  float A_data[8];
  float ct_data[4];
  float lamt[2];
  float a21;
  float a21_tmp;
  float minrat;
  int rdt_size[2];
  int aoffset;
  int i;
  int j;
  int r1;
  int r2;
  int trueCount;
  signed char b_tmp_data[4];
  unsigned char inD_data[4];
  signed char tmp_data[4];
  bool tf[4];
  bool b1;
  bool b_b;
  bool errout;
  bool exitg1;
  bool indz_idx_0;
  bool y;
  /* Optional Inputs  	 */
  /* Tolerance for unknown == 0 */
  /* Index list for non-basic variables */
  /* Partition A */
  /*  #include <iostream> */
  /*  #include <set> */
  /*  #include <vector> */
  /*   */
  /*  std::vector<int> setdiff(const std::vector<int>& a, const
   * std::vector<int>& b) { */
  /*      std::set<int> a_set(a.begin(), a.end()); */
  /*      std::set<int> b_set(b.begin(), b.end()); */
  /*      std::vector<int> result; */
  /*   */
  /*      for (int elem : a_set) { */
  /*          if (b_set.find(elem) == b_set.end()) { */
  /*              result.push_back(elem); */
  /*          } */
  /*      } */
  /*   */
  /*      return result; */
  /*  } */
  /*   */
  /*  int main() { */
  /*      std::vector<int> array1 = {1, 2, 3, 4, 5}; */
  /*      std::vector<int> array2 = {4, 5, 6, 7, 8}; */
  /*   */
  /*      std::vector<int> diff = setdiff(array1, array2); */
  /*   */
  /*      for (int elem : diff) { */
  /*          std::cout << elem << " "; */
  /*      } */
  /*   */
  /*      return 0; */
  /*  } */
  /*  inD = setdiff(uint8(1:n), inB); */
  for (j = 0; j < 4; j++) {
    tf[j] = false;
    r2 = 0;
    exitg1 = false;
    while ((!exitg1) && (r2 < 2)) {
      if (j + 1 == inB[r2]) {
        tf[j] = true;
        exitg1 = true;
      } else {
        r2++;
      }
    }
  }
  trueCount = 0;
  indz_idx_0 = !tf[0];
  if (indz_idx_0) {
    trueCount = 1;
  }
  y = !tf[1];
  if (y) {
    trueCount++;
  }
  b_b = !tf[2];
  if (b_b) {
    trueCount++;
  }
  b1 = !tf[3];
  if (b1) {
    trueCount++;
  }
  j = 0;
  if (indz_idx_0) {
    inD_data[0] = 1U;
    j = 1;
  }
  if (y) {
    inD_data[j] = 2U;
    j++;
  }
  if (b_b) {
    inD_data[j] = 3U;
    j++;
  }
  if (b1) {
    inD_data[j] = 4U;
  }
  /* Adjust signs problem if variables are initialized at upper */
  /*  bounds. */
  r1 = 0;
  if (e[0] == 0) {
    r1 = 1;
  }
  if (e[1] == 0) {
    r1++;
  }
  if (e[2] == 0) {
    r1++;
  }
  if (e[3] == 0) {
    r1++;
  }
  j = 0;
  if (e[0] == 0) {
    tmp_data[0] = 0;
    j = 1;
  }
  if (e[1] == 0) {
    tmp_data[j] = 1;
    j++;
  }
  if (e[2] == 0) {
    tmp_data[j] = 2;
    j++;
  }
  if (e[3] == 0) {
    tmp_data[j] = 3;
  }
  for (i = 0; i < r1; i++) {
    aoffset = tmp_data[i] << 1;
    A_data[2 * i] = -A[aoffset];
    A_data[2 * i + 1] = -A[aoffset + 1];
  }
  for (i = 0; i < r1; i++) {
    signed char i1;
    i1 = tmp_data[i];
    aoffset = i1 << 1;
    A[aoffset] = A_data[2 * i];
    A[aoffset + 1] = A_data[2 * i + 1];
    ct_data[i] = -ct[i1];
  }
  lamt[0] = 0.0F;
  lamt[1] = 0.0F;
  for (r2 = 0; r2 < r1; r2++) {
    ct[tmp_data[r2]] = ct_data[r2];
    aoffset = r2 << 1;
    minrat = h[tmp_data[r2]];
    lamt[0] += A[aoffset % 2 + (tmp_data[aoffset / 2] << 1)] * minrat;
    lamt[1] +=
        A[(aoffset + 1) % 2 + (tmp_data[(aoffset + 1) / 2] << 1)] * minrat;
  }
  b[0] += lamt[0];
  b[1] += lamt[1];
  i = (inB[0] - 1) << 1;
  if (fabsf(A[i + 1]) > fabsf(A[i])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }
  a21_tmp = A[r1 + i];
  a21 = A[r2 + i] / a21_tmp;
  aoffset = (inB[1] - 1) << 1;
  minrat = A[r1 + aoffset];
  b_y0[1] = (b[r2] - b[r1] * a21) / (A[r2 + aoffset] - a21 * minrat);
  b_y0[0] = (b[r1] - b_y0[1] * minrat) / a21_tmp;
  /* Initial Solution */
  /* Initialize Loop Termination Conditions */
  errout = false;
  /* Main Simplex loop */
  exitg1 = false;
  while ((!exitg1) && (*itlim > 0U)) {
    float rdt_data[4];
    float f;
    float f1;
    float yq_tmp;
    int b_lamt_tmp;
    int c_lamt_tmp;
    int lamt_tmp;
    unsigned int q0;
    unsigned char qind;
    bool exitg2;
    q0 = *itlim;
    *itlim = q0 - 1U;
    if (q0 - 1U > q0) {
      *itlim = 0U;
    }
    /* Calculate transpose of relative cost vector based on current basis */
    i = (inB[0] - 1) << 1;
    f = fabsf(A[i + 1]);
    f1 = fabsf(A[i]);
    if (f > f1) {
      r1 = 1;
      r2 = 0;
    } else {
      r1 = 0;
      r2 = 1;
    }
    a21_tmp = A[r1 + i];
    a21 = A[r2 + i] / a21_tmp;
    lamt_tmp = inB[0] - 1;
    lamt[r1] = ct[lamt_tmp] / a21_tmp;
    b_lamt_tmp = (inB[1] - 1) << 1;
    minrat = A[r1 + b_lamt_tmp];
    c_lamt_tmp = inB[1] - 1;
    lamt[r2] = (ct[c_lamt_tmp] - lamt[r1] * minrat) /
               (A[r2 + b_lamt_tmp] - a21 * minrat);
    lamt[r1] -= lamt[r2] * a21;
    for (j = 0; j < trueCount; j++) {
      aoffset = (inD_data[j] - 1) << 1;
      ct_data[j] = lamt[0] * A[aoffset] + lamt[1] * A[aoffset + 1];
    }
    rdt_size[1] = trueCount;
    for (r1 = 0; r1 < trueCount; r1++) {
      rdt_data[r1] = ct[inD_data[r1] - 1] - ct_data[r1];
    }
    /* Find minimum relative cost */
    aoffset = rdt_size[1];
    if (rdt_size[1] <= 2) {
      if (rdt_size[1] == 1) {
        minrat = rdt_data[0];
        j = 1;
      } else {
        minrat = rdt_data[rdt_size[1] - 1];
        if ((rdt_data[0] > minrat) ||
            (rtIsNaNF(rdt_data[0]) && (!rtIsNaNF(minrat)))) {
          j = rdt_size[1];
        } else {
          minrat = rdt_data[0];
          j = 1;
        }
      }
    } else {
      if (!rtIsNaNF(rdt_data[0])) {
        j = 1;
      } else {
        j = 0;
        r2 = 2;
        exitg2 = false;
        while ((!exitg2) && (r2 <= aoffset)) {
          if (!rtIsNaNF(rdt_data[r2 - 1])) {
            j = r2;
            exitg2 = true;
          } else {
            r2++;
          }
        }
      }
      if (j == 0) {
        minrat = rdt_data[0];
        j = 1;
      } else {
        minrat = rdt_data[j - 1];
        r1 = j + 1;
        for (r2 = r1; r2 <= aoffset; r2++) {
          yq_tmp = rdt_data[r2 - 1];
          if (minrat > yq_tmp) {
            minrat = yq_tmp;
            j = r2;
          }
        }
      }
    }
    qind = (unsigned char)j;
    if (minrat >= 0.0F) {
      /*  If all relative costs are positive then the solution is optimal */
      exitg1 = true;
    } else {
      float yq[2];
      unsigned char qel;
      qel = inD_data[j - 1];
      /*  Unknown to Enter the basis minimizes relative cost */
      if (f > f1) {
        r1 = 1;
        r2 = 0;
      } else {
        r1 = 0;
        r2 = 1;
      }
      a21_tmp = A[r1 + i];
      a21 = A[r2 + i] / a21_tmp;
      j = (qel - 1) << 1;
      minrat = A[r1 + j];
      yq_tmp = A[r1 + b_lamt_tmp];
      yq[1] = (A[r2 + j] - minrat * a21) / (A[r2 + b_lamt_tmp] - a21 * yq_tmp);
      yq[0] = (minrat - yq[1] * yq_tmp) / a21_tmp;
      /* Vector to enter in terms of the current Basis vector */
      yq_tmp = fabsf(yq[0]);
      lamt[0] = yq_tmp;
      indz_idx_0 = (yq_tmp <= 1.0E-8F);
      yq_tmp = fabsf(yq[1]);
      lamt[1] = yq_tmp;
      y = true;
      r2 = 0;
      exitg2 = false;
      while ((!exitg2) && (r2 < 2)) {
        if (!(lamt[r2] <= 1.0E-8F)) {
          y = false;
          exitg2 = true;
        } else {
          r2++;
        }
      }
      if (y) {
        errout = true;
        /*  Check this condition */
        exitg1 = true;
      } else {
        float hinB_idx_0;
        float hinB_idx_1;
        bool guard1;
        /* Compute ratio how much each current basic variable will have to move
         * for the entering */
        /*  variable. */
        /*  If yq < 0 then increasing variable when it leaves the basis will
         * minimize cost */
        lamt[0] = b_y0[0] / yq[0];
        hinB_idx_0 = h[lamt_tmp];
        lamt[1] = b_y0[1] / yq[1];
        hinB_idx_1 = h[c_lamt_tmp];
        if (yq[0] < 0.0F) {
          lamt[0] -= hinB_idx_0 / yq[0];
        }
        if (yq[1] < 0.0F) {
          lamt[1] -= hinB_idx_1 / yq[1];
        }
        /*  If an element yq ~=0 then it doesn't change for the entering
         * variable and shouldn't */
        /*   be chosen */
        if (indz_idx_0) {
          lamt[0] = rtInfF;
        }
        if (yq_tmp <= 1.0E-8F) {
          lamt[1] = rtInfF;
        }
        /*  Variable to exit is moving to its minimum value */
        if ((lamt[0] > lamt[1]) ||
            (rtIsNaNF(lamt[0]) && (!rtIsNaNF(lamt[1])))) {
          minrat = lamt[1];
          aoffset = 1;
        } else {
          minrat = lamt[0];
          aoffset = 0;
        }
        /*  If the minimum ratio is zero, then the solution is degenerate and
         * the entering */
        /*    variable will not change the basis---invoke Bland's selection rule
         * to avoid */
        /*    cycling. */
        guard1 = false;
        if (fabsf(minrat) <= 1.0E-8F) {
          /*  Find negative relative cost */
          aoffset = rdt_size[1] - 1;
          j = 0;
          for (r1 = 0; r1 <= aoffset; r1++) {
            if (rdt_data[r1] < 0.0F) {
              b_tmp_data[j] = (signed char)r1;
              j++;
            }
          }
          /* Note that since minr <0 indm is not empty    */
          qind = (unsigned char)(b_tmp_data[0] + 1);
          qel = inD_data[b_tmp_data[0]];
          /*  Unknown to Enter the basis is first indexed to avoid cycling */
          if (f > f1) {
            r1 = 1;
            r2 = 0;
          } else {
            r1 = 0;
            r2 = 1;
          }
          a21_tmp = A[r1 + i];
          a21 = A[r2 + i] / a21_tmp;
          j = (qel - 1) << 1;
          minrat = A[r1 + j];
          yq_tmp = A[r1 + b_lamt_tmp];
          yq[1] =
              (A[r2 + j] - minrat * a21) / (A[r2 + b_lamt_tmp] - a21 * yq_tmp);
          yq[0] = (minrat - yq[1] * yq_tmp) / a21_tmp;
          /* Vector to enter in terms of the current Basis vector */
          f = fabsf(yq[0]);
          lamt[0] = f;
          indz_idx_0 = (f <= 1.0E-8F);
          f = fabsf(yq[1]);
          lamt[1] = f;
          y = true;
          r2 = 0;
          exitg2 = false;
          while ((!exitg2) && (r2 < 2)) {
            if (!(lamt[r2] <= 1.0E-8F)) {
              y = false;
              exitg2 = true;
            } else {
              r2++;
            }
          }
          if (y) {
            errout = true;
            /*  Check this condition */
            exitg1 = true;
          } else {
            /*  Recompute rations and determine variable to leave */
            lamt[0] = b_y0[0] / yq[0];
            lamt[1] = b_y0[1] / yq[1];
            /*  If yq < 0 then increasing variable when it leaves the basis will
             * minimize cost */
            if (yq[0] < 0.0F) {
              lamt[0] -= hinB_idx_0 / yq[0];
            }
            if (yq[1] < 0.0F) {
              lamt[1] -= hinB_idx_1 / yq[1];
            }
            /*  If an element yq ~=0 then it doesn't change for the entering
             * variable and shouldn't */
            /*   be chosen */
            if (indz_idx_0) {
              lamt[0] = rtInfF;
            }
            if (f <= 1.0E-8F) {
              lamt[1] = rtInfF;
            }
            /*  Variable to exit is moving to its minimum value--Note that min
             * returns the lowest index minimum */
            if ((lamt[0] > lamt[1]) ||
                (rtIsNaNF(lamt[0]) && (!rtIsNaNF(lamt[1])))) {
              minrat = lamt[1];
              aoffset = 1;
            } else {
              minrat = lamt[0];
              aoffset = 0;
            }
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
        if (guard1) {
          /*  Maintain the bounded simplex as only having lower bounds by
           * recasting  */
          /*  any variable that needs to move to its opposite bound. */
          f = h[qel - 1];
          if (minrat >= f) {
            /* Case 1: Entering variable goes to opposite bound and current
             * basis is maintained */
            e[qel - 1] = (signed char)(e[qel - 1] == 0);
            j = (qel - 1) << 1;
            minrat = A[j + 1];
            A[j] = -A[j];
            A[j + 1] = -minrat;
            b[0] += A[j] * f;
            b[1] += A[j + 1] * f;
            ct[qel - 1] = -ct[qel - 1];
          } else if (yq[aoffset] > 0.0F) {
            /* Case 2: Leaving variable returns to lower bound (0)	 */
            inD_data[qind - 1] = inB[aoffset];
            inB[aoffset] = qel;
          } else {
            unsigned char pel;
            /* Case 2: Leaving variable moves to upper bound	 */
            pel = inB[aoffset];
            i = inB[aoffset] - 1;
            e[i] = (signed char)(e[i] == 0);
            j = (inB[aoffset] - 1) << 1;
            minrat = A[j + 1];
            A[j] = -A[j];
            A[j + 1] = -minrat;
            inB[aoffset] = qel;
            inD_data[qind - 1] = pel;
            ct[i] = -ct[i];
            minrat = h[i];
            b[0] += A[j] * minrat;
            b[1] += A[j + 1] * minrat;
          }
          i = (inB[0] - 1) << 1;
          if (fabsf(A[i + 1]) > fabsf(A[i])) {
            r1 = 1;
            r2 = 0;
          } else {
            r1 = 0;
            r2 = 1;
          }
          a21_tmp = A[r1 + i];
          a21 = A[r2 + i] / a21_tmp;
          aoffset = (inB[1] - 1) << 1;
          minrat = A[r1 + aoffset];
          b_y0[1] = (b[r2] - b[r1] * a21) / (A[r2 + aoffset] - a21 * minrat);
          b_y0[0] = (b[r1] - b_y0[1] * minrat) / a21_tmp;
          /*  Compute new Basic solution; */
        }
      }
    }
  }
  return errout;
}

/*
 * come from [u] = LPwrap(IN_MAT) and using sigle data, define k, m for df4
 *  [k,m]=size(B);
 *
 * Arguments    : const float B[12]
 *                const float v[3]
 *                const float umin[4]
 *                const float umax[4]
 *                float u[4]
 *                float *z
 *                unsigned int *iters
 * Return Type  : void
 */
void allocator_dir_LPwrap_4(const float B[12], const float v[3],
                            const float umin[4], const float umax[4],
                            float u[4], float *z, unsigned int *iters)
{
  static float Bt_data[16];
  static float Ai[12];
  static float Bt[8];
  static const signed char iv[6] = {0, 0, 0, 0, 1, 1};
  float hi[6];
  float varargin_1[3];
  float minrat;
  int ia_data[3];
  int i;
  int ia_size;
  int iindx;
  unsigned int itlim;
  int r1;
  int r2;
  signed char tmp_data[4];
  signed char a__1;
  /*  Direction Preserving Control Allocation Linear Program */
  /*      Reduced formulation (Solution Scaled from Boundary) */
  /*  */
  /*  function [u,itlim,errout] = DPscaled_LPCA(yd,B,uMin,uMax,itlim); */
  /*  */
  /*     Solves the control allocation problem while preserving the */
  /*   objective direction for unattainable commands. The reduced */
  /*   dimension of the linear program passed to the Bounded Revised */
  /*   Simplex solver is formed by forcing the solution to be on the */
  /*   boundary of the AMS and eliminating the highest magnitude */
  /*   objective by solving the other constraints in terms of it. */
  /*  */
  /*   For yd outside the AMS, the solution returned is that the */
  /*   maximum in the direction of yd */
  /*     B*u= lamda*yd */
  /*     max lamda s.t. uMin <= u <= uMax */
  /*  */
  /*   Reducing the degrees of freedom elminates the problems of redundant */
  /*   solutions for attainable objectives. If the desired objective is on the
   */
  /*   interior of the AMS the solution is scaled from the solution on the */
  /*   boundary, yielding the same controls as the Direct Allocation solution.
   */
  /*    */
  /*   (In the text this solution is discussed in section A.5.3) */
  /*  */
  /*    (See Bodson, M., "Evaluation of Optimization Methods for */
  /*           Control Allocation",  AIAA 2001-4223). */
  /*  */
  /*   Inputs: */
  /*           yd [n]    = Desired objective */
  /*           B [n,m]   = Control Effectiveness matrix */
  /*           uMin[m,1] = Lower bound for controls */
  /*           uMax[m,1] = Upper bound for controls */
  /*           itlim     = Number of allowed iterations limit */
  /*                          (Sum of iterations in both branches) */
  /*  */
  /*  Outputs: */
  /*          u[m,1]     = Control Solution */
  /*          errout     = Error Status code */
  /*                          0 = found solution */
  /*                          <0 = Error in finding initial basic feasible
   * solution */
  /*                          >0 = Error in finding final solution */
  /*                          -1,1 = Solver error (unbounded solution) */
  /*                          -2   = Initial feasible solution not found */
  /*                          -3,3 = Iteration limit exceeded */
  /*          itlim      = Number of iterations remaining after solution found
   */
  /*  */
  /*  Calls: */
  /*          simplxuprevsol_C = Bounded Revised Simplex solver
   * (simplxuprevsol_C.m) */
  /*  */
  /*  Notes: */
  /*     If yd is close to zero, u = 0; */
  /*  */
  /*     Error code < 0 implies an error in the initialization and there is no
   * guarantee on */
  /*   the quality of the output solution other than the control limits. */
  /*     Error code > 0 for errors in final solution. */
  /*  */
  /*  Modification History */
  /*    2002      Roger Beck  Original ( DPcaLP2.m) */
  /*    8/2014    Roger Beck  Update */
  itlim = 500U;
  /* Initialize error code to zero */
  a__1 = 0;
  *z = 0.0F;
  /* Figure out how big the problem is (use standard CA definitions for m & n)
   */
  /*  [n,m] = size(B); */
  /*  Locate the maximum magnitude element in the desired objective */
  varargin_1[0] = fabsf(v[0]);
  varargin_1[1] = fabsf(v[1]);
  varargin_1[2] = fabsf(v[2]);
  minrat = maximum(varargin_1, &iindx);
  /* Trivial solution, if desired moment is close to zero */
  /*   May want to adjust the tolerance to improve numerics of later steps */
  if (minrat < 1.0E-8F) {
    /* yd = 0 ==> u=0 */
    /* Set flag to let caller know that it wasn't solved */
    u[0] = 0.0F;
    u[1] = 0.0F;
    u[2] = 0.0F;
    u[3] = 0.0F;
  } else {
    float A[8];
    float ydt[6];
    float h[4];
    float rdt[4];
    float ydt_data[4];
    float b[2];
    float b_b[2];
    float sb[2];
    float a21;
    float f;
    float f1;
    float ydt_idx_0;
    float ydt_idx_1;
    float yq_tmp;
    int iindx_data[4];
    int c_size[2];
    int loop_ub;
    int loop_ub_tmp;
    signed char ct[6];
    signed char e1[6];
    unsigned char inD[4];
    unsigned char c_data[3];
    unsigned char inB1[2];
    bool errsimp;
    bool exitg1;
    bool y;
    /* Transform Problem by Reordering Objectives with maximum first */
    do_vectors(iindx, c_data, c_size, ia_data, &r1);
    loop_ub_tmp = c_size[1L] + 1;
    i = iindx;
    if (iindx < 0) {
      i = 0;
    } else if (iindx > 255) {
      i = 255;
    }
    iindx_data[0] = i - 1;
    loop_ub = c_size[1L];
    for (i = 0; i < loop_ub; i++) {
      iindx_data[i + 1] = c_data[i] - 1;
    }
    for (i = 0; i < 4; i++) {
      for (r2 = 0; r2 < loop_ub_tmp; r2++) {
        Bt_data[r2 + loop_ub_tmp * i] = B[iindx_data[r2] + 3 * i];
      }
    }
    do_vectors(iindx, c_data, c_size, ia_data, &r1);
    loop_ub = c_size[1L] + 1;
    if (iindx < 0) {
      iindx = 0;
    } else if (iindx > 255) {
      iindx = 255;
    }
    iindx_data[0] = iindx - 1;
    ia_size = c_size[1L];
    for (i = 0; i < ia_size; i++) {
      iindx_data[i + 1] = c_data[i] - 1;
    }
    for (i = 0; i < loop_ub; i++) {
      ydt_data[i] = v[iindx_data[i]];
    }
    ydt_idx_1 = ydt_data[1];
    ydt_data[1] = ydt_data[2];
    ydt_data[2] = ydt_idx_1;
    for (i = 0; i < 4; i++) {
      r2 = loop_ub_tmp * i;
      f = Bt_data[r2 + 1];
      Bt_data[r2 + 1] = Bt_data[r2 + 2];
      Bt_data[r2 + 2] = f;
    }
    /* Convert into a LP problem */
    ydt[0] = ydt_data[1];
    ydt[2] = -ydt_data[0];
    minrat = -ydt_data[0] * 0.0F;
    ydt[3] = minrat;
    ydt[1] = ydt_idx_1;
    ydt[4] = minrat;
    ydt[5] = -ydt_data[0];
    for (i = 0; i < 2; i++) {
      f = ydt[i];
      f1 = ydt[i + 2];
      yq_tmp = ydt[i + 4];
      for (r2 = 0; r2 < 4; r2++) {
        A[i + (r2 << 1)] = (f * Bt_data[3 * r2] + f1 * Bt_data[3 * r2 + 1]) +
                           yq_tmp * Bt_data[3 * r2 + 2];
      }
    }
    for (i = 0; i < 8; i++) {
      Bt[i] = -A[i];
    }
    f = umin[0];
    f1 = umin[1];
    yq_tmp = umin[2];
    minrat = umin[3];
    for (i = 0; i < 2; i++) {
      b[i] = ((Bt[i] * f + Bt[i + 2] * f1) + Bt[i + 4] * yq_tmp) +
             Bt[i + 6] * minrat;
    }
    h[0] = umax[0] - umin[0];
    h[1] = umax[1] - umin[1];
    h[2] = umax[2] - umin[2];
    h[3] = umax[3] - umin[3];
    /* To find Feasible solution construct problem with appended slack variables
     */
    for (i = 0; i < 4; i++) {
      ia_size = i << 1;
      Ai[ia_size] = A[ia_size];
      Ai[ia_size + 1] = A[ia_size + 1];
    }
    Ai[8] = 2.0F * (float)(b[0] > 0.0F) - 1.0F;
    Ai[9] = 0.0F;
    Ai[10] = 0.0F;
    Ai[11] = 2.0F * (float)(b[1] > 0.0F) - 1.0F;
    hi[0] = h[0];
    hi[1] = h[1];
    hi[2] = h[2];
    hi[3] = h[3];
    hi[4] = 2.0F * fabsf(b[0]);
    hi[5] = 2.0F * fabsf(b[1]);
    /* Use Bounded Revised Simplex to find initial basic feasible point */
    /*   Bounded Revised Simplex */
    /*  */
    /* function [yout, inBout,bout, itout,errout] =
     * simplxuprevsol(A,ct,b,inB,inD,h,e,m,n,itlim) */
    /*  */
    /*    Solves the linear program: */
    /*           minimize c'y  */
    /*           subject to  */
    /*           Ay = b */
    /*           0<= y <= h */
    /*  */
    /*   Inputs:  */
    /*           A [m,n]   = lhs Matrix of equaltity constraints */
    /*           ct [1,n]  = transpose of cost vector */
    /*           b [m,1]   = rhs vector for equality constraint */
    /*           inB [m]   = Vector of indices of unknowns in the initial basic
     * set */
    /*           inD [n-m] = Vector of indices of unknowns not in the initial
     * basic set */
    /*           h[n,1]    = Upper Bound for unknowns */
    /*           e[n,1]    = Sign for unknown variables (+ lower bound, - upper
     * bound) */
    /*   Optional inputs: */
    /*           m,n       = number of constraints, unknowns (Opposite standard
     */
    /*                       CA convention */
    /*           itlim     = Upper bound on the allowed iterations\ */
    /*  */
    /*  Outputs: */
    /*          yout[n,1]  = Optimal output variable */
    /*          inBout     = indices of Basic vectors in output */
    /*          eout       = sign associate with output unknowns */
    /*          itout      = number of iterations remaining out of itlim */
    /*          errout     = Flag (=true) if unbounded is set */
    /*  */
    /*  Modification History */
    /*    2002      Roger Beck  Original */
    /*    8/2014    Roger Beck  Update for use */
    /*    9/2014    Roger Beck  Added anti-cycling rule */
    itlim = 500U;
    for (ia_size = 0; ia_size < 6; ia_size++) {
      ct[ia_size] = iv[ia_size];
      e1[ia_size] = 1;
    }
    inB1[0] = 5U;
    inB1[1] = 6U;
    /* Optional Inputs  	 */
    /* Tolerance for unknown == 0 */
    /* Index list for non-basic variables */
    /* Partition A */
    /*  #include <iostream> */
    /*  #include <set> */
    /*  #include <vector> */
    /*   */
    /*  std::vector<int> setdiff(const std::vector<int>& a, const
     * std::vector<int>& b) { */
    /*      std::set<int> a_set(a.begin(), a.end()); */
    /*      std::set<int> b_set(b.begin(), b.end()); */
    /*      std::vector<int> result; */
    /*   */
    /*      for (int elem : a_set) { */
    /*          if (b_set.find(elem) == b_set.end()) { */
    /*              result.push_back(elem); */
    /*          } */
    /*      } */
    /*   */
    /*      return result; */
    /*  } */
    /*   */
    /*  int main() { */
    /*      std::vector<int> array1 = {1, 2, 3, 4, 5}; */
    /*      std::vector<int> array2 = {4, 5, 6, 7, 8}; */
    /*   */
    /*      std::vector<int> diff = setdiff(array1, array2); */
    /*   */
    /*      for (int elem : diff) { */
    /*          std::cout << elem << " "; */
    /*      } */
    /*   */
    /*      return 0; */
    /*  } */
    /*  inD = setdiff(uint8(1:n), inB); */
    inD[0] = 1U;
    inD[1] = 2U;
    inD[2] = 3U;
    inD[3] = 4U;
    /* Adjust signs problem if variables are initialized at upper */
    /*  bounds. */
    b_b[0] = b[0];
    b_b[1] = b[1];
    a21 = 0.0F / Ai[8];
    ydt_idx_1 = (b[1] - b[0] * a21) / (Ai[11] - a21 * 0.0F);
    ydt_idx_0 = (b[0] - ydt_idx_1 * 0.0F) / Ai[8];
    /* Initial Solution */
    /* Initialize Loop Termination Conditions */
    errsimp = false;
    /* Main Simplex loop */
    exitg1 = false;
    while ((!exitg1) && (itlim > 0U)) {
      float a21_tmp;
      int sb_tmp_tmp_tmp;
      unsigned char qind;
      itlim--;
      /* Calculate transpose of relative cost vector based on current basis */
      i = (inB1[0] - 1) << 1;
      f = fabsf(Ai[i + 1]);
      f1 = fabsf(Ai[i]);
      if (f > f1) {
        r1 = 1;
        r2 = 0;
      } else {
        r1 = 0;
        r2 = 1;
      }
      a21_tmp = Ai[r1 + i];
      a21 = Ai[r2 + i] / a21_tmp;
      sb[r1] = (float)ct[inB1[0] - 1] / a21_tmp;
      sb_tmp_tmp_tmp = (inB1[1] - 1) << 1;
      minrat = Ai[r1 + sb_tmp_tmp_tmp];
      sb[r2] = ((float)ct[inB1[1] - 1] - sb[r1] * minrat) /
               (Ai[r2 + sb_tmp_tmp_tmp] - a21 * minrat);
      sb[r1] -= sb[r2] * a21;
      yq_tmp = sb[0];
      minrat = sb[1];
      for (r2 = 0; r2 < 4; r2++) {
        ia_size = inD[r2] - 1;
        r1 = ia_size << 1;
        rdt[r2] = (float)ct[ia_size] - (yq_tmp * Ai[r1] + minrat * Ai[r1 + 1]);
      }
      /* Find minimum relative cost */
      minrat = minimum(rdt, &iindx);
      if (iindx < 0) {
        iindx = 0;
      } else if (iindx > 255) {
        iindx = 255;
      }
      qind = (unsigned char)iindx;
      if (minrat >= 0.0F) {
        /*  If all relative costs are positive then the solution is optimal */
        exitg1 = true;
      } else {
        float yq[2];
        unsigned char qel;
        bool exitg2;
        bool indz_idx_0;
        qel = inD[(unsigned char)iindx - 1];
        /*  Unknown to Enter the basis minimizes relative cost */
        if (f > f1) {
          r1 = 1;
          r2 = 0;
        } else {
          r1 = 0;
          r2 = 1;
        }
        a21_tmp = Ai[r1 + i];
        a21 = Ai[r2 + i] / a21_tmp;
        ia_size = (qel - 1) << 1;
        yq_tmp = Ai[r1 + ia_size];
        minrat = Ai[r1 + sb_tmp_tmp_tmp];
        yq[1] = (Ai[r2 + ia_size] - yq_tmp * a21) /
                (Ai[r2 + sb_tmp_tmp_tmp] - a21 * minrat);
        yq[0] = (yq_tmp - yq[1] * minrat) / a21_tmp;
        /* Vector to enter in terms of the current Basis vector */
        yq_tmp = fabsf(yq[0]);
        sb[0] = yq_tmp;
        indz_idx_0 = (yq_tmp <= 1.0E-8F);
        yq_tmp = fabsf(yq[1]);
        sb[1] = yq_tmp;
        y = true;
        iindx = 0;
        exitg2 = false;
        while ((!exitg2) && (iindx < 2)) {
          if (!(sb[iindx] <= 1.0E-8F)) {
            y = false;
            exitg2 = true;
          } else {
            iindx++;
          }
        }
        if (y) {
          errsimp = true;
          /*  Check this condition */
          exitg1 = true;
        } else {
          float hinB_idx_0;
          float hinB_idx_1;
          bool guard1;
          /* Compute ratio how much each current basic variable will have to
           * move for the entering */
          /*  variable. */
          /*  If yq < 0 then increasing variable when it leaves the basis will
           * minimize cost */
          sb[0] = ydt_idx_0 / yq[0];
          hinB_idx_0 = hi[inB1[0] - 1];
          sb[1] = ydt_idx_1 / yq[1];
          hinB_idx_1 = hi[inB1[1] - 1];
          if (yq[0] < 0.0F) {
            sb[0] -= hinB_idx_0 / yq[0];
          }
          if (yq[1] < 0.0F) {
            sb[1] -= hinB_idx_1 / yq[1];
          }
          /*  If an element yq ~=0 then it doesn't change for the entering
           * variable and shouldn't */
          /*   be chosen */
          if (indz_idx_0) {
            sb[0] = rtInfF;
          }
          if (yq_tmp <= 1.0E-8F) {
            sb[1] = rtInfF;
          }
          /*  Variable to exit is moving to its minimum value */
          if ((sb[0] > sb[1]) || (rtIsNaNF(sb[0]) && (!rtIsNaNF(sb[1])))) {
            minrat = sb[1];
            r1 = 1;
          } else {
            minrat = sb[0];
            r1 = 0;
          }
          /*  If the minimum ratio is zero, then the solution is degenerate and
           * the entering */
          /*    variable will not change the basis---invoke Bland's selection
           * rule to avoid */
          /*    cycling. */
          guard1 = false;
          if (fabsf(minrat) <= 1.0E-8F) {
            /*  Find negative relative cost */
            ia_size = 0;
            if (rdt[0] < 0.0F) {
              tmp_data[0] = 0;
              ia_size = 1;
            }
            if (rdt[1] < 0.0F) {
              tmp_data[ia_size] = 1;
              ia_size++;
            }
            if (rdt[2] < 0.0F) {
              tmp_data[ia_size] = 2;
              ia_size++;
            }
            if (rdt[3] < 0.0F) {
              tmp_data[ia_size] = 3;
            }
            /* Note that since minr <0 indm is not empty    */
            qind = (unsigned char)(tmp_data[0] + 1);
            qel = inD[tmp_data[0]];
            /*  Unknown to Enter the basis is first indexed to avoid cycling */
            if (f > f1) {
              r1 = 1;
              r2 = 0;
            } else {
              r1 = 0;
              r2 = 1;
            }
            a21_tmp = Ai[r1 + i];
            a21 = Ai[r2 + i] / a21_tmp;
            ia_size = (qel - 1) << 1;
            yq_tmp = Ai[r1 + ia_size];
            minrat = Ai[r1 + sb_tmp_tmp_tmp];
            yq[1] = (Ai[r2 + ia_size] - yq_tmp * a21) /
                    (Ai[r2 + sb_tmp_tmp_tmp] - a21 * minrat);
            yq[0] = (yq_tmp - yq[1] * minrat) / a21_tmp;
            /* Vector to enter in terms of the current Basis vector */
            f = fabsf(yq[0]);
            sb[0] = f;
            indz_idx_0 = (f <= 1.0E-8F);
            f = fabsf(yq[1]);
            sb[1] = f;
            y = true;
            iindx = 0;
            exitg2 = false;
            while ((!exitg2) && (iindx < 2)) {
              if (!(sb[iindx] <= 1.0E-8F)) {
                y = false;
                exitg2 = true;
              } else {
                iindx++;
              }
            }
            if (y) {
              errsimp = true;
              /*  Check this condition */
              exitg1 = true;
            } else {
              /*  Recompute rations and determine variable to leave */
              sb[0] = ydt_idx_0 / yq[0];
              sb[1] = ydt_idx_1 / yq[1];
              /*  If yq < 0 then increasing variable when it leaves the basis
               * will minimize cost */
              if (yq[0] < 0.0F) {
                sb[0] -= hinB_idx_0 / yq[0];
              }
              if (yq[1] < 0.0F) {
                sb[1] -= hinB_idx_1 / yq[1];
              }
              /*  If an element yq ~=0 then it doesn't change for the entering
               * variable and shouldn't */
              /*   be chosen */
              if (indz_idx_0) {
                sb[0] = rtInfF;
              }
              if (f <= 1.0E-8F) {
                sb[1] = rtInfF;
              }
              /*  Variable to exit is moving to its minimum value--Note that min
               * returns the lowest index minimum */
              if ((sb[0] > sb[1]) || (rtIsNaNF(sb[0]) && (!rtIsNaNF(sb[1])))) {
                minrat = sb[1];
                r1 = 1;
              } else {
                minrat = sb[0];
                r1 = 0;
              }
              guard1 = true;
            }
          } else {
            guard1 = true;
          }
          if (guard1) {
            /*  Maintain the bounded simplex as only having lower bounds by
             * recasting  */
            /*  any variable that needs to move to its opposite bound. */
            f = hi[qel - 1];
            if (minrat >= f) {
              /* Case 1: Entering variable goes to opposite bound and current
               * basis is maintained */
              e1[qel - 1] = (signed char)(e1[qel - 1] == 0);
              iindx = (qel - 1) << 1;
              minrat = Ai[iindx + 1];
              Ai[iindx] = -Ai[iindx];
              Ai[iindx + 1] = -minrat;
              b_b[0] += Ai[iindx] * f;
              b_b[1] += Ai[iindx + 1] * f;
              ct[qel - 1] = (signed char)-ct[qel - 1];
            } else if (yq[r1] > 0.0F) {
              /* Case 2: Leaving variable returns to lower bound (0)	 */
              inD[qind - 1] = inB1[r1];
              inB1[r1] = qel;
            } else {
              unsigned char pel;
              /* Case 2: Leaving variable moves to upper bound	 */
              pel = inB1[r1];
              ia_size = inB1[r1] - 1;
              e1[ia_size] = (signed char)(e1[ia_size] == 0);
              iindx = ia_size << 1;
              minrat = Ai[iindx + 1];
              Ai[iindx] = -Ai[iindx];
              Ai[iindx + 1] = -minrat;
              inB1[r1] = qel;
              inD[qind - 1] = pel;
              ct[ia_size] = (signed char)-ct[ia_size];
              minrat = hi[ia_size];
              b_b[0] += Ai[iindx] * minrat;
              b_b[1] += Ai[iindx + 1] * minrat;
            }
            i = (inB1[0] - 1) << 1;
            if (fabsf(Ai[i + 1]) > fabsf(Ai[i])) {
              r1 = 1;
              r2 = 0;
            } else {
              r1 = 0;
              r2 = 1;
            }
            a21_tmp = Ai[r1 + i];
            a21 = Ai[r2 + i] / a21_tmp;
            iindx = (inB1[1] - 1) << 1;
            minrat = Ai[r1 + iindx];
            ydt_idx_1 =
                (b_b[r2] - b_b[r1] * a21) / (Ai[r2 + iindx] - a21 * minrat);
            ydt_idx_0 = (b_b[r1] - ydt_idx_1 * minrat) / a21_tmp;
            /*  Compute new Basic solution; */
          }
        }
      }
    }
    /* Check that Feasible Solution was found */
    if (itlim <= 0U) {
      a__1 = -3;
    }
    y = false;
    iindx = 0;
    exitg1 = false;
    while ((!exitg1) && (iindx < 2)) {
      if (inB1[iindx] > 4) {
        y = true;
        exitg1 = true;
      } else {
        iindx++;
      }
    }
    if (y) {
      a__1 = -2;
    }
    if (errsimp) {
      a__1 = -1;
    }
    if (a__1 == 0) {
      /*  No Error continue to solve problem */
      /* Solve using initial problem from above */
      rdt[0] = 0.0F;
      rdt[1] = 0.0F;
      rdt[2] = 0.0F;
      rdt[3] = 0.0F;
      for (iindx = 0; iindx < loop_ub_tmp; iindx++) {
        f = ydt_data[iindx];
        rdt[0] += -Bt_data[iindx] * f;
        rdt[1] += -Bt_data[iindx + loop_ub_tmp] * f;
        rdt[2] += -Bt_data[iindx + loop_ub_tmp * 2] * f;
        rdt[3] += -Bt_data[iindx + loop_ub_tmp * 3] * f;
      }
      tmp_data[0] = e1[0];
      tmp_data[1] = e1[1];
      tmp_data[2] = e1[2];
      tmp_data[3] = e1[3];
      simplxuprevsol_C(A, rdt, b, inB1, h, tmp_data, &itlim, sb);
      /* Construct solution to original LP problem from bounded simplex output
       */
      /*   Set non-basic variables to 0 or h based on e2 */
      /*   Set basic variables to y2 or h-y2. */
      u[0] = 0.0F;
      u[1] = 0.0F;
      u[2] = 0.0F;
      u[3] = 0.0F;
      u[inB1[0] - 1] = sb[0];
      u[inB1[1] - 1] = sb[1];
      if (tmp_data[0] == 0) {
        u[0] = -u[0] + h[0];
      }
      if (tmp_data[1] == 0) {
        u[1] = -u[1] + h[1];
      }
      if (tmp_data[2] == 0) {
        u[2] = -u[2] + h[2];
      }
      if (tmp_data[3] == 0) {
        u[3] = -u[3] + h[3];
      }
    } else {
      /*  Construct an incorrect solution to accompany error flags */
      /* out-of-bounds matrix access would cause program termination and was
       * eliminated */
    }
    /* Transform Solution Back Into control variables */
    /*  u(i) = x(i)+umin(i) if e(i) */
    /* Rescale controls so solution is not on boundary of Omega. */
    minrat = 0.0F;
    for (ia_size = 0; ia_size < 4; ia_size++) {
      u[ia_size] += umin[ia_size];
      r1 = ia_size * loop_ub_tmp;
      rdt[ia_size] = 0.0F;
      for (iindx = 0; iindx < loop_ub; iindx++) {
        rdt[ia_size] += ydt_data[iindx] * Bt_data[r1 + iindx];
      }
      minrat += rdt[ia_size] * u[ia_size];
    }
    yq_tmp = 0.0F;
    for (i = 0; i < loop_ub; i++) {
      f = ydt_data[i];
      yq_tmp += f * f;
    }
    *z = minrat / yq_tmp;
    if (*z > 1.0F) {
      u[0] /= *z;
      u[1] /= *z;
      u[2] /= *z;
      u[3] /= *z;
    }
  }
  *iters = 500U - itlim;
  if (500U - itlim > 500U) {
    *iters = 0U;
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void allocator_dir_LPwrap_4_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void allocator_dir_LPwrap_4_terminate(void)
{
}

/*
 * File trailer for allocator_dir_LPwrap_4.c
 *
 * [EOF]
 */
