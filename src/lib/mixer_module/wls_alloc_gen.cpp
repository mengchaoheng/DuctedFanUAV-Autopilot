//
// File: wls_alloc_gen.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 2025-06-12 19:52:30
//

// Include Files
#include "wls_alloc_gen.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Declarations
static void binary_expand_op(float in1[4], float in2,
                             const coder::array<float, 1U> &in3);

static void binary_expand_op_1(bool in1[4], const bool in2[4],
                               const coder::array<float, 1U> &in3);

static void binary_expand_op_2(bool in1[4], const bool in2[4],
                               const coder::array<float, 1U> &in3);

namespace coder {
namespace internal {
namespace blas {
static float xnrm2(int n, const float x_data[], int ix0);

}
namespace lapack {
static int xgeqp3(float A_data[], const int A_size[2], float tau_data[],
                  int jpvt_data[], int jpvt_size[2]);

}
} // namespace internal
} // namespace coder
static int div_nde_s32_floor(int numerator);

static void plus(float in1[4], const float in2[4],
                 const coder::array<float, 1U> &in3);

static float rt_hypotf(float u0, float u1);

// Function Definitions
//
// Arguments    : float in1[4]
//                float in2
//                const coder::array<float, 1U> &in3
// Return Type  : void
//
static void binary_expand_op(float in1[4], float in2,
                             const coder::array<float, 1U> &in3)
{
  int stride_0_0;
  stride_0_0 = (in3.size(0) != 1);
  in1[0] += in2 * in3[0];
  in1[1] += in2 * in3[stride_0_0];
  in1[2] += in2 * in3[stride_0_0 << 1];
  in1[3] += in2 * in3[3 * stride_0_0];
}

//
// Arguments    : bool in1[4]
//                const bool in2[4]
//                const coder::array<float, 1U> &in3
// Return Type  : void
//
static void binary_expand_op_1(bool in1[4], const bool in2[4],
                               const coder::array<float, 1U> &in3)
{
  int stride_0_0;
  stride_0_0 = (in3.size(0) != 1);
  in1[0] = (in2[0] && (in3[0] > 0.0F));
  in1[1] = (in2[1] && (in3[stride_0_0] > 0.0F));
  in1[2] = (in2[2] && (in3[stride_0_0 << 1] > 0.0F));
  in1[3] = (in2[3] && (in3[3 * stride_0_0] > 0.0F));
}

//
// Arguments    : bool in1[4]
//                const bool in2[4]
//                const coder::array<float, 1U> &in3
// Return Type  : void
//
static void binary_expand_op_2(bool in1[4], const bool in2[4],
                               const coder::array<float, 1U> &in3)
{
  int stride_0_0;
  stride_0_0 = (in3.size(0) != 1);
  in1[0] = (in2[0] && (in3[0] < 0.0F));
  in1[1] = (in2[1] && (in3[stride_0_0] < 0.0F));
  in1[2] = (in2[2] && (in3[stride_0_0 << 1] < 0.0F));
  in1[3] = (in2[3] && (in3[3 * stride_0_0] < 0.0F));
}

//
// Arguments    : int n
//                const float x_data[]
//                int ix0
// Return Type  : float
//
namespace coder {
namespace internal {
namespace blas {
static float xnrm2(int n, const float x_data[], int ix0)
{
  float scale;
  float y;
  int kend;
  y = 0.0F;
  scale = 1.29246971E-26F;
  kend = (ix0 + n) - 1;
  for (int k{ix0}; k <= kend; k++) {
    float absxk;
    absxk = std::abs(x_data[k - 1]);
    if (absxk > scale) {
      float t;
      t = scale / absxk;
      y = y * t * t + 1.0F;
      scale = absxk;
    } else {
      float t;
      t = absxk / scale;
      y += t * t;
    }
  }
  return scale * std::sqrt(y);
}

//
// Arguments    : float A_data[]
//                const int A_size[2]
//                float tau_data[]
//                int jpvt_data[]
//                int jpvt_size[2]
// Return Type  : int
//
} // namespace blas
namespace lapack {
static int xgeqp3(float A_data[], const int A_size[2], float tau_data[],
                  int jpvt_data[], int jpvt_size[2])
{
  float work_data[4];
  int tau_size;
  tau_size = A_size[1];
  if (tau_size - 1 >= 0) {
    std::memset(&tau_data[0], 0,
                static_cast<unsigned int>(tau_size) * sizeof(float));
  }
  if (A_size[1] < 1) {
    jpvt_size[0] = 1;
    jpvt_size[1] = tau_size;
    for (int iy{0}; iy < tau_size; iy++) {
      jpvt_data[iy] = iy + 1;
    }
  } else {
    float vn1_data[4];
    float vn2_data[4];
    float absxk;
    float scale;
    float smax;
    float t;
    int ix0;
    int iy;
    int k;
    int kend;
    jpvt_size[0] = 1;
    jpvt_size[1] = tau_size;
    for (k = 0; k < tau_size; k++) {
      jpvt_data[k] = k + 1;
      work_data[k] = 0.0F;
      vn1_data[k] = 0.0F;
      vn2_data[k] = 0.0F;
      ix0 = k * 7;
      smax = 0.0F;
      scale = 1.29246971E-26F;
      kend = ix0 + 7;
      for (iy = ix0 + 1; iy <= kend; iy++) {
        absxk = std::abs(A_data[iy - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          smax = smax * t * t + 1.0F;
          scale = absxk;
        } else {
          t = absxk / scale;
          smax += t * t;
        }
      }
      absxk = scale * std::sqrt(smax);
      vn1_data[k] = absxk;
      vn2_data[k] = absxk;
    }
    for (int i{0}; i < tau_size; i++) {
      int b_i;
      int ii;
      int ip1;
      int nmi;
      int pvt;
      ip1 = i + 2;
      ii = i * 7 + i;
      nmi = tau_size - i;
      if (nmi < 1) {
        kend = -1;
      } else {
        kend = 0;
        if (nmi > 1) {
          smax = std::abs(vn1_data[i]);
          for (k = 2; k <= nmi; k++) {
            scale = std::abs(vn1_data[(i + k) - 1]);
            if (scale > smax) {
              kend = k - 1;
              smax = scale;
            }
          }
        }
      }
      pvt = i + kend;
      if (pvt != i) {
        kend = pvt * 7;
        iy = i * 7;
        for (k = 0; k < 7; k++) {
          ix0 = kend + k;
          smax = A_data[ix0];
          b_i = iy + k;
          A_data[ix0] = A_data[b_i];
          A_data[b_i] = smax;
        }
        kend = jpvt_data[pvt];
        jpvt_data[pvt] = jpvt_data[i];
        jpvt_data[i] = kend;
        vn1_data[pvt] = vn1_data[i];
        vn2_data[pvt] = vn2_data[i];
      }
      t = A_data[ii];
      ix0 = ii + 2;
      tau_data[i] = 0.0F;
      smax = blas::xnrm2(6 - i, A_data, ii + 2);
      if (smax != 0.0F) {
        absxk = A_data[ii];
        scale = rt_hypotf(absxk, smax);
        if (absxk >= 0.0F) {
          scale = -scale;
        }
        if (std::abs(scale) < 9.86076132E-32F) {
          kend = 0;
          b_i = (ii - i) + 7;
          do {
            kend++;
            for (k = ix0; k <= b_i; k++) {
              A_data[k - 1] *= 1.01412048E+31F;
            }
            scale *= 1.01412048E+31F;
            t *= 1.01412048E+31F;
          } while ((std::abs(scale) < 9.86076132E-32F) && (kend < 20));
          scale = rt_hypotf(t, blas::xnrm2(6 - i, A_data, ii + 2));
          if (t >= 0.0F) {
            scale = -scale;
          }
          tau_data[i] = (scale - t) / scale;
          smax = 1.0F / (t - scale);
          for (k = ix0; k <= b_i; k++) {
            A_data[k - 1] *= smax;
          }
          for (k = 0; k < kend; k++) {
            scale *= 9.86076132E-32F;
          }
          t = scale;
        } else {
          tau_data[i] = (scale - absxk) / scale;
          smax = 1.0F / (absxk - scale);
          b_i = (ii - i) + 7;
          for (k = ix0; k <= b_i; k++) {
            A_data[k - 1] *= smax;
          }
          t = scale;
        }
      }
      A_data[ii] = t;
      if (i + 1 < tau_size) {
        int lastv;
        A_data[ii] = 1.0F;
        k = ii + 8;
        if (tau_data[i] != 0.0F) {
          bool exitg2;
          lastv = 7 - i;
          kend = (ii - i) + 6;
          while ((lastv > 0) && (A_data[kend] == 0.0F)) {
            lastv--;
            kend--;
          }
          nmi -= 2;
          exitg2 = false;
          while ((!exitg2) && (nmi + 1 > 0)) {
            int exitg1;
            kend = (ii + nmi * 7) + 7;
            ix0 = kend;
            do {
              exitg1 = 0;
              if (ix0 + 1 <= kend + lastv) {
                if (A_data[ix0] != 0.0F) {
                  exitg1 = 1;
                } else {
                  ix0++;
                }
              } else {
                nmi--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);
            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = 0;
          nmi = -1;
        }
        if (lastv > 0) {
          if (nmi + 1 != 0) {
            if (nmi >= 0) {
              std::memset(&work_data[0], 0,
                          static_cast<unsigned int>(nmi + 1) * sizeof(float));
            }
            b_i = (ii + 7 * nmi) + 8;
            for (iy = k; iy <= b_i; iy += 7) {
              smax = 0.0F;
              pvt = (iy + lastv) - 1;
              for (ix0 = iy; ix0 <= pvt; ix0++) {
                smax += A_data[ix0 - 1] * A_data[(ii + ix0) - iy];
              }
              kend = div_nde_s32_floor((iy - ii) - 8);
              work_data[kend] += smax;
            }
          }
          if (-tau_data[i] != 0.0F) {
            kend = ii;
            for (iy = 0; iy <= nmi; iy++) {
              absxk = work_data[iy];
              if (absxk != 0.0F) {
                smax = absxk * -tau_data[i];
                b_i = kend + 8;
                pvt = lastv + kend;
                for (ix0 = b_i; ix0 <= pvt + 7; ix0++) {
                  A_data[ix0 - 1] += A_data[((ii + ix0) - kend) - 8] * smax;
                }
              }
              kend += 7;
            }
          }
        }
        A_data[ii] = t;
      }
      for (iy = ip1; iy <= tau_size; iy++) {
        kend = i + (iy - 1) * 7;
        absxk = vn1_data[iy - 1];
        if (absxk != 0.0F) {
          smax = std::abs(A_data[kend]) / absxk;
          smax = 1.0F - smax * smax;
          if (smax < 0.0F) {
            smax = 0.0F;
          }
          scale = absxk / vn2_data[iy - 1];
          scale = smax * (scale * scale);
          if (scale <= 0.000345266977F) {
            absxk = blas::xnrm2(6 - i, A_data, kend + 2);
            vn1_data[iy - 1] = absxk;
            vn2_data[iy - 1] = absxk;
          } else {
            vn1_data[iy - 1] = absxk * std::sqrt(smax);
          }
        }
      }
    }
  }
  return tau_size;
}

//
// Arguments    : int numerator
// Return Type  : int
//
} // namespace lapack
} // namespace internal
} // namespace coder
static int div_nde_s32_floor(int numerator)
{
  int i;
  if ((numerator < 0) && (numerator % 7 != 0)) {
    i = -1;
  } else {
    i = 0;
  }
  return numerator / 7 + i;
}

//
// Arguments    : float in1[4]
//                const float in2[4]
//                const coder::array<float, 1U> &in3
// Return Type  : void
//
static void plus(float in1[4], const float in2[4],
                 const coder::array<float, 1U> &in3)
{
  int stride_0_0;
  stride_0_0 = (in3.size(0) != 1);
  in1[0] = in2[0] + in3[0];
  in1[1] = in2[1] + in3[stride_0_0];
  in1[2] = in2[2] + in3[stride_0_0 << 1];
  in1[3] = in2[3] + in3[3 * stride_0_0];
}

//
// Arguments    : float u0
//                float u1
// Return Type  : float
//
static float rt_hypotf(float u0, float u1)
{
  float a;
  float b;
  float y;
  a = std::abs(u0);
  b = std::abs(u1);
  if (a < b) {
    a /= b;
    y = b * std::sqrt(a * a + 1.0F);
  } else if (a > b) {
    b /= a;
    y = a * std::sqrt(b * b + 1.0F);
  } else {
    y = a * 1.41421354F;
  }
  return y;
}

//
// change for matlab generate C, just add the param m, remove values of optional
// arguments, then this fuction can be used as lib.
//  WLS_ALLOC - Control allocation using weighted least squares.
//
//   [u,W,iter] = wls_alloc(B,v,umin,umax,[Wv,Wu,ud,gamma,u0,W0,imax])
//
//  Solves the weighted, bounded least-squares problem
//
//    min ||Wu(u-ud)||^2 + gamma ||Wv(Bu-v)||^2
//
//    subj. to  umin <= u <= umax
//
//  using an active set method.
//
//   Inputs:
//   -------
//  B     control effectiveness matrix (k x m)
//  v     commanded virtual control (k x 1)
//  umin  lower position limits (m x 1)
//  umax  upper position limits (m x 1)
//  Wv    virtual control weighting matrix (k x k) [I]
//  Wu    control weighting matrix (m x m) [I]
//  ud    desired control (m x 1) [0]
//  gamma weight (scalar) [1e6]
//  u0    initial point (m x 1)
//  W0    initial working set (m x 1) [empty]
//  imax  max no. of iterations [100]
//
//   Outputs:
//   -------
//  u     optimal control
//  W     optimal active set
//  iter  no. of iterations (= no. of changes in the working set + 1)
//
//                             0 if u_i not saturated
//  Working set syntax: W_i = -1 if u_i = umin_i
//                            +1 if u_i = umax_i
//
//  See also: WLSC_ALLOC, IP_ALLOC, FXP_ALLOC, QP_SIM.
//
// Arguments    : const float B[12]
//                const float v[3]
//                const float umin[4]
//                const float umax[4]
//                const float Wv[9]
//                const float Wu[16]
//                const float ud[4]
//                float gam
//                float u[4]
//                float W[4]
//                float imax
//                float m
// Return Type  : void
//
void wls_alloc_gen(const float B[12], const float v[3], const float umin[4],
                   const float umax[4], const float Wv[9], const float Wu[16],
                   const float ud[4], float gam, float u[4], float W[4],
                   float imax, float m)
{
  coder::array<float, 1U> dist;
  coder::array<float, 1U> p;
  float A[28];
  float b_gam_sq[12];
  float d[7];
  float b_Wu[4];
  float u_opt[4];
  float c_gam_sq[3];
  float f;
  float f1;
  float f2;
  float gam_sq;
  int jpvt_data[4];
  int A_size[2];
  int aoffset;
  int i;
  int iter;
  int k;
  signed char tmp_data[4];
  bool bv[4];
  bool bv1[4];
  bool i_free[4];
  bool exitg1;
  //  Number of variables
  //  m = length(umin);
  //  Set default values of optional arguments
  //  if nargin < 11
  //    imax = 100; % Heuristic value
  //    [k,m] = size(B);
  //    if nargin < 10, u = (umin+umax)/2; W = zeros(m,1); end
  //    if nargin < 8,  gam = 1e6;       end
  //    if nargin < 7,  ud = zeros(m,1); end
  //    if nargin < 6,  Wu = eye(m);     end
  //    if nargin < 5,  Wv = eye(k);     end
  //  end
  gam_sq = std::sqrt(gam);
  for (i = 0; i < 3; i++) {
    f = Wv[i];
    f1 = Wv[i + 3];
    f2 = Wv[i + 6];
    for (k = 0; k < 4; k++) {
      b_gam_sq[i + 3 * k] =
          (gam_sq * f * B[3 * k] + gam_sq * f1 * B[3 * k + 1]) +
          gam_sq * f2 * B[3 * k + 2];
    }
  }
  for (i = 0; i < 4; i++) {
    A[7 * i] = b_gam_sq[3 * i];
    A[7 * i + 1] = b_gam_sq[3 * i + 1];
    A[7 * i + 2] = b_gam_sq[3 * i + 2];
    aoffset = i << 2;
    A[7 * i + 3] = Wu[aoffset];
    A[7 * i + 4] = Wu[aoffset + 1];
    A[7 * i + 5] = Wu[aoffset + 2];
    A[7 * i + 6] = Wu[aoffset + 3];
  }
  //  Initial residual.
  f = v[0];
  f1 = v[1];
  f2 = v[2];
  for (i = 0; i < 3; i++) {
    c_gam_sq[i] = (gam_sq * Wv[i] * f + gam_sq * Wv[i + 3] * f1) +
                  gam_sq * Wv[i + 6] * f2;
  }
  f = ud[0];
  f1 = ud[1];
  f2 = ud[2];
  gam_sq = ud[3];
  for (i = 0; i < 4; i++) {
    b_Wu[i] =
        ((Wu[i] * f + Wu[i + 4] * f1) + Wu[i + 8] * f2) + Wu[i + 12] * gam_sq;
  }
  d[0] = c_gam_sq[0];
  d[1] = c_gam_sq[1];
  d[2] = c_gam_sq[2];
  d[3] = b_Wu[0];
  d[4] = b_Wu[1];
  d[5] = b_Wu[2];
  d[6] = b_Wu[3];
  f = u[0];
  f1 = u[1];
  f2 = u[2];
  gam_sq = u[3];
  for (i = 0; i < 7; i++) {
    d[i] -= ((A[i] * f + A[i + 7] * f1) + A[i + 14] * f2) + A[i + 21] * gam_sq;
  }
  //  Determine indeces of free variables.
  i_free[0] = (W[0] == 0.0F);
  i_free[1] = (W[1] == 0.0F);
  i_free[2] = (W[2] == 0.0F);
  i_free[3] = (W[3] == 0.0F);
  //  Iterate until optimum is found or maximum number of iterations
  //  is reached.
  iter = 0;
  exitg1 = false;
  while ((!exitg1) && (iter <= static_cast<int>(imax) - 1)) {
    float A_data[28];
    float b_B[7];
    float p_free_data[4];
    int p_free_size;
    int rankA;
    int trueCount;
    bool exitg2;
    bool out;
    //  ----------------------------------------
    //   Compute optimal perturbation vector p.
    //  ----------------------------------------
    //  Eliminate saturated variables.
    trueCount = 0;
    if (i_free[0]) {
      trueCount = 1;
    }
    if (i_free[1]) {
      trueCount++;
    }
    if (i_free[2]) {
      trueCount++;
    }
    if (i_free[3]) {
      trueCount++;
    }
    aoffset = 0;
    if (i_free[0]) {
      tmp_data[0] = 0;
      aoffset = 1;
    }
    if (i_free[1]) {
      tmp_data[aoffset] = 1;
      aoffset++;
    }
    if (i_free[2]) {
      tmp_data[aoffset] = 2;
      aoffset++;
    }
    if (i_free[3]) {
      tmp_data[aoffset] = 3;
    }
    //  Solve the reduced optimization problem for free variables.
    if (trueCount == 0) {
      p_free_size = 0;
    } else {
      A_size[0] = 7;
      A_size[1] = trueCount;
      for (i = 0; i < trueCount; i++) {
        for (k = 0; k < 7; k++) {
          A_data[k + 7 * i] = A[k + 7 * tmp_data[i]];
        }
      }
      int jpvt_size[2];
      coder::internal::lapack::xgeqp3(A_data, A_size, b_Wu, jpvt_data,
                                      jpvt_size);
      rankA = 0;
      if (A_size[1] > 0) {
        gam_sq = 8.34465E-6F * std::abs(A_data[0]);
        while ((rankA < A_size[1]) &&
               (std::abs(A_data[rankA + 7 * rankA]) > gam_sq)) {
          rankA++;
        }
      }
      for (int b_i{0}; b_i < 7; b_i++) {
        b_B[b_i] = d[b_i];
      }
      aoffset = A_size[1];
      p_free_size = A_size[1];
      for (int j{0}; j < aoffset; j++) {
        p_free_data[j] = 0.0F;
        if (b_Wu[j] != 0.0F) {
          gam_sq = b_B[j];
          i = j + 2;
          for (int b_i{i}; b_i < 8; b_i++) {
            gam_sq += A_data[(b_i + 7 * j) - 1] * b_B[b_i - 1];
          }
          gam_sq *= b_Wu[j];
          if (gam_sq != 0.0F) {
            b_B[j] -= gam_sq;
            for (int b_i{i}; b_i < 8; b_i++) {
              b_B[b_i - 1] -= A_data[(b_i + 7 * j) - 1] * gam_sq;
            }
          }
        }
      }
      for (int b_i{0}; b_i < rankA; b_i++) {
        p_free_data[jpvt_data[b_i] - 1] = b_B[b_i];
      }
      for (int j{rankA}; j >= 1; j--) {
        i = jpvt_data[j - 1];
        aoffset = 7 * (j - 1);
        p_free_data[i - 1] /= A_data[(j + aoffset) - 1];
        for (int b_i{0}; b_i <= j - 2; b_i++) {
          k = jpvt_data[b_i];
          p_free_data[k - 1] -= p_free_data[i - 1] * A_data[b_i + aoffset];
        }
      }
    }
    //  Zero all perturbations corresponding to active constraints.
    rankA = static_cast<int>(m);
    p.set_size(rankA);
    for (i = 0; i < rankA; i++) {
      p[i] = 0.0F;
    }
    //  Insert perturbations from p_free into free the variables.
    for (i = 0; i < p_free_size; i++) {
      p[static_cast<int>(tmp_data[i])] = p_free_data[i];
    }
    //  ----------------------------
    //   Is the new point feasible?
    //  ----------------------------
    if (p.size(0) == 4) {
      u_opt[0] = u[0] + p[0];
      u_opt[1] = u[1] + p[1];
      u_opt[2] = u[2] + p[2];
      u_opt[3] = u[3] + p[3];
    } else {
      plus(u_opt, u, p);
    }
    out = false;
    aoffset = 0;
    exitg2 = false;
    while ((!exitg2) && (aoffset + 1 <= trueCount)) {
      f = u_opt[tmp_data[aoffset]];
      if ((f < umin[tmp_data[aoffset]]) || (f > umax[tmp_data[aoffset]])) {
        out = true;
        exitg2 = true;
      } else {
        aoffset++;
      }
    }
    if (!out) {
      //  ----------------------------
      //   Yes, check for optimality.
      //  ----------------------------
      //  Update point and residual.
      u[0] = u_opt[0];
      u[1] = u_opt[1];
      u[2] = u_opt[2];
      u[3] = u_opt[3];
      for (int b_i{0}; b_i < 7; b_i++) {
        b_B[b_i] = 0.0F;
      }
      for (k = 0; k < trueCount; k++) {
        aoffset = k * 7;
        for (int b_i{0}; b_i < 7; b_i++) {
          i = aoffset + b_i;
          b_B[b_i] += A[i % 7 + 7 * tmp_data[i / 7]] * p_free_data[k];
        }
      }
      for (i = 0; i < 7; i++) {
        d[i] -= b_B[i];
      }
      //  Compute Lagrangian multipliers.
      for (i = 0; i < 4; i++) {
        f = 0.0F;
        for (k = 0; k < 7; k++) {
          f += A[k + 7 * i] * d[k];
        }
        u_opt[i] = W[i] * f;
      }
      //  Are all lambda non-negative?
      out = true;
      k = 0;
      exitg2 = false;
      while ((!exitg2) && (k < 4)) {
        if (u_opt[k] < -2.22044605E-16F) {
          out = false;
          exitg2 = true;
        } else {
          k++;
        }
      }
      if (out) {
        //  / ------------------------ \
                //  | Optimum found, bail out. |
        //  \ ------------------------ /
        exitg1 = true;
      } else {
        //  --------------------------------------------------
        //   Optimum not found, remove one active constraint.
        //  --------------------------------------------------
        //  Remove constraint with most negative lambda from the
        //  working set.
        gam_sq = u_opt[0];
        p_free_size = 0;
        if (u_opt[0] > u_opt[1]) {
          gam_sq = u_opt[1];
          p_free_size = 1;
        }
        if (gam_sq > u_opt[2]) {
          gam_sq = u_opt[2];
          p_free_size = 2;
        }
        if (gam_sq > u_opt[3]) {
          p_free_size = 3;
        }
        W[p_free_size] = 0.0F;
        i_free[p_free_size] = true;
        iter++;
      }
    } else {
      //  ---------------------------------------
      //   No, find primary bounding constraint.
      //  ---------------------------------------
      //  Compute distances to the different boundaries. Since alpha < 1
      //  is the maximum step length, initiate with ones.
      dist.set_size(rankA);
      for (i = 0; i < rankA; i++) {
        dist[i] = 1.0F;
      }
      if (p.size(0) == 4) {
        bv[0] = (i_free[0] && (p[0] < 0.0F));
        bv[1] = (i_free[1] && (p[1] < 0.0F));
        bv[2] = (i_free[2] && (p[2] < 0.0F));
        bv[3] = (i_free[3] && (p[3] < 0.0F));
      } else {
        binary_expand_op_2(bv, i_free, p);
      }
      if (bv[0]) {
        dist[0] = (umin[0] - u[0]) / p[0];
      }
      if (bv[1]) {
        dist[1] = (umin[1] - u[1]) / p[1];
      }
      if (bv[2]) {
        dist[2] = (umin[2] - u[2]) / p[2];
      }
      if (bv[3]) {
        dist[3] = (umin[3] - u[3]) / p[3];
      }
      if (p.size(0) == 4) {
        bv1[0] = (i_free[0] && (p[0] > 0.0F));
        bv1[1] = (i_free[1] && (p[1] > 0.0F));
        bv1[2] = (i_free[2] && (p[2] > 0.0F));
        bv1[3] = (i_free[3] && (p[3] > 0.0F));
      } else {
        binary_expand_op_1(bv1, i_free, p);
      }
      if (bv1[0]) {
        dist[0] = (umax[0] - u[0]) / p[0];
      }
      if (bv1[1]) {
        dist[1] = (umax[1] - u[1]) / p[1];
      }
      if (bv1[2]) {
        dist[2] = (umax[2] - u[2]) / p[2];
      }
      if (bv1[3]) {
        dist[3] = (umax[3] - u[3]) / p[3];
      }
      //  Proportion of p to travel
      if (dist.size(0) <= 2) {
        if (dist.size(0) == 1) {
          gam_sq = dist[0];
          p_free_size = 1;
        } else {
          gam_sq = dist[dist.size(0) - 1];
          if (dist[0] > gam_sq) {
            p_free_size = static_cast<int>(m);
          } else {
            gam_sq = dist[0];
            p_free_size = 1;
          }
        }
      } else {
        gam_sq = dist[0];
        p_free_size = 1;
        for (k = 2; k <= rankA; k++) {
          f = dist[k - 1];
          if (gam_sq > f) {
            gam_sq = f;
            p_free_size = k;
          }
        }
      }
      //  Update point and residual.
      if (p.size(0) == 4) {
        u[0] += gam_sq * p[0];
        u[1] += gam_sq * p[1];
        u[2] += gam_sq * p[2];
        u[3] += gam_sq * p[3];
      } else {
        binary_expand_op(u, gam_sq, p);
      }
      for (i = 0; i < trueCount; i++) {
        for (k = 0; k < 7; k++) {
          A_data[k + 7 * i] = A[k + 7 * tmp_data[i]] * gam_sq;
        }
      }
      for (int b_i{0}; b_i < 7; b_i++) {
        b_B[b_i] = 0.0F;
      }
      for (k = 0; k < trueCount; k++) {
        aoffset = k * 7;
        for (int b_i{0}; b_i < 7; b_i++) {
          b_B[b_i] += A_data[aoffset + b_i] * p_free_data[k];
        }
      }
      for (i = 0; i < 7; i++) {
        d[i] -= b_B[i];
      }
      //  Add corresponding constraint to working set.
      gam_sq = p[static_cast<int>(static_cast<float>(p_free_size)) - 1];
      if (gam_sq < 0.0F) {
        W[static_cast<int>(static_cast<float>(p_free_size)) - 1] = -1.0F;
      } else {
        W[static_cast<int>(static_cast<float>(p_free_size)) - 1] =
            (gam_sq > 0.0F);
      }
      i_free[static_cast<int>(static_cast<float>(p_free_size)) - 1] = false;
      iter++;
    }
  }
}

//
// Arguments    : void
// Return Type  : void
//
void wls_alloc_gen_initialize()
{
}

//
// Arguments    : void
// Return Type  : void
//
void wls_alloc_gen_terminate()
{
}

//
// File trailer for wls_alloc_gen.cpp
//
// [EOF]
//
