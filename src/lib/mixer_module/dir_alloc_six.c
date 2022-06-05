/*
 * File: dir_alloc_six.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 03-Jun-2022 18:26:31
 */

/* Include Files */
#include "dir_alloc_six.h"
#include <string.h>

/* Function Definitions */
/*
 * (c) mengchaoheng
 *  Last edited 2019-11
 *    min z=c*x   subj. to  A*x (=、 >=、 <=) b
 *    x
 *  原问题
 *  Performs direct control allocation by solving the LP
 *    max z=a   subj. to  Bu = av
 *    a,u               umin <= u <= umax
 *  If a > 1, set u = u/a.
 *  Note: This function has not been optimized for speed.
 *   Inputs:
 *   -------
 *  B     control effectiveness matrix (k x m)
 *  v     commanded virtual control (k x 1)
 *  umin  lower position limits (m x 1)
 *  umax  upper position limits (m x 1)
 *   Outputs:
 *   -------
 *  u     optimal control (m x 1)
 *  a     scaling factor
 *  整理成
 *    min z=[0 -1]x   subj. to  [B -v]x = 0
 *    x                       [I 0;-I 0]x <= [umax; -umin]
 *    其中 x=[u; a]
 *  对应《凸优化》p139,记为
 *    min z=c*x   subj. to  Aeq*x = beq
 *    x                     G*x <= h
 *  合并
 *    min z=c*x   subj. to  [Aeq; G]*x (=、<=) [beq;h]
 *    x
 *  保证x>=0，变形
 *    min z=[c -c]*X   subj. to  [Aeq -Aeq;G -G]*X (=、<=) [beq;h]
 *     X
 *  其中 X=[x^+; x^-]
 *
 *
 * Arguments    : const float umin[6]
 *                const float umax[6]
 *                const float v[3]
 *                const float B[18]
 *                float u[6]
 *                float *z
 *                short *iters
 * Return Type  : void
 */
void dir_alloc_six(const float umin[6], const float umax[6], const float v[3],
                   const float B[18], float u[6], float *z, short *iters)
{
  static float A[476];
  static float P[289];
  static float b_B[289];
  static float Ad[238];
  static float Ad_eye[238];
  static float c[28];
  static float x[28];
  static const signed char iv[196] = {
      1, 0, 0,  0,  0, 0, 0, -1, 0,  0, 0, 0, 0,  0,  0, 1, 0, 0,  0,  0,
      0, 0, -1, 0,  0, 0, 0, 0,  0,  0, 1, 0, 0,  0,  0, 0, 0, -1, 0,  0,
      0, 0, 0,  0,  0, 1, 0, 0,  0,  0, 0, 0, -1, 0,  0, 0, 0, 0,  0,  0,
      1, 0, 0,  0,  0, 0, 0, -1, 0,  0, 0, 0, 0,  0,  0, 1, 0, 0,  0,  0,
      0, 0, -1, 0,  0, 0, 0, 0,  0,  0, 1, 0, 0,  0,  0, 0, 0, -1, -1, 0,
      0, 0, 0,  0,  0, 1, 0, 0,  0,  0, 0, 0, 0,  -1, 0, 0, 0, 0,  0,  0,
      1, 0, 0,  0,  0, 0, 0, 0,  -1, 0, 0, 0, 0,  0,  0, 1, 0, 0,  0,  0,
      0, 0, 0,  -1, 0, 0, 0, 0,  0,  0, 1, 0, 0,  0,  0, 0, 0, 0,  -1, 0,
      0, 0, 0,  0,  0, 1, 0, 0,  0,  0, 0, 0, 0,  -1, 0, 0, 0, 0,  0,  0,
      1, 0, 0,  0,  0, 0, 0, 0,  -1, 0, 0, 0, 0,  0,  0, 1};
  static const signed char iv1[196] = {
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  static const signed char iv3[28] = {0, 0, 0, 0, 0, 0, -1, 0, 0, 0,
                                      0, 0, 0, 1, 0, 0, 0,  0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0,  0};
  static const signed char iv2[17] = {1,  2,  3,  15, 16, 17, 18, 19, 20,
                                      21, 22, 23, 24, 25, 26, 27, 28};
  static signed char b_I[289];
  float b_A[28];
  float Aeq[21];
  float b[17];
  float f;
  float temp2;
  float tmp_ii;
  int Aeq_tmp;
  int P_tmp;
  int b_P_tmp;
  int b_div_i_tmp;
  int div_i_tmp;
  int exitg1;
  int i;
  int jj;
  short L;
  short e;
  signed char c_B[17];
  bool exitg2;
  bool flag;
  /*  function [u,z,iters] = dir_alloc_six(umin,umax,v,A,P_inv) */
  /* b求解线性规划 */
  b[0] = 0.0F;
  b[1] = 0.0F;
  b[2] = 0.0F;
  b[9] = 20.0F;
  for (i = 0; i < 6; i++) {
    Aeq[3 * i] = B[3 * i];
    Aeq_tmp = 3 * i + 1;
    Aeq[Aeq_tmp] = B[Aeq_tmp];
    Aeq_tmp = 3 * i + 2;
    Aeq[Aeq_tmp] = B[Aeq_tmp];
    b[i + 3] = umax[i];
    b[i + 10] = -umin[i];
  }
  Aeq[18] = -v[0];
  Aeq[19] = -v[1];
  Aeq[20] = -v[2];
  b[16] = 0.0F;
  /*  构造线性规划标准型 */
  /*  Convert free variables to positively constrained variables */
  for (div_i_tmp = 0; div_i_tmp < 7; div_i_tmp++) {
    f = Aeq[3 * div_i_tmp];
    Ad[17 * div_i_tmp] = f;
    Aeq_tmp = 17 * (div_i_tmp + 7);
    Ad[Aeq_tmp] = -f;
    f = Aeq[3 * div_i_tmp + 1];
    Ad[17 * div_i_tmp + 1] = f;
    Ad[Aeq_tmp + 1] = -f;
    f = Aeq[3 * div_i_tmp + 2];
    Ad[17 * div_i_tmp + 2] = f;
    Ad[Aeq_tmp + 2] = -f;
  }
  for (div_i_tmp = 0; div_i_tmp < 14; div_i_tmp++) {
    for (Aeq_tmp = 0; Aeq_tmp < 14; Aeq_tmp++) {
      Ad[(Aeq_tmp + 17 * div_i_tmp) + 3] = iv[Aeq_tmp + 14 * div_i_tmp];
    }
  }
  /*  Ad只有第7、第14列的前三行根据v不同而不同，其他固定不变 */
  /*  Ad=[B -v -B v; eye(7) -eye(7);-eye(7) eye(7)]; */
  /*   Ad7=[-v;0;0;0;0;0;0;1;0;0;0;0;0;0;-1]; */
  /*  [mad,~]= size(Ad); */
  /*  先把前三个等式的基找到，并化简 */
  for (div_i_tmp = 0; div_i_tmp < 3; div_i_tmp++) {
    P[17 * div_i_tmp] = Ad[17 * div_i_tmp];
    P_tmp = 17 * div_i_tmp + 1;
    P[P_tmp] = Ad[P_tmp];
    P_tmp = 17 * div_i_tmp + 2;
    P[P_tmp] = Ad[P_tmp];
  }
  for (div_i_tmp = 0; div_i_tmp < 14; div_i_tmp++) {
    P_tmp = 17 * (div_i_tmp + 3);
    P[P_tmp] = 0.0F;
    P[P_tmp + 1] = 0.0F;
    P[P_tmp + 2] = 0.0F;
  }
  for (div_i_tmp = 0; div_i_tmp < 3; div_i_tmp++) {
    for (Aeq_tmp = 0; Aeq_tmp < 14; Aeq_tmp++) {
      P_tmp = (Aeq_tmp + 17 * div_i_tmp) + 3;
      P[P_tmp] = Ad[P_tmp];
    }
  }
  for (div_i_tmp = 0; div_i_tmp < 14; div_i_tmp++) {
    for (Aeq_tmp = 0; Aeq_tmp < 14; Aeq_tmp++) {
      P[(Aeq_tmp + 17 * (div_i_tmp + 3)) + 3] = iv1[Aeq_tmp + 14 * div_i_tmp];
    }
  }
  /*  常量 */
  /*  P_inv=[ -2    -4     6     0     0     0     0     0     0     0     0
   * 0     0     0     0     0     0; */
  /*           6     4    -6     0     0     0     0     0     0     0     0
   * 0     0     0     0     0     0; */
  /*          -4     0     6     0     0     0     0     0     0     0     0
   * 0     0     0     0     0     0; */
  /*           2     4    -6     1     0     0     0     0     0     0     0
   * 0     0     0     0     0     0; */
  /*          -6    -4     6     0     1     0     0     0     0     0     0
   * 0     0     0     0     0     0; */
  /*           4     0    -6     0     0     1     0     0     0     0     0
   * 0     0     0     0     0     0; */
  /*           0     0     0     0     0     0     1     0     0     0     0
   * 0     0     0     0     0     0; */
  /*           0     0     0     0     0     0     0     1     0     0     0
   * 0     0     0     0     0     0; */
  /*           0     0     0     0     0     0     0     0     1     0     0
   * 0     0     0     0     0     0; */
  /*           0     0     0     0     0     0     0     0     0     1     0
   * 0     0     0     0     0     0; */
  /*          -2    -4     6     0     0     0     0     0     0     0     1
   * 0     0     0     0     0     0; */
  /*           6     4    -6     0     0     0     0     0     0     0     0
   * 1     0     0     0     0     0; */
  /*          -4     0     6     0     0     0     0     0     0     0     0
   * 0     1     0     0     0     0; */
  /*           0     0     0     0     0     0     0     0     0     0     0
   * 0     0     1     0     0     0; */
  /*           0     0     0     0     0     0     0     0     0     0     0
   * 0     0     0     1     0     0; */
  /*           0     0     0     0     0     0     0     0     0     0     0
   * 0     0     0     0     1     0; */
  /*           0     0     0     0     0     0     0     0     0     0     0
   * 0     0     0     0     0     1]; */
  /*  求逆 */
  /*  Ad_eye=P\Ad;% 化简 */
  /*  无关列的逆阵P_inv是常矩阵 */
  /*  对矩阵进行初等行变换求其逆 */
  /*  function Ad_eye=inv_mvh(B_inv,Ad) */
  /*  Ad_eye=B_inv\Ad;% 化简 */
  /*  [row, col] = size(A); */
  /*  B为单位矩阵 */
  memset(&b_I[0], 0, 289U * sizeof(signed char));
  for (Aeq_tmp = 0; Aeq_tmp < 17; Aeq_tmp++) {
    b_I[Aeq_tmp + 17 * Aeq_tmp] = 1;
  }
  for (div_i_tmp = 0; div_i_tmp < 289; div_i_tmp++) {
    b_B[div_i_tmp] = b_I[div_i_tmp];
  }
  for (i = 0; i < 17; i++) {
    /*  依次将对角行的元素归一化 */
    b_div_i_tmp = 17 * i;
    div_i_tmp = i + b_div_i_tmp;
    tmp_ii = P[div_i_tmp];
    for (Aeq_tmp = 0; Aeq_tmp < 17; Aeq_tmp++) {
      P_tmp = i + 17 * Aeq_tmp;
      P[P_tmp] /= tmp_ii;
      b_B[P_tmp] /= tmp_ii;
    }
    for (Aeq_tmp = 0; Aeq_tmp < 17; Aeq_tmp++) {
      tmp_ii = -P[Aeq_tmp + b_div_i_tmp] / P[div_i_tmp];
      if (i + 1 == Aeq_tmp + 1) {
        tmp_ii = 0.0F;
      }
      /*  初等行变换 */
      for (jj = 0; jj < 17; jj++) {
        P_tmp = 17 * jj;
        b_P_tmp = Aeq_tmp + P_tmp;
        P_tmp += i;
        P[b_P_tmp] += tmp_ii * P[P_tmp];
        b_B[b_P_tmp] += tmp_ii * b_B[P_tmp];
      }
    }
  }
  /*  常量 */
  /*  Ad_eye =[ */
  /*   */
  /*      1.0000   -0.0000   -0.0000    1.0000    2.0000    2.0000  0   -1.0000
   * 0.0000    0.0000   -1.0000   -2.0000   -2.0000  0; */
  /*      0.0000    1.0000    0.0000   -2.0000   -3.0000   -2.0000  0   -0.0000
   * -1.0000   -0.0000    2.0000    3.0000    2.0000  0; */
  /*     -0.0000   -0.0000    1.0000    2.0000    2.0000    1.0000  0    0.0000
   * 0.0000   -1.0000   -2.0000   -2.0000   -1.0000  0; */
  /*           0    0.0000    0.0000   -1.0000   -2.0000   -2.0000  0         0
   * -0.0000   -0.0000    1.0000    2.0000    2.0000  0; */
  /*     -0.0000         0   -0.0000    2.0000    3.0000    2.0000  0    0.0000
   * 0    0.0000   -2.0000   -3.0000   -2.0000  0; */
  /*      0.0000    0.0000         0   -2.0000   -2.0000   -1.0000  0   -0.0000
   * -0.0000         0    2.0000    2.0000    1.0000  0; */
  /*           0         0         0    1.0000         0         0  0         0
   * 0         0   -1.0000         0         0  0; */
  /*           0         0         0         0    1.0000         0  0         0
   * 0         0         0   -1.0000         0  0; */
  /*           0         0         0         0         0    1.0000  0         0
   * 0         0         0         0   -1.0000  0; */
  /*           0         0         0         0         0         0  0         0
   * 0         0         0         0         0  0; */
  /*           0   -0.0000   -0.0000    1.0000    2.0000    2.0000  0         0
   * 0.0000    0.0000   -1.0000   -2.0000   -2.0000  0; */
  /*      0.0000         0    0.0000   -2.0000   -3.0000   -2.0000  0   -0.0000
   * 0   -0.0000    2.0000    3.0000    2.0000  0; */
  /*     -0.0000   -0.0000         0    2.0000    2.0000    1.0000  0    0.0000
   * 0.0000         0   -2.0000   -2.0000   -1.0000  0; */
  /*           0         0         0   -1.0000         0         0  0         0
   * 0         0    1.0000         0         0  0; */
  /*           0         0         0         0   -1.0000         0  0         0
   * 0         0         0    1.0000         0  0; */
  /*           0         0         0         0         0   -1.0000  0         0
   * 0         0         0         0    1.0000  0; */
  /*           0         0         0         0         0         0  0         0
   * 0         0         0         0         0  0]; */
  /*   Ad_eye1=[ */
  /*      1.0000   -0.0000   -0.0000    1.0000    2.0000    2.0000  ; */
  /*      0.0000    1.0000    0.0000   -2.0000   -3.0000   -2.0000  ; */
  /*     -0.0000   -0.0000    1.0000    2.0000    2.0000    1.0000  ; */
  /*           0    0.0000    0.0000   -1.0000   -2.0000   -2.0000  ; */
  /*     -0.0000         0   -0.0000    2.0000    3.0000    2.0000  ; */
  /*      0.0000    0.0000         0   -2.0000   -2.0000   -1.0000  ; */
  /*           0         0         0    1.0000         0         0  ; */
  /*           0         0         0         0    1.0000         0  ; */
  /*           0         0         0         0         0    1.0000  ; */
  /*           0         0         0         0         0         0  ; */
  /*           0   -0.0000   -0.0000    1.0000    2.0000    2.0000  ; */
  /*      0.0000         0    0.0000   -2.0000   -3.0000   -2.0000  ; */
  /*     -0.0000   -0.0000         0    2.0000    2.0000    1.0000  ;  */
  /*           0         0         0   -1.0000         0         0  ; */
  /*           0         0         0         0   -1.0000         0  ; */
  /*           0         0         0         0         0   -1.0000  ; */
  /*           0         0         0         0         0         0  ]; */
  /*   Ad_eye2 =[ */
  /*          -1.0000    0.0000    0.0000   -1.0000   -2.0000   -2.0000  ; */
  /*          -0.0000   -1.0000   -0.0000    2.0000    3.0000    2.0000  ; */
  /*           0.0000    0.0000   -1.0000   -2.0000   -2.0000   -1.0000  ; */
  /*                0   -0.0000   -0.0000    1.0000    2.0000    2.0000  ; */
  /*           0.0000         0    0.0000   -2.0000   -3.0000   -2.0000  ; */
  /*          -0.0000   -0.0000         0    2.0000    2.0000    1.0000  ; */
  /*                0         0         0   -1.0000         0         0  ; */
  /*                0         0         0         0   -1.0000         0  ; */
  /*                0         0         0         0         0   -1.0000  ; */
  /*                0         0         0         0         0         0  ; */
  /*                0    0.0000    0.0000   -1.0000   -2.0000   -2.0000  ; */
  /*          -0.0000         0   -0.0000    2.0000    3.0000    2.0000  ; */
  /*           0.0000    0.0000         0   -2.0000   -2.0000   -1.0000  ; */
  /*                0         0         0    1.0000         0         0  ; */
  /*                0         0         0         0    1.0000         0  ; */
  /*                0         0         0         0         0    1.0000  ; */
  /*                0         0         0         0         0         0  ]; */
  /*  根据以上分析，P_inv*Ad只有第5，第10列依v不同而变化。 */
  for (i = 0; i < 17; i++) {
    for (div_i_tmp = 0; div_i_tmp < 14; div_i_tmp++) {
      f = 0.0F;
      for (Aeq_tmp = 0; Aeq_tmp < 17; Aeq_tmp++) {
        f += b_B[i + 17 * Aeq_tmp] * Ad[Aeq_tmp + 17 * div_i_tmp];
      }
      Ad_eye[i + 17 * div_i_tmp] = f;
    }
    tmp_ii = 0.0F;
    temp2 = 0.0F;
    for (Aeq_tmp = 0; Aeq_tmp < 17; Aeq_tmp++) {
      f = b_B[i + 17 * Aeq_tmp];
      tmp_ii += f * Ad[Aeq_tmp + 102];
      temp2 += f * Ad[Aeq_tmp + 221];
      /*          temp1=temp1 + P_inv(i,k)*Ad7(k); */
      /*          temp2=temp2 + P_inv(i,k)*-Ad7(k); */
    }
    Ad_eye[i + 102] = tmp_ii;
    Ad_eye[i + 221] = temp2;
    /*      Ak1(i)=temp1; */
    /*      Ak2(i)=temp2; */
  }
  /*  加上松弛变量对应的基 */
  for (div_i_tmp = 0; div_i_tmp < 14; div_i_tmp++) {
    A[17 * div_i_tmp] = Ad_eye[17 * div_i_tmp];
    b_P_tmp = 17 * (div_i_tmp + 14);
    A[b_P_tmp] = 0.0F;
    b_div_i_tmp = 17 * div_i_tmp + 1;
    A[b_div_i_tmp] = Ad_eye[b_div_i_tmp];
    A[b_P_tmp + 1] = 0.0F;
    b_div_i_tmp = 17 * div_i_tmp + 2;
    A[b_div_i_tmp] = Ad_eye[b_div_i_tmp];
    A[b_P_tmp + 2] = 0.0F;
    for (Aeq_tmp = 0; Aeq_tmp < 14; Aeq_tmp++) {
      b_div_i_tmp = (Aeq_tmp + 17 * div_i_tmp) + 3;
      A[b_div_i_tmp] = Ad_eye[b_div_i_tmp];
      A[(Aeq_tmp + b_P_tmp) + 3] = iv1[Aeq_tmp + 14 * div_i_tmp];
    }
  }
  /*  A1=[zeros(3,14); eye(14)]; */
  /*  A=[Ad_eye1 Ak1 Ad_eye2 Ak2 A1] */
  /*  A是Ad_eye的扩充，第7，第14列与P_inv*Ad变化的部分列有关，其他是常数 */
  /*  A(:,7)=Ak1; */
  /*  A(:,14)=Ak2; */
  /*  转C需要特别注意下标的区别 */
  /*  Simplex algorithm */
  /*  Iterate through simplex algorithm main loop */
  for (div_i_tmp = 0; div_i_tmp < 17; div_i_tmp++) {
    c_B[div_i_tmp] = iv2[div_i_tmp];
  }
  /*  (c) mengchaoheng */
  /*  不考虑无解的情形 */
  /*  Last edited 2019-11 */
  /*    min z=c*x   subj. to  A*x (=、 >=、 <=) b */
  /*    x  */
  /*     %% Initialization */
  /*  Iterate through simplex algorithm main loop */
  for (i = 0; i < 28; i++) {
    c[i] = iv3[i];
    x[i] = 0.0F;
  }
  *iters = 0;
  *z = 0.0F;
  /*      [m,n] = size(A); */
  /*      while ~all(c>=0)                      % 3.~isempty(c(c(N)<0)) */
  /*      e = find(c < 0, 1, 'first'); % 进基变量索引    % 4. e =
   * N(find(c(N)<0,1)) */
  do {
    exitg1 = 0;
    flag = false;
    e = 0;
    L = 0;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i < 28)) {
      if (c[i] < -1.0E-6F) {
        /*  <0 */
        flag = true;
        e = (short)(i + 1);
        exitg2 = true;
      } else {
        i++;
      }
    }
    if (flag) {
      /*              a_ie=A(:,e); */
      /*              ip=a_ie>(1/tol); */
      /*              delta=tol*ones(m,1); */
      /*              if ~isempty(ip) */
      /*                  delta(ip)=b(ip)./a_ie(ip); */
      /*              end */
      tmp_ii = 1.0E+6F;
      for (i = 0; i < 17; i++) {
        f = A[i + 17 * (e - 1)];
        if (f > 1.0E-6F) {
          f = b[i] / f;
        } else {
          f = 1.0E+6F;
        }
        if (f < tmp_ii) {
          L = (short)(i + 1);
          tmp_ii = f;
        }
      }
      /*              [~,L]=min(delta);%选择离基 (离基在B数组中的行索引) */
      /*          li = B(L);    % 离基变量索引                */
      /*              if delta(L) >= tol     */
      if (tmp_ii >= 1.0E+6F) {
        exitg1 = 1;
      } else {
        /*  此时一定有一个L */
        /*  (c) mengchaoheng */
        /*  Last edited 2019-11 */
        /*    min z=c*x   subj. to  A*x (=、 >=、 <=) b */
        /*    x  */
        /*     %% Compute the coefficients of the equation for new basic
         * variabLe x_e. */
        /*      [m, n] = size(A); */
        /*  row of Leaving var    L = find(B==li,1); */
        /*  Perform pivot operation, exchanging L-row with e-coLumn variabLe */
        jj = 17 * (e - 1);
        tmp_ii = A[(L + jj) - 1];
        b[L - 1] /= tmp_ii;
        /*  4. */
        for (div_i_tmp = 0; div_i_tmp < 28; div_i_tmp++) {
          b_A[div_i_tmp] = A[(L + 17 * div_i_tmp) - 1] / tmp_ii;
        }
        for (div_i_tmp = 0; div_i_tmp < 28; div_i_tmp++) {
          A[(L + 17 * div_i_tmp) - 1] = b_A[div_i_tmp];
        }
        /*     %% Compute the coefficients of the remaining constraints. */
        /*      i=[1:L-1 L+1:m];     %  i = find(B~=li); */
        /*      if ~isempty(i) */
        /*          b(i) = b(i) - A(i,e)*b(L); */
        /*          A(i,1:n) = A(i,1:n) - A(i,e)*A(L,1:n);	 */
        /*      end */
        f = b[L - 1];
        for (i = 0; i < 17; i++) {
          if (i + 1 != L) {
            tmp_ii = A[i + jj];
            b[i] -= tmp_ii * f;
            for (Aeq_tmp = 0; Aeq_tmp < 28; Aeq_tmp++) {
              b_P_tmp = 17 * Aeq_tmp;
              b_div_i_tmp = i + b_P_tmp;
              A[b_div_i_tmp] -= tmp_ii * A[(L + b_P_tmp) - 1];
            }
          }
        }
        /*     %% Compute the objective function */
        tmp_ii = c[e - 1];
        *z -= tmp_ii * b[L - 1];
        for (Aeq_tmp = 0; Aeq_tmp < 28; Aeq_tmp++) {
          c[Aeq_tmp] -= tmp_ii * A[(L + 17 * Aeq_tmp) - 1];
        }
        /*      c(1:n) = c(1:n) - c_e * A(L,1:n);       */
        /*     %% Compute new sets of basic and nonbasic variabLes. */
        /*  N(find(N==e,1)) = li;  */
        c_B[L - 1] = (signed char)e;
        /*   B(find(B==li,1)) = e; */
        /* 换基，即进行初等行变换 */
        (*iters)++;
      }
    } else {
      for (div_i_tmp = 0; div_i_tmp < 17; div_i_tmp++) {
        x[c_B[div_i_tmp] - 1] = b[div_i_tmp];
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  /*  线性规划单纯形法 */
  /*  [x,z,iters]=Simplex_loop_mch(basis, A, b, c, z); */
  for (i = 0; i < 6; i++) {
    u[i] = x[i] - x[i + 7];
  }
  if (*z > 1.0F) {
    /*  放大了倍数，再还原，若小于1，则表示需要缩小，x已经自然到达边界 */
    for (div_i_tmp = 0; div_i_tmp < 6; div_i_tmp++) {
      u[div_i_tmp] /= *z;
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void dir_alloc_six_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void dir_alloc_six_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for dir_alloc_six.c
 *
 * [EOF]
 */
