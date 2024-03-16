/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: allocator_dir_simplex_4.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2024-03-16 15:46:10
 */

/* Include Files */
#include "allocator_dir_simplex_4.h"

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
 *  B=[-0.5   0       0.5   0;
 *       0  -0.5    0       0.5;
 *      0.25   0.25   0.25   0.25];
 *  B=[-0.5   0       0.5   0;
 *       0  -0.5    0       0.5;
 *      0.25   0.25   0.25   0.25];
 *  Aeq=[B -v];
 *  beq=zeros(3,1);
 *  G=[eye(5);-eye(5)];
 *  h=[umax; 20; -umin; 0];
 *  求解线性规划
 *  b=[beq;h];
 *
 * Arguments    : const float v[3]
 *                const float umin[4]
 *                const float umax[4]
 *                float u[4]
 *                float *z
 *                unsigned long *iters
 * Return Type  : void
 */
void allocator_dir_simplex_4(const float v[3], const float umin[4],
                             const float umax[4], float u[4], float *z,
                             unsigned long *iters)
{
  static float A[260];
  static const signed char iv2[260] = {
      1,  0, 0,  0, 0,  0,  0,  0, 0,  0, 0,  0, 0,  0,  1,  0, 0, 0, 0, 0,
      0,  0, 0,  0, 0,  0,  0,  0, 1,  0, 0,  0, 0,  0,  0,  0, 0, 0, 0, 1,
      -1, 1, -1, 1, -1, 1,  0,  1, -1, 1, -1, 0, 0,  0,  0,  0, 0, 0, 0, 0,
      0,  0, 0,  0, 0,  -1, 0,  0, 0,  0, 0,  0, 0,  0,  0,  0, 0, 0, 0, -1,
      0,  0, 0,  0, 0,  0,  0,  0, 0,  0, 0,  0, 0,  -1, 0,  0, 0, 0, 0, 0,
      0,  0, 0,  0, -1, 1,  -1, 1, -1, 1, -1, 0, -1, 1,  -1, 1, 0, 0, 0, 0,
      0,  0, 0,  0, 0,  0,  0,  0, 0,  0, 0,  0, 0,  1,  0,  0, 0, 0, 0, 0,
      0,  0, 0,  0, 0,  0,  0,  1, 0,  0, 0,  0, 0,  0,  0,  0, 0, 0, 0, 0,
      0,  1, 0,  0, 0,  0,  0,  0, 0,  0, 0,  0, 0,  0,  0,  1, 0, 0, 0, 0,
      0,  0, 0,  0, 0,  0,  0,  0, 0,  1, 0,  0, 0,  0,  0,  0, 0, 0, 0, 0,
      0,  0, 0,  1, 0,  0,  0,  0, 0,  0, 0,  0, 0,  0,  0,  0, 0, 1, 0, 0,
      0,  0, 0,  0, 0,  0,  0,  0, 0,  0, 0,  1, 0,  0,  0,  0, 0, 0, 0, 0,
      0,  0, 0,  0, 0,  1,  0,  0, 0,  0, 0,  0, 0,  0,  0,  0, 0, 0, 0, 1};
  static const signed char P_inv[169] = {
      -1, 0, 1, 1, 0, -1, 0, 0,  -1, 0,  1, 0, 0, 1, -2, 1, -1, 2, -1, 0, 0, 1,
      -2, 1, 0, 0, 2, 0,  2, -2, 0,  -2, 0, 0, 2, 0, 2,  0, 0,  0, 0,  0, 1, 0,
      0,  0, 0, 0, 0, 0,  0, 0,  0,  0,  0, 0, 1, 0, 0,  0, 0,  0, 0,  0, 0, 0,
      0,  0, 0, 0, 1, 0,  0, 0,  0,  0,  0, 0, 0, 0, 0,  0, 0,  0, 1,  0, 0, 0,
      0,  0, 0, 0, 0, 0,  0, 0,  0,  0,  1, 0, 0, 0, 0,  0, 0,  0, 0,  0, 0, 0,
      0,  0, 1, 0, 0, 0,  0, 0,  0,  0,  0, 0, 0, 0, 0,  0, 1,  0, 0,  0, 0, 0,
      0,  0, 0, 0, 0, 0,  0, 0,  1,  0,  0, 0, 0, 0, 0,  0, 0,  0, 0,  0, 0, 0,
      1,  0, 0, 0, 0, 0,  0, 0,  0,  0,  0, 0, 0, 0, 1};
  static const signed char iv3[20] = {0, 0, 0, 0, -1, 0, 0, 0, 0, 1,
                                      0, 0, 0, 0, 0,  0, 0, 0, 0, 0};
  static const signed char iv[13] = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1};
  static const signed char iv1[13] = {0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1};
  static const unsigned char uv[13] = {1U,  2U,  3U,  11U, 12U, 13U, 14U,
                                       15U, 16U, 17U, 18U, 19U, 20U};
  float c[20];
  float x[20];
  float Ad10[13];
  float Ad5[13];
  float b[13];
  float temp1;
  float temp2;
  int i;
  int k;
  unsigned char B[13];
  b[0] = 0.0F;
  b[1] = 0.0F;
  b[2] = 0.0F;
  b[7] = 10000.0F;
  b[3] = umax[0];
  b[8] = -umin[0];
  b[4] = umax[1];
  b[9] = -umin[1];
  b[5] = umax[2];
  b[10] = -umin[2];
  b[6] = umax[3];
  b[11] = -umin[3];
  b[12] = 0.0F;
  /*  构造线性规划标准型 */
  /*  Convert free variables to positively constrained variables */
  /*  Ad=[Aeq -Aeq; G -G]; */
  /*  Ad=[B -v -B v; eye(5) -eye(5);-eye(5) eye(5)]; */
  /*  Ad=[-0.5000         0    0.5000         0         0    0.5000         0
   * -0.5000         0         0; */
  /*            0   -0.5000         0    0.5000         0         0    0.5000 0
   * -0.5000         0; */
  /*       0.2500    0.2500    0.2500    0.2500         0   -0.2500   -0.2500
   * -0.2500   -0.2500   	    0; */
  /*       1.0000         0         0         0         0   -1.0000         0 0
   * 0         0; */
  /*            0    1.0000         0         0         0         0   -1.0000 0
   * 0         0; */
  /*            0         0    1.0000         0         0         0         0
   * -1.0000         0         0; */
  /*            0         0         0    1.0000         0         0         0 0
   * -1.0000         0; */
  /*            0         0         0         0    1.0000         0         0 0
   * 0   -1.0000; */
  /*      -1.0000         0         0         0         0    1.0000         0 0
   * 0         0; */
  /*            0   -1.0000         0         0         0         0    1.0000 0
   * 0         0; */
  /*            0         0   -1.0000         0         0         0 0    1.0000
   * 0         0; */
  /*            0         0         0   -1.0000         0         0         0 0
   * 1.0000         0; */
  /*            0         0         0         0   -1.0000         0         0 0
   * 0    1.0000]; */
  /*  Ad只有第5，第10列根据v不同而不同，其他固定不变 */
  for (i = 0; i < 13; i++) {
    Ad5[i] = iv[i];
    Ad10[i] = iv1[i];
  }
  Ad5[0] = -v[0];
  Ad10[0] = v[0];
  Ad5[1] = -v[1];
  Ad10[1] = v[1];
  Ad5[2] = -v[2];
  Ad10[2] = v[2];
  /*  [mad,~]= size(Ad); */
  /*  先把前三个等式的基找到，并化简 */
  /*  B_inv=[Ad(1:3,1:3) zeros(3,mad-3);Ad(4:mad,1:3) eye(mad-3)]; */
  /*  B_inv=[Ad(1:3,1:3) zeros(3,10);Ad(4:mad,1:3) eye(10)]; */
  /*  P=[Ad(1:3,1:3) zeros(3,10);Ad(4:13,1:3) eye(10)]; */
  /*  求逆 */
  /*  Ad_eye=P\Ad;% 化简 */
  /*  无关列的逆阵P_inv是常矩阵 */
  /*  P_inv=inv_mch(P); */
  /*  Ad_eye=P_inv*Ad; */
  /*  Ad_eye=[1     0     0     1     0    -1     0     0    -1     0; */
  /*          0     1     0    -1     0     0    -1     0     1     0; */
  /*          0     0     1     1     0     0     0    -1    -1     0; */
  /*          0     0     0    -1     0     0     0     0     1     0; */
  /*          0     0     0     1     0     0     0     0    -1     0; */
  /*          0     0     0    -1     0     0     0     0     1     0; */
  /*          0     0     0     1     0     0     0     0    -1     0; */
  /*          0     0     0     0     0     0     0     0     0     0; */
  /*          0     0     0     1     0     0     0     0    -1     0; */
  /*          0     0     0    -1     0     0     0     0     1     0; */
  /*          0     0     0     1     0     0     0     0    -1     0; */
  /*          0     0     0    -1     0     0     0     0     1     0; */
  /*          0     0     0     0     0     0     0     0     0     0]; */
  /*  根据以上分析，P_inv*Ad只有第5，第10列依v不同而变化。 */
  /*  for i=1:13 */
  /*      temp1=0; */
  /*      temp2=0; */
  /*      for k=1:13 */
  /*          temp1=temp1 + P_inv(i,k)*Ad(k,5); */
  /*          temp2=temp2 + P_inv(i,k)*Ad(k,10); */
  /*      end */
  /*      Ad_eye(i,5)=temp1; */
  /*      Ad_eye(i,10)=temp2; */
  /*  end */
  /*  加上松弛变量对应的基 */
  /*  A=[Ad_eye(1:3,1:10) zeros(3,10); Ad_eye(4:13,1:10) eye(10)]; */
  /*  A是Ad_eye的扩充，第5，第10列与P_inv*Ad变化的部分列有关，其他是常数 */
  for (i = 0; i < 260; i++) {
    A[i] = iv2[i];
  }
  /*  转C需要特别注意下标的区别 */
  /*  Simplex algorithm */
  /*  Iterate through simplex algorithm main loop */
  for (i = 0; i < 13; i++) {
    temp1 = 0.0F;
    temp2 = 0.0F;
    for (k = 0; k < 13; k++) {
      signed char b_i;
      b_i = P_inv[i + 13 * k];
      temp1 += (float)b_i * Ad5[k];
      temp2 += (float)b_i * Ad10[k];
    }
    A[i + 52] = temp1;
    A[i + 117] = temp2;
    B[i] = uv[i];
  }
  /*  (c) mengchaoheng */
  /*  不考虑无解的情形 */
  /*  Last edited 2019-11 */
  /*    min z=c*x   subj. to  A*x (=、 >=、 <=) b */
  /*    x  */
  /*     %% Initialization */
  /*  Iterate through simplex algorithm main loop */
  for (i = 0; i < 20; i++) {
    c[i] = iv3[i];
    x[i] = 0.0F;
  }
  *iters = 0UL;
  *z = 0.0F;
  /*      [m,n] = size(A); */
  /*      while ~all(c>=0)                      % 3.~isempty(c(c(N)<0)) */
  /*      e = find(c < 0, 1, 'first'); % 进基变量索引    % 4. e =
   * N(find(c(N)<0,1)) */
  long exitg1;
  unsigned char L;
  unsigned char e;
  bool exitg2;
  bool flag;
  do {
    exitg1 = 0L;
    flag = false;
    e = 0U;
    L = 0U;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i < 20)) {
      if (c[i] < -1.0E-6F) {
        /*  <0 */
        flag = true;
        e = (unsigned char)(i + 1);
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
      temp1 = 1.0E+6F;
      for (i = 0; i < 13; i++) {
        temp2 = A[i + 13 * (e - 1)];
        if (temp2 > 1.0E-6F) {
          temp2 = b[i] / temp2;
        } else {
          temp2 = 1.0E+6F;
        }
        if (temp2 < temp1) {
          L = (unsigned char)(i + 1);
          temp1 = temp2;
        }
      }
      /*              [~,L]=min(delta);%选择离基 (离基在B数组中的行索引) */
      /*          li = B(L);    % 离基变量索引                */
      /*              if delta(L) >= tol     */
      if (temp1 >= 1.0E+6F) {
        exitg1 = 1L;
      } else {
        float b_A[20];
        unsigned long qY;
        int b_tmp_tmp;
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
        b_tmp_tmp = 13 * (e - 1);
        temp1 = A[(L + b_tmp_tmp) - 1];
        b[L - 1] /= temp1;
        /*  4. */
        for (i = 0; i < 20; i++) {
          b_A[i] = A[(L + 13 * i) - 1] / temp1;
        }
        for (i = 0; i < 20; i++) {
          A[(L + 13 * i) - 1] = b_A[i];
        }
        /*     %% Compute the coefficients of the remaining constraints. */
        /*      i=[1:L-1 L+1:m];     %  i = find(B~=li); */
        /*      if ~isempty(i) */
        /*          b(i) = b(i) - A(i,e)*b(L); */
        /*          A(i,1:n) = A(i,1:n) - A(i,e)*A(L,1:n);	 */
        /*      end */
        temp2 = b[L - 1];
        for (i = 0; i < 13; i++) {
          if (i + 1 != L) {
            temp1 = A[i + b_tmp_tmp];
            b[i] -= temp1 * temp2;
            for (k = 0; k < 20; k++) {
              int A_tmp;
              A_tmp = i + 13 * k;
              A[A_tmp] -= temp1 * A[(L + 13 * k) - 1];
            }
          }
        }
        /*     %% Compute the objective function */
        temp1 = c[e - 1];
        *z -= temp1 * b[L - 1];
        for (k = 0; k < 20; k++) {
          c[k] -= temp1 * A[(L + 13 * k) - 1];
        }
        /*      c(1:n) = c(1:n) - c_e * A(L,1:n);       */
        /*     %% Compute new sets of basic and nonbasic variabLes. */
        /*  N(find(N==e,1)) = li;  */
        B[L - 1] = e;
        /*   B(find(B==li,1)) = e; */
        /* 换基，即进行初等行变换 */
        qY = *iters + 1UL;
        if (*iters + 1UL < *iters) {
          qY = MAX_uint32_T;
        }
        *iters = qY;
      }
    } else {
      for (i = 0; i < 13; i++) {
        x[B[i] - 1] = b[i];
      }
      exitg1 = 1L;
    }
  } while (exitg1 == 0L);
  /*  线性规划单纯形法 */
  u[0] = x[0] - x[5];
  u[1] = x[1] - x[6];
  u[2] = x[2] - x[7];
  u[3] = x[3] - x[8];
  if (*z > 1.0F) {
    /*  放大了倍数，再还原，若小于1，则表示需要缩小，x已经自然到达边界 */
    u[0] /= *z;
    u[1] /= *z;
    u[2] /= *z;
    u[3] /= *z;
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void allocator_dir_simplex_4_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void allocator_dir_simplex_4_terminate(void)
{
}

/*
 * File trailer for allocator_dir_simplex_4.c
 *
 * [EOF]
 */
