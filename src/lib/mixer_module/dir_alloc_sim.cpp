/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: dir_alloc_sim.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 2024-03-11 20:38:05
 */

/* Include Files */
#include "dir_alloc_sim.h"

/* Function Definitions */
/*
 * (c) mengchaoheng
 *  Last edited 2019-11
 *    min z=c*x   subj. to  A*x (=�� >=�� <=) b
 *    x
 *  ԭ����
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
 *  ������
 *    min z=[0 -1]x   subj. to  [B -v]x = 0
 *    x                       [I 0;-I 0]x <= [umax; -umin]
 *    ���� x=[u; a]
 *  ��Ӧ��͹�Ż���p139,��Ϊ
 *    min z=c*x   subj. to  Aeq*x = beq
 *    x                     G*x <= h
 *  �ϲ�
 *    min z=c*x   subj. to  [Aeq; G]*x (=��<=) [beq;h]
 *    x
 *  ��֤x>=0������
 *    min z=[c -c]*X   subj. to  [Aeq -Aeq;G -G]*X (=��<=) [beq;h]
 *     X
 *  ���� X=[x^+; x^-]
 *
 *  B=[-0.5   0       0.5   0;
 *       0  -0.5    0       0.5;
 *      0.25   0.25   0.25   0.25];
 *
 * Arguments    : const float v[3]
 *                const float umin[4]
 *                const float umax[4]
 *                const float B[12]
 *                float u[4]
 *                float *z
 *                float *iters
 * Return Type  : void
 */
void dir_alloc_sim(const float v[3], const float umin[4], const float umax[4],
                   const float B[12], float u[4], float *z, float *iters)
{
  static float A[260];
  static float P[169];
  static float b_B[169];
  static float Ad[130];
  static float Ad_eye[130];
  static const signed char iv2[169] = {
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  static const signed char iv[100] = {
      1, 0,  0, 0,  0, -1, 0, 0,  0, 0,  0,  1, 0,  0, 0,  0, -1, 0, 0,  0,
      0, 0,  1, 0,  0, 0,  0, -1, 0, 0,  0,  0, 0,  1, 0,  0, 0,  0, -1, 0,
      0, 0,  0, 0,  1, 0,  0, 0,  0, -1, -1, 0, 0,  0, 0,  1, 0,  0, 0,  0,
      0, -1, 0, 0,  0, 0,  1, 0,  0, 0,  0,  0, -1, 0, 0,  0, 0,  1, 0,  0,
      0, 0,  0, -1, 0, 0,  0, 0,  1, 0,  0,  0, 0,  0, -1, 0, 0,  0, 0,  1};
  static const signed char iv1[100] = {
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  static const signed char iv4[20] = {0, 0, 0, 0, -1, 0, 0, 0, 0, 1,
                                      0, 0, 0, 0, 0,  0, 0, 0, 0, 0};
  static const signed char iv3[13] = {1,  2,  3,  11, 12, 13, 14,
                                      15, 16, 17, 18, 19, 20};
  float c[20];
  float x[20];
  float Aeq[15];
  float b[13];
  float div_i;
  int Aeq_tmp;
  int L;
  int P_tmp;
  int div_i_tmp;
  int e;
  int i;
  signed char c_B[13];
  for (i = 0; i < 4; i++) {
    Aeq[3 * i] = B[3 * i];
    Aeq_tmp = 3 * i + 1;
    Aeq[Aeq_tmp] = B[Aeq_tmp];
    Aeq_tmp = 3 * i + 2;
    Aeq[Aeq_tmp] = B[Aeq_tmp];
  }
  /* b������Թ滮 */
  Aeq[12] = -v[0];
  b[0] = 0.0F;
  Aeq[13] = -v[1];
  b[1] = 0.0F;
  Aeq[14] = -v[2];
  b[2] = 0.0F;
  b[7] = 20.0F;
  b[3] = umax[0];
  b[8] = -umin[0];
  b[4] = umax[1];
  b[9] = -umin[1];
  b[5] = umax[2];
  b[10] = -umin[2];
  b[6] = umax[3];
  b[11] = -umin[3];
  b[12] = 0.0F;
  /*  �������Թ滮��׼�� */
  /*  Convert free variables to positively constrained variables */
  for (i = 0; i < 5; i++) {
    div_i = Aeq[3 * i];
    Ad[13 * i] = div_i;
    Aeq_tmp = 13 * (i + 5);
    Ad[Aeq_tmp] = -div_i;
    div_i = Aeq[3 * i + 1];
    Ad[13 * i + 1] = div_i;
    Ad[Aeq_tmp + 1] = -div_i;
    div_i = Aeq[3 * i + 2];
    Ad[13 * i + 2] = div_i;
    Ad[Aeq_tmp + 2] = -div_i;
  }
  for (i = 0; i < 10; i++) {
    for (e = 0; e < 10; e++) {
      Ad[(e + 13 * i) + 3] = iv[e + 10 * i];
    }
  }
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
  /*  Adֻ�е�5����10�и���v��ͬ����ͬ�������̶����� */
  /*  [mad,~]= size(Ad); */
  /*  �Ȱ�ǰ������ʽ�Ļ��ҵ��������� */
  for (i = 0; i < 3; i++) {
    P[13 * i] = Ad[13 * i];
    P_tmp = 13 * i + 1;
    P[P_tmp] = Ad[P_tmp];
    P_tmp = 13 * i + 2;
    P[P_tmp] = Ad[P_tmp];
  }
  for (i = 0; i < 10; i++) {
    P_tmp = 13 * (i + 3);
    P[P_tmp] = 0.0F;
    P[P_tmp + 1] = 0.0F;
    P[P_tmp + 2] = 0.0F;
  }
  for (i = 0; i < 3; i++) {
    for (e = 0; e < 10; e++) {
      P_tmp = (e + 13 * i) + 3;
      P[P_tmp] = Ad[P_tmp];
    }
  }
  for (i = 0; i < 10; i++) {
    for (e = 0; e < 10; e++) {
      P[(e + 13 * (i + 3)) + 3] = iv1[e + 10 * i];
    }
  }
  /*  P=[Ad(1:3,1:3) zeros(3,10);Ad(4:mad,1:3) eye(10)]; */
  /*  P=[Ad(1:3,1:3) zeros(3,10);Ad(4:13,1:3) eye(10)]; */
  /*  ���� */
  /*  Ad_eye=P\Ad;% ���� */
  /*  �޹��е�����P_inv�ǳ����� */
  /*  P_inv=[-1     1     2     0     0     0     0     0     0     0     0 0 0;
   */
  /*       0    -2     0     0     0     0     0     0     0     0     0     0
   * 0; */
  /*       1     1     2     0     0     0     0     0     0     0     0     0
   * 0; */
  /*       1    -1    -2     1     0     0     0     0     0     0     0     0
   * 0; */
  /*       0     2     0     0     1     0     0     0     0     0     0     0
   * 0; */
  /*      -1    -1    -2     0     0     1     0     0     0     0     0     0
   * 0; */
  /*       0     0     0     0     0     0     1     0     0     0     0     0
   * 0; */
  /*       0     0     0     0     0     0     0     1     0     0     0     0
   * 0; */
  /*      -1     1     2     0     0     0     0     0     1     0     0     0
   * 0; */
  /*       0    -2     0     0     0     0     0     0     0     1     0     0
   * 0; */
  /*       1     1     2     0     0     0     0     0     0     0     1     0
   * 0; */
  /*       0     0     0     0     0     0     0     0     0     0     0     1
   * 0; */
  /*       0     0     0     0     0     0     0     0     0     0     0     0
   * 1]; */
  /*  对矩阵进行初等行变换求其逆 */
  /*  function Ad_eye=inv_mvh(B_inv,Ad) */
  /*  Ad_eye=B_inv\Ad;% 化简 */
  /*  [row, col] = size(A); */
  /*  B为单位矩阵 */
  for (i = 0; i < 169; i++) {
    b_B[i] = iv2[i];
  }
  for (i = 0; i < 13; i++) {
    /*  依次将对角行的元素归一化 */
    div_i_tmp = i + 13 * i;
    div_i = P[div_i_tmp];
    for (Aeq_tmp = 0; Aeq_tmp < 13; Aeq_tmp++) {
      P_tmp = i + 13 * Aeq_tmp;
      P[P_tmp] /= div_i;
      b_B[P_tmp] /= div_i;
    }
    for (Aeq_tmp = 0; Aeq_tmp < 13; Aeq_tmp++) {
      div_i = -P[Aeq_tmp + 13 * i] / P[div_i_tmp];
      if (i == Aeq_tmp) {
        div_i = 0.0F;
      }
      /*  初等行变换 */
      for (e = 0; e < 13; e++) {
        P_tmp = Aeq_tmp + 13 * e;
        L = i + 13 * e;
        P[P_tmp] += div_i * P[L];
        b_B[P_tmp] += div_i * b_B[L];
      }
    }
  }
  for (i = 0; i < 13; i++) {
    for (e = 0; e < 10; e++) {
      div_i = 0.0F;
      for (Aeq_tmp = 0; Aeq_tmp < 13; Aeq_tmp++) {
        div_i += b_B[i + 13 * Aeq_tmp] * Ad[Aeq_tmp + 13 * e];
      }
      Ad_eye[i + 13 * e] = div_i;
    }
  }
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
  /*  �������Ϸ�����P_inv*Adֻ�е�5����10����v��ͬ���仯�� */
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
  /*  �����ɳڱ�����Ӧ�Ļ� */
  for (i = 0; i < 10; i++) {
    A[13 * i] = Ad_eye[13 * i];
    div_i_tmp = 13 * (i + 10);
    A[div_i_tmp] = 0.0F;
    Aeq_tmp = 13 * i + 1;
    A[Aeq_tmp] = Ad_eye[Aeq_tmp];
    A[div_i_tmp + 1] = 0.0F;
    Aeq_tmp = 13 * i + 2;
    A[Aeq_tmp] = Ad_eye[Aeq_tmp];
    A[div_i_tmp + 2] = 0.0F;
    for (e = 0; e < 10; e++) {
      Aeq_tmp = (e + 13 * i) + 3;
      A[Aeq_tmp] = Ad_eye[Aeq_tmp];
      A[(e + div_i_tmp) + 3] = iv1[e + 10 * i];
    }
  }
  /*  A��Ad_eye�����䣬��5����10����P_inv*Ad�仯�Ĳ������йأ������ǳ��� */
  /*  A=[1     0     0     1     0    -1     0     0    -1     0     0     0 0
   * 0     0     0     0     0     0     0; */
  /*     0     1     0    -1     0     0    -1     0     1     0     0     0 0
   * 0     0     0     0     0     0     0; */
  /*     0     0     1     1     0     0     0    -1    -1     0     0     0 0
   * 0     0     0     0     0     0     0; */
  /*     0     0     0    -1     0     0     0     0     1     0     1     0 0
   * 0     0     0     0     0     0     0; */
  /*     0     0     0     1     0     0     0     0    -1     0     0     1 0
   * 0     0     0     0     0     0     0; */
  /*     0     0     0    -1     0     0     0     0     1     0     0     0 1
   * 0     0     0     0     0     0     0; */
  /*     0     0     0     1     0     0     0     0    -1     0     0     0 0
   * 1     0     0     0     0     0     0; */
  /*     0     0     0     0     0     0     0     0     0     0     0     0 0
   * 0     1     0     0     0     0     0; */
  /*     0     0     0     1     0     0     0     0    -1     0     0     0 0
   * 0     0     1     0     0     0     0; */
  /*     0     0     0    -1     0     0     0     0     1     0     0     0 0
   * 0     0     0     1     0     0     0; */
  /*     0     0     0     1     0     0     0     0    -1     0     0     0 0
   * 0     0     0     0     1     0     0; */
  /*     0     0     0    -1     0     0     0     0     1     0     0     0 0
   * 0     0     0     0     0     1     0; */
  /*     0     0     0     0     0     0     0     0     0     0     0     0 0
   * 0     0     0     0     0     0     1]; */
  /*  for i=1:13 */
  /*      temp1=0; */
  /*      temp2=0; */
  /*      for k=1:13 */
  /*          temp1=temp1 + P_inv(i,k)*Ad5(k); */
  /*          temp2=temp2 + P_inv(i,k)*Ad10(k); */
  /*      end */
  /*      A(i,5)=temp1; */
  /*      A(i,10)=temp2; */
  /*  end */
  /*  תC��Ҫ�ر�ע���±������ */
  /*  Simplex algorithm */
  /*  Iterate through simplex algorithm main loop */
  for (i = 0; i < 13; i++) {
    c_B[i] = iv3[i];
  }
  /*  (c) mengchaoheng */
  /*  不考虑无解的情形 */
  /*  Last edited 2019-11 */
  /*    min z=c*x   subj. to  A*x (=、 >=、 <=) b */
  /*    x  */
  /*     %% Initialization */
  /*  Iterate through simplex algorithm main loop */
  for (i = 0; i < 20; i++) {
    c[i] = iv4[i];
    x[i] = 0.0F;
  }
  *iters = 0.0F;
  *z = 0.0F;
  /*      [m,n] = size(A); */
  /*      while ~all(c>=0)                      % 3.~isempty(c(c(N)<0)) */
  /*      e = find(c < 0, 1, 'first'); % 进基变量索引    % 4. e =
   * N(find(c(N)<0,1)) */
  long exitg1;
  bool exitg2;
  bool flag;
  do {
    exitg1 = 0L;
    flag = false;
    e = -1;
    L = -1;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i < 20)) {
      if (c[i] < -1.0E-6F) {
        /*  <0 */
        flag = true;
        e = i;
        exitg2 = true;
      } else {
        i++;
      }
    }
    if (flag) {
      float MIN;
      /*              a_ie=A(:,e); */
      /*              ip=a_ie>(1/tol); */
      /*              delta=tol*ones(m,1); */
      /*              if ~isempty(ip) */
      /*                  delta(ip)=b(ip)./a_ie(ip); */
      /*              end */
      MIN = 1.0E+6F;
      for (i = 0; i < 13; i++) {
        div_i = A[i + 13 * e];
        if (div_i > 1.0E-6F) {
          div_i = b[i] / div_i;
        } else {
          div_i = 1.0E+6F;
        }
        if (div_i < MIN) {
          L = i;
          MIN = div_i;
        }
      }
      /*              [~,L]=min(delta);%选择离基 (离基在B数组中的行索引) */
      /*          li = B(L);    % 离基变量索引                */
      /*              if delta(L) >= tol     */
      if (MIN >= 1.0E+6F) {
        exitg1 = 1L;
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
        div_i = A[L + 13 * e];
        b[L] /= div_i;
        /*  4. */
        for (i = 0; i < 20; i++) {
          div_i_tmp = L + 13 * i;
          A[div_i_tmp] /= div_i;
        }
        /*     %% Compute the coefficients of the remaining constraints. */
        /*      i=[1:L-1 L+1:m];     %  i = find(B~=li); */
        /*      if ~isempty(i) */
        /*          b(i) = b(i) - A(i,e)*b(L); */
        /*          A(i,1:n) = A(i,1:n) - A(i,e)*A(L,1:n);	 */
        /*      end */
        MIN = b[L];
        for (i = 0; i < 13; i++) {
          if (i != L) {
            div_i = A[i + 13 * e];
            b[i] -= div_i * MIN;
            for (Aeq_tmp = 0; Aeq_tmp < 20; Aeq_tmp++) {
              div_i_tmp = i + 13 * Aeq_tmp;
              A[div_i_tmp] -= div_i * A[L + 13 * Aeq_tmp];
            }
          }
        }
        /*     %% Compute the objective function */
        div_i = c[e];
        *z -= c[e] * b[L];
        for (Aeq_tmp = 0; Aeq_tmp < 20; Aeq_tmp++) {
          c[Aeq_tmp] -= div_i * A[L + 13 * Aeq_tmp];
        }
        /*      c(1:n) = c(1:n) - c_e * A(L,1:n);       */
        /*     %% Compute new sets of basic and nonbasic variabLes. */
        /*  N(find(N==e,1)) = li;  */
        c_B[L] = (signed char)(e + 1);
        /*   B(find(B==li,1)) = e; */
        /* 换基，即进行初等行变换 */
        (*iters)++;
      }
    } else {
      for (i = 0; i < 13; i++) {
        x[c_B[i] - 1] = b[i];
      }
      exitg1 = 1L;
    }
  } while (exitg1 == 0L);
  /*  ���Թ滮�����η� */
  /*  [x,z,iters]=Simplex_loop_mch(basis, A, b, c, z); */
  u[0] = x[0] - x[5];
  u[1] = x[1] - x[6];
  u[2] = x[2] - x[7];
  u[3] = x[3] - x[8];
  if (*z > 1.0F) {
    /*  �Ŵ��˱������ٻ�ԭ����С��1�����ʾ��Ҫ��С��x�Ѿ���Ȼ����߽� */
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
void dir_alloc_sim_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void dir_alloc_sim_terminate(void)
{
}

/*
 * File trailer for dir_alloc_sim.c
 *
 * [EOF]
 */
