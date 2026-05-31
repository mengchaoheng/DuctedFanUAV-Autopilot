/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */


#include <matrix/math.hpp>
// #include <iostream>
#include <stdlib.h>
// #include <limits>
#include <cmath>
#include <cfloat>
using namespace matrix;
#if !defined(FLT_MAX)
#define FLT_MAX     __FLT_MAX__
#endif

struct SimplexTieBreakOptions {
    // Setting guide:
    //   1. Daily MCU float build: keep defaults.
    //   2. Target uses double throughout the simplex: use fromMachineEps(DBL_EPSILON).
    //   3. MATLAB/host simulation of a low-precision target: set machine_eps
    //      to the target arithmetic epsilon, then keep the derived fields.
    //   4. Only tune tie_abs_tol/tie_rel_tol if reduced-cost ties still chatter.
    //      These affect which equally optimal actuator vertex is chosen.
    //   5. Avoid scaling leaving_tie/bound/feasibility tolerances by command
    //      size; they protect pivot feasibility and should track arithmetic eps.

    // Main hardware knob.  Use FLT_EPSILON for normal MCU float builds.
    // Use DBL_EPSILON only if the complete solver arithmetic is really double.
    float machine_eps;

    // Entering variable tie: reduced-cost candidates within
    // tie_abs_tol + tie_rel_tol * max(1, max(abs(rdt))) choose the smallest
    // variable index.  These are control-problem scale tolerances.
    float tie_rel_tol;
    float tie_abs_tol;

    // Leaving ratio and feasibility protection.  These should follow machine
    // precision, not command magnitude.  The defaults mirror
    // PCA/simplxuprevsol_tiebreak.m.
    float leaving_tie_abs_tol;
    float leaving_tie_rel_tol;
    float zero_step_tol;
    float bound_abs_tol;
    float bound_rel_tol;
    float feasibility_abs_tol;
    float feasibility_rel_tol;
    float pivot_zero_tol;

    SimplexTieBreakOptions()
    {
        setFromMachineEps(FLT_EPSILON);
        tie_rel_tol = 1.0e-5f;
        tie_abs_tol = 1.0e-6f;
    }

    void setFromMachineEps(float eps)
    {
        if (!(eps > 0.0f) || !std::isfinite(eps)) {
            eps = FLT_EPSILON;
        }

        machine_eps = eps;
        leaving_tie_abs_tol = 16.0f * eps;
        leaving_tie_rel_tol = 16.0f * eps;
        zero_step_tol = 16.0f * eps;
        bound_abs_tol = 16.0f * eps;
        bound_rel_tol = 16.0f * eps;
        feasibility_abs_tol = 128.0f * eps;
        feasibility_rel_tol = 128.0f * eps;
        pivot_zero_tol = matrix::typeFunction::max(1.0e-10f, 8.0f * eps);
    }

    static SimplexTieBreakOptions fromMachineEps(float eps)
    {
        SimplexTieBreakOptions opts;
        opts.setFromMachineEps(eps);
        opts.tie_rel_tol = 1.0e-5f;
        opts.tie_abs_tol = 1.0e-6f;
        return opts;
    }
};

// Add the min_user function definition here
template<typename Type, size_t M, size_t N>
inline void min_user(const Matrix<Type, M, N> &x, Type &x_min, size_t &x_index)
{
    const Matrix<Type, M, N> &self = x;
    x_min = self(0, 0);
    x_index = 0;

    for (size_t i = 0; i < M; ++i) {
        for (size_t j = 0; j < N; ++j) {
            if (self(i, j) < x_min) {
                x_min = self(i, j);
                x_index = i; // Assuming that you want the index along the row
            }
        }
    }
}

inline float lp_tie_tolerance_from_scale(float scale, float tie_rel_tol, float tie_abs_tol)
{
    if (scale < 1.0f) {
        scale = 1.0f;
    }
    return tie_abs_tol + tie_rel_tol * scale;
}

inline bool lp_is_finite(float value)
{
    return value < INFINITY && value > -INFINITY;
}

template<typename Type, size_t M, size_t N>
inline float lp_tie_tolerance(const Matrix<Type, M, N> &x, float tie_rel_tol, float tie_abs_tol)
{
    float scale = 1.0f;
    for (size_t i = 0; i < M; ++i) {
        for (size_t j = 0; j < N; ++j) {
            const float value = fabs(static_cast<float>(x(i, j)));
            if (lp_is_finite(value) && value > scale) {
                scale = value;
            }
        }
    }
    return lp_tie_tolerance_from_scale(scale, tie_rel_tol, tie_abs_tol);
}

inline float lp_compare_tolerance(float a, float b, float tie_rel_tol, float tie_abs_tol)
{
    float scale = fabs(a);
    const float scale_b = fabs(b);
    if (scale_b > scale) {
        scale = scale_b;
    }
    return lp_tie_tolerance_from_scale(scale, tie_rel_tol, tie_abs_tol);
}

template<size_t K>
inline void select_entering_tiebreak(const Matrix<float, 1, K> &rdt, const int inD[K],
                                     float &minr, size_t &qind,
                                     const SimplexTieBreakOptions &opts)
{
    minr = rdt(0, 0);
    qind = 0;
    for (size_t i = 1; i < K; ++i) {
        if (rdt(0, i) < minr) {
            minr = rdt(0, i);
            qind = i;
        }
    }

    const float raw_min = minr;
    const float tol = lp_tie_tolerance(rdt, opts.tie_rel_tol, opts.tie_abs_tol);
    int best_variable = inD[qind];
    for (size_t i = 0; i < K; ++i) {
        if (fabs(rdt(0, i) - raw_min) <= tol && inD[i] < best_variable) {
            best_variable = inD[i];
            qind = i;
        }
    }
    minr = rdt(0, qind);
}

template<size_t K>
inline bool select_bland_negative_tiebreak(const Matrix<float, 1, K> &rdt, const int inD[K],
                                           size_t &qind)
{
    bool found = false;
    int best_variable = 0;
    for (size_t i = 0; i < K; ++i) {
        if (rdt(0, i) < 0.0f && (!found || inD[i] < best_variable)) {
            found = true;
            best_variable = inD[i];
            qind = i;
        }
    }
    return found;
}

template<size_t M>
inline void select_leaving_tiebreak(const Vector<float, M> &rat, const int inB[M],
                                    float &minrat, size_t &p, size_t &raw_p,
                                    const SimplexTieBreakOptions &opts)
{
    minrat = rat(0);
    p = 0;
    for (size_t i = 1; i < M; ++i) {
        if (rat(i) < minrat) {
            minrat = rat(i);
            p = i;
        }
    }
    raw_p = p;

    int best_variable = inB[p];
    if (lp_is_finite(minrat)) {
        const float tol = lp_tie_tolerance_from_scale(fabs(minrat),
                                                      opts.leaving_tie_rel_tol,
                                                      opts.leaving_tie_abs_tol);
        for (size_t i = 0; i < M; ++i) {
            if (fabs(rat(i) - minrat) <= tol && inB[i] < best_variable) {
                best_variable = inB[i];
                p = i;
            }
        }
    } else {
        for (size_t i = 0; i < M; ++i) {
            if (rat(i) == minrat && inB[i] < best_variable) {
                best_variable = inB[i];
                p = i;
            }
        }
    }
    minrat = rat(p);
}

inline float lp_bound_compare_tolerance(float a, float b, const SimplexTieBreakOptions &opts)
{
    return lp_compare_tolerance(a, b, opts.bound_rel_tol, opts.bound_abs_tol);
}

template<size_t M>
inline bool lp_violates_box(const Vector<float, M> &y, const float h[M],
                            const SimplexTieBreakOptions &opts)
{
    float violation = 0.0f;
    float scale = 1.0f;
    for (size_t i = 0; i < M; ++i) {
        if (-y(i) > violation) {
            violation = -y(i);
        }
        if (y(i) - h[i] > violation) {
            violation = y(i) - h[i];
        }
        const float h_abs = fabs(h[i]);
        if (h_abs > scale) {
            scale = h_abs;
        }
    }

    const float tol = opts.feasibility_abs_tol + opts.feasibility_rel_tol * scale;
    return violation > tol;
}

template<typename Type, size_t M>
inline bool solveSquareLU(Matrix<Type, M, M> A, Vector<Type, M> b, Vector<Type, M> &x)
{
    // Dense square solve using LU with partial row pivoting.  This is used
    // only for A(:,inB)\b style basis solves; simplex pivot selection is
    // handled separately by the tie-break helpers above.
    int piv[M];
    for (size_t i = 0; i < M; ++i) {
        piv[i] = static_cast<int>(i);
    }

    for (size_t k = 0; k + 1 < M; ++k) {
        size_t pivot_row = k;
        Type pivot_abs = fabs(A(k, k));
        for (size_t i = k + 1; i < M; ++i) {
            const Type candidate = fabs(A(i, k));
            if (candidate > pivot_abs) {
                pivot_abs = candidate;
                pivot_row = i;
            }
        }

        if (pivot_abs <= std::numeric_limits<Type>::epsilon() || !lp_is_finite(static_cast<float>(pivot_abs))) {
            x.setZero();
            return false;
        }

        if (pivot_row != k) {
            for (size_t j = 0; j < M; ++j) {
                const Type tmp = A(k, j);
                A(k, j) = A(pivot_row, j);
                A(pivot_row, j) = tmp;
            }
            const int tmp_piv = piv[k];
            piv[k] = piv[pivot_row];
            piv[pivot_row] = tmp_piv;
        }

        for (size_t i = k + 1; i < M; ++i) {
            A(i, k) /= A(k, k);
            for (size_t j = k + 1; j < M; ++j) {
                A(i, j) -= A(i, k) * A(k, j);
            }
        }
    }

    if (fabs(A(M - 1, M - 1)) <= std::numeric_limits<Type>::epsilon()
        || !lp_is_finite(static_cast<float>(A(M - 1, M - 1)))) {
        x.setZero();
        return false;
    }

    Vector<Type, M> y;
    for (size_t i = 0; i < M; ++i) {
        y(i) = b(piv[i]);
    }

    for (size_t k = 0; k < M; ++k) {
        for (size_t i = k + 1; i < M; ++i) {
            y(i) -= A(i, k) * y(k);
        }
    }

    for (int k = static_cast<int>(M) - 1; k >= 0; --k) {
        Type value = y(k);
        for (size_t j = static_cast<size_t>(k) + 1; j < M; ++j) {
            value -= A(k, j) * x(j);
        }
        x(k) = value / A(k, k);
    }

    return true;
}
//rho = ydt'*Bt*u/(ydt'*ydt)
template<int ControlSize, int EffectorSize>
inline float calculateRho(float ydt[ControlSize], float u[EffectorSize], float Bt[ControlSize][EffectorSize], float tol) {
    float numerator = 0.0f;
    float denominator = 0.0f;
    float ydt_T_Bt[EffectorSize];
    for (int j = 0; j < EffectorSize; ++j) {
        ydt_T_Bt[j] = 0;
        for (int k = 0; k < ControlSize; ++k) {
            ydt_T_Bt[j] += ydt[k] * Bt[k][j];
        }
    }
    for (int k = 0; k < EffectorSize; ++k) {
        numerator += ydt_T_Bt[k] * u[k];
    }
    // Calculate the 2-norm of ydt
    // std::cout << "ydt: [";
    //     for (size_t i = 0; i < SIZE_ydt; ++i) {
    //         std::cout << ydt[i];
    //         if (i < SIZE_ydt - 1) {
    //             std::cout << ", ";
    //         }
    //     }
    //     std::cout << "]" << std::endl;
    for (int i = 0; i < ControlSize; ++i) {
        denominator += ydt[i] * ydt[i];
        // std::cout <<"denominator"<< denominator<< std::endl;
    }
    // Avoid division by zero  // The norm of ydt will not be very small
    // std::cout <<"denominator"<< denominator<< std::endl;
    // std::cout <<"fabs(denominator)"<< fabs(denominator)<< std::endl;
    // std::cout <<"tol"<< tol<< std::endl;
    float relativeEpsilon = tol * fabs(numerator); // Dynamic threshold
    // // or
    // const double ABSOLUTE_EPSILON = 1e-10;
    // const double RELATIVE_EPSILON = 1e-10;

    // double safeDivide(double numerator, double denominator) {
    //     if (abs(denominator) < ABSOLUTE_EPSILON) {
    //         if (abs(denominator) < RELATIVE_EPSILON * abs(numerator)) {
    //             throw invalid_argument("Denominator is too close to zero compared to the numerator.");
    //         }
    //     }
    //     return numerator / denominator;
    // }

    if (fabs(denominator) < relativeEpsilon ) {
        // std::cerr << "Error: Division by zero." << std::endl;
        return 1.0f;
    }
    // Calculate rho
    return numerator / denominator;
}

// Function that computes the difference between two sets of positive integers
inline void setdiff(int setA[], int sizeA, int setB[], int sizeB, int result[]) {
    int sizeResult = 0;
    for (int i = 0; i < sizeA; ++i) {
        bool foundInB = false;
        // Check if the current element of setA is in setB
        for (int j = 0; j < sizeB; ++j) {
            if (setA[i] == setB[j]) {
                foundInB = true;
                break;
            }
        }
        // If the current element is not in setB, add it to the result.
        if (!foundInB) {
            result[sizeResult++] = setA[i];
        }
    }
}

template<int ControlSize>
inline void build_dp_scaled_row_order(int iy, int row_order[ControlSize])
{
    if (iy < 0 || iy >= ControlSize) {
        iy = 0;
    }

    row_order[0] = iy;
    int dst = 1;
    for (int row = 0; row < ControlSize; ++row) {
        if (row != iy) {
            row_order[dst++] = row;
        }
    }

    // Match MATLAB DPscaled_LPCA.m:
    //   ydt(2:3) = ydt([3 2]);
    //   Bt([2 3], :) = Bt([3 2], :);
    if (ControlSize > 2) {
        const int tmp = row_order[1];
        row_order[1] = row_order[2];
        row_order[2] = tmp;
    }
}

// Define the structure of the linear programming problem
template<int M, int N>
struct LinearProgrammingProblem {
    int m=M;
    int n=N;
    int inB[M];
    int inD[N-M]; //
    int itlim;
    float A[M][N];
    float b[N];
    float c[N];
    float h[N];
    bool e[N];
    float tol=10*FLT_EPSILON; // important value, if control surface saturation, Use a larger value
    // Runtime simplex parameters.  The solver always uses the tiebreak path;
    // change simplex_opts only to match target precision or diagnose numerics.
    SimplexTieBreakOptions simplex_opts;
    LinearProgrammingProblem() : m(M), n(N), itlim(0) {
        // Initialize array member variables to 0
        for (int i = 0; i < M; ++i) {
            inB[i] = 0;
        }
        for (int i = 0; i < N-M; ++i) {
            inD[i] = 0;
        }
        for (int i = 0; i < M; ++i) {
            for (int j = 0; j < N; ++j) {
                A[i][j] = 0.0f;
            }
        }
        for (int i = 0; i < N; ++i) {
            b[i] = 0.0f;
            c[i] = 0.0f;
            h[i] = 0.0f;
            e[i] = false;
        }
    }
    // Assignment operator
    LinearProgrammingProblem& operator=(const LinearProgrammingProblem& other) {
        if (this != &other) {
            m = other.m;
            n = other.n;
            itlim = other.itlim;
            tol = other.tol;
            simplex_opts = other.simplex_opts;
            // Copy array member variables
            for (int i = 0; i < M; ++i) {
                inB[i] = other.inB[i];
            }
            for (int i = 0; i < N - M; ++i) {
                inD[i] = other.inD[i];
            }
            for (int i = 0; i < M; ++i) {
                for (int j = 0; j < N; ++j) {
                    A[i][j] = other.A[i][j];
                }
            }
            for (int i = 0; i < N; ++i) {
                b[i] = other.b[i];
                c[i] = other.c[i];
                h[i] = other.h[i];
                e[i] = other.e[i];
            }
        }
        return *this;
    }
    // Copy constructor
    LinearProgrammingProblem(const LinearProgrammingProblem<M, N>& other) {
        m = other.m;
        n = other.n;
        itlim = other.itlim;
        tol = other.tol;
        simplex_opts = other.simplex_opts;
        // Copy array member variables
        for (int i = 0; i < M; ++i) {
            inB[i] = other.inB[i];
        }
        for (int i = 0; i < N - M; ++i) {
            inD[i] = other.inD[i];
        }
        for (int i = 0; i < M; ++i) {
            for (int j = 0; j < N; ++j) {
                A[i][j] = other.A[i][j];
            }
        }
        for (int i = 0; i < N; ++i) {
            b[i] = other.b[i];
            c[i] = other.c[i];
            h[i] = other.h[i];
            e[i] = other.e[i];
        }
    }
};
// Define the result structure
template<int M, int N>
struct LinearProgrammingResult {
    float y0[M];
    int inB[M];
    bool e[N];
    int iters;
    bool errout;
    // Other result members
    // Default constructor, initializes all member variables to 0
    LinearProgrammingResult() : iters(0), errout(false) {
        // Initialize array member variables to 0
        for (int i = 0; i < M; ++i) {
            y0[i] = 0.0f;
            inB[i] = 0;
        }
        for (int i = 0; i < N; ++i) {
            e[i] = false;
        }
    }
    // Copy constructor
    LinearProgrammingResult(const LinearProgrammingResult<M, N>& other) {
        // Copy member variables from another object to the current object
        for (int i = 0; i < M; ++i) {
            y0[i] = other.y0[i];
            inB[i] = other.inB[i];
        }
        for (int i = 0; i < N; ++i) {
            e[i] = other.e[i];
        }
        iters = other.iters;
        errout = other.errout;
    }
};


// Define the function template, which is the implementation of the modified simplex algorithm. It is required that row A has full rank, and inB and e are known before solving the problem, that is, an initial basic feasible solution needs to be found to start the algorithm iteration. e=0 indicates that the initial solution is on the upper limit h, otherwise it is 0.
// see A.6.3 Simplex Method[1]. this function reimplement the simplxuprevsol of this book:
// [1] W. Durham, K. A. Bordignon, and R. Beck, Aircraft control allocation. none: John Wiley & Sons, 2017.
// and you can download the code on: https://github.com/mengchaoheng/control_allocation.git
//
// Current reference implementation:
//   PCA/simplxuprevsol_tiebreak.m
//
// The original MATLAB reference is simplxuprevsol.m.  The C++ code follows
// the same bounded revised simplex algorithm, but the public implementation is
// simplxuprevsol_tiebreak: in degenerate LPs multiple bases can give the same
// B*u but very different actuator vectors, so the tie-break rules below make
// the entering/leaving choices reproducible:
//   - reduced-cost ties choose the smallest variable index in inD;
//   - ratio ties choose the smallest current basic variable index inB;
//   - ratio ties use machine-epsilon scale, then fall back to the raw minimum
//     ratio pivot if the guarded pivot violates 0 <= y <= h.
template<int M, int N>
LinearProgrammingResult<M, N> simplxuprevsol_tiebreak(LinearProgrammingProblem<M, N> problem) {
    // Bounded Revised Simplex

    // function [yout, inBout,eout, itout,errout] = simplxuprevsol(A,ct,b,inB,inD,h,e,m,n,itlim)

    // Solves the linear program:
    //         minimize c'y
    //         subject to
    //         Ay = b
    //         0<= y <= h

    // Inputs:
    //         A [m,n]   = lhs Matrix of equaltity constraints
    //         ct [1,n]  = transpose of cost vector
    //         b [m,1]   = rhs vector for equality constraint
    //         inB [m]   = Vector of indices of unknowns in the initial basic set
    //         inD [n-m] = Vector of indices of unknowns not in the initial basic set
    //         h[n,1]    = Upper Bound for unknowns
    //         e[n,1]    = Sign for unknown variables (+ lower bound, - upper bound)
    // Optional inputs:
    //         m,n       = number of constraints, unknowns (Opposite standard
    //                     CA convention
    //         itlim     = Upper bound on the allowed iterations

    // Outputs:
    //         yout[n,1]  = Optimal output variable
    //         inBout     = indices of Basic vectors in output
    //         eout       = sign associate with output unknowns
    //         itout      = number of iterations remaining out of itlim
    //         errout     = Flag (=true) if unbounded is set

    // Modification History
    // 2002      Roger Beck  Original
    // 8/2014    Roger Beck  Update for use
    // 9/2014    Roger Beck  Added anti-cycling rule
    // 4/2024    Meng ChaoHeng
    // 5/2026    Meng ChaoHeng  Align pivot tie-breaks with
    //           PCA/simplxuprevsol_tiebreak.m for deterministic C++/MATLAB tests.

    LinearProgrammingResult<M, N> result;
    const SimplexTieBreakOptions &opts = problem.simplex_opts;
    const float pivot_tol = opts.pivot_zero_tol;
    // 使用 problem.inB, problem.inD, problem.itlim, problem.A, problem.b, problem.c, problem.h, problem.e, problem.tol
    const int n_m=N-M;
    // Index list for all variables
    int ind_all[N];
    for (int num = 0, index = 0; num < N; ++num, ++index) {
        ind_all[index] = num;
    }
    // Partition A, we have inD, which the element in ind_all but not in inB
    setdiff(ind_all, N, problem.inB, M, problem.inD);
    //djust signs problem if variables are initialized at upper bounds.
    for(int i=0; i<M; ++i)
    {
        for(int j=0; j<N; ++j)
        {
            if(!problem.e[j])
            {
                problem.A[i][j] *=-1;
                problem.b[i]+=problem.A[i][j]*problem.h[j];
            }
        }
    }
    for(int j=0; j<N; ++j)
    {
        if(!problem.e[j])
        {
            problem.c[j] *=-1;
        }
    }
    //==============================
    matrix::SquareMatrix<float, M> A_inB;
    matrix::Matrix<float, M, n_m> A_inD;
    matrix::Vector<float, M> c_inB;
    matrix::Vector<float, n_m> c_inD;
    //===============use inB and inD to inital==============
    for(int i=0; i<M; ++i)
    {
        for(int j=0; j<M; ++j)
        {
            A_inB(i,j)=problem.A[i][problem.inB[j]];
        }
        c_inB(i)=problem.c[problem.inB[i]];
    }
    for(int i=0; i<M; ++i)
    {
        for(int j=0; j<N-M; ++j)
        {
            A_inD(i,j)=problem.A[i][problem.inD[j]];
        }
    }
    for(int i=0; i<N-M; ++i)
    {
        c_inD(i)=problem.c[problem.inD[i]];
    }
    matrix::Vector<float, M> b_vec(problem.b);
    // inital some value
    Matrix<float, 1, M> lamt;
    lamt.setZero();
    Matrix<float, 1, n_m> rdt;
    rdt.setZero();
    matrix::Vector<float, M> A_qel;
    A_qel.setZero();
    matrix::Vector<float, M> yq;
    yq.setZero();
    matrix::Vector<float, M> rat;
    rat.setZero();
    //==============================
    //  %Initial Solution.  A(:,inB) is square in the simplex basis; use
    // LU with partial pivoting to match MATLAB mldivide's square dense path.
    matrix::Vector<float, M> y0;
    solveSquareLU(A_inB, b_vec, y0);
    // Initialize Loop Termination Conditionss
    bool done = false;
    bool unbounded = false;
    int iters =0;
     while ((!done) && (!unbounded) && (iters < problem.itlim))
    {
        iters = iters+1;
        // Calculate transpose of relative cost vector based on current basis
        matrix::Vector<float, M> lamt_col;
        solveSquareLU(A_inB.transpose(), c_inB, lamt_col);
        lamt = lamt_col.transpose();
        rdt = c_inD.transpose()-lamt*A_inD;
        float minr;
        size_t qind;
        // Reduced-cost ties choose the smallest variable index qel.
        select_entering_tiebreak<n_m>(rdt, problem.inD, minr, qind, opts);
        if(minr >=0)  // If all relative costs are positive then the solution is optimal. have to compare with 0 !
        {
            done = true;
            break;
        }
        int qel = problem.inD[qind];  // Unknown to Enter the basis minimizes relative cost
        for(int i=0;i<problem.m;++i){
            A_qel(i)=problem.A[i][qel];
        }
        solveSquareLU(A_inB, A_qel, yq); // Vector to enter in terms of the current Basis vector
         // Check wether all the abs of yq[i] is greater than tol.
        bool flag=false;
        for(int i=0;i<M;++i){
            if(fabs(yq(i)) > pivot_tol)
            {
                flag = true;
                break;
            }
        }
        if(!flag)
        {
            unbounded = true; // Check this condition
            break;
        }
        // Compute ratio how much each current basic variable will have to move for the entering variable.
        // careful here
        float hinB[M];
        for(int i=0;i<M;++i)
        {
            if(fabs(yq(i))>pivot_tol)
            {
                rat(i)=y0(i)/yq(i);
                hinB[i]=problem.h[problem.inB[i]];
                // If yq < 0 then increasing variable when it leaves the basis will minimize cost
                if(yq(i)<0 ) // have to be compare with 0!!!
                {
                    rat(i)-=hinB[i]/yq(i);
                }
            }
            else // If an element yq ~=0 then it doesn't change for the entering variable and shouldn't be chosen
            {
                rat(i)=INFINITY;
            }
        }
        // Variable to exit is moving to its minimum value.  Ratio ties choose
        // the smallest current basic variable index inB(p), matching
        // PCA/simplxuprevsol_tiebreak.m.
        float minrat=rat(0);
        size_t p=0;
        size_t raw_p=0;
        select_leaving_tiebreak<M>(rat, problem.inB, minrat, p, raw_p, opts);
        // If the minimum ratio is zero, then the solution is degenerate and the entering
        // variable will not change the basis---invoke Bland's selection rule to avoid
        // cycling.
        if (fabs(minrat) <= opts.zero_step_tol)
        {
            // Bland anti-cycling rule: choose the smallest variable index
            // among negative reduced costs, matching MATLAB tiebreak.
            select_bland_negative_tiebreak<n_m>(rdt, problem.inD, qind);
            qel = problem.inD[qind];
            for(int i=0;i<problem.m;++i){
                A_qel(i)=problem.A[i][qel];
            }
            solveSquareLU(A_inB, A_qel, yq); // Vector to enter in terms of the current Basis vector
            bool flag1=false;
            for(int i=0;i<M;++i){
                if(fabs(yq(i)) > pivot_tol)
                {
                    flag1 = true;
                    break;
                }
            }
            if(!flag1)
            {
                unbounded = true; // Check this condition
                break;
            }
            // Recompute rations and determine variable to leave
            float hinB1[M];
            for(int i=0;i<M;++i)
            {
                hinB1[i]=problem.h[problem.inB[i]];
                if(fabs(yq(i))>pivot_tol)
                {
                    rat(i)=y0(i)/yq(i);
                    if(yq(i)<0) // If yq < 0 then increasing variable when it leaves the basis will minimize cost
                    {
                        rat(i)-=hinB1[i]/yq(i);
                    }
                }
                else
                {
                    rat(i)=INFINITY; // If an element yq ~=0 then it doesn't change for the entering variable and shouldn't be chosen
                }
            }
            // Variable to exit is moving to its minimum value.
            minrat=rat(0);
            p=0;
            raw_p=0;
            select_leaving_tiebreak<M>(rat, problem.inB, minrat, p, raw_p, opts);
        }
        const LinearProgrammingProblem<M, N> problem_before(problem);
        const matrix::SquareMatrix<float, M> A_inB_before(A_inB);
        const matrix::Matrix<float, M, n_m> A_inD_before(A_inD);
        const matrix::Vector<float, M> c_inB_before(c_inB);
        const matrix::Vector<float, n_m> c_inD_before(c_inD);
        const matrix::Vector<float, M> b_vec_before(b_vec);

        auto apply_pivot = [&](size_t pivot_p, float pivot_minrat) {
            // Maintain the bounded simplex as only having lower bounds by
            // recasting any variable that needs to move to its opposite bound.
            const float bound_tol = lp_bound_compare_tolerance(pivot_minrat, problem.h[qel], opts);
            if (pivot_minrat > problem.h[qel] + bound_tol)
            {
                // Case 1: Entering variable goes to opposite bound and current basis is maintained.
                problem.e[qel] =!problem.e[qel];
                for(int i=0; i<M; ++i)
                {
                    problem.A[i][qel] *= -1;
                    b_vec(i)+=problem.A[i][qel]*problem.h[qel];
                    problem.b[i]=b_vec(i);
                }
                problem.c[qel] *= -1;

                for(int i=0; i<M; ++i)
                {
                    A_inD(i,qind)=problem.A[i][qel];
                }
                c_inD(qind)=problem.c[qel];
            }
            else if(yq(pivot_p) > 0)
            {
                // Case 2: Leaving variable returns to lower bound (0).
                int pel = problem.inB[pivot_p];
                problem.inB[pivot_p]= qel;
                problem.inD[qind]= pel;
                for(int i=0; i<M; ++i)
                {
                    A_inB(i,pivot_p)=problem.A[i][qel];
                }
                c_inB(pivot_p)=problem.c[qel];
                for(int i=0; i<M; ++i)
                {
                    A_inD(i,qind)=problem.A[i][pel];
                }
                c_inD(qind)=problem.c[pel];
            }
            else
            {
                // Case 3: Leaving variable moves to upper bound.
                int pel = problem.inB[pivot_p];
                problem.e[pel]=!problem.e[pel];
                for(int i=0; i<M; ++i)
                {
                    problem.A[i][pel] *= -1;
                    b_vec(i)+=problem.A[i][pel]*problem.h[pel];
                    problem.b[i]=b_vec(i);
                }
                problem.inB[pivot_p]= qel;
                problem.inD[qind]= pel;
                problem.c[pel] *= -1;

                for(int i=0; i<M; ++i)
                {
                    A_inB(i,pivot_p)=problem.A[i][qel];
                }
                c_inB(pivot_p)=problem.c[qel];
                for(int i=0; i<M; ++i)
                {
                    A_inD(i,qind)=problem.A[i][pel];
                }
                c_inD(qind)=problem.c[pel];
            }
        };

        auto current_basis_violates_box = [&]() {
            float h_inB_current[M];
            for (int i=0; i<M; ++i) {
                h_inB_current[i] = problem.h[problem.inB[i]];
            }
            return lp_violates_box<M>(y0, h_inB_current, opts);
        };

        apply_pivot(p, minrat);
        solveSquareLU(A_inB, b_vec, y0);

        // MATLAB tiebreak does not blindly keep a tied-ratio pivot: if the
        // deterministic pivot makes the basic solution infeasible, restore the
        // state and use the raw minimum-ratio pivot.
        if (p != raw_p && current_basis_violates_box())
        {
            problem = problem_before;
            A_inB = A_inB_before;
            A_inD = A_inD_before;
            c_inB = c_inB_before;
            c_inD = c_inD_before;
            b_vec = b_vec_before;

            p = raw_p;
            minrat = rat(p);
            apply_pivot(p, minrat);
            solveSquareLU(A_inB, b_vec, y0);
        }

        if (current_basis_violates_box())
        {
            unbounded = true;
            break;
        }
    }
    result.errout = unbounded;
    // Set result.y0, result.inB, result.e, etc.
    for(int i=0; i<M; ++i)
    {
        result.y0[i]=y0(i);
        result.inB[i]=problem.inB[i];
    }
    for(int i=0; i<N; ++i)
    {
        result.e[i]=problem.e[i];
    }
    result.iters=iters;
    return result;
}

template<int M, int N>
LinearProgrammingResult<M, N> BoundedRevisedSimplex(LinearProgrammingProblem<M, N> problem) {
    // Compatibility wrapper for older code.  New allocator code calls
    // simplxuprevsol_tiebreak() directly.
    return simplxuprevsol_tiebreak(problem);
}




// Aerocraft base class template
template <int ControlSize, int EffectorSize>
class AircraftBase {
public:
    float controlVector[EffectorSize]; // Control vector
    float controlEffectMatrix[ControlSize][EffectorSize]; // Control effect matrix (generalizedMomentSize X controlVectorSize)
    float upperLimits[EffectorSize]; // Control vector upper limits
    float lowerLimits[EffectorSize]; // Control vector lower limits
    float BuMin[ControlSize];
    // Constructor
    // Copy constructor
    AircraftBase(const AircraftBase& other) {
        // Copy controlVector
        for (int i = 0; i < EffectorSize; ++i) {
            controlVector[i] = other.controlVector[i];
            upperLimits[i] = other.upperLimits[i];
            lowerLimits[i] = other.lowerLimits[i];
        }

        // Copy controlEffectMatrix
        for (int i = 0; i < ControlSize; ++i) {
            for (int j = 0; j < EffectorSize; ++j) {
                controlEffectMatrix[i][j] = other.controlEffectMatrix[i][j];
            }
            BuMin[i]=other.BuMin[i];
        }
    }
    AircraftBase() {
        // Initialize controlVector, controlEffectMatrix, generalizedMoment, upperLimits, lowerLimits arrays
        // Can use default initialization or custom initialization methods
        // For example:
        for (int i = 0; i < EffectorSize; ++i) {
            controlVector[i] = 0.0f;
            upperLimits[i] = 0.0f;
            lowerLimits[i] = 0.0f;
            for (int j = 0; j < ControlSize; ++j) {
                controlEffectMatrix[j][i] = 0.0f;
            }
        }
        for (int i = 0; i < ControlSize; ++i) {
            BuMin[i]=0;
        }
    }
    // Destructor
    ~AircraftBase() {
        // No need to manually release memory because arrays are allocated on the stack and will be automatically released at the end of the object's lifecycle
    }
};

// Aircraft class template, different aircraft define new classes inheriting from the base class
template <int ControlSize, int EffectorSize>
class Aircraft : public AircraftBase<ControlSize, EffectorSize> {
private:
    // Add model parameters specific to the aircraft type
public:
    // Constructor
    // Copy constructor
    // Copy constructor
    Aircraft(const Aircraft& other) : AircraftBase<ControlSize, EffectorSize>(other) {
        // Copy member variables from the other object to the new object
        l1 = other.l1;
        l2 = other.l2;
        k_v = other.k_v;
        upper = other.upper;
        lower = other.lower;
    }
    Aircraft() : AircraftBase<ControlSize, EffectorSize>() {
        // Optional initialization code
        l1=0;
        l2=0;
        k_v=0;
    }
    // Constructor accepting initialization parameters corresponding to the aircraft class template parameters
    Aircraft(const float (&controlEffectMatrixInit)[ControlSize][EffectorSize],
             const float (&upperLimitsInit)[EffectorSize],
             const float (&lowerLimitsInit)[EffectorSize]) {
        // Use the passed initialization parameters to initialize the aircraft's array members
        for (int i = 0; i < EffectorSize; ++i) {
            this->controlVector[i] = 0;
            this->upperLimits[i] = upperLimitsInit[i];
            this->lowerLimits[i] = lowerLimitsInit[i];
            for (int j = 0; j < ControlSize; ++j) {
                this->controlEffectMatrix[j][i] = controlEffectMatrixInit[j][i];
            }
        }
        for (int i = 0; i < ControlSize; ++i) {
            float temp = 0.0f;
            for (int j = 0; j < EffectorSize; ++j) {
                temp +=  this->controlEffectMatrix[i][j]*this->lowerLimits[j];
            }
            this->BuMin[i] = temp; // Calculate BuMin
        }
    }
    // Constructor accepting initialization parameters corresponding to the aircraft class template parameters
    Aircraft(const float (&upperLimitsInit)[EffectorSize],
             const float (&lowerLimitsInit)[EffectorSize]) {
        // Use the passed initialization parameters and aircraft
        for (int i = 0; i < EffectorSize; ++i) {
            this->controlVector[i] = 0;
            this->upperLimits[i] = upperLimitsInit[i];
            this->lowerLimits[i] = lowerLimitsInit[i];
            for (int j = 0; j < ControlSize; ++j) {
                this->controlEffectMatrix[j][i] = 0;
            }
        }
        // and define other value manual to set B (controlEffectMatrix).
        for (int i = 0; i < ControlSize; ++i) {
            float temp = 0.0f;
            for (int j = 0; j < EffectorSize; ++j) {
                temp +=  this->controlEffectMatrix[i][j]*this->lowerLimits[j];
            }
            this->BuMin[i] = temp; // Calculate BuMin
        }
    }
    Aircraft(const float (&controlEffectMatrixInit)[ControlSize][EffectorSize], const float& lowerBound, const float& upperBound) : lower(lowerBound), upper(upperBound){
        // Use the passed initialization parameters and aircraft
        for (int i = 0; i < EffectorSize; ++i) {
            this->controlVector[i] = 0;
            this->upperLimits[i] = upper;
            this->lowerLimits[i] = lower;
            for (int j = 0; j < ControlSize; ++j) {
                this->controlEffectMatrix[j][i] = controlEffectMatrixInit[j][i];
            }
        }
        for (int i = 0; i < ControlSize; ++i) {
            float temp = 0.0f;
            for (int j = 0; j < EffectorSize; ++j) {
                temp +=  this->controlEffectMatrix[i][j]*this->lowerLimits[j];
            }
            this->BuMin[i] = temp; // Calculate BuMin
        }
    }

    // Set model parameters function
    int num_control=ControlSize;
    int num_effector=EffectorSize;
    float l1;
    float l2;
    float k_v;
    float lower;
    float upper;

    // Destructor
    ~Aircraft() {
        // If there are resources to release, you can add code here
    }

    // Other member functions and member variable definitions
};
// Control allocation base class template
template <int ControlSize, int EffectorSize>
class ControlAllocatorBase {
public:
    // Constructor
    ControlAllocatorBase() : aircraft() {
        // Optional initialization code
    }
    // Parameterized constructor
    ControlAllocatorBase(const Aircraft<ControlSize, EffectorSize>& ac)
        : aircraft(ac) {
        // Use the passed aircraft object to initialize the aircraft member
    }

    // Other mathematical functions and member variable definitions
    Aircraft<ControlSize, EffectorSize> aircraft; // Constructor initialized

};
// Control allocation class template
template <int ControlSize, int EffectorSize>
class DP_LP_ControlAllocator : public ControlAllocatorBase<ControlSize, EffectorSize> {
private:
    // Add algorithm setting parameters
public:
    // Constructor, use aircraft to preset LinearProgrammingProblem
    // Constructor
    DP_LP_ControlAllocator(const Aircraft<ControlSize, EffectorSize>& ac)
        : ControlAllocatorBase<ControlSize, EffectorSize>(ac){
        // Use aircraft, generalizedMoment to initialize member variables DP_LPCA_problem and Pre_DP_LPCA_problem
        // Linear programming data
        //=====================================DP_LPCA_problem================================
        // float cs_max=this->aircraft.upperLimits[0]-this->aircraft.lowerLimits[0]; // Store the maximum absolute value
        // for (int i = 0; i < ControlSize; ++i) {
        //     float absValue = fabs(this->aircraft.upperLimits[i]-this->aircraft.lowerLimits[i]); // Calculate the absolute value of the i-th element in 'yd'
        //     if (absValue > cs_max) { // If the current absolute value is greater than 'my', update my and 'iy'
        //         cs_max = absValue;
        //     }
        // }
        // upper_lam = cs_max/std::numeric_limits<float>::epsilon();
        DP_LPCA_problem.itlim = 100;
        for(int i=0; i<DP_LPCA_problem.n-1; ++i)
        {
            DP_LPCA_problem.c[i] = 0;
        }
        DP_LPCA_problem.c[DP_LPCA_problem.n-1] = -1;

        DP_LPCA_problem.h[DP_LPCA_problem.n-1] = upper_lam;
        // update A b h every time
        for(int i=0; i<DP_LPCA_problem.m; ++i)
        {
            for(int j=0; j<DP_LPCA_problem.n-1; ++j)
            {
                DP_LPCA_problem.A[i][j] = this->aircraft.controlEffectMatrix[i][j];
            }
            DP_LPCA_problem.A[i][DP_LPCA_problem.n-1] = 0;
            DP_LPCA_problem.b[i] = -this->aircraft.BuMin[i];
        }
        for(int i=0; i<DP_LPCA_problem.n-1; ++i)
        {
            DP_LPCA_problem.h[i] = this->aircraft.upperLimits[i]-this->aircraft.lowerLimits[i];
        }
        //==================================PreDP_LPCA_problem================================
        Pre_DP_LPCA_problem.itlim = 100;

        //ci
        for(int i=0; i<DP_LPCA_problem.n; ++i)
        {
            Pre_DP_LPCA_problem.c[i] =0;
        }
        for(int i=0; i<DP_LPCA_problem.m; ++i)
        {
            Pre_DP_LPCA_problem.c[i+DP_LPCA_problem.n] = 1;
        }
        // inBi
        for(int i=0; i<DP_LPCA_problem.m; ++i)
        {
            Pre_DP_LPCA_problem.inB[i] = DP_LPCA_problem.n+i;
        }
        // ei
        for(int i=0; i<DP_LPCA_problem.m+DP_LPCA_problem.n; ++i)
        {
            Pre_DP_LPCA_problem.e[i] = true;
        }
        //Ai bi=b
        for(int i=0; i<DP_LPCA_problem.m; ++i)
        {
            for(int j=0; j<DP_LPCA_problem.n; ++j)
            {
                Pre_DP_LPCA_problem.A[i][j] = DP_LPCA_problem.A[i][j];
            }
            Pre_DP_LPCA_problem.b[i] = DP_LPCA_problem.b[i]; // the same as DP_LPCA_problem

        }
        for(int i=0; i<DP_LPCA_problem.m; ++i)
        {
            Pre_DP_LPCA_problem.A[i][i + DP_LPCA_problem.n] = (DP_LPCA_problem.b[i] > 0) ? 1 : -1; // sb = 2*(b > 0)-1; Ai = [A diag(sb)];
        }
        // hi
        for(int i=0; i<DP_LPCA_problem.n; ++i)
        {
            Pre_DP_LPCA_problem.h[i] = DP_LPCA_problem.h[i];
        }
        for(int i=0; i<DP_LPCA_problem.m; ++i)
        {
            Pre_DP_LPCA_problem.h[i+DP_LPCA_problem.n] = 2*fabs(DP_LPCA_problem.b[i]);
        }
        //================================== DPscaled_LPCA_problem ================================
        DPscaled_LPCA_problem.itlim = 100;
        float yd[ControlSize]; // random non-zero value for initial problem sizing.
        for (int i = 0; i < ControlSize; ++i) {
            yd[i] = 0.1f * static_cast<float>(i + 1);
        }
        if (ControlSize > 2) {
            yd[2] = -0.1f;
        }
        // update A b c h every time
        float my=fabs(yd[0]); // Store the maximum absolute value
        int iy=0; // Store the index of the maximum absolute value
        for (int i = 0; i < ControlSize; ++i) {
            float absValue = fabs(yd[i]); // Calculate the absolute value of the i-th element in 'yd'
            if (absValue > my) { // If the current absolute value is greater than 'my', update my and 'iy'
                my = absValue;
                iy = i;
            }
        }
        float Bt[ControlSize][EffectorSize];
        float ydt[ControlSize];
        int row_order[ControlSize];
        build_dp_scaled_row_order<ControlSize>(iy, row_order);
        for(int i=0;i<ControlSize;++i){
            const int src = row_order[i];
            ydt[i]=yd[src];
            for (int j = 0; j <EffectorSize; ++j) {
                Bt[i][j] = this->aircraft.controlEffectMatrix[src][j];
            }
        }
        // M = [ydt(2:ControlSize) -ydt(1)*eye(ControlSize-1)];
        float M[ControlSize-1][ControlSize];
        // M[0][0]=ydt[1];
        // M[1][0]=ydt[2];
        // M[0][1]=-ydt[0];
        // M[1][1]=0;
        // M[0][2]=0;
        // M[1][2]=-ydt[0];
        // or
        // Initialize all elements of M to 0, and simultaneously fill the first column and diagonal elements of M
        for (int i = 0; i < ControlSize - 1; ++i) {
            for (int j = 0; j < ControlSize; ++j) {
                if (j == 0) {
                    // Fill the first column of M
                    M[i][0] = ydt[i + 1];
                } else if (j == i + 1) {
                    // Fill the diagonal elements
                    M[i][j] = -ydt[0];
                } else {
                    // Initialize other elements to 0
                    M[i][j] = 0.0f;
                }
            }
        }
        for (int i = 0; i < ControlSize-1; ++i) {
            for (int j = 0; j < EffectorSize; ++j) {
                DPscaled_LPCA_problem.A[i][j] = 0;
                for (int k = 0; k < ControlSize; ++k) {
                    DPscaled_LPCA_problem.A[i][j] += M[i][k] * Bt[k][j];
                }
            }
        }
        for(int i=0; i<ControlSize-1; ++i)
        {
            float temp=0;
            for(int j=0; j<EffectorSize; ++j)
            {
                temp += -DPscaled_LPCA_problem.A[i][j]*this->aircraft.lowerLimits[j];
            }
            DPscaled_LPCA_problem.b[i] = temp;
        }
        for (int i = 0; i < EffectorSize; ++i) {
            DPscaled_LPCA_problem.c[i] = 0;
            for (int j = 0; j < ControlSize; ++j) {
                DPscaled_LPCA_problem.c[i] += -Bt[j][i] * ydt[j];
            }
        }
        for (int i = 0; i < EffectorSize; ++i) {
            DPscaled_LPCA_problem.h[i] = this->aircraft.upperLimits[i]-this->aircraft.lowerLimits[i];
        }
        //==================================Pre_DPscaled_LPCA_problem================================
        Pre_DPscaled_LPCA_problem.itlim = 100;
        for(int i=0; i<DPscaled_LPCA_problem.n; ++i)
        {
            Pre_DPscaled_LPCA_problem.c[i] =0;
        }
        for(int i=0; i<DPscaled_LPCA_problem.m; ++i)
        {
            Pre_DPscaled_LPCA_problem.c[i+DPscaled_LPCA_problem.n] = 1;
        }
        for(int i=0; i<DPscaled_LPCA_problem.m; ++i)
        {
            Pre_DPscaled_LPCA_problem.inB[i] = DPscaled_LPCA_problem.n+i;
        }
        for(int i=0; i<DPscaled_LPCA_problem.m+DPscaled_LPCA_problem.n; ++i)
        {
            Pre_DPscaled_LPCA_problem.e[i] = true;
        }
        // update Ai bi hi every time
        for(int i=0; i<DPscaled_LPCA_problem.m; ++i)
        {
            for(int j=0; j<DPscaled_LPCA_problem.n; ++j)
            {
                Pre_DPscaled_LPCA_problem.A[i][j] = DPscaled_LPCA_problem.A[i][j];
            }
            Pre_DPscaled_LPCA_problem.b[i] = DPscaled_LPCA_problem.b[i]; // the same as DPscaled_LPCA_problem

        }
        for(int i=0; i<DPscaled_LPCA_problem.m; ++i)
        {
            Pre_DPscaled_LPCA_problem.A[i][i + DPscaled_LPCA_problem.n] = (DPscaled_LPCA_problem.b[i] > 0) ? 1 : -1; // sb = 2*(b > 0)-1; Ai = [A diag(sb)];
        }
        for(int i=0; i<DPscaled_LPCA_problem.n; ++i)
        {
            Pre_DPscaled_LPCA_problem.h[i] = DPscaled_LPCA_problem.h[i];
        }
        for(int i=0; i<DPscaled_LPCA_problem.m; ++i)
        {
            Pre_DPscaled_LPCA_problem.h[i+DPscaled_LPCA_problem.n] = 2*fabs(DPscaled_LPCA_problem.b[i]);
        }
        //   for restoring
        B.setZero();
        for (int i = 0; i < ControlSize; ++i) {
            for (int j = 0; j < EffectorSize; ++j) {
                B(i,j) = this->aircraft.controlEffectMatrix[i][j];
            }
        }
    }
    // Set Algorithm Parameter Function
    // Destructor
    ~DP_LP_ControlAllocator() {
        // If there are resources that need to be released, you can add code here
    }

    void setSimplexTieBreakOptions(const SimplexTieBreakOptions &opts) {
        DP_LPCA_problem.simplex_opts = opts;
        Pre_DP_LPCA_problem.simplex_opts = opts;
        DPscaled_LPCA_problem.simplex_opts = opts;
        Pre_DPscaled_LPCA_problem.simplex_opts = opts;
    }

    void setSimplexMachineEps(float machine_eps) {
        setSimplexTieBreakOptions(SimplexTieBreakOptions::fromMachineEps(machine_eps));
    }

    // To find an initial condition, many linear programming solvers treat the solution in two phases. Phase one solves a specially constructed problem designed to yield a basic feasible solution that is used to initialize the original problem in phase two.
    // So we have DP_LPCA and DPscaled_LPCA
    void DP_LPCA(float input[ControlSize], float output[EffectorSize], int& err, float & rho){
        // Direction Preserving Control Allocation Linear Program

        // function [u, errout] = DP_LPCA(yd,B,uMin,uMax,itlim,upper_lam);

        // Solves the control allocation problem while preserving the
        // objective direction for unattainable commands. The solution
        // is found by solving the problem,
        // min -lambda,
        // s.t. B*u = lambda*yd, uMin<=u<=uMax, 0<= lambda <=1

        // For yd outside the AMS, the solution returned is that the
        // maximum in the direction of yd.

        // For yd strictly inside the AMS, the solution achieves
        // Bu=yd and m-n controls will be at their limits; but there
        // is no explicit preference to which solution will be
        // returned. This limits the usefulness of this routine as
        // a practical allocator unless preferences for attainable solutions
        // are handled externally.

        // (For derivation of a similar formulation see A.1.2 and A.2.3 in the
        // text)


        // Inputs:
        //         yd [n]    = Desired objective
        //         B [n,m]   = Control Effectiveness matrix
        //         uMin[m,1] = Lower bound for controls
        //         uMax[m,1] = Upper bound for controls
        //         itlim     = Number of allowed iterations limit
        //                         (Sum of iterations in both branches)

        // Outputs:
        //         u[m,1]     = Control Solution
        //         errout     = Error Status code
        //                         0 = found solution
        //                         <0 = Error in finding initial basic feasible solution
        //                         >0 = Error in finding final solution
        //                         -1,1 = Solver error (unbounded solution)
        //                         -2   = Initial feasible solution not found
        //                         -3,3 = Iteration limit exceeded
        //         itlim      = Number of iterations remaining after solution found
        //         upper_lam  = the upper limit of lambda

        // Calls:
        //         simplxuprevsol_tiebreak = Bounded Revised Simplex solver
        //         with deterministic guarded pivot tie-breaks.

        // Notes:
        // If errout ~0 there was a problem in the solution. %

        // Error code < 0 implies an error in the initialization and there is no guarantee on
        // the quality of the output solution other than the control limits.
        // Error code > 0 for errors in final solution--B*u is in the correct direction and has
        // magnitude < yd, but B*u may not equal yd (for yd attainable)
        // or be maximized (for yd unattainable)

        // Modification History
        // 2002      Roger Beck  Original (DPcaLP8.m)
        // 8/2014    Roger Beck  Update for use in text
        // 4/2024    Meng ChaoHeng  Implement in cpp

        err = 0;
        rho = 0;

        // DP_LPCA function uses aircraft data to describe the allocation
        // problem as a DP_LP problem and solves it using simplxuprevsol_tiebreak.
        //=======================
        // Figure out how big the problem is (use standard CA definitions for m & n)
        // but in here we use [m,k] = size(B) instead of [n,m] = size(B) in matlab. just for adapt to BoundedRevisedSimplex
        // we use [m,n] = size(A) in BoundedRevisedSimplex, that is, k + 1 = n.
        // Check to see if yd == 0
        // May want to adjust the tolerance to improve numerics of later steps
        bool flag=false;
        for(int i=0;i<ControlSize;++i){
            if(fabs(input[i]) > DP_LPCA_problem.tol)
            {
                flag = true; // Check this condition
                break;
            }
        }
        if(!flag){
            for(int i=0;i<EffectorSize;++i){
                output[i]=0;
            }
            err=-1;
            return;
        }
        //=======================
        // We inital this problem in constructor.
        // Construct an LP using scaling parameter to enforce direction preserving
        // To find Feasible solution construct problem with appended slack variables
        // ref. is A.6.4 Initialization of the Simplex Algorithm of <Aircraft control allocation>

        // now we update the problem by input data.
        if(isupdate){
            Update();
            isupdate = false; // only update once.
        }
        // update A b h by input data.
        // update sb(since b is update) Ai bi hi every time
        for(int i=0; i<DP_LPCA_problem.m; ++i)
        {
            DP_LPCA_problem.A[i][DP_LPCA_problem.n-1] = -input[i];
            DP_LPCA_problem.b[i] = -this->aircraft.BuMin[i]; //

            Pre_DP_LPCA_problem.A[i][DP_LPCA_problem.n-1] = -input[i]; // the same as DP_LPCA_problem.A[i][DP_LPCA_problem.n-1]
            Pre_DP_LPCA_problem.b[i] = -this->aircraft.BuMin[i]; // the same as DP_LPCA_problem

            Pre_DP_LPCA_problem.A[i][i + DP_LPCA_problem.n] = (DP_LPCA_problem.b[i] > 0) ? 1 : -1; // sb = 2*(b > 0)-1; Ai = [A diag(sb)];
            Pre_DP_LPCA_problem.h[i+DP_LPCA_problem.n] = 2*fabs(DP_LPCA_problem.b[i]);
        }

        // Use Bounded Revised Simplex to find initial basic feasible point of original program
        auto result_init = simplxuprevsol_tiebreak(Pre_DP_LPCA_problem);

        // Check that Feasible Solution was found
        if(result_init.iters>=Pre_DP_LPCA_problem.itlim){
            err = -3;
        }
        for(int i=0;i<ControlSize;++i){
            if(result_init.inB[i]> EffectorSize) // DP_LPCA_problem is origin problem, k=DP_LPCA_problem.n-1 = EffectorSize
            {
                // which mean inital basic index is out of the origin problem.
                err = -2;
                break;
            }
        }
        if(result_init.errout){
            err = -1;
        }
        // solve Pre_DP_LPCA_problem but proccess DP_LPCA_problem
        float xout[DP_LPCA_problem.n];
        for(int i=0;i<DP_LPCA_problem.n;++i){
            xout[i]=0;
        }
        if(err!=0) // Construct an incorrect solution to accompany error flags
        {
            // use result_init data
            // matlab: indv = inB1<=(k+1); xout(inB1(indv)) = y1(indv); % in matlab the index from 1 to k, but cpp is 0 to k-1
            for(int i=0;i<ControlSize;++i){
                if(result_init.inB[i] <= EffectorSize)
                {
                    xout[result_init.inB[i]]=result_init.y0[i];
                }
            }
            // xout(~e1(1:k+1)) = -xout(~e1(1:k+1))+h(~e1(1:k+1));
            for(int i=0;i<DP_LPCA_problem.n;++i){
                if(!result_init.e[i]){
                    xout[i] = -xout[i] + DP_LPCA_problem.h[i];
                }
            }
        }
        else //No Error continue to solve problem
        {
            // Solve using initial problem from above
            // Construct solution to original LP problem from bounded simplex output
            // Set non-basic variables to 0 or h based on result_init.e
            // Set basic variables to result_init.y0 or h-result_init.y0.

            // update DP_LPCA_problem.inB and DP_LPCA_problem.e by result_init.inB and result_init.e[0 to k=EffectorSize] that is e1(1:k+1) in matlab.  k+1 at all, so int (i=0;i<EffectorSize+1;++i) or (int i=0;i<DP_LPCA_problem.n;++i)
            for(int i=0;i<ControlSize;++i){
                DP_LPCA_problem.inB[i]=result_init.inB[i];
            }
            for(int i=0;i<DP_LPCA_problem.n;++i){
                DP_LPCA_problem.e[i]=result_init.e[i];
            }
            auto result = simplxuprevsol_tiebreak(DP_LPCA_problem);

            for(int i=0;i<ControlSize;++i){
                xout[result.inB[i]]=result.y0[i];
            }
            for(int i=0;i<DP_LPCA_problem.n;++i){
                if(!result.e[i]){
                    xout[i]=-xout[i]+DP_LPCA_problem.h[i];
                }
            }

            if(result.iters>=DP_LPCA_problem.itlim){
                err = 3;
            }
            if(result.errout)
            {
                err = 1;
            }
        }
        // Transform back to control variables
        for(int i=0;i<EffectorSize;++i){
            output[i]=xout[i]+this->aircraft.lowerLimits[i];
        }
        // Use upper_lam to prevent control surfaces from approaching position limits
        rho = xout[EffectorSize];
        return;
    }
    void DPscaled_LPCA(float input[ControlSize], float output[EffectorSize], int& err, float & rho){
        // Direction Preserving Control Allocation Linear Program
        //     Reduced formulation (Solution Scaled from Boundary)

        // function [u,errout] = DPscaled_LPCA(yd,B,uMin,uMax,itlim);

        // Solves the control allocation problem while preserving the
        // objective direction for unattainable commands. The reduced
        // dimension of the linear program passed to the Bounded Revised
        // Simplex solver is formed by forcing the solution to be on the
        // boundary of the AMS and eliminating the highest magnitude
        // objective by solving the other constraints in terms of it.

        // For yd outside the AMS, the solution returned is that the
        // maximum in the direction of yd
        // B*u= lamda*yd
        // max lamda s.t. uMin <= u <= uMax

        // Reducing the degrees of freedom elminates the problems of redundant
        // solutions for attainable objectives. If the desired objective is on the
        // interior of the AMS the solution is scaled from the solution on the
        // boundary, yielding the same controls as the Direct Allocation solution.

        // (In the text this solution is discussed in section A.5.3)

        // (See Bodson, M., "Evaluation of Optimization Methods for
        //         Control Allocation",  AIAA 2001-4223).

        // Inputs:
        //         yd [n]    = Desired objective
        //         B [n,m]   = Control Effectiveness matrix
        //         uMin[m,1] = Lower bound for controls
        //         uMax[m,1] = Upper bound for controls
        //         itlim     = Number of allowed iterations limit
        //                         (Sum of iterations in both branches)

        // Outputs:
        //         u[m,1]     = Control Solution
        //         errout     = Error Status code
        //                         0 = found solution
        //                         <0 = Error in finding initial basic feasible solution
        //                         >0 = Error in finding final solution
        //                         -1,1 = Solver error (unbounded solution)
        //                         -2   = Initial feasible solution not found
        //                         -3,3 = Iteration limit exceeded

        // Calls:
        //         simplxuprevsol_tiebreak = Bounded Revised Simplex solver
        //         with deterministic guarded pivot tie-breaks.

        // Notes:
        // If yd is close to zero, u = 0;

        // Error code < 0 implies an error in the initialization and there is no guarantee on
        // the quality of the output solution other than the control limits.
        // Error code > 0 for errors in final solution.

        // Modification History
        // 2002      Roger Beck  Original ( DPcaLP2.m)
        // 8/2014    Roger Beck  Update
        // 4/2024    Meng ChaoHeng  Implement in cpp

        err = 0;
        rho = 0;

        // The DPscaled_LPCA function formulates the allocation problem as a
        // DP_LP problem and solves it using simplxuprevsol_tiebreak.
        //=======================
        // Figure out how big the problem is (use standard CA definitions for m & n)
        // but in here we use [m,k] = size(B) instead of [n,m] = size(B) in matlab. just for adapt to BoundedRevisedSimplex
        // we use [m,n] = size(A) in BoundedRevisedSimplex, that is, k + 1 = n.
        // Check to see if yd == 0
        // May want to adjust the tolerance to improve numerics of later steps
        bool flag=false;
        for(int i=0;i<ControlSize;++i){
            if(fabs(input[i]) > DPscaled_LPCA_problem.tol)
            {
                flag = true; // Check this condition
                break;
            }
        }
        if(!flag){
            for(int i=0;i<EffectorSize;++i){
                output[i]=0;
            }
            err = -1;
            return;
        }
        //=======================
        // We inital this problem in constructor.
        // Construct an LP using scaling parameter to enforce direction preserving
        // To find Feasible solution construct problem with appended slack variables
        // ref. is A.6.4 Initialization of the Simplex Algorithm of <Aircraft control allocation>

        // now we update the problem by input data.
        // update A b c but not h, h = uMax-uMin is assumpt always the same.
        // float yd[3]={0.1,0.1,0.2}; // random value for inital.
        //================================== DPscaled_LPCA_problem ================================
        // update A b c h every time
        float my=fabs(input[0]); // Store the maximum absolute value
        int iy=0; // Store the index of the maximum absolute value
        for (int i = 0; i < ControlSize; ++i) {
            float absValue = fabs(input[i]); // Calculate the absolute value of the i-th element in yd
            if (absValue > my) { // If the current absolute value is greater than my, update my and iy
                my = absValue;
                iy = i;
            }
        }
        float Bt[ControlSize][EffectorSize];
        float ydt[ControlSize];

        for(int i=0;i<ControlSize;++i){
            this->generalizedMoment[i] = input[i]; // just record.
        }

        int row_order[ControlSize];
        build_dp_scaled_row_order<ControlSize>(iy, row_order);
        for(int i=0;i<ControlSize;++i){
            const int src = row_order[i];
            ydt[i]=input[src];
            for (int j = 0; j <EffectorSize; ++j) {
                Bt[i][j] = this->aircraft.controlEffectMatrix[src][j];
            }
        }
        // M = [ydt(2:ControlSize) -ydt(1)*eye(ControlSize-1)];
        float M[ControlSize-1][ControlSize];
        // M[0][0]=ydt[1];
        // M[1][0]=ydt[2];
        // M[0][1]=-ydt[0];
        // M[1][1]=0;
        // M[0][2]=0;
        // M[1][2]=-ydt[0];
        // or
        // Initialize all elements of M to 0, while filling the first column and diagonal elements of M
        for (int i = 0; i < ControlSize - 1; ++i) {
            for (int j = 0; j < ControlSize; ++j) {
                if (j == 0) {
                    // Fill the first column of M
                    M[i][0] = ydt[i + 1];
                } else if (j == i + 1) {
                    // Fill the diagonal elements
                    M[i][j] = -ydt[0];
                } else {
                    // Initialize other elements to 0
                    M[i][j] = 0.0f;
                }
            }
        }
        for (int i = 0; i < ControlSize-1; ++i) {
            for (int j = 0; j < EffectorSize; ++j) {
                DPscaled_LPCA_problem.A[i][j] = 0;
                for (int k = 0; k < ControlSize; ++k) {
                    DPscaled_LPCA_problem.A[i][j] += M[i][k] * Bt[k][j];
                }
            }
        }
        for(int i=0; i<ControlSize-1; ++i)
        {
            float temp=0;
            for(int j=0; j<EffectorSize; ++j)
            {
                temp += -DPscaled_LPCA_problem.A[i][j]*this->aircraft.lowerLimits[j];
            }
            DPscaled_LPCA_problem.b[i] = temp;
        }
        for (int i = 0; i < EffectorSize; ++i) {
            DPscaled_LPCA_problem.c[i] = 0;
            for (int j = 0; j < ControlSize; ++j) {
                DPscaled_LPCA_problem.c[i] += -Bt[j][i] * ydt[j];
            }
        }
        for (int i = 0; i < EffectorSize; ++i) {
            DPscaled_LPCA_problem.h[i] = this->aircraft.upperLimits[i]-this->aircraft.lowerLimits[i];
        }
        //==================================Pre_DPscaled_LPCA_problem================================
        // update Ai bi hi every time
        for(int i=0; i<DPscaled_LPCA_problem.m; ++i)
        {
            for(int j=0; j<DPscaled_LPCA_problem.n; ++j)
            {
                Pre_DPscaled_LPCA_problem.A[i][j] = DPscaled_LPCA_problem.A[i][j];
            }
            Pre_DPscaled_LPCA_problem.b[i] = DPscaled_LPCA_problem.b[i]; // the same as DPscaled_LPCA_problem

        }
        for(int i=0; i<DPscaled_LPCA_problem.m; ++i)
        {
            Pre_DPscaled_LPCA_problem.A[i][i + DPscaled_LPCA_problem.n] = (DPscaled_LPCA_problem.b[i] > 0) ? 1 : -1; // sb = 2*(b > 0)-1; Ai = [A diag(sb)];
        }
        for(int i=0; i<DPscaled_LPCA_problem.n; ++i)
        {
            Pre_DPscaled_LPCA_problem.h[i] = DPscaled_LPCA_problem.h[i];
        }
        for(int i=0; i<DPscaled_LPCA_problem.m; ++i)
        {
            Pre_DPscaled_LPCA_problem.h[i+DPscaled_LPCA_problem.n] = 2*fabs(DPscaled_LPCA_problem.b[i]);
        }

        // Use Bounded Revised Simplex to find initial basic feasible point of original program
        auto result_init = simplxuprevsol_tiebreak(Pre_DPscaled_LPCA_problem);

        // Check that Feasible Solution was found
        if(result_init.iters>=Pre_DPscaled_LPCA_problem.itlim){
            err = -3;
            // std::cout << "Pre Too Many Iterations Finding Final Solution"<< std::endl;
            // for (int i = 0; i < ControlSize; ++i) {
            //     std::cout << this->generalizedMoment[i] << std::endl;
            // }
        }
        for(int i=0;i<ControlSize-1;++i){
            if(result_init.inB[i]> EffectorSize-1) // DPscaled_LPCA_problem is origin problem, k=DPscaled_LPCA_problem.n-1 = EffectorSize
            {
                // which mean inital basic index is out of the origin problem.
                err = -2;
                break;
            }
        }
        if(result_init.errout){
            err = -1;
        }
        // solve Pre_DPscaled_LPCA_problem but proccess DPscaled_LPCA_problem
        float xout[EffectorSize];
        for(int i=0;i<DPscaled_LPCA_problem.n;++i){
            xout[i]=0;
        }
        if(err!=0) // Construct an incorrect solution to accompany error flags
        {
            // use result_init data
            // matlab: indv = inB1<=(k+1); xout(inB1(indv)) = y1(indv); % in matlab the index from 1 to k, but cpp is 0 to k-1
            for(int i=0;i<ControlSize-1;++i){
                if(result_init.inB[i] < EffectorSize)
                {
                    xout[result_init.inB[i]]=result_init.y0[i];
                }
            }
            // xout(~e1(1:k+1)) = -xout(~e1(1:k+1))+h(~e1(1:k+1));
            for(int i=0;i<EffectorSize;++i){
                if(!result_init.e[i]){
                    xout[i] = -xout[i] + DPscaled_LPCA_problem.h[i];
                }
            }
        }
        else //No Error continue to solve problem
        {
            // Solve using initial problem from above
            // Construct solution to original LP problem from bounded simplex output
            // Set non-basic variables to 0 or h based on result_init.e
            // Set basic variables to result_init.y0 or h-result_init.y0.

            // update DPscaled_LPCA_problem.inB and DPscaled_LPCA_problem.e by result_init.inB and result_init.e[0 to k=EffectorSize] that is e1(1:k+1) in matlab.  k+1 at all, so int (i=0;i<EffectorSize+1;++i) or (int i=0;i<DPscaled_LPCA_problem.n;++i)
            for(int i=0;i<DPscaled_LPCA_problem.m;++i){
                DPscaled_LPCA_problem.inB[i]=result_init.inB[i];
            }
            for(int i=0;i<DPscaled_LPCA_problem.n;++i){
                DPscaled_LPCA_problem.e[i]=result_init.e[i];
            }

            auto result = simplxuprevsol_tiebreak(DPscaled_LPCA_problem);

            for(int i=0;i<ControlSize-1;++i){
                xout[result.inB[i]]=result.y0[i];
            }
            for(int i=0;i<EffectorSize;++i){
                if(!result.e[i]){
                    xout[i]=-xout[i]+DPscaled_LPCA_problem.h[i];
                }
            }

            if(result.iters>=DPscaled_LPCA_problem.itlim){
                err = 3;
            }
            if(result.errout)
            {
                err = 1;
            }
        }
        // Transform back to control variables
        for(int i=0;i<EffectorSize;++i){
            output[i]=xout[i]+this->aircraft.lowerLimits[i];
        }
        rho = calculateRho<ControlSize, EffectorSize>(ydt, output, Bt, DPscaled_LPCA_problem.tol);
        if(rho>1){
            for(int i=0;i<EffectorSize;++i){
                output[i]/=rho;
            }
        }
        return;
    }
    void DP_LPCA_copy(float input_higher[ControlSize], float input_lower[ControlSize], float output[EffectorSize], int& err, float & rho){
        // yd=input_lower
        // % Prioritizing Commands by DP_LPCA
        // % Direction Preserving Control Allocation Linear Program
        // % For the DP_LPCA_prio:
        // % function [u, errout,lambda] = DP_LPCA_prio(m_higher,m_lower,B,uMin,uMax,itlim)
        // % A.5 Building a Control Allocator for Feasible and Infeasible Solutions
        // %
        // % This DP_LPCA_copy:
        // %  Inputs:
        // %          input_higher [n]    = higher objective
        // %          input_lower [n]    = lower objective
        // %          B [n,m]   = Control Effectiveness matrix
        // %          uMin[m,1] = Lower bound for controls
        // %          uMax[m,1] = Upper bound for controls
        // %          itlim     = Number of allowed iterations limit
        // %                         (Sum of iterations in both branches)
        // %
        // % Outputs:
        // %         u[m,1]     = Control Solution
        // %         errout     = Error Status code
        // %                         0 = found solution
        // %                         <0 = Error in finding initial basic feasible solution
        // %                         >0 = Error in finding final solution
        // %                         -1,1 = Solver error (unbounded solution)
        // %                         -2   = Initial feasible solution not found
        // %                         -3,3 = Iteration limit exceeded
        // %         itlim      = Number of iterations remaining after solution found
        // %
        // % Calls:
        // %         simplxuprevsol = Bounded Revised Simplex solver (simplxuprevsol.m)
        // %
        // 4/2024    Meng ChaoHeng  Implement in cpp

        err = 0;
        rho = 0;

        // The DP_LPCA function describes the prioritized allocation problem
        // as a DP_LP problem and solves it using simplxuprevsol_tiebreak.
        //=======================
        // Figure out how big the problem is (use standard CA definitions for m & n)
        // but in here we use [m,k] = size(B) instead of [n,m] = size(B) in matlab. just for adapt to BoundedRevisedSimplex
        // we use [m,n] = size(A) in BoundedRevisedSimplex, that is, k + 1 = n.
        // Check to see if yd == 0
        // May want to adjust the tolerance to improve numerics of later steps
        bool flag=false;
        for(int i=0;i<ControlSize;++i){
            if(fabs(input_lower[i]) > DP_LPCA_problem.tol)
            {
                flag = true; // Check this condition
                break;
            }
        }
        if(!flag){
            for(int i=0;i<EffectorSize;++i){
                output[i]=0;
            }
            err=-1;
            return;
        }
        //=======================
        // We inital this problem in constructor.
        // Construct an LP using scaling parameter to enforce direction preserving
        // To find Feasible solution construct problem with appended slack variables
        // ref. is A.6.4 Initialization of the Simplex Algorithm of <Aircraft control allocation>

        // now we update the problem by aircraft.
        if(isupdate){
            Update();
            isupdate = false; // only update once.
        }
        // update A b h by input data.
        // update sb(since b is update) Ai bi hi every time
        for(int i=0; i<DP_LPCA_problem.m; ++i)
        {
            DP_LPCA_problem.A[i][DP_LPCA_problem.n-1] = -input_lower[i];
            DP_LPCA_problem.b[i] = input_higher[i]-this->aircraft.BuMin[i]; //

            Pre_DP_LPCA_problem.A[i][DP_LPCA_problem.n-1] = -input_lower[i]; // the same as DP_LPCA_problem.A[i][DP_LPCA_problem.n-1]
            Pre_DP_LPCA_problem.b[i] = DP_LPCA_problem.b[i]; // the same as DP_LPCA_problem

            Pre_DP_LPCA_problem.A[i][i + DP_LPCA_problem.n] = (DP_LPCA_problem.b[i] > 0) ? 1 : -1; // sb = 2*(b > 0)-1; Ai = [A diag(sb)];
            Pre_DP_LPCA_problem.h[i+DP_LPCA_problem.n] = 2*fabs(DP_LPCA_problem.b[i]);
        }

        // Use Bounded Revised Simplex to find initial basic feasible point of original program
        auto result_init = simplxuprevsol_tiebreak(Pre_DP_LPCA_problem);

        // Check that Feasible Solution was found
        if(result_init.iters>=Pre_DP_LPCA_problem.itlim){
            err = -3;
        }
        for(int i=0;i<ControlSize;++i){
            if(result_init.inB[i]> EffectorSize) // DP_LPCA_problem is origin problem, k=DP_LPCA_problem.n-1 = EffectorSize
            {
                // which mean inital basic index is out of the origin problem.
                err = -2;
                break;
            }
        }
        if(result_init.errout){
            err = -1;
        }
        // solve Pre_DP_LPCA_problem but proccess DP_LPCA_problem
        float xout[DP_LPCA_problem.n];
        for(int i=0;i<DP_LPCA_problem.n;++i){
            xout[i]=0;
        }
        if(err!=0) // Construct an incorrect solution to accompany error flags
        {
            // use result_init data
            // matlab: indv = inB1<=(k+1); xout(inB1(indv)) = y1(indv); % in matlab the index from 1 to k, but cpp is 0 to k-1
            for(int i=0;i<ControlSize;++i){
                if(result_init.inB[i] <= EffectorSize)
                {
                    xout[result_init.inB[i]]=result_init.y0[i];
                }
            }
            // xout(~e1(1:k+1)) = -xout(~e1(1:k+1))+h(~e1(1:k+1));
            for(int i=0;i<DP_LPCA_problem.n;++i){
                if(!result_init.e[i]){
                    xout[i] = -xout[i] + DP_LPCA_problem.h[i];
                }
            }
        }
        else //No Error continue to solve problem
        {
            // Solve using initial problem from above
            // Construct solution to original LP problem from bounded simplex output
            // Set non-basic variables to 0 or h based on result_init.e
            // Set basic variables to result_init.y0 or h-result_init.y0.

            // update DP_LPCA_problem.inB and DP_LPCA_problem.e by result_init.inB and result_init.e[0 to k=EffectorSize] that is e1(1:k+1) in matlab.  k+1 at all, so int (i=0;i<EffectorSize+1;++i) or (int i=0;i<DP_LPCA_problem.n;++i)
            for(int i=0;i<ControlSize;++i){
                DP_LPCA_problem.inB[i]=result_init.inB[i];
            }
            for(int i=0;i<DP_LPCA_problem.n;++i){
                DP_LPCA_problem.e[i]=result_init.e[i];
            }
            auto result = simplxuprevsol_tiebreak(DP_LPCA_problem);

            for(int i=0;i<ControlSize;++i){
                xout[result.inB[i]]=result.y0[i];
            }
            for(int i=0;i<DP_LPCA_problem.n;++i){
                if(!result.e[i]){
                    xout[i]=-xout[i]+DP_LPCA_problem.h[i];
                }
            }

            if(result.iters>=DP_LPCA_problem.itlim){
                err = 3;
            }
            if(result.errout)
            {
                err = 1;
            }
        }
        // Transform back to control variables
        for(int i=0;i<EffectorSize;++i){
            output[i]=xout[i]+this->aircraft.lowerLimits[i];
        }
        rho = xout[EffectorSize];
        return;
    }
    void DP_LPCA_prio(float input_higher[ControlSize], float input_lower[ControlSize], float output[EffectorSize], int& err, float & rho){
        // C++ counterpart of PCA/DP_LPCA_prio.m.
        //
        // DP_LPCA_copy is the lower-level prioritized LP.  This wrapper keeps
        // the public call structure aligned with MATLAB: first try to allocate
        // the lower-priority command around input_higher; if initialization
        // fails, fall back to allocating input_higher alone.
        DP_LPCA_copy(input_higher, input_lower, output, err, rho);
        if(err < 0)
        {
            float zero_higher[ControlSize];
            for(int i=0; i<ControlSize; ++i)
            {
                zero_higher[i] = 0.0f;
            }
            DP_LPCA_copy(zero_higher, input_higher, output, err, rho);
        }
    }
    void restoring(float u[EffectorSize], float u_rest[EffectorSize]){
        const float restoring_tol = 1e-5f;
        Vector<float, EffectorSize> u_current(u);
        bool all_small = true;
        for(int i=0;i<EffectorSize;++i){
            if(fabs(u[i]) >= restoring_tol){
                all_small = false;
                break;
            }
        }
        if(all_small){
            for(int i=0;i<EffectorSize;++i){
                u_rest[i]=u[i];
            }
            return;
        }

        // Mirror the maintained MATLAB restoring_cpp.m projection form.  That
        // MATLAB version was first checked against the original restoring
        // variants, then this C++ implementation followed it because the target
        // should not need null(B), SVD, or pinv([B;u']).
        //
        // For full-row-rank B,
        //   u - B' * ((B*B') \ (B*u))
        // equals N*(N'*u), the component of u in null(B).  This avoids
        // directly inverting the often ill-conditioned augmented matrix
        // [B;u'] and avoids needing an SVD/null-space routine on target.
        Vector<float, ControlSize> achieved = B * u_current;
        SquareMatrix<float, ControlSize> B_B_t = B * B.transpose();
        Vector<float, ControlSize> row_solution;
        if(!solveSquareLU(B_B_t, achieved, row_solution)){
            for(int i=0;i<EffectorSize;++i){
                u_rest[i]=u[i];
            }
            return;
        }
        Vector<float, EffectorSize> u_pseudo = B.transpose() * row_solution;
        Vector<float, EffectorSize> null_component = u_current - u_pseudo;
        const float null_component_norm_squared = null_component.norm_squared();
        if(null_component_norm_squared < restoring_tol * restoring_tol){
            for(int i=0;i<EffectorSize;++i){
                u_rest[i]=u[i];
            }
            return;
        }
        Vector<float, EffectorSize> u_null;
        for(int i=0;i<EffectorSize;++i){
            u_null(i)=a_constant * null_component(i) / null_component_norm_squared;
        }

        float K_opt=-a_constant/u_null.norm_squared();
        //% update limits
        float uMax_new[EffectorSize]; // Control vector upper limit variables
        float uMin_new[EffectorSize]; // Control vector lower limit variables
        for(int i=0;i<EffectorSize;++i){
            uMax_new[i]=this->aircraft.upperLimits[i]-u[i];
            uMin_new[i]=this->aircraft.lowerLimits[i]-u[i];
        }
        float K_max=FLT_MAX; // 1.0/FLT_EPSILON; or FLT_MAX
        for(int i=0;i<EffectorSize;++i){
            if(fabs(u_null(i))<restoring_tol){
                continue; // if u_null(i) is zero, then skip;
            }
            float tmpu=0.0f;
            if(u_null(i)>0){
                tmpu=uMax_new[i]/u_null(i);
            }else{
                tmpu=uMin_new[i]/u_null(i);
            }
            if(tmpu<K_max){ // find smaller
                K_max=tmpu;
            }
        }
        for(int i=0;i<EffectorSize;++i){
            u_rest[i]=u[i] + matrix::typeFunction::min(K_max,K_opt) *u_null(i);
        }
    }
    void Update(){
        //for DP_LPCA and DP_LPCA_copy.   A b h

        // B*uMin of aircraft， b
        for (int i = 0; i < ControlSize; ++i) {
            float temp = 0.0f;
            for (int j = 0; j < EffectorSize; ++j) {
                temp +=  this->aircraft.controlEffectMatrix[i][j]*this->aircraft.lowerLimits[j];
            }
            this->aircraft.BuMin[i] = temp; // Calculate BuMin
            DP_LPCA_problem.b[i]=-temp; // Temporarily. Needs recalculation

        }

        // A  h  (c is fixed)
        for(int i=0; i<DP_LPCA_problem.m; ++i)
        {
            for(int j=0; j<DP_LPCA_problem.n-1; ++j)
            {
                DP_LPCA_problem.A[i][j] = this->aircraft.controlEffectMatrix[i][j]; // [n-1] will update every time

                Pre_DP_LPCA_problem.A[i][j] = this->aircraft.controlEffectMatrix[i][j]; // the same as DP_LPCA_problem.A[i][j]
            }
            Pre_DP_LPCA_problem.b[i] = DP_LPCA_problem.b[i];

            Pre_DP_LPCA_problem.A[i][i + DP_LPCA_problem.n] = (DP_LPCA_problem.b[i] > 0) ? 1 : -1; // sb = 2*(b > 0)-1; Ai = [A diag(sb)];
            Pre_DP_LPCA_problem.h[i+DP_LPCA_problem.n] = 2*fabs(DP_LPCA_problem.b[i]);
        }
        for(int i=0; i<DP_LPCA_problem.n-1; ++i)
        {
            DP_LPCA_problem.h[i] = this->aircraft.upperLimits[i]-this->aircraft.lowerLimits[i];  //[n] is fixed 1
            Pre_DP_LPCA_problem.h[i] = DP_LPCA_problem.h[i];
        }
        // for DPscaled_LPCA is update every time
        //   for restoring
        B.setZero();
        for (int i = 0; i < ControlSize; ++i) {
            for (int j = 0; j < EffectorSize; ++j) {
                B(i,j) = this->aircraft.controlEffectMatrix[i][j];
            }
        }

        // std::cout << "is updated" << std::endl;
        isupdate = false;
    }
    // Other member functions and member variables definitions
    float generalizedMoment[ControlSize]; // Set in constructor
    // Linear programming related
    LinearProgrammingProblem<ControlSize, EffectorSize+1> DP_LPCA_problem;// Pre-set initial by aircraft data
    LinearProgrammingProblem<ControlSize, (EffectorSize+1) + ControlSize> Pre_DP_LPCA_problem;// Pre-set initial by aircraft data
    LinearProgrammingProblem<ControlSize-1, EffectorSize> DPscaled_LPCA_problem;// Pre-set initial by aircraft data
    LinearProgrammingProblem<ControlSize-1, EffectorSize + (ControlSize-1)> Pre_DPscaled_LPCA_problem;// Pre-set initial by aircraft data
    float upper_lam=1; // 2024-10-18 upper_lam=1
    // for restoring
    float a_constant=-2; // arbitrary a<0; matches restoring_cpp.m step direction.
    matrix::Matrix<float, ControlSize, EffectorSize> B;
    bool isupdate{false}; //if update aircraft data, then set isupdate = true.

};
// and user can define more...
// template <int ControlSize, int EffectSize>
// class XX_ControlAllocator : public ControlAllocatorBase<ControlSize, EffectSize> {}
