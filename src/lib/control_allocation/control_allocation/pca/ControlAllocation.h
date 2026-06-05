
#include <matrix/math.hpp>
#include <iostream>
#include <stdlib.h>
#include <limits>
#include <cmath>
#include <cfloat>
using namespace matrix;
#if !defined(FLT_MAX)
#define FLT_MAX     __FLT_MAX__
#endif

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

inline bool lp_is_finite(float value)
{
    return value < INFINITY && value > -INFINITY;
}

template<size_t K>
inline void select_entering_original(const Matrix<float, 1, K> &rdt,
                                     float &minr, size_t &qind)
{
    minr = rdt(0, 0);
    qind = 0;
    for (size_t i = 1; i < K; ++i) {
        if (rdt(0, i) < minr) {
            minr = rdt(0, i);
            qind = i;
        }
    }
}

template<size_t K>
inline bool select_bland_negative_original(const Matrix<float, 1, K> &rdt,
                                           size_t &qind)
{
    for (size_t i = 0; i < K; ++i) {
        if (rdt(0, i) < 0.0f) {
            qind = i;
            return true;
        }
    }
    return false;
}

template<size_t M>
inline void select_leaving_original(const Vector<float, M> &rat,
                                    float &minrat, size_t &p)
{
    minrat = rat(0);
    p = 0;
    for (size_t i = 1; i < M; ++i) {
        if (rat(i) < minrat) {
            minrat = rat(i);
            p = i;
        }
    }
}

template<typename Type, size_t M>
inline bool solveSquareLU(Matrix<Type, M, M> A, Vector<Type, M> b, Vector<Type, M> &x)
{
    // Dense square solve using LU with partial row pivoting.  This is used
    // only for A(:,inB)\b style basis solves; simplex pivot selection follows
    // MATLAB simplxuprevsol.m.
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

template<typename Type, size_t M>
inline bool solveBasis(const Matrix<Type, M, M> &A, const Vector<Type, M> &b, Vector<Type, M> &x)
{
#if defined(CA_SIMPLEX_USE_LS_SOLVE)
    // This is the 408fda5 simplex basis-solve path.  Keep it as the default
    // only when explicitly auditing old 3x4 behavior.
    matrix::LeastSquaresSolver<Type, M, M> solver(A);
    x = solver.solve(b);
    return true;
#else
    // LU with partial pivoting is the default generic square solve.  In the
    // current df4 tests it does not disturb the 3x4 split instance, and it
    // avoids large DPscaled errors on the 4x5 single-instance case.
    return solveSquareLU(A, b, x);
#endif
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
        std::cerr << "Error: Division by zero." << std::endl;
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
    float tol=1.0e-10f; // Match MATLAB simplxuprevsol.m tolerance.
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


// Bounded revised simplex aligned with MATLAB simplxuprevsol.m.
//
// It intentionally uses the same pivot choices as the book/MATLAB reference:
// min(rdt), first negative reduced cost in the Bland branch, and min(rat).
// No deterministic near-tie rule or feasibility repair is applied here.
template<int M, int N>
LinearProgrammingResult<M, N> simplxuprevsol(LinearProgrammingProblem<M, N> problem) {
    LinearProgrammingResult<M, N> result;

    const int n_m = N - M;
    int ind_all[N];
    for (int num = 0, index = 0; num < N; ++num, ++index) {
        ind_all[index] = num;
    }

    // Partition A into current basic and non-basic variables.
    setdiff(ind_all, N, problem.inB, M, problem.inD);

    // Variables initialized at their upper bound are represented by sign flips.
    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < N; ++j) {
            if (!problem.e[j]) {
                problem.A[i][j] *= -1;
                problem.b[i] += problem.A[i][j] * problem.h[j];
            }
        }
    }
    for (int j = 0; j < N; ++j) {
        if (!problem.e[j]) {
            problem.c[j] *= -1;
        }
    }

    matrix::SquareMatrix<float, M> A_inB;
    matrix::Matrix<float, M, n_m> A_inD;
    matrix::Vector<float, M> c_inB;
    matrix::Vector<float, n_m> c_inD;

    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < M; ++j) {
            A_inB(i, j) = problem.A[i][problem.inB[j]];
        }
        c_inB(i) = problem.c[problem.inB[i]];
    }
    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < n_m; ++j) {
            A_inD(i, j) = problem.A[i][problem.inD[j]];
        }
    }
    for (int i = 0; i < n_m; ++i) {
        c_inD(i) = problem.c[problem.inD[i]];
    }

    matrix::Vector<float, M> b_vec(problem.b);
    Matrix<float, 1, M> lamt;
    Matrix<float, 1, n_m> rdt;
    matrix::Vector<float, M> A_qel;
    matrix::Vector<float, M> yq;
    matrix::Vector<float, M> rat;

    matrix::Vector<float, M> y0;
    solveBasis(A_inB, b_vec, y0);

    bool done = false;
    bool unbounded = false;
    int iters = 0;

    while ((!done) && (!unbounded) && (iters < problem.itlim)) {
        iters++;

        // Relative cost: lamt = c_B' / A_B, rdt = c_D' - lamt*A_D.
        matrix::Vector<float, M> lamt_col;
        solveBasis(A_inB.transpose(), c_inB, lamt_col);
        lamt = lamt_col.transpose();
        rdt = c_inD.transpose() - lamt * A_inD;

        float minr;
        size_t qind;
        select_entering_original<n_m>(rdt, minr, qind);

        // MATLAB simplxuprevsol.m uses minr >= 0 exactly.
        if (minr >= 0.0f) {
            done = true;
            break;
        }

        int qel = problem.inD[qind];
        for (int i = 0; i < M; ++i) {
            A_qel(i) = problem.A[i][qel];
        }
        solveBasis(A_inB, A_qel, yq);

        bool has_pivot = false;
        for (int i = 0; i < M; ++i) {
            if (fabs(yq(i)) > problem.tol) {
                has_pivot = true;
                break;
            }
        }
        if (!has_pivot) {
            unbounded = true;
            break;
        }

        // Ratio test copied from the original bounded revised simplex.
        float hinB[M];
        for (int i = 0; i < M; ++i) {
            hinB[i] = problem.h[problem.inB[i]];
            if (fabs(yq(i)) > problem.tol) {
                rat(i) = y0(i) / yq(i);
                if (yq(i) < 0.0f) {
                    rat(i) -= hinB[i] / yq(i);
                }
            } else {
                rat(i) = INFINITY;
            }
        }

        float minrat;
        size_t p;
        select_leaving_original<M>(rat, minrat, p);

        // Keep the original zero-step Bland branch and original tolerance.
        if (fabs(minrat) <= problem.tol) {
            select_bland_negative_original<n_m>(rdt, qind);
            qel = problem.inD[qind];
            for (int i = 0; i < M; ++i) {
                A_qel(i) = problem.A[i][qel];
            }
            solveBasis(A_inB, A_qel, yq);

            bool has_pivot1 = false;
            for (int i = 0; i < M; ++i) {
                if (fabs(yq(i)) > problem.tol) {
                    has_pivot1 = true;
                    break;
                }
            }
            if (!has_pivot1) {
                unbounded = true;
                break;
            }

            float hinB1[M];
            for (int i = 0; i < M; ++i) {
                hinB1[i] = problem.h[problem.inB[i]];
                if (fabs(yq(i)) > problem.tol) {
                    rat(i) = y0(i) / yq(i);
                    if (yq(i) < 0.0f) {
                        rat(i) -= hinB1[i] / yq(i);
                    }
                } else {
                    rat(i) = INFINITY;
                }
            }
            select_leaving_original<M>(rat, minrat, p);
        }

        // Pivot/update logic copied from the original path.
        if (minrat >= problem.h[qel]) {
            problem.e[qel] = !problem.e[qel];
            for (int i = 0; i < M; ++i) {
                problem.A[i][qel] *= -1;
                b_vec(i) += problem.A[i][qel] * problem.h[qel];
                problem.b[i] = b_vec(i);
            }
            problem.c[qel] *= -1;
            for (int i = 0; i < M; ++i) {
                A_inD(i, qind) = problem.A[i][qel];
            }
            c_inD(qind) = problem.c[qel];
        } else if (yq(p) > 0.0f) {
            int pel = problem.inB[p];
            problem.inB[p] = qel;
            problem.inD[qind] = pel;
            for (int i = 0; i < M; ++i) {
                A_inB(i, p) = problem.A[i][qel];
                A_inD(i, qind) = problem.A[i][pel];
            }
            c_inB(p) = problem.c[qel];
            c_inD(qind) = problem.c[pel];
        } else {
            int pel = problem.inB[p];
            problem.e[pel] = !problem.e[pel];
            for (int i = 0; i < M; ++i) {
                problem.A[i][pel] *= -1;
                b_vec(i) += problem.A[i][pel] * problem.h[pel];
                problem.b[i] = b_vec(i);
            }
            problem.inB[p] = qel;
            problem.inD[qind] = pel;
            problem.c[pel] *= -1;
            for (int i = 0; i < M; ++i) {
                A_inB(i, p) = problem.A[i][qel];
                A_inD(i, qind) = problem.A[i][pel];
            }
            c_inB(p) = problem.c[qel];
            c_inD(qind) = problem.c[pel];
        }

        solveBasis(A_inB, b_vec, y0);
    }

    result.errout = unbounded;
    for (int i = 0; i < M; ++i) {
        result.y0[i] = y0(i);
        result.inB[i] = problem.inB[i];
    }
    for (int i = 0; i < N; ++i) {
        result.e[i] = problem.e[i];
    }
    result.iters = iters;
    return result;
}

template<int M, int N>
LinearProgrammingResult<M, N> BoundedRevisedSimplex(LinearProgrammingProblem<M, N> problem) {
    return simplxuprevsol(problem);
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
        //         simplxuprevsol = Bounded Revised Simplex solver
        //         aligned with MATLAB simplxuprevsol.m.

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
        // problem as a DP_LP problem and solves it using simplxuprevsol.
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
        auto result_init = simplxuprevsol(Pre_DP_LPCA_problem);

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
            auto result = simplxuprevsol(DP_LPCA_problem);

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
        //         simplxuprevsol = Bounded Revised Simplex solver
        //         aligned with MATLAB simplxuprevsol.m.

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
        // DP_LP problem and solves it using simplxuprevsol.
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
        auto result_init = simplxuprevsol(Pre_DPscaled_LPCA_problem);

        auto cleanup_zero_artificial_basis = [&]() {
            const float cleanup_tol = Pre_DPscaled_LPCA_problem.tol;
            bool used_original[EffectorSize];
            for (int i = 0; i < EffectorSize; ++i) {
                used_original[i] = false;
            }

            for (int i = 0; i < ControlSize - 1; ++i) {
                if (result_init.inB[i] < EffectorSize) {
                    used_original[result_init.inB[i]] = true;
                }
            }

            for (int basis_row = 0; basis_row < ControlSize - 1; ++basis_row) {
                if (result_init.inB[basis_row] < EffectorSize) {
                    continue;
                }

                if (fabs(result_init.y0[basis_row]) > cleanup_tol) {
                    return false;
                }

                bool replaced = false;

                for (int candidate = 0; candidate < EffectorSize; ++candidate) {
                    if (used_original[candidate]) {
                        continue;
                    }

                    matrix::SquareMatrix<float, ControlSize - 1> A_trial;

                    for (int r = 0; r < ControlSize - 1; ++r) {
                        for (int c = 0; c < ControlSize - 1; ++c) {
                            const int variable = (c == basis_row) ? candidate : result_init.inB[c];
                            float value = DPscaled_LPCA_problem.A[r][variable];

                            if (!result_init.e[variable]) {
                                value = -value;
                            }

                            A_trial(r, c) = value;
                        }
                    }

                    matrix::Vector<float, ControlSize - 1> zero_b;
                    matrix::Vector<float, ControlSize - 1> zero_x;
                    zero_b.setZero();

                    if (solveSquareLU(A_trial, zero_b, zero_x)) {
                        result_init.inB[basis_row] = candidate;
                        result_init.y0[basis_row] = 0.0f;
                        used_original[candidate] = true;
                        replaced = true;
                        break;
                    }
                }

                if (!replaced) {
                    return false;
                }
            }

            return true;
        };

        const bool cleaned_init_basis = cleanup_zero_artificial_basis();

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
        if(!cleaned_init_basis){
            err = -2;
        }
        if(result_init.errout && !cleaned_init_basis){
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

            auto result = simplxuprevsol(DPscaled_LPCA_problem);

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
        // as a DP_LP problem and solves it using simplxuprevsol.
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
        auto result_init = simplxuprevsol(Pre_DP_LPCA_problem);

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
            auto result = simplxuprevsol(DP_LPCA_problem);

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
