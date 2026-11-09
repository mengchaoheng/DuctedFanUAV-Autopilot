#include <cmath>
#include <cstring>
#include "OmQpSolver.hpp"
#include "float_prefix.h"
#include "OmQpArena.h"
static_assert(sizeof(real_t) == 4, "float32 backend only");
extern "C" { int om_float_om_structured_c = 0; }

namespace
{
qpData_t qp{};
bool initialized = false;
bool have_solution = false;
int horizon = 0;
float saved_q[9], saved_p[9], saved_r[4];
float previous_lambda[9 * OmMaxHorizon];
bool predictor_ready = false;
}

void om_float_close()
{
	om_float_om_structured_c = 0;
	predictor_ready = false;
	om_qp_arena_close();
	qp = {};
	initialized = false;
	have_solution = false;
	horizon = 0;
}

bool om_float_update(const OmFloatProblem &p, float tolerance, OmQpLayout layout)
{
	auto reject = []() {
		om_float_close();
		return false;
	};

	if (p.horizon < 1 || p.horizon > OmMaxHorizon || !std::isfinite(tolerance) || tolerance <= 0.f || tolerance > 1e-3f) {
		return reject();
	}

	for (int i = 0; i < 9; i++)
		if (!std::isfinite(p.x0[i]) || !std::isfinite(p.q[i]) || !std::isfinite(p.p[i]) || p.q[i] <= 0.f || p.p[i] <= 0.f) {
			return reject();
		}

	for (int j = 0; j < 4; j++)
		if (!std::isfinite(p.r[j]) || p.r[j] <= 0.f || !std::isfinite(p.reference_input[j])) {
			return reject();
		}

	for (int k = 0; k < p.horizon; k++) {
		for (float v : p.a[k])
			if (!std::isfinite(v)) {
				return reject();
			}

		for (float v : p.b[k])
			if (!std::isfinite(v)) {
				return reject();
			}

		for (float v : p.c[k])
			if (!std::isfinite(v)) {
				return reject();
			}

		for (int j = 0; j < 4; j++)
			if (!std::isfinite(p.lower[k][j]) || !std::isfinite(p.upper[k][j]) || p.lower[k][j] > p.upper[k][j]) {
				return reject();
			}
	}

	const bool costs_changed = memcmp(saved_q, p.q, sizeof(p.q)) || memcmp(saved_p, p.p, sizeof(p.p)) || memcmp(saved_r, p.r, sizeof(p.r));
	const bool cold = !initialized || p.horizon != horizon || costs_changed;

	if (cold) {
		om_float_close();

		if (!om_qp_arena_open(om_arena_capacity(p.horizon))) {
			return reject();
		}

		auto opts = qpDUNES_setupDefaultOptions();
		opts.maxIter = 100;
		opts.maxNumLineSearchIterations = 20;
		opts.maxNumLineSearchRefinementIterations = 20;
		opts.stationarityTolerance = tolerance;
		opts.logLevel = QPDUNES_LOG_OFF;
		opts.printLevel = 0;

		if (om_qp_setup_horizon_checked(&qp, &opts, p.horizon) != QPDUNES_OK) {
			return reject();
		}
	}

	qp.options.lsType = QPDUNES_LS_ACCELERATED_GRADIENT_BISECTION_LS;
	// Cold and resumed workspaces use the same float32 stopping criterion.
	// Tightening it after a rejection can perpetuate roundoff-limited solves.
	qp.options.stationarityTolerance = tolerance;
	float q[81], r[16], terminal[81], lower_x[9], upper_x[9];

	for (int j = 0; j < 9; j++) {
		lower_x[j] = -qp.options.QPDUNES_INFTY;
		upper_x[j] = qp.options.QPDUNES_INFTY;
	}

	if (cold) {
		memset(q, 0, sizeof(q));
		memset(r, 0, sizeof(r));
		memset(terminal, 0, sizeof(terminal));

		for (int j = 0; j < 9; j++) {
			q[j * 9 + j] = p.q[j];
			terminal[j * 9 + j] = p.p[j];
		}

		for (int j = 0; j < 4; j++) {
			r[j * 4 + j] = p.r[j];
		}
	}

	for (int k = 0; k < p.horizon; k++) {
		return_t status;

		if (cold) {
			status = qpDUNES_setupSimpleBoundedInterval(&qp, qp.intervals[k], q, r, nullptr, p.a[k], p.b[k], p.c[k], k ? lower_x : p.x0,
				k ? upper_x : p.x0, p.lower[k], p.upper[k]);

		} else {
			// Costs, dimensions and storage layout were checked above. Write the
			// dense [A B] and vectors directly into the existing stage workspace,
			// avoiding a temporary matrix and a second full matrix copy.
			auto *stage = qp.intervals[k];

			if (stage->C.sparsityType != QPDUNES_DENSE || stage->nV != 13) {
				return reject();
			}

			for (int i = 0; i < 9; i++) {
				memcpy(stage->C.data + i * 13, p.a[k] + i * 9, 9 * sizeof(float));
				memcpy(stage->C.data + i * 13 + 9, p.b[k] + i * 4, 4 * sizeof(float));
				stage->zLow.data[i] = k ? lower_x[i] : p.x0[i];
				stage->zUpp.data[i] = k ? upper_x[i] : p.x0[i];
			}

			memcpy(stage->c.data, p.c[k], 9 * sizeof(float));
			memcpy(stage->zLow.data + 9, p.lower[k], 4 * sizeof(float));
			memcpy(stage->zUpp.data + 9, p.upper[k], 4 * sizeof(float));
			status = QPDUNES_OK;
		}

		if (status != QPDUNES_OK) {
			return reject();
		}
	}

	if (cold && qpDUNES_setupSimpleBoundedInterval(&qp, qp.intervals[p.horizon], terminal, nullptr, nullptr, nullptr, nullptr, nullptr, lower_x,
			upper_x, nullptr, nullptr) != QPDUNES_OK) {
		return reject();
	}

	// Predict only from consecutive successful QP solves. This is an initial
	// dual guess; qpDUNES still performs all feasibility/convergence checks.

	if (!cold && have_solution) {
		for (int i = 0; i < p.horizon * 9; i++) {
			const float current = qp.lambda.data[i];

			if (predictor_ready) {
				qp.lambda.data[i] = current + (current - previous_lambda[i]);
			}

			previous_lambda[i] = current;
		}

		predictor_ready = true;

	} else {
		predictor_ready = false;
	}

	om_float_om_structured_c = layout == OmQpLayout::OmErrorState;
	qpDUNES_indicateDataChange(&qp);

	if (cold) {
		if (qpDUNES_setupAllLocalQPs(&qp, QPDUNES_FALSE) != QPDUNES_OK) {
			return reject();
		}

	} else {
		// Only A/B/c and bounds changed. The diagonal stage costs are unchanged;
		// setupStageQP refreshes lambda and local first-order data without refactorH.
		for (int k = 0; k <= p.horizon; k++)
			if (qpDUNES_setupStageQP(&qp, qp.intervals[k], QPDUNES_FALSE) != QPDUNES_OK) {
				return reject();
			}
	}

	initialized = true;
	horizon = p.horizon;
	memcpy(saved_q, p.q, sizeof(p.q));
	memcpy(saved_p, p.p, sizeof(p.p));
	memcpy(saved_r, p.r, sizeof(p.r));
	return true;
}

OmFloatResult om_float_solve(const OmFloatProblem &p, int iteration_limit)
{
	OmFloatResult result{};
	result.status = 7;
	result.finite = false;

	if (!initialized || horizon != p.horizon || iteration_limit < 1 || iteration_limit > 100) {
		return result;
	}

	qp.options.maxIter = iteration_limit;
	result.status = static_cast<int>(qpDUNES_solve(&qp));
	result.iterations = qp.log.numIter;

	// Both extrapolated and ordinary warm seeds can encounter a float32
	// line-search loss of ascent at an active-set change. Retry from the last
	// converged seed with gradient bisection, sharing the original iteration
	// and deadline budgets. Never accept an unconverged or infeasible result.
	if (result.status == QPDUNES_ERR_NEWTON_SYSTEM_NO_ASCENT_DIRECTION && have_solution
	    && result.iterations < iteration_limit && !om_qp_deadline_expired()) {
		result.recovery_attempted = true;
		const int spent = result.iterations;
		memcpy(qp.lambda.data, previous_lambda, horizon * 9 * sizeof(float));
		predictor_ready = false;
		qpDUNES_indicateDataChange(&qp);
		bool refreshed = true;

		for (int k = 0; k <= horizon; k++) {
			refreshed &= qpDUNES_setupStageQP(&qp, qp.intervals[k], QPDUNES_FALSE) == QPDUNES_OK;
		}

		if (refreshed) {
			qp.options.maxIter = iteration_limit - spent;
			qp.options.lsType = QPDUNES_LS_GRADIENT_BISECTION_LS;
			result.status = static_cast<int>(qpDUNES_solve(&qp));
			result.iterations = spent + qp.log.numIter;
		}
	}

	// qpDUNES tests convergence at the start of each Newton iteration. A
	// final allowed step can already converge, yet return the iteration cap.
	// Recheck the identical gradient tolerance without performing another step.
	if (result.status == QPDUNES_ERR_ITERATION_LIMIT_REACHED && !om_qp_deadline_expired()) {
		qpDUNES_computeNewtonGradient(&qp, &qp.gradient, &qp.xVecTmp);

		if (vectorNorm(&qp.gradient, horizon * 9) < qp.options.stationarityTolerance) {
			result.status = QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND;
		}
	}

	result.arena_used = om_qp_arena_used();
	result.finite = true;

	for (int k = 0; k <= horizon; k++) {
		const float *z = qp.intervals[k]->z.data;

		for (int j = 0; j < 9; j++) {
			result.finite &= std::isfinite(z[j]);

			if (k == 0) {
				result.primal_residual = fmaxf(result.primal_residual, fabsf(z[j] - p.x0[j]));
			}
		}

		if (k == horizon) {
			break;
		}

		for (int j = 0; j < 4; j++) {
			const float u = z[9 + j];

			if (k == 0) {
				result.inputs[j] = u;
			}

			result.finite &= std::isfinite(u);
			result.primal_residual = fmaxf(result.primal_residual, fmaxf(p.lower[k][j] - u, u - p.upper[k][j]));
		}

		for (int i = 0; i < 9; i++) {
			float next = p.c[k][i];

			for (int j = 0; j < 9; j++) {
				next += p.a[k][i * 9 + j] * z[j];
			}

			for (int j = 0; j < 4; j++) {
				next += p.b[k][i * 4 + j] * z[9 + j];
			}

			result.primal_residual = fmaxf(result.primal_residual, fabsf(next - qp.intervals[k + 1]->z.data[i]));
		}
	}

	bool dual_finite = true;

	for (int j = 0; j < 9 * horizon; j++) {
		dual_finite &= std::isfinite(qp.lambda.data[j]) && std::isfinite(qp.gradient.data[j]) && std::isfinite(qp.deltaLambda.data[j]);
	}

	result.resumable = result.status == QPDUNES_ERR_ITERATION_LIMIT_REACHED && result.finite && std::isfinite(result.primal_residual)
			   && dual_finite;
	have_solution = result.status == 1 && result.finite && result.primal_residual <= 2e-5f;
	result.recovery_succeeded = result.recovery_attempted && have_solution;
	return result;
}
