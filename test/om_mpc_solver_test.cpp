// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
#include "OmQpSolver.hpp"
#include "float_prefix.h"
#include "OmQpArena.h"
#include "OmMpcBenchCase.hpp"
#include <cassert>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <limits>
#include <initializer_list>

static bool expired;
extern "C" int om_float_om_qp_deadline_expired(void) { return expired; }

static OmFloatProblem problem(int n)
{
	OmFloatProblem p{};
	p.horizon = n;
	p.dt = .04f;
	for (int i = 0; i < 9; ++i) { p.q[i] = p.p[i] = i < 3 ? 550.f : i < 6 ? 100.f : 500.f; }
	for (int j = 0; j < 4; ++j) { p.r[j] = j ? 350.f : .25f; }
	p.x0[0] = .1f;
	p.x0[2] = -.1f;
	p.x0[7] = .02f;
	for (int k = 0; k < n; ++k) {
		for (int i = 0; i < 9; ++i) { p.a[k][i * 9 + i] = 1.f; }
		for (int i = 0; i < 3; ++i) {
			p.a[k][i * 9 + i + 3] = p.dt;
			p.b[k][(i + 6) * 4 + i + 1] = p.dt;
		}
		p.a[k][3 * 9 + 7] = -.392266f;
		p.a[k][4 * 9 + 6] = .392266f;
		p.b[k][5 * 4] = -p.dt;
		for (int j = 0; j < 4; ++j) { p.lower[k][j] = -10.f; p.upper[k][j] = 10.f; }
	}
	return p;
}

int main()
{
	auto options = qpDUNES_setupDefaultOptions();
	assert(std::isnormal(options.QPDUNES_ZERO));
	assert(!om_float_divisor_valid(0.f, 0.f));
	assert(!om_float_divisor_valid(0.f, 1.f));
	assert(!om_float_divisor_valid(NAN, 1.f));
	assert(!om_float_divisor_valid(INFINITY, 1.f));
	assert(!om_float_divisor_valid(1.f, INFINITY));
	assert(!om_float_divisor_valid(1.f, NAN));
	assert(!om_float_divisor_valid(std::numeric_limits<float>::denorm_min(), 0.f));
	assert(om_float_divisor_valid(.25f, .01f));
	assert(om_float_divisor_valid(-.25f, -.01f));
	assert(!om_float_divisor_valid(FLT_MIN, FLT_MAX));
	// Check the optimized IEEE classification against the standard operation,
	// including arbitrary signs, subnormals, infinities and NaN payloads.
	uint32_t bits = UINT32_C(0x12345678);
	for (int i = 0; i < 100000; ++i) {
		bits ^= bits << 13; bits ^= bits >> 17; bits ^= bits << 5;
		float divisor;
		memcpy(&divisor, &bits, sizeof(divisor));
		bits ^= bits << 13; bits ^= bits >> 17; bits ^= bits << 5;
		float value;
		memcpy(&value, &bits, sizeof(value));
		const bool expected = std::isnormal(divisor) && fabsf(divisor) >= FLT_MIN * fabsf(value);
		assert(bool(om_float_divisor_valid(divisor, value)) == expected);
	}
	qpData_t data{};
	data.options = options;
	float zero = 0.f, numerator = 1.f, result = 0.f;
	assert(backsolveDenseL(&data, &result, &zero, &numerator, QPDUNES_FALSE, 1) == QPDUNES_ERR_DIVISION_BY_ZERO);
	assert(qpDUNES_goldenSectionIntervalSearch(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, 0, 0.f, 0.f) == QPDUNES_ERR_UNKNOWN_LS_TYPE);
	assert(qpDUNES_reductionLineSearchWithASChange(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, 0, 0.f, 0.f, 0.f) == QPDUNES_ERR_UNKNOWN_LS_TYPE);
	float rectangular[] = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f};
	assert(qpDUNES_detectMatrixSparsity(rectangular, 2, 3) == QPDUNES_DENSE);
	assert(qpDUNES_detectMatrixSparsity(nullptr, 0, 3) == QPDUNES_MATRIX_UNDEFINED);
	// Allocation failure must unwind only C frames and leave no partial workspace.
	assert(om_qp_arena_open(16));
	assert(om_qp_setup_horizon_checked(&data, &options, 20) != QPDUNES_OK);
	om_qp_arena_close();
	for (int n : {10, 15, 20}) {
		auto p = problem(n);
		assert(om_float_update(p));
		auto r = om_float_solve(p);
		assert(r.status == QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND && r.finite && r.primal_residual < 2e-5f);
		const auto used = r.arena_used;
		for (int step = 0; step < 20; ++step) {
			p.x0[0] += .0001f;
			assert(om_float_update(p));
			r = om_float_solve(p);
			assert(r.status == QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND && r.finite && r.primal_residual < 2e-5f);
			assert(r.arena_used == used); // warm updates never allocate
		}
		// A genuine iteration-boundary deadline rejects output, then can recover.
		expired = true;
		assert(om_float_update(p));
		r = om_float_solve(p);
		assert(r.status != QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND);
		expired = false;
		assert(om_float_update(p));
		r = om_float_solve(p);
		assert(r.status == QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND && r.finite);
		p.q[0] *= .9f; // cost changes use the existing cold workspace path
		assert(om_float_update(p));
		assert(om_float_solve(p).status == QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND);
		p.x0[0] = std::numeric_limits<float>::quiet_NaN();
		assert(!om_float_update(p));
		assert(!om_float_solve(p).finite);
		om_float_close();
	}
	std::puts("PASS: float32 guards, unsupported algorithms, sparsity, allocation failure, N10/15/20, warm solves, deadlines, recovery, cost updates, nonfinite rejection");
	// Both hardware benchmark release periods traverse a complete eight with
	// nonzero, bounded tracking error; exercise the real structured warm solver.
	for (float release_dt : {.016f, .008f}) {
		OmFloatProblem p{};
		bool reuse = false;
		unsigned count = 0;
		for (float t = 0.f; t < 21.f; t += release_dt) {
			om_mpc::Path path;
			matrix::Vector3f position, velocity;
			matrix::Dcmf rotation;
			assert(om_mpc::bench_state(t, path, position, velocity, rotation));
			assert(om_mpc::build(path, 0.f, position, velocity, rotation, 19.22873f, {20.f, 20.f, 8.f}, p, reuse, .05f, 20));
			if (!reuse) {
				for (int i = 0; i < 9; ++i) { p.q[i] = p.p[i] = i < 3 ? 550.f : i < 6 ? 100.f : 500.f; }
				for (int i = 0; i < 4; ++i) { p.r[i] = i ? 350.f : .25f; }
				reuse = true;
			}
			assert(std::fabs(p.x0[0]) <= .15001f && std::fabs(p.x0[1]) <= .12001f && std::fabs(p.x0[2]) <= .05001f);
			assert(om_float_update(p, OmQpTolerance, OmQpLayout::OmErrorState));
			const auto r = om_float_solve(p);
			assert(r.status == QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND && r.finite && r.primal_residual <= 2e-5f);
			++count;
		}
		om_float_close();
		std::printf("PASS: synthetic hardware bench input/solver %.1f Hz, N20 dt=.05, %u solves\n", 1.f / release_dt, count);
	}
}
