// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
#include <lib/controllib/OmMpcCommandHold.hpp>
#include <lib/controllib/OmMpcTakeover.hpp>
#include <cassert>
#include <cstdio>
#include <initializer_list>

static om_mpc_command_s good(uint64_t time)
{
	om_mpc_command_s command{};
	command.timestamp = time;
	command.timestamp_sample = time - 4000;
	command.solver_status = 1;
	command.hover_thrust = .51f;
	command.rates[0] = .1f;
	command.rates[1] = -.2f;
	command.thrust_body[2] = -.52f;
	command.mpc_disturbance[1] = .3f;
	return command;
}

static OmMpcTakeover::Command controls(const om_mpc_command_s &command)
{
	OmMpcTakeover::Command value;

	for (unsigned i = 0; i < 3; ++i) { value(i) = command.rates[i]; value(i + 3) = command.thrust_body[i]; }

	return value;
}

int main()
{
	constexpr uint64_t start = 1000000;
	OmMpcCommandHold hold;
	om_mpc_command_s failed{};
	failed.timestamp = start;
	failed.solver_status = 7;
	assert(!hold.select(failed, true, false, start)); // cold/no previous success
	assert(!failed.active && !failed.held && !failed.timestamp_valid);
	auto command = good(start);
	assert(hold.select(command, true, true, start));
	assert(command.active && !command.held && command.timestamp_valid == start);

	for (uint64_t age : {8000ULL, 16000ULL, 59999ULL}) {
		failed = {};
		failed.timestamp = start + age;
		failed.timestamp_sample = start + age - 4000;
		failed.reference_timestamp = start + age - 8000;
		failed.deadline_missed = true;
		failed.solver_status = 8;
		failed.mpc_compute_us = 8043;
		failed.qp_solve_us = 4032;
		failed.cpu_us = 3438;
		failed.position_error[0] = .4f;
		assert(hold.select(failed, true, false, start + age));
		assert(failed.active && failed.held && failed.timestamp_valid == start);
		assert(failed.timestamp == start + age && failed.timestamp_sample == start + age - 4000);
		assert(failed.solver_status == 8 && failed.deadline_missed && failed.mpc_compute_us == 8043 && failed.cpu_us == 3438);
		assert(failed.qp_solve_us == 4032);
		assert(failed.position_error[0] == .4f); // attempt diagnostics are untouched
		assert(matrix::isEqual(controls(failed), controls(command)));
		assert(failed.hover_thrust == command.hover_thrust && failed.mpc_disturbance[1] == command.mpc_disturbance[1]);
	}

	assert(!hold.select(failed, true, false, start + 60000)); // repeated failures cannot renew age
	assert(!failed.active && !failed.held && !failed.timestamp_valid);
	assert(!hold.select(failed, true, false, start + 60001));
	command = good(start + 100000);
	assert(hold.select(command, true, true, command.timestamp));
	assert(!hold.select(failed, false, false, command.timestamp + 1)); // manual exit/mode/state invalid
	assert(!hold.select(failed, true, false, command.timestamp + 2)); // cannot revive pre-exit cache
	assert(hold.select(command, true, true, command.timestamp));
	hold.reset(); // estimator reset / QP configuration change
	assert(!hold.select(failed, true, false, command.timestamp + 1));
	assert(hold.select(command, true, true, command.timestamp));
	assert(!hold.select(failed, true, false, command.timestamp - 1)); // no future timestamp
	command.rates[0] = NAN;
	assert(!hold.select(command, true, true, command.timestamp));

	// A retained command keeps entry progress. Expiry returns PID directly;
	// a later successful solve starts the existing entry-only transfer again.
	OmMpcTakeover takeover;
	OmMpcTakeover::Command pid{};
	pid(0) = -.1f;
	command = good(2000000);
	assert(hold.select(command, true, true, command.timestamp));
	takeover.update(command.active, pid, controls(command), command.timestamp, 1.f);
	command = good(2250000);
	assert(hold.select(command, true, true, command.timestamp));
	takeover.update(command.active, pid, controls(command), command.timestamp, 1.f);
	const float progress = takeover.progress();
	failed = {};
	failed.timestamp = 2258000;
	failed.deadline_missed = true;
	assert(hold.select(failed, true, false, failed.timestamp));
	takeover.update(failed.active, pid, controls(failed), failed.timestamp, 1.f);
	assert(takeover.progress() > progress);
	failed.timestamp = 2310000;
	assert(!hold.select(failed, true, false, failed.timestamp));
	assert(matrix::isEqual(takeover.update(failed.active, pid, controls(failed), failed.timestamp, 1.f), pid));
	assert(takeover.progress() == 0.f);
	command = good(2318000);
	assert(hold.select(command, true, true, command.timestamp));
	assert(matrix::isEqual(takeover.update(command.active, pid, controls(command), command.timestamp, 1.f), pid));
	assert(takeover.progress() == 0.f);
	std::puts("PASS: MPC hold preserves controls/diagnostics/original age, 60 ms expiry, explicit invalidation, invalid input, continuous takeover and PID re-entry");
}
