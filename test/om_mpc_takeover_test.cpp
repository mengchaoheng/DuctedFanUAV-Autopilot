// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
#include <lib/controllib/OmMpcTakeover.hpp>
#include <cassert>
#include <cstdio>

using Command = OmMpcTakeover::Command;

static void near(const Command &a, const Command &b)
{
	assert(Command(a - b).norm() < 1e-5f);
}

int main()
{
	Command pid{}, mpc{}, next_pid{};

	for (int i = 0; i < 6; ++i) {
		pid(i) = .1f * i;
		mpc(i) = -.2f * i;
		next_pid(i) = .15f * i;
	}

	OmMpcTakeover transfer;
	near(transfer.update(false, pid, mpc, 1000000, .5f), pid);
	// First selected MPC output equals the last actual output, even if the
	// independently running PID has computed a different output in this cycle.
	near(transfer.update(true, next_pid, mpc, 1004000, .5f), pid);
	assert(fabsf(transfer.progress()) < 1e-6f);
	near(transfer.update(true, next_pid, mpc, 1254000, .5f), (pid + mpc) * .5f);
	near(transfer.update(true, next_pid, mpc, 1504000, .5f), mpc);
	assert(fabsf(transfer.progress() - 1.f) < 1e-6f);
	near(transfer.update(true, next_pid, mpc * .7f, 1508000, .5f), mpc * .7f);
	// Exit never blends: PID is returned immediately, without using its old value.
	near(transfer.update(false, next_pid, mpc, 1512000, .5f), next_pid);
	near(transfer.update(true, pid, mpc, 1516000, .5f), next_pid);
	// Aborted entry followed by a re-entry starts from the new applied PID output.
	near(transfer.update(false, pid, mpc, 1520000, .5f), pid);
	near(transfer.update(true, next_pid, mpc, 1524000, .5f), pid);
	transfer.reset();
	near(transfer.update(true, next_pid, mpc, 2000000, .5f), next_pid);
	// Stale output history cannot create a spurious jump after a control-mode gap.
	transfer.update(false, pid, mpc, 2100000, .5f);
	near(transfer.update(true, next_pid, mpc, 2300000, .5f), next_pid);
	transfer.reset();
	near(transfer.update(true, pid, mpc, 2400000, 0.f), mpc);
	mpc(0) = NAN;
	near(transfer.update(true, pid, mpc, 2404000, .5f), pid);
	transfer.reset();
	mpc(0) = 0.f;
	near(transfer.update(true, pid, mpc, 2500000, 2.f), pid);
	// Duration changes apply on the next entry, never prolong the current one.
	near(transfer.update(true, pid, mpc, 4500000, 4.f), mpc);
	transfer.reset();
	near(transfer.update(true, pid, mpc, 5000000, 10.f), pid);
	near(transfer.update(true, pid, mpc, 10000000, 10.f), mpc);
	transfer.reset();
	mpc(0) = 0.f;
	pid(0) = NAN;
	near(transfer.update(true, pid, mpc, 2500000, .5f), mpc);
	std::puts("PASS: takeover continuity, endpoint, unmodified steady output, direct PID exit, re-entry, stale history, invalid-command fallback, latched duration and 5-second cap");
}
