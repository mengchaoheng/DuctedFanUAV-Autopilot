// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cmath>
#include <uORB/topics/om_mpc_command.h>

// One owner (the MPC worker). Keep attempt diagnostics separate from the last
// successful control values and their original age. No timers or allocations.
class OmMpcCommandHold
{
public:
	static constexpr uint64_t TimeoutUs = 60000;

	bool select(om_mpc_command_s &attempt, bool eligible, bool accept_new, uint64_t now)
	{
		attempt.active = false;
		attempt.held = false;
		attempt.timestamp_valid = 0;

		if (!eligible) {
			reset();
			return false;
		}

		if (accept_new && now && finite(attempt)) {
			attempt.active = true;
			attempt.timestamp_valid = now;
			_last = attempt;
			return true;
		}

		if (_last.active && _last.timestamp_valid && now >= _last.timestamp_valid
		    && now - _last.timestamp_valid < TimeoutUs) {
			for (unsigned i = 0; i < 3; ++i) {
				attempt.rates[i] = _last.rates[i];
				attempt.thrust_body[i] = _last.thrust_body[i];
				attempt.mpc_disturbance[i] = _last.mpc_disturbance[i];
			}

			attempt.hover_thrust = _last.hover_thrust;
			attempt.timestamp_valid = _last.timestamp_valid;
			attempt.active = true;
			attempt.held = true;
			return true;
		}

		reset();
		return false;
	}

	void reset() { _last = {}; }

private:
	static bool finite(const om_mpc_command_s &command)
	{
		if (!std::isfinite(command.hover_thrust) || command.hover_thrust <= .01f) { return false; }

		for (unsigned i = 0; i < 3; ++i) {
			if (!std::isfinite(command.rates[i]) || !std::isfinite(command.thrust_body[i])
			    || !std::isfinite(command.mpc_disturbance[i])) { return false; }
		}

		return true;
	}

	om_mpc_command_s _last{};
};
