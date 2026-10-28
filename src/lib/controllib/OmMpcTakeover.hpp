// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <matrix/matrix/math.hpp>
#include <cmath>
#include <stdint.h>

// Entry-only output-offset transfer, matching PositionControl's INDI transfer.
// PID keeps running independently. Disable/invalid commands return PID directly.
class OmMpcTakeover
{
public:
	using Command = matrix::Vector<float, 6>;

	Command update(bool active, const Command &pid, const Command &mpc, uint64_t now, float duration)
	{
		if (!active || !mpc.isAllFinite()) {
			_active = false;
			_progress = 0.f;
			recordApplied(pid, now);
			return pid;
		}

		if (!_active) {
			// Use the preceding actual limited output, including after an aborted
			// transfer. Never reuse a stale output after a control-mode interruption.
			const bool previous_fresh = _have_previous && now >= _last_time && now - _last_time < 100000;
			const Command &anchor = previous_fresh ? _last_output : pid;
			_offset.setZero();

			if (anchor.isAllFinite()) {
				_offset = anchor - mpc;
			}

			_start = now;
			// Latch on entry so parameter changes cannot prolong this transfer.
			_duration = std::isfinite(duration) && duration > 0.f ? fminf(duration, 5.f) : 0.f;
			_active = true;
		}

		const float elapsed = now >= _start ? (now - _start) * 1e-6f : 0.f;
		const float ratio = _duration > 0.f ? fminf(elapsed / _duration, 1.f) : 1.f;
		_progress = ratio * ratio * (3.f - 2.f * ratio);
		const Command output = mpc + _offset * (1.f - _progress);
		recordApplied(output, now);
		return output;
	}

	void recordApplied(const Command &output, uint64_t now)
	{
		_have_previous = output.isAllFinite();
		_last_output = output;
		_last_time = now;
	}

	void reset()
	{
		_active = false;
		_have_previous = false;
		_progress = 0.f;
	}

	float progress() const { return _progress; }

private:
	Command _offset{}, _last_output{};
	uint64_t _start{0}, _last_time{0};
	float _progress{0.f}, _duration{0.f};
	bool _active{false}, _have_previous{false};
};
