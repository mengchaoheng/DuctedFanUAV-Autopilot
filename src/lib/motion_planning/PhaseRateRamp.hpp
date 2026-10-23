// Copyright 2026 PX4 Development Team. SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cmath>

namespace motion_planning
{
// Exact phase integral of a finite linear rate ramp. Zero duration keeps the
// configured start rate; a completed finite ramp holds its terminal rate.
class PhaseRateRamp
{
public:
	void reset(float start, float end, float duration)
	{
		_start = start;
		_end = end;
		_duration = std::isfinite(duration) && duration > 0.f ? duration : 0.f;
		_slope = _duration > 0.f ? (end - start) / _duration : 0.f;
		_elapsed = 0.f;
	}

	void retarget(float end, float duration)
	{
		if (duration > 0.f && (fabsf(end - _end) > 1e-6f || fabsf(duration - _duration) > 1e-6f)) {
			reset(rate(), end, duration);
		}
	}

	float remaining() const { return fmaxf(_duration - _elapsed, 0.f); }
	float rate() const { return _start + _slope * fminf(_elapsed, _duration); }
	float acceleration() const { return remaining() > 0.f ? _slope : 0.f; }

	float advance(float dt)
	{
		const float ramp_dt = fminf(fmaxf(dt, 0.f), remaining());
		const float phase_step = rate() * dt + _slope * ramp_dt * (dt - .5f * ramp_dt);
		_elapsed = fminf(_elapsed + fmaxf(dt, 0.f), _duration);
		return phase_step;
	}

private:
	float _start{}, _end{}, _slope{}, _duration{}, _elapsed{};
};
}
