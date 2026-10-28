// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cmath>
#include <stdint.h>
#include <uORB/topics/rc_channels.h>

// Shared request gates for the solver, output arbitration and INDI bridge.
// Like RC11/RC12 INDI, a parameter or a fresh RC request enables the feature.
// These requests never bypass flight, reference or measurement validity checks.
namespace om_mpc
{
inline bool rcRequested(const rc_channels_s &rc, uint64_t now, unsigned channel, float threshold)
{
	return rc.channel_count > channel && !rc.signal_lost && rc.timestamp != 0
	       && now >= rc.timestamp && now - rc.timestamp < 500000
	       && std::isfinite(rc.channels[channel]) && rc.channels[channel] >= threshold;
}

inline bool mpcRequested(bool parameter_enabled, const rc_channels_s &rc, uint64_t now)
{
	return parameter_enabled || rcRequested(rc, now, 8, 0.f); // RC9: middle/high
}

inline bool middleLayerRequested(bool parameter_enabled, const rc_channels_s &rc, uint64_t now)
{
	return parameter_enabled || rcRequested(rc, now, 9, 0.f); // RC10: middle/high
}

inline bool disturbanceRequested(bool parameter_enabled, const rc_channels_s &rc, uint64_t now)
{
	return parameter_enabled || rcRequested(rc, now, 9, .5f); // RC10: high only
}
}
