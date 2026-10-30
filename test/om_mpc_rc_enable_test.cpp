// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
// Standalone request-gate regression test; uses the generated rc_channels topic.
#include <lib/controllib/OmMpcEnable.hpp>
#include <cassert>
#include <cstdio>
#include <initializer_list>

int main()
{
	constexpr uint64_t now = 1000000;
	rc_channels_s rc{};
	rc.timestamp = now;
	rc.channel_count = 12;

	// All parameter combinations and all nine RC9/RC10 switch combinations.
	for (unsigned parameters = 0; parameters < 8; ++parameters) {
		for (float rc9 : {-1.f, 0.f, 1.f}) {
			for (float rc10 : {-1.f, 0.f, 1.f}) {
				rc.channels[8] = rc9;
				rc.channels[9] = rc10;
				assert(om_mpc::mpcRequested(parameters & 1, rc, now) == (bool(parameters & 1) || rc9 >= 0));
				assert(om_mpc::middleLayerRequested(parameters & 2, rc, now) == (bool(parameters & 2) || rc10 >= 0));
				assert(om_mpc::disturbanceRequested(parameters & 4, rc, now) == (bool(parameters & 4) || rc10 >= .5f));
			}
		}
	}

	// log_131: RC10 middle = trim = 1514, min/max = 1094/1934.
	// Reproduce RCUpdate's interpolation with the two fused operations emitted
	// on FMUv5; ideal {-1, 0, 1} inputs alone miss this regression.
	const float slope = 1.f / (1514.f - 1094.f);
	const float intercept = std::fma(-slope, 1094.f, -1.f);
	const float middle = std::fma(slope, 1514.f, intercept);
	assert(middle < 0.f && middle > -1e-6f);

	// All custom switches RC7..RC12 use the same calibrated-position thresholds.
	// Replay PWM values around low, trim and high, including trim +/-84 us.
	for (unsigned channel = 6; channel <= 11; ++channel) {
		for (float pwm : {1094.f, 1303.f, 1305.f, 1430.f, 1513.f, 1514.f, 1515.f, 1598.f, 1723.f, 1725.f, 1934.f}) {
			const float offset = pwm <= 1514.f ? intercept : std::fma(-slope, 1514.f, 0.f);
			rc.channels[channel] = std::fma(slope, pwm, offset);
			assert(om_mpc::rcRequested(rc, now, channel, -.5f) == (pwm >= 1305.f));
			assert(om_mpc::rcRequested(rc, now, channel, .5f) == (pwm >= 1725.f));
		}

		rc.channels[channel] = middle;
		assert(om_mpc::rcRequested(rc, now, channel, -.5f));
		assert(!om_mpc::rcRequested(rc, now, channel, .5f));
		assert(om_mpc::rcRequested(rc, now + 499999, channel, -.5f));
		assert(!om_mpc::rcRequested(rc, now + 500000, channel, -.5f));
		assert(!om_mpc::rcRequested(rc, now - 1, channel, -.5f));
		rc.timestamp = 0;
		assert(!om_mpc::rcRequested(rc, now, channel, -.5f));
		rc.timestamp = now;
		rc.signal_lost = true;
		assert(!om_mpc::rcRequested(rc, now, channel, -.5f));
		rc.signal_lost = false;
		rc.channel_count = channel;
		assert(!om_mpc::rcRequested(rc, now, channel, -.5f));
		rc.channel_count = 12;

		for (float invalid : {NAN, INFINITY, -INFINITY}) {
			rc.channels[channel] = invalid;
			assert(!om_mpc::rcRequested(rc, now, channel, -.5f));
			assert(!om_mpc::rcRequested(rc, now, channel, .5f));
		}
	}

	rc.channels[8] = rc.channels[9] = middle;
	assert(om_mpc::mpcRequested(false, rc, now));
	assert(om_mpc::middleLayerRequested(false, rc, now));
	assert(!om_mpc::disturbanceRequested(false, rc, now));

	// Small calibration offsets below trim still belong to the middle position.
	rc.channels[8] = rc.channels[9] = std::fma(slope, 1513.f, intercept);
	assert(om_mpc::mpcRequested(false, rc, now));
	assert(om_mpc::middleLayerRequested(false, rc, now));
	assert(!om_mpc::disturbanceRequested(false, rc, now));

	// The low/middle/high ranges tolerate offsets around all nominal positions.
	for (float low : {-1.f, -.9f, -.75f, -.51f}) {
		rc.channels[8] = rc.channels[9] = low;
		assert(!om_mpc::mpcRequested(false, rc, now));
		assert(!om_mpc::middleLayerRequested(false, rc, now));
		assert(!om_mpc::disturbanceRequested(false, rc, now));
	}

	for (float mid : {-.49f, -.3f, -.1f, -1e-5f, 0.f, .1f, .3f, .49f}) {
		rc.channels[8] = rc.channels[9] = mid;
		assert(om_mpc::mpcRequested(false, rc, now));
		assert(om_mpc::middleLayerRequested(false, rc, now));
		assert(!om_mpc::disturbanceRequested(false, rc, now));
	}

	for (float high : {.51f, .75f, .9f, 1.f}) {
		rc.channels[8] = rc.channels[9] = high;
		assert(om_mpc::mpcRequested(false, rc, now));
		assert(om_mpc::middleLayerRequested(false, rc, now));
		assert(om_mpc::disturbanceRequested(false, rc, now));
	}

	// Boundaries lie halfway between nominal switch positions, not at trim.
	rc.channels[8] = rc.channels[9] = std::nextafter(-.5f, -1.f);
	assert(!om_mpc::mpcRequested(false, rc, now));
	assert(!om_mpc::middleLayerRequested(false, rc, now));
	rc.channels[8] = rc.channels[9] = -.5f;
	assert(om_mpc::mpcRequested(false, rc, now));
	assert(om_mpc::middleLayerRequested(false, rc, now));
	assert(!om_mpc::disturbanceRequested(false, rc, now));
	rc.channels[9] = std::nextafter(.5f, 0.f);
	assert(!om_mpc::disturbanceRequested(false, rc, now));

	rc.channels[8] = rc.channels[9] = 1.f;
	assert(om_mpc::mpcRequested(false, rc, now + 499999));
	assert(!om_mpc::mpcRequested(false, rc, now + 500000));
	assert(!om_mpc::mpcRequested(false, rc, now - 1));
	assert(om_mpc::mpcRequested(true, rc, now + 500000));
	rc.timestamp = 0;
	assert(!om_mpc::mpcRequested(false, rc, now));
	rc.timestamp = now;
	rc.signal_lost = true;
	assert(!om_mpc::mpcRequested(false, rc, now));
	assert(!om_mpc::middleLayerRequested(false, rc, now));
	assert(!om_mpc::disturbanceRequested(false, rc, now));
	assert(om_mpc::mpcRequested(true, rc, now));
	assert(om_mpc::middleLayerRequested(true, rc, now));
	assert(om_mpc::disturbanceRequested(true, rc, now));
	rc.signal_lost = false;
	rc.channel_count = 8;
	assert(!om_mpc::mpcRequested(false, rc, now));
	rc.channel_count = 9;
	assert(om_mpc::mpcRequested(false, rc, now));
	assert(!om_mpc::middleLayerRequested(false, rc, now));
	assert(!om_mpc::disturbanceRequested(false, rc, now));
	rc.channel_count = 12;
	for (float invalid : {NAN, INFINITY, -INFINITY}) {
		rc.channels[8] = rc.channels[9] = invalid;
		assert(!om_mpc::mpcRequested(false, rc, now));
		assert(!om_mpc::middleLayerRequested(false, rc, now));
		assert(!om_mpc::disturbanceRequested(false, rc, now));
	}
	rc.channels[9] = .499f;
	assert(om_mpc::middleLayerRequested(false, rc, now));
	assert(!om_mpc::disturbanceRequested(false, rc, now));
	rc.channels[9] = .5f;
	assert(om_mpc::disturbanceRequested(false, rc, now));
	std::puts("PASS: RC7..RC12 threshold PWM replay, 72 MPC parameter/RC combinations, trim roundoff, ranges and boundaries");
}
