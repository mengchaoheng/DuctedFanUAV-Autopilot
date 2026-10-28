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
	std::puts("PASS: 72 parameter/RC combinations, thresholds, timeout, loss, missing channels and non-finite inputs");
}
