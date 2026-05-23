/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ControlAllocationMixer.hpp"

#include "ControlAllocation.h"
#include "../../mixer_module/wls_alloc_gen.h"

#define MODULE_NAME "control_allocation_mixer"

#include <float.h>
#include <math.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <uORB/topics/actuator_controls.h>

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)

template<int ControlSize, int EffectorSize, bool CanRestore>
struct AllocationRestorer {
	static void apply(DP_LP_ControlAllocator<ControlSize, EffectorSize> &, float u_tmp[EffectorSize],
			  float u[EffectorSize])
	{
		for (int i = 0; i < EffectorSize; i++) {
			u[i] = u_tmp[i];
		}
	}
};

template<int ControlSize, int EffectorSize>
struct AllocationRestorer<ControlSize, EffectorSize, true> {
	static void apply(DP_LP_ControlAllocator<ControlSize, EffectorSize> &allocator, float u_tmp[EffectorSize],
			  float u[EffectorSize])
	{
		allocator.restoring(u_tmp, u);
	}
};

template<int ControlSize, int EffectorSize>
static void update_allocator_from_config(DP_LP_ControlAllocator<ControlSize, EffectorSize> &allocator,
		const ControlAllocationMixer::Config &config)
{
	for (int actuator = 0; actuator < EffectorSize; actuator++) {
		allocator.aircraft.lowerLimits[actuator] = config.ranges[actuator].active_min;
		allocator.aircraft.upperLimits[actuator] = config.ranges[actuator].active_max;
	}

	for (int row = 0; row < ControlSize; row++) {
		for (int col = 0; col < EffectorSize; col++) {
			allocator.aircraft.controlEffectMatrix[row][col] = config.B[row][col];
		}
	}

	allocator.isupdate = true;
}

template<int ControlSize, int EffectorSize>
static bool allocate_dir_with_allocator(DP_LP_ControlAllocator<ControlSize, EffectorSize> &allocator,
					const ControlAllocationMixer::Config &config, const float *y, float *u)
{
	update_allocator_from_config(allocator, config);

	float input_higher[ControlSize] {};
	float input_lower[ControlSize] {};
	float u_tmp[EffectorSize] {};
	int err = 0;
	float rho = 0.f;

	for (int i = 0; i < ControlSize; i++) {
		input_lower[i] = y[i];
	}

	allocator.DP_LPCA_copy(input_higher, input_lower, u_tmp, err, rho);
	AllocationRestorer<ControlSize, EffectorSize, (EffectorSize <= ControlSize + 1)>::apply(allocator, u_tmp, u);

	return true;
}

template<int ControlSize, int EffectorSize>
static bool allocate_pca_with_allocator(DP_LP_ControlAllocator<ControlSize, EffectorSize> &allocator,
					const ControlAllocationMixer::Config &config, const float *higher, const float *lower, float *u)
{
	update_allocator_from_config(allocator, config);

	float input_higher[ControlSize] {};
	float input_lower[ControlSize] {};
	float u_tmp[EffectorSize] {};
	int err = 0;
	float rho = 0.f;

	for (int i = 0; i < ControlSize; i++) {
		input_higher[i] = higher[i];
		input_lower[i] = lower[i];
	}

	allocator.DP_LPCA_copy(input_higher, input_lower, u_tmp, err, rho);

	if (err < 0) {
		float zero[ControlSize] {};
		float fallback_tmp[EffectorSize] {};
		int fallback_err = 0;
		float fallback_rho = 0.f;

		allocator.DP_LPCA_copy(zero, input_higher, fallback_tmp, fallback_err, fallback_rho);
		AllocationRestorer<ControlSize, EffectorSize, (EffectorSize <= ControlSize + 1)>::apply(allocator, fallback_tmp, u);

	} else {
		AllocationRestorer<ControlSize, EffectorSize, (EffectorSize <= ControlSize + 1)>::apply(allocator, u_tmp, u);
	}

	return true;
}

class ControlAllocationMixer::DpAllocator
{
public:
	virtual ~DpAllocator() = default;
	virtual bool allocate_dir(const Config &config, const float *y, float *u) = 0;
	virtual bool allocate_pca(const Config &config, const float *higher, const float *lower, float *u) = 0;
};

template<int ControlSize, int EffectorSize>
class DpAllocatorImpl : public ControlAllocationMixer::DpAllocator
{
public:
	explicit DpAllocatorImpl(const ControlAllocationMixer::Config &config) :
		_allocator(_aircraft)
	{
		update_allocator_from_config(_allocator, config);
	}

	bool allocate_dir(const ControlAllocationMixer::Config &config, const float *y, float *u) override
	{
		return allocate_dir_with_allocator(_allocator, config, y, u);
	}

	bool allocate_pca(const ControlAllocationMixer::Config &config, const float *higher, const float *lower,
			  float *u) override
	{
		return allocate_pca_with_allocator(_allocator, config, higher, lower, u);
	}

private:
	Aircraft<ControlSize, EffectorSize> _aircraft {};
	DP_LP_ControlAllocator<ControlSize, EffectorSize> _allocator;
};

template<int ControlSize, int EffectorSize>
static ControlAllocationMixer::DpAllocator *
create_dp_allocator_impl(const ControlAllocationMixer::Config &config)
{
	return new DpAllocatorImpl<ControlSize, EffectorSize>(config);
}

struct DpAllocatorFactory {
	uint8_t y_dim;
	uint8_t u_dim;
	ControlAllocationMixer::DpAllocator *(*create)(const ControlAllocationMixer::Config &config);
};

static constexpr DpAllocatorFactory kDpAllocatorFactories[] {
	{3, 4, create_dp_allocator_impl<3, 4>},
	{3, 6, create_dp_allocator_impl<3, 6>},
	{3, 8, create_dp_allocator_impl<3, 8>},
};

ControlAllocationMixer::ControlAllocationMixer(ControlAllocationCallback control_allocation_cb, uintptr_t cb_handle,
		const Config &config) :
	Mixer(nullptr, 0),
	_config(config),
	_control_allocation_cb(control_allocation_cb),
	_control_allocation_cb_handle(cb_handle),
	_method_param(param_find("USER_AC_METHOD")),
	_dist_enable_param(param_find("USER_ADD_DIST")),
	_dist_mag_param(param_find("USER_DIST_MAG")),
	_omega_2_force_param(param_find("USER_OMEGA_2_F")),
	_cs_cutoff_param(param_find("USER_CS_CUTOFF")),
	_time_const_param(param_find("USER_TIME_CONST")),
	_motor_time_const_param(param_find("USER_MOT_TCONST")),
	_use_actuator_param(param_find("USER_ACTUATOR")),
	_wls_available(select_wls(config)),
	_dp_allocator(create_dp_allocator(config))
{
	update_feedback_params();

	if ((is_ductedfan4_physical_config(_config) || is_shc09_physical_config(_config)) &&
	    _omega_2_force_param != PARAM_INVALID) {
		float k = NAN;

		if (param_get(_omega_2_force_param, &k) == 0 && PX4_ISFINITE(k)) {
			update_physical_model_B(k);
			_last_omega_2_force = k;
		}
	}
}

ControlAllocationMixer::~ControlAllocationMixer()
{
	delete _dp_allocator;
}

ControlAllocationMixer *
ControlAllocationMixer::from_text(Mixer::ControlAllocationCallback control_allocation_cb, uintptr_t cb_handle,
				  const char *buf, unsigned &buflen)
{
	Config config {};
	int y_dim = 0;
	int u_dim = 0;
	int method = 0;
	int used = 0;

	if (!string_well_formed(buf, buflen)) {
		return nullptr;
	}

	if (control_allocation_cb == nullptr) {
		return nullptr;
	}

	// C: <y_dim> <u_dim> <method>
	// method: 0 INV, 1 WLS, 2 DIR, 3 PCA.
	const int parsed_c = sscanf(buf, "C: %d %d %d%n", &y_dim, &u_dim, &method, &used);

	if (parsed_c != 3) {
		debug("control allocation parse failed on '%s'", buf);
		return nullptr;
	}

	if (y_dim <= 0 || y_dim > static_cast<int>(MAX_Y) || u_dim <= 0 || u_dim > static_cast<int>(MAX_U)) {
		debug("invalid control allocation dimensions y=%d u=%d", y_dim, u_dim);
		return nullptr;
	}

	if (method < static_cast<int>(Method::INV) || method > static_cast<int>(Method::PCA)) {
		debug("invalid control allocation method %d", method);
		return nullptr;
	}

	config.y_dim = static_cast<uint8_t>(y_dim);
	config.u_dim = static_cast<uint8_t>(u_dim);
	config.method = static_cast<Method>(method);

	buf = skipline(buf, buflen);

	if (buf == nullptr) {
		debug("no line ending after C line");
		return nullptr;
	}

	for (unsigned i = 0; i < config.y_dim; i++) {
		unsigned group = 0;
		unsigned index = 0;

		buf = findtag(buf, buflen, 'S');

		if (buf == nullptr) {
			debug("failed finding S line");
			return nullptr;
		}

		if (sscanf(buf, "S: %u %u%n", &group, &index, &used) != 2) {
			debug("failed parsing S line '%s'", buf);
			return nullptr;
		}

		if (group >= 32 || index > UINT8_MAX) {
			debug("invalid S line group=%u index=%u", group, index);
			return nullptr;
		}

		config.sources[i].group = static_cast<uint8_t>(group);
		config.sources[i].index = static_cast<uint8_t>(index);

		buf = skipline(buf, buflen);

		if (buf == nullptr) {
			debug("no line ending after S line");
			return nullptr;
		}
	}

	for (unsigned i = 0; i < config.y_dim; i++) {
		buf = findtag(buf, buflen, 'B');

		if (buf == nullptr) {
			debug("failed finding B line");
			return nullptr;
		}

		if (!parse_b_row(buf, config.u_dim, config.B[i])) {
			debug("failed parsing B line '%s'", buf);
			return nullptr;
		}

		buf = skipline(buf, buflen);

		if (buf == nullptr) {
			debug("no line ending after B line");
			return nullptr;
		}
	}

	bool output_slot_used[MAX_OUTPUTS] {};
	bool feedback_slot_used[MAX_FEEDBACK] {};

	for (unsigned i = 0; i < config.u_dim; i++) {
		float physical_min = 0.f;
		float physical_max = 0.f;
		int output_slot = static_cast<int>(i);
		int feedback_slot = -1;
		int actuator_type = static_cast<int>(ActuatorType::Servo);

		buf = findtag(buf, buflen, 'U');

		if (buf == nullptr) {
			debug("failed finding U line");
			return nullptr;
		}

		const int parsed = sscanf(buf, "U: %f %f %d %d %d%n", &physical_min, &physical_max, &output_slot,
					  &feedback_slot, &actuator_type, &used);

		if (parsed < 2 || parsed > 5) {
			debug("failed parsing U line '%s'", buf);
			return nullptr;
		}

		if (!PX4_ISFINITE(physical_min) || !PX4_ISFINITE(physical_max) || physical_max <= physical_min) {
			debug("invalid physical range min=%f max=%f", static_cast<double>(physical_min),
			      static_cast<double>(physical_max));
			return nullptr;
		}

		if (output_slot < 0 || output_slot >= static_cast<int>(MAX_OUTPUTS) || output_slot_used[output_slot]) {
			debug("invalid or duplicate output slot %d", output_slot);
			return nullptr;
		}

		if (feedback_slot < -1 || feedback_slot >= static_cast<int>(MAX_FEEDBACK) ||
		    (feedback_slot >= 0 && feedback_slot_used[feedback_slot])) {
			debug("invalid or duplicate feedback slot %d", feedback_slot);
			return nullptr;
		}

		if (!valid_actuator_type(actuator_type)) {
			debug("invalid actuator type %d", actuator_type);
			return nullptr;
		}

		output_slot_used[output_slot] = true;
		if (feedback_slot >= 0) {
			feedback_slot_used[feedback_slot] = true;
		}

		config.ranges[i].physical_min = physical_min;
		config.ranges[i].physical_max = physical_max;
		config.ranges[i].active_min = physical_min;
		config.ranges[i].active_max = physical_max;
		config.ranges[i].output_slot = static_cast<uint8_t>(output_slot);
		config.ranges[i].feedback_slot = static_cast<int8_t>(feedback_slot);
		config.ranges[i].actuator_type = static_cast<ActuatorType>(actuator_type);

		const uint8_t output_count = static_cast<uint8_t>(output_slot + 1);

		if (output_count > config.output_count) {
			config.output_count = output_count;
		}

		buf = skipline(buf, buflen);

		if (buf == nullptr) {
			debug("no line ending after U line");
			return nullptr;
		}
	}

	if (!compute_pseudo_inverse(config)) {
		debug("failed computing B pseudo-inverse");
		return nullptr;
	}

	return new ControlAllocationMixer(control_allocation_cb, cb_handle, config);
}

unsigned
ControlAllocationMixer::mix(float *outputs, unsigned space)
{
	if (space < _config.output_count) {
		return 0;
	}

	update_sample_freq();
	update_runtime_config();
	update_feedback_params();

	for (unsigned i = 0; i < _config.output_count; i++) {
		outputs[i] = NAN;
	}

	float y[MAX_Y] {};

	for (unsigned i = 0; i < _config.y_dim; i++) {
		float input = 0.f;

		if (!read_source(_config.sources[i], input)) {
			return 0;
		}

		y[i] = input;
	}

	float u[MAX_U] {};
	bool allocated = false;
	const Method requested_method = selected_method();
	Method actual_method = requested_method;
	bool fallback = false;

	// Dispatch the allocation method selected by the C: line. Unsupported
	// backend dimensions deliberately fall through to INV below.
	switch (requested_method) {
	case Method::INV:
		allocated = allocate_inv(_config, y, u);
		break;

	case Method::WLS:
		allocated = allocate_wls(y, u);
		break;

	case Method::DIR:
		allocated = _dp_allocator != nullptr && _dp_allocator->allocate_dir(_config, y, u);
		break;

	case Method::PCA:
		if (_dp_allocator != nullptr) {
			float higher[MAX_Y] {};
			float lower[MAX_Y] {};

			if (read_pca_inputs(y, higher, lower)) {
				allocated = _dp_allocator->allocate_pca(_config, higher, lower, u);
			}
		}

		if (!allocated) {
			allocated = _dp_allocator != nullptr && _dp_allocator->allocate_dir(_config, y, u);

			if (allocated) {
				actual_method = Method::DIR;
				fallback = true;
			}
		}

		break;
	}

	if (!allocated) {
		allocated = allocate_inv(_config, y, u);
		actual_method = Method::INV;
		fallback = requested_method != Method::INV;
	}

	if (!allocated) {
		return 0;
	}

	for (unsigned actuator = 0; actuator < _config.u_dim; actuator++) {
		u[actuator] = math::constrain(u[actuator], _config.ranges[actuator].active_min,
					      _config.ranges[actuator].active_max);
	}

	float u_cmd[MAX_U] {};

	for (unsigned actuator = 0; actuator < _config.u_dim; actuator++) {
		const float requested_u = math::constrain(u[actuator],
					  _config.ranges[actuator].physical_min,
					  _config.ranges[actuator].physical_max);
		u_cmd[actuator] = apply_actuator_model(requested_u, actuator);
	}

	report_method(requested_method, actual_method, fallback);
	publish_debug(y, u, u_cmd, requested_method, actual_method, fallback);
	publish_actuator_feedback();

	for (unsigned actuator = 0; actuator < _config.u_dim; actuator++) {
		const float output_u = math::constrain(u_cmd[actuator] + runtime_disturbance_bias(actuator),
						       _config.ranges[actuator].physical_min,
						       _config.ranges[actuator].physical_max);
		outputs[_config.ranges[actuator].output_slot] = map_u_to_output(output_u, actuator);
	}

	return _config.output_count;
}

void
ControlAllocationMixer::groups_required(uint32_t &groups)
{
	for (unsigned i = 0; i < _config.y_dim; i++) {
		groups |= 1 << _config.sources[i].group;
	}
}

unsigned
ControlAllocationMixer::get_trim(float *trim)
{
	if (trim != nullptr) {
		*trim = 0.f;
	}

	return _config.output_count;
}

bool
ControlAllocationMixer::parse_b_row(const char *buf, unsigned columns, float *row)
{
	if (buf == nullptr || buf[0] != 'B' || buf[1] != ':') {
		return false;
	}

	const char *cursor = buf + 2;

	for (unsigned i = 0; i < columns; i++) {
		char *end = nullptr;
		const float value = strtof(cursor, &end);

		if (end == cursor || !PX4_ISFINITE(value)) {
			return false;
		}

		row[i] = value;
		cursor = end;
	}

	return true;
}

bool
ControlAllocationMixer::compute_pseudo_inverse(Config &config)
{
	matrix::Matrix<float, MAX_Y, MAX_U> B;
	matrix::Matrix<float, MAX_U, MAX_Y> B_pinv;

	for (unsigned row = 0; row < config.y_dim; row++) {
		for (unsigned actuator = 0; actuator < config.u_dim; actuator++) {
			B(row, actuator) = config.B[row][actuator];
		}
	}

	if (!matrix::geninv(B, B_pinv)) {
		return false;
	}

	for (unsigned actuator = 0; actuator < config.u_dim; actuator++) {
		for (unsigned row = 0; row < config.y_dim; row++) {
			config.B_pinv[actuator][row] = B_pinv(actuator, row);
		}
	}

	// debug_b_pinv(config);
	return true;
}

bool
ControlAllocationMixer::allocate_inv(const Config &config, const float *y, float *u)
{
	for (unsigned actuator = 0; actuator < config.u_dim; actuator++) {
		u[actuator] = 0.f;

		for (unsigned i = 0; i < config.y_dim; i++) {
			u[actuator] += config.B_pinv[actuator][i] * y[i];
		}
	}

	return true;
}

bool
ControlAllocationMixer::allocate_wls(const float *y, float *u) const
{
	if (!_wls_available) {
		return false;
	}

	float B[12] {};
	float umin[4] {};
	float umax[4] {};
	float Wv[9] {
		1.f, 0.f, 0.f,
		0.f, 1.f, 0.f,
		0.f, 0.f, 1.f
	};
	float Wu[16] {
		1.f, 0.f, 0.f, 0.f,
		0.f, 1.f, 0.f, 0.f,
		0.f, 0.f, 1.f, 0.f,
		0.f, 0.f, 0.f, 1.f
	};
	float ud[4] {};
	float W[4] {};
	const float gam = 1e6f;

	for (unsigned row = 0; row < _config.y_dim; row++) {
		for (unsigned col = 0; col < _config.u_dim; col++) {
			B[row + _config.y_dim * col] = _config.B[row][col];
		}
	}

	for (unsigned actuator = 0; actuator < _config.u_dim; actuator++) {
		umin[actuator] = _config.ranges[actuator].active_min;
		umax[actuator] = _config.ranges[actuator].active_max;
	}

	wls_alloc_gen(B, y, umin, umax, Wv, Wu, ud, gam, u, W, 100.f, 4.f);
	return true;
}

bool
ControlAllocationMixer::select_wls(const Config &config)
{
	return config.y_dim == 3 && config.u_dim == 4;
}

ControlAllocationMixer::DpAllocator *
ControlAllocationMixer::create_dp_allocator(const Config &config)
{
	for (const DpAllocatorFactory &factory : kDpAllocatorFactories) {
		if (factory.y_dim == config.y_dim && factory.u_dim == config.u_dim) {
			return factory.create(config);
		}
	}

	return nullptr;
}

void
ControlAllocationMixer::debug_b_pinv(const Config &config)
{
	debug("B_pinv (%u x %u)", static_cast<unsigned>(config.u_dim), static_cast<unsigned>(config.y_dim));

	for (unsigned actuator = 0; actuator < config.u_dim; actuator++) {
		debug("B_pinv[%u]: % .6f % .6f % .6f % .6f % .6f % .6f",
		      actuator,
		      static_cast<double>(config.B_pinv[actuator][0]),
		      static_cast<double>(config.y_dim > 1 ? config.B_pinv[actuator][1] : 0.f),
		      static_cast<double>(config.y_dim > 2 ? config.B_pinv[actuator][2] : 0.f),
		      static_cast<double>(config.y_dim > 3 ? config.B_pinv[actuator][3] : 0.f),
		      static_cast<double>(config.y_dim > 4 ? config.B_pinv[actuator][4] : 0.f),
		      static_cast<double>(config.y_dim > 5 ? config.B_pinv[actuator][5] : 0.f));
	}
}

bool
ControlAllocationMixer::control_axis_from_index(uint8_t index, uint8_t &axis)
{
	if (index >= actuator_controls_s::NUM_ACTUATOR_CONTROLS) {
		return false;
	}

	axis = index;
	return axis <= actuator_controls_s::INDEX_YAW;
}

bool
ControlAllocationMixer::is_ductedfan4_physical_config(const Config &config)
{
	static constexpr float df4_min = -0.3491f;
	static constexpr float df4_max = 0.3491f;
	static constexpr float tolerance = 0.01f;

	if (config.y_dim != 3 || config.u_dim != 4) {
		return false;
	}

	for (unsigned actuator = 0; actuator < config.u_dim; actuator++) {
		if (fabsf(config.ranges[actuator].physical_min - df4_min) > tolerance ||
		    fabsf(config.ranges[actuator].physical_max - df4_max) > tolerance) {
			return false;
		}
	}

	return true;
}

bool
ControlAllocationMixer::is_ductedfan4_normalized_config(const Config &config)
{
	static constexpr float tolerance = 0.01f;

	if (config.y_dim != 3 || config.u_dim != 4) {
		return false;
	}

	for (unsigned actuator = 0; actuator < config.u_dim; actuator++) {
		if (fabsf(config.ranges[actuator].physical_min + 1.f) > tolerance ||
		    fabsf(config.ranges[actuator].physical_max - 1.f) > tolerance) {
			return false;
		}
	}

	return fabsf(config.B[0][0] + 0.5f) <= tolerance &&
	       fabsf(config.B[0][1]) <= tolerance &&
	       fabsf(config.B[0][2] - 0.5f) <= tolerance &&
	       fabsf(config.B[0][3]) <= tolerance &&
	       fabsf(config.B[1][0]) <= tolerance &&
	       fabsf(config.B[1][1] + 0.5f) <= tolerance &&
	       fabsf(config.B[1][2]) <= tolerance &&
	       fabsf(config.B[1][3] - 0.5f) <= tolerance &&
	       fabsf(config.B[2][0] - 0.25f) <= tolerance &&
	       fabsf(config.B[2][1] - 0.25f) <= tolerance &&
	       fabsf(config.B[2][2] - 0.25f) <= tolerance &&
	       fabsf(config.B[2][3] - 0.25f) <= tolerance;
}

bool
ControlAllocationMixer::is_shc09_physical_config(const Config &config)
{
	static constexpr float shc09_min = -0.6981f;
	static constexpr float shc09_max = 0.6981f;
	static constexpr float tolerance = 0.01f;

	if (config.y_dim != 3 || config.u_dim != 6) {
		return false;
	}

	for (unsigned actuator = 0; actuator < config.u_dim; actuator++) {
		if (fabsf(config.ranges[actuator].physical_min - shc09_min) > tolerance ||
		    fabsf(config.ranges[actuator].physical_max - shc09_max) > tolerance) {
			return false;
		}
	}

	return true;
}

bool
ControlAllocationMixer::build_ductedfan4_B(float k, float B[MAX_Y][MAX_U])
{
	static constexpr float I_x = 0.01149f;
	static constexpr float I_y = 0.01153f;
	static constexpr float I_z = 0.00487f;
	static constexpr float L_1 = 0.167f;
	static constexpr float L_2 = 0.069f;

	if (!PX4_ISFINITE(k) || k <= FLT_EPSILON) {
		return false;
	}

	for (unsigned row = 0; row < MAX_Y; row++) {
		for (unsigned col = 0; col < MAX_U; col++) {
			B[row][col] = 0.f;
		}
	}

	B[0][0] = -L_1 * k / I_x;
	B[0][2] =  L_1 * k / I_x;
	B[1][1] = -L_1 * k / I_y;
	B[1][3] =  L_1 * k / I_y;
	B[2][0] =  L_2 * k / I_z;
	B[2][1] =  L_2 * k / I_z;
	B[2][2] =  L_2 * k / I_z;
	B[2][3] =  L_2 * k / I_z;

	return true;
}

bool
ControlAllocationMixer::build_shc09_B(float k, float B[MAX_Y][MAX_U])
{
	static constexpr float I_x = 0.0438f;
	static constexpr float I_y = 0.0436f;
	static constexpr float I_z = 0.005006f;
	static constexpr float L_1 = 0.292166f;
	static constexpr float L_2 = 0.073699f;
	static constexpr float cos_60 = 0.5f;
	static constexpr float sin_60 = 0.8660254038f;

	if (!PX4_ISFINITE(k) || k <= FLT_EPSILON) {
		return false;
	}

	for (unsigned row = 0; row < MAX_Y; row++) {
		for (unsigned col = 0; col < MAX_U; col++) {
			B[row][col] = 0.f;
		}
	}

	B[0][0] = -L_1 * k / I_x;
	B[0][1] = -cos_60 * L_1 * k / I_x;
	B[0][2] =  cos_60 * L_1 * k / I_x;
	B[0][3] =  L_1 * k / I_x;
	B[0][4] =  cos_60 * L_1 * k / I_x;
	B[0][5] = -cos_60 * L_1 * k / I_x;

	B[1][1] =  sin_60 * L_1 * k / I_y;
	B[1][2] =  sin_60 * L_1 * k / I_y;
	B[1][4] = -sin_60 * L_1 * k / I_y;
	B[1][5] = -sin_60 * L_1 * k / I_y;

	for (unsigned actuator = 0; actuator < 6; actuator++) {
		B[2][actuator] = L_2 * k / I_z;
	}

	return true;
}

void
ControlAllocationMixer::update_runtime_config()
{
	update_feedback_params();

	static constexpr float df4_surface_limit_rad = 0.3491f;
	const bool df4_physical = is_ductedfan4_physical_config(_config);
	const bool df4_normalized = is_ductedfan4_normalized_config(_config);
	const bool shc09_physical = is_shc09_physical_config(_config);

	if (!df4_physical && !df4_normalized && !shc09_physical) {
		return;
	}

	update_runtime_limits((df4_physical || shc09_physical) ? 1.f : (1.f / df4_surface_limit_rad));

	if (!df4_physical && !shc09_physical) {
		return;
	}

	float k = NAN;

	if (_omega_2_force_param != PARAM_INVALID && param_get(_omega_2_force_param, &k) == 0 &&
	    PX4_ISFINITE(k) && (!PX4_ISFINITE(_last_omega_2_force) || fabsf(k - _last_omega_2_force) > FLT_EPSILON)) {
		update_physical_model_B(k);
	}
}

void
ControlAllocationMixer::update_runtime_limits(float dist_to_u_scale)
{
	int32_t dist_enable = 0;
	float dist_mag = 0.f;

	if (_dist_enable_param != PARAM_INVALID) {
		param_get(_dist_enable_param, &dist_enable);
	}

	if (_dist_mag_param != PARAM_INVALID) {
		param_get(_dist_mag_param, &dist_mag);
	}

	if (!PX4_ISFINITE(dist_mag) || dist_mag < 0.f) {
		dist_mag = 0.f;
	}

	if (dist_enable == _last_dist_enable && fabsf(dist_mag - _last_dist_mag) <= FLT_EPSILON) {
		return;
	}

	_last_dist_enable = dist_enable;
	_last_dist_mag = dist_mag;
	_runtime_disturbance_enabled = dist_enable == 1;
	_runtime_disturbance_mag_u = _runtime_disturbance_enabled ? dist_mag * dist_to_u_scale : 0.f;

	for (unsigned actuator = 0; actuator < _config.u_dim; actuator++) {
		ActuatorRange &range = _config.ranges[actuator];
		const float half_range = 0.5f * (range.physical_max - range.physical_min);
		const float margin = _runtime_disturbance_enabled ? math::constrain(_runtime_disturbance_mag_u, 0.f, half_range) : 0.f;

		range.active_min = range.physical_min + margin;
		range.active_max = range.physical_max - margin;
	}

	PX4_INFO("C mixer active limits updated dist=%d mag=%.4f", static_cast<int>(dist_enable),
		 static_cast<double>(dist_mag));
}

void
ControlAllocationMixer::update_physical_model_B(float k)
{
	Config updated_config = _config;
	bool updated = false;

	if (is_ductedfan4_physical_config(_config)) {
		updated = build_ductedfan4_B(k, updated_config.B);

	} else if (is_shc09_physical_config(_config)) {
		updated = build_shc09_B(k, updated_config.B);
	}

	if (!updated || !compute_pseudo_inverse(updated_config)) {
		return;
	}

	_config = updated_config;
	_last_omega_2_force = k;
	PX4_INFO("C mixer physical B updated k=%.4f", static_cast<double>(k));
}

float
ControlAllocationMixer::first_order_update_zoh(float u, float y_prev, float time_constant, float dt)
{
	if (time_constant < 1e-6f) {
		return u;
	}

	const float a = expf(-dt / time_constant);
	return a * y_prev + (1.f - a) * u;
}

bool
ControlAllocationMixer::valid_actuator_type(int actuator_type)
{
	return actuator_type == static_cast<int>(ActuatorType::Servo) ||
	       actuator_type == static_cast<int>(ActuatorType::Motor);
}

void
ControlAllocationMixer::update_feedback_params()
{
	float cutoff = 10.f;

	if (_cs_cutoff_param != PARAM_INVALID) {
		param_get(_cs_cutoff_param, &cutoff);
	}

	if (!PX4_ISFINITE(cutoff) || cutoff < 0.f) {
		cutoff = 0.f;
	}

	if (!PX4_ISFINITE(_last_cs_cutoff) || fabsf(cutoff - _last_cs_cutoff) > 0.1f) {
		for (unsigned i = 0; i < MAX_FEEDBACK; i++) {
			_feedback_filter[i].set_cutoff_frequency(_sample_freq, cutoff);
			_feedback_filter[i].reset(_delta_prev[i]);
		}

		_last_cs_cutoff = cutoff;
	}

	float servo_time_const = 0.03f;

	if (_time_const_param != PARAM_INVALID) {
		param_get(_time_const_param, &servo_time_const);
	}

	float motor_time_const = 0.01f;

	if (_motor_time_const_param != PARAM_INVALID) {
		param_get(_motor_time_const_param, &motor_time_const);
	}

	_servo_time_const = math::constrain(servo_time_const, 1.f / _sample_freq, 0.2f);
	_motor_time_const = math::constrain(motor_time_const, 1.f / _sample_freq, 0.2f);

	int32_t use_actuator = 0;

	if (_use_actuator_param != PARAM_INVALID) {
		param_get(_use_actuator_param, &use_actuator);
	}

	_use_actuator = use_actuator;
}

void
ControlAllocationMixer::update_sample_freq()
{
	const hrt_abstime now = hrt_absolute_time();

	if (_last_mix_timestamp != 0) {
		const float dt = math::constrain((now - _last_mix_timestamp) / 1e6f, 0.0001f, 0.02f);
		_sample_freq = 1.f / dt;
	}

	_last_mix_timestamp = now;
}

ControlAllocationMixer::Method
ControlAllocationMixer::selected_method() const
{
	int32_t method_override = -1;

	if (_method_param != PARAM_INVALID && param_get(_method_param, &method_override) == 0 &&
	    method_override >= static_cast<int32_t>(Method::INV) &&
	    method_override <= static_cast<int32_t>(Method::PCA)) {
		return static_cast<Method>(method_override);
	}

	return _config.method;
}

bool
ControlAllocationMixer::read_source(const ControlSource &source, float &input) const
{
	if (_control_allocation_cb == nullptr ||
	    _control_allocation_cb(_control_allocation_cb_handle, source.group, Mixer::ControlAllocationInput::Control,
				   source.index, input) != 0) {
		return false;
	}

	return true;
}

bool
ControlAllocationMixer::read_pca_inputs(const float *y, float *higher, float *lower) const
{
	if (_config.y_dim != 3) {
		return false;
	}

	float control_flag = 0.f;

	if (_control_allocation_cb == nullptr ||
	    _control_allocation_cb(_control_allocation_cb_handle, _config.sources[0].group,
				   Mixer::ControlAllocationInput::ControlFlag, 0, control_flag) != 0 ||
	    control_flag <= 0.5f) {
		for (unsigned i = 0; i < _config.y_dim; i++) {
			higher[i] = 0.f;
			lower[i] = y[i];
		}

		return true;
	}

	for (unsigned i = 0; i < _config.y_dim; i++) {
		uint8_t index = 0;

		if (!control_axis_from_index(_config.sources[i].index, index)) {
			return false;
		}

		if (_control_allocation_cb(_control_allocation_cb_handle, _config.sources[i].group,
					   Mixer::ControlAllocationInput::IndiFb, index, higher[i]) != 0) {
			return false;
		}

		if (_control_allocation_cb(_control_allocation_cb_handle, _config.sources[i].group,
					   Mixer::ControlAllocationInput::ErrorFb, index, lower[i]) != 0) {
			return false;
		}
	}

	return true;
}

float
ControlAllocationMixer::runtime_disturbance_bias(unsigned actuator_index) const
{
	static constexpr float df4_signs[4] {-1.f, 1.f, 1.f, -1.f};
	static constexpr float shc09_signs[6] {-1.f, 1.f, 1.f, 1.f, -1.f, -1.f};

	if (!_runtime_disturbance_enabled || _config.y_dim != 3) {
		return 0.f;
	}

	if (_config.u_dim == 4 && actuator_index < 4) {
		return df4_signs[actuator_index] * _runtime_disturbance_mag_u;
	}

	if (_config.u_dim == 6 && actuator_index < 6) {
		return shc09_signs[actuator_index] * _runtime_disturbance_mag_u;
	}

	return 0.f;
}

float
ControlAllocationMixer::apply_actuator_model(float u, unsigned actuator_index)
{
	if (actuator_index >= _config.u_dim) {
		return u;
	}

	const ActuatorRange &range = _config.ranges[actuator_index];
	const float time_const = (range.actuator_type == ActuatorType::Motor) ? _motor_time_const : _servo_time_const;

	_u_estimate[actuator_index] = first_order_update_zoh(u, _last_u[actuator_index], time_const,
					      1.f / _sample_freq);
	const float final_u = (_use_actuator == 1) ? _u_estimate[actuator_index] : u;
	_last_u[actuator_index] = final_u;

	return math::constrain(final_u, range.physical_min, range.physical_max);
}

float
ControlAllocationMixer::map_u_to_output(float u, unsigned actuator_index) const
{
	const ActuatorRange &range = _config.ranges[actuator_index];
	// Keep the u->PWM calibration fixed. Runtime active_min/max are allocation
	// constraints only; using them here would remap the shrunken range back to
	// full PWM and the real actuator would not be limited.
	const float denominator = range.physical_max - range.physical_min;

	if (denominator <= FLT_EPSILON) {
		return NAN;
	}

	const float output = 2.f * (u - range.physical_min) / denominator - 1.f;
	return math::constrain(output, -1.f, 1.f);
}

void
ControlAllocationMixer::publish_actuator_feedback()
{
	bool has_feedback = false;
	actuator_outputs_value_s msg {};

	for (unsigned actuator = 0; actuator < _config.u_dim; actuator++) {
		const ActuatorRange &range = _config.ranges[actuator];

		if (range.feedback_slot < 0 || range.feedback_slot >= static_cast<int8_t>(MAX_FEEDBACK)) {
			continue;
		}

		const unsigned slot = static_cast<unsigned>(range.feedback_slot);
		const float delta = math::constrain(_feedback_filter[slot].apply(_u_estimate[actuator]),
						    range.physical_min, range.physical_max);
		msg.delta[slot] = delta;
		_delta_prev[slot] = delta;
		has_feedback = true;
	}

	if (!has_feedback) {
		return;
	}

	msg.timestamp = hrt_absolute_time();
	_outputs_value_pub.publish(msg);
}

void
ControlAllocationMixer::publish_debug(const float *y, const float *u, const float *u_cmd, Method requested_method,
				      Method actual_method, bool fallback) const
{
	allocation_value_s msg {};
	msg.timestamp = hrt_absolute_time();
	msg.flag = static_cast<int8_t>(actual_method);
	msg.y_dim = _config.y_dim;
	msg.u_dim = _config.u_dim;
	msg.requested_method = static_cast<int8_t>(requested_method);
	msg.fallback = fallback ? 1 : 0;

	for (unsigned row = 0; row < _config.y_dim && row < 6; row++) {
		msg.y[row] = y[row];
	}

	for (unsigned row = 0; row < _config.y_dim && row < allocation_value_s::MAX_Y; row++) {
		for (unsigned actuator = 0; actuator < _config.u_dim && actuator < allocation_value_s::MAX_U; actuator++) {
			msg.b[row * allocation_value_s::MAX_U + actuator] = _config.B[row][actuator];
		}
	}

	for (unsigned row = 0; row < _config.y_dim && row < 3; row++) {
		float achieved = 0.f;

		for (unsigned actuator = 0; actuator < _config.u_dim; actuator++) {
			achieved += _config.B[row][actuator] * u[actuator];
		}

		msg.error[row] = y[row] - achieved;
	}

	for (unsigned actuator = 0; actuator < _config.u_dim && actuator < 16; actuator++) {
		const float constrained_u = math::constrain(u[actuator], _config.ranges[actuator].active_min,
					    _config.ranges[actuator].active_max);
		msg.u[actuator] = constrained_u;
		msg.u_ultimate[actuator] = u_cmd[actuator];
		msg.umin[actuator] = _config.ranges[actuator].active_min;
		msg.umax[actuator] = _config.ranges[actuator].active_max;
	}

	_allocation_value_pub.publish(msg);
}

void
ControlAllocationMixer::report_method(Method requested_method, Method actual_method, bool fallback)
{
	const int8_t requested = static_cast<int8_t>(requested_method);
	const int8_t actual = static_cast<int8_t>(actual_method);

	if (requested != _last_reported_requested_method ||
	    actual != _last_reported_actual_method ||
	    fallback != _last_reported_fallback) {
		PX4_INFO("C mixer method request=%d actual=%d%s", static_cast<int>(requested), static_cast<int>(actual),
			 fallback ? " fallback" : "");
		_last_reported_requested_method = requested;
		_last_reported_actual_method = actual;
		_last_reported_fallback = fallback;
	}
}
