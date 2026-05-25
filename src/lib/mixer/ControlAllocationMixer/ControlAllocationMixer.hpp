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

#pragma once

#include <mixer/MixerBase/Mixer.hpp>

#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <parameters/param.h>
#include <stdint.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_outputs_value.h>
#include <uORB/topics/allocation_value.h>
#include <uORB/topics/vtol_vehicle_status.h>

/**
 * Control allocation mixer.
 * Besides replacing fixed mixer matrices with allocator backends, this mixer
 * is the common place for features tied to the allocated actuator vector:
 * virtual actuator simulation, filtered actuator feedback publication,
 * model-based runtime B reconstruction, active limit updates, and optional
 * actuator disturbances.
 *
 * Text format:
 *
 * C: <y_dim> <u_dim> <b_unit>
 * b_unit: 0 normalized/unitless B, 1 physical/dimensional B. INDI may only use
 * allocation_value.b and allocation_value.u_ultimate when b_unit is physical.
 * USER_AC_METHOD selects the allocation method at runtime:
 * 0 INV, 1 WLS, 2 DIR, 3 PCA.
 * USER_PINV_ALWAYS can force B_pinv recomputation every mix cycle. Leave it
 * disabled for the normal path where B_pinv updates only when B changes.
 * S: <group> <index>
 * ...
 * B: <b_row_0_col_0> ... <b_row_0_col_n>
 * ...
 * U: <physical_umin> <physical_umax> [output_slot] [feedback_slot] [actuator_type]
 * ...
 *
 * B is the control effectiveness matrix. The B lines in the mix file are still
 * required: they define the dimensions, provide a valid initial/equilibrium
 * matrix for parser-time checks and B_pinv initialization, and help identify
 * known physical airframe hooks. For supported physical-B aircraft the active B
 * is rebuilt from model constants, parameters, or vehicle state before
 * allocation; allocation_value.b publishes that active matrix so INDI and the
 * allocator use the same model.
 *
 * U defines the physical actuator range corresponding to the configured PWM
 * range. physical_umin maps to mixer output -1 and physical_umax maps to
 * mixer output +1. output_limit_calc() then maps mixer output to PWM. Runtime
 * allocation limits should constrain u, but must not rescale this physical
 * mapping. output_slot is local to this C: mixer in the MixerGroup output
 * stream; preceding M:/Z:/R: entries have already consumed their outputs.
 * feedback_slot is optional. Set it to 0..3 for a physical control-surface
 * deflection that should be published to actuator_outputs_value.delta[] for
 * INDI feedback. Omit it, or set it to -1, for motors or effectors that should
 * not publish control-surface feedback.
 * actuator_type is optional: 0 servo/control surface, 1 motor. It selects the
 * virtual actuator time constant for the outgoing command and the filtered
 * feedback estimate published in allocation_value.u_ultimate.
 *
 * The C: mixer always allocates the full u vector. When b_unit is physical it
 * also publishes the full active B matrix and full filtered u_feedback vector
 * for model-based controllers. The current rate INDI implementation assumes
 * B1 = B and u1 = u_feedback. Future model-specific INDI extensions can select
 * submatrices or rebuild a different B1 from the same airframe model.
 * This is a known limitation, not a property of the allocator itself: mixed
 * thrust/torque allocations, outer-loop INDI, or aircraft where only a subset
 * of actuators produces angular acceleration need a separate INDI mapping.
 *
 * Runtime B reconstruction, active limit shrinking, and actuator disturbance
 * signs are also model/layout-specific hooks. The current physical hooks cover
 * ductedfan4, ductedfan6, SHC09, SHW09, and SHW09_vtol. ductedfan2 currently
 * uses a normalized B in its mix file, so it intentionally does not publish a
 * physical INDI model or run dynamic physical-B updates yet.
 *
 * S lines define the y vector order. They do not imply normalized units:
 * B, y and U must use a consistent convention chosen by the mix file.
 * If the selected method has no backend for the current dimensions, the
 * mixer falls back to INV so the mix file remains usable while allocator
 * backends are generalized.
 */
class ControlAllocationMixer : public Mixer
{
public:
	static constexpr unsigned MAX_Y = 6;
	static constexpr unsigned MAX_U = 16;
	static constexpr unsigned MAX_OUTPUTS = 16;
	static constexpr unsigned MAX_FEEDBACK = 4;

		enum class Method : uint8_t {
			INV = 0,
			WLS = 1,
			DIR = 2,
			PCA = 3
		};

		enum class ActuatorType : uint8_t {
			Servo = 0,
			Motor = 1
		};

		enum class BUnit : uint8_t {
			Normalized = 0,
			Physical = 1
		};

	struct ControlSource {
		uint8_t group{0};
		uint8_t index{0};
	};

	struct ActuatorRange {
		// physical_min/max are the fixed u values corresponding to PWM_MIN/PWM_MAX.
		// They come from the mix file and must not change when allocation limits
		// are tightened at runtime.
		float physical_min{0.f};
		float physical_max{0.f};
		// active_min/max are solver constraints only. They can change at runtime,
		// but map_u_to_output() still uses physical_min/max so the PWM calibration
		// remains physically meaningful.
		float active_min{0.f};
		float active_max{0.f};
			uint8_t output_slot{0};
			int8_t feedback_slot{-1};
			ActuatorType actuator_type{ActuatorType::Servo};
		};

	struct Config {
		uint8_t y_dim{0};
		uint8_t u_dim{0};
		uint8_t output_count{0};
		Method method{Method::INV};
		BUnit b_unit{BUnit::Normalized};
		ControlSource sources[MAX_Y] {};
		ActuatorRange ranges[MAX_U] {};
		float B[MAX_Y][MAX_U] {};
		float B_pinv[MAX_U][MAX_Y] {};
	};

	class DpAllocator;

	ControlAllocationMixer(ControlAllocationCallback control_allocation_cb, uintptr_t cb_handle, const Config &config);
	virtual ~ControlAllocationMixer();

	ControlAllocationMixer(const ControlAllocationMixer &) = delete;
	ControlAllocationMixer &operator=(const ControlAllocationMixer &) = delete;
	ControlAllocationMixer(ControlAllocationMixer &&) = delete;
	ControlAllocationMixer &operator=(ControlAllocationMixer &&) = delete;

	static ControlAllocationMixer *from_text(Mixer::ControlAllocationCallback control_allocation_cb,
			uintptr_t cb_handle, const char *buf, unsigned &buflen);

	unsigned mix(float *outputs, unsigned space) override;
	void groups_required(uint32_t &groups) override;

	unsigned set_trim(float) override { return _config.output_count; }
	unsigned get_trim(float *trim) override;

private:
	static bool parse_b_row(const char *buf, unsigned columns, float *row);
	static bool compute_pseudo_inverse(Config &config);
	static bool allocate_inv(const Config &config, const float *y, float *u);
	static bool select_wls(const Config &config);
	static DpAllocator *create_dp_allocator(const Config &config);
	static void debug_b_pinv(const Config &config);
	static bool control_axis_from_index(uint8_t index, uint8_t &axis);
	static bool is_ductedfan4_physical_config(const Config &config);
	static bool is_ductedfan6_physical_config(const Config &config);
	static bool is_ductedfan4_normalized_config(const Config &config);
	static bool is_shc09_physical_config(const Config &config);
	static bool is_shw09_physical_config(const Config &config);
	static bool is_shw09_vtol_physical_config(const Config &config);
	static bool build_ductedfan4_B(float k, float B[MAX_Y][MAX_U]);
	static bool build_ductedfan6_B(float k, float B[MAX_Y][MAX_U]);
	static bool build_shc09_B(float k, float B[MAX_Y][MAX_U]);
	static bool build_shw09_B(float k, float B[MAX_Y][MAX_U]);
	static bool build_shw09_vtol_B(float k, float elevon_k, bool elevons_enabled, float B[MAX_Y][MAX_U]);
	static float first_order_update_zoh(float u, float y_prev, float time_constant, float dt);
	static bool valid_actuator_type(int actuator_type);

	void update_runtime_config();
	void update_runtime_limits(float dist_to_u_scale);
	bool update_shw09_vtol_elevon_state();
	void apply_shw09_vtol_elevon_limits();
	void update_physical_model_B(float k);
	void update_sample_freq();
	void update_feedback_params();
	Method selected_method() const;
	bool read_source(const ControlSource &source, float &input) const;
	bool allocate_wls(const float *y, float *u) const;
	bool read_pca_inputs(const float *y, float *higher, float *lower) const;
	float runtime_disturbance_bias(unsigned actuator_index) const;
	float apply_actuator_model(float u, unsigned actuator_index);
	float update_feedback_estimate(unsigned actuator_index);
	float map_u_to_output(float u, unsigned actuator_index) const;
	void publish_debug(const float *y, const float *u, const float *u_feedback, Method requested_method,
			   Method actual_method, bool fallback) const;
	void publish_actuator_feedback(const float *u_feedback);
	void report_method(Method requested_method, Method actual_method, bool fallback);

	Config _config {};
	ControlAllocationCallback _control_allocation_cb{nullptr};
	uintptr_t _control_allocation_cb_handle{0};
	param_t _method_param{PARAM_INVALID};
	param_t _dist_enable_param{PARAM_INVALID};
	param_t _dist_mag_param{PARAM_INVALID};
	param_t _omega_2_force_param{PARAM_INVALID};
	param_t _omega_2_force_fw_param{PARAM_INVALID};
	param_t _elevon_2_force_param{PARAM_INVALID};
	param_t _airframe_param{PARAM_INVALID};
	param_t _pinv_always_param{PARAM_INVALID};
	param_t _cs_cutoff_param{PARAM_INVALID};
	param_t _time_const_param{PARAM_INVALID};
	param_t _motor_time_const_param{PARAM_INVALID};
	param_t _use_actuator_param{PARAM_INVALID};
	int32_t _last_dist_enable{-1};
	float _last_dist_mag{NAN};
	bool _runtime_disturbance_enabled{false};
	float _runtime_disturbance_mag_u{0.f};
	float _last_omega_2_force{NAN};
	float _last_elevon_2_force{NAN};
	bool _model_ductedfan4_physical{false};
	bool _model_ductedfan6_physical{false};
	bool _model_ductedfan4_normalized{false};
	bool _model_shc09_physical{false};
	bool _model_shw09_physical{false};
	bool _model_shw09_vtol_physical{false};
	bool _shw09_vtol_elevons_enabled{false};
	float _last_cs_cutoff{NAN};
	float _last_feedback_sample_freq{NAN};
	float _servo_time_const{0.03f};
	float _motor_time_const{0.01f};
	int32_t _use_actuator{0};
	hrt_abstime _last_mix_timestamp{0};
	float _sample_freq{200.f};
	float _u_estimate[MAX_U] {};
	float _u_cmd[MAX_U] {};
	float _u_feedback[MAX_U] {};
	math::LowPassFilter2p<float> _u_feedback_filter[MAX_U] {};
	bool _wls_available{false};
	DpAllocator *_dp_allocator{nullptr};
	int8_t _last_reported_requested_method{-128};
	int8_t _last_reported_actual_method{-128};
	bool _last_reported_fallback{false};
	mutable uORB::PublicationMulti<allocation_value_s> _allocation_value_pub{ORB_ID(allocation_value)};
	uORB::Publication<actuator_outputs_value_s> _outputs_value_pub{ORB_ID(actuator_outputs_value)};
	uORB::Subscription _vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};
};
