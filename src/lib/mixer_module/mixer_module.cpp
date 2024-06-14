/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include "mixer_module.hpp"

#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>

#include <uORB/Publication.hpp>
#include <px4_platform_common/log.h>


#include "dir_alloc_sim.h"
#include "dir_alloc_six.h"
#include "allocator_dir_LPwrap_4.h"
#include "rt_nonfinite.h"
extern "C" {
    void allocator_dir_LPwrap_4(const float B[12], const float v[3],
                            const float umin[4], const float umax[4],
                            float u[4], float *z, unsigned int *iters);
}

using namespace time_literals;


MixingOutput::MixingOutput(uint8_t max_num_outputs, OutputModuleInterface &interface,
			   SchedulingPolicy scheduling_policy,
			   bool support_esc_calibration, bool ramp_up)
	: ModuleParams(&interface),
	  _control_subs{
	{&interface, ORB_ID(actuator_controls_0)},
	{&interface, ORB_ID(actuator_controls_1)},
	{&interface, ORB_ID(actuator_controls_2)},
	{&interface, ORB_ID(actuator_controls_3)},
	{&interface, ORB_ID(actuator_controls_4)},
	{&interface, ORB_ID(actuator_controls_5)},
},
_scheduling_policy(scheduling_policy),
_support_esc_calibration(support_esc_calibration),
_max_num_outputs(max_num_outputs < MAX_ACTUATORS ? max_num_outputs : MAX_ACTUATORS),
_interface(interface),
_control_latency_perf(perf_alloc(PC_ELAPSED, "control latency")),
df_4(_B, lower, upper),
Allocator(df_4)
{
	output_limit_init(&_output_limit);
	_output_limit.ramp_up = ramp_up;

	/* Safely initialize armed flags */
	_armed.armed = false;
	_armed.prearmed = false;
	_armed.ready_to_arm = false;
	_armed.lockdown = false;
	_armed.force_failsafe = false;
	_armed.in_esc_calibration_mode = false;

	px4_sem_init(&_lock, 0, 1);

	// Enforce the existence of the test_motor topic, so we won't miss initial publications
	test_motor_s test{};
	uORB::Publication<test_motor_s> test_motor_pub{ORB_ID(test_motor)};
	test_motor_pub.publish(test);
	_motor_test.test_motor_sub.subscribe();

	// filter init
	_sample_freq = (int) (1.0/((double) _param_cycle_time.get()/1e6));

	// last_delta_cmd_rad
	for (size_t i = 0; i < 4; ++i) {
		_lp_filter_actuator[i].set_cutoff_frequency(_sample_freq, _param_cs1_cutoff.get());
		_lp_filter_actuator[i].reset(0);

		_notch_filter_actuator[i].setParameters(_sample_freq, _param_imu_gyro_nf_freq.get(), _param_imu_gyro_nf_bw.get());
		_notch_filter_actuator[i].reset(0);
	}
	B_inv.setZero();
	B_inv(0, 0)=-0.0108f;
	B_inv(0, 2)=0.0053f;

	B_inv(1, 1)=-0.0109f;
	B_inv(1, 2)=0.0053f;

	B_inv(2, 0)=0.0108f;
	B_inv(2, 2)=0.0053f;

	B_inv(3, 1)=0.0109f;
	B_inv(3, 2)=0.0053f;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			B[i+j*3] = _B[i][j];
		}
	}

	for (size_t i = 0; i < 4; i++)
	{
		_uMin[i] = lower;
		_uMax[i] = upper;
		// _uMin_new[i] = -0.3491f;
		// _uMax_new[i] = 0.3491f;
	}
}

MixingOutput::~MixingOutput()
{
	perf_free(_control_latency_perf);
	delete _mixers;
	px4_sem_destroy(&_lock);
}
void MixingOutput::printStatus() const
{
	perf_print_counter(_control_latency_perf);
	PX4_INFO("Mixer loaded: %s", _mixers ? "yes" : "no");
	PX4_INFO("Switched to rate_ctrl work queue: %i", (int)_wq_switched);
	PX4_INFO("Driver instance: %i", _driver_instance);

	PX4_INFO("Channel Configuration:");

	for (unsigned i = 0; i < _max_num_outputs; i++) {
		int reordered_i = reorderedMotorIndex(i);
		PX4_INFO("Channel %i: value: %i, failsafe: %d, disarmed: %d, min: %d, max: %d", reordered_i, _current_output_value[i],
			 _failsafe_value[reordered_i], _disarmed_value[reordered_i], _min_value[reordered_i], _max_value[reordered_i]);
	}
}

void MixingOutput::CheckAndUpdateFilters()
{
	// update software low pass filters

	// last_delta_cmd_rad
	for (size_t i = 0; i < 4; ++i) {
		if ((fabsf(_lp_filter_actuator[i].get_cutoff_freq() - _param_cs1_cutoff.get()) > 0.1f)) {
			_lp_filter_actuator[i].set_cutoff_frequency(_sample_freq, _param_cs1_cutoff.get());
			_lp_filter_actuator[i].reset(_delta_prev[i]);

		}
		if ((fabsf(_notch_filter_actuator[i].getNotchFreq() - _param_imu_gyro_nf_freq.get()) > 0.1f)
		|| (fabsf(_notch_filter_actuator[i].getBandwidth() - _param_imu_gyro_nf_bw.get()) > 0.1f)
		) {
			_notch_filter_actuator[i].setParameters(_sample_freq, _param_imu_gyro_nf_freq.get(), _param_imu_gyro_nf_bw.get());
			_notch_filter_actuator[i].reset(_delta_prev[i]);
		}
	}
}

void MixingOutput::updateParams()
{
	ModuleParams::updateParams();

	CheckAndUpdateFilters();
	_use_indi = _param_use_indi.get();
	_use_alloc = _param_use_alloc.get();
	_use_pca = _param_use_pca.get();

	// update mixer if we have one
	if (_mixers) {
		if (_param_mot_slew_max.get() <= FLT_EPSILON) {
			_mixers->set_max_delta_out_once(0.f);
		}

		_mixers->set_thrust_factor(_param_thr_mdl_fac.get());
		_mixers->set_airmode((Mixer::Airmode)_param_mc_airmode.get());
	}
}

bool MixingOutput::updateSubscriptions(bool allow_wq_switch, bool limit_callbacks_to_primary)
{
	if (_groups_subscribed == _groups_required) {
		return false;
	}

	// must be locked to potentially change WorkQueue
	lock();

	if (_scheduling_policy == SchedulingPolicy::Auto) {
		// first clear everything
		unregister();
		_interface.ScheduleClear();

		// if subscribed to control group 0 or 1 then move to the rate_ctrl WQ
		const bool sub_group_0 = (_groups_required & (1 << 0));
		const bool sub_group_1 = (_groups_required & (1 << 1));

		if (allow_wq_switch && !_wq_switched && (sub_group_0 || sub_group_1)) {
			if (_interface.ChangeWorkQeue(px4::wq_configurations::rate_ctrl)) {
				// let the new WQ handle the subscribe update
				_wq_switched = true;
				_interface.ScheduleNow();
				unlock();
				return false;
			}
		}

		bool sub_group_0_callback_registered = false;
		bool sub_group_1_callback_registered = false;

		// register callback to all required actuator control groups
		for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {

			if (limit_callbacks_to_primary) {
				// don't register additional callbacks if actuator_controls_0 or actuator_controls_1 are already registered
				if ((i > 1) && (sub_group_0_callback_registered || sub_group_1_callback_registered)) {
					break;
				}
			}

			if (_groups_required & (1 << i)) {
				if (_control_subs[i].registerCallback()) {
					PX4_DEBUG("subscribed to actuator_controls_%d", i);

					if (limit_callbacks_to_primary) {
						if (i == 0) {
							sub_group_0_callback_registered = true;

						} else if (i == 1) {
							sub_group_1_callback_registered = true;
						}
					}

				} else {
					PX4_ERR("actuator_controls_%d register callback failed!", i);
				}
			}
		}

		// if nothing required keep periodic schedule (so the module can update other things)
		if (_groups_required == 0) {
			// TODO: this might need to be configurable depending on the module
			_interface.ScheduleOnInterval(100_ms);
		}
	}

	_groups_subscribed = _groups_required;
	setMaxTopicUpdateRate(_max_topic_update_interval_us);

	PX4_DEBUG("_groups_required 0x%08x", _groups_required);
	PX4_DEBUG("_groups_subscribed 0x%08x", _groups_subscribed);

	unlock();

	return true;
}

void MixingOutput::setMaxTopicUpdateRate(unsigned max_topic_update_interval_us)
{
	_max_topic_update_interval_us = max_topic_update_interval_us;

	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_groups_subscribed & (1 << i)) {
			_control_subs[i].set_interval_us(_max_topic_update_interval_us);
		}
	}
}

void MixingOutput::setAllMinValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_min_value[i] = value;
		// PX4_INFO("value %d", value);
	}
}

void MixingOutput::setAllMaxValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_max_value[i] = value;
	}
}

void MixingOutput::setAllFailsafeValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_failsafe_value[i] = value;
	}
}

void MixingOutput::setAllDisarmedValues(uint16_t value)
{
	for (unsigned i = 0; i < MAX_ACTUATORS; i++) {
		_disarmed_value[i] = value;
	}
}

void MixingOutput::unregister()
{
	for (auto &control_sub : _control_subs) {
		control_sub.unregisterCallback();
	}
}

void MixingOutput::updateOutputSlewrateMultirotorMixer()
{
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _time_last_dt_update_multicopter) / 1e6f, 0.0001f, 0.02f);
	_time_last_dt_update_multicopter = now;

	// maximum value the outputs of the multirotor mixer are allowed to change in this cycle
	// factor 2 is needed because actuator outputs are in the range [-1,1]
	const float delta_out_max = 2.0f * 1000.0f * dt / (_max_value[0] - _min_value[0]) / _param_mot_slew_max.get();
	_mixers->set_max_delta_out_once(delta_out_max);
}

void MixingOutput::updateOutputSlewrateSimplemixer()
{
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _time_last_dt_update_simple_mixer) / 1e6f, 0.0001f, 0.02f);
	_time_last_dt_update_simple_mixer = now;

	// set dt for slew rate limiter in SimpleMixer (is reset internally after usig it, so needs to be set on every update)
	_mixers->set_dt_once(dt);
}


unsigned MixingOutput::motorTest()
{
	test_motor_s test_motor;
	bool had_update = false;

	while (_motor_test.test_motor_sub.update(&test_motor)) {
		if (test_motor.driver_instance != _driver_instance ||
		    test_motor.timestamp == 0 ||
		    hrt_elapsed_time(&test_motor.timestamp) > 100_ms) {
			continue;
		}

		bool in_test_mode = test_motor.action == test_motor_s::ACTION_RUN;

		if (in_test_mode != _motor_test.in_test_mode) {
			// reset all outputs to disarmed on state change
			for (int i = 0; i < MAX_ACTUATORS; ++i) {
				_current_output_value[i] = _disarmed_value[i];
			}
		}

		if (in_test_mode) {
			int idx = test_motor.motor_number;

			if (idx < MAX_ACTUATORS) {
				if (test_motor.value < 0.f) {
					_current_output_value[reorderedMotorIndex(idx)] = _disarmed_value[idx];

				} else {
					_current_output_value[reorderedMotorIndex(idx)] =
						math::constrain<uint16_t>(_min_value[idx] + (uint16_t)((_max_value[idx] - _min_value[idx]) * test_motor.value),
									  _min_value[idx], _max_value[idx]);
				}
			}

			if (test_motor.timeout_ms > 0) {
				_motor_test.timeout = test_motor.timestamp + test_motor.timeout_ms * 1000;

			} else {
				_motor_test.timeout = 0;
			}
		}

		_motor_test.in_test_mode = in_test_mode;
		had_update = true;
	}

	// check for timeouts
	if (_motor_test.timeout != 0 && hrt_absolute_time() > _motor_test.timeout) {
		_motor_test.in_test_mode = false;
		_motor_test.timeout = 0;

		for (int i = 0; i < MAX_ACTUATORS; ++i) {
			_current_output_value[i] = _disarmed_value[i];
		}

		had_update = true;
	}

	return (_motor_test.in_test_mode || had_update) ? _max_num_outputs : 0;
}

bool MixingOutput::update()
{
	if (!_mixers) {
		handleCommands();
		// do nothing until we have a valid mixer
		return false;
	}

	// check arming state
	if (_armed_sub.update(&_armed)) {
		_armed.in_esc_calibration_mode &= _support_esc_calibration;

		if (_ignore_lockdown) {
			_armed.lockdown = false;
		}

		/* Update the armed status and check that we're not locked down.
		 * We also need to arm throttle for the ESC calibration. */
		_throttle_armed = (_armed.armed && !_armed.lockdown) || _armed.in_esc_calibration_mode;

		if (_armed.armed) {
			_motor_test.in_test_mode = false;
		}
	}

	// check for motor test
	if (!_armed.armed && !_armed.manual_lockdown) {
		unsigned num_motor_test = motorTest();

		if (num_motor_test > 0) {
			if (_interface.updateOutputs(false, _current_output_value, num_motor_test, 1)) {
				actuator_outputs_s actuator_outputs{};
				setAndPublishActuatorOutputs(num_motor_test, actuator_outputs);
			}

			handleCommands();
			return true;
		}
	}

	if (_param_mot_slew_max.get() > FLT_EPSILON) {
		updateOutputSlewrateMultirotorMixer();
	}

	updateOutputSlewrateSimplemixer(); // update dt for output slew rate in simple mixer

	unsigned n_updates = 0;
	//_controls is roll, pitch, yaw moment[-1, 1] and thrust[0, 1], which is output of rate_control
	/* get controls for required topics */
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_groups_subscribed & (1 << i)) {
			if (_control_subs[i].copy(&_controls[i])) {
				n_updates++;
			}

			/* During ESC calibration, we overwrite the throttle value. */
			if (i == 0 && _armed.in_esc_calibration_mode) {

				/* Set all controls to 0 */
				memset(&_controls[i], 0, sizeof(_controls[i]));

				/* except thrust to maximum. */
				_controls[i].control[actuator_controls_s::INDEX_THROTTLE] = 1.0f;

				/* Switch off the output limit ramp for the calibration. */
				_output_limit.state = OUTPUT_LIMIT_STATE_ON;
			}
		}
	}

	/* do mixing */
	float outputs[MAX_ACTUATORS] {};
	const unsigned mixed_num_outputs = _mixers->mix(outputs, _max_num_outputs);

	// // "outputs" is the value alfter mix, range from [-1, 1]. _current_output_value is pwm value alfter output_limit_calc.
	// // just using in ductedfan
	// // PX4_INFO("dir_alloc_sim:\n");
	uint64_t timestamp_ca_start=hrt_absolute_time();
	uint64_t timestamp_ca_end=hrt_absolute_time();
	allocation_value_s allocation_value{};
	if (_use_alloc == 1 || _use_indi == 1) // indi have to use allocator, since it use the model for control value
	{
		_fb[0] = _controls[0].control[actuator_controls_s::INDEX_ROLL];
		_fb[1] = _controls[0].control[actuator_controls_s::INDEX_PITCH];
		_fb[2] = _controls[0].control[actuator_controls_s::INDEX_YAW];

		if (_use_pca==1)
		{
			// if(_use_indi == 1)
			// {

			// }
			float u_all[4];
			int err = 0;
			float rho;

			Allocator.DP_LPCA(_fb,u_all,err, rho); // what happen with pid? allocation_value.flag=1;
			// Allocator.DPscaled_LPCA(_fb, u_all, err, rho);

			if(rho<1 && _use_indi == 1)
			// if(0)
			{


				_indi_fb[0] = _controls[0].indi_fb[actuator_controls_s::INDEX_ROLL];
				_indi_fb[1] = _controls[0].indi_fb[actuator_controls_s::INDEX_PITCH];
				_indi_fb[2] = _controls[0].indi_fb[actuator_controls_s::INDEX_YAW];
				float u_e[4] = {0.0, 0.0, 0.0, 0.0};
				int err_e = 0;
				float rho_e;
				Allocator.DP_LPCA(_indi_fb,u_e,err_e, rho_e);
				for (size_t i = 0; i < 3; i++)
				{
					float  temp = 0.0f;
					for(int k = 0 ; k < 4 ; k++)
					{
						temp += _B[i][k] * u_e[k];
					}
					allocation_value.ue_error[i] =_indi_fb[i] - temp;
				}
				if (rho_e<1)
				{
					for (size_t i = 0; i < 4; i++)
					{
						_u[i] = math::constrain((float) u_all[i], (float) (_uMin[i]), (float) (_uMax[i]));
					}
					// PX4_INFO("dir 2");
					allocation_value.flag=2;
				}
				else
				{
					_error_fb[0] = _controls[0].error_fb[actuator_controls_s::INDEX_ROLL];
					_error_fb[1] = _controls[0].error_fb[actuator_controls_s::INDEX_PITCH];
					_error_fb[2] = _controls[0].error_fb[actuator_controls_s::INDEX_YAW];
					// float uMin_new[4];
					// float uMax_new[4];
					// for (size_t i = 0; i < 4; i++)
					// {
					// 	uMin_new[i] = _uMin[i] - u_e[i];
					// 	uMax_new[i] = _uMax[i] - u_e[i];
					// }

					float u_d[4] = {0.0, 0.0, 0.0, 0.0};
					int err_d = 0;
					float rho_d;
					// change limits
					for (int i = 0; i < 4; ++i) {
						Allocator.aircraft.upperLimits[i] = _uMax[i] - u_e[i];
						Allocator.aircraft.lowerLimits[i] = _uMin[i] - u_e[i];
					}

					Allocator.DP_LPCA(_error_fb,u_d,err_d, rho_d);

					// change limits
					for (int i = 0; i < 4; ++i) {
						Allocator.aircraft.upperLimits[i] = _uMax[i];
						Allocator.aircraft.lowerLimits[i] = _uMin[i];
					}
					for (size_t i = 0; i < 3; i++)
					{
						float  temp = 0.0f;
						for(int k = 0 ; k < 4 ; k++)
						{
							temp += _B[i][k] * u_d[k];
						}
						allocation_value.ud_error[i] =_error_fb[i] - temp;
					}
					for (size_t i = 0; i < 4; i++)
					{
						_u[i] = math::constrain((float) (u_d[i] + u_e[i]), (float) (_uMin[i]), (float) (_uMax[i]));
					}
					PX4_INFO("dir 3");
					allocation_value.flag=3;
				}
			}
			else
			{
				for (size_t i = 0; i < 4; i++)
				{
					_u[i] =  math::constrain( u_all[i], _uMin[i], _uMax[i]);

				}
				allocation_value.flag=1;
				// PX4_INFO(" dir 1");
			}

		}
		else
		{
			//inv
			// PX4_INFO("inv");
			matrix::Matrix<float, 3, 1> y_desire (_fb);
			matrix::Matrix<float, 4, 1> u_inv = B_inv * y_desire;
			for (size_t i = 0; i < 4; i++)
			{
				_u[i] =  math::constrain( u_inv(i,0), _uMin[i], _uMax[i]);
			}
			allocation_value.flag=-1;
		}
		timestamp_ca_end = hrt_absolute_time();
		// PX4_INFO("alloc time: %lld \n", (timestamp_ca_end - timestamp_ca_start) ); //nuttx

		for (size_t i = 0; i < 3; i++)
		{
			float  temp = 0.0f;
			for(int k = 0 ; k < 4 ; k++)
			{
				temp += _B[i][k] * _u[k];
			}
			allocation_value.error[i] =_fb[i] - temp;
		}

		for (size_t i = 0; i < 4; i++)
		{
			allocation_value.u[i] = _u[i];
			allocation_value.umin[i] = _uMin[i];
			allocation_value.umax[i] = _uMax[i];
		}
		for (size_t i = 0; i < 4; i++)
		{
			outputs[i+4] = (_u[i])/0.3491f;
		}

	}
	else
	{
		for (size_t i = 0; i < 4; i++)
		{
			_u[i] = outputs[i+4];
		}
	}
	allocation_value.timestamp = (timestamp_ca_end - timestamp_ca_start);//hrt_absolute_time();
	_allocation_value_pub.publish(allocation_value);

	/* the output limit call takes care of out of band errors, NaN and constrains */ // [-1, 1] -> [min_rad, max_rad] == [min_pwm, max_pwm]
	output_limit_calc(_throttle_armed, armNoThrottle(), mixed_num_outputs, _reverse_output_mask,
			  _disarmed_value, _min_value, _max_value, outputs, _current_output_value, &_output_limit);

	/* overwrite outputs in case of force_failsafe with _failsafe_value values */
	if (_armed.force_failsafe) {
		for (size_t i = 0; i < mixed_num_outputs; i++) {
			_current_output_value[i] = _failsafe_value[i];
		}
	}

	bool stop_motors = mixed_num_outputs == 0 || !_throttle_armed;

	/* overwrite outputs in case of lockdown or parachute triggering with disarmed values */
	if (_armed.lockdown || _armed.manual_lockdown) {
		for (size_t i = 0; i < mixed_num_outputs; i++) {
			_current_output_value[i] = _disarmed_value[i];
		}

		stop_motors = true;
	}

	/* apply _param_mot_ordering */
	reorderOutputs(_current_output_value);

	//updateParams();
	// _current_output_value[1] = _param_ductedfan_mid1.get();
	//_current_output_value[2] = _param_ductedfan_mid2.get();



	/* now return the outputs to the driver */
	if (_interface.updateOutputs(stop_motors, _current_output_value, mixed_num_outputs, n_updates)) {
		actuator_outputs_s actuator_outputs{};
		setAndPublishActuatorOutputs(mixed_num_outputs, actuator_outputs);

		publishMixerStatus(actuator_outputs);
		updateLatencyPerfCounter(actuator_outputs);
	}



	handleCommands();

	return true;
}

void
MixingOutput::setAndPublishActuatorOutputs(unsigned num_outputs, actuator_outputs_s &actuator_outputs)
{
	actuator_outputs.noutputs = num_outputs;

	for (size_t i = 0; i < num_outputs; ++i) {
		actuator_outputs.output[i] = _current_output_value[i];
	}

	actuator_outputs.timestamp = hrt_absolute_time();
	_outputs_pub.publish(actuator_outputs);

	actuator_outputs_value_s actuator_outputs_value{};
	//control surfaces, 0.3491 rad -> 180pwm.
	for (size_t i = 0; i < 4 ; ++i) {
		actuator_outputs_value.last_deltacmd[i] =(float) _last_u[i];
	}
	// actuator filtering:
	// - Apply general notch filter (IMU_GYRO_NF_FREQ)
	// - Apply general low-pass filter (IMU_GYRO_CUTOFF)
	// - Differentiate & apply specific angular acceleration (D-term) low-pass (IMU_DGYRO_CUTOFF)
	float actuator_notched[4];
	//control surfaces
	for (size_t i = 0; i < 4; ++i) {
		actuator_notched[i] = _notch_filter_actuator[i].apply(actuator_outputs_value.last_deltacmd[i]);
		actuator_outputs_value.delta[i] = math::constrain(_lp_filter_actuator[i].apply(actuator_notched[i]), (float) (_uMin[i]), (float) (_uMax[i]));// 100%
		_delta_prev[i] = actuator_outputs_value.delta[i];
	}
	//------------------------publish-----------------------------
	actuator_outputs_value.timestamp = hrt_absolute_time();
	// _last_config_update = actuator_outputs_value.timestamp;
	_outputs_value_pub.publish(actuator_outputs_value);

	//update _last_output_value
	_last_output_value[0] = _current_output_value[0];
	for (size_t i = 0; i < 4; ++i) {
		_last_u[i] = (double) _u[i];
	}

}

void
MixingOutput::publishMixerStatus(const actuator_outputs_s &actuator_outputs)
{
	MultirotorMixer::saturation_status saturation_status;
	saturation_status.value = _mixers->get_saturation_status();

	if (saturation_status.flags.valid) {
		multirotor_motor_limits_s motor_limits;
		motor_limits.timestamp = actuator_outputs.timestamp;
		motor_limits.saturation_status = saturation_status.value;

		_to_mixer_status.publish(motor_limits);
	}
}

void
MixingOutput::updateLatencyPerfCounter(const actuator_outputs_s &actuator_outputs)
{
	// use first valid timestamp_sample for latency tracking
	for (int i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		const bool required = _groups_required & (1 << i);
		const hrt_abstime &timestamp_sample = _controls[i].timestamp_sample;

		if (required && (timestamp_sample > 0)) {
			perf_set_elapsed(_control_latency_perf, actuator_outputs.timestamp - timestamp_sample);
			break;
		}
	}
}

void
MixingOutput::reorderOutputs(uint16_t values[MAX_ACTUATORS])
{
	if (MAX_ACTUATORS < 4) {
		return;
	}

	if ((MotorOrdering)_param_mot_ordering.get() == MotorOrdering::Betaflight) {
		/*
		 * Betaflight default motor ordering:
		 * 4     2
		 *    ^
		 * 3     1
		 */
		const uint16_t value_tmp[4] = {values[0], values[1], values[2], values[3] };
		values[0] = value_tmp[3];
		values[1] = value_tmp[0];
		values[2] = value_tmp[1];
		values[3] = value_tmp[2];
	}

	/* else: PX4, no need to reorder
	 * 3     1
	 *    ^
	 * 2     4
	 */
}

int MixingOutput::reorderedMotorIndex(int index) const
{
	if ((MotorOrdering)_param_mot_ordering.get() == MotorOrdering::Betaflight) {
		switch (index) {
		case 0: return 1;

		case 1: return 2;

		case 2: return 3;

		case 3: return 0;
		}
	}

	return index;
}

int MixingOutput::controlCallback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input)
{
	const MixingOutput *output = (const MixingOutput *)handle;

	input = output->_controls[control_group].control[control_index];
	/* limit control input */
	input = math::constrain(input, -1.f, 1.f);//already done
	/* motor spinup phase - lock throttle to zero */
	if (output->_output_limit.state == OUTPUT_LIMIT_STATE_RAMP) {
		if (((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		      control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		     control_index == actuator_controls_s::INDEX_THROTTLE) ||
		    (control_group == actuator_controls_s::GROUP_INDEX_ALLOCATED_PART1 ||
		     control_group == actuator_controls_s::GROUP_INDEX_ALLOCATED_PART2)) {
			/* limit the throttle output to zero during motor spinup,
			 * as the motors cannot follow any demand yet
			 */
			input = 0.0f;
		}
	}

	/* throttle not arming - mark throttle input as invalid */
	if (output->armNoThrottle() && !output->_armed.in_esc_calibration_mode) {
		if (((control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE ||
		      control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE_ALTERNATE) &&
		     control_index == actuator_controls_s::INDEX_THROTTLE) ||
		    (control_group == actuator_controls_s::GROUP_INDEX_ALLOCATED_PART1 ||
		     control_group == actuator_controls_s::GROUP_INDEX_ALLOCATED_PART2)) {
			/* set the throttle to an invalid value */
			input = NAN;
		}
	}

	return 0;
}

void MixingOutput::resetMixer()
{
	if (_mixers != nullptr) {
		delete _mixers;
		_mixers = nullptr;
		_groups_required = 0;
	}

	_interface.mixerChanged();
}

int MixingOutput::loadMixer(const char *buf, unsigned len)
{
	if (_mixers == nullptr) {
		_mixers = new MixerGroup();
	}

	if (_mixers == nullptr) {
		_groups_required = 0;
		return -ENOMEM;
	}

	int ret = _mixers->load_from_buf(controlCallback, (uintptr_t)this, buf, len);

	if (ret != 0) {
		PX4_ERR("mixer load failed with %d", ret);
		delete _mixers;
		_mixers = nullptr;
		_groups_required = 0;
		return ret;
	}

	_mixers->groups_required(_groups_required);
	PX4_DEBUG("loaded mixers \n%s\n", buf);

	updateParams();
	_interface.mixerChanged();
	return ret;
}

void MixingOutput::handleCommands()
{
	if ((Command::Type)_command.command.load() == Command::Type::None) {
		return;
	}

	switch ((Command::Type)_command.command.load()) {
	case Command::Type::loadMixer:
		_command.result = loadMixer(_command.mixer_buf, _command.mixer_buf_length);
		break;

	case Command::Type::resetMixer:
		resetMixer();
		_command.result = 0;
		break;

	default:
		break;
	}

	// mark as done
	_command.command.store((int)Command::Type::None);
}

void MixingOutput::resetMixerThreadSafe()
{
	if ((Command::Type)_command.command.load() != Command::Type::None) {
		// Cannot happen, because we expect only one other thread to call this.
		// But as a safety precaution we return here.
		PX4_ERR("Command not None");
		return;
	}

	lock();

	_command.command.store((int)Command::Type::resetMixer);

	_interface.ScheduleNow();

	unlock();

	// wait until processed
	while ((Command::Type)_command.command.load() != Command::Type::None) {
		usleep(1000);
	}

}

int MixingOutput::loadMixerThreadSafe(const char *buf, unsigned len)
{
	if ((Command::Type)_command.command.load() != Command::Type::None) {
		// Cannot happen, because we expect only one other thread to call this.
		// But as a safety precaution we return here.
		PX4_ERR("Command not None");
		return -1;
	}

	lock();

	_command.mixer_buf = buf;
	_command.mixer_buf_length = len;
	_command.command.store((int)Command::Type::loadMixer);

	_interface.ScheduleNow();

	unlock();

	// wait until processed
	while ((Command::Type)_command.command.load() != Command::Type::None) {
		usleep(1000);
	}

	return _command.result;
}
