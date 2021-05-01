/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "INDI.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;

INDI::INDI() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

INDI::~INDI()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool INDI::init()
{
	ScheduleOnInterval(1000_us); // 1000 us interval, 1000 Hz rate

	return true;
}

void INDI::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// const bool manual_control_updated = _manual_control_setpoint_sub.update(&_manual_control_setpoint);
	const bool rc_channels_updated = _rc_channels_sub.update(&_rc_channels);
	if (rc_channels_updated)
	{
		PX4_INFO("Hello INDI! %f.", (double) _rc_channels.channels[6]);
		// PX4_INFO("Hello INDI! %f, %f, %f, %f, %f.", (double) _rc_channels.channels[6], (double) _rc_channels.channels[7], (double) _rc_channels.channels[8], (double) _rc_channels.channels[9] ,(double) _rc_channels.channels[12]);//7,8,9,10,13
		//PX4_INFO("Hello INDI! %f, %f, %f, %f.", (double) _manual_control_setpoint.x, (double) _manual_control_setpoint.y, (double) _manual_control_setpoint.z, (double) _manual_control_setpoint.r);
		//PX4_INFO("Hello INDI! %f, %f, %f, %f.", (double) _manual_control_setpoint.flaps, (double) _manual_control_setpoint.aux1, (double) _manual_control_setpoint.aux2, (double) _manual_control_setpoint.aux3);
		//PX4_INFO("Hello INDI! %f, %f, %f.", (double) _manual_control_setpoint.aux4, (double) _manual_control_setpoint.aux5, (double) _manual_control_setpoint.aux6);

	}
	// DO WORK
	//printf("printf INDI\n");



	// Example
	// grab latest accelerometer data
	_sensor_accel_sub.update();
	const sensor_accel_s &accel = _sensor_accel_sub.get();


	// Example
	// publish some data
	orb_test_s data{};
	data.timestamp = hrt_absolute_time();
	data.val = accel.device_id;
	_orb_test_pub.publish(data);




	perf_end(_loop_perf);
}

int INDI::task_spawn(int argc, char *argv[])
{
	INDI *instance = new INDI();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int INDI::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int INDI::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int INDI::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
control module of INDI.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("indi", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int indi_main(int argc, char *argv[])
{
	return INDI::main(argc, argv);
}
