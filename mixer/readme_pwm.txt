systemcmds/mixer:

Load or append mixer files to the ESC driver.

run in ROMFS/px4fmu_common/init.d/rc.interface

using lib/mixer
-------------------------------------------------------------------------------------------------------------------------------------------------
systemcmds/pwm:

This command is used to configure PWM outputs for servo and ESC control.
The default device `/dev/pwm_output0` are the Main channels, AUX channels are on `/dev/pwm_output1` (`-d` parameter).
It is used in the startup script to make sure the PWM parameters (`PWM_*`) are applied (or the ones provided
by the airframe config if specified). `pwm info` shows the current settings (the trim value is an offset
and configured with `PWM_MAIN_TRIMx` and `PWM_AUX_TRIMx`).
-------------------------------------------------------------------------------------------------------------------------------------------------












1.lib/mixer:

lib of mixer and Group of mixers, built up from single mixers and processed in order when mixing.
-------------------------------------------------------------------------------------------------------------------------------------------------

2.lib/mixer_module:

This handles the mixing, arming/disarming and all subscriptions required for that.

using lib/mixer
-------------------------------------------------------------------------------------------------------------------------------------------------

3.drivers/pwm_out:

This module is responsible for driving the output and reading the input pins. For boards without a separate IO chip
(eg. Pixracer), it uses the main channels. On boards with an IO chip (eg. Pixhawk), it uses the AUX channels, and the
px4io driver is used for main ones. It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.
The module is configured via mode_* commands. This defines which of the first N pins the driver should occupy.
By using mode_pwm4 for example, pins 5 and 6 can be used by the camera trigger driver or by a PWM rangefinder
driver. Alternatively, pwm_out can be started in one of the capture modes, and then drivers can register a capture
callback with ioctl calls.

using lib/mixer_module

run in ROMFS/px4fmu_common/init.d/rc.interface
-------------------------------------------------------------------------------------------------------------------------------------------------

















drivers/px4io:

Encapsulates PX4FMU to PX4IO communications modeled as file operations.

using lib/mixer
-------------------------------------------------------------------------------------------------------------------------------------------------
