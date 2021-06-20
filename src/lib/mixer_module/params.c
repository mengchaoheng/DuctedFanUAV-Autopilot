
/**
 * Multicopter air-mode
 *
 * The air-mode enables the mixer to increase the total thrust of the multirotor
 * in order to keep attitude and rate control even at low and high throttle.
 *
 * This function should be disabled during tuning as it will help the controller
 * to diverge if the closed-loop is unstable (i.e. the vehicle is not tuned yet).
 *
 * Enabling air-mode for yaw requires the use of an arming switch.
 *
 * @value 0 Disabled
 * @value 1 Roll/Pitch
 * @value 2 Roll/Pitch/Yaw
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(MC_AIRMODE, 0);

/**
 * Motor Ordering
 *
 * Determines the motor ordering. This can be used for example in combination with
 * a 4-in-1 ESC that assumes a motor ordering which is different from PX4.
 *
 * ONLY supported for Quads.
 *
 * When changing this, make sure to test the motor response without props first.
 *
 * @value 0 PX4
 * @value 1 Betaflight / Cleanflight
 *
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(MOT_ORDERING, 0);

/**
 * PWM_DUCTEDFAN_MID1
 *
 * The PWM value of the midpoint of the servo.
 *
 * @min 1000
 * @max 2000
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(DUCTEDFAN_MID1, 1500);

/**
 * PWM_DUCTEDFAN_MID2
 *
 * The PWM value of the midpoint of the servo.
 *
 * @min 1000
 * @max 2000
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(DUCTEDFAN_MID2, 1500);

/**
 * USE_CA
 *
 * use control allocation or not.
 *
 * @value 0 not
 * @value 1 use CA
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(USE_CA, 0);

/**
 * USE_LPCA
 *
 * use LP control allocation or not.
 *
 * @value 0 not
 * @value 1 use LPCA
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(USE_LPCA, 0);

/**
 * PWM_HOVER
 *
 * The PWM value of the hover of the ductedfan.
 *
 * @min 1000
 * @max 2000
 * @group Mixer Output
 */
PARAM_DEFINE_FLOAT(MC_PWM_HOVER, 1500.f);

/**
 * OMEGA_HOVER
 *
 * The speed of the propeller when hovering of the ductedfan. rad/s
 *
 * @min 1000
 * @max 2000
 * @group Mixer Output
 */
PARAM_DEFINE_FLOAT(MC_OMEGA_HOVER, 1225.f);

/**
* Low pass filter cutoff frequency for control surface
*
* The cutoff frequency for the 2nd order butterworth filter on the primary gyro.
* This only affects the deflection angular sent to the controllers.
*
* A value of 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @reboot_required true
* @group Mixer Output
*/
PARAM_DEFINE_FLOAT(CS1_CUTOFF, 30.0f);

/**
* Low pass filter cutoff frequency for omega
*
* The cutoff frequency for the 2nd order butterworth filter on the primary gyro.
* This only affects the deflection angular sent to the controllers.
*
* A value of 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @reboot_required true
* @group Mixer Output
*/
PARAM_DEFINE_FLOAT(OMEGA_CUTOFF, 30.0f);

/**
* Low pass filter cutoff frequency for domega_d
*
* The cutoff frequency for the 2nd order butterworth filter on the primary gyro.
* This only affects the deflection angular sent to the controllers.
*
* A value of 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @reboot_required true
* @group Mixer Output
*/
PARAM_DEFINE_FLOAT(DOMEGA_D_CUTOFF, 30.0f);

/**
* Low pass filter cutoff frequency for domega_0
*
* The cutoff frequency for the 2nd order butterworth filter on the primary gyro.
* This only affects the deflection angular sent to the controllers.
*
* A value of 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @reboot_required true
* @group Mixer Output
*/
PARAM_DEFINE_FLOAT(DOMEGA_CUTOFF, 30.0f);

/**
 * sample_freq
 *
 * sample_freq of filter
 *
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(SAMPLE_FREQ, 200);
