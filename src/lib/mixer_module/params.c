
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
 * @value 0 use the default mix=inv
 * @value 1 use CA
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(USE_CA, 0);

/**
 * USE_PCA
 *
 * use priority control allocation or not.
 *
 * @value 0 use inv
 * @value 1 use PCA
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(USE_PCA, 0);

/**
 * USE_DIST
 *
 * use disturbance or not.
 *
 * @value 0 not use
 * @value 1 use USE_DISTURB
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(USE_DIST, 0);

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
PARAM_DEFINE_FLOAT(CS_CUTOFF, 10.0f);

/**
* DIST_MAG
*
* The magnitude of the disturbance added to the control surfaces.
*
* @min 0
* @max 0.3491
* @unit rad
* @reboot_required false
* @group Mixer Output
*/
PARAM_DEFINE_FLOAT(DIST_MAG, 0.0f);
