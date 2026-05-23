
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
 * USER_AC_METHOD
 *
 * Runtime method override for C: ControlAllocationMixer.
 *
 * @value -1 Use method from mix file
 * @value 0 INV
 * @value 1 WLS
 * @value 2 DIR
 * @value 3 PCA
 * @min -1
 * @max 3
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(USER_AC_METHOD, -1);

/**
 * USER_ADD_DIST
 *
 * Enable the C: ControlAllocationMixer test disturbance. This tightens the
 * allocation limits and adds the fixed ductedfan4 bias pattern after
 * allocation.
 *
 * @value 0 Disabled
 * @value 1 Enabled
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(USER_ADD_DIST, 0);

/**
* Low pass filter cutoff frequency for servos/control surfaces.
*
* The cutoff frequency for the 2nd order butterworth filter on virtual
* servo/control-surface feedback sent to controllers.
*
* A value of 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @group Mixer Output
*/
PARAM_DEFINE_FLOAT(USER_CS_CUTOFF, 10.0f);

/**
* Low pass filter cutoff frequency for motors.
*
* The cutoff frequency for the 2nd order butterworth filter on virtual motor
* feedback sent to controllers/logs.
*
* A value of 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @group Mixer Output
*/
PARAM_DEFINE_FLOAT(USER_MOT_CUTOFF, 30.0f);

/**
* USER_DIST_MAG
*
* Disturbance magnitude for the C: ControlAllocationMixer. For the ductedfan4
* physical control effectiveness matrix B this is radians.
*
* @min 0
* @max 0.3491
* @unit rad
* @decimal 4
* @increment 0.0001
* @group Mixer Output
*/
PARAM_DEFINE_FLOAT(USER_DIST_MAG, 0.0f);

/**
* USER_TIME_CONST
*
* Time constant of virtual servos/control surfaces.
*
* @min 0.0001
* @max 0.1
* @unit s
* @decimal 4
* @increment 0.0001
* @group Mixer Output
*/
PARAM_DEFINE_FLOAT(USER_TIME_CONST, 0.03f);

/**
* USER_MOT_TCONST
*
* Time constant of virtual motors.
*
* @min 0.0001
* @max 0.1
* @unit s
* @decimal 4
* @increment 0.0001
* @group Mixer Output
*/
PARAM_DEFINE_FLOAT(USER_MOT_TCONST, 0.01f);

/**
 * USER_ACTUATOR
 *
 * use ACTUATOR simulate or not.
 *
 * @value 0 without
 * @value 1 actuator simulation
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(USER_ACTUATOR, 0);
