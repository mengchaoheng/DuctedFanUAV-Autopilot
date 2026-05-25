
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
 * Runtime allocation method for C: ControlAllocationMixer.
 *
 * @value 0 INV
 * @value 1 WLS
 * @value 2 DIR
 * @value 3 PCA
 * @min 0
 * @max 3
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(USER_AC_METHOD, 0);

/**
 * USER_PINV_ALWAYS
 *
 * Force the C: ControlAllocationMixer to recompute B pseudo-inverse on every
 * mix cycle. Leave disabled to recompute only when the mix file is loaded or
 * the active physical B is rebuilt.
 *
 * @value 0 Recompute only when B changes
 * @value 1 Recompute every mix cycle
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(USER_PINV_ALWAYS, 0);

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
* USER_MOT_CUTOFF
*
* Low pass filter cutoff frequency for virtual motor feedback in the C:
* ControlAllocationMixer path.
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
* USER_THR_CUTOFF
*
* Low pass filter cutoff frequency for the first thrust motor feedback
* published by marked M: SimpleMixer outputs.
*
* This is intentionally separate from USER_MOT_CUTOFF, which is reserved for
* C: ControlAllocationMixer actuator feedback.
*
* A value of 0 disables the filter.
*
* @min 0
* @max 1000
* @unit Hz
* @group Mixer Output
*/
PARAM_DEFINE_FLOAT(USER_THR_CUTOFF, 30.0f);

/**
 * USER_OMEGA2F_MC
 *
 * Control-surface omega-to-force/effectiveness gain used by physical-B C:
 * mixers.
 *
 * Most ducted-fan physical-B models use only this parameter. SHW09_vtol uses
 * it in hover/transition and switches to USER_OMEGA2F_FW in fixed-wing mode.
 *
 * @min 0
 * @decimal 4
 * @group Mixer Output
 */
PARAM_DEFINE_FLOAT(USER_OMEGA2F_MC, 1.0f);

/**
 * USER_OMEGA2F_FW
 *
 * SHW09_vtol fixed-wing effectiveness for the bottom 1-6 ducted-fan control
 * surfaces used by the physical-B C: mixer.
 *
 * This parameter is paired with USER_OMEGA2F_MC and is only used when
 * SHW09_vtol is in fixed-wing mode.
 *
 * @min 0
 * @decimal 4
 * @group Mixer Output
 */
PARAM_DEFINE_FLOAT(USER_OMEGA2F_FW, 7.2f);

/**
 * USER_ELEV_2_F
 *
 * SHW09_vtol fixed-wing elevon effectiveness used by the physical-B C: mixer.
 *
 * This parameter only controls the 7/8 wing elevon columns. It is independent
 * of USER_OMEGA2F_MC/FW, which control the bottom ducted-fan control surfaces.
 *
 * @min 0
 * @decimal 4
 * @group Mixer Output
 */
PARAM_DEFINE_FLOAT(USER_ELEV_2_F, 4.0f);

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
* Time constant of virtual motors in the C: ControlAllocationMixer path.
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
* USER_THR_TCONST
*
* Time constant of the virtual first thrust motor used by marked M:
* SimpleMixer outputs.
*
* This is intentionally separate from USER_MOT_TCONST, which is reserved for
* C: ControlAllocationMixer actuator feedback.
*
* @min 0
* @max 0.1
* @unit s
* @decimal 4
* @increment 0.0001
* @group Mixer Output
*/
PARAM_DEFINE_FLOAT(USER_THR_TCONST, 0.01f);

/**
 * USER_ACTUATOR
 *
 * Enable virtual actuator dynamics in the SITL output path.
 *
 * This parameter is only effective in SITL/POSIX builds. Set it to 1 to send
 * the first-order actuator response to Gazebo, or 0 to send the mixer command
 * directly. It applies to C: ControlAllocationMixer actuators. Real flight
 * builds ignore it and always send the mixer
 * command; controller feedback still uses the internal actuator estimate.
 *
 * @value 0 send allocated command directly
 * @value 1 send first-order virtual actuator output
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(USER_ACTUATOR, 0);

/**
 * USER_THR_ACT
 *
 * Enable virtual first thrust motor dynamics in the SITL M: SimpleMixer output
 * path.
 *
 * This parameter is only effective in SITL/POSIX builds and only for marked
 * M: outputs such as "M: 1 0". Set it to 1 to send the first-order virtual
 * motor response to Gazebo, or 0 to send the mixer command directly. Real
 * flight builds ignore it and always send the mixer command; controller
 * feedback still uses the internal motor estimate.
 *
 * This is intentionally separate from USER_ACTUATOR, which is reserved for
 * C: ControlAllocationMixer actuator feedback.
 *
 * @value 0 send mixer command directly
 * @value 1 send first-order virtual motor output
 * @group Mixer Output
 */
PARAM_DEFINE_INT32(USER_THR_ACT, 0);
