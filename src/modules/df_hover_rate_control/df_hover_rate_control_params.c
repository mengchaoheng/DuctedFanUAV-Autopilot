/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

/**
 * Roll rate P gain
 *
 * Roll rate proportional gain used by df_hover_rate_control.
 *
 * @min 0.01
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_ROLLRATE_P, 0.15f);

/**
 * Roll rate I gain
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_ROLLRATE_I, 0.2f);

/**
 * Roll rate integrator limit
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_RR_INT_LIM, 0.30f);

/**
 * Roll rate D gain
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_ROLLRATE_D, 0.003f);

/**
 * Roll rate feedforward
 *
 * @min 0.0
 * @decimal 4
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_ROLLRATE_FF, 0.0f);

/**
 * Roll rate controller gain
 *
 * Global gain scaling the roll P, I and D terms.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_ROLLRATE_K, 1.0f);

/**
 * Pitch rate P gain
 *
 * Pitch rate proportional gain used by df_hover_rate_control.
 *
 * @min 0.01
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_PITCHRATE_P, 0.15f);

/**
 * Pitch rate I gain
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_PITCHRATE_I, 0.2f);

/**
 * Pitch rate integrator limit
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_PR_INT_LIM, 0.30f);

/**
 * Pitch rate D gain
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_PITCHRATE_D, 0.003f);

/**
 * Pitch rate feedforward
 *
 * @min 0.0
 * @decimal 4
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_PITCHRATE_FF, 0.0f);

/**
 * Pitch rate controller gain
 *
 * Global gain scaling the pitch P, I and D terms.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_PITCHRATE_K, 1.0f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain used by df_hover_rate_control.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_YAWRATE_P, 0.2f);

/**
 * Yaw rate I gain
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_YAWRATE_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_YR_INT_LIM, 0.30f);

/**
 * Yaw rate D gain
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_YAWRATE_D, 0.0f);

/**
 * Yaw rate feedforward
 *
 * @min 0.0
 * @decimal 4
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_YAWRATE_FF, 0.0f);

/**
 * Yaw rate controller gain
 *
 * Global gain scaling the yaw P, I and D terms.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_YAWRATE_K, 1.0f);

/**
 * Battery power level scaler
 *
 * Enables battery voltage scaling for df_hover_rate_control torque and thrust setpoints.
 *
 * @boolean
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_INT32(DF_BAT_SCALE_EN, 0);

/**
 * Low pass filter cutoff frequency for yaw torque setpoint
 *
 * 0 disables the filter.
 *
 * @min 0
 * @max 10
 * @unit Hz
 * @decimal 3
 * @group Ducted Fan Rate Control
 */
PARAM_DEFINE_FLOAT(DF_YAW_TQ_CUTOFF, 2.f);

/**
 * Acro mode maximum roll rate
 *
 * Full stick deflection leads to this rate in df_hover_rate_control.
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Ducted Fan Acro Mode
 */
PARAM_DEFINE_FLOAT(DF_ACRO_R_MAX, 100.f);

/**
 * Acro mode maximum pitch rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Ducted Fan Acro Mode
 */
PARAM_DEFINE_FLOAT(DF_ACRO_P_MAX, 100.f);

/**
 * Acro mode maximum yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Ducted Fan Acro Mode
 */
PARAM_DEFINE_FLOAT(DF_ACRO_Y_MAX, 100.f);

/**
 * Acro mode roll, pitch expo factor
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Ducted Fan Acro Mode
 */
PARAM_DEFINE_FLOAT(DF_ACRO_EXPO, 0.f);

/**
 * Acro mode yaw expo factor
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Ducted Fan Acro Mode
 */
PARAM_DEFINE_FLOAT(DF_ACRO_EXPO_Y, 0.f);

/**
 * Acro mode roll, pitch super expo factor
 *
 * @min 0
 * @max 0.95
 * @decimal 2
 * @group Ducted Fan Acro Mode
 */
PARAM_DEFINE_FLOAT(DF_ACRO_SUPEXPO, 0.f);

/**
 * Acro mode yaw super expo factor
 *
 * @min 0
 * @max 0.95
 * @decimal 2
 * @group Ducted Fan Acro Mode
 */
PARAM_DEFINE_FLOAT(DF_ACRO_SUPEXPOY, 0.f);

/**
 * INDI roll rate proportional gain
 *
 * Physical rate-error gain used before conversion to the normalized allocation
 * coordinates published by control_allocator.
 *
 * @min 0
 * @decimal 3
 * @group INDI Control
 */
PARAM_DEFINE_FLOAT(DF_INDI_R_P, 10.f);

/**
 * INDI pitch rate proportional gain
 *
 * Physical rate-error gain used before conversion to the normalized allocation
 * coordinates published by control_allocator.
 *
 * @min 0
 * @decimal 3
 * @group INDI Control
 */
PARAM_DEFINE_FLOAT(DF_INDI_P_P, 10.f);

/**
 * INDI yaw rate proportional gain
 *
 * Physical rate-error gain used before conversion to the normalized allocation
 * coordinates published by control_allocator.
 *
 * @min 0
 * @decimal 3
 * @group INDI Control
 */
PARAM_DEFINE_FLOAT(DF_INDI_Y_P, 10.f);

/**
 * Enable INDI rate control
 *
 * When enabled, df_hover_rate_control publishes the sum of INDI feedback and
 * rate-error feedback as the normalized torque setpoint.
 *
 * @boolean
 * @group INDI Control
 */
PARAM_DEFINE_INT32(DF_USE_INDI, 0);

/**
 * Enable INDI angular acceleration feedback
 *
 * @boolean
 * @group INDI Control
 */
PARAM_DEFINE_INT32(DF_USE_TAUI, 1);

/**
 * Enable INDI actuator feedback term
 *
 * @boolean
 * @group INDI Control
 */
PARAM_DEFINE_INT32(DF_USE_U, 1);
