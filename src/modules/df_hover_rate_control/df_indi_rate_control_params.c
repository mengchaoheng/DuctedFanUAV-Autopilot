/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

/**
 * INDI roll rate proportional gain
 *
 * Physical rate-error gain used before conversion to the normalized allocation
 * coordinates published by control_allocator.
 *
 * @min 0
 * @decimal 3
 * @group Ducted Fan Rate INDI
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
 * @group Ducted Fan Rate INDI
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
 * @group Ducted Fan Rate INDI
 */
PARAM_DEFINE_FLOAT(DF_INDI_Y_P, 10.f);

/**
 * Enable INDI rate control
 *
 * When enabled, df_hover_rate_control publishes the sum of INDI feedback and
 * rate-error feedback as the normalized torque setpoint.
 *
 * @boolean
 * @group Ducted Fan Rate INDI
 */
PARAM_DEFINE_INT32(DF_USE_INDI, 0);

/**
 * Enable INDI angular acceleration feedback
 *
 * @boolean
 * @group Ducted Fan Rate INDI
 */
PARAM_DEFINE_INT32(DF_USE_TAUI, 1);

/**
 * Enable INDI actuator feedback term
 *
 * @boolean
 * @group Ducted Fan Rate INDI
 */
PARAM_DEFINE_INT32(DF_USE_U, 1);
