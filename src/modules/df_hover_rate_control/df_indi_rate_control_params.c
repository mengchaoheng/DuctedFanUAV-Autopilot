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
 * The controller only enters the INDI branch when allocation_value instance 1
 * provides recent complete roll, pitch, and yaw torque authority. Otherwise
 * df_hover_rate_control falls back to the standard rate controller for that
 * cycle. This parameter is intended for ducted-fan airframes using
 * df_hover_rate_control.
 *
 * This requires allocation_value to be published in physical units. PX4 does
 * not validate the numerical B values; the user must ensure CA_SV_CS*_TRQ_*
 * maps physical surface deflection in rad to angular acceleration in rad/s^2,
 * and allocation_value.u_ultimate_phys uses the matching physical actuator
 * units. Do not enable this with a normalized example allocation matrix.
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
