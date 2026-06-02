/**
 * Author: Chaoheng Meng <chaohengmeng@163.com>
 */

/**
 * Enable ducted fan acceleration INDI
 *
 * When enabled, mc_pos_control keeps the PX4 position/velocity PID loop and
 * replaces the final acceleration-to-thrust conversion with an acceleration
 * INDI correction based on motor thrust feedback.
 *
 * This requires allocation_value to be published in physical units. PX4 does
 * not validate the numerical B values; the user must ensure the motor force row
 * uses physical thrust units with matching actuator scaling such as
 * CA_ROTOR0_CT = 1 and DF_MOT_MAX set to the maximum thrust. Do not enable this
 * with a normalized example allocation matrix.
 *
 * @boolean
 * @group Ducted Fan Acceleration INDI
 */
PARAM_DEFINE_INT32(DF_USE_ACC_INDI, 0);

/**
 * Vehicle mass for acceleration INDI
 *
 * Mass used in thrust_acc = F / mass. The thrust force is estimated with
 * F = omega^2 * k_T.
 *
 * @min 0.01
 * @unit kg
 * @decimal 4
 * @group Ducted Fan Acceleration INDI
 */
PARAM_DEFINE_FLOAT(DF_ACC_MASS, 2.1f);
