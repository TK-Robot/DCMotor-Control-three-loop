/**
 * @file MechanicalModel.c
 * @brief Fixed-point shaft load torque estimator.
 * @brief 定点轴端负载力矩估算模型。
 */

#include "MechanicalModel.h"
#include "FixedPointMath.h"

#include <limits.h>
#include <stddef.h>

#define MODEL_TWO_PI_MILLI 6283LL

static int32_t MechanicalModel_SaturateI32(int64_t value)
{
    if (value > INT32_MAX) return INT32_MAX;
    if (value < INT32_MIN) return INT32_MIN;
    return (int32_t)value;
}

static int64_t MechanicalModel_AbsI64(int64_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t MechanicalModel_ClampForProduct(int32_t value,
                                               uint32_t coefficient,
                                               uint32_t multiplier)
{
    int64_t limit;

    if (coefficient == 0U || multiplier == 0U) return 0;
    limit = FixedPoint_DivideS64ByU32ToS64(INT64_MAX, coefficient);
    limit = FixedPoint_DivideS64ByU32ToS64(limit, multiplier);
    if ((int64_t)value > limit) return (int32_t)limit;
    if ((int64_t)value < -limit) return (int32_t)-limit;
    return value;
}

bool MechanicalModel_IsValid(const MechanicalModelParams *params,
                             uint16_t encoder_counts_per_rev)
{
    return params != NULL && encoder_counts_per_rev != 0U;
}

void MechanicalModel_Evaluate(const MechanicalModelParams *params,
                              int32_t electromagnetic_torque_uNm,
                              int32_t speed_cps,
                              int32_t acceleration_cps2,
                              uint16_t encoder_counts_per_rev,
                              MechanicalModelResult *result)
{
    int64_t inertia_torque;
    int64_t viscous_torque;
    int64_t coulomb_torque = 0;
    int64_t internal_loss;
    int64_t shaft_load;
    int32_t bounded_acceleration;
    int32_t bounded_speed;

    if (result == NULL) return;
    *result = (MechanicalModelResult){0};
    result->valid = MechanicalModel_IsValid(params, encoder_counts_per_rev);
    if (!result->valid) return;

    bounded_acceleration = MechanicalModel_ClampForProduct(
        acceleration_cps2, params->total_inertia_ug_cm2,
        (uint32_t)MODEL_TWO_PI_MILLI);
    bounded_speed = MechanicalModel_ClampForProduct(
        speed_cps, params->viscous_friction_nNm_per_rpm, 60U);

    /* ug·cm² and count/s² are converted to uN·m using 2*pi radians/revolution. */
    inertia_torque = FixedPoint_DivideS64ByU32ToS64(
        (int64_t)params->total_inertia_ug_cm2
            * bounded_acceleration * MODEL_TWO_PI_MILLI,
        encoder_counts_per_rev);
    inertia_torque = FixedPoint_DivideS64ByU32ToS64(inertia_torque, 100000U);
    inertia_torque = FixedPoint_DivideS64ByU32ToS64(inertia_torque, 100000U);
    viscous_torque = FixedPoint_DivideS64ByU32(
        (int64_t)params->viscous_friction_nNm_per_rpm * bounded_speed * 60LL,
        (uint32_t)encoder_counts_per_rev * 1000UL);

    if (MechanicalModel_AbsI64(speed_cps) > params->friction_deadband_cps)
    {
        coulomb_torque = (speed_cps > 0)
                       ? params->coulomb_friction_uNm
                       : -(int64_t)params->coulomb_friction_uNm;
    }

    internal_loss = coulomb_torque + viscous_torque;
    shaft_load = (int64_t)electromagnetic_torque_uNm
               - inertia_torque - internal_loss;
    result->inertia_torque_uNm = MechanicalModel_SaturateI32(inertia_torque);
    result->coulomb_friction_torque_uNm = MechanicalModel_SaturateI32(coulomb_torque);
    result->viscous_friction_torque_uNm = MechanicalModel_SaturateI32(viscous_torque);
    result->internal_loss_torque_uNm = MechanicalModel_SaturateI32(internal_loss);
    result->shaft_load_torque_uNm = MechanicalModel_SaturateI32(shaft_load);
}

int32_t MechanicalModel_CompensateShaftTarget(const MechanicalModelParams *params,
                                              int32_t shaft_target_torque_uNm,
                                              int32_t speed_cps,
                                              int32_t acceleration_cps2,
                                              uint16_t encoder_counts_per_rev)
{
    MechanicalModelResult losses;
    int64_t electromagnetic_target;
    int64_t static_friction = 0;

    MechanicalModel_Evaluate(params, 0, speed_cps, acceleration_cps2,
                             encoder_counts_per_rev, &losses);
    if (!losses.valid) return shaft_target_torque_uNm;

    /* Inside the speed deadband there is no measured motion direction yet.
     * Use the requested shaft-torque direction for breakaway compensation;
     * a zero torque request remains exactly zero and cannot self-start. */
    if (MechanicalModel_AbsI64(speed_cps) <= params->friction_deadband_cps)
    {
        if (shaft_target_torque_uNm > 0)
            static_friction = params->coulomb_friction_uNm;
        else if (shaft_target_torque_uNm < 0)
            static_friction = -(int64_t)params->coulomb_friction_uNm;
    }
    electromagnetic_target = (int64_t)shaft_target_torque_uNm
                           + losses.inertia_torque_uNm
                           + losses.internal_loss_torque_uNm
                           + static_friction;
    return MechanicalModel_SaturateI32(electromagnetic_target);
}
