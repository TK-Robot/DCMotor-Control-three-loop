/**
 * @file MechanicalModel.h
 * @brief Fixed-point shaft load torque estimator.
 * @brief 定点轴端负载力矩估算模型。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_MECHANICALMODEL_H
#define TRIPLE_CASCADECONTROLDCMOTOR_MECHANICALMODEL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t total_inertia_ug_cm2;      ///< Rotor plus reflected load inertia. / 转子及折算负载总惯量，ug·cm²。
    uint32_t coulomb_friction_uNm;      ///< Direction-independent friction magnitude. / 库仑摩擦幅值，uN·m。
    uint32_t viscous_friction_nNm_per_rpm; ///< Speed-dependent friction coefficient. / 粘性阻尼系数，nN·m/rpm。
    uint16_t friction_deadband_cps;     ///< No friction sign below this speed. / 摩擦方向判定死区，count/s。
} MechanicalModelParams;

typedef struct
{
    int32_t inertia_torque_uNm;
    int32_t coulomb_friction_torque_uNm;
    int32_t viscous_friction_torque_uNm;
    int32_t internal_loss_torque_uNm;
    int32_t shaft_load_torque_uNm;
    bool valid;
} MechanicalModelResult;

bool MechanicalModel_IsValid(const MechanicalModelParams *params,
                             uint16_t encoder_counts_per_rev);

void MechanicalModel_Evaluate(const MechanicalModelParams *params,
                              int32_t electromagnetic_torque_uNm,
                              int32_t speed_cps,
                              int32_t acceleration_cps2,
                              uint16_t encoder_counts_per_rev,
                              MechanicalModelResult *result);

int32_t MechanicalModel_CompensateShaftTarget(const MechanicalModelParams *params,
                                              int32_t shaft_target_torque_uNm,
                                              int32_t speed_cps,
                                              int32_t acceleration_cps2,
                                              uint16_t encoder_counts_per_rev);

#endif /* TRIPLE_CASCADECONTROLDCMOTOR_MECHANICALMODEL_H */
