/**
 * @file MotorTorqueModel.h
 * @brief Fixed-point brushed DC motor electromagnetic torque model.
 * @brief 有刷直流电机定点电磁力矩模型。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_MOTORTORQUEMODEL_H
#define TRIPLE_CASCADECONTROLDCMOTOR_MOTORTORQUEMODEL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t torque_constant_uNm_per_A;       ///< Torque constant at reference temperature. / 参考温度下力矩常数，uN·m/A。
    int32_t torque_temp_coefficient_ppm_per_C;///< Optional magnet temperature coefficient. / 可选磁体温度系数，ppm/°C。
    uint32_t back_emf_uV_per_rpm;             ///< Back-EMF constant. / 反电动势常数，uV/rpm。
    uint32_t terminal_resistance_mOhm;        ///< Terminal resistance at reference temperature. / 参考温度下端电阻，mΩ。
    uint16_t resistance_temp_coefficient_ppm_per_C; ///< Winding resistance coefficient. / 绕组电阻温度系数，ppm/°C。
    uint16_t brush_drop_mV;                   ///< Combined brush voltage drop. / 电刷总压降，mV。
    int16_t reference_temperature_C;          ///< Parameter reference temperature. / 参数参考温度，°C。
} MotorTorqueModelParams;

typedef struct
{
    int32_t electromagnetic_torque_uNm; ///< Kt times measured current. / Kt 与实测电流得到的电磁力矩。
    uint32_t effective_torque_constant_uNm_per_A;
    uint32_t effective_resistance_mOhm;
    int32_t back_emf_mV;
    int32_t required_voltage_mV;
    int32_t available_current_mA;
    bool electrical_model_valid;
    bool voltage_limited;
} MotorTorqueModelResult;

bool MotorTorqueModel_IsTorqueValid(const MotorTorqueModelParams *params);

bool MotorTorqueModel_IsElectricalValid(const MotorTorqueModelParams *params,
                                        uint16_t encoder_counts_per_rev);

bool MotorTorqueModel_IsOperatingValid(const MotorTorqueModelParams *params,
                                       uint16_t encoder_counts_per_rev,
                                       int16_t temperature_C);

int32_t MotorTorqueModel_CurrentToTorque(const MotorTorqueModelParams *params,
                                         int32_t current_mA,
                                         int16_t temperature_C);

bool MotorTorqueModel_TorqueToCurrent(const MotorTorqueModelParams *params,
                                      int32_t torque_uNm,
                                      int16_t temperature_C,
                                      int16_t *current_mA);

int16_t MotorTorqueModel_LimitCurrentByVoltage(const MotorTorqueModelParams *params,
                                               int16_t requested_current_mA,
                                               uint16_t supply_voltage_mV,
                                               int32_t speed_cps,
                                               uint16_t encoder_counts_per_rev,
                                               int16_t temperature_C,
                                               bool *limited);

int16_t MotorTorqueModel_CurrentToDutyPermille(
    const MotorTorqueModelParams *params,
    int16_t requested_current_mA,
    uint16_t supply_voltage_mV,
    int32_t speed_cps,
    uint16_t encoder_counts_per_rev,
    int16_t temperature_C,
    bool *valid);

int16_t MotorTorqueModel_DutyToCurrent(
    const MotorTorqueModelParams *params,
    int16_t duty_permille,
    uint16_t supply_voltage_mV,
    int32_t speed_cps,
    uint16_t encoder_counts_per_rev,
    int16_t temperature_C,
    bool *valid);

void MotorTorqueModel_Evaluate(const MotorTorqueModelParams *params,
                               int32_t current_mA,
                               uint16_t supply_voltage_mV,
                               int32_t speed_cps,
                               uint16_t encoder_counts_per_rev,
                               int16_t temperature_C,
                               MotorTorqueModelResult *result);

#endif /* TRIPLE_CASCADECONTROLDCMOTOR_MOTORTORQUEMODEL_H */
