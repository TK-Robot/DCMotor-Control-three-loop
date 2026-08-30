/**
 * @file MotorTorqueModel.c
 * @brief Fixed-point brushed DC motor electromagnetic torque model.
 * @brief 有刷直流电机定点电磁力矩模型。
 */

#include "MotorTorqueModel.h"
#include "FixedPointMath.h"

#include <limits.h>
#include <stddef.h>

#define MODEL_PPM_SCALE 1000000LL
#define MODEL_MAX_TEMP_COEFFICIENT_PPM_PER_C 10000L

static int32_t MotorTorqueModel_SaturateI32(int64_t value)
{
    if (value > INT32_MAX) return INT32_MAX;
    if (value < INT32_MIN) return INT32_MIN;
    return (int32_t)value;
}

static int16_t MotorTorqueModel_SaturateI16(int64_t value)
{
    if (value > INT16_MAX) return INT16_MAX;
    if (value < INT16_MIN) return INT16_MIN;
    return (int16_t)value;
}

static int64_t MotorTorqueModel_AbsI64(int64_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t MotorTorqueModel_ClampI32(int32_t value, int64_t magnitude_limit)
{
    if ((int64_t)value > magnitude_limit) return (int32_t)magnitude_limit;
    if ((int64_t)value < -magnitude_limit) return (int32_t)-magnitude_limit;
    return value;
}

static uint32_t MotorTorqueModel_AdjustByTemperature(uint32_t reference_value,
                                                     int32_t coefficient_ppm_per_C,
                                                     int16_t reference_temperature_C,
                                                     int16_t temperature_C)
{
    int32_t bounded_coefficient = coefficient_ppm_per_C;
    int64_t factor;
    int64_t adjusted;

    if (bounded_coefficient > MODEL_MAX_TEMP_COEFFICIENT_PPM_PER_C)
        bounded_coefficient = MODEL_MAX_TEMP_COEFFICIENT_PPM_PER_C;
    else if (bounded_coefficient < -MODEL_MAX_TEMP_COEFFICIENT_PPM_PER_C)
        bounded_coefficient = -MODEL_MAX_TEMP_COEFFICIENT_PPM_PER_C;
    factor = MODEL_PPM_SCALE
           + (int64_t)bounded_coefficient
           * ((int32_t)temperature_C - reference_temperature_C);
    if (reference_value == 0U || factor <= 0)
    {
        return 0U;
    }
    adjusted = FixedPoint_DivideS64ByU32(
        (int64_t)reference_value * factor + MODEL_PPM_SCALE / 2,
        (uint32_t)MODEL_PPM_SCALE);
    if (adjusted > UINT32_MAX) return UINT32_MAX;
    return (uint32_t)adjusted;
}

static int32_t MotorTorqueModel_BackEmf(const MotorTorqueModelParams *params,
                                        int32_t speed_cps,
                                        uint16_t encoder_counts_per_rev)
{
    int64_t numerator;
    int64_t denominator;
    int64_t maximum_speed;

    if (params == NULL || params->back_emf_uV_per_rpm == 0U
        || encoder_counts_per_rev == 0U)
    {
        return 0;
    }
    maximum_speed = FixedPoint_DivideS64ByU32ToS64(
        INT64_MAX, params->back_emf_uV_per_rpm);
    maximum_speed = FixedPoint_DivideS64ByU32ToS64(maximum_speed, 60U);
    speed_cps = MotorTorqueModel_ClampI32(speed_cps, maximum_speed);
    numerator = (int64_t)params->back_emf_uV_per_rpm * speed_cps * 60LL;
    denominator = (int64_t)encoder_counts_per_rev * 1000LL;
    return FixedPoint_DivideS64ByU32(numerator, (uint32_t)denominator);
}

bool MotorTorqueModel_IsTorqueValid(const MotorTorqueModelParams *params)
{
    return params != NULL && params->torque_constant_uNm_per_A != 0U;
}

bool MotorTorqueModel_IsElectricalValid(const MotorTorqueModelParams *params,
                                        uint16_t encoder_counts_per_rev)
{
    return params != NULL && params->terminal_resistance_mOhm != 0U
           && params->back_emf_uV_per_rpm != 0U
           && encoder_counts_per_rev != 0U;
}

bool MotorTorqueModel_IsOperatingValid(const MotorTorqueModelParams *params,
                                       uint16_t encoder_counts_per_rev,
                                       int16_t temperature_C)
{
    return MotorTorqueModel_IsTorqueValid(params)
           && MotorTorqueModel_IsElectricalValid(params, encoder_counts_per_rev)
           && MotorTorqueModel_AdjustByTemperature(
                  params->torque_constant_uNm_per_A,
                  params->torque_temp_coefficient_ppm_per_C,
                  params->reference_temperature_C,
                  temperature_C) != 0U
           && MotorTorqueModel_AdjustByTemperature(
                  params->terminal_resistance_mOhm,
                  params->resistance_temp_coefficient_ppm_per_C,
                  params->reference_temperature_C,
                  temperature_C) != 0U;
}

int32_t MotorTorqueModel_CurrentToTorque(const MotorTorqueModelParams *params,
                                         int32_t current_mA,
                                         int16_t temperature_C)
{
    uint32_t torque_constant;
    int64_t torque;

    if (!MotorTorqueModel_IsTorqueValid(params)) return 0;
    torque_constant = MotorTorqueModel_AdjustByTemperature(
        params->torque_constant_uNm_per_A,
        params->torque_temp_coefficient_ppm_per_C,
        params->reference_temperature_C,
        temperature_C);
    torque = FixedPoint_DivideS64ByU32(
        (int64_t)torque_constant * current_mA, 1000U);
    return MotorTorqueModel_SaturateI32(torque);
}

bool MotorTorqueModel_TorqueToCurrent(const MotorTorqueModelParams *params,
                                      int32_t torque_uNm,
                                      int16_t temperature_C,
                                      int16_t *current_mA)
{
    uint32_t torque_constant;
    int64_t numerator;
    int64_t current;

    if (current_mA == NULL || !MotorTorqueModel_IsTorqueValid(params)) return false;
    torque_constant = MotorTorqueModel_AdjustByTemperature(
        params->torque_constant_uNm_per_A,
        params->torque_temp_coefficient_ppm_per_C,
        params->reference_temperature_C,
        temperature_C);
    if (torque_constant == 0U) return false;

    numerator = (int64_t)torque_uNm * 1000LL;
    if (numerator >= 0) numerator += torque_constant / 2U;
    else numerator -= torque_constant / 2U;
    current = FixedPoint_DivideS64ByU32(numerator, torque_constant);
    *current_mA = MotorTorqueModel_SaturateI16(current);
    return current <= INT16_MAX && current >= INT16_MIN;
}

void MotorTorqueModel_Evaluate(const MotorTorqueModelParams *params,
                               int32_t current_mA,
                               uint16_t supply_voltage_mV,
                               int32_t speed_cps,
                               uint16_t encoder_counts_per_rev,
                               int16_t temperature_C,
                               MotorTorqueModelResult *result)
{
    int32_t current_direction;
    int64_t resistive_voltage_mV;
    int64_t required_voltage_mV;
    int64_t available_voltage_mV;
    int64_t available_current_mA;

    if (result == NULL) return;
    *result = (MotorTorqueModelResult){0};
    if (params == NULL) return;

    result->effective_torque_constant_uNm_per_A =
        MotorTorqueModel_AdjustByTemperature(params->torque_constant_uNm_per_A,
                                             params->torque_temp_coefficient_ppm_per_C,
                                             params->reference_temperature_C,
                                             temperature_C);
    result->effective_resistance_mOhm =
        MotorTorqueModel_AdjustByTemperature(params->terminal_resistance_mOhm,
                                             params->resistance_temp_coefficient_ppm_per_C,
                                             params->reference_temperature_C,
                                             temperature_C);
    result->electromagnetic_torque_uNm =
        MotorTorqueModel_CurrentToTorque(params, current_mA, temperature_C);
    result->back_emf_mV = MotorTorqueModel_BackEmf(params, speed_cps,
                                                   encoder_counts_per_rev);
    result->electrical_model_valid =
        MotorTorqueModel_IsElectricalValid(params, encoder_counts_per_rev)
        && result->effective_resistance_mOhm != 0U;
    if (!result->electrical_model_valid || current_mA == 0)
    {
        return;
    }

    current_direction = (current_mA > 0) ? 1 : -1;
    resistive_voltage_mV = FixedPoint_DivideS64ByU32(
        (int64_t)current_mA * result->effective_resistance_mOhm, 1000U);
    required_voltage_mV = resistive_voltage_mV + result->back_emf_mV
                        + (int64_t)current_direction * params->brush_drop_mV;
    result->required_voltage_mV = MotorTorqueModel_SaturateI32(required_voltage_mV);
    result->voltage_limited =
        MotorTorqueModel_AbsI64(required_voltage_mV) > supply_voltage_mV;

    available_voltage_mV = (int64_t)current_direction * supply_voltage_mV
                         - result->back_emf_mV
                         - (int64_t)current_direction * params->brush_drop_mV;
    available_current_mA = FixedPoint_DivideS64ByU32(
        available_voltage_mV * 1000LL,
        result->effective_resistance_mOhm);
    if (available_current_mA * current_direction < 0)
    {
        available_current_mA = 0;
    }
    result->available_current_mA =
        MotorTorqueModel_SaturateI32(available_current_mA);
}

int16_t MotorTorqueModel_LimitCurrentByVoltage(const MotorTorqueModelParams *params,
                                               int16_t requested_current_mA,
                                               uint16_t supply_voltage_mV,
                                               int32_t speed_cps,
                                               uint16_t encoder_counts_per_rev,
                                               int16_t temperature_C,
                                               bool *limited)
{
    MotorTorqueModelResult result;

    if (limited != NULL) *limited = false;
    MotorTorqueModel_Evaluate(params, requested_current_mA, supply_voltage_mV,
                              speed_cps, encoder_counts_per_rev, temperature_C,
                              &result);
    if (!result.electrical_model_valid || !result.voltage_limited)
    {
        return requested_current_mA;
    }
    if (limited != NULL) *limited = true;
    return MotorTorqueModel_SaturateI16(result.available_current_mA);
}

int16_t MotorTorqueModel_CurrentToDutyPermille(
    const MotorTorqueModelParams *params,
    int16_t requested_current_mA,
    uint16_t supply_voltage_mV,
    int32_t speed_cps,
    uint16_t encoder_counts_per_rev,
    int16_t temperature_C,
    bool *valid)
{
    MotorTorqueModelResult result;
    int64_t numerator;
    int64_t duty;

    if (valid != NULL) *valid = false;
    if (supply_voltage_mV == 0U) return 0;

    MotorTorqueModel_Evaluate(params, requested_current_mA,
                              supply_voltage_mV, speed_cps,
                              encoder_counts_per_rev, temperature_C, &result);
    if (!result.electrical_model_valid) return 0;
    if (valid != NULL) *valid = true;
    if (requested_current_mA == 0) return 0;

    numerator = (int64_t)result.required_voltage_mV * 1000LL;
    if (numerator >= 0) numerator += supply_voltage_mV / 2U;
    else numerator -= supply_voltage_mV / 2U;
    duty = FixedPoint_DivideS64ByU32(numerator, supply_voltage_mV);
    if (duty > 1000LL) duty = 1000LL;
    else if (duty < -1000LL) duty = -1000LL;
    return (int16_t)duty;
}

int16_t MotorTorqueModel_DutyToCurrent(
    const MotorTorqueModelParams *params,
    int16_t duty_permille,
    uint16_t supply_voltage_mV,
    int32_t speed_cps,
    uint16_t encoder_counts_per_rev,
    int16_t temperature_C,
    bool *valid)
{
    MotorTorqueModelResult result;
    int32_t drive_direction;
    int64_t applied_voltage_mV;
    int64_t winding_voltage_mV;
    int64_t current_mA;

    if (valid != NULL) *valid = false;
    MotorTorqueModel_Evaluate(params, 1, supply_voltage_mV, speed_cps,
                              encoder_counts_per_rev, temperature_C, &result);
    if (!result.electrical_model_valid || supply_voltage_mV == 0U)
    {
        return 0;
    }
    if (valid != NULL) *valid = true;
    if (duty_permille == 0) return 0;

    drive_direction = (duty_permille > 0) ? 1 : -1;
    applied_voltage_mV = FixedPoint_DivideS64ByU32(
        (int64_t)supply_voltage_mV * duty_permille, 1000U);
    winding_voltage_mV = applied_voltage_mV - result.back_emf_mV
                       - (int64_t)drive_direction * params->brush_drop_mV;
    current_mA = FixedPoint_DivideS64ByU32(
        winding_voltage_mV * 1000LL,
        result.effective_resistance_mOhm);
    return MotorTorqueModel_SaturateI16(current_mA);
}
