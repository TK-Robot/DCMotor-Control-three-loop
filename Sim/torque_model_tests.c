/**
 * @file torque_model_tests.c
 * @brief Host tests for the fixed-point motor and mechanical models.
 * @brief 电机与机械定点模型的主机侧测试。
 */

#include <stdio.h>

#include "MechanicalModel.h"
#include "MotorTorqueModel.h"
#include "CurrentSenseModel.h"
#include "FixedPointMath.h"
#include "Sds1601Calibration.h"

#define CHECK(expr) do { if (!(expr)) { \
    printf("FAIL: %s:%d: %s\n", __FILE__, __LINE__, #expr); return 1; } \
} while (0)

static MotorTorqueModelParams example_motor(void)
{
    MotorTorqueModelParams params = {
        .torque_constant_uNm_per_A = 7990U,
        .torque_temp_coefficient_ppm_per_C = 0,
        .back_emf_uV_per_rpm = 837U,
        .terminal_resistance_mOhm = 40700U,
        .resistance_temp_coefficient_ppm_per_C = 4000U,
        .brush_drop_mV = 0U,
        .reference_temperature_C = 22
    };

    return params;
}

static int test_electromagnetic_conversion(void)
{
    MotorTorqueModelParams params = example_motor();
    int16_t current_mA = 0;

    CHECK(MotorTorqueModel_IsTorqueValid(&params));
    CHECK(MotorTorqueModel_IsOperatingValid(&params, 16384U, 22));
    CHECK(MotorTorqueModel_CurrentToTorque(&params, 100, 22) == 799);
    CHECK(MotorTorqueModel_CurrentToTorque(&params, -100, 22) == -799);
    CHECK(MotorTorqueModel_TorqueToCurrent(&params, 1598, 22, &current_mA));
    CHECK(current_mA == 200);
    CHECK(MotorTorqueModel_TorqueToCurrent(&params, -1598, 22, &current_mA));
    CHECK(current_mA == -200);
    return 0;
}

static int test_voltage_and_temperature_model(void)
{
    MotorTorqueModelParams params = example_motor();
    MotorTorqueModelResult result;
    bool limited = false;
    int16_t current_mA;

    MotorTorqueModel_Evaluate(&params, 100, 4000U, 16384, 16384U, 22, &result);
    CHECK(result.electrical_model_valid);
    CHECK(result.back_emf_mV == 50);
    CHECK(result.effective_resistance_mOhm == 40700U);
    CHECK(result.required_voltage_mV == 4120);
    CHECK(result.available_current_mA == 97);
    CHECK(result.voltage_limited);

    current_mA = MotorTorqueModel_LimitCurrentByVoltage(
        &params, 100, 4000U, 16384, 16384U, 22, &limited);
    CHECK(limited);
    CHECK(current_mA == 97);

    MotorTorqueModel_Evaluate(&params, 100, 5000U, 0, 16384U, 47, &result);
    CHECK(result.effective_resistance_mOhm == 44770U);
    CHECK(!result.voltage_limited);
    return 0;
}

static int test_current_feedforward_model(void)
{
    MotorTorqueModelParams params = {
        .torque_constant_uNm_per_A = 50000U,
        .back_emf_uV_per_rpm = 7500U,
        .terminal_resistance_mOhm = 2650U,
        .reference_temperature_C = 25
    };
    bool valid = false;
    int16_t duty;
    int16_t current;

    duty = MotorTorqueModel_CurrentToDutyPermille(
        &params, 20, 5000U, 0, 16384U, 25, &valid);
    CHECK(valid);
    CHECK(duty == 11);
    current = MotorTorqueModel_DutyToCurrent(
        &params, duty, 5000U, 0, 16384U, 25, &valid);
    CHECK(valid);
    CHECK(current >= 20 && current <= 21);

    duty = MotorTorqueModel_CurrentToDutyPermille(
        &params, 100, 5000U, 16384, 16384U, 25, &valid);
    CHECK(duty == 143);
    CHECK(MotorTorqueModel_DutyToCurrent(
              &params, duty, 5000U, 16384, 16384U, 25, &valid) == 100);

    duty = MotorTorqueModel_CurrentToDutyPermille(
        &params, -100, 5000U, -16384, 16384U, 25, &valid);
    CHECK(duty == -143);
    CHECK(MotorTorqueModel_DutyToCurrent(
              &params, duty, 5000U, -16384, 16384U, 25, &valid) == -100);

    /* At voltage saturation, the current inferred from actual duty must be
     * lower than the unreachable request instead of echoing that request. */
    duty = MotorTorqueModel_CurrentToDutyPermille(
        &params, 2000, 5000U, 16384, 16384U, 25, &valid);
    CHECK(valid);
    CHECK(duty == 1000);
    current = MotorTorqueModel_DutyToCurrent(
        &params, duty, 5000U, 16384, 16384U, 25, &valid);
    CHECK(valid);
    CHECK(current >= 1716 && current <= 1718);
    return 0;
}

static int test_pwm_current_sampling_model(void)
{
    CHECK(CurrentSenseModel_DutyTicks(1249U, 70) == 87U);
    CHECK(CurrentSenseModel_TriggerCompare(1249U, 87U, 2U) == 625U);
    CHECK(CurrentSenseModel_TriggerCompare(1249U, 87U, 3U) == 625U);
    CHECK(!CurrentSenseModel_IsSampleValid(1249U, 70, 2U));
    CHECK(!CurrentSenseModel_IsSampleValid(1249U, 70, 3U));

    /* An 88-tick active window is the first one that contains 64 ticks of
     * edge blanking, both sample-and-hold phases (16 ticks), and an 8-tick
     * trailing guard. The final conversion may finish after the drive edge. */
    CHECK(CurrentSenseModel_DutyTicks(1249U, 71) == 88U);
    CHECK(CurrentSenseModel_TriggerCompare(1249U, 88U, 2U) == 1226U);
    CHECK(CurrentSenseModel_TriggerCompare(1249U, 88U, 3U) == 64U);
    CHECK(CurrentSenseModel_IsSampleValid(1249U, 71, 2U));
    CHECK(CurrentSenseModel_IsSampleValid(1249U, -71, 3U));
    CHECK(CurrentSenseModel_TriggerCompare(1249U, 1000U, 2U) == 1226U);
    CHECK(CurrentSenseModel_TriggerCompare(1249U, 1000U, 3U) == 976U);
    CHECK(!CurrentSenseModel_IsSampleValid(1249U, 800, 1U));
    CHECK(!CurrentSenseModel_IsSampleValid(1999U, 800, 2U));
    CHECK(CurrentSenseModel_CanEstimateFromAverageDuty(2U, false));
    CHECK(!CurrentSenseModel_CanEstimateFromAverageDuty(2U, true));
    CHECK(!CurrentSenseModel_CanEstimateFromAverageDuty(1U, false));
    CHECK(!CurrentSenseModel_CanEstimateFromAverageDuty(3U, false));
    CHECK(CurrentSenseModel_AdcToMilliamp(233U, 10U, 1504U) == 100);
    CHECK(CurrentSenseModel_AdcToMilliamp(466U, 20U, 3008U) == 100);
    return 0;
}

static int test_mechanical_load_estimate(void)
{
    const MechanicalModelParams params = {
        .total_inertia_ug_cm2 = 120000U,
        .coulomb_friction_uNm = 40U,
        .viscous_friction_nNm_per_rpm = 10U,
        .friction_deadband_cps = 10U
    };
    MechanicalModelResult result;

    MechanicalModel_Evaluate(&params, 1000, 163840, 1638400, 16384U, &result);
    CHECK(result.valid);
    CHECK(result.inertia_torque_uNm == 7);
    CHECK(result.coulomb_friction_torque_uNm == 40);
    CHECK(result.viscous_friction_torque_uNm == 6);
    CHECK(result.internal_loss_torque_uNm == 46);
    CHECK(result.shaft_load_torque_uNm == 947);
    CHECK(MechanicalModel_CompensateShaftTarget(&params, 947, 163840,
                                                1638400, 16384U) == 1000);

    MechanicalModel_Evaluate(&params, -1000, -163840, -1638400, 16384U, &result);
    CHECK(result.inertia_torque_uNm == -7);
    CHECK(result.internal_loss_torque_uNm == -46);
    CHECK(result.shaft_load_torque_uNm == -947);

    MechanicalModel_Evaluate(&params, 100, 0, 0, 16384U, &result);
    CHECK(result.internal_loss_torque_uNm == 0);
    CHECK(result.shaft_load_torque_uNm == 100);
    CHECK(MechanicalModel_CompensateShaftTarget(&params, 60, 0,
                                                0, 16384U) == 100);
    CHECK(MechanicalModel_CompensateShaftTarget(&params, -60, 0,
                                                0, 16384U) == -100);
    CHECK(MechanicalModel_CompensateShaftTarget(&params, 0, 0,
                                                0, 16384U) == 0);
    return 0;
}

static uint16_t sds1601_no_load_voltage_at_rpm(uint16_t rpm)
{
    const MotorTorqueModelParams motor = {
        .torque_constant_uNm_per_A = SDS1601_TORQUE_CONSTANT_UNM_PER_A,
        .back_emf_uV_per_rpm = SDS1601_BACK_EMF_UV_PER_RPM,
        .terminal_resistance_mOhm = SDS1601_TERMINAL_RESISTANCE_MOHM,
        .reference_temperature_C = 25
    };
    const MechanicalModelParams mechanical = {
        .coulomb_friction_uNm = SDS1601_COULOMB_FRICTION_UNM,
        .viscous_friction_nNm_per_rpm =
            SDS1601_VISCOUS_FRICTION_NNM_PER_RPM,
        .friction_deadband_cps = SDS1601_FRICTION_DEADBAND_CPS
    };
    const int32_t speed_cps = ((int32_t)rpm * 16384L + 30L) / 60L;
    MechanicalModelResult losses;
    MotorTorqueModelResult electrical;
    int16_t no_load_current_mA = 0;

    MechanicalModel_Evaluate(&mechanical, 0, speed_cps, 0, 16384U, &losses);
    CHECK(MotorTorqueModel_TorqueToCurrent(
        &motor, losses.internal_loss_torque_uNm, 25, &no_load_current_mA));
    MotorTorqueModel_Evaluate(&motor, no_load_current_mA, UINT16_MAX,
                              speed_cps, 16384U, 25, &electrical);
    return (uint16_t)electrical.required_voltage_mV;
}

static int test_sds1601_8v4_datasheet_calibration(void)
{
    const MotorTorqueModelParams motor = {
        .torque_constant_uNm_per_A = SDS1601_TORQUE_CONSTANT_UNM_PER_A,
        .back_emf_uV_per_rpm = SDS1601_BACK_EMF_UV_PER_RPM,
        .terminal_resistance_mOhm = SDS1601_TERMINAL_RESISTANCE_MOHM,
        .reference_temperature_C = 25
    };
    const uint16_t voltages_mV[] = {5000U, 6000U, 7400U, 8400U};
    const uint16_t expected_rpm[] = {83U, 100U, 125U, 143U};
    int32_t stall_current_mA;
    int32_t external_stall_torque_uNm;
    uint32_t i;

    /* 8.4 V / 2.65 ohm, with the measured static-friction loss removed,
     * produces the datasheet output-shaft stall torque (7.5 kg.cm). */
    stall_current_mA = (int32_t)SDS1601_RATED_MAX_VOLTAGE_MV * 1000L
                     / SDS1601_TERMINAL_RESISTANCE_MOHM;
    external_stall_torque_uNm = MotorTorqueModel_CurrentToTorque(
        &motor, stall_current_mA, 25) - SDS1601_COULOMB_FRICTION_UNM;
    CHECK(external_stall_torque_uNm >= 734000);
    CHECK(external_stall_torque_uNm <= 737000);

    /* Nameplate travel times include the vendor servo controller and are only
     * a broad sanity bound.  The direct 8.2 V PWM sweep measured about 145 rpm,
     * so the fitted Ke/friction model takes precedence over those rounded
     * values while remaining within 0.60 V of every nameplate point. */
    for (i = 0U; i < sizeof(voltages_mV) / sizeof(voltages_mV[0]); ++i)
    {
        CHECK(sds1601_no_load_voltage_at_rpm(expected_rpm[i])
              <= voltages_mV[i] + 600U);
        CHECK(sds1601_no_load_voltage_at_rpm(expected_rpm[i]) + 600U
              >= voltages_mV[i]);
    }
    CHECK(sds1601_no_load_voltage_at_rpm(145U) >= 7900U);
    CHECK(sds1601_no_load_voltage_at_rpm(145U) <= 8150U);
    return 0;
}

static int test_invalid_parameters_are_safe(void)
{
    MotorTorqueModelParams motor = {0};
    MechanicalModelParams mechanical = {0};
    MotorTorqueModelResult motor_result;
    MechanicalModelResult mechanical_result;
    int16_t current_mA = 123;

    CHECK(!MotorTorqueModel_TorqueToCurrent(&motor, 1000, 25, &current_mA));
    CHECK(!MotorTorqueModel_IsOperatingValid(&motor, 16384U, 25));
    CHECK(current_mA == 123);
    MotorTorqueModel_Evaluate(&motor, 1000, 12000U, 1000, 0U, 25, &motor_result);
    CHECK(!motor_result.electrical_model_valid);
    CHECK(motor_result.electromagnetic_torque_uNm == 0);
    MechanicalModel_Evaluate(&mechanical, 1000, 1000, 1000, 0U,
                             &mechanical_result);
    CHECK(!mechanical_result.valid);
    CHECK(mechanical_result.shaft_load_torque_uNm == 0);
    return 0;
}

static int test_fixed_point_large_division(void)
{
    CHECK(FixedPoint_DivideS64ByU32((int64_t)1 << 40, 1024U)
          == (int32_t)1 << 30);
    CHECK(FixedPoint_DivideS64ByU32(-((int64_t)1 << 40), 1024U)
          == -((int32_t)1 << 30));
    CHECK(FixedPoint_DivideS64ByU32(INT64_MAX, 1U) == INT32_MAX);
    CHECK(FixedPoint_DivideS64ByU32(INT64_MIN, 1U) == INT32_MIN);
    CHECK(FixedPoint_DivideS64ByU32ToS64(INT64_MIN, 2U)
          == INT64_MIN / 2);
    return 0;
}

int main(void)
{
    CHECK(test_electromagnetic_conversion() == 0);
    CHECK(test_voltage_and_temperature_model() == 0);
    CHECK(test_current_feedforward_model() == 0);
    CHECK(test_pwm_current_sampling_model() == 0);
    CHECK(test_mechanical_load_estimate() == 0);
    CHECK(test_sds1601_8v4_datasheet_calibration() == 0);
    CHECK(test_invalid_parameters_are_safe() == 0);
    CHECK(test_fixed_point_large_division() == 0);
    printf("PASS\n");
    return 0;
}
