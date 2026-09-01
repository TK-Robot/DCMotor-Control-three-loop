/**
 * @file servo_tests.c
 * @brief PC-side unit tests for PID and ServoControl logic.
 * @brief PID 和 ServoControl 逻辑的 PC 端单元测试。
 */

#include <stdio.h>
#include <stdlib.h>

#include "PID.h"
#include "ServoControl.h"

#define CHECK(expr) do { if (!(expr)) { \
    printf("FAIL: %s:%d: %s\n", __FILE__, __LINE__, #expr); return 1; } \
} while (0)

static void default_param(Param *param)
{
    PID_Init(param);
    param->CycleTimeMs = 1;
    param->TempLimit = 80;
    param->SpeedMax = 30000;
    param->PositionDeadbandCounts = PID_POSITION_DEADBAND_DEFAULT_COUNTS;
    param->AccelMax = 60U;
    param->DecelMax = 60U;
    param->DrivePwmMode = 2U;
    param->PowerSaveVoltage_mV = 4000;
    param->VCC_mV = 12000;
    param->TorqueEncoderCountsPerRev = 16384U;
    param->TorqueCurrentLimit_mA = 1000U;
    param->MotorInductance_uH = 10U;
    param->CurrentPeakLimit_mA = 1500U;
    param->CurrentAbsoluteLimit_mA = 1750U;
    param->StallCurrentThreshold_mA = 300U;
    param->StallSpeedThreshold_cps = 300U;
    param->StallConfirmTimeMs = 3000U;
    param->MotorWindingTemperature_C = 22;
    param->EncoderFeedbackValid = true;
    param->EncoderSampleAgeMs = 0U;
    param->MotorTorqueParams = (MotorTorqueModelParams){
        .torque_constant_uNm_per_A = 245000U,
        .back_emf_uV_per_rpm = 40000U,
        .terminal_resistance_mOhm = 2650U,
        .reference_temperature_C = 25
    };
}

static int test_pid_reset(void)
{
    PID_Int pid = {0};

    pid.integral = 123;
    pid.prev_error = 22;
    pid.prev_prev_error = 11;
    pid.prev_feedback = 44;
    pid.prev_out = 55;
    PID_Reset(&pid);

    CHECK(pid.integral == 0);
    CHECK(pid.prev_error == 0);
    CHECK(pid.prev_prev_error == 0);
    CHECK(pid.prev_feedback == 0);
    CHECK(pid.prev_out == 0);
    return 0;
}

static int test_loop_limits(void)
{
    Param param = {0};
    Param damped = {0};

    default_param(&param);
    param.EncoderMultiTurnValue = 0;
    param.Pid_Pos.integral = 1234;
    param.Pid_Pos.prev_error = 56;
    param.SpeedRef = 700;
    param.Pid_PosVel.integral = 1234;
    CHECK(PID_PositionLoop(&param, param.PositionDeadbandCounts) == 0);
    CHECK(param.Pid_Pos.integral == 0);
    CHECK(param.SpeedRef == 0);
    CHECK(param.Pid_PosVel.integral == 0);
    CHECK(PID_PositionLoop(&param, param.PositionDeadbandCounts + 1) > 0);
    CHECK(PID_PositionLoop(&param, -(int32_t)param.PositionDeadbandCounts) == 0);
    CHECK(PID_PositionLoop(&param, -(int32_t)param.PositionDeadbandCounts - 1) < 0);
    param.PositionDeadbandCounts = 4U;
    CHECK(PID_PositionLoop(&param, 5) > 0);
    CHECK(PID_PositionLoop(&param, 1000000) <= param.SpeedMax);

    default_param(&damped);
    damped.Pid_Pos.Kp = 0U;
    damped.Pid_Pos.Ki = 0U;
    damped.Pid_Pos.Kd = 40000U;
    damped.EncoderMultiTurnValue = 1000;
    CHECK(PID_PositionLoop(&damped, 1500) == 0);
    damped.EncoderMultiTurnValue = 1010;
    CHECK(PID_PositionLoop(&damped, 1500) < 0);
    PID_Reset(&damped.Pid_Pos);
    CHECK(PID_PositionLoop(&damped, 2500) == 0);

    param.EncoderSpeed = -50000;
    CHECK(abs(PID_SpeedLoop(&param, 50000, 1U)) <= param.Pid_PosVel.out_max);

    param.SpeedRef = 0;
    param.EncoderSpeed = 0;
    param.Pid_PosVel.integral = 12345;
    CHECK(PID_SpeedLoop(&param, 0, 10U) == 0);
    CHECK(param.Pid_PosVel.integral == 0);

    param.INA181_mA = 5000;
    CHECK(abs(PID_CurrentLoop(&param, 5000)) <= 1000);

    param.INA181_mA = 0;
    PID_Reset(&param.Pid_PosEle);
    CHECK(PID_CurrentLoop(&param, -5000) < 0);

    param.INA181_mA = 0;
    PID_Reset(&param.Pid_PosEle);
    CHECK(PID_CurrentLoop(&param, 5000) > 0);

    param.EncoderVeer = true;
    param.INA181_mA = 0;
    PID_Reset(&param.Pid_PosEle);
    CHECK(PID_CurrentLoop(&param, 5000) < 0);

    param.INA181_mA = 5000;
    CHECK(PID_CurrentLoop(&param, 0) == 0);
    return 0;
}

static int test_breakaway_uses_speed_pi_integrator(void)
{
    Param stalled = {0};
    Param moving = {0};

    default_param(&stalled);
    default_param(&moving);
    stalled.SpeedRef = 1500;
    moving.SpeedRef = 1500;
    stalled.EncoderSpeed = 0;
    moving.EncoderSpeed = 301;

    (void)PID_SpeedLoop(&stalled, 1500, 1U);
    (void)PID_SpeedLoop(&moving, 1500, 1U);
    CHECK(stalled.Pid_PosVel.integral
          > moving.Pid_PosVel.integral * 4L);
    CHECK(stalled.Pid_PosVel.Ki == moving.Pid_PosVel.Ki);
    return 0;
}

static int test_low_speed_uses_same_direction_pulse_density(void)
{
    Param param = {0};
    int16_t pulse = 0;

    default_param(&param);
    param.AccelMax = 1000U;
    param.DecelMax = 1000U;
    param.Pid_PosVel.Kp = 1000U;
    param.Pid_PosVel.Ki = 0U;

    /* The pulse floor is an actuator requirement and must not change when the
     * separately configurable sustained-stall threshold changes. */
    param.StallCurrentThreshold_mA = 250U;
    for (uint8_t tick = 0U; tick < 8U && pulse == 0; ++tick)
        pulse = PID_SpeedLoop(&param, 100, 1U);
    CHECK(param.EncoderSpeedExpect == 100);
    CHECK(pulse == (int16_t)SPEED_LOW_CURRENT_PULSE_MA);
    CHECK(param.ExpectMA == (int16_t)SPEED_LOW_CURRENT_PULSE_MA);

    /* Crawl-speed overspeed must coast instead of repeatedly loading the
     * opposite gearbox tooth face. */
    param.EncoderSpeed = 500;
    param.Pid_PosVel.integral = 500000;
    CHECK(PID_SpeedLoop(&param, 100, 1U) == 0);
    CHECK(param.Pid_PosVel.integral <= 0);
    CHECK(param.target_speed == 0);

    /* The measured 1000 cps failure case is inside the pulse-density band. */
    PID_Reset(&param.Pid_PosVel);
    param.SpeedRef = 1000;
    param.EncoderSpeed = 0;
    param.Pid_PosVel.Kp = 100U;
    pulse = 0;
    for (uint8_t tick = 0U; tick < 8U && pulse == 0; ++tick)
        pulse = PID_SpeedLoop(&param, 1000, 1U);
    CHECK(pulse == (int16_t)SPEED_LOW_CURRENT_PULSE_MA);
    param.EncoderSpeed = 1500;
    CHECK(PID_SpeedLoop(&param, 1000, 1U) == 0);
    param.EncoderSpeed = 3000;
    CHECK(PID_SpeedLoop(&param, 1000, 1U) < 0);

    PID_Reset(&param.Pid_PosVel);
    param.SpeedRef = 3000;
    param.EncoderSpeed = 0;
    pulse = 0;
    for (uint8_t tick = 0U; tick < 8U && pulse == 0; ++tick)
        pulse = PID_SpeedLoop(&param, 3000, 1U);
    CHECK(pulse == (int16_t)SPEED_LOW_CURRENT_PULSE_MA);
    param.EncoderSpeed = 3500;
    CHECK(PID_SpeedLoop(&param, 3000, 1U) == 0);
    param.EncoderSpeed = 4500;
    CHECK(PID_SpeedLoop(&param, 3000, 1U) < 0);

    /* Above the crawl band, active regenerative braking remains available. */
    PID_Reset(&param.Pid_PosVel);
    param.SpeedRef = 4000;
    param.EncoderSpeed = 4500;
    CHECK(PID_SpeedLoop(&param, 4000, 1U) < 0);

    PID_Reset(&param.Pid_PosVel);
    param.SpeedRef = 0;
    param.target_speed = 0;
    param.Pid_PosVel.Kp = 1000U;
    param.EncoderSpeed = 0;
    pulse = 0;
    for (uint8_t tick = 0U; tick < 8U && pulse == 0; ++tick)
        pulse = PID_SpeedLoop(&param, -100, 1U);
    CHECK(pulse == -(int16_t)SPEED_LOW_CURRENT_PULSE_MA);
    CHECK(param.EncoderSpeedExpect < 0);
    CHECK(param.ExpectMA == -(int16_t)SPEED_LOW_CURRENT_PULSE_MA);

    /* The quantizer must respect a lower mode-specific current ceiling. */
    PID_Reset(&param.Pid_PosVel);
    param.SpeedRef = 0;
    param.target_speed = 0;
    (void)PID_SpeedTorqueLoop(&param, 100, 1U, 80U, NULL);
    CHECK(param.ExpectMA == 80);
    return 0;
}

static int test_model_fed_low_current_loop(void)
{
    Param param = {0};

    default_param(&param);
    param.VCC_mV = 5000U;
    param.MotorTorqueParams = (MotorTorqueModelParams){
        .torque_constant_uNm_per_A = 50000U,
        .back_emf_uV_per_rpm = 7500U,
        .terminal_resistance_mOhm = 2650U,
        .reference_temperature_C = 25
    };
    param.MotorWindingTemperature_C = 25;
    param.CurrentSampleValid = false;
    CHECK(PID_CurrentLoop(&param, 20) == 11);

    param.CurrentSampleValid = true;
    param.INA181_mA = 20;
    param.CurrentAverage_mA = 20;
    CHECK(PID_CurrentLoop(&param, 20) == 11);

    param.INA181_mA = 80;
    param.CurrentAverage_mA = 80;
    CHECK(PID_CurrentLoop(&param, 20) == 11);

    PID_Reset(&param.Pid_PosEle);
    param.CurrentSampleValid = false;
    param.CurrentSampleAgeMs = 4U;
    param.INA181_mA = 0;
    param.CurrentAverage_mA = 0;
    CHECK(PID_CurrentLoop(&param, 200) == 106);

    param.VCC_mV = 8200U;
    param.CurrentSampleValid = true;
    param.CurrentSampleAgeMs = 0U;
    param.INA181_mA = 100;
    param.CurrentAverage_mA = 100;
    CHECK(PID_CurrentLoop(&param, 100) == 32);

    param.INA181_mA = 151;
    param.CurrentAverage_mA = 151;
    CHECK(PID_CurrentLoop(&param, 100) == 32);

    param.INA181_mA = 1501;
    param.CurrentAverage_mA = 1501;
    CHECK(PID_CurrentLoop(&param, 100) == 32);

    param.INA181_mA = 1501;
    CHECK(PID_CurrentLoop(&param, 1000) == 323);
    return 0;
}

static int test_active_window_peak_never_trims_average_model(void)
{
    Param param = {0};
    int16_t model_pwm;

    default_param(&param);
    param.VCC_mV = 8200U;
    param.DriveRunMode = 2U;
    param.DrivePower = 226;
    param.CurrentSampleValid = true;
    param.INA181_mA = 500;
    param.CurrentAverage_mA = 500;

    /* The shunt value is an active-window peak in every decay mode. Repeated
     * qualified samples must not alter average-current model voltage. */
    model_pwm = PID_CurrentLoop(&param, 700);
    CHECK(model_pwm == 226);
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        param.CurrentAverage_mA = (cycle & 1) ? 50 : 1500;
        CHECK(PID_CurrentLoop(&param, 700) == 226);
        CHECK(param.CurrentCorrectionPwm == 0);
        CHECK(param.Pid_PosEle.integral == 0);
        CHECK(param.Pid_PosEle.prev_feedback == 0);
        CHECK((param.CurrentLoopStatus & 0x0001U) == 0U);
        CHECK((param.CurrentLoopStatus & 0x0008U) != 0U);
    }

    param.DriveRunMode = 3U;
    param.CurrentEstimated = false;
    param.CurrentSampleValid = true;
    param.CurrentSampleAgeMs = 0U;
    CHECK(PID_CurrentLoop(&param, 700) == 226);
    CHECK(param.CurrentCorrectionPwm == 0);

    /* Direction changes still reset stale PI state before bridge reversal. */
    CHECK(PID_CurrentLoop(&param, -700) == -226);
    CHECK(param.Pid_PosEle.integral == 0);
    CHECK(param.Pid_PosEle.prev_feedback == 0);

    /* At regenerative operating points the ground shunt cannot verify signed
     * current. The model supplies a bounded brake/coast duty while PI freezes. */
    PID_Reset(&param.Pid_PosEle);
    param.CurrentSampleValid = true;
    param.CurrentSampleAgeMs = 0U;
    param.DriveRunMode = 2U;
    param.DrivePower = 260;
    param.EncoderSpeed = 16384;
    param.INA181_mA = 0;
    param.CurrentAverage_mA = 0;
    MotorTorqueModel_Evaluate(&param.MotorTorqueParams, 0, param.VCC_mV,
                              param.EncoderSpeed,
                              param.TorqueEncoderCountsPerRev,
                              param.MotorWindingTemperature_C,
                              &param.MotorTorqueResult);
    CHECK(PID_CurrentLoop(&param, -100) < 0);
    CHECK(param.Pid_PosEle.integral == 0);
    CHECK(param.CurrentModelPwm < 0);
    CHECK(param.CurrentCorrectionPwm == 0);
    CHECK((param.CurrentLoopStatus & 0x0080U) != 0U);

    /* Once speed/back-EMF falls, the same request resumes in the commanded
     * direction without host intervention. */
    param.EncoderSpeed = 0;
    CHECK(PID_CurrentLoop(&param, -100) < 0);
    CHECK((param.CurrentLoopStatus & 0x0080U) == 0U);

    return 0;
}

static int test_legacy_current_tuning_migration(void)
{
    Param param = {0};

    param.Pid_Pos.out_max = 35000U;
    param.Pid_PosEle.Kp = 3200U;
    param.Pid_PosEle.Ki = 420U;
    param.Pid_PosEle.out_min = 5U;
    param.Pid_PosEle.out_max = 750U;
    param.Pid_PosEle.integral_max = 8000;
    param.Pid_PosVel.Kp = 8U;
    param.Pid_PosVel.Ki = 15U;
    param.Pid_PosVel.Kd = 14U;
    param.Pid_PosVel.out_min = 10U;
    PID_MigrateLegacyCurrentTuning(&param);
    CHECK(param.Pid_Pos.out_max == 30000U);
    CHECK(param.Pid_Pos.Kd == 0U);
    CHECK(param.Pid_PosEle.Kp == 3200U);
    CHECK(param.Pid_PosEle.Ki == 420U);
    CHECK(param.Pid_PosEle.out_min == 5U);
    CHECK(param.Pid_PosEle.out_max == 750U);
    CHECK(param.Pid_PosEle.integral_max == 8000);
    CHECK(param.Pid_PosVel.Ki == 15U);
    CHECK(param.Pid_PosVel.Kd == 0U);
    CHECK(param.Pid_PosVel.out_min == 0U);

    param.Pid_Pos.out_max = 8000U;
    param.Pid_Pos.Kd = 5000U;
    PID_MigrateLegacyCurrentTuning(&param);
    CHECK(param.Pid_Pos.out_max == 8000U);
    CHECK(param.Pid_Pos.Kd == 13U);

    param.Pid_Pos.Kd = 25U;
    PID_MigrateLegacyCurrentTuning(&param);
    CHECK(param.Pid_Pos.Kd == 25U);

    param.Pid_Pos.Kd = 1000U;
    PID_MigrateLegacyCurrentTuning(&param);
    CHECK(param.Pid_Pos.Kd == 1000U);

    param.Pid_PosVel.Kp = 20U;
    param.Pid_PosVel.Ki = 0U;
    param.Pid_PosVel.Kd = 0U;
    param.Pid_PosVel.integral_max = 8000;
    param.Pid_PosVel.out_max = 400U;
    param.Pid_PosVel.out_min = 0U;
    PID_MigrateLegacyCurrentTuning(&param);
    CHECK(param.Pid_PosVel.Kp == 120U);
    CHECK(param.Pid_PosVel.Ki == 10U);

    /* The one-sided peak sampler cannot close an average-current PI, so its
     * dormant tuning is preserved instead of spending runtime code migrating
     * values which do not participate in the actuator command. */
    param.Pid_PosEle.Kp = 250U;
    param.Pid_PosEle.Ki = 30U;
    param.Pid_PosEle.Kd = 0U;
    param.Pid_PosEle.out_min = 0U;
    param.Pid_PosEle.out_max = 500U;
    param.Pid_PosEle.integral_max = 8000;
    PID_MigrateLegacyCurrentTuning(&param);
    CHECK(param.Pid_PosEle.integral_max == 8000);

    param.Pid_PosEle.Kp = 200U;
    param.Pid_PosEle.Ki = 25U;
    param.Pid_PosEle.integral_max = 8000;
    PID_MigrateLegacyCurrentTuning(&param);
    CHECK(param.Pid_PosEle.Kp == 200U);
    CHECK(param.Pid_PosEle.Ki == 25U);
    CHECK(param.Pid_PosEle.integral_max == 8000);
    return 0;
}

static int test_position_derivative_uses_speed_units(void)
{
    PID_Int pid = {
        .Kd = 13U,
        .integral_max = 35000,
        .out_max = 30000U
    };

    CHECK(PID_POSITION_UPDATE_PERIOD_MS == SERVO_POSITION_PERIOD_MS);
    CHECK(PID_AbsCalculate(&pid, 10000, 1000) == 0);
    /* 50 counts per 5 ms is 10000 count/s; Kd=13 yields -130 count/s. */
    CHECK(PID_AbsCalculate(&pid, 20000, 1050) == -130);
    /* Derivative on feedback must not react to a target-only step. */
    CHECK(PID_AbsCalculate(&pid, 30000, 1050) == 0);
    return 0;
}

static int test_speed_plan_uses_scaled_limits(void)
{
    Param param = {0};

    param.AccelMax = 200U;
    param.DecelMax = 300U;

    CHECK(SpeedPlan_Update(&param, 5000, 1U) == 200);
    param.SpeedRef = 1000;
    CHECK(SpeedPlan_Update(&param, 0, 1U) == 700);
    param.SpeedRef = -1000;
    CHECK(SpeedPlan_Update(&param, -5000, 1U) == -1200);
    param.SpeedRef = -1000;
    CHECK(SpeedPlan_Update(&param, 0, 1U) == -700);

    param.SpeedRef = 100;
    CHECK(SpeedPlan_Update(&param, -5000, 1U) == 0);
    CHECK(SpeedPlan_Update(&param, -5000, 1U) == -200);

    param.SpeedRef = 0;
    param.AccelMax = UINT16_MAX;
    CHECK(SpeedPlan_Update(&param, 30000, 1U) == 30000);
    return 0;
}

static int test_pwm_position_command(void)
{
    ServoCommand command = {0};

    ServoControl_BuildPwmPositionCommand(1000U, true, &command);
    CHECK(command.mode == SERVO_MODE_POSITION);
    CHECK(command.enable);
    CHECK(!command.position_multi_turn);
    CHECK(command.target_position == 0);

    ServoControl_BuildPwmPositionCommand(1500U, true, &command);
    CHECK(command.target_position >= 8190 && command.target_position <= 8192);

    ServoControl_BuildPwmPositionCommand(2000U, true, &command);
    CHECK(command.target_position == 16383);

    ServoControl_BuildPwmPositionCommand(1500U, false, &command);
    CHECK(!command.enable);
    return 0;
}

static int test_mode_switch_and_power_save(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    ServoControl_Init(&servo, &param);

    command.mode = SERVO_MODE_SPEED;
    command.enable = true;
    command.target_speed = 1000;
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.OutputEnabled);

    param.Pid_PosVel.prev_out = 123;
    command.mode = SERVO_MODE_POSITION;
    command.target_position = 2000;
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.Pid_PosVel.prev_out == 0);

    param.VCC_mV = 3900;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(ServoControl_ConsumeSaveRequest(&servo));
    CHECK(!ServoControl_ConsumeSaveRequest(&servo));
    CHECK(param.DrivePower == 0);
    CHECK(!param.OutputEnabled);
    return 0;
}

static int test_fault_brake_policy(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    param.FaultCode = 0x000AU;
    param.FailSafePolicy = FAILSAFE_BRAKE;
    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);

    CHECK(param.DriveRunMode == 1U);
    CHECK(param.DrivePower == 0);
    CHECK(!param.OutputEnabled);
    return 0;
}

static int test_protection_inhibit_and_watchdog_fallback(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    param.FailSafePolicy = FAILSAFE_FALLBACK_PWM;
    param.FaultCode = 0x000AU;
    command.mode = SERVO_MODE_CURRENT;
    command.enable = true;
    command.target_current_mA = 100;
    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);

    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.OutputEnabled);
    CHECK(param.FaultCode == 0x000AU);
    CHECK(param.ProtectionFlags == PROTECTION_NONE);

    param.VCC_mV = 3900U;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(!param.OutputEnabled);
    CHECK(param.DrivePower == 0);
    CHECK((param.ProtectionFlags & PROTECTION_UNDERVOLTAGE) != 0U);

    param.VCC_mV = 4400U;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK((param.ProtectionFlags & PROTECTION_UNDERVOLTAGE) != 0U);

    param.VCC_mV = 4501U;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK((param.ProtectionFlags & PROTECTION_UNDERVOLTAGE) == 0U);
    CHECK(param.OutputEnabled);

    param.Temp = 81;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(!param.OutputEnabled);
    CHECK((param.ProtectionFlags & PROTECTION_OVERTEMPERATURE) != 0U);

    param.Temp = 80;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK((param.ProtectionFlags & PROTECTION_OVERTEMPERATURE) == 0U);
    CHECK(param.OutputEnabled);
    return 0;
}

static int test_speed_and_position_cascades(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    ServoControl_Init(&servo, &param);

    command.mode = SERVO_MODE_SPEED;
    command.enable = true;
    command.target_speed = 30000;
    ServoControl_SetCommand(&servo, &command);
    for (uint16_t tick = 0; tick < 100U; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }
    CHECK(param.EncoderSpeedExpect > 0);
    CHECK(param.ExpectMA != 0);
    CHECK(param.TargetTorque_uNm != 0);
    CHECK(param.TargetElectromagneticTorque_uNm != 0);

    command.mode = SERVO_MODE_POSITION;
    command.target_position = 4000;
    ServoControl_SetCommand(&servo, &command);
    for (uint8_t tick = 0; tick < 20U; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }
    CHECK(param.EncoderSpeedExpect > 0);
    CHECK(param.EncoderSpeedExpect <= (int32_t)param.SpeedMax);

    command.enable = false;
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.EncoderSpeedExpect == 0);
    CHECK(param.ExpectMA == 0);
    CHECK(param.DrivePower == 0);
    return 0;
}

static int test_model_assisted_speed_cascade(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    param.Pid_PosVel.Kp = 1000U;
    param.Pid_PosVel.Ki = 0U;
    param.Pid_PosVel.Kd = 0U;
    param.Pid_PosVel.out_min = 0U;
    param.Pid_PosVel.out_max = 1000U;
    param.MotorTorqueParams = (MotorTorqueModelParams){
        .torque_constant_uNm_per_A = 10000U,
        .back_emf_uV_per_rpm = 1000U,
        .terminal_resistance_mOhm = 1000U,
        .reference_temperature_C = 22
    };
    param.MechanicalParams = (MechanicalModelParams){
        .total_inertia_ug_cm2 = 100000000U,
        .coulomb_friction_uNm = 50U,
        .friction_deadband_cps = 0U
    };
    command.mode = SERVO_MODE_SPEED;
    command.enable = true;
    command.target_speed = 30000;

    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);
    for (uint8_t tick = 0; tick < SERVO_SPEED_PERIOD_MS; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }

    CHECK(param.EncoderSpeedExpect == 60);
    CHECK(param.TargetTorque_uNm == 600);
    CHECK(param.TargetElectromagneticTorque_uNm > param.TargetTorque_uNm);
    CHECK(param.ExpectMA > 60);
    CHECK(!param.TorqueCommandVoltageLimited);

    param.TorqueCurrentLimit_mA = 65U;
    for (uint8_t tick = 0; tick < SERVO_SPEED_PERIOD_MS; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }
    CHECK(param.ExpectMA == 65);
    return 0;
}

static int test_speed_integral_adapts_breakaway_load(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};
    int16_t initial_current;
    int16_t accumulated_current;

    default_param(&param);
    param.TorqueCurrentLimit_mA = 250U;
    param.Pid_PosVel.Kp = 80U;
    param.Pid_PosVel.Ki = 5U;
    param.MechanicalParams = (MechanicalModelParams){0};
    command.mode = SERVO_MODE_SPEED;
    command.enable = true;
    command.target_speed = 5000;

    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);
    for (uint8_t tick = 0; tick < 25U; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }
    initial_current = param.ExpectMA;

    for (uint16_t tick = 0; tick < 250U; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }
    accumulated_current = param.ExpectMA;
    CHECK(param.ExpectMA > initial_current);
    CHECK(param.ExpectMA <= (int16_t)param.TorqueCurrentLimit_mA);

    /* Once the measured speed overtakes the target, the same integrator
     * unwinds progressively instead of dropping a separate start boost. */
    param.EncoderSpeed = 2500;
    for (uint16_t tick = 0; tick < 250U; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }
    CHECK(param.ExpectMA < accumulated_current);
    return 0;
}

static int test_output_phase_map_does_not_modify_control(void)
{
    Param baseline = {0};
    Param compensated = {0};
    ServoControl baseline_servo;
    ServoControl compensated_servo;
    ServoCommand command = {0};

    default_param(&baseline);
    default_param(&compensated);
    compensated.LowSpeedCompMaxSpeed_cps = 3000U;
    compensated.LowSpeedCompMap_mA[0][0] = 20;
    compensated.LowSpeedCompMap_mA[0][1] = 40;
    compensated.LowSpeedCompMap_mA[1][1] = 10;
    compensated.LowSpeedCompMap_mA[1][2] = 30;

    command.mode = SERVO_MODE_SPEED;
    command.enable = true;
    command.target_speed = 1500;
    baseline.EncoderValue = 256U;
    compensated.EncoderValue = 256U;
    ServoControl_Init(&baseline_servo, &baseline);
    ServoControl_Init(&compensated_servo, &compensated);
    ServoControl_SetCommand(&baseline_servo, &command);
    ServoControl_SetCommand(&compensated_servo, &command);
    ServoControl_Begin1ms(&baseline_servo);
    ServoControl_Run1ms(&baseline_servo);
    ServoControl_Begin1ms(&compensated_servo);
    ServoControl_Run1ms(&compensated_servo);
    CHECK(compensated.ExpectMA == baseline.ExpectMA);

    default_param(&baseline);
    default_param(&compensated);
    compensated.LowSpeedCompMaxSpeed_cps = 3000U;
    compensated.LowSpeedCompMap_mA[1][1] = 10;
    compensated.LowSpeedCompMap_mA[1][2] = 30;
    command.target_speed = -1500;
    baseline.EncoderValue = 768U;
    compensated.EncoderValue = 768U;
    ServoControl_Init(&baseline_servo, &baseline);
    ServoControl_Init(&compensated_servo, &compensated);
    ServoControl_SetCommand(&baseline_servo, &command);
    ServoControl_SetCommand(&compensated_servo, &command);
    ServoControl_Begin1ms(&baseline_servo);
    ServoControl_Run1ms(&baseline_servo);
    ServoControl_Begin1ms(&compensated_servo);
    ServoControl_Run1ms(&compensated_servo);
    CHECK(compensated.ExpectMA == baseline.ExpectMA);

    compensated.SpeedRef = 3000;
    compensated.EncoderSpeed = 3000;
    command.target_speed = 4000;
    ServoControl_SetCommand(&compensated_servo, &command);
    ServoControl_Begin1ms(&compensated_servo);
    ServoControl_Run1ms(&compensated_servo);
    CHECK(compensated.ExpectMA > 0);
    return 0;
}

static int test_physical_reversal_waits_for_load_speed_zero(void)
{
    Param param = {0};

    default_param(&param);
    param.DecelMax = 300U;
    param.AccelMax = 200U;
    param.SpeedRef = 600;
    param.EncoderSpeed = 1200;
    param.Pid_PosVel.integral = 400000;

    (void)PID_SpeedLoop(&param, -5000, 1U);
    CHECK(param.SpeedRef == 300);
    CHECK(param.Pid_PosVel.integral == 0);
    (void)PID_SpeedLoop(&param, -5000, 1U);
    CHECK(param.SpeedRef == 0);
    CHECK(param.Pid_PosVel.integral == 0);

    /* The planned reference stays at zero while the output shaft is still
     * moving on the old tooth face. */
    (void)PID_SpeedLoop(&param, -5000, 1U);
    CHECK(param.SpeedRef == 0);
    CHECK(param.Pid_PosVel.integral == 0);

    /* Once measured speed enters the existing stall/zero-speed window, the
     * opposite reference starts from zero instead of a preloaded integrator. */
    param.EncoderSpeed = 300;
    (void)PID_SpeedLoop(&param, -5000, 1U);
    CHECK(param.SpeedRef == -200);
    CHECK(param.Pid_PosVel.integral < 0);
    return 0;
}

static int test_velocity_overspeed_brakes_and_reversal_remains_active(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    command.mode = SERVO_MODE_SPEED;
    command.enable = true;
    command.target_speed = 5000;
    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    param.SpeedRef = 5000;
    param.EncoderSpeed = 8000;
    param.Pid_PosVel.integral = 12345;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.ExpectMA < 0);
    CHECK(param.DrivePower < 0);
    CHECK((param.CurrentLoopStatus & 0x0080U) != 0U);

    command.target_speed = -5000;
    ServoControl_SetCommand(&servo, &command);
    param.SpeedRef = -5000;
    param.EncoderSpeed = 3000;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.ExpectMA < 0);
    return 0;
}

static int test_mixed_decay_is_delegated_to_pwm_loop(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    param.DrivePwmMode = 4U;
    param.INA181_mA = 200;
    command.mode = SERVO_MODE_CURRENT;
    command.enable = true;
    command.target_current_mA = 100;

    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.DriveRunMode == 2U);

    param.INA181_mA = 100;
    for (uint8_t i = 0; i < 4U; i++)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }
    CHECK(param.DriveRunMode == 2U);
    return 0;
}

static int test_current_mode_speed_limit(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    command.mode = SERVO_MODE_CURRENT;
    command.enable = true;
    command.target_current_mA = 100;

    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);

    param.EncoderSpeed = 40000;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.DrivePower == 0);

    param.EncoderSpeed = 26000;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.DrivePower > 0);

    command.target_current_mA = -100;
    ServoControl_SetCommand(&servo, &command);
    param.EncoderSpeed = 40000;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.ExpectMA < 0);

    command.target_current_mA = 1500;
    ServoControl_SetCommand(&servo, &command);
    param.TorqueCurrentLimit_mA = 400U;
    param.EncoderSpeed = 0;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.ExpectMA == 400);
    return 0;
}

static int test_torque_mode_and_voltage_limit(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    param.MotorTorqueParams = (MotorTorqueModelParams){
        .torque_constant_uNm_per_A = 7990U,
        .back_emf_uV_per_rpm = 837U,
        .terminal_resistance_mOhm = 40700U,
        .resistance_temp_coefficient_ppm_per_C = 4000U,
        .reference_temperature_C = 22
    };
    param.MechanicalParams.coulomb_friction_uNm = 9450U;
    param.MechanicalParams.friction_deadband_cps = 300U;
    command.mode = SERVO_MODE_TORQUE;
    command.enable = true;
    command.target_torque_uNm = 1598;

    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.OutputEnabled);
    CHECK(param.ExpectMA == 200);
    CHECK(param.TargetTorque_uNm == 1598);
    CHECK(param.TargetElectromagneticTorque_uNm == 1598);
    CHECK(!param.TorqueCommandVoltageLimited);

    param.VCC_mV = 4000U;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.OutputEnabled);
    CHECK(param.ExpectMA == 98);
    CHECK(param.TorqueCommandVoltageLimited);

    param.VCC_mV = 12000U;
    param.TorqueCurrentLimit_mA = 100U;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.ExpectMA == 100);
    return 0;
}

static int test_speed_loop_uses_effective_current_limit(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    param.Pid_PosVel.out_max = 750U;
    param.TorqueCurrentLimit_mA = 400U;
    command.mode = SERVO_MODE_SPEED;
    command.enable = true;
    command.target_speed = 30000;
    command.current_limit_mA = 100U;
    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);

    for (uint16_t tick = 0U; tick < 100U; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
        CHECK(abs(param.ExpectMA) <= 100);
    }
    CHECK(abs(param.Pid_PosVel.prev_out) <= 100);
    return 0;
}

static int test_stale_encoder_inhibits_closed_motion(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    param.EncoderSampleAgeMs = SERVO_ENCODER_TIMEOUT_MS + 1U;
    command.mode = SERVO_MODE_SPEED;
    command.enable = true;
    command.target_speed = 1000;
    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);

    CHECK(!param.OutputEnabled);
    CHECK(param.DrivePower == 0);
    CHECK(param.FaultCode == SERVO_FAULT_ENCODER);
    CHECK((param.ProtectionFlags & PROTECTION_ENCODER) != 0U);
    return 0;
}

static int test_invalid_torque_model_inhibits_output(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    param.MotorTorqueParams = (MotorTorqueModelParams){0};
    command.mode = SERVO_MODE_TORQUE;
    command.enable = true;
    command.target_torque_uNm = 1000;
    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);

    CHECK(!param.OutputEnabled);
    CHECK(param.ExpectMA == 0);
    CHECK(param.DrivePower == 0);
    CHECK((param.ProtectionFlags & PROTECTION_TORQUE_MODEL_INVALID) != 0U);

    param.MotorTorqueParams = (MotorTorqueModelParams){
        .torque_constant_uNm_per_A = 1000U,
        .torque_temp_coefficient_ppm_per_C = -10000,
        .back_emf_uV_per_rpm = 100U,
        .terminal_resistance_mOhm = 1000U,
        .reference_temperature_C = 20
    };
    param.MotorWindingTemperature_C = 200;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(!param.OutputEnabled);
    CHECK((param.ProtectionFlags & PROTECTION_TORQUE_MODEL_INVALID) != 0U);

    default_param(&param);
    param.MotorTorqueParams = (MotorTorqueModelParams){0};
    command.mode = SERVO_MODE_SPEED;
    command.target_speed = 1000;
    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(!param.OutputEnabled);
    CHECK(param.DrivePower == 0);
    CHECK((param.ProtectionFlags & PROTECTION_TORQUE_MODEL_INVALID) != 0U);
    return 0;
}

static int test_cycle_overcurrent_does_not_latch_fault(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    command.mode = SERVO_MODE_CURRENT;
    command.enable = true;
    command.target_current_mA = 100;
    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);
    param.CurrentHardLimitActive = true;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);

    CHECK(param.OutputEnabled);
    CHECK((param.ProtectionFlags & PROTECTION_OVERCURRENT) == 0U);
    CHECK(param.FaultCode == 0U);
    return 0;
}

static int test_sustained_overcurrent_stops_then_recovers(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    param.CurrentPeakLimit_mA = 1500U;
    param.StallConfirmTimeMs = 3U;
    param.CurrentAverage_mA = 1600;
    param.ControlSource = CONTROL_SOURCE_PWM_INPUT;
    command.mode = SERVO_MODE_CURRENT;
    command.enable = true;
    command.target_current_mA = 100;
    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);

    for (uint8_t tick = 0U; tick < 2U; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
        CHECK(param.FaultCode == 0U);
        CHECK(param.OutputEnabled);
    }
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.FaultCode == 0U);
    CHECK(!param.OutputEnabled);
    CHECK((param.ProtectionFlags & PROTECTION_OVERCURRENT) != 0U);

    param.CurrentAverage_mA = 0;
    for (uint8_t tick = 0U; tick < 3U; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
        CHECK(param.FaultCode == 0U);
        CHECK(!param.OutputEnabled);
        CHECK((param.ProtectionFlags & PROTECTION_OVERCURRENT) != 0U);
    }
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.FaultCode == 0U);
    CHECK(param.OutputEnabled);
    CHECK((param.ProtectionFlags & PROTECTION_OVERCURRENT) == 0U);

    default_param(&param);
    param.CurrentPeakLimit_mA = 1500U;
    param.StallConfirmTimeMs = 2U;
    param.CurrentAverage_mA = 1600;
    param.ControlSource = CONTROL_SOURCE_SERIAL;
    command.enable = true;
    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);
    for (uint8_t tick = 0U; tick < 2U; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }
    CHECK(param.FaultCode == 0U);
    CHECK(!param.OutputEnabled);
    param.CurrentAverage_mA = 0;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(!param.OutputEnabled);
    CHECK((param.ProtectionFlags & PROTECTION_OVERCURRENT) != 0U);

    command.enable = false;
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    command.enable = true;
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.FaultCode == 0U);
    CHECK(param.OutputEnabled);
    CHECK((param.ProtectionFlags & PROTECTION_OVERCURRENT) == 0U);

    default_param(&param);
    param.CurrentPeakLimit_mA = 1500U;
    param.StallConfirmTimeMs = 2U;
    param.CurrentAverage_mA = 1600;
    param.ControlSource = CONTROL_SOURCE_CRSF;
    command.enable = true;
    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);
    for (uint8_t tick = 0U; tick < 2U; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }
    CHECK(!param.OutputEnabled);
    CHECK(param.FaultCode == 0U);
    param.CurrentAverage_mA = 0;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(!param.OutputEnabled);
    command.enable = false;
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    command.enable = true;
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.OutputEnabled);
    return 0;
}

static int test_disarmed_command_does_not_relatch_stale_overcurrent(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    command.mode = SERVO_MODE_CURRENT;
    command.enable = false;
    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);

    /* The actuator latch is cleared after ServoControl_Run1ms by the main
     * loop.  A clear-fault command must remain cleared during this interval. */
    param.CurrentHardLimitActive = true;
    param.FaultCode = 0U;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);

    CHECK(!param.OutputEnabled);
    CHECK((param.ProtectionFlags & PROTECTION_OVERCURRENT) == 0U);
    CHECK(param.FaultCode == 0U);
    CHECK(param.DrivePower == 0);
    return 0;
}

static int test_stall_is_delayed_and_closed_motion_only(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    param.FaultCode = 0U;
    param.StallCurrentThreshold_mA = 100U;
    param.StallSpeedThreshold_cps = 50U;
    param.StallConfirmTimeMs = 3U;
    param.CurrentAverage_mA = 400;
    command.mode = SERVO_MODE_SPEED;
    command.enable = true;
    command.target_speed = 1000;
    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);

    for (uint8_t tick = 0U; tick < 2U; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
        CHECK(param.FaultCode == 0U);
    }
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.FaultCode == SERVO_FAULT_STALL);
    CHECK(!param.OutputEnabled);
    CHECK((param.ProtectionFlags & PROTECTION_STALL) != 0U);

    default_param(&param);
    param.FaultCode = 0U;
    param.StallCurrentThreshold_mA = 100U;
    param.StallSpeedThreshold_cps = 50U;
    param.StallConfirmTimeMs = 3U;
    param.CurrentAverage_mA = 400;
    command.mode = SERVO_MODE_CURRENT;
    command.target_current_mA = 400;
    command.target_speed = 0;
    ServoControl_Init(&servo, &param);
    ServoControl_SetCommand(&servo, &command);
    for (uint8_t tick = 0U; tick < 10U; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }
    CHECK(param.FaultCode == 0U);
    CHECK(param.StallElapsedMs == 0U);
    return 0;
}

typedef struct
{
    int32_t speed_cps;
    int32_t position_milli_count;
} CascadePlant;

static void cascade_plant_tick(Param *param, CascadePlant *plant)
{
    int32_t current_mA = param->OutputEnabled ? param->ExpectMA : 0;
    int32_t steady_speed = current_mA * 20;

    plant->speed_cps += (steady_speed - plant->speed_cps) / 50;
    plant->position_milli_count += plant->speed_cps;
    param->EncoderSpeed = plant->speed_cps;
    param->EncoderMultiTurnValue = plant->position_milli_count / 1000;
    param->EncoderValue = (uint16_t)param->EncoderMultiTurnValue & 0x3FFFU;
    param->EncoderFeedbackValid = true;
    param->EncoderSampleAgeMs = 0U;
    param->CurrentAverage_mA = (int16_t)abs(current_mA);
    param->CurrentLogical_mA = (int16_t)current_mA;
    param->CurrentSampleValid = true;
    param->CurrentSampleAgeMs = 0U;
}

static void run_cascade_ticks(ServoControl *servo, Param *param,
                              CascadePlant *plant, uint16_t ticks)
{
    for (uint16_t tick = 0U; tick < ticks; ++tick)
    {
        ServoControl_Begin1ms(servo);
        ServoControl_Run1ms(servo);
        cascade_plant_tick(param, plant);
    }
}

static int test_dynamic_speed_and_position_convergence(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};
    CascadePlant plant = {0};

    default_param(&param);
    param.TorqueCurrentLimit_mA = 400U;
    param.StallConfirmTimeMs = 5000U;
    ServoControl_Init(&servo, &param);

    command.mode = SERVO_MODE_SPEED;
    command.enable = true;
    command.target_speed = 5000;
    ServoControl_SetCommand(&servo, &command);
    run_cascade_ticks(&servo, &param, &plant, 3000U);
    if (abs(param.EncoderSpeed - 5000) >= 250)
        printf("dynamic +speed=%ld iref=%d integral=%ld\n",
               (long)param.EncoderSpeed, param.ExpectMA,
               (long)param.Pid_PosVel.integral);
    CHECK(abs(param.EncoderSpeed - 5000) < 250);
    CHECK(param.ExpectMA > 0);

    command.target_speed = -3000;
    ServoControl_SetCommand(&servo, &command);
    run_cascade_ticks(&servo, &param, &plant, 2500U);
    if (abs(param.EncoderSpeed + 3000) >= 250)
        printf("dynamic -speed=%ld iref=%d integral=%ld\n",
               (long)param.EncoderSpeed, param.ExpectMA,
               (long)param.Pid_PosVel.integral);
    CHECK(abs(param.EncoderSpeed + 3000) < 250);
    CHECK(param.ExpectMA <= 0);

    command.mode = SERVO_MODE_POSITION;
    command.position_multi_turn = true;
    command.target_position = param.EncoderMultiTurnValue + 4000;
    command.speed_limit_cps = 6000U;
    ServoControl_SetCommand(&servo, &command);
    run_cascade_ticks(&servo, &param, &plant, 4000U);
    CHECK(abs(param.EncoderMultiTurnValue - command.target_position) < 100);
    CHECK(abs(param.EncoderSpeed) < 500);
    CHECK(param.FaultCode == 0U);
    return 0;
}

static int test_single_and_multi_turn_position_targets(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    param.EncoderMultiTurnValue = 16000;
    param.EncoderValue = 16000;
    ServoControl_Init(&servo, &param);
    command.mode = SERVO_MODE_POSITION;
    command.enable = true;
    command.target_position = 200;
    ServoControl_SetCommand(&servo, &command);
    for (uint8_t tick = 0U; tick < SERVO_POSITION_PERIOD_MS; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }
    CHECK(servo.position_speed_target < 0);

    /* A small boundary overshoot must recover toward the latched target instead
     * of moving the target into the adjacent revolution. */
    param.EncoderMultiTurnValue = -20;
    param.EncoderValue = 16364;
    ServoControl_SetCommand(&servo, &command);
    for (uint8_t tick = 0U; tick < SERVO_POSITION_PERIOD_MS; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }
    CHECK(servo.position_speed_target > 0);

    param.EncoderMultiTurnValue = 200;
    param.EncoderValue = 200;
    command.target_position = 16000;
    ServoControl_SetCommand(&servo, &command);
    for (uint8_t tick = 0U; tick < SERVO_POSITION_PERIOD_MS; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }
    CHECK(servo.position_speed_target > 0);

    command.position_multi_turn = true;
    command.target_position = 33768;
    ServoControl_SetCommand(&servo, &command);
    for (uint8_t tick = 0U; tick < SERVO_POSITION_PERIOD_MS; ++tick)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
    }
    CHECK(servo.position_speed_target > 0);
    return 0;
}

static int test_diagnostic_pwm_duty_is_direct_and_protected(void)
{
    Param param = {0};
    ServoControl servo;
    ServoCommand command = {0};

    default_param(&param);
    ServoControl_Init(&servo, &param);
    command.mode = SERVO_MODE_PWM_DUTY;
    command.enable = true;
    command.target_current_mA = 375;
    ServoControl_SetCommand(&servo, &command);
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(param.OutputEnabled);
    CHECK(param.DrivePower == 375);
    CHECK(param.ExpectMA == 375);

    param.EncoderFeedbackValid = false;
    ServoControl_Begin1ms(&servo);
    ServoControl_Run1ms(&servo);
    CHECK(!param.OutputEnabled);
    CHECK(param.DrivePower == 0);
    CHECK(param.FaultCode == SERVO_FAULT_ENCODER);
    return 0;
}

int main(void)
{
    CHECK(test_pid_reset() == 0);
    CHECK(test_loop_limits() == 0);
    CHECK(test_breakaway_uses_speed_pi_integrator() == 0);
    CHECK(test_low_speed_uses_same_direction_pulse_density() == 0);
    CHECK(test_model_fed_low_current_loop() == 0);
    CHECK(test_active_window_peak_never_trims_average_model() == 0);
    CHECK(test_legacy_current_tuning_migration() == 0);
    CHECK(test_position_derivative_uses_speed_units() == 0);
    CHECK(test_speed_plan_uses_scaled_limits() == 0);
    CHECK(test_pwm_position_command() == 0);
    CHECK(test_mode_switch_and_power_save() == 0);
    CHECK(test_fault_brake_policy() == 0);
    CHECK(test_protection_inhibit_and_watchdog_fallback() == 0);
    CHECK(test_speed_and_position_cascades() == 0);
    CHECK(test_model_assisted_speed_cascade() == 0);
    CHECK(test_speed_integral_adapts_breakaway_load() == 0);
    CHECK(test_output_phase_map_does_not_modify_control() == 0);
    CHECK(test_physical_reversal_waits_for_load_speed_zero() == 0);
    CHECK(test_velocity_overspeed_brakes_and_reversal_remains_active() == 0);
    CHECK(test_mixed_decay_is_delegated_to_pwm_loop() == 0);
    CHECK(test_current_mode_speed_limit() == 0);
    CHECK(test_torque_mode_and_voltage_limit() == 0);
    CHECK(test_speed_loop_uses_effective_current_limit() == 0);
    CHECK(test_stale_encoder_inhibits_closed_motion() == 0);
    CHECK(test_invalid_torque_model_inhibits_output() == 0);
    CHECK(test_cycle_overcurrent_does_not_latch_fault() == 0);
    CHECK(test_sustained_overcurrent_stops_then_recovers() == 0);
    CHECK(test_disarmed_command_does_not_relatch_stale_overcurrent() == 0);
    CHECK(test_stall_is_delayed_and_closed_motion_only() == 0);
    CHECK(test_dynamic_speed_and_position_convergence() == 0);
    CHECK(test_single_and_multi_turn_position_targets() == 0);
    CHECK(test_diagnostic_pwm_duty_is_direct_and_protected() == 0);
    printf("PASS\n");
    return 0;
}
