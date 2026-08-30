/**
 * @file ServoControl.c
 * @brief Four-mode cascaded servo control scheduler implementation.
 * @brief 四模式级联伺服控制调度器实现。
 */

#include "ServoControl.h"

#include "MechanicalModel.h"
#include "MotorTorqueModel.h"
#include "PID.h"

#include <stddef.h>

static bool ServoControl_IsPowerLow(const ServoControl *servo)
{
    const Param *param = servo->param;

    return (param->PowerSaveVoltage_mV != 0U) &&
           (param->VCC_mV < param->PowerSaveVoltage_mV);
}

static void ServoControl_UpdateTorqueEstimate(ServoControl *servo)
{
    Param *param = servo->param;

    if (!MotorTorqueModel_IsTorqueValid(&param->MotorTorqueParams))
    {
        param->MotorTorqueResult = (MotorTorqueModelResult){0};
        param->MechanicalResult = (MechanicalModelResult){0};
        return;
    }
    MotorTorqueModel_Evaluate(&param->MotorTorqueParams,
                              param->CurrentLogical_mA,
                              param->VCC_mV,
                              param->EncoderSpeed,
                              param->TorqueEncoderCountsPerRev,
                              param->MotorWindingTemperature_C,
                              &param->MotorTorqueResult);
    MechanicalModel_Evaluate(&param->MechanicalParams,
                             param->MotorTorqueResult.electromagnetic_torque_uNm,
                             param->EncoderSpeed,
                             param->AccDec,
                             param->TorqueEncoderCountsPerRev,
                             &param->MechanicalResult);
}

void ServoControl_BuildPwmPositionCommand(uint16_t low_width_us, bool signal_valid,
                                          ServoCommand *command)
{
    uint16_t clamped_width = low_width_us;

    if (command == NULL)
    {
        return;
    }
    command->mode = SERVO_MODE_POSITION;
    command->enable = signal_valid;
    command->position_multi_turn = false;
    command->target_current_mA = 0;
    command->target_torque_uNm = 0;
    command->target_speed = 0;
    command->current_limit_mA = 0U;
    command->speed_limit_cps = 0U;
    if (clamped_width < SERVO_PWM_POSITION_MIN_US) clamped_width = SERVO_PWM_POSITION_MIN_US;
    if (clamped_width > SERVO_PWM_POSITION_MAX_US) clamped_width = SERVO_PWM_POSITION_MAX_US;
    command->target_position = ((int32_t)(clamped_width - SERVO_PWM_POSITION_MIN_US)
                                * (SERVO_POSITION_COUNTS_PER_REV - 1L))
                               / (SERVO_PWM_POSITION_MAX_US - SERVO_PWM_POSITION_MIN_US);
}

static bool ServoControl_IsPowerRecovered(const ServoControl *servo)
{
    const Param *param = servo->param;

    return (param->PowerSaveVoltage_mV == 0U) ||
           (param->VCC_mV > (uint16_t)(param->PowerSaveVoltage_mV + SERVO_POWER_RECOVER_HYST_MV));
}

static int32_t ServoControl_AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static bool ServoControl_UpdateStallProtection(ServoControl *servo)
{
    Param *param = servo->param;
    bool closed_motion_mode = servo->command.mode == SERVO_MODE_SPEED
                           || servo->command.mode == SERVO_MODE_POSITION;
    bool motion_requested = ServoControl_AbsI32(param->EncoderSpeedExpect)
                         > param->StallSpeedThreshold_cps;
    bool speed_low = ServoControl_AbsI32(param->EncoderSpeed)
                  <= param->StallSpeedThreshold_cps;
    bool current_high = ServoControl_AbsI32(param->CurrentAverage_mA)
                     >= param->StallCurrentThreshold_mA;

    if (closed_motion_mode && motion_requested && speed_low && current_high
        && param->StallConfirmTimeMs != 0U)
    {
        uint32_t elapsed = param->StallElapsedMs + param->CycleTimeMs;
        param->StallElapsedMs = elapsed > UINT16_MAX
                                  ? UINT16_MAX : (uint16_t)elapsed;
    }
    else
    {
        param->StallElapsedMs = 0U;
    }

    if (param->StallConfirmTimeMs != 0U
        && param->StallElapsedMs >= param->StallConfirmTimeMs)
    {
        param->FaultCode = SERVO_FAULT_STALL;
        param->ProtectionFlags |= PROTECTION_STALL;
        param->OutputEnabled = false;
        param->DriveRunMode = 0U;
        param->DrivePower = 0;
        ServoControl_ResetLoops(servo);
        return true;
    }
    return false;
}

static int16_t ServoControl_ClampCurrent(int16_t current_mA, uint16_t limit_mA)
{
    if (limit_mA == 0U || limit_mA > 30000U)
    {
        return current_mA;
    }
    if ((int32_t)current_mA > (int32_t)limit_mA)
        return (int16_t)limit_mA;
    if ((int32_t)current_mA < -(int32_t)limit_mA)
        return (int16_t)-(int32_t)limit_mA;
    return current_mA;
}

static uint16_t ServoControl_EffectiveCurrentLimit(const ServoControl *servo);

static int16_t ServoControl_ApplyLowSpeedCompensation(
    ServoControl *servo, int16_t base_current_mA,
    int32_t reference_speed_cps, bool compensate_motion)
{
    Param *param = servo->param;
    uint16_t phase = param->EncoderValue & 0x3FFFU;
    uint8_t bin = (uint8_t)(phase >> 10U);
    int32_t speed_abs = ServoControl_AbsI32(reference_speed_cps);
    uint16_t cutoff = param->LowSpeedCompMaxSpeed_cps;
    const int8_t *map;
    int32_t correction;
    int32_t candidate;

    if (!compensate_motion || cutoff < 500U
        || speed_abs == 0 || speed_abs >= cutoff
        || base_current_mA == 0
        || ((reference_speed_cps ^ base_current_mA) < 0))
    {
        return base_current_mA;
    }

    map = param->LowSpeedCompMap_mA[reference_speed_cps < 0];
    correction = map[bin]
               + ((int32_t)(map[(bin + 1U) & 0x0FU] - map[bin])
                  * (phase & 0x03FFU)) / 1024L;

    /* Keep full calibration below half the cutoff, then fade continuously. */
    if (speed_abs > (int32_t)cutoff / 2L)
    {
        correction = correction * ((int32_t)cutoff - speed_abs) * 2L
                   / cutoff;
    }
    if (reference_speed_cps < 0) correction = -correction;

    candidate = (int32_t)base_current_mA + correction;
    if ((base_current_mA > 0 && candidate < 0)
        || (base_current_mA < 0 && candidate > 0))
    {
        candidate = 0;
    }
    candidate = ServoControl_ClampCurrent(
        (int16_t)candidate, ServoControl_EffectiveCurrentLimit(servo));
    return (int16_t)candidate;
}

static uint16_t ServoControl_EffectiveCurrentLimit(const ServoControl *servo)
{
    uint16_t limit = servo->param->TorqueCurrentLimit_mA;

    if (servo->command.current_limit_mA != 0U &&
        (limit == 0U || servo->command.current_limit_mA < limit))
    {
        limit = servo->command.current_limit_mA;
    }
    return limit;
}

static void ServoControl_ResolveShaftTorque(ServoControl *servo,
                                             int32_t shaft_target_torque_uNm,
                                             int32_t reference_speed_cps,
                                             int32_t reference_acceleration_cps2,
                                             bool compensate_motion)
{
    Param *param = servo->param;
    int32_t electromagnetic_target_uNm;
    int16_t target_current_mA = 0;

    electromagnetic_target_uNm = shaft_target_torque_uNm;
    if (compensate_motion)
    {
        electromagnetic_target_uNm = MechanicalModel_CompensateShaftTarget(
            &param->MechanicalParams, shaft_target_torque_uNm,
            reference_speed_cps, reference_acceleration_cps2,
            param->TorqueEncoderCountsPerRev);
    }
    (void)MotorTorqueModel_TorqueToCurrent(
        &param->MotorTorqueParams, electromagnetic_target_uNm,
        param->MotorWindingTemperature_C, &target_current_mA);

    param->TargetTorque_uNm = shaft_target_torque_uNm;
    param->TargetElectromagneticTorque_uNm = electromagnetic_target_uNm;
    target_current_mA = ServoControl_ApplyLowSpeedCompensation(
        servo, target_current_mA, reference_speed_cps, compensate_motion);
    target_current_mA = MotorTorqueModel_LimitCurrentByVoltage(
        &param->MotorTorqueParams,
        target_current_mA,
        param->VCC_mV,
        param->EncoderSpeed,
        param->TorqueEncoderCountsPerRev,
        param->MotorWindingTemperature_C,
        &param->TorqueCommandVoltageLimited);
    param->ExpectMA = ServoControl_ClampCurrent(
        target_current_mA, ServoControl_EffectiveCurrentLimit(servo));

}

static bool ServoControl_CurrentSpeedLimited(ServoControl *servo)
{
    const int32_t speed = servo->param->EncoderSpeed;
    const int32_t target = servo->param->ExpectMA;
    const int32_t limit = servo->param->SpeedMax;
    const int32_t release_limit = (limit * 9) / 10;
    const int32_t speed_abs = ServoControl_AbsI32(speed);

    if ((servo->command.mode != SERVO_MODE_CURRENT
         && servo->command.mode != SERVO_MODE_TORQUE) ||
        limit == 0U || target == 0 || speed == 0)
    {
        if (speed_abs <= release_limit)
        {
            servo->current_speed_limit_active = false;
        }
        return false;
    }

    /* Opposite current is braking and remains allowed. */
    /* 反向电流属于制动指令，允许通过速度限制。 */
    if ((speed > 0) != (target > 0))
    {
        servo->current_speed_limit_active = false;
        return false;
    }

    if (!servo->current_speed_limit_active && speed_abs >= limit)
    {
        servo->current_speed_limit_active = true;
    }
    else if (servo->current_speed_limit_active && speed_abs <= release_limit)
    {
        servo->current_speed_limit_active = false;
    }

    return servo->current_speed_limit_active;
}

static void ServoControl_RunPositionSpeedCascade(ServoControl *servo)
{
    Param *param = servo->param;
    int32_t speed_target = servo->position_speed_target;

    /* Position loop produces the speed reference; speed loop produces current. */
    /* 位置环生成速度给定，速度环再生成电流给定。 */
    if (servo->position_due)
    {
        int32_t target = servo->command.target_position;
        if (!servo->command.position_multi_turn)
        {
            const int32_t phase_target = target & 0x3FFFL;
            if ((servo->single_turn_target_absolute & 0x3FFFL) != phase_target)
            {
                servo->single_turn_target_absolute =
                    param->EncoderMultiTurnValue - (int32_t)param->EncoderValue
                    + phase_target;
            }
            target = servo->single_turn_target_absolute;
        }
        speed_target = PID_PositionLoop(param, target);
        if (servo->command.speed_limit_cps != 0U)
        {
            const int32_t limit = (int32_t)servo->command.speed_limit_cps;
            if (speed_target > limit) speed_target = limit;
            else if (speed_target < -limit) speed_target = -limit;
        }
        servo->position_speed_target = speed_target;
    }
    if (servo->speed_due)
    {
        int32_t reference_acceleration_cps2 = 0;
        int32_t shaft_target_torque_uNm;

        shaft_target_torque_uNm = PID_SpeedTorqueLoop(
            param, speed_target,
            (uint16_t)(SERVO_SPEED_PERIOD_MS * param->CycleTimeMs),
            ServoControl_EffectiveCurrentLimit(servo),
            &reference_acceleration_cps2);
        ServoControl_ResolveShaftTorque(servo,
                                        shaft_target_torque_uNm,
                                        param->EncoderSpeedExpect,
                                        reference_acceleration_cps2,
                                        true);
    }
}

void ServoControl_Init(ServoControl *servo, Param *param)
{
    servo->param = param;
    servo->command.mode = SERVO_MODE_CURRENT;
    servo->command.enable = false;
    servo->command.position_multi_turn = false;
    servo->command.target_current_mA = 0;
    servo->command.target_torque_uNm = 0;
    servo->command.target_speed = 0;
    servo->command.target_position = 0;
    servo->command.current_limit_mA = 0U;
    servo->command.speed_limit_cps = 0U;
    servo->last_mode = SERVO_MODE_CURRENT;
    servo->speed_count = 0;
    servo->position_count = 0;
    servo->telemetry_count = 0;
    servo->speed_due = false;
    servo->position_due = false;
    servo->telemetry_due = false;
    servo->save_request = false;
    servo->power_low_latched = false;
    servo->current_speed_limit_active = false;
    servo->position_speed_target = 0;
    servo->single_turn_target_absolute = 0;
    param->OutputEnabled = false;
    param->ProtectionFlags = PROTECTION_NONE;
    param->TargetTorque_uNm = 0;
    param->TargetElectromagneticTorque_uNm = 0;
    param->TorqueCommandVoltageLimited = false;
    param->StallElapsedMs = 0U;
    ServoControl_UpdateTorqueEstimate(servo);
}

void ServoControl_SetCommand(ServoControl *servo, const ServoCommand *command)
{
    servo->command = *command;
}

void ServoControl_ResetLoops(ServoControl *servo)
{
    Param *param = servo->param;

    PID_Reset(&param->Pid_Pos);
    PID_Reset(&param->Pid_PosVel);
    PID_Reset(&param->Pid_PosEle);
    param->SpeedRef = 0;
    param->EncoderSpeedExpect = 0;
    servo->position_speed_target = 0;
    servo->single_turn_target_absolute =
        param->EncoderMultiTurnValue - (int32_t)param->EncoderValue
        + (servo->command.target_position & 0x3FFFL);
    param->ExpectMA = 0;
    param->CurrentFeedforwardPwm = 0;
    param->TargetTorque_uNm = 0;
    param->TargetElectromagneticTorque_uNm = 0;
    param->TorqueCommandVoltageLimited = false;
    servo->current_speed_limit_active = false;
}

void ServoControl_Begin1ms(ServoControl *servo)
{
    servo->speed_due = false;
    servo->position_due = false;
    servo->telemetry_due = false;

    if (++servo->speed_count >= SERVO_SPEED_PERIOD_MS)
    {
        servo->speed_count = 0;
        servo->speed_due = true;
    }

    if (++servo->position_count >= SERVO_POSITION_PERIOD_MS)
    {
        servo->position_count = 0;
        servo->position_due = true;
    }

    if (++servo->telemetry_count >= SERVO_TELEMETRY_PERIOD_MS)
    {
        servo->telemetry_count = 0;
        servo->telemetry_due = true;
    }
}

void ServoControl_Run1ms(ServoControl *servo)
{
    Param *param = servo->param;
    bool power_low = ServoControl_IsPowerLow(servo);
    bool overtemperature = param->Temp > param->TempLimit;
    bool model_control_mode = servo->command.mode == SERVO_MODE_TORQUE
                           || servo->command.mode == SERVO_MODE_SPEED
                           || servo->command.mode == SERVO_MODE_POSITION;
    bool torque_model_invalid =
        model_control_mode
        && (!MotorTorqueModel_IsOperatingValid(&param->MotorTorqueParams,
                                               param->TorqueEncoderCountsPerRev,
                                               param->MotorWindingTemperature_C)
            || param->TorqueCurrentLimit_mA == 0U
            || param->TorqueCurrentLimit_mA > 30000U);
    bool encoder_feedback_invalid = servo->command.enable &&
        (servo->command.mode == SERVO_MODE_SPEED ||
         servo->command.mode == SERVO_MODE_POSITION) &&
        (!param->EncoderFeedbackValid ||
         param->EncoderSampleAgeMs > SERVO_ENCODER_TIMEOUT_MS);
    bool watchdog_fallback =
        param->FaultCode == SERVO_FAULT_SERIAL_WATCHDOG
        && param->FailSafePolicy == FAILSAFE_FALLBACK_PWM;
    bool latched_fault_inhibits =
        param->FaultCode != 0U && !watchdog_fallback;

    param->OutputEnabled = false;
    param->ProtectionFlags = PROTECTION_NONE;
    if (servo->speed_due)
    {
        ServoControl_UpdateTorqueEstimate(servo);
    }

    if (servo->command.mode != servo->last_mode)
    {
        ServoControl_ResetLoops(servo);
        servo->last_mode = servo->command.mode;
    }

    if (power_low)
    {
        if (!servo->power_low_latched)
        {
            servo->save_request = true;
        }
        servo->power_low_latched = true;
    }
    else if (servo->power_low_latched && ServoControl_IsPowerRecovered(servo))
    {
        servo->power_low_latched = false;
    }

    if (servo->power_low_latched)
    {
        param->ProtectionFlags |= PROTECTION_UNDERVOLTAGE;
    }
    if (overtemperature)
    {
        param->ProtectionFlags |= PROTECTION_OVERTEMPERATURE;
    }
    if (torque_model_invalid)
    {
        param->ProtectionFlags |= PROTECTION_TORQUE_MODEL_INVALID;
    }
    if (encoder_feedback_invalid)
    {
        param->FaultCode = SERVO_FAULT_ENCODER;
        param->ProtectionFlags |= PROTECTION_ENCODER;
    }
    if (param->FaultCode == SERVO_FAULT_ENCODER)
        param->ProtectionFlags |= PROTECTION_ENCODER;
    /* Clearing a fault disarms the command before CurrentControl is reset later
     * in the 1 ms loop.  Do not relatch the just-cleared fault from that stale
     * actuator latch while output is already disarmed. */
    if (param->CurrentHardLimitActive && servo->command.enable)
    {
        param->FaultCode = SERVO_FAULT_OVERCURRENT;
    }
    if (param->FaultCode == SERVO_FAULT_OVERCURRENT)
    {
        param->ProtectionFlags |= PROTECTION_OVERCURRENT;
    }
    if (param->FaultCode == SERVO_FAULT_STALL)
    {
        param->ProtectionFlags |= PROTECTION_STALL;
    }
    if (param->ProtectionFlags != PROTECTION_NONE || latched_fault_inhibits)
    {
        param->DriveRunMode =
            param->ProtectionFlags == PROTECTION_NONE
            && param->FaultCode == SERVO_FAULT_SERIAL_WATCHDOG
            && param->FailSafePolicy == FAILSAFE_BRAKE ? 1U : 0U;
        param->DrivePower = 0;
        ServoControl_ResetLoops(servo);
        return;
    }

    if (!servo->command.enable || servo->power_low_latched)
    {
        /* A latched fault may request passive braking; it never marks output enabled. */
        /* 锁存故障可请求被动制动，但不会把输出状态标记为已使能。 */
        param->DriveRunMode =
            param->FaultCode == SERVO_FAULT_SERIAL_WATCHDOG
            && param->FailSafePolicy == FAILSAFE_BRAKE ? 1U : 0U;
        param->DrivePower = 0;
        ServoControl_ResetLoops(servo);
        return;
    }

    param->OutputEnabled = true;

    /* A watchdog fault may coexist with valid PWM output only for explicit fallback policy 2. */
    /* 仅在显式选择回退策略 2 时，看门狗故障才可与有效 PWM 输出同时存在。 */

    if (servo->command.mode == SERVO_MODE_POSITION)
    {
        ServoControl_RunPositionSpeedCascade(servo);
    }
    else if (servo->command.mode == SERVO_MODE_SPEED)
    {
        if (servo->speed_due)
        {
            int32_t speed_target = servo->command.target_speed;
            int32_t reference_acceleration_cps2 = 0;
            int32_t shaft_target_torque_uNm;

            if (servo->command.speed_limit_cps != 0U)
            {
                const int32_t limit = (int32_t)servo->command.speed_limit_cps;
                if (speed_target > limit) speed_target = limit;
                else if (speed_target < -limit) speed_target = -limit;
            }
            shaft_target_torque_uNm = PID_SpeedTorqueLoop(
                param, speed_target,
                (uint16_t)(SERVO_SPEED_PERIOD_MS * param->CycleTimeMs),
                ServoControl_EffectiveCurrentLimit(servo),
                &reference_acceleration_cps2);
            ServoControl_ResolveShaftTorque(servo,
                                            shaft_target_torque_uNm,
                                            param->EncoderSpeedExpect,
                                            reference_acceleration_cps2,
                                            true);
            if ((param->EncoderSpeedExpect ^ param->ExpectMA) < 0)
            {
                param->TargetElectromagneticTorque_uNm = 0;
                param->ExpectMA = 0;
            }
        }
    }
    else if (servo->command.mode == SERVO_MODE_TORQUE)
    {
        ServoControl_ResolveShaftTorque(servo,
                                        servo->command.target_torque_uNm,
                                        param->EncoderSpeed,
                                        param->AccDec,
                                        false);
    }
    else
    {
        param->TargetTorque_uNm = 0;
        param->TargetElectromagneticTorque_uNm = 0;
        param->TorqueCommandVoltageLimited = false;
        param->ExpectMA = servo->command.target_current_mA;
    }

    if (param->TorqueCurrentLimit_mA != 0U)
    {
        param->ExpectMA = ServoControl_ClampCurrent(
            param->ExpectMA, param->TorqueCurrentLimit_mA);
    }

    if (servo->command.current_limit_mA != 0U)
    {
        const int16_t limit = (int16_t)servo->command.current_limit_mA;
        if (param->ExpectMA > limit) param->ExpectMA = limit;
        else if (param->ExpectMA < -limit) param->ExpectMA = (int16_t)-limit;
    }

    if (ServoControl_CurrentSpeedLimited(servo))
    {
        /* Remove same-direction torque above SpeedMax; do not auto-enable braking. */
        /* 超过 SpeedMax 时去除同向转矩，不自动切换为制动使能。 */
        PID_Reset(&param->Pid_PosEle);
        (void)PID_CurrentLoop(param, 0);
    }
    else
    {
        (void)PID_CurrentLoop(param, param->ExpectMA);
    }
    /* Mode 4 starts in slow decay. The PWM-synchronous actuator may select a
     * bounded fast-decay interval from target transients or qualified current
     * overshoot; this 1 kHz layer only publishes the configured baseline. */
    param->DriveRunMode = param->DrivePwmMode == 3U ? 3U : 2U;
    (void)ServoControl_UpdateStallProtection(servo);
}

bool ServoControl_IsSpeedDue(const ServoControl *servo)
{
    return servo->speed_due;
}

bool ServoControl_IsTelemetryDue(const ServoControl *servo)
{
    return servo->telemetry_due;
}

bool ServoControl_ConsumeSaveRequest(ServoControl *servo)
{
    bool request = servo->save_request;

    servo->save_request = false;
    return request;
}
