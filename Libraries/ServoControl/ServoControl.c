/**
 * @file ServoControl.c
 * @brief Three-mode cascaded servo control scheduler implementation.
 * @brief 三模式级联伺服控制调度器实现。
 */

#include "ServoControl.h"

#include "PID.h"

#include <stddef.h>

static bool ServoControl_IsPowerLow(const ServoControl *servo)
{
    const Param *param = servo->param;

    return (param->PowerSaveVoltage_mV != 0U) &&
           (param->VCC_mV < param->PowerSaveVoltage_mV);
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
    command->target_current_mA = 0;
    command->target_speed = 0;
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

static void ServoControl_ResetDecayState(ServoControl *servo)
{
    servo->decay_mode = 2U;
    servo->decay_hold_ms = 0U;
    servo->last_target_current_mA = 0;
}

static bool ServoControl_CurrentSpeedLimited(ServoControl *servo)
{
    const int32_t speed = servo->param->EncoderSpeed;
    const int32_t target = servo->command.target_current_mA;
    const int32_t limit = servo->param->SpeedMax;
    const int32_t release_limit = (limit * 9) / 10;
    const int32_t speed_abs = ServoControl_AbsI32(speed);

    if (servo->command.mode != SERVO_MODE_CURRENT ||
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

static uint8_t ServoControl_SelectDecayMode(ServoControl *servo)
{
    Param *param = servo->param;
    int32_t target_mA;
    int32_t actual_mA;
    int32_t current_error_mA;
    bool target_reversed;

    if (param->DrivePwmMode == 2U || param->DrivePwmMode == 3U)
    {
        return param->DrivePwmMode;
    }

    if (param->DrivePwmMode != 4U)
    {
        return 2U;
    }

    target_mA = ServoControl_AbsI32(param->ExpectMA);
    actual_mA = ServoControl_AbsI32(param->INA181_mA);
    current_error_mA = actual_mA - target_mA;
    target_reversed = (param->ExpectMA != 0) &&
                      (servo->last_target_current_mA != 0) &&
                      ((param->ExpectMA < 0) != (servo->last_target_current_mA < 0));
    servo->last_target_current_mA = param->ExpectMA;

    if (servo->decay_hold_ms > 0U)
    {
        servo->decay_hold_ms--;
        return servo->decay_mode;
    }

    if (servo->decay_mode != 3U &&
        (current_error_mA > SERVO_DECAY_FAST_ERROR_MA ||
         (target_reversed && actual_mA > SERVO_DECAY_SLOW_ERROR_MA)))
    {
        servo->decay_mode = 3U;
        servo->decay_hold_ms = SERVO_DECAY_MIN_HOLD_MS;
    }
    else if (servo->decay_mode == 3U &&
             current_error_mA < SERVO_DECAY_SLOW_ERROR_MA &&
             !target_reversed)
    {
        servo->decay_mode = 2U;
        servo->decay_hold_ms = SERVO_DECAY_MIN_HOLD_MS;
    }

    return servo->decay_mode;
}

static void ServoControl_RunPositionSpeedCascade(ServoControl *servo)
{
    Param *param = servo->param;
    int32_t speed_target = param->EncoderSpeedExpect;

    /* Position loop produces the speed reference; speed loop produces current. */
    /* 位置环生成速度给定，速度环再生成电流给定。 */
    if (servo->position_due)
    {
        speed_target = PID_PositionLoop(param, servo->command.target_position);
    }
    if (servo->speed_due)
    {
        (void)PID_SpeedLoop(param, speed_target);
    }
}

void ServoControl_Init(ServoControl *servo, Param *param)
{
    servo->param = param;
    servo->command.mode = SERVO_MODE_CURRENT;
    servo->command.enable = false;
    servo->command.target_current_mA = 0;
    servo->command.target_speed = 0;
    servo->command.target_position = 0;
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
    param->OutputEnabled = false;
    param->ProtectionFlags = PROTECTION_NONE;
    ServoControl_ResetDecayState(servo);
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
    PID_Reset(&param->Pid_Vel);
    PID_Reset(&param->Pid_Ele);
    param->SpeedRef = 0;
    param->EncoderSpeedExpect = 0;
    param->ExpectMA = 0;
    servo->current_speed_limit_active = false;
    ServoControl_ResetDecayState(servo);
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

    param->OutputEnabled = false;
    param->ProtectionFlags = PROTECTION_NONE;

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
    if (param->ProtectionFlags != PROTECTION_NONE)
    {
        param->DriveRunMode = 0;
        param->DrivePower = 0;
        ServoControl_ResetLoops(servo);
        return;
    }

    if (!servo->command.enable || servo->power_low_latched)
    {
        /* A latched fault may request passive braking; it never marks output enabled. */
        /* 锁存故障可请求被动制动，但不会把输出状态标记为已使能。 */
        param->DriveRunMode = (param->FaultCode != 0U
                               && param->FailSafePolicy == FAILSAFE_BRAKE) ? 1U : 0U;
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
            (void)PID_SpeedLoop(param, servo->command.target_speed);
        }
    }
    else
    {
        param->ExpectMA = servo->command.target_current_mA;
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
    param->DriveRunMode = ServoControl_SelectDecayMode(servo);
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
