/**
 * @file ServoControl.c
 * @brief Three-mode cascaded servo control scheduler implementation.
 * @brief 三模式级联伺服控制调度器实现。
 */

#include "ServoControl.h"

#include "PID.h"

static bool ServoControl_IsPowerLow(const ServoControl *servo)
{
    const Param *param = servo->param;

    return (param->PowerSaveVoltage_mV != 0U) &&
           (param->VCC_mV < param->PowerSaveVoltage_mV);
}

static bool ServoControl_IsPowerRecovered(const ServoControl *servo)
{
    const Param *param = servo->param;

    return (param->PowerSaveVoltage_mV == 0U) ||
           (param->VCC_mV > (uint16_t)(param->PowerSaveVoltage_mV + SERVO_POWER_RECOVER_HYST_MV));
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

    if (servo->command.mode != servo->last_mode)
    {
        ServoControl_ResetLoops(servo);
        servo->last_mode = servo->command.mode;
    }

    if (ServoControl_IsPowerLow(servo) || (param->Temp > param->TempLimit))
    {
        param->DriveRunMode = 0;
        param->DrivePower = 0;
        if (!servo->power_low_latched && ServoControl_IsPowerLow(servo))
        {
            servo->save_request = true;
        }
        servo->power_low_latched = true;
        return;
    }

    if (servo->power_low_latched && ServoControl_IsPowerRecovered(servo))
    {
        servo->power_low_latched = false;
    }

    if (!servo->command.enable || servo->power_low_latched)
    {
        param->DriveRunMode = 0;
        param->DrivePower = 0;
        ServoControl_ResetLoops(servo);
        return;
    }

    param->DriveRunMode = 2;

    if (servo->command.mode == SERVO_MODE_POSITION)
    {
        if (servo->position_due)
        {
            (void)PID_PositionLoop(param, servo->command.target_position);
        }
        if (servo->speed_due)
        {
            (void)PID_SpeedLoop(param, param->EncoderSpeedExpect);
        }
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

    (void)PID_CurrentLoop(param, param->ExpectMA);
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
