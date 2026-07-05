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
    param->AccelMax = 60000;
    param->DecelMax = 60000;
    param->PowerSaveVoltage_mV = 4000;
    param->VCC_mV = 12000;
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

    default_param(&param);
    param.EncoderMultiTurnValue = 0;
    CHECK(PID_PositionLoop(&param, 1000000) <= param.SpeedMax);

    param.EncoderSpeed = -50000;
    CHECK(abs(PID_SpeedLoop(&param, 50000)) <= param.Pid_PosVel.out_max);

    param.INA181_mA = -5000;
    CHECK(abs(PID_CurrentLoop(&param, 5000)) <= 1000);
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
    return 0;
}

int main(void)
{
    CHECK(test_pid_reset() == 0);
    CHECK(test_loop_limits() == 0);
    CHECK(test_mode_switch_and_power_save() == 0);
    printf("PASS\n");
    return 0;
}
