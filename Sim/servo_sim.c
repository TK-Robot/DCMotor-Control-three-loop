/**
 * @file servo_sim.c
 * @brief PC-side servo control simulation entry.
 * @brief PC 端伺服控制仿真入口。
 */

#include <stdio.h>
#include <stdlib.h>

#include "PID.h"
#include "ServoControl.h"

/**
 * @brief Minimal first-order DC motor model for control-logic simulation.
 * @brief 用于控制逻辑仿真的简化一阶直流电机模型。
 */
typedef struct
{
    int32_t position; ///< Simulated position. / 仿真位置。
    int32_t speed;    ///< Simulated speed. / 仿真速度。
    int16_t current;  ///< Simulated current. / 仿真电流。
} MotorModel;

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

static void model_update(MotorModel *model, Param *param)
{
    int32_t load = model->speed / 80;
    int32_t accel = ((int32_t)param->DrivePower - load) / 2;

    model->current = (int16_t)(param->DrivePower - model->speed / 120);
    model->speed += accel;
    model->position += model->speed / 1000;

    param->INA181_mA = model->current;
    param->EncoderSpeed = model->speed;
    param->EncoderMultiTurnValue = model->position;
    param->EncoderValue = (uint16_t)(model->position & 0x3FFF);
}

static void run_case(ServoMode mode, int32_t target, const char *name)
{
    Param param = {0};
    ServoControl servo;
    MotorModel model = {0};
    ServoCommand command = {0};

    default_param(&param);
    ServoControl_Init(&servo, &param);

    command.mode = mode;
    command.enable = true;
    command.target_current_mA = (int16_t)target;
    command.target_speed = target;
    command.target_position = target;
    ServoControl_SetCommand(&servo, &command);

    printf("# case=%s\n", name);
    printf("time,mode,target,position,speed,current,pwm\n");
    for (uint32_t t = 0; t < 1000; t++)
    {
        ServoControl_Begin1ms(&servo);
        ServoControl_Run1ms(&servo);
        model_update(&model, &param);

        if ((t % 10U) == 0U)
        {
            printf("%lu,%d,%ld,%ld,%ld,%d,%d\n",
                   (unsigned long)t,
                   (int)mode,
                   (long)target,
                   (long)param.EncoderMultiTurnValue,
                   (long)param.EncoderSpeed,
                   param.INA181_mA,
                   param.DrivePower);
        }
    }
}

int main(void)
{
    run_case(SERVO_MODE_CURRENT, 120, "current");
    run_case(SERVO_MODE_SPEED, 8000, "speed");
    run_case(SERVO_MODE_POSITION, 3000, "position");
    return 0;
}
