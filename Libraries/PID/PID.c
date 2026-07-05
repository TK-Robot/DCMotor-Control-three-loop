#include "PID.h"

#include <stdlib.h>

void PID_Init(Param *param)
{
    param->Pid_Pos.Kp = 10500;
    param->Pid_Pos.Ki = 0;
    param->Pid_Pos.Kd = 0;
    param->Pid_Pos.out_min = 200;
    param->Pid_Pos.out_max = 35000;
    param->Pid_Pos.integral_max = 35000;

    param->Pid_PosVel.Kp = 8;
    param->Pid_PosVel.Ki = 15;
    param->Pid_PosVel.Kd = 14;
    param->Pid_PosVel.out_min = 10;
    param->Pid_PosVel.out_max = 750;
    param->Pid_PosVel.integral_max = 8000;

    param->Pid_PosEle.Kp = 3200;
    param->Pid_PosEle.Ki = 420;
    param->Pid_PosEle.Kd = 0;
    param->Pid_PosEle.out_min = 5;
    param->Pid_PosEle.out_max = 750;
    param->Pid_PosEle.integral_max = 8000;

    param->Pid_Vel = param->Pid_PosVel;
    param->Pid_Ele = param->Pid_PosEle;
}

void PID_Reset(PID_Int *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_prev_error = 0;
    pid->prev_feedback = 0;
    pid->prev_out = 0;
}

int32_t PID_AbsCalculate(PID_Int* pid,int32_t setValue,int32_t CurrentValue)
{
    int32_t err,out;
    int32_t K,I,D;

    err = setValue - CurrentValue;
    if(abs(err)<=PID_AbsDEADBAND) err=0;

    pid->integral += err;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;

    K=pid->Kp*err/PID_SCALE;
    I=pid->Ki*pid->integral/PID_SCALE;
    D=pid->Kd*(err-pid->prev_error)/PID_SCALE;

    out=K+I+D;
    pid->prev_error=err;

    if (out > pid->out_max) out = pid->out_max;
    if (out < -pid->out_max) out = -pid->out_max;
    if (out > 0 && out < pid->out_min) out = pid->out_min;
    else if (out < 0 && -out < pid->out_min) out = -(int32_t)pid->out_min;

    return out;
}

int16_t PID_Vel_Calc(PID_Int* pid, int32_t target, int32_t feedback)
{
    int32_t error;
    int32_t P, I, D;
    int32_t out;

    error = target - feedback;
    P = pid->Kp * (error - pid->prev_error) / PID_SCALE;
    I = pid->Ki * error / PID_SCALE / 10;
    D = -pid->Kd * (feedback - pid->prev_feedback) / PID_SCALE;

    out = pid->prev_out + P + I + D;
    if (out > pid->out_max) out = pid->out_max;
    if (out < -pid->out_max) out = -pid->out_max;

    pid->prev_feedback = feedback;
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;
    pid->prev_out = (int16_t)out;

    return (int16_t)out;
}

int32_t PID_PositionLoop(Param *param, int32_t target_position)
{
    int32_t speed = PID_AbsCalculate(&param->Pid_Pos, target_position, param->EncoderMultiTurnValue);

    if (speed > param->SpeedMax) speed = param->SpeedMax;
    else if (speed < -(int32_t)param->SpeedMax) speed = -(int32_t)param->SpeedMax;

    param->EncoderExpect = (int16_t)target_position;
    param->EncoderSpeedExpect = speed;
    return speed;
}

int16_t PID_SpeedLoop(Param *param, int32_t target_speed)
{
    int32_t planned_speed = SpeedPlan_Update(param, target_speed);
    int16_t current = PID_Vel_Calc(&param->Pid_PosVel, planned_speed, param->EncoderSpeed);

    param->EncoderSpeedExpect = planned_speed;
    param->ExpectMA = current;
    return current;
}

int16_t PID_CurrentLoop(Param *param, int16_t target_current_mA)
{
    int32_t pwm = PID_AbsCalculate(&param->Pid_PosEle, target_current_mA, param->INA181_mA);

    if (pwm > 1000) pwm = 1000;
    else if (pwm < -1000) pwm = -1000;

    param->ExpectMA = target_current_mA;
    param->DrivePower = (int16_t)pwm;
    return (int16_t)pwm;
}

int16_t PID_Pos(Param *param)
{
    int32_t pos_err = param->EncoderMultiTurnValue - param->EncoderExpect;

    if (abs(pos_err) <= 3)
    {
        param->SpeedRef = 0;
        PID_Reset(&param->Pid_Pos);
        PID_Reset(&param->Pid_PosVel);
        return 0;
    }

    (void)PID_PositionLoop(param, param->EncoderExpect);
    (void)PID_SpeedLoop(param, param->EncoderSpeedExpect);
    return PID_CurrentLoop(param, param->ExpectMA);
}

int32_t SpeedPlan_Update(Param *param,int32_t SpeedCmd)
{
    int32_t dv = SpeedCmd - param->SpeedRef;
    int32_t dv_acc = param->AccelMax * param->CycleTimeMs / 1000;
    int32_t dv_dec = param->DecelMax * param->CycleTimeMs / 1000;

    if (dv > 0)
    {
        if (dv > dv_acc) dv = dv_acc;
    }
    else
    {
        if (dv < -dv_dec) dv = -dv_dec;
    }

    param->SpeedRef += dv;
    return param->SpeedRef;
}

int16_t FeedForward_LUT(int32_t speed_abs)
{
    if (speed_abs <= speed_lut[0]) return pwm_lut[0];

    for (uint8_t i = 0; i < LUT_SIZE - 1; i++)
    {
        if (speed_abs <= speed_lut[i + 1])
        {
            int32_t ds = speed_lut[i + 1] - speed_lut[i];
            int32_t dp = pwm_lut[i + 1] - pwm_lut[i];
            int32_t dx = speed_abs - speed_lut[i];

            return pwm_lut[i] + (dp * dx) / ds;
        }
    }

    return pwm_lut[LUT_SIZE - 1];
}

int16_t Speed_FeedForward(int32_t speed)
{
    int32_t abs_spd = abs(speed);
    int16_t pwm;

    if (abs_spd < SPEED_LOW) pwm = FeedForward_LUT(abs_spd);
    else if (abs_spd < SPEED_MID) pwm = (abs_spd * KV_Q) / FF_SCALE + PWM_DEAD;
    else pwm = (abs_spd * KV_Q) / FF_SCALE;

    if (pwm > PWM_MAX) pwm = PWM_MAX;
    return (speed >= 0) ? pwm : -pwm;
}
