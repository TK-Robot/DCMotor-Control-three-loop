//
// Created by Administrator on 2025/12/15.
//

#include "PID.h"

#include <stdlib.h>

void PID_Init(Param *param)
{
    // //param->Pid_Pos.Kp = 40;
    // param->Pid_Pos.Kp = 800;
    // param->Pid_Pos.Ki = 10;
    // //param->Pid_Pos.Kd = 1600;
    // param->Pid_Pos.Kd = 30;
    // param->Pid_Pos.out_min =110;
    // param->Pid_Pos.out_max =750;
    // param->Pid_Pos.integral_max = 60000;

    param->Pid_Pos.Kp = 10500;
    param->Pid_Pos.Ki = 0;
    param->Pid_Pos.Kd = 0;
    param->Pid_Pos.out_min =200;
    param->Pid_Pos.out_max =35000;
    param->Pid_Pos.integral_max = 35000;

    // param->Pid_PosVel.Kp = 8;
    // param->Pid_PosVel.Ki = 22;
    // param->Pid_PosVel.Kd = 14;
    // param->Pid_PosVel.out_min = 110;
    // param->Pid_PosVel.out_max = 750;

    param->Pid_PosVel.Kp = 8;
    param->Pid_PosVel.Ki = 15;
    param->Pid_PosVel.Kd = 14;
    param->Pid_PosVel.out_min = 10;
    param->Pid_PosVel.out_max = 750;

    param->Pid_PosEle.Kp = 3200;
    param->Pid_PosEle.Ki = 420;
    param->Pid_PosEle.Kd = 0;
    param->Pid_PosEle.out_min = 110;
    param->Pid_PosEle.out_max = 750;
    param->Pid_PosEle.out_min =5;
    param->Pid_PosEle.out_max =750;
    param->Pid_PosEle.integral_max = 8000;
}

//绝对式PID
int32_t PID_AbsCalculate(PID_Int* pid,int32_t setValue,int32_t CurrentValue)
{
    int32_t err,out;
    int32_t K,I,D;

    err = (int16_t)setValue-(int16_t)CurrentValue;//误差值=设定值-当前值
    if(abs(err)<=PID_AbsDEADBAND) err=0;//允许误差

    pid->integral=pid->integral+err;//累计误差=累计误差+当前误差

    // 积分限幅（双向）
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }

    K=pid->Kp*err/PID_SCALE;//比例=比例系数*当前误差/放大系数
    I=pid->Ki*pid->integral/PID_SCALE;//积分=积分系数*累计误差/放大系数
    D=pid->Kd*(err-pid->prev_error)/10;//微分=比例系数*(当前误差-上次误差)/放大系数

    out=K+I+D;//PID输出计算
    pid->prev_error=err;//上次误差更新

    // 输出限幅，将输出限制在[-out_max, out_max]之间
    if (out > pid->out_max) out = pid->out_max;
    if (out < -pid->out_max) out = -pid->out_max;
    // 应用死区：如果输出绝对值小于out_min，则输出为0
    if (0<abs(out) && abs(out)< pid->out_min) out = pid->out_min;

    return out;
}

//增量式PID
int16_t PID_Vel_Calc(PID_Int* pid, int32_t target, int32_t feedback)
{
    int32_t error;
    int32_t P, I, D;
    int16_t out;

    // --- 1. 当前误差 ---
    error = target - feedback;  // error = setpoint - measured_value

    // --- 2. 增量式比例项 ---
    P = pid->Kp * (error - pid->prev_error) / PID_SCALE;

    // --- 3. 增量式积分项 ---
    // 积分可以只对本次误差乘 Ki * dt
    I = pid->Ki * error / PID_SCALE/10;  // 注意这里直接用 Ki*error，不累加到 integral
    // 如果你想做累加也可以用 pid->integral += I;

    // --- 4. 增量式微分项 ---
    /* 测速微分（推荐） */
    D = - pid->Kd * (feedback - pid->prev_feedback) / PID_SCALE;

    // --- 5. 计算本次增量 ---
    out =pid->prev_out+ P + I + D;

    // --- 6. 限幅 ---
    if (out > pid->out_max) out = pid->out_max;
    if (out < -pid->out_max) out = -pid->out_max;

    // --- 7. 更新历史误差 ---
    pid->prev_feedback = feedback;
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;
    pid->prev_out = out;

    return (int16_t)(out);
}

int16_t PID_Pos(Param *param)
{
    int32_t pos_err =param->EncoderMultiTurnValue-param->EncoderExpect;

    if (abs(pos_err) <= 3)
    {
        /* 彻底锁死 */
        param->SpeedRef = 0;

        param->Pid_Pos.integral = 0;
        param->Pid_Pos.prev_error = 0;

        param->Pid_PosVel.integral = 0;
        param->Pid_PosVel.prev_error = 0;
        param->Pid_PosVel.prev_feedback = param->EncoderSpeed;

        return 0;
    }

    param->EncoderSpeedExpect = PID_AbsCalculate(&param->Pid_Pos, param->EncoderExpect, param->EncoderMultiTurnValue);
    if (abs(param->EncoderSpeedExpect)>param->SpeedMax)
    {
        if (param->EncoderSpeedExpect>0)
        {
            param->EncoderSpeedExpect=param->SpeedMax;
        }
        else
        {
            param->EncoderSpeedExpect=-(int32_t)param->SpeedMax;
        }
    }
    //param->target_speed=SpeedPlan_Update(param,param->target_speed);
    param->ExpectMA=PID_Vel_Calc(&param->Pid_PosVel, param->EncoderSpeedExpect, param->EncoderSpeed);
    return PID_AbsCalculate(&param->Pid_PosEle,param->ExpectMA,param->INA181_mA);
}

int32_t SpeedPlan_Update(Param *param,int32_t SpeedCmd)
{
    int32_t dv;
    int32_t dv_acc;
    int32_t dv_dec;

    /* 目标与当前规划速度差 */
    dv = SpeedCmd - param->SpeedRef;

    /* 每周期最大变化量 */
    dv_acc =  param->AccelMax * param->CycleTimeMs / 1000;
    dv_dec =  param->DecelMax * param->CycleTimeMs / 1000;

    if (dv > 0)
    {
        /* 需要加速 */
        if (dv > dv_acc)
            dv = dv_acc;
    }
    else
    {
        /* 需要减速 */
        if (dv < -dv_dec)
            dv = -dv_dec;
    }

    param->SpeedRef += dv;
    return param->SpeedRef;
}


/**
 * @brief  低速前馈查表 + 线性插值
 * @param  speed_abs  速度绝对值（>0）
 * @retval pwm 前馈 PWM
 */
int16_t FeedForward_LUT(int32_t speed_abs)
{
    // 小于最小速度，直接给最小起转 PWM
    if (speed_abs <= speed_lut[0])
        return pwm_lut[0];

    // 在表内做线性插值
    for (uint8_t i = 0; i < LUT_SIZE - 1; i++)
    {
        if (speed_abs <= speed_lut[i + 1])
        {
            int32_t ds = speed_lut[i + 1] - speed_lut[i];
            int32_t dp = pwm_lut[i + 1]   - pwm_lut[i];
            int32_t dx = speed_abs        - speed_lut[i];

            return pwm_lut[i] + (dp * dx) / ds;
        }
    }

    // 超过表范围，返回最大表值
    return pwm_lut[LUT_SIZE - 1];
}

/**
 * @brief  速度前馈计算（支持低速 / 中速 / 高速）
 * @param  speed  目标速度（可正可负）
 * @retval pwm    前馈 PWM（可正可负）
 */
int16_t Speed_FeedForward(int32_t speed)
{
    int32_t abs_spd = abs(speed);
    int16_t pwm;

    /* ========== 低速区：查表 ========= */
    if (abs_spd < SPEED_LOW)
    {
        pwm = FeedForward_LUT(abs_spd);
    }
    /* ========== 中速区：线性 + 死区补偿 ========= */
    else if (abs_spd < SPEED_MID)
    {
        pwm = (abs_spd * KV_Q) / FF_SCALE;
        pwm += PWM_DEAD;
    }
    /* ========== 高速区：纯线性 ========= */
    else
    {
        pwm = (abs_spd * KV_Q) / FF_SCALE;
    }

    /* PWM 限幅 */
    if (pwm > PWM_MAX) pwm = PWM_MAX;

    /* 恢复方向 */
    return (speed >= 0) ? pwm : -pwm;
}
