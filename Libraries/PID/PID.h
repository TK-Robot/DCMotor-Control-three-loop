//
// Created by Administrator on 2025/12/15.
//

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_PID_H
#define TRIPLE_CASCADECONTROLDCMOTOR_PID_H

#include "MT6701.h"
#include "TypeDefine.h"
#include "stdint.h"

#define PID_SCALE   1000
#define PID_AbsDEADBAND 2  // 误差
#define PID_IncDEADBAND 24   // 误差

/************** 前馈模型参数 **************/
#define SPEED_LOW      8000     // 低速区上限（经验值）
#define SPEED_MID      15000    // 中速区上限

#define FF_SCALE       1000     // 定点缩放因子
#define KV_Q           24       // Kv ≈ 0.024 → 24/1000

#define PWM_DEAD       110       // 电机起转死区补偿（根据实测微调）
#define PWM_MAX        750

/************** 低速前馈查表（顺时针） **************/
static const int16_t speed_lut[] = {
    1250, 6000, 7500, 9000
};

static const int16_t pwm_lut[] = {
    110,  142,  174,  206
};

#define LUT_SIZE   (sizeof(speed_lut)/sizeof(speed_lut[0]))


void PID_Init(Param *param);
int32_t PID_AbsCalculate(PID_Int* pid,int32_t setValue,int32_t CurrentValue);
int16_t PID_Vel_Calc(PID_Int* pid, int32_t target, int32_t feedback);
int16_t PID_Pos(Param *param);
int16_t FeedForward_LUT(int32_t speed_abs);
int16_t Speed_FeedForward(int32_t speed);
int32_t SpeedPlan_Update(Param *param,int32_t SpeedCmd);
#endif //TRIPLE_CASCADECONTROLDCMOTOR_PID_H