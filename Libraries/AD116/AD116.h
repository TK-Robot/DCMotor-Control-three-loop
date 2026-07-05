//
// Created by Administrator on 2025/12/8.
//

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_AD116_H
#define TRIPLE_CASCADECONTROLDCMOTOR_AD116_H

#include <stdbool.h>
#include "tim.h"
#include "TypeDefine.h"

#define  PowerMin 10


typedef struct
{
    // 属性
    TIM_HandleTypeDef* htim;//PWM TIM句柄
    Param *param;
    uint32_t channel1;         // PWM通道1
    uint32_t channel2;         // PWM通道2

    // 方法（函数指针）
    // void (*AD116_init)(void*,TIM_HandleTypeDef*,uint16_t, uint16_t);
    // void (*AD116_setTimerFrequency)(void*, uint32_t, uint32_t);
    // void (*AD116_Update)(void*, uint16_t, bool);
}AD116;

void AD116_init(AD116* ad116,TIM_HandleTypeDef* htim, uint32_t Channel1, uint32_t Channel2,Param *params);
void AD116_setTimerFrequency(const AD116* ad116, uint32_t psc, uint32_t arr);
void AD116_Update(AD116* ad116, Param* param);
void CycleStart(AD116* ad116,TIM_HandleTypeDef* htim);
void CycleBlockingTimer(AD116* ad116,TIM_HandleTypeDef* htim);
#endif //TRIPLE_CASCADECONTROLDCMOTOR_AD116_H