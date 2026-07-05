//
// Created by Administrator on 2026/1/11.
//

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_PWMCAPTURE_H
#define TRIPLE_CASCADECONTROLDCMOTOR_PWMCAPTURE_H

#include "TypeDefine.h"
#include "stdint.h"
#include "tim.h"

typedef struct
{
    // 属性
    uint16_t CaptureOneUpTime;
    uint16_t CaptureOneDownTime;
    uint16_t CaptureTwoUpTime;
    uint16_t DutyRatio;
    uint8_t EdgeNumber;
    TIM_HandleTypeDef *htim;
    bool CaptureState;
} CaptureData;

void PWMCapture_Init(CaptureData* Data,TIM_HandleTypeDef *htim);
uint16_t PWMCapture_Calculate(CaptureData* Data,TIM_HandleTypeDef *htim);
void SetCaptureToRising(CaptureData* Data);
void SetCaptureToFalling(CaptureData* Data);
#endif //TRIPLE_CASCADECONTROLDCMOTOR_PWMCAPTURE_H