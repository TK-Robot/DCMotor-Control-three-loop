//
// Created by Administrator on 2025/12/9.
//

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_INA181_H
#define TRIPLE_CASCADECONTROLDCMOTOR_INA181_H

#include <stdbool.h>
#include "adc.h"
#include "TypeDefine.h"
#include "Filter.h"

#define MaxVoltageBit 16383
#define VrefInt 1212
#define VREF 3300 //3.3mV
#define SamplingMR 90
#define SamplingGainV 20
#define Sampling 2 //采样

#define TS_CAL1   (*(uint16_t*)0x1FFF75A8)  // 30°C
#define TS_CAL2   (*(uint16_t*)0x1FFF75CA)  // 130°C
#define VREFINT_CAL (*(uint16_t*)0x1FFF75AA)

typedef struct
{
    // 属性
    ADC_HandleTypeDef* hadc;//TIM句柄
    Param* param;
    LPF_Filter SampMaFilter;

    // 方法（函数指针）
    // void (*INA181_init)(void*,ADC_HandleTypeDef*);
    // void (*INA181_Update)(void*);
    // uint16_t (*ADC_to_mV)(uint16_t, uint16_t);
}VoltageStatus;

void VoltageStatus_init(VoltageStatus* VoltageStatus,ADC_HandleTypeDef* hadc1,Param* params);
uint16_t ADC_to_mV(uint16_t Adc, uint16_t Vref);
void VoltageStatus_AnalyzeData(VoltageStatus* VoltageStatus);
void TempLimit(const VoltageStatus* VoltageStatus);
#endif //TRIPLE_CASCADECONTROLDCMOTOR_INA181_H