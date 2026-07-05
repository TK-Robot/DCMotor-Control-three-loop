//
// Created by Administrator on 2025/12/9.
//

#include "VoltageStatus.h"

void VoltageStatus_init(VoltageStatus* VoltageStatus,ADC_HandleTypeDef* hadc1,Param* params)
{
    VoltageStatus->hadc=hadc1;
    VoltageStatus->param=params;
    //初始化
    HAL_ADCEx_Calibration_Start(VoltageStatus->hadc);
    //校准
    HAL_ADC_Start_DMA(VoltageStatus->hadc,(uint32_t *)VoltageStatus->param->VoltageBuf,5);
    LPF_Filter_Init(&VoltageStatus->SampMaFilter,16);
}

uint16_t ADC_to_mV(uint16_t adc, uint16_t Vref)
{
    return VrefInt * adc / Vref;
}

int8_t STM32_Temp_Calc(uint16_t adc, uint16_t Vref)
{

    int16_t temp;
    adc=adc>>2;
    Vref=Vref>>2;
    /* 使用 Vrefint 修正电压 */
    temp = (int32_t)adc * VREFINT_CAL / Vref;

    /* 线性插值 */
    temp = (temp - TS_CAL1) * (130 - 30)
           / (TS_CAL2 - TS_CAL1)
           + 30;

    return (int8_t)temp;
}

void VoltageStatus_AnalyzeData(VoltageStatus* VoltageStatus)
{
    VoltageStatus->param->INA181_mV=ADC_to_mV(VoltageStatus->param->VoltageBuf[0],VoltageStatus->param->VoltageBuf[4]);
    VoltageStatus->param->VCC_mV=ADC_to_mV(VoltageStatus->param->VoltageBuf[1],VoltageStatus->param->VoltageBuf[4]);
    VoltageStatus->param->INA181REF_mV=ADC_to_mV(VoltageStatus->param->VoltageBuf[2],VoltageStatus->param->VoltageBuf[4]);
    VoltageStatus->param->INA181_mA=(VoltageStatus->param->INA181_mV-VoltageStatus->param->INA181REF_mV)/(Sampling);

    VoltageStatus->param->INA181_mA=LPF_Filter_Update(&VoltageStatus->SampMaFilter,VoltageStatus->param->INA181_mA);

    VoltageStatus->param->Temp=STM32_Temp_Calc(VoltageStatus->param->VoltageBuf[3],VoltageStatus->param->VoltageBuf[4])-10;
}

void TempLimit(const VoltageStatus* VoltageStatus)
{
    if (VoltageStatus->param->Temp > VoltageStatus->param->TempLimit)
    {
        VoltageStatus->param->DriveRunMode=0;
    }
}