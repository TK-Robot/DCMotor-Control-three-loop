/**
 * @file VoltageStatus.c
 * @brief ADC voltage, current, temperature, and power-loss status implementation.
 * @brief ADC 电压、电流、温度和掉电状态检测实现。
 */

#include "VoltageStatus.h"

void VoltageStatus_init(VoltageStatus* VoltageStatus,ADC_HandleTypeDef* hadc1,Param* params)
{
    VoltageStatus->hadc=hadc1;
    VoltageStatus->param=params;

    HAL_ADCEx_Calibration_Start(VoltageStatus->hadc);
    HAL_ADC_Start_DMA(VoltageStatus->hadc,(uint32_t *)VoltageStatus->param->VoltageBuf,5);
    LPF_Filter_Init(&VoltageStatus->SampMaFilter,16);
}

uint16_t ADC_to_mV(uint16_t adc, uint16_t Vref)
{
    if (Vref == 0U)
    {
        return 0U;
    }
    return VrefInt * adc / Vref;
}

uint16_t VoltageStatus_VccAdcToPower_mV(uint16_t adc, uint16_t Vref)
{
    uint32_t divider_mV = ADC_to_mV(adc, Vref);

    /* Vin = Vadc * (Rhigh + Rlow) / Rlow. */
    /* 输入电压 = ADC 引脚电压 * (上分压电阻 + 下分压电阻) / 下分压电阻。 */
    return (uint16_t)(divider_mV * (VCC_DIVIDER_HIGH_KOHM + VCC_DIVIDER_LOW_KOHM) / VCC_DIVIDER_LOW_KOHM);
}

bool VoltageStatus_IsPowerLow(const VoltageStatus* VoltageStatus)
{
    if ((VoltageStatus == NULL) || (VoltageStatus->param == NULL))
    {
        return false;
    }

    /* A zero threshold disables automatic power-loss saving. */
    /* 阈值为 0 时关闭自动低压保存。 */
    if (VoltageStatus->param->PowerSaveVoltage_mV == 0U)
    {
        return false;
    }

    return VoltageStatus->param->VCC_mV < VoltageStatus->param->PowerSaveVoltage_mV;
}

static int8_t STM32_Temp_Calc(uint16_t adc, uint16_t Vref)
{
    int16_t temp;
    if (Vref == 0U)
    {
        return 0;
    }

    adc=adc>>2;
    Vref=Vref>>2;

    /* Correct temperature ADC reading with factory VREFINT calibration. */
    /* 使用出厂 VREFINT 校准值修正温度 ADC 采样。 */
    temp = (int32_t)adc * VREFINT_CAL / Vref;

    /* Linear interpolation between 30 C and 130 C calibration points. */
    /* 在 30 摄氏度和 130 摄氏度两个校准点之间做线性插值。 */
    temp = (temp - TS_CAL1) * (130 - 30)
           / (TS_CAL2 - TS_CAL1)
           + 30;

    return (int8_t)temp;
}

void VoltageStatus_AnalyzeData(VoltageStatus* VoltageStatus)
{
    VoltageStatus->param->INA181_mV=ADC_to_mV(VoltageStatus->param->VoltageBuf[0],VoltageStatus->param->VoltageBuf[4]);
    VoltageStatus->param->VCC_mV=VoltageStatus_VccAdcToPower_mV(VoltageStatus->param->VoltageBuf[1],VoltageStatus->param->VoltageBuf[4]);
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
