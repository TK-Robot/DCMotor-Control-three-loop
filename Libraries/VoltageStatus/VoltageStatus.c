/**
 * @file VoltageStatus.c
 * @brief ADC voltage, current, temperature, and power-loss status implementation.
 * @brief ADC 电压、电流、温度和掉电状态检测实现。
 */

#include "VoltageStatus.h"

#include <stddef.h>

void VoltageStatus_init(VoltageStatus *status, ADC_TypeDef *adc, Param *params)
{
    status->adc = adc;
    status->param = params;

    /* Keep current sampling synchronized to the TIM3 PWM-center trigger. */
    /* 保持电流采样与 TIM3 PWM 中点触发同步。 */
    LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM3_TRGO);

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_ClearFlag_GI1(DMA1);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1,
                            LL_ADC_DMA_GetRegAddr(adc,
                                                 LL_ADC_DMA_REG_REGULAR_DATA));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1,
                            (uint32_t)status->param->VoltageBuf);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1,
                         ADC_STATUS_CONVERSION_COUNT);

    LL_ADC_StartCalibration(adc);
    while (LL_ADC_IsCalibrationOnGoing(adc) != 0U) {}
    for (volatile uint32_t wait = 0U; wait < 128U; ++wait) {}

    LL_ADC_ClearFlag_ADRDY(adc);
    LL_ADC_Enable(adc);
    while (LL_ADC_IsActiveFlag_ADRDY(adc) == 0U) {}
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_ADC_REG_StartConversion(adc);
    LPF_Filter_Init(&status->SampMaFilter,16);
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
    int32_t current_delta_mV;
    int32_t current_mA;

    VoltageStatus->param->INA181_mV=ADC_to_mV(VoltageStatus->param->VoltageBuf[0],VoltageStatus->param->VoltageBuf[3]);
    VoltageStatus->param->VCC_mV=VoltageStatus_VccAdcToPower_mV(VoltageStatus->param->VoltageBuf[1],VoltageStatus->param->VoltageBuf[3]);
    /* INA181 REF is tied to GND; PA4 is NC and is not part of the ADC scan. */
    /* INA181 REF 接地；PA4 为 NC，不参与 ADC 扫描和电流偏置计算。 */
    VoltageStatus->param->INA181REF_mV=0U;

    /*
     * The low-side shunt and INA181 reference used on this board measure
     * current magnitude. Keep the subtraction signed so a small zero-current
     * offset cannot wrap an unsigned value into a large current.
     *
     * 本板低侧分流电阻和 INA181 参考连接测量的是电流幅值。差值必须先按有符号数
     * 计算，避免零电流偏置导致无符号下溢并变成很大的电流。
     */
    current_delta_mV = (int32_t)VoltageStatus->param->INA181_mV;
    current_mA = (current_delta_mV * 1000) /
                 ((int32_t)SamplingMR * (int32_t)SamplingGainV);
    if (current_mA < 0)
    {
        current_mA = 0;
    }
    if (current_mA > INT16_MAX)
    {
        current_mA = INT16_MAX;
    }
    VoltageStatus->param->INA181_mA=(int16_t)current_mA;

    VoltageStatus->param->INA181_mA=LPF_Filter_Update(&VoltageStatus->SampMaFilter,VoltageStatus->param->INA181_mA);

    VoltageStatus->param->Temp=STM32_Temp_Calc(VoltageStatus->param->VoltageBuf[2],VoltageStatus->param->VoltageBuf[3])-10;

    /* The generated ADC uses one four-rank scan; restart it for the next 1 ms sample. */
    /* 当前 ADC 配置为四通道单次扫描；每个 1 ms 周期重新启动下一次采样。 */
    if (LL_ADC_REG_IsConversionOngoing(VoltageStatus->adc) == 0U)
    {
        LL_ADC_REG_StartConversion(VoltageStatus->adc);
    }
}

void VoltageStatus_UpdateLogicalCurrent(VoltageStatus* VoltageStatus)
{
    Param *param = VoltageStatus->param;
    int32_t logical_current = 0;

    /*
     * The shunt reports magnitude only. Active PWM uses the final physical
     * command plus the complete-axis direction. Brake current has no PWM
     * sign, so its logical torque direction is opposite the corrected speed.
     * Coast/stop reports zero logical current.
     *
     * 分流采样只有幅值。主动 PWM 驱动时，根据最终物理指令和编码器方向恢复逻辑符号；
     * 滑行或刹车状态没有明确的电流方向，因此返回 0，避免伪造双向实测值。
     */
    if ((param->DriveRunMode == 2U || param->DriveRunMode == 3U) &&
        (param->DrivePower != 0))
    {
        logical_current = param->INA181_mA;
        if (param->DrivePower < 0)
        {
            logical_current = -logical_current;
        }
        if (param->EncoderVeer)
        {
            logical_current = -logical_current;
        }
    }
    else if ((param->DriveRunMode == 1U) && (param->EncoderSpeed != 0))
    {
        /* Dynamic braking torque opposes the corrected logical speed. */
        /* 动态制动转矩与校正后的逻辑速度方向相反。 */
        logical_current = (param->EncoderSpeed > 0) ?
                          -param->INA181_mA : param->INA181_mA;
    }

    if (logical_current > INT16_MAX)
    {
        logical_current = INT16_MAX;
    }
    else if (logical_current < INT16_MIN)
    {
        logical_current = INT16_MIN;
    }
    param->CurrentLogical_mA = (int16_t)logical_current;
}

void TempLimit(const VoltageStatus* VoltageStatus)
{
    if (VoltageStatus->param->Temp > VoltageStatus->param->TempLimit)
    {
        VoltageStatus->param->DriveRunMode=0;
    }
}
