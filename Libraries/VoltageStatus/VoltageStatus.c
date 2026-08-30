/**
 * @file VoltageStatus.c
 * @brief ADC voltage, current, temperature, and power-loss status implementation.
 * @brief ADC 电压、电流、温度和掉电状态检测实现。
 */

#include "VoltageStatus.h"
#include "CurrentSenseModel.h"

#include <stddef.h>
#include <stdlib.h>

#define CURRENT_FILTER_SLOW_DECAY_ALPHA 64U
#define CURRENT_FILTER_FAST_DECAY_ALPHA 128U

static int8_t VoltageStatus_DriveSign(int16_t power_permille)
{
    if (power_permille > 0) return 1;
    if (power_permille < 0) return -1;
    return 0;
}

static void VoltageStatus_CalibrateAdc(ADC_TypeDef *adc)
{
    uint32_t dma_transfer = LL_ADC_REG_GetDMATransfer(adc);

    /* STM32G0 publishes the calibration factor through ADC_DR. Keep ADC DMA
     * requests disabled so that value cannot shift the four-rank DMA buffer. */
    LL_ADC_REG_SetDMATransfer(adc, LL_ADC_REG_DMA_TRANSFER_NONE);
    LL_ADC_StartCalibration(adc);
    while (LL_ADC_IsCalibrationOnGoing(adc) != 0U) {}

    /* At least two ADC clock cycles are required before enabling the ADC. */
    for (volatile uint32_t wait = 0U; wait < 16U; ++wait)
    {
        __NOP();
    }

    LL_ADC_ClearFlag_EOC(adc);
    LL_ADC_ClearFlag_EOS(adc);
    LL_ADC_ClearFlag_OVR(adc);
    LL_ADC_REG_SetDMATransfer(adc, dma_transfer);
}

static void VoltageStatus_ReadSnapshot(const Param *params,
                                       uint16_t samples[ADC_STATUS_CONVERSION_COUNT])
{
    uint32_t remaining_before;
    uint32_t remaining_after;

    /* A scan takes much longer than this four-value copy. Retry if DMA changed
     * CNDTR during the copy so all values belong to one coherent scan. */
    for (uint32_t attempt = 0U; attempt < 3U; ++attempt)
    {
        remaining_before = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);
        for (uint32_t i = 0U; i < ADC_STATUS_CONVERSION_COUNT; ++i)
        {
            samples[i] = params->VoltageBuf[i];
        }
        remaining_after = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1);
        if (remaining_before == remaining_after)
        {
            break;
        }
    }
}

static uint16_t VoltageStatus_CalibrateCurrentOffset(VoltageStatus *status)
{
    uint32_t sum = 0U;

    for (uint32_t sample = 0U; sample < 16U; ++sample)
    {
        LL_DMA_ClearFlag_TC1(DMA1);
        while (LL_DMA_IsActiveFlag_TC1(DMA1) == 0U) {}
        sum += status->param->VoltageBuf[0];
    }
    LL_DMA_ClearFlag_TC1(DMA1);
    return (uint16_t)((sum + 8U) / 16U);
}

void VoltageStatus_init(VoltageStatus *status, ADC_TypeDef *adc, Param *params)
{
    status->adc = adc;
    status->param = params;

    /* PA0 uses the original 16-conversion hardware average. Its roughly
     * 26-us aperture averages the narrow PWM pulse and INA181 response. */
    /* 四通道 16 倍过采样超过一个 PWM 周期；每次触发仅做一次短扫描，
     * CH4 由 AD116 动态移动到有效驱动区。 */
    LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM3_TRGO);
    LL_ADC_SetOverSamplingScope(adc, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
    LL_ADC_ConfigOverSamplingRatioShift(adc, LL_ADC_OVS_RATIO_16,
                                        LL_ADC_OVS_SHIFT_RIGHT_2);
    LL_ADC_SetOverSamplingDiscont(adc, LL_ADC_OVS_REG_CONT);
    LL_ADC_SetSamplingTimeCommonChannels(adc,
                                         LL_ADC_SAMPLINGTIME_COMMON_1,
                                         LL_ADC_SAMPLINGTIME_39CYCLES_5);
    LL_ADC_SetSamplingTimeCommonChannels(adc,
                                         LL_ADC_SAMPLINGTIME_COMMON_2,
                                         LL_ADC_SAMPLINGTIME_79CYCLES_5);
    LL_ADC_SetChannelSamplingTime(adc, LL_ADC_CHANNEL_0,
                                  LL_ADC_SAMPLINGTIME_COMMON_1);
    LL_ADC_SetChannelSamplingTime(adc, LL_ADC_CHANNEL_1,
                                  LL_ADC_SAMPLINGTIME_COMMON_2);
    LL_ADC_SetChannelSamplingTime(adc, LL_ADC_CHANNEL_TEMPSENSOR,
                                  LL_ADC_SAMPLINGTIME_COMMON_2);
    LL_ADC_SetChannelSamplingTime(adc, LL_ADC_CHANNEL_VREFINT,
                                  LL_ADC_SAMPLINGTIME_COMMON_2);

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_ClearFlag_GI1(DMA1);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1,
                            LL_ADC_DMA_GetRegAddr(adc,
                                                 LL_ADC_DMA_REG_REGULAR_DATA));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1,
                            (uint32_t)status->param->VoltageBuf);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1,
                         ADC_STATUS_CONVERSION_COUNT);

    VoltageStatus_CalibrateAdc(adc);

    /* MX_ADC1_Init enabled VREFINT and the temperature sensor. This startup-only
     * delay covers their specified stabilization time before the first scan. */
    LL_mDelay(1U);
    LL_ADC_ClearFlag_ADRDY(adc);
    LL_ADC_Enable(adc);
    while (LL_ADC_IsActiveFlag_ADRDY(adc) == 0U)
    {
        /* STM32G0 may reject ADEN if it is asserted too soon after ADCAL. */
        if (LL_ADC_IsEnabled(adc) == 0U)
        {
            LL_ADC_Enable(adc);
        }
    }

    /* Temperature-sensor buffer stabilization starts when ADC is enabled. */
    LL_mDelay(1U);
    LL_ADC_ClearFlag_EOC(adc);
    LL_ADC_ClearFlag_EOS(adc);
    LL_ADC_ClearFlag_OVR(adc);
    LL_DMA_ClearFlag_GI1(DMA1);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_ADC_REG_StartConversion(adc);

    /* Do not let the first control iteration consume a partial DMA sequence. */
    while (LL_DMA_IsActiveFlag_TC1(DMA1) == 0U) {}
    LL_DMA_ClearFlag_TC1(DMA1);
    status->current_offset_adc = VoltageStatus_CalibrateCurrentOffset(status);
    status->window_sum_mA = 0;
    status->window_valid = 0U;
    status->window_invalid = 0U;
    status->window_min_mA = 0;
    status->window_max_mA = 0;
    status->window_tick_ms = 0U;
    status->last_hard_limit = false;
    status->param->CurrentAdcOffset = status->current_offset_adc;
    status->sample_drive_mode = 0U;
    status->sample_drive_sign = 0;
    status->sample_valid = false;
    status->sample_filter_initialized = false;
    status->param->CurrentSampleValid = false;
    status->param->CurrentEstimated = false;
    status->param->CurrentHardLimitActive = false;
    LPF_Filter_Init(&status->SampMaFilter,
                    CURRENT_FILTER_SLOW_DECAY_ALPHA);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
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
    uint16_t samples[ADC_STATUS_CONVERSION_COUNT];

    VoltageStatus_ReadSnapshot(VoltageStatus->param, samples);
    VoltageStatus->param->VCC_mV=VoltageStatus_VccAdcToPower_mV(samples[1],samples[3]);
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
    VoltageStatus->param->Temp=STM32_Temp_Calc(samples[2],samples[3])-10;

    /*
     * Sample age and 20 ms statistics window folding. The main loop calls this
     * once per millisecond; the window gives the host a min/max/average view
     * even though individual PWM-cycle samples are far faster than the link.
     *
     * 样本年龄维护与 20 ms 统计窗口折叠。本函数由 1 ms 主循环调用；
     * 窗口统计让上位机在链路远慢于 PWM 周期的情况下仍能看到包络。
     */
    if (VoltageStatus->param->CurrentSampleAgeMs < 60000U)
    {
        VoltageStatus->param->CurrentSampleAgeMs++;
    }
    if (++VoltageStatus->window_tick_ms >= 20U)
    {
        Param *p = VoltageStatus->param;
        VoltageStatus->window_tick_ms = 0U;
        p->CurrentWindowValid = VoltageStatus->window_valid;
        p->CurrentWindowInvalid = VoltageStatus->window_invalid;
        p->CurrentWindowMin_mA = VoltageStatus->window_min_mA;
        p->CurrentWindowMax_mA = VoltageStatus->window_max_mA;
        p->CurrentWindowAvg_mA = VoltageStatus->window_valid > 0U
            ? (int16_t)(VoltageStatus->window_sum_mA
                        / VoltageStatus->window_valid)
            : 0;
        p->CurrentValidTotal += VoltageStatus->window_valid;
        p->CurrentInvalidTotal += VoltageStatus->window_invalid;
        VoltageStatus->window_sum_mA = 0;
        VoltageStatus->window_valid = 0U;
        VoltageStatus->window_invalid = 0U;
        VoltageStatus->window_min_mA = 0;
        VoltageStatus->window_max_mA = 0;
    }
}

void VoltageStatus_DmaIrqHandler(VoltageStatus *status,
                                 CurrentControl *current_control,
                                 AD116 *drive)
{
    Param *param;
    int16_t measured_mA;
    int16_t filtered_mA;
    int8_t sample_drive_sign;
    uint8_t sample_drive_mode;
    bool sample_valid;
    bool sample_context_changed;
    bool control_sample_valid = false;
    CurrentControlOutput output;

    if (status == NULL || current_control == NULL || drive == NULL) return;
    param = status->param;
    measured_mA = CurrentSenseModel_AdcToMilliamp(
        param->VoltageBuf[0], status->current_offset_adc,
        param->VoltageBuf[3]);
    sample_drive_mode = current_control->output_mode;
    sample_drive_sign = VoltageStatus_DriveSign(
        current_control->output_power);
    sample_valid = CurrentSenseModel_IsSampleValid(
        current_control->output_power, sample_drive_mode);
    sample_context_changed = sample_valid &&
        (status->sample_drive_mode != sample_drive_mode ||
         status->sample_drive_sign != sample_drive_sign);

    /* Prime a new bridge context with one sample, but never let that first
     * switching-edge observation enter either PI feedback or protection.
     * Stable samples are decay-specific low-pass filtered before control. */
    filtered_mA = param->INA181_mA;
    if (sample_valid)
    {
        status->SampMaFilter.alpha =
            sample_drive_mode == 3U
                ? CURRENT_FILTER_FAST_DECAY_ALPHA
                : CURRENT_FILTER_SLOW_DECAY_ALPHA;
        if (!status->sample_filter_initialized || sample_context_changed)
        {
            status->SampMaFilter.prev_output = measured_mA;
            filtered_mA = measured_mA;
        }
        else
        {
            filtered_mA = (int16_t)LPF_Filter_Update(&status->SampMaFilter,
                                                     measured_mA);
            control_sample_valid = true;
        }
        status->sample_valid = true;
        status->sample_filter_initialized = true;
        status->sample_drive_mode = sample_drive_mode;
        status->sample_drive_sign = sample_drive_sign;
        param->INA181_mA = filtered_mA;
        param->INA181_mV = (uint16_t)((uint32_t)filtered_mA
                                     * CURRENT_SENSE_SHUNT_MILLIOHM
                                     * CURRENT_SENSE_GAIN / 1000U);
    }
    else
    {
        status->sample_valid = false;
        if (!current_control->enabled)
            status->sample_filter_initialized = false;
    }

    output = CurrentControl_Step(current_control, filtered_mA,
                                 control_sample_valid);
    param->CurrentHardLimitActive = output.hard_limit_active;
    param->CurrentPeakLimitActive = output.peak_limit_active;
    param->CurrentAverage_mA = output.average_current_mA;
    param->CurrentPeakChopEvents = current_control->peak_chop_events;

    /* Diagnostics: raw ADC, qualified instantaneous sample, rolling window,
     * hard-limit trip edge, bridge context, and sample age reset. */
    /* 诊断：ADC 原始值、合格瞬时样本、滚动统计、硬限流边沿、桥状态与样本年龄复位。 */
    if (sample_valid)
    {
        /* Keep the raw code paired with the newest qualified current sample.
         * Invalid coast/stop scans must not overwrite the evidence after a
         * hard-limit trip. */
        param->CurrentAdcRaw = param->VoltageBuf[0];
        param->CurrentInstant_mA = measured_mA;
        param->CurrentSampleAgeMs = 0U;
        if (status->window_valid == 0U)
        {
            status->window_min_mA = measured_mA;
            status->window_max_mA = measured_mA;
        }
        else
        {
            if (measured_mA < status->window_min_mA) status->window_min_mA = measured_mA;
            if (measured_mA > status->window_max_mA) status->window_max_mA = measured_mA;
        }
        status->window_sum_mA += measured_mA;
        status->window_valid++;
    }
    else
    {
        status->window_invalid++;
    }
    if (output.hard_limit_active && !status->last_hard_limit)
    {
        param->CurrentHardLimitTrips++;
    }
    status->last_hard_limit = output.hard_limit_active;
    {
        uint16_t last_sample_mode = param->CurrentBridgeStatus >> 8;
        if (sample_valid)
        {
            last_sample_mode = sample_drive_mode;
        }
        param->CurrentBridgeStatus = (uint16_t)((last_sample_mode << 8)
                                                | (param->DriveRunMode & 0xFFu));
    }

    /* Step may switch mixed decay after consuming this sample. Do not publish
     * a sample captured under the previous bridge state as current data. */
    if (output.drive_mode != sample_drive_mode ||
        VoltageStatus_DriveSign(output.power_permille) != sample_drive_sign)
    {
        status->sample_valid = false;
    }

    param->CurrentSampleValid = control_sample_valid && status->sample_valid;
    if (current_control->enabled)
        AD116_ApplyPwm(drive, output.power_permille, output.drive_mode);
}

void VoltageStatus_UpdateLogicalCurrent(VoltageStatus* VoltageStatus)
{
    Param *param = VoltageStatus->param;

    if (!param->OutputEnabled && param->DriveRunMode == 0U)
    {
        param->CurrentLogical_mA = 0;
        param->CurrentSampleValid = false;
        param->CurrentEstimated = false;
        return;
    }

    int32_t logical_current = 0;

    param->CurrentEstimated = false;

    /*
     * The shunt reports magnitude only. Active PWM uses the final physical
     * command plus the complete-axis direction. Brake current has no PWM
     * sign, so its logical torque direction is opposite the corrected speed.
     * Coast/stop reports zero logical current.
     *
     * 分流采样只有幅值。主动 PWM 驱动时，根据最终物理指令和编码器方向恢复逻辑符号；
     * 滑行或刹车状态没有明确的电流方向，因此返回 0，避免伪造双向实测值。
     */
    if (param->CurrentSampleAgeMs <= 3U &&
        (param->CurrentAverage_mA > 0 || param->CurrentSampleValid))
    {
        logical_current = param->CurrentAverage_mA;
        if (param->ExpectMA < 0) logical_current = -logical_current;
        /* The magnitude observer is corrected by active-window measurements;
         * only its sign is reconstructed from the requested bridge direction. */
        param->CurrentEstimated = true;
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
