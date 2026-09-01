/**
 * @file VoltageStatus.c
 * @brief ADC voltage, current, temperature, and power-loss status implementation.
 * @brief ADC 电压、电流、温度和掉电状态检测实现。
 */

#include "VoltageStatus.h"
#include "CurrentSenseModel.h"
#include "MotorTorqueModel.h"

#include <stddef.h>
#include <stdlib.h>

#define CURRENT_FILTER_SLOW_DECAY_ALPHA 64U
#define CURRENT_FILTER_FAST_DECAY_ALPHA 128U
#define ADC_STATUS_VREFINT_MIN_CODE 2000U
#define ADC_STATUS_VREFINT_MAX_CODE 4500U

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

static void VoltageStatus_PublishSnapshot(VoltageStatus *status)
{
    status->adc_snapshot_sequence++;
    __DMB();
    for (uint32_t i = 0U; i < ADC_STATUS_CONVERSION_COUNT; ++i)
    {
        status->adc_snapshot[i] = status->param->VoltageBuf[i];
    }
    __DMB();
    status->adc_snapshot_sequence++;
}

static void VoltageStatus_ReadSnapshot(const VoltageStatus *status,
                                       uint16_t samples[ADC_STATUS_CONVERSION_COUNT])
{
    uint32_t sequence_before;
    uint32_t sequence_after;

    /* The DMA interrupt publishes only complete four-rank scans. A sequence
     * lock prevents the 1 ms loop from observing a partially copied frame. */
    for (;;)
    {
        sequence_before = status->adc_snapshot_sequence;
        if ((sequence_before & 1U) != 0U) continue;
        __DMB();
        for (uint32_t i = 0U; i < ADC_STATUS_CONVERSION_COUNT; ++i)
        {
            samples[i] = status->adc_snapshot[i];
        }
        __DMB();
        sequence_after = status->adc_snapshot_sequence;
        if (sequence_before == sequence_after
            && (sequence_after & 1U) == 0U)
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
        sum += status->param->VoltageBuf[ADC_STATUS_CURRENT_INDEX];
    }
    LL_DMA_ClearFlag_TC1(DMA1);
    return (uint16_t)((sum + 8U) / 16U);
}

static void VoltageStatus_RestartSampling(VoltageStatus *status)
{
    ADC_TypeDef *adc = status->adc;

    /* Stop ADC requests before resetting the DMA write index. This restores
     * rank 1 -> buffer[0] without racing a conversion already in progress. */
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    if (LL_ADC_REG_IsConversionOngoing(adc) != 0U)
    {
        LL_ADC_REG_StopConversion(adc);
        while (LL_ADC_REG_IsConversionOngoing(adc) != 0U) {}
    }
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_ClearFlag_GI1(DMA1);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1,
                         ADC_STATUS_CONVERSION_COUNT);
    LL_ADC_ClearFlag_OVR(adc);
    NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_ADC_REG_StartConversion(adc);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    status->sample_valid = false;
    status->sample_filter_initialized = false;
    status->param->CurrentSampleValid = false;
}

void VoltageStatus_init(VoltageStatus *status, ADC_TypeDef *adc, Param *params)
{
    status->adc = adc;
    status->param = params;

    /* At 32 MHz ADC clock the 2x four-rank scan remains below the 39.06-us PWM
     * period with useful DMA margin. PA1 has a 100 nF hold capacitor, so it
     * shares the short sampling bank with low-impedance INA181 output while
     * VREFINT and the temperature sensor retain their required long sample. */
    LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM3_TRGO);
    LL_ADC_SetOverSamplingScope(adc, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
    LL_ADC_ConfigOverSamplingRatioShift(adc, LL_ADC_OVS_RATIO_2,
                                        LL_ADC_OVS_SHIFT_NONE);
    LL_ADC_SetOverSamplingDiscont(adc, LL_ADC_OVS_REG_CONT);
    LL_ADC_SetSamplingTimeCommonChannels(adc,
                                         LL_ADC_SAMPLINGTIME_COMMON_1,
                                         LL_ADC_SAMPLINGTIME_7CYCLES_5);
    LL_ADC_SetSamplingTimeCommonChannels(adc,
                                         LL_ADC_SAMPLINGTIME_COMMON_2,
                                         LL_ADC_SAMPLINGTIME_160CYCLES_5);
    LL_ADC_SetChannelSamplingTime(adc, LL_ADC_CHANNEL_0,
                                  LL_ADC_SAMPLINGTIME_COMMON_1);
    LL_ADC_SetChannelSamplingTime(adc, LL_ADC_CHANNEL_1,
                                  LL_ADC_SAMPLINGTIME_COMMON_1);
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
    status->adc_snapshot_sequence = 0U;
    for (uint32_t i = 0U; i < ADC_STATUS_CONVERSION_COUNT; ++i)
    {
        status->adc_snapshot[i] = status->param->VoltageBuf[i];
    }
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

static int16_t STM32_Temp_Calc(uint16_t adc, uint16_t Vref)
{
    int16_t temp;
    if (Vref == 0U)
    {
        return 0;
    }

    /* Correct temperature ADC reading with factory VREFINT calibration. */
    /* 使用出厂 VREFINT 校准值修正温度 ADC 采样。 */
    temp = (int32_t)adc * VREFINT_CAL / Vref;

    /* Linear interpolation between 30 C and 130 C calibration points. */
    /* 在 30 摄氏度和 130 摄氏度两个校准点之间做线性插值。 */
    temp = (temp - TS_CAL1) * (130 - 30)
           / (TS_CAL2 - TS_CAL1)
           + 30;

    return temp;
}

static bool VoltageStatus_IsFrameValid(
    const uint16_t samples[ADC_STATUS_CONVERSION_COUNT])
{
    uint16_t vref = samples[ADC_STATUS_VREFINT_INDEX];

    /* With 2x accumulation, VREFINT is around 3000 counts at VDDA=3.3 V.
     * A rank-shifted scan puts bus/current/temperature in this slot and must
     * not be allowed to become an apparent undervoltage sample. */
    if (vref < ADC_STATUS_VREFINT_MIN_CODE
        || vref > ADC_STATUS_VREFINT_MAX_CODE)
    {
        return false;
    }
    return samples[ADC_STATUS_TEMPERATURE_INDEX] >= 500U
        && samples[ADC_STATUS_TEMPERATURE_INDEX] <= 4500U;
}

void VoltageStatus_AnalyzeData(VoltageStatus* VoltageStatus)
{
    uint16_t samples[ADC_STATUS_CONVERSION_COUNT];

    VoltageStatus_ReadSnapshot(VoltageStatus, samples);
    if (!VoltageStatus_IsFrameValid(samples))
    {
        VoltageStatus_RestartSampling(VoltageStatus);
        return;
    }
    VoltageStatus->param->VCC_mV = VoltageStatus_VccAdcToPower_mV(
        samples[ADC_STATUS_BUS_INDEX], samples[ADC_STATUS_VREFINT_INDEX]);
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
    VoltageStatus->param->Temp = (int8_t)(STM32_Temp_Calc(
        samples[ADC_STATUS_TEMPERATURE_INDEX],
        samples[ADC_STATUS_VREFINT_INDEX]) - 10);

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
    VoltageStatus_PublishSnapshot(status);
    if (!VoltageStatus_IsFrameValid(
            (const uint16_t *)status->adc_snapshot))
    {
        param->CurrentSampleValid = false;
        return;
    }
    measured_mA = CurrentSenseModel_AdcToMilliamp(
        status->adc_snapshot[ADC_STATUS_CURRENT_INDEX],
        status->current_offset_adc,
        status->adc_snapshot[ADC_STATUS_VREFINT_INDEX]);
    sample_drive_mode = current_control->output_mode;
    sample_drive_sign = VoltageStatus_DriveSign(
        current_control->output_power);
    sample_valid = CurrentSenseModel_IsSampleValid(
        (uint16_t)LL_TIM_GetAutoReload(drive->timer),
        current_control->output_power, sample_drive_mode);
    sample_context_changed = sample_valid &&
        (!status->sample_valid ||
         status->sample_drive_mode != sample_drive_mode ||
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
    param->CurrentPeakLimitActive = output.hard_limit_active;
    param->CurrentPeakChopEvents = current_control->peak_chop_events;

    /* Diagnostics: raw ADC, qualified instantaneous sample, rolling window,
     * hard-limit trip edge, bridge context, and sample age reset. */
    /* 诊断：ADC 原始值、合格瞬时样本、滚动统计、硬限流边沿、桥状态与样本年龄复位。 */
    if (sample_valid)
    {
        /* Keep the raw code paired with the newest qualified current sample.
         * Invalid coast/stop scans must not overwrite the evidence after a
         * hard-limit trip. */
        param->CurrentAdcRaw = status->adc_snapshot[ADC_STATUS_CURRENT_INDEX];
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
        param->CurrentAverage_mA = 0;
        param->CurrentLogical_mA = 0;
        param->CurrentSampleValid = false;
        param->CurrentEstimated = false;
        return;
    }

    int32_t logical_current = 0;
    int16_t logical_duty;
    int16_t model_current;
    bool model_valid = false;
    bool regenerative_braking;

    param->CurrentEstimated = false;

    /* The ground shunt observes the active-window peak, not cycle-average
     * winding current. Slow-decay PWM has a defined average terminal voltage,
     * so estimate average current from the voltage actually applied after
     * saturation/chopping. Keep the measured peak exclusively in diagnostics
     * and protection. */
    logical_duty = param->EncoderVeer
        ? (int16_t)-param->DrivePower
        : param->DrivePower;
    regenerative_braking = (param->CurrentLoopStatus & 0x0080U) != 0U;
    if (regenerative_braking)
    {
        int32_t current_magnitude = param->ExpectMA;

        /* CurrentLoopStatus is published before the PWM ISR applies mode 1.
         * During that one-tick handover DriveRunMode may still say mode 2; do
         * not feed the stale bridge state into the R/Ke motoring equation. */
        logical_current = param->ExpectMA;
        if (current_magnitude < 0) current_magnitude = -current_magnitude;
        if (current_magnitude > INT16_MAX) current_magnitude = INT16_MAX;
        param->CurrentAverage_mA = (int16_t)current_magnitude;
        param->CurrentEstimated = true;
        model_valid = true;
    }
    else if (CurrentSenseModel_CanEstimateFromAverageDuty(
                 param->DriveRunMode, regenerative_braking))
    {
        model_current = MotorTorqueModel_DutyToCurrent(
            &param->MotorTorqueParams, logical_duty, param->VCC_mV,
            param->EncoderSpeed, param->TorqueEncoderCountsPerRev,
            param->MotorWindingTemperature_C, &model_valid);
        if (model_valid)
        {
            logical_current = model_current;
            param->CurrentAverage_mA = model_current == INT16_MIN
            ? INT16_MAX
                : (model_current < 0 ? (int16_t)-model_current : model_current);
            param->CurrentEstimated = true;
        }
    }
    if (!model_valid)
    {
        param->CurrentAverage_mA = 0;
        if (param->DriveRunMode == 1U)
        {
            int32_t current_magnitude = param->ExpectMA;

            /* Brake/coast has no qualified shunt sample: switching-edge INA181
             * peaks are not cycle-average winding current.  Publish the model
             * target explicitly as an estimate instead of inventing feedback. */
            logical_current = param->ExpectMA;
            if (current_magnitude < 0) current_magnitude = -current_magnitude;
            if (current_magnitude > INT16_MAX) current_magnitude = INT16_MAX;
            param->CurrentAverage_mA = (int16_t)current_magnitude;
            param->CurrentEstimated = true;
        }
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
