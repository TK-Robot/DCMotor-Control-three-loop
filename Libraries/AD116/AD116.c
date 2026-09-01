/**
 * @file AD116.c
 * @brief AD116 H-bridge motor driver implementation.
 * @brief AD116 H 桥电机驱动实现。
 */

#include "AD116.h"
#include "CurrentSenseModel.h"

#include <stdlib.h>

static void AD116_SetCompare(TIM_TypeDef *timer, uint32_t channel,
                             uint32_t compare)
{
    if (channel == LL_TIM_CHANNEL_CH2)
    {
        LL_TIM_OC_SetCompareCH2(timer, compare);
    }
    else if (channel == LL_TIM_CHANNEL_CH3)
    {
        LL_TIM_OC_SetCompareCH3(timer, compare);
    }
}

void AD116_init(AD116 *ad116, TIM_TypeDef *timer, const uint32_t channel1,
                const uint32_t channel2, Param *params)
{
    ad116->timer = timer;
    ad116->channel1 = channel1;
    ad116->channel2 = channel2;
    ad116->param=params;
    ad116->param->DriveRunMode=0;
    /* CH4 generates the PWM-center ADC trigger without driving a GPIO. */
    /* CH4 只在 PWM 中点产生 ADC 触发，不输出到 GPIO。 */
    LL_TIM_EnableARRPreload(timer);
    LL_TIM_OC_SetMode(timer, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM2);
    LL_TIM_OC_SetCompareCH4(timer,
        CurrentSenseModel_TriggerCompare((uint16_t)LL_TIM_GetAutoReload(timer),
                                         0U, 0U));
    LL_TIM_OC_EnablePreload(timer, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(timer, LL_TIM_TRGO_OC4REF);
    LL_TIM_CC_EnableChannel(timer, channel1 | channel2 | LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableCounter(timer);
}

void AD116_setTimerFrequency(const AD116* ad116, const uint32_t psc, const uint32_t arr)
{
    LL_TIM_DisableCounter(ad116->timer);
    LL_TIM_SetPrescaler(ad116->timer, psc);
    LL_TIM_SetAutoReload(ad116->timer, arr);
    LL_TIM_SetCounter(ad116->timer, 0U);
    LL_TIM_OC_SetCompareCH4(ad116->timer,
        CurrentSenseModel_TriggerCompare((uint16_t)arr,
                                         CurrentSenseModel_DutyTicks(
                                             (uint16_t)arr,
                                             ad116->param->DrivePower),
                                         ad116->param->DriveRunMode));
    LL_TIM_GenerateEvent_UPDATE(ad116->timer);
    LL_TIM_CC_EnableChannel(ad116->timer,
                           ad116->channel1 | ad116->channel2
                           | LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableCounter(ad116->timer);
}

void AD116_Update(AD116* ad116, Param *param)
{
    (void)param;
    AD116_ApplyPwm(ad116, ad116->param->DrivePower,
                   ad116->param->DriveRunMode);
}

void AD116_ApplyPwm(AD116 *ad116, int16_t power_permille,
                    uint8_t drive_mode)
{
    uint32_t period_ticks = LL_TIM_GetAutoReload(ad116->timer) + 1U;
    uint32_t duty_ticks;

    ad116->param->DrivePower = power_permille;
    ad116->param->DriveRunMode = drive_mode;

    /* Clamp command before mapping it to timer compare values. */
    /* 先限制输出指令，再映射到定时器比较值。 */
    if (ad116->param->DrivePower > 1000) ad116->param->DrivePower = 1000;
    if (ad116->param->DrivePower < -1000) ad116->param->DrivePower = -1000;
    if (drive_mode != 1U)
    {
        if (ad116->param->DrivePower>=0 && ad116->param->DrivePower < PowerMin)ad116->param->DrivePower = 0;
        if (ad116->param->DrivePower<=0 && ad116->param->DrivePower > -PowerMin)ad116->param->DrivePower = 0;
    }

    duty_ticks = CurrentSenseModel_DutyTicks(
        (uint16_t)LL_TIM_GetAutoReload(ad116->timer),
        ad116->param->DrivePower);
    LL_TIM_OC_SetCompareCH4(ad116->timer,
        CurrentSenseModel_TriggerCompare(
            (uint16_t)LL_TIM_GetAutoReload(ad116->timer),
            duty_ticks,
            ad116->param->DriveRunMode));
    if (ad116->param->DriveRunMode == 0)
    {
        /* Coast mode: both outputs disabled. */
        /* 滑行模式：两个输出关闭。 */
        AD116_SetCompare(ad116->timer, ad116->channel1, 0U);
        AD116_SetCompare(ad116->timer, ad116->channel2, 0U);
    }
    else if (ad116->param->DriveRunMode == 1)
    {
        /* Equal in-phase PWM alternates brake (high/high) and coast (low/low).
         * A zero command retains the fail-safe full-brake convention. */
        uint32_t brake_ticks = ad116->param->DrivePower == 0
                                 ? period_ticks : duty_ticks;
        AD116_SetCompare(ad116->timer, ad116->channel1, brake_ticks);
        AD116_SetCompare(ad116->timer, ad116->channel2, brake_ticks);
    }
    else if (ad116->param->DriveRunMode == 2)
    {
        /* Slow-decay mode: direction selects which side is PWM-modulated. */
        /* 慢衰减模式：方向决定哪一路使用 PWM 调制。 */
        uint32_t compare = period_ticks - duty_ticks;
        if (ad116->param->DrivePower>0) ad116->param->DriveVeerFlag =true;
        if (ad116->param->DrivePower<0) ad116->param->DriveVeerFlag =false;
        if (ad116->param->DriveVeerFlag == false)
        {
            AD116_SetCompare(ad116->timer, ad116->channel1, period_ticks);
            AD116_SetCompare(ad116->timer, ad116->channel2, compare);
        }
        else if (ad116->param->DriveVeerFlag == true)
        {
            AD116_SetCompare(ad116->timer, ad116->channel1, compare);
            AD116_SetCompare(ad116->timer, ad116->channel2, period_ticks);
        }
    }
    else if (ad116->param->DriveRunMode == 3)
    {
        /* Fast-decay mode: one output PWM, the other output off. */
        /* 快衰减模式：一路 PWM，另一路关闭。 */
        if (ad116->param->DrivePower>0) ad116->param->DriveVeerFlag =true;
        if (ad116->param->DrivePower<0) ad116->param->DriveVeerFlag =false;
        if (ad116->param->DriveVeerFlag == false)
        {
            AD116_SetCompare(ad116->timer, ad116->channel1, duty_ticks);
            AD116_SetCompare(ad116->timer, ad116->channel2, 0U);
        }
        else if (ad116->param->DriveVeerFlag == true)
        {
            AD116_SetCompare(ad116->timer, ad116->channel1, 0U);
            AD116_SetCompare(ad116->timer, ad116->channel2, duty_ticks);
        }
    }
}

void CycleStart(AD116 *ad116, TIM_TypeDef *timer)
{
    (void)ad116;
    LL_TIM_SetCounter(timer, 0U);
    LL_TIM_EnableCounter(timer);
}

void CycleBlockingTimer(AD116 *ad116, TIM_TypeDef *timer)
{
    uint32_t target = ad116->param->CycleTimeMs * 1000;
    while (target > LL_TIM_GetCounter(timer)) {}
    ad116->param->ProcessTimeUs = LL_TIM_GetCounter(timer);
    LL_TIM_DisableCounter(timer);
}
