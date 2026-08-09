/**
 * @file PWMCapture.c
 * @brief PWM input capture implementation.
 * @brief PWM 输入捕获实现。
 */

#include "PWMCapture.h"

#include <stddef.h>

void PWMCapture_Init(CaptureData *Data, TIM_TypeDef *timer)
{
    Data->timer = timer;
    Data->DutyRatio = 0U;
    Data->SignalAgeMs = UINT16_MAX;
    Data->SignalValid = false;
    Data->EdgeNumber = 0;
    LL_TIM_CC_EnableChannel(timer, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableIT_CC1(timer);
    LL_TIM_EnableCounter(timer);
}

uint16_t PWMCapture_Calculate(CaptureData *Data, TIM_TypeDef *timer)
{
    if(Data->timer == timer)
    {
        bool pin_high = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6) != 0U;
        uint16_t capture = (uint16_t)LL_TIM_IC_GetCaptureCH1(timer);
        if (Data->EdgeNumber == 0U && pin_high && capture < 50000U)
        {
            /* First rising edge marks the start of one PWM period. */
            /* 第一次上升沿标记一个 PWM 周期的起点。 */
            Data->CaptureOneUpTime = capture;
            Data->EdgeNumber= 1;
        }
        else if (Data->EdgeNumber == 1U && !pin_high)
        {
            /* Falling edge marks the end of the high-level interval. */
            /* 下降沿标记高电平区间结束。 */
            Data->CaptureOneDownTime = capture;
            Data->EdgeNumber= 2;
        }
        else if (Data->EdgeNumber == 2U && pin_high)
        {
            /* Second rising edge closes the period and updates the measured value. */
            /* 第二次上升沿闭合周期并更新测量值。 */
            Data->CaptureTwoUpTime = capture;
            uint16_t PulseWidth =Data->CaptureTwoUpTime-Data->CaptureOneUpTime;
            uint16_t UpWidth= Data->CaptureOneDownTime-Data->CaptureOneUpTime;
            Data->EdgeNumber= 0;
            Data->DutyRatio = PulseWidth-UpWidth;
            Data->SignalAgeMs = 0U;
            Data->SignalValid = Data->DutyRatio >= PWM_INPUT_VALID_MIN_US
                                && Data->DutyRatio <= PWM_INPUT_VALID_MAX_US;
        }
    }
    return Data->DutyRatio;
}

void PWMCapture_1msTick(CaptureData *Data)
{
    if (Data == NULL)
    {
        return;
    }
    if (Data->SignalAgeMs < UINT16_MAX)
    {
        ++Data->SignalAgeMs;
    }
    if (Data->SignalAgeMs > PWM_INPUT_TIMEOUT_MS)
    {
        Data->SignalValid = false;
    }
}
