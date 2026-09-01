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
        if (pin_high)
        {
            /* A rising edge always starts a new active-high command pulse. */
            /* 每个上升沿都重新开始一次高电平有效的命令脉冲。 */
            Data->CaptureOneUpTime = capture;
            Data->EdgeNumber= 1;
        }
        else if (Data->EdgeNumber == 1U && !pin_high)
        {
            uint16_t high_width;

            /* Unsigned subtraction also handles one 16-bit timer wrap. */
            /* 无符号减法同时覆盖一次 16 位定时器回绕。 */
            Data->CaptureOneDownTime = capture;
            high_width = (uint16_t)(Data->CaptureOneDownTime
                                    - Data->CaptureOneUpTime);
            Data->EdgeNumber= 0;
            Data->DutyRatio = high_width;
            Data->SignalAgeMs = 0U;
            if (Data->DutyRatio >= PWM_INPUT_VALID_MIN_US
                && Data->DutyRatio <= PWM_INPUT_VALID_MAX_US)
            {
                Data->SignalValid = true;
            }
            else
            {
                Data->SignalValid = false;
            }
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
        Data->SignalAgeMs = UINT16_MAX;
        Data->SignalValid = false;
    }
}
