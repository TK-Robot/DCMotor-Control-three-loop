/**
 * @file PWMCapture.c
 * @brief PWM input capture implementation.
 * @brief PWM 输入捕获实现。
 */

#include "PWMCapture.h"

void PWMCapture_Init(CaptureData* Data,TIM_HandleTypeDef *htim)
{
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
    Data->htim = htim;
    Data->CaptureState = false;
    Data->EdgeNumber= 0;
}

uint16_t PWMCapture_Calculate(CaptureData* Data,TIM_HandleTypeDef *htim)
{
    if(Data->htim == htim)
    {
        GPIO_PinState state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
        if (Data->EdgeNumber ==0 && state == GPIO_PIN_SET && HAL_TIM_ReadCapturedValue(Data->htim , TIM_CHANNEL_1)<50000)
        {
            /* First rising edge marks the start of one PWM period. */
            /* 第一次上升沿标记一个 PWM 周期的起点。 */
            Data->CaptureOneUpTime=HAL_TIM_ReadCapturedValue(Data->htim , TIM_CHANNEL_1);
            Data->EdgeNumber= 1;
        }
        else if (Data->EdgeNumber ==1 && state == GPIO_PIN_RESET )
        {
            /* Falling edge marks the end of the high-level interval. */
            /* 下降沿标记高电平区间结束。 */
            Data->CaptureOneDownTime=HAL_TIM_ReadCapturedValue(Data->htim , TIM_CHANNEL_1);
            Data->EdgeNumber= 2;
        }
        else if (Data->EdgeNumber ==2 && state == GPIO_PIN_SET)
        {
            /* Second rising edge closes the period and updates the measured value. */
            /* 第二次上升沿闭合周期并更新测量值。 */
            Data->CaptureTwoUpTime=HAL_TIM_ReadCapturedValue(Data->htim , TIM_CHANNEL_1);
            uint16_t PulseWidth =Data->CaptureTwoUpTime-Data->CaptureOneUpTime;
            uint16_t UpWidth= Data->CaptureOneDownTime-Data->CaptureOneUpTime;
            Data->EdgeNumber= 0;
            Data->DutyRatio = PulseWidth-UpWidth;
        }
    }
    return Data->DutyRatio;
}
