//
// Created by Administrator on 2026/1/11.
//

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

            Data->CaptureOneUpTime=HAL_TIM_ReadCapturedValue(Data->htim , TIM_CHANNEL_1);
            Data->EdgeNumber= 1;
        }
        else if (Data->EdgeNumber ==1 && state == GPIO_PIN_RESET )
        {
            Data->CaptureOneDownTime=HAL_TIM_ReadCapturedValue(Data->htim , TIM_CHANNEL_1);
            Data->EdgeNumber= 2;
        }
        else if (Data->EdgeNumber ==2 && state == GPIO_PIN_SET)
        {
            Data->CaptureTwoUpTime=HAL_TIM_ReadCapturedValue(Data->htim , TIM_CHANNEL_1);
            uint16_t PulseWidth =Data->CaptureTwoUpTime-Data->CaptureOneUpTime;
            uint16_t UpWidth= Data->CaptureOneDownTime-Data->CaptureOneUpTime;
            Data->EdgeNumber= 0;
            Data->DutyRatio = PulseWidth-UpWidth;

        }
    }
    return Data->DutyRatio;
}
