/**
 * @file AD116.c
 * @brief AD116 H-bridge motor driver implementation.
 * @brief AD116 H 桥电机驱动实现。
 */

#include "AD116.h"

#include <stdlib.h>

void AD116_init(AD116* ad116,TIM_HandleTypeDef* htim, const uint32_t Channel1, const uint32_t Channel2,Param *params)
{
    ad116->htim = htim;
    ad116->channel1 = Channel1;
    ad116->channel2 = Channel2;
    ad116->param=params;
    ad116->param->DriveRunMode=0;
    HAL_TIM_PWM_Start(ad116->htim,ad116->channel1);
    HAL_TIM_PWM_Start(ad116->htim,ad116->channel2);
}

void AD116_setTimerFrequency(const AD116* ad116, const uint32_t psc, const uint32_t arr)
{
    __HAL_TIM_DISABLE(ad116->htim);

    ad116->htim->Instance->PSC = psc;
    ad116->htim->Instance->ARR = arr;

    __HAL_TIM_SET_COUNTER(ad116->htim, 0);
    __HAL_TIM_ENABLE(ad116->htim);

    /* Restart PWM after timer register updates. */
    /* 更新定时器寄存器后重新启动 PWM。 */
    HAL_TIM_PWM_Start(ad116->htim, ad116->channel1);
    HAL_TIM_PWM_Start(ad116->htim, ad116->channel2);
}

void AD116_Update(AD116* ad116, Param *param)
{
    (void)param;

    /* Clamp command before mapping it to timer compare values. */
    /* 先限制输出指令，再映射到定时器比较值。 */
    if (ad116->param->DrivePower > 1000) ad116->param->DrivePower = 1000;
    if (ad116->param->DrivePower < -1000) ad116->param->DrivePower = -1000;
    if (ad116->param->DrivePower>=0 && ad116->param->DrivePower < PowerMin)ad116->param->DrivePower = 0;
    if (ad116->param->DrivePower<=0 && ad116->param->DrivePower > -PowerMin)ad116->param->DrivePower = 0;

    if (ad116->param->DriveRunMode == 0)
    {
        /* Coast mode: both outputs disabled. */
        /* 滑行模式：两个输出关闭。 */
        __HAL_TIM_SetCompare(ad116->htim, ad116->channel1, 0);
        __HAL_TIM_SetCompare(ad116->htim, ad116->channel2, 0);
    }
    else if (ad116->param->DriveRunMode == 1)
    {
        /* Brake mode: both outputs driven high. */
        /* 刹车模式：两个输出拉高。 */
        __HAL_TIM_SetCompare(ad116->htim, ad116->channel1, ad116->htim->Instance->ARR);
        __HAL_TIM_SetCompare(ad116->htim, ad116->channel2, ad116->htim->Instance->ARR);
    }
    else if (ad116->param->DriveRunMode == 2)
    {
        /* Slow-decay mode: direction selects which side is PWM-modulated. */
        /* 慢衰减模式：方向决定哪一路使用 PWM 调制。 */
        uint16_t DutyRatio = MAP(abs(ad116->param->DrivePower), 0, 1000, 0, ad116->htim->Instance->ARR);
        if (ad116->param->DrivePower>0) ad116->param->DriveVeerFlag =true;
        if (ad116->param->DrivePower<0) ad116->param->DriveVeerFlag =false;
        DutyRatio = ad116->htim->Instance->ARR - DutyRatio;
        if (ad116->param->DriveVeerFlag == false)
        {
            __HAL_TIM_SetCompare(ad116->htim, ad116->channel1, ad116->htim->Instance->ARR);
            __HAL_TIM_SetCompare(ad116->htim, ad116->channel2, DutyRatio);
        }
        else if (ad116->param->DriveVeerFlag == true)
        {
            __HAL_TIM_SetCompare(ad116->htim, ad116->channel1, DutyRatio);
            __HAL_TIM_SetCompare(ad116->htim, ad116->channel2, ad116->htim->Instance->ARR);
        }
    }
    else if (ad116->param->DriveRunMode == 3)
    {
        /* Fast-decay mode: one output PWM, the other output off. */
        /* 快衰减模式：一路 PWM，另一路关闭。 */
        uint16_t DutyRatio = MAP(abs(ad116->param->DrivePower), 0, 1000, 0, ad116->htim->Instance->ARR);
        if (ad116->param->DrivePower>0) ad116->param->DriveVeerFlag =true;
        if (ad116->param->DrivePower<0) ad116->param->DriveVeerFlag =false;
        if (ad116->param->DriveVeerFlag == false)
        {
            __HAL_TIM_SetCompare(ad116->htim, ad116->channel1, DutyRatio);
            __HAL_TIM_SetCompare(ad116->htim, ad116->channel2, 0);
        }
        else if (ad116->param->DriveVeerFlag == true)
        {
            __HAL_TIM_SetCompare(ad116->htim, ad116->channel1, 0);
            __HAL_TIM_SetCompare(ad116->htim, ad116->channel2, DutyRatio);
        }
    }
}

void CycleStart(AD116* ad116,TIM_HandleTypeDef* htim)
{
    (void)ad116;
    __HAL_TIM_SET_COUNTER(htim, 0);
    HAL_TIM_Base_Start(htim);
}

void CycleBlockingTimer(AD116* ad116,TIM_HandleTypeDef* htim)
{
    ad116->param->ProcessTimeUs=__HAL_TIM_GET_COUNTER(htim);
    uint32_t target = ad116->param->CycleTimeMs * 1000;
    while (target > __HAL_TIM_GET_COUNTER(htim));
    HAL_TIM_Base_Stop(htim);
}
