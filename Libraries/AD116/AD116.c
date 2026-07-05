//
// Created by Administrator on 2025/12/8.
//

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


    // 重新启动 PWM，不影响占空比
    HAL_TIM_PWM_Start(ad116->htim, ad116->channel1);
    HAL_TIM_PWM_Start(ad116->htim, ad116->channel2);
}

void AD116_Update(AD116* ad116, Param *param)
{
    if (ad116->param->DrivePower>=0 && ad116->param->DrivePower < PowerMin)ad116->param->DrivePower = 0;
    if (ad116->param->DrivePower<=0 && ad116->param->DrivePower > -PowerMin)ad116->param->DrivePower = 0;
    if (ad116->param->DriveRunMode == 0)
    {
        //滑行
        __HAL_TIM_SetCompare(ad116->htim, ad116->channel1, 0);
        __HAL_TIM_SetCompare(ad116->htim, ad116->channel2, 0);
    }
    else if (ad116->param->DriveRunMode == 1)
    {
        //刹车
        __HAL_TIM_SetCompare(ad116->htim, ad116->channel1, ad116->htim->Instance->ARR);
        __HAL_TIM_SetCompare(ad116->htim, ad116->channel2, ad116->htim->Instance->ARR);
    }
    else if (ad116->param->DriveRunMode == 2)
    {
        //慢衰减 正反转&刹车
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
        //快衰减 正反转&滑行
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
