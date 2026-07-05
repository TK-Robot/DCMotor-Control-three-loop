//
// Created by Administrator on 2025/12/7.
//

#include "MT6701.h"

#include <stdlib.h>

void MT6701_init(MT6701* MT,I2C_HandleTypeDef *hi2c1,Param* params)
{
    MT->iic = hi2c1;
    MT->param=params;
    MT6701_Update(MT);
    LPF_Filter_Init(&MT->SpeedFilter,32);
    LPF_Filter_Init(&MT->AccDecSpeedFilter,8);
    //HAL_Delay(100);
}
void I2C_Bus_Recovery(void) {
    GPIO_InitTypeDef gpio = {0};

    // 关闭I2C外设，防止冲突
    __HAL_I2C_DISABLE(&hi2c1);

    // 将SCL和SDA配置为推挽输出，初始高电平
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull = GPIO_NOPULL;

    gpio.Pin = 7;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, 7, GPIO_PIN_SET);

    gpio.Pin = 6;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, 6, GPIO_PIN_SET);

    // 发送最多9个时钟脉冲，唤醒可能卡住的从机
    for (int i = 0; i < 9; i++) {
        if (HAL_GPIO_ReadPin(GPIOB, 6) == GPIO_PIN_SET) {
            break; // SDA已释放，无需继续
        }
        HAL_GPIO_WritePin(GPIOB, 7, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, 7, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    // 恢复为正常AF开漏模式
    MX_I2C1_Init();// 重新初始化
}


void MT6701_Update(MT6701* MT)
{
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Read(MT->iic,MT6701_ADDRESS ,ReadAddressH,I2C_MEMADD_SIZE_8BIT,(uint8_t*)MT->param->EncoderReadData,2,500);
    if(ret != HAL_OK) {
        //I2C_Bus_Recovery();
        // 打印错误码
        // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET);
        // GPIO_PinState state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
        // GPIO_PinState state1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
        uint32_t err = HAL_I2C_GetError(MT->iic );
        if(err == HAL_I2C_ERROR_AF)
        {
            MT->dma_busy= true;
        }
        return;
    }
    MT6701_CodedManage(MT);

    int32_t diff = (int32_t)MT->param->EncoderValue
                 - (int32_t)MT->param->LastEncoderValue;

    // 正转过零/反转过零
    if (diff < -8192)MT->param->EncoderMultiTurn++;
    else if (diff > 8192)MT->param->EncoderMultiTurn--;

    MT->param->EncoderMultiTurnValue=MT->param->EncoderValue+MT->param->EncoderMultiTurn*16384;

    // if (MT->dma_busy) return;
    // if (HAL_I2C_Mem_Read_DMA(MT->iic,MT6701_ADDRESS ,ReadAddressH,I2C_MEMADD_SIZE_8BIT,(uint8_t*)MT->param->EncoderReadData,2) == HAL_OK)
    // {
    //     MT->dma_busy= true;
    // }
}

void MT6701_CodedManage(MT6701* MT)
{
    uint16_t data = MT->param->EncoderReadData[0];
    data <<= 8;
    data += MT->param->EncoderReadData[1];
    data >>= 2;
    MT->param->EncoderValue = data;

    int32_t val = (int32_t)data - (int32_t)MT->param->EncoderOffset;

    // 单圈回绕
    if (val < 0)        val += 16384;
    else if (val >= 16384) val -= 16384;

    MT->param->EncoderValue = (uint16_t)val;
}

void MT6701_SpeedUpdate(MT6701* MT)
{
    int32_t diff = (int32_t)MT->param->EncoderValue
                 - (int32_t)MT->param->LastEncoderValue;

    // 跨零修正
    if (diff >  8192) diff -= 16384;
    if (diff < -8192) diff += 16384;

    // 单周期最大变化保护
    // if (abs(diff) > 500)   // ← 根据你最高转速算
    // {
    //     MT->param->LastEncoderValue = MT->param->EncoderValue;
    //     return;
    // }

    MT->param->EncoderSpeed =(diff * 1000 / MT->param->CycleTimeMs);
    MT->param->EncoderSpeed=LPF_Filter_Update(&MT->SpeedFilter,MT->param->EncoderSpeed);

    MT->param->AccDec=(MT->param->EncoderSpeed-MT->param->LastEncoderSpeed)*(1000 / MT->param->CycleTimeMs);
    MT->param->AccDec=LPF_Filter_Update(&MT->AccDecSpeedFilter,MT->param->AccDec);

    MT->param->LastEncoderSpeed=MT->param->EncoderSpeed;

    MT->param->LastEncoderMultiTurnValue=MT->param->EncoderMultiTurnValue;
    MT->param->LastEncoderValue = MT->param->EncoderValue;
}
