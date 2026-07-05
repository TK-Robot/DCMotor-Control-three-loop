/**
 * @file MT6701.c
 * @brief MT6701 magnetic encoder implementation.
 * @brief MT6701 磁编码器实现。
 */

#include "MT6701.h"

#include <stdlib.h>

void MT6701_init(MT6701* MT,I2C_HandleTypeDef *hi2c1,Param* params)
{
    MT->iic = hi2c1;
    MT->param=params;
    MT6701_Update(MT);
    LPF_Filter_Init(&MT->SpeedFilter,32);
    LPF_Filter_Init(&MT->AccDecSpeedFilter,8);
}

void I2C_Bus_Recovery(void) {
    GPIO_InitTypeDef gpio = {0};

    /* Temporarily release the I2C peripheral before manual clock pulsing. */
    /* 手动恢复总线前先临时关闭 I2C 外设。 */
    __HAL_I2C_DISABLE(&hi2c1);

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pull = GPIO_NOPULL;

    gpio.Pin = 7;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, 7, GPIO_PIN_SET);

    gpio.Pin = 6;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, 6, GPIO_PIN_SET);

    /* Generate up to 9 clock pulses to release a slave holding SDA low. */
    /* 最多发送 9 个时钟脉冲，用于释放可能拉低 SDA 的从机。 */
    for (int i = 0; i < 9; i++) {
        if (HAL_GPIO_ReadPin(GPIOB, 6) == GPIO_PIN_SET) {
            break;
        }
        HAL_GPIO_WritePin(GPIOB, 7, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, 7, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    MX_I2C1_Init();
}

void MT6701_Update(MT6701* MT)
{
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Read(MT->iic,MT6701_ADDRESS ,ReadAddressH,I2C_MEMADD_SIZE_8BIT,(uint8_t*)MT->param->EncoderReadData,2,MT6701_I2C_TIMEOUT_MS);
    if(ret != HAL_OK) {
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

    /* Detect single-turn wraparound and update the multi-turn counter. */
    /* 检测单圈跨零并更新多圈计数。 */
    if (diff < -8192)MT->param->EncoderMultiTurn++;
    else if (diff > 8192)MT->param->EncoderMultiTurn--;

    MT->param->EncoderMultiTurnValue=MT->param->EncoderValue+MT->param->EncoderMultiTurn*16384;
}

void MT6701_CodedManage(MT6701* MT)
{
    uint16_t data = MT->param->EncoderReadData[0];
    data <<= 8;
    data += MT->param->EncoderReadData[1];
    data >>= 2;
    MT->param->EncoderValue = data;

    int32_t val = (int32_t)data - (int32_t)MT->param->EncoderOffset;

    /* Apply zero offset and wrap the corrected angle into 0..16383. */
    /* 应用零位偏移，并把校正后的角度限制在 0..16383。 */
    if (val < 0)        val += 16384;
    else if (val >= 16384) val -= 16384;

    MT->param->EncoderValue = (uint16_t)val;
}

void MT6701_SpeedUpdate(MT6701* MT)
{
    int32_t diff = (int32_t)MT->param->EncoderValue
                 - (int32_t)MT->param->LastEncoderValue;

    /* Correct speed delta when the encoder crosses the zero point. */
    /* 编码器跨零时修正速度差值。 */
    if (diff >  8192) diff -= 16384;
    if (diff < -8192) diff += 16384;

    MT->param->EncoderSpeed =(diff * 1000 / MT->param->CycleTimeMs);
    MT->param->EncoderSpeed=LPF_Filter_Update(&MT->SpeedFilter,MT->param->EncoderSpeed);

    MT->param->AccDec=(MT->param->EncoderSpeed-MT->param->LastEncoderSpeed)*(1000 / MT->param->CycleTimeMs);
    MT->param->AccDec=LPF_Filter_Update(&MT->AccDecSpeedFilter,MT->param->AccDec);

    MT->param->LastEncoderSpeed=MT->param->EncoderSpeed;

    MT->param->LastEncoderMultiTurnValue=MT->param->EncoderMultiTurnValue;
    MT->param->LastEncoderValue = MT->param->EncoderValue;
}
