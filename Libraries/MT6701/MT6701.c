/**
 * @file MT6701.c
 * @brief MT6701 magnetic encoder implementation.
 * @brief MT6701 磁编码器实现。
 */

#include "MT6701.h"

#include <stdlib.h>

extern uint32_t SystemCoreClock;

static bool MT6701_I2cFailed(I2C_TypeDef *i2c)
{
    return LL_I2C_IsActiveFlag_NACK(i2c)
           || LL_I2C_IsActiveFlag_BERR(i2c)
           || LL_I2C_IsActiveFlag_ARLO(i2c)
           || LL_I2C_IsActiveFlag_OVR(i2c);
}

static void MT6701_ClearI2cFlags(I2C_TypeDef *i2c)
{
    LL_I2C_ClearFlag_NACK(i2c);
    LL_I2C_ClearFlag_STOP(i2c);
    LL_I2C_ClearFlag_BERR(i2c);
    LL_I2C_ClearFlag_ARLO(i2c);
    LL_I2C_ClearFlag_OVR(i2c);
}

static bool MT6701_ReadAngle(I2C_TypeDef *i2c, volatile uint8_t data[2])
{
    uint32_t timeout = SystemCoreClock / 250U;

    MT6701_ClearI2cFlags(i2c);
    while (LL_I2C_IsActiveFlag_BUSY(i2c))
    {
        if (timeout-- == 0U) return false;
    }

    LL_I2C_HandleTransfer(i2c, MT6701_ADDRESS, LL_I2C_ADDRSLAVE_7BIT,
                          1U, LL_I2C_MODE_SOFTEND,
                          LL_I2C_GENERATE_START_WRITE);
    timeout = SystemCoreClock / 250U;
    while (!LL_I2C_IsActiveFlag_TXIS(i2c))
    {
        if (MT6701_I2cFailed(i2c) || timeout-- == 0U) goto failed;
    }
    LL_I2C_TransmitData8(i2c, MT6701_REG_ANGLE_H);

    timeout = SystemCoreClock / 250U;
    while (!LL_I2C_IsActiveFlag_TC(i2c))
    {
        if (MT6701_I2cFailed(i2c) || timeout-- == 0U) goto failed;
    }

    LL_I2C_HandleTransfer(i2c, MT6701_ADDRESS, LL_I2C_ADDRSLAVE_7BIT,
                          2U, LL_I2C_MODE_AUTOEND,
                          LL_I2C_GENERATE_START_READ);
    for (uint32_t index = 0U; index < 2U; ++index)
    {
        timeout = SystemCoreClock / 250U;
        while (!LL_I2C_IsActiveFlag_RXNE(i2c))
        {
            if (MT6701_I2cFailed(i2c) || timeout-- == 0U) goto failed;
        }
        data[index] = LL_I2C_ReceiveData8(i2c);
    }

    timeout = SystemCoreClock / 250U;
    while (!LL_I2C_IsActiveFlag_STOP(i2c))
    {
        if (MT6701_I2cFailed(i2c) || timeout-- == 0U) goto failed;
    }
    LL_I2C_ClearFlag_STOP(i2c);
    return true;

failed:
    LL_I2C_GenerateStopCondition(i2c);
    MT6701_ClearI2cFlags(i2c);
    return false;
}

void MT6701_init(MT6701 *MT, I2C_TypeDef *i2c, Param *params)
{
    MT->i2c = i2c;
    MT->param=params;
    MT->position_initialized = false;
    MT6701_Update(MT);
    LPF_Filter_Init(&MT->SpeedFilter,32);
    LPF_Filter_Init(&MT->AccDecSpeedFilter,8);
}

void I2C_Bus_Recovery(void) {
    LL_GPIO_InitTypeDef gpio = {0};

    /* Temporarily release the I2C peripheral before manual clock pulsing. */
    /* 手动恢复总线前先临时关闭 I2C 外设。 */
    LL_I2C_Disable(I2C1);

    gpio.Mode = LL_GPIO_MODE_OUTPUT;
    gpio.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio.Pull = LL_GPIO_PULL_NO;
    gpio.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    gpio.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
    LL_GPIO_Init(GPIOB, &gpio);
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8 | LL_GPIO_PIN_9);

    /* Generate up to 9 clock pulses to release a slave holding SDA low. */
    /* 最多发送 9 个时钟脉冲，用于释放可能拉低 SDA 的从机。 */
    for (int i = 0; i < 9; i++) {
        if (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_9)) {
            break;
        }
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8);
        for (volatile uint32_t wait = 0U; wait < SystemCoreClock / 8000U; ++wait) {}
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8);
        for (volatile uint32_t wait = 0U; wait < SystemCoreClock / 8000U; ++wait) {}
    }

    MX_I2C1_Init();
}

void MT6701_Update(MT6701* MT)
{
    if (!MT6701_ReadAngle(MT->i2c, MT->param->EncoderReadData))
    {
        MT->dma_busy = true;
        return;
    }
    MT->dma_busy = false;
    MT6701_CodedManage(MT);

    if (MT->param->EncoderRebaseline)
    {
        /* Configuration changes start a new logical position baseline. */
        /* 编码器配置变化后重新建立逻辑位置基准。 */
        MT->param->EncoderRebaseline = false;
        MT->param->EncoderMultiTurn = 0;
        MT->param->LastEncoderValue = MT->param->EncoderValue;
        MT->param->EncoderMultiTurnValue = MT->param->EncoderValue;
        MT->param->LastEncoderMultiTurnValue = MT->param->EncoderValue;
        MT->position_initialized = true;
        return;
    }

    if (!MT->position_initialized)
    {
        /* Establish the baseline without treating the first sample as motion. */
        /* 建立首个基准样本，避免把上电首帧误判为跨圈运动。 */
        MT->param->LastEncoderValue = MT->param->EncoderValue;
        MT->param->EncoderMultiTurnValue =
            (int32_t)MT->param->EncoderValue + MT->param->EncoderMultiTurn * 16384L;
        MT->param->LastEncoderMultiTurnValue = MT->param->EncoderMultiTurnValue;
        MT->position_initialized = true;
        return;
    }

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

    if (MT->param->EncoderVeer && val != 0)
    {
        val = 16384 - val;
    }

    MT->param->EncoderValue = (uint16_t)val;
}

void MT6701_SpeedUpdate(MT6701* MT, uint32_t sample_period_ms)
{
    int32_t diff = (int32_t)MT->param->EncoderValue
                 - (int32_t)MT->param->LastEncoderValue;

    if (sample_period_ms == 0U)
    {
        sample_period_ms = (MT->param->CycleTimeMs == 0U) ? 1U : MT->param->CycleTimeMs;
    }

    /* Correct speed delta when the encoder crosses the zero point. */
    /* 编码器跨零时修正速度差值。 */
    if (diff >  8192) diff -= 16384;
    if (diff < -8192) diff += 16384;

    /* Use the actual encoder refresh period, not the 1 ms base scheduler period. */
    /* 使用编码器实际刷新周期，而不是 1 ms 基础调度周期。 */
    MT->param->EncoderSpeed =(diff * 1000 / (int32_t)sample_period_ms);
    MT->param->EncoderSpeed=LPF_Filter_Update(&MT->SpeedFilter,MT->param->EncoderSpeed);

    MT->param->AccDec=(MT->param->EncoderSpeed-MT->param->LastEncoderSpeed)
                    * (1000 / (int32_t)sample_period_ms);
    MT->param->AccDec=LPF_Filter_Update(&MT->AccDecSpeedFilter,MT->param->AccDec);

    MT->param->LastEncoderSpeed=MT->param->EncoderSpeed;

    MT->param->LastEncoderMultiTurnValue=MT->param->EncoderMultiTurnValue;
    MT->param->LastEncoderValue = MT->param->EncoderValue;
}
