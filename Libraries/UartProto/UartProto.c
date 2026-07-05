/**
 * @file UartProto.c
 * @brief UART receive and telemetry protocol implementation.
 * @brief 串口接收与遥测协议实现。
 */

#include "UartProto.h"

#include <iso646.h>
#include <stdlib.h>

void UartProto_init(UartProto* Proto,UART_HandleTypeDef *huart,Param* params)
{
    Proto->huart=huart;
    Proto->param=params;
    Proto->tx_busy=false;
    HAL_UARTEx_ReceiveToIdle_DMA(Proto->huart,Proto->param->RxBuf,sizeof(Proto->param->RxBuf));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
    Proto->param->ReturnEn=true;
}

void UartProto_RxEventCallback(UartProto* Proto, const UART_HandleTypeDef *huart,uint16_t Size)
{
    if (huart == Proto->huart)
    {
        /* Restart receive-to-idle DMA before optional echo. */
        /* 先重新启动空闲接收 DMA，再根据配置回显数据。 */
        HAL_UARTEx_ReceiveToIdle_DMA(Proto->huart, Proto->param->RxBuf, sizeof(Proto->param->RxBuf));
        if (Proto->param->ReturnEn)
        {
            UartProto_DMATxData(Proto, Proto->param->RxBuf,Size);
        }
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
    }
}

void UartProto_DMATxData(const UartProto* Proto, const uint8_t* TxData,uint16_t Size)
{
    /* RX echo stays blocking to avoid sharing the telemetry DMA buffer. */
    /* 接收回显仍使用阻塞发送，避免和遥测 DMA 缓冲区冲突。 */
    HAL_UART_Transmit(Proto->huart, TxData, Size,10);
}

void UartProto_SendUint8(const UartProto* Proto, uint8_t data)
{
    UartProto_DMATxData(Proto, &data, sizeof(data));
}

void UartProto_SendInt16(const UartProto* Proto, int16_t data)
{
    uint8_t bytes[2];

    bytes[0] = (uint8_t)(data & 0xFF);
    bytes[1] = (uint8_t)(((uint16_t)data >> 8) & 0xFF);
    UartProto_DMATxData(Proto, bytes, sizeof(bytes));
}

void UartProto_SendLongInt32(const UartProto* Proto,const int32_t* data,int8_t size)
{
    static char buf[64];
    uint16_t len = 0;

    if (Proto->tx_busy || size <= 0)
    {
        return;
    }

    for (uint8_t i = 0; i < size; i++)
    {
        int32_t value = data[i];
        uint32_t magnitude;

        /* Convert manually to avoid pulling in printf on a small MCU. */
        /* 手动转换整数，避免在小容量 MCU 上引入 printf。 */
        if (value < 0)
        {
            buf[len++] = '-';
            magnitude = (uint32_t)(-(value + 1)) + 1U;
        }
        else
        {
            magnitude = (uint32_t)value;
        }

        uint32_t divisor = 1000000000U;
        while (divisor > 1U && magnitude < divisor)
        {
            divisor /= 10U;
        }
        while (divisor > 0U)
        {
            buf[len++] = (char)('0' + magnitude / divisor);
            magnitude %= divisor;
            divisor /= 10U;
        }

        if (size>1 && i< size-1)
        {
            buf[len++] = ',';
        }
    }

    buf[len++] = '\n';

    ((UartProto*)Proto)->tx_busy = true;
    if (HAL_UART_Transmit_DMA(Proto->huart, (uint8_t*)buf, len) != HAL_OK)
    {
        ((UartProto*)Proto)->tx_busy = false;
    }
}
