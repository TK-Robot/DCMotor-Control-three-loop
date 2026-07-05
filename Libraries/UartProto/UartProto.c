//
// Created by Administrator on 2025/12/7.
//

#include "UartProto.h"

#include <iso646.h>
#include <stdlib.h>

void UartProto_init(UartProto* Proto,UART_HandleTypeDef *huart,Param* params)
{
    Proto->huart=huart;
    Proto->param=params;
    HAL_UARTEx_ReceiveToIdle_DMA(Proto->huart,Proto->param->RxBuf,sizeof(Proto->param->RxBuf));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
    Proto->param->ReturnEn=true;//接收返回关闭
}

void UartProto_RxEventCallback(UartProto* Proto, const UART_HandleTypeDef *huart,uint16_t Size)
{
    if (huart == Proto->huart)
    {
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
    //HAL_UART_Transmit_DMA(Proto->huart, TxData, Size);
    HAL_UART_Transmit(Proto->huart, TxData, Size,10);
}

void UartProto_SendLongInt32(const UartProto* Proto,int32_t* data,int8_t size)
{
    static char buf[64];   // 最大: "65535\r\n" => 7字节 + 结尾安全空间
    uint16_t len = 0;


    for (uint8_t i = 0; i < size; i++)
    {
        if (data[i] < 0)
        {
            buf[len++] = '-';
            data[i] = abs(data[i]);
        }
        if (data[i] >= 1000000000) buf[len++] = 0x30+ data[i] / 10000000;
        if (data[i] >= 100000000)  buf[len++] = 0x30+ (data[i]/ 100000000) % 10;
        if (data[i] >= 10000000)  buf[len++] = 0x30+ (data[i]/ 10000000) % 10;
        if (data[i] >= 1000000)  buf[len++] = 0x30+ (data[i]/ 1000000) % 10;
        if (data[i] >= 100000)  buf[len++] = 0x30+ (data[i]/ 100000) % 10;
        if (data[i] >= 10000)  buf[len++] = 0x30+ (data[i]/ 10000) % 10;
        if (data[i] >= 1000)  buf[len++] = 0x30+ (data[i]/ 1000) % 10;
        if (data[i] >= 100)   buf[len++] = 0x30+ (data[i]/ 100) % 10;
        if (data[i] >= 10)    buf[len++] = 0x30+ (data[i] / 10) % 10;
        buf[len++] = 0x30 + (data[i] % 10);

        if (size>1 && i< size-1)
        {
            buf[len++] = ',';
        }

    }

    //buf[len++] = '\r';
    buf[len++] = '\n';

    UartProto_DMATxData(Proto, (uint8_t*)buf, len);
}