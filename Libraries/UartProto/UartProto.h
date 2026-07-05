//
// Created by Administrator on 2025/12/7.
//

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_UARTPROTO_H
#define TRIPLE_CASCADECONTROLDCMOTOR_UARTPROTO_H

#include <stdbool.h>
#include "usart.h"
#include <TypeDefine.h>

// 外部 DMA 句柄
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
typedef struct
{
    // 属性
    Param* param;
    UART_HandleTypeDef* huart;

    // 方法（函数指针）
    // void (*UartProto_init)(void*,UART_HandleTypeDef*,Param*); // 用于打招呼的方法
    // void (*UartProto_RxEventCallback)(void*, uint16_t);
    // void (*UartProto_DMATxData)(void*, uint8_t*, uint16_t);
    // void (*UartProto_SendUint8)(void*, uint8_t*, uint16_t);
    // void (*UartProto_SendInt16)(void*, uint16_t, int16_t);
} UartProto;

void UartProto_init(UartProto* Proto,UART_HandleTypeDef *huart,Param* params);
void UartProto_RxEventCallback(UartProto* Proto, const UART_HandleTypeDef *huart,uint16_t Size);
void UartProto_DMATxData(const UartProto* Proto, const uint8_t* TxData,uint16_t Size);
void UartProto_SendUint8(const UartProto* Proto, uint8_t data);
void UartProto_SendInt16(const UartProto* Proto, int16_t data);
void UartProto_SendLongInt32(const UartProto* Proto,int32_t* data,int8_t size);
#endif // TRIPLE_CASCADECONTROLDCMOTOR_UARTPROTO_H