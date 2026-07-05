/**
 * @file UartProto.h
 * @brief Lightweight UART receive/telemetry protocol.
 * @brief 轻量串口接收与遥测协议。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_UARTPROTO_H
#define TRIPLE_CASCADECONTROLDCMOTOR_UARTPROTO_H

#include <stdbool.h>

#include "usart.h"
#include "TypeDefine.h"

extern DMA_HandleTypeDef hdma_usart2_rx; ///< USART2 RX DMA handle. / USART2 接收 DMA 句柄。
extern DMA_HandleTypeDef hdma_usart2_tx; ///< USART2 TX DMA handle. / USART2 发送 DMA 句柄。

/**
 * @brief Runtime UART protocol context.
 * @brief 串口协议运行上下文。
 */
typedef struct
{
    Param* param;               ///< Shared runtime parameters. / 共享运行参数。
    UART_HandleTypeDef* huart;  ///< UART peripheral handle. / UART 外设句柄。
    volatile bool tx_busy;      ///< DMA TX busy flag. / DMA 发送忙标志。
} UartProto;

/**
 * @brief Initialize UART idle DMA reception.
 * @brief 初始化串口空闲中断 DMA 接收。
 */
void UartProto_init(UartProto* Proto,UART_HandleTypeDef *huart,Param* params);

/**
 * @brief Handle HAL receive-to-idle callback and restart reception.
 * @brief 处理 HAL 空闲接收回调并重新启动接收。
 */
void UartProto_RxEventCallback(UartProto* Proto, const UART_HandleTypeDef *huart,uint16_t Size);

/**
 * @brief Send raw bytes through UART.
 * @brief 通过串口发送原始字节。
 */
void UartProto_DMATxData(const UartProto* Proto, const uint8_t* TxData,uint16_t Size);

/**
 * @brief Send one uint8 value.
 * @brief 发送一个 uint8 数值。
 */
void UartProto_SendUint8(const UartProto* Proto, uint8_t data);

/**
 * @brief Send one int16 value.
 * @brief 发送一个 int16 数值。
 */
void UartProto_SendInt16(const UartProto* Proto, int16_t data);

/**
 * @brief Send comma-separated int32 telemetry values using DMA.
 * @brief 使用 DMA 发送逗号分隔的 int32 遥测数据。
 */
void UartProto_SendLongInt32(const UartProto* Proto,const int32_t* data,int8_t size);

#endif // TRIPLE_CASCADECONTROLDCMOTOR_UARTPROTO_H
