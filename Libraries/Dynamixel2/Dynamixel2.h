/**
 * @file Dynamixel2.h
 * @brief TK Servo DYNAMIXEL Protocol 2.0 slave interface.
 * @brief TK Servo DYNAMIXEL Protocol 2.0 从站接口。
 */

#ifndef TK_DYNAMIXEL2_H
#define TK_DYNAMIXEL2_H

#include "Dynamixel2Codec.h"
#include "TypeDefine.h"
#include "usart.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DXL2_CONTROL_TABLE_SIZE  153U   /**< Highest address 152 plus one. / 最高地址 152 加一。 */
#define DXL2_RX_STREAM_SIZE      256U   /**< Reassembly buffer across DMA idle events. / 跨 DMA 空闲事件的重组缓冲区。 */
#define DXL2_MODEL_NUMBER        0x0001U /**< TK Servo model number returned by Ping. / Ping 返回的 TK Servo 型号。 */
#define DXL2_FIRMWARE_VERSION    0x01U   /**< One-byte firmware revision. / 单字节固件版本。 */

typedef enum
{
    DXL2_DIAG_NONE = 0,          /**< No recorded communication fault. / 无通信诊断故障。 */
    DXL2_DIAG_UART_ERROR = 1,    /**< HAL UART/DMA error. / HAL UART 或 DMA 错误。 */
    DXL2_DIAG_RX_CRC = 2,        /**< Packet CRC mismatch. / 数据包 CRC 不匹配。 */
    DXL2_DIAG_RX_BAD_PACKET = 3, /**< Header, ID, length, or body error. / 包头、ID、长度或包体错误。 */
    DXL2_DIAG_TX_DROP = 4,       /**< Both active and one-deep pending TX were occupied. / 当前发送和一级待发槽均被占用。 */
    DXL2_DIAG_WATCHDOG = 5       /**< Enabled serial control stopped refreshing. / 已使能串口控制停止刷新。 */
} Dynamixel2DiagnosticCode;

/**
 * @brief Runtime owner for one DYNAMIXEL slave endpoint. / 单个 DYNAMIXEL 从站端点的运行期所有者。
 *
 * The UART callbacks and 1 ms loop share this object; fields are not exposed as a
 * second source of motor truth. Motor state remains in Param and ServoControl.
 * UART 回调与 1 ms 主循环共享本对象；本结构不是第二份电机状态源，电机状态仍由 Param 和 ServoControl 持有。
 */
typedef struct
{
    UART_HandleTypeDef *huart;       /**< Non-owning UART2 handle. / 非持有的 UART2 句柄。 */
    Param *param;                    /**< Shared live parameter/state owner. / 共享运行参数与状态所有者。 */
    ServoCommand pending_command;   /**< Validated command awaiting its apply tick. / 已校验、等待生效时刻的命令。 */
    ServoCommand active_command;    /**< Command consumed by ServoControl. / ServoControl 当前使用的命令。 */
    uint32_t tick_ms;                /**< Monotonic 1 ms protocol time base. / 单调递增的 1 ms 协议时基。 */
    uint32_t execute_tick;           /**< Scheduled command apply tick; zero means immediate. / 命令计划生效时刻；零表示立即。 */
    uint16_t serial_watchdog_count_ms; /**< Elapsed enabled-serial silence. / 串口使能状态下未刷新的毫秒数。 */
    uint8_t node_id;                 /**< Active node ID, 1..252. / 当前节点 ID，范围 1..252。 */
    bool watchdog_latched;           /**< Prevent repeated timeout accounting. / 防止重复记录同一次超时。 */
    bool pending_valid;              /**< Pending command is ready for atomic apply. / 待处理命令可原子生效。 */
    bool tx_busy;                    /**< UART DMA currently owns Param.TxBuf. / UART DMA 当前占用 Param.TxBuf。 */
    bool rx_active;                  /**< Receive-to-idle DMA has been armed. / Receive-to-idle DMA 已启动。 */
    bool save_request;               /**< Deferred NVM save request for main loop. / 交给主循环处理的延迟 NVM 保存请求。 */
    bool pending_tx_valid;           /**< One queued status packet is present. / 存在一个排队状态包。 */
    uint16_t pending_tx_length;      /**< Queued encoded status length. / 排队状态包的编码长度。 */
    uint8_t pending_tx[DXL2_MAX_PACKET_SIZE]; /**< One-deep TX queue storage. / 深度为一的发送队列存储。 */
    uint16_t rx_stream_length;       /**< Valid bytes in rx_stream. / rx_stream 中的有效字节数。 */
    uint8_t rx_stream[DXL2_RX_STREAM_SIZE]; /**< Packet reassembly storage. / 数据包重组存储区。 */
    bool registered_write_valid;     /**< Reg Write data awaits Action. / Reg Write 数据正在等待 Action。 */
    uint16_t registered_address;     /**< Registered Control Table address. / 已登记控制表地址。 */
    uint16_t registered_length;      /**< Registered data length. / 已登记数据长度。 */
    uint8_t registered_data[DXL2_MAX_PARAMETERS]; /**< Registered data snapshot. / 已登记数据快照。 */
    uint16_t last_diag_error;        /**< Latest communication diagnostic code. / 最近通信诊断码。 */
    uint32_t diagnostic_error_count; /**< Saturating aggregate diagnostic count. / 饱和累计诊断计数。 */
    uint32_t last_uart_error;        /**< Raw HAL UART error flags. / 最近一次 HAL UART 原始错误位。 */
    uint32_t uart_error_count;       /**< Saturating UART error count. / 饱和累计 UART 错误数。 */
    uint32_t rx_packet_count;        /**< Valid decoded packet count. / 有效解码包计数。 */
    uint32_t rx_crc_error_count;     /**< CRC rejection count. / CRC 拒收计数。 */
    uint32_t rx_bad_packet_count;    /**< Non-CRC packet rejection count. / 非 CRC 坏包计数。 */
    uint32_t tx_drop_count;          /**< Status responses dropped by bounded TX queue. / 有界发送队列丢弃的状态响应数。 */
} Dynamixel2Context;

/**
 * @brief Initialize a disarmed slave and arm receive-to-idle DMA. / 初始化未使能从站并启动空闲 DMA 接收。
 * @param context Caller-owned lifetime context. / 调用者持有、全生命周期有效的上下文。
 * @param huart UART2 handle used for the servo bus. / 伺服总线使用的 UART2 句柄。
 * @param param Shared live parameter/state object. / 共享运行参数与状态对象。
 */
void Dynamixel2_Init(Dynamixel2Context *context, UART_HandleTypeDef *huart, Param *param);

/**
 * @brief Re-arm receive-to-idle DMA after an error. / 发生错误后重新启动空闲 DMA 接收。
 * @return true when HAL accepted the receive request. / HAL 接受接收请求时返回 true。
 */
bool Dynamixel2_RestartRx(Dynamixel2Context *context);

/**
 * @brief Preserve raw HAL UART evidence and increment diagnostics. / 保留 HAL UART 原始错误证据并累计诊断。
 */
void Dynamixel2_RecordUartError(Dynamixel2Context *context, uint32_t error_code);

/**
 * @brief Copy one DMA idle chunk, re-arm RX, and consume complete packets. / 复制一次 DMA 空闲分段、重启接收并处理完整包。
 * @note Call only from HAL_UARTEx_RxEventCallback for the configured UART. / 仅从已配置 UART 的 HAL 回调调用。
 */
void Dynamixel2_RxEventCallback(Dynamixel2Context *context,
                                const UART_HandleTypeDef *huart, uint16_t size);

/**
 * @brief Release the DMA TX buffer and start one queued response. / 释放 DMA 发送缓冲并启动一个排队响应。
 */
void Dynamixel2_TxCpltCallback(Dynamixel2Context *context,
                               const UART_HandleTypeDef *huart);

/**
 * @brief Advance watchdog/scheduling state from the bounded 1 ms loop. / 在有界 1 ms 循环中推进看门狗与命令调度。
 */
void Dynamixel2_1msTick(Dynamixel2Context *context);

/**
 * @brief Return the immutable command currently consumed by ServoControl. / 返回 ServoControl 当前读取的只读命令。
 */
const ServoCommand *Dynamixel2_GetActiveCommand(const Dynamixel2Context *context);

/**
 * @brief Consume one deferred address-152 NVM save request. / 取走一次由地址 152 触发的延迟 NVM 保存请求。
 * @return true exactly once per accepted save command. / 每个已接受保存命令仅返回一次 true。
 */
bool Dynamixel2_ConsumeSaveRequest(Dynamixel2Context *context);

#ifdef __cplusplus
}
#endif

#endif /* TK_DYNAMIXEL2_H */
