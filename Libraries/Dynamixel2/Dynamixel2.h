/**
 * @file Dynamixel2.h
 * @brief TK Servo DYNAMIXEL Protocol 2.0 slave interface.
 * @brief TK Servo DYNAMIXEL Protocol 2.0 从站接口。
 */

#ifndef TK_DYNAMIXEL2_H
#define TK_DYNAMIXEL2_H

#include "Dynamixel2ControlTable.h"
#include "Dynamixel2Codec.h"
#include "TypeDefine.h"
#include "usart.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DXL2_RX_STREAM_SIZE      256U   /**< Reassembly buffer across DMA idle events. / 跨 DMA 空闲事件的重组缓冲区。 */
#define DXL2_MODEL_NUMBER        0x0001U /**< TK Servo model number returned by Ping. / Ping 返回的 TK Servo 型号。 */
#define DXL2_FIRMWARE_VERSION    0x19U   /**< One-byte firmware revision. / 单字节固件版本。 */

/** Observe copied RX bytes without taking ownership of the UART transport. / 观察已复制的接收字节，但不取得 UART 传输所有权。 */
typedef void (*Dynamixel2RxObserver)(void *user, const uint8_t *data,
                                     uint16_t length);

typedef struct
{
    int32_t actual_velocity;
    int32_t actual_position;
    int32_t multi_turn_position;
    uint32_t current_tick;
    uint16_t status_word;
    uint16_t fault_code;
    int16_t actual_current;
    int16_t drive_output;
    uint16_t supply_voltage;
    int8_t temperature;
} Dynamixel2StatusSnapshot;

/**
 * @brief Runtime owner for one DYNAMIXEL slave endpoint. / 单个 DYNAMIXEL 从站端点的运行期所有者。
 *
 * The UART callbacks and 1 ms loop share this object; fields are not exposed as a
 * second source of motor truth. Motor state remains in Param and ServoControl.
 * UART 回调与 1 ms 主循环共享本对象；本结构不是第二份电机状态源，电机状态仍由 Param 和 ServoControl 持有。
 */
typedef struct
{
    USART_TypeDef *uart;             /**< Non-owning USART instance. / 非持有的 USART 外设实例。 */
    Param *param;                    /**< Shared live parameter/state owner. / 共享运行参数与状态所有者。 */
    Dynamixel2RxObserver rx_observer; /**< Optional shared-UART protocol observer. / 可选的共享 UART 协议观察者。 */
    void *rx_observer_user;          /**< Observer-owned context. / 观察者持有的上下文。 */
    ServoCommand pending_command;   /**< Validated command awaiting its apply tick. / 已校验、等待生效时刻的命令。 */
    ServoCommand active_command;    /**< Command consumed by ServoControl. / ServoControl 当前使用的命令。 */
    uint32_t tick_ms;                /**< Monotonic 1 ms protocol time base. / 单调递增的 1 ms 协议时基。 */
    uint32_t execute_tick;           /**< Absolute scheduled apply tick; zero is a valid wrapped time. / 绝对计划生效时刻；零是有效回绕时刻。 */
    bool execute_scheduled;          /**< Pending command uses execute_tick instead of immediate apply. / 待生效命令使用 execute_tick 而非立即执行。 */
    uint16_t pending_sequence;       /**< Sequence attached to the pending command image. / 待生效命令映像携带的序号。 */
    uint16_t accepted_sequence;      /**< Latest sequence accepted into the pending slot. / 最近成功进入待生效槽的序号。 */
    uint16_t applied_sequence;       /**< Sequence of the latest applied command image. / 最近已生效命令映像的序号。 */
    bool accepted_sequence_valid;    /**< At least one sequenced command was accepted. / 至少接受过一条带序号命令。 */
    bool pending_sequence_valid;     /**< pending_sequence belongs to a full command image. / pending_sequence 属于完整命令映像。 */
    bool applied_sequence_valid;     /**< At least one sequenced command has been applied. / 至少已有一条带序号命令生效。 */
    uint8_t last_command_result;     /**< Standard Status error for the latest command image. / 最近命令映像的标准 Status 错误码。 */
    uint16_t serial_watchdog_count_ms; /**< Elapsed enabled-serial silence. / 串口使能状态下未刷新的毫秒数。 */
    uint8_t node_id;                 /**< Active node ID, 1..252. / 当前节点 ID，范围 1..252。 */
    uint8_t pending_node_id;         /**< Validated ID applied after the old-ID ACK completes. / 旧 ID ACK 完成后应用的新 ID。 */
    bool node_id_change_after_tx;    /**< Current DMA TX completion commits pending_node_id. / 当前 DMA TX 完成时提交 pending_node_id。 */
    uint32_t pending_baud_rate;      /**< Validated baud applied outside the UART callback. / 已校验、等待在 UART 回调外应用的波特率。 */
    bool baud_change_after_tx;       /**< TX completion releases the staged baud change. / 当前发送完成后释放已暂存的波特率切换。 */
    bool baud_change_ready;          /**< Main loop must reconfigure UART and RX DMA. / 主循环需要重配 UART 与 RX DMA。 */
    bool baud_change_in_progress;    /**< Responses wait while main reconfigures UART. / 主循环重配 UART 期间暂停响应发送。 */
    bool watchdog_latched;           /**< Prevent repeated timeout accounting. / 防止重复记录同一次超时。 */
    bool pending_valid;              /**< Pending command is ready for atomic apply. / 待处理命令可原子生效。 */
    bool tx_busy;                    /**< UART DMA currently owns Param.TxBuf. / UART DMA 当前占用 Param.TxBuf。 */
    bool rx_active;                  /**< Receive-to-idle DMA has been armed. / Receive-to-idle DMA 已启动。 */
    bool save_request;               /**< Deferred NVM save request for main loop. / 交给主循环处理的延迟 NVM 保存请求。 */
    bool save_status_pending;        /**< Unicast Save NVM awaits durable completion status. / 单播 NVM 保存正等待持久化完成状态。 */
    bool pending_tx_valid;           /**< One queued status packet is present. / 存在一个排队状态包。 */
    bool pending_tx_delayed;         /**< TIM1 owns the queued packet until its reply deadline. / TIM1 在回复时限前持有排队包。 */
    uint16_t pending_tx_length;      /**< Queued encoded status length. / 排队状态包的编码长度。 */
    uint8_t pending_tx[DXL2_MAX_PACKET_SIZE]; /**< One-deep TX queue storage. / 深度为一的发送队列存储。 */
    uint16_t rx_stream_length;       /**< Valid bytes in rx_stream. / rx_stream 中的有效字节数。 */
    uint16_t rx_packet_end_us;       /**< TIM1 timestamp captured at the receive-idle event. / 接收空闲事件捕获的 TIM1 时间戳。 */
    uint8_t rx_stream[DXL2_RX_STREAM_SIZE]; /**< Packet reassembly storage. / 数据包重组存储区。 */
    bool registered_write_valid;     /**< Reg Write data awaits Action. / Reg Write 数据正在等待 Action。 */
    uint16_t registered_address;     /**< Registered Control Table address. / 已登记控制表地址。 */
    uint16_t registered_length;      /**< Registered data length. / 已登记数据长度。 */
    uint8_t registered_data[DXL2_MAX_PARAMETERS]; /**< Registered data snapshot. / 已登记数据快照。 */
    uint16_t last_diag_error;        /**< Latest communication diagnostic code. / 最近通信诊断码。 */
    uint32_t diagnostic_error_count; /**< Saturating aggregate diagnostic count. / 饱和累计诊断计数。 */
    uint32_t last_uart_error;        /**< Raw LL USART/DMA error flags. / 最近一次 LL USART/DMA 原始错误位。 */
    uint32_t uart_error_count;       /**< Saturating UART error count. / 饱和累计 UART 错误数。 */
    uint32_t rx_packet_count;        /**< Valid decoded packet count. / 有效解码包计数。 */
    uint32_t rx_crc_error_count;     /**< CRC rejection count. / CRC 拒收计数。 */
    uint32_t rx_bad_packet_count;    /**< Non-CRC packet rejection count. / 非 CRC 坏包计数。 */
    uint32_t tx_drop_count;          /**< Status responses dropped by bounded TX queue. / 有界发送队列丢弃的状态响应数。 */
    Dynamixel2StatusSnapshot status_snapshot[2]; /**< Control-loop-published immutable snapshots. / 控制循环发布的不可变快照。 */
    uint8_t status_snapshot_index;   /**< Currently published snapshot index. / 当前已发布快照索引。 */
} Dynamixel2Context;

/**
 * @brief Initialize a disarmed slave and arm receive-to-idle DMA. / 初始化未使能从站并启动空闲 DMA 接收。
 * @param context Caller-owned lifetime context. / 调用者持有、全生命周期有效的上下文。
 * @param huart UART2 handle used for the servo bus. / 伺服总线使用的 UART2 句柄。
 * @param param Shared live parameter/state object. / 共享运行参数与状态对象。
 */
void Dynamixel2_Init(Dynamixel2Context *context, USART_TypeDef *uart, Param *param);

/** Register a bounded observer called once per copied DMA RX chunk. / 注册每个已复制 DMA 接收分段调用一次的有界观察者。 */
void Dynamixel2_SetRxObserver(Dynamixel2Context *context,
                              Dynamixel2RxObserver observer, void *user);

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
 * @note Called by the LL USART/DMA interrupt path after DMA is stopped. / 由 LL USART/DMA 中断路径在停止 DMA 后调用。
 */
void Dynamixel2_RxEventCallback(Dynamixel2Context *context, uint16_t size);

/**
 * @brief Release the DMA TX buffer and start one queued response. / 释放 DMA 发送缓冲并启动一个排队响应。
 */
void Dynamixel2_TxCpltCallback(Dynamixel2Context *context);

/**
 * @brief Handle USART IDLE, TC, and receive-error interrupts.
 * @brief 处理 USART 空闲、发送完成和接收错误中断。
 */
void Dynamixel2_UartIrqHandler(Dynamixel2Context *context);

/**
 * @brief Handle USART RX/TX DMA channel interrupts.
 * @brief 处理 USART 收发 DMA 通道中断。
 */
void Dynamixel2_DmaIrqHandler(Dynamixel2Context *context);

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

/**
 * @brief Complete a deferred unicast Save NVM transaction with a Status Packet.
 * @brief 用状态包完成一个延迟的单播 NVM 保存事务。
 * @param context Protocol context. / 协议上下文。
 * @param success true only after Flash save completed successfully. / 仅在 Flash 保存成功完成后传 true。
 */
void Dynamixel2_CompleteSaveRequest(Dynamixel2Context *context, bool success);

/**
 * @brief Consume a baud change only after the old-baud ACK reached UART TC.
 * @brief 仅在旧波特率 ACK 到达 UART TC 后取走波特率切换请求。
 * @param baud Receives the validated new baud rate. / 接收已校验的新波特率。
 * @return true once for each ready runtime baud change. / 每个可执行的运行期波特率切换仅返回一次 true。
 */
bool Dynamixel2_ConsumeBaudRateChange(Dynamixel2Context *context,
                                      uint32_t *baud);

/**
 * @brief Finish the main-loop UART reconfiguration and release queued replies.
 * @brief 完成主循环 UART 重配置，并允许继续发送排队响应。
 */
void Dynamixel2_CompleteBaudRateChange(Dynamixel2Context *context,
                                       bool success);

#ifdef __cplusplus
}
#endif

#endif /* TK_DYNAMIXEL2_H */
