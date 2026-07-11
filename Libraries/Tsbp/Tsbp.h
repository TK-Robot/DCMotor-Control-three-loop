/**
 * @file Tsbp.h
 * @brief TK Servo Bus Protocol slave core interface.
 * @brief TK Servo Bus Protocol 从站核心接口。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_TSBP_H
#define TRIPLE_CASCADECONTROLDCMOTOR_TSBP_H

#include <stdbool.h>
#include <stdint.h>

#include "TypeDefine.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TSBP_VERSION              0x01U
#define TSBP_SOF0                 0xA5U
#define TSBP_SOF1                 0x5AU
#define TSBP_FAST_SOF             0xA6U
#define TSBP_MASTER_ID            0x00U
#define TSBP_DEFAULT_NODE_ID      0x01U
#define TSBP_BROADCAST_ID         0x7FU
#define TSBP_MAX_PAYLOAD          32U
#define TSBP_MAX_NODES            4U
#define TSBP_RX_PDO_SIZE          13U
#define TSBP_TX_PDO_SIZE          17U
#define TSBP_DEFAULT_REPLY_SLOT_US 120U
#define TSBP_MAX_REPLY_DELAY_US   1000U

typedef enum
{
    TSBP_STATE_BOOT = 0,
    TSBP_STATE_INIT,
    TSBP_STATE_PREOP,
    TSBP_STATE_SAFEOP,
    TSBP_STATE_OP,
    TSBP_STATE_FAULT
} TsbpState;

typedef enum
{
    TSBP_TYPE_SYNC = 0x01,
    TSBP_TYPE_PDO_CYCLE = 0x02,
    TSBP_TYPE_SDO_READ = 0x10,
    TSBP_TYPE_SDO_WRITE = 0x11,
    TSBP_TYPE_SDO_REPLY = 0x12,
    TSBP_TYPE_PING = 0x30,
    TSBP_TYPE_ERROR = 0x7F
} TsbpFrameType;

typedef enum
{
    TSBP_ERR_NONE = 0,
    TSBP_ERR_CRC = 0x01,
    TSBP_ERR_UNSUPPORTED_VERSION = 0x02,
    TSBP_ERR_UNSUPPORTED_TYPE = 0x03,
    TSBP_ERR_BAD_LENGTH = 0x04,
    TSBP_ERR_BAD_STATE = 0x05,
    TSBP_ERR_ACCESS = 0x06,
    TSBP_ERR_RANGE = 0x07,
    TSBP_ERR_BUSY = 0x08,
    TSBP_ERR_PDO_MAP = 0x09,
    TSBP_ERR_WATCHDOG = 0x0A
} TsbpErrorCode;

typedef struct
{
    UART_HandleTypeDef *huart;
    Param *param;
    ServoCommand pending_command;
    ServoCommand active_command;
    TsbpState state;
    uint8_t node_id;
    uint8_t last_seq;
    uint8_t pdo_miss_count;
    uint16_t serial_watchdog_count_ms;
    bool pdo_seq_valid;
    bool pending_valid;
    bool tx_busy;
    bool save_request;
} TsbpContext;

void Tsbp_Init(TsbpContext *ctx, UART_HandleTypeDef *huart, Param *param);
void Tsbp_RxEventCallback(TsbpContext *ctx, const UART_HandleTypeDef *huart, uint16_t size);
void Tsbp_TxCpltCallback(TsbpContext *ctx, const UART_HandleTypeDef *huart);
void Tsbp_1msTick(TsbpContext *ctx);
const ServoCommand *Tsbp_GetActiveCommand(const TsbpContext *ctx);
bool Tsbp_ConsumeSaveRequest(TsbpContext *ctx);

#ifdef __cplusplus
}
#endif

#endif // TRIPLE_CASCADECONTROLDCMOTOR_TSBP_H
