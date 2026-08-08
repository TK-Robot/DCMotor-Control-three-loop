/**
 * @file Dynamixel2ControlTable.h
 * @brief Public DYNAMIXEL 2.0 Control Table contract for the TK servo.
 * @brief TK 伺服 DYNAMIXEL 2.0 控制表公开契约。
 */

#ifndef TK_DYNAMIXEL2_CONTROL_TABLE_H
#define TK_DYNAMIXEL2_CONTROL_TABLE_H

#include <stdint.h>

/** Multi-byte Control Table values use little-endian byte order. / 控制表多字节值使用小端序。 */
#define DXL2_CONTROL_TABLE_SIZE             153U
#define DXL2_ENCODER_COUNTS_PER_REV         16384L
#define DXL2_PID_GAIN_SCALE                 1000L
#define DXL2_VELOCITY_PID_I_SCALE           10000L
#define DXL2_DRIVE_OUTPUT_FULL_SCALE        1000L
#define DXL2_EXECUTE_TICK_MAX_FUTURE_MS     0x7FFFFFFFUL
#define DXL2_STATUS_PACKET_OVERHEAD_BYTES   11U
#define DXL2_UART_BITS_PER_BYTE             10U
#define DXL2_REPLY_GUARD_US                 50U
#define DXL2_REPLY_SLOT_MIN_US              50U
#define DXL2_REPLY_SLOT_MAX_US              8000U
#define DXL2_LEGACY_COMMAND_IMAGE_SIZE      14U
#define DXL2_FULL_COMMAND_IMAGE_SIZE        20U
#define DXL2_ACK_FEEDBACK_IMAGE_SIZE        27U

/**
 * @brief Control Table start addresses. / 控制表起始地址。
 */
typedef enum
{
    DXL2_ADDR_MODEL_NUMBER = 0,
    DXL2_ADDR_FIRMWARE_VERSION = 2,
    DXL2_ADDR_PROTOCOL_VERSION = 3,
    DXL2_ADDR_NODE_ID = 4,
    DXL2_ADDR_BAUD_CODE = 5,
    DXL2_ADDR_SERIAL_WATCHDOG_MS = 6,
    DXL2_ADDR_NODE_POSITION = 8,
    DXL2_ADDR_REPLY_SLOT_US = 9,

    DXL2_ADDR_CONTROL_SOURCE = 16,
    DXL2_ADDR_SERVO_MODE = 17,
    DXL2_ADDR_CONTROL_WORD = 18,
    DXL2_ADDR_TARGET_CURRENT_MA = 20,
    DXL2_ADDR_TARGET_VELOCITY_CPS = 22,
    DXL2_ADDR_TARGET_POSITION_COUNT = 26,
    DXL2_ADDR_EXECUTE_TICK_MS = 30,
    DXL2_ADDR_COMMAND_SEQUENCE = 34,
    DXL2_ADDR_APPLIED_SEQUENCE = 36,
    DXL2_ADDR_LAST_COMMAND_RESULT = 38,
    DXL2_ADDR_RESERVED_39 = 39,

    DXL2_ADDR_STATUS_WORD = 40,
    DXL2_ADDR_FAULT_CODE = 42,
    DXL2_ADDR_ACTUAL_CURRENT_MA = 44,
    DXL2_ADDR_ACTUAL_VELOCITY_CPS = 46,
    DXL2_ADDR_ACTUAL_POSITION_COUNT = 50,
    DXL2_ADDR_MULTI_TURN_POSITION_COUNT = 54,
    DXL2_ADDR_DRIVE_OUTPUT_PERMILLE = 58,
    DXL2_ADDR_SUPPLY_VOLTAGE_MV = 60,
    DXL2_ADDR_TEMPERATURE_C = 62,

    DXL2_ADDR_CURRENT_PID = 64,
    DXL2_ADDR_VELOCITY_PID = 80,
    DXL2_ADDR_POSITION_PID = 96,
    DXL2_ADDR_TEMPERATURE_LIMIT_C = 112,
    DXL2_ADDR_SPEED_LIMIT_CPS = 114,
    DXL2_ADDR_PWM_MODE = 116,
    DXL2_ADDR_ENCODER_DIRECTION = 117,
    DXL2_ADDR_ENCODER_OFFSET_COUNT = 118,
    DXL2_ADDR_FAIL_SAFE_POLICY = 120,
    DXL2_ADDR_CURRENT_TICK_MS = 122,
    DXL2_ADDR_PWM_INPUT_LOW_US = 126,

    DXL2_ADDR_LAST_DIAGNOSTIC = 128,
    DXL2_ADDR_DIAGNOSTIC_COUNT = 130,
    DXL2_ADDR_UART_ERROR_COUNT = 134,
    DXL2_ADDR_CRC_ERROR_COUNT = 138,
    DXL2_ADDR_BAD_PACKET_COUNT = 142,
    DXL2_ADDR_RX_PACKET_COUNT = 146,
    DXL2_ADDR_CLEAR_DIAGNOSTICS = 150,
    DXL2_ADDR_SAVE_NVM = 152
} Dynamixel2ControlTableAddress;

/**
 * @brief Control Word bit masks. Reserved bits must be written as zero.
 * @brief Control Word 位掩码；保留位写入时必须为零。
 */
typedef enum
{
    DXL2_CONTROL_ENABLE = (1U << 0),
    DXL2_CONTROL_USE_EXECUTE_TICK = (1U << 1),
    DXL2_CONTROL_CLEAR_FAULT = (1U << 2)
} Dynamixel2ControlMask;

/** TK control ACK optional fields, serialized from bit 0 through bit 9. */
/** TK 控制 ACK 可选字段，严格按 bit0 到 bit9 顺序序列化。 */
typedef enum
{
    DXL2_ACK_STATUS_WORD = (1U << 0),
    DXL2_ACK_FAULT_CODE = (1U << 1),
    DXL2_ACK_ACTUAL_CURRENT = (1U << 2),
    DXL2_ACK_ACTUAL_VELOCITY = (1U << 3),
    DXL2_ACK_ACTUAL_POSITION = (1U << 4),
    DXL2_ACK_MULTI_TURN_POSITION = (1U << 5),
    DXL2_ACK_DRIVE_OUTPUT = (1U << 6),
    DXL2_ACK_SUPPLY_VOLTAGE = (1U << 7),
    DXL2_ACK_TEMPERATURE = (1U << 8),
    DXL2_ACK_CURRENT_TICK = (1U << 9)
} Dynamixel2AckMask;

typedef enum
{
    DXL2_ACK_ACCEPTED_VALID = (1U << 0),
    DXL2_ACK_APPLIED_VALID = (1U << 1),
    DXL2_ACK_PENDING = (1U << 2),
    DXL2_ACK_DUPLICATE = (1U << 3)
} Dynamixel2AckStateMask;

/**
 * @brief Status Word bit masks. Reserved bits read as zero.
 * @brief Status Word 位掩码；保留位读取为零。
 */
typedef enum
{
    DXL2_STATUS_READY = (1U << 0),
    DXL2_STATUS_PWM_INPUT_VALID = (1U << 1),
    DXL2_STATUS_OUTPUT_ENABLED = (1U << 2),
    DXL2_STATUS_FAULT_PRESENT = (1U << 3),
    DXL2_STATUS_PROTECTION_INHIBIT = (1U << 4),
    DXL2_STATUS_UNDERVOLTAGE = (1U << 5),
    DXL2_STATUS_OVERTEMPERATURE = (1U << 6),
    DXL2_STATUS_PWM_SOURCE = (1U << 8),
    DXL2_STATUS_SERIAL_SOURCE = (1U << 9),
    DXL2_STATUS_FAULT_FREE = (1U << 11),
    DXL2_STATUS_PROTOCOL_ACTIVE = (1U << 12)
} Dynamixel2StatusMask;

/**
 * @brief Latched drive fault codes exposed at address 42.
 * @brief 地址 42 对外提供的锁存驱动故障码。
 */
typedef enum
{
    DXL2_FAULT_NONE = 0x0000,
    DXL2_FAULT_SERIAL_WATCHDOG = 0x000A
} Dynamixel2FaultCode;

/**
 * @brief Communication diagnostic codes exposed at address 128.
 * @brief 地址 128 对外提供的通信诊断码。
 */
typedef enum
{
    DXL2_DIAG_NONE = 0,
    DXL2_DIAG_UART_ERROR = 1,
    DXL2_DIAG_RX_CRC = 2,
    DXL2_DIAG_RX_BAD_PACKET = 3,
    DXL2_DIAG_TX_DROP = 4,
    DXL2_DIAG_WATCHDOG = 5
} Dynamixel2DiagnosticCode;

/**
 * @brief Return whether an absolute Execute Tick is due, including uint32 wraparound.
 * @brief 判断绝对 Execute Tick 是否到期，并正确处理 uint32 回绕。
 * @note The scheduled future distance must not exceed 0x7fffffff ms. / 未来调度距离不得超过 0x7fffffff ms。
 */
static inline uint8_t Dynamixel2_IsExecuteTickDue(uint32_t current_tick,
                                                  uint32_t execute_tick)
{
    return (uint8_t)((int32_t)(current_tick - execute_tick) >= 0);
}

#endif /* TK_DYNAMIXEL2_CONTROL_TABLE_H */
