/**
 * @file Crsf.h
 * @brief Bounded CRSF RC-channel stream decoder.
 * @brief 有界 CRSF 遥控通道流解码器。
 */

#ifndef TK_CRSF_H
#define TK_CRSF_H

#include <stdbool.h>
#include <stdint.h>

#include "TypeDefine.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CRSF_DEVICE_ADDRESS_FC          0xC8U
#define CRSF_FRAME_TYPE_RC_CHANNELS     0x16U
#define CRSF_CHANNEL_COUNT              16U
#define CRSF_CHANNEL_BITS               11U
#define CRSF_RC_PAYLOAD_SIZE            22U
#define CRSF_RC_FRAME_LENGTH            24U
#define CRSF_MAX_FRAME_SIZE             64U

typedef enum
{
    CRSF_CONTROL_DISABLED = 0,
    CRSF_CONTROL_ARM_TRACK,
    CRSF_CONTROL_ACTIVE,
    CRSF_CONTROL_ARM_FAILED
} CrsfControlState;

/**
 * @brief CRSF parser state owned by the application main context.
 * @brief 由应用主上下文持有的 CRSF 解析状态。
 */
typedef struct
{
    uint8_t frame[CRSF_MAX_FRAME_SIZE];
    uint8_t frame_size;
    uint8_t expected_size;
    uint16_t channel_buffers[2][CRSF_CHANNEL_COUNT];
    volatile uint8_t active_channel_buffer;
    uint32_t valid_frame_count;
    uint32_t crc_error_count;
    uint32_t malformed_frame_count;
    volatile bool channels_valid;
    volatile bool frame_ever_received;
    volatile uint32_t frame_generation;
    uint32_t processed_frame_generation;
    uint16_t frame_age_ms;
    Param *param;
    ServoCommand command;
    uint8_t control_state;
    bool enable_latched;
    bool enable_channel_seen_low;
    bool center_latched;
    bool center_channel_seen_low;
    bool center_reference_valid;
    uint16_t arm_elapsed_ms;
    int32_t arm_target_position;
} CrsfContext;

/** Initialize an empty decoder. / 初始化空解码器。 */
void Crsf_Init(CrsfContext *context, Param *param);

/**
 * @brief Consume one bounded UART RX chunk; complete frames may span chunks.
 * @brief 消费一段有界 UART 接收数据；完整帧允许跨越多个分段。
 */
void Crsf_ProcessBytes(CrsfContext *context, const uint8_t *data, uint16_t length);

/** Calculate CRSF CRC8 using polynomial 0xD5. / 使用 0xD5 多项式计算 CRSF CRC8。 */
uint8_t Crsf_Crc8(const uint8_t *data, uint8_t length);

/** Advance CRSF watchdog and control state by one millisecond. / 推进 CRSF 看门狗与控制状态一毫秒。 */
void Crsf_1msTick(CrsfContext *context);

/** Return the command selected by the CRSF state machine. / 返回 CRSF 状态机生成的指令。 */
const ServoCommand *Crsf_GetActiveCommand(const CrsfContext *context);

/** Read one published channel without exposing the inactive decode buffer. / 读取一个已发布通道，不暴露非活动解码缓冲。 */
uint16_t Crsf_GetChannel(const CrsfContext *context, uint8_t channel_index);

#ifdef __cplusplus
}
#endif

#endif /* TK_CRSF_H */
