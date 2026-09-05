/**
 * @file Dynamixel2Codec.h
 * @brief HAL-independent DYNAMIXEL Protocol 2.0 packet codec.
 * @brief 与 HAL 无关的 DYNAMIXEL Protocol 2.0 数据包编解码器。
 */

#ifndef TK_DYNAMIXEL2_CODEC_H
#define TK_DYNAMIXEL2_CODEC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DXL2_BROADCAST_ID       0xFEU  /**< Broadcast device ID. / 广播设备 ID。 */
#define DXL2_INST_PING          0x01U  /**< Device discovery. / 设备探测。 */
#define DXL2_INST_READ          0x02U  /**< Control Table read. / 控制表读取。 */
#define DXL2_INST_WRITE         0x03U  /**< Immediate Control Table write. / 控制表立即写入。 */
#define DXL2_INST_REG_WRITE     0x04U  /**< Deferred write committed by Action. / 由 Action 提交的延迟写入。 */
#define DXL2_INST_ACTION        0x05U  /**< Commit a registered write. / 提交已登记写入。 */
#define DXL2_INST_STATUS        0x55U  /**< Slave response packet. / 从站状态响应包。 */
#define DXL2_INST_SYNC_READ     0x82U  /**< Ordered multi-node read. / 按 ID 列表排序的多节点读取。 */
#define DXL2_INST_SYNC_WRITE    0x83U  /**< Broadcast multi-node write. / 广播多节点写入。 */
#define DXL2_INST_TK_SYNC_CONTROL 0xA0U /**< TK atomic broadcast control. / TK 原子广播控制。 */
#define DXL2_INST_TK_TIMED_READ DXL2_INST_TK_SYNC_CONTROL /**< Unicast A0 returns tick + data. / 单播 A0 返回时刻 + 数据。 */
#define DXL2_INST_TK_STREAM_SYNC 0xA1U /**< Broadcast clock sync and stream control. / 广播校时与主动上报控制。 */
#define DXL2_INST_TK_STREAM_READ 0xA2U /**< Merge one read range into a stream frame. / 将一个读取区间并入上报帧。 */
#define DXL2_MAX_PARAMETERS     160U   /**< Supports one eight-node TK control request. / 支持一个八节点 TK 控制请求。 */
#define DXL2_MAX_PACKET_SIZE    256U   /**< Covers worst-case stuffing at maximum parameters. / 覆盖最大参数包的最坏填充长度。 */
#define DXL2_FIXED_STATUS_RETURN_LEVEL 2U /**< All supported unicast instructions return Status. / 所有受支持单播指令均返回状态包。 */

#define DXL2_TK_SYNC_HEADER_SIZE       10U   /**< Common A0 request fields. / A0 请求公共字段长度。 */
#define DXL2_TK_SYNC_RECORD_SIZE       15U   /**< One node command record. / 单节点命令记录长度。 */
#define DXL2_TK_SYNC_MAX_NODES         8U    /**< V1 bounded node count. / V1 有界节点数。 */
#define DXL2_TK_ACK_SUPPORTED_MASK     0x03FFU /**< Supported optional ACK fields. / 支持的 ACK 可选字段。 */
#define DXL2_TK_EXECUTE_NEXT_UPDATE    0U    /**< Apply at the next local update. / 在下一本地控制更新点生效。 */
#define DXL2_TK_TIMED_READ_REQUEST_SIZE 3U   /**< address u16 + 7-bit data length. / 地址 u16 + 7 位数据长度。 */
#define DXL2_TK_TIMED_READ_TICK_SIZE   4U    /**< uint32 little-endian millisecond tick. / uint32 小端毫秒时刻。 */
#define DXL2_TK_STREAM_VERSION          1U
#define DXL2_TK_STREAM_SYNC_HEADER_SIZE 13U  /**< version, flags, session, tick, period, slot, count. */
#define DXL2_TK_STREAM_MAX_NODES        8U
#define DXL2_TK_STREAM_MAX_RANGES       8U
#define DXL2_TK_STREAM_FRAME_HEADER_SIZE 12U /**< marker, version, session, sequence, tick, flags, count. */
#define DXL2_TK_STREAM_SYNC_ENABLE      0x01U
#define DXL2_TK_STREAM_SYNC_CLEAR       0x02U
#define DXL2_TK_STREAM_READ_REPLACE     0x01U
#define DXL2_TK_STREAM_FLAG_OVERWRITE   0x01U
#define DXL2_TK_STREAM_STATUS_MARKER     0xA3U

/**
 * @brief Standard DYNAMIXEL Protocol 2.0 Status Packet error numbers.
 * @brief 标准 DYNAMIXEL Protocol 2.0 状态包错误编号。
 *
 * Bit 7 is the hardware-alert flag; bits 0..6 contain one of these values.
 * bit7 是硬件告警标志；bit0..6 保存下列错误编号之一。
 */
typedef enum
{
    DXL2_STATUS_ERROR_NONE = 0x00,
    DXL2_STATUS_ERROR_RESULT_FAIL = 0x01,
    DXL2_STATUS_ERROR_INSTRUCTION = 0x02,
    DXL2_STATUS_ERROR_CRC = 0x03,
    DXL2_STATUS_ERROR_DATA_RANGE = 0x04,
    DXL2_STATUS_ERROR_DATA_LENGTH = 0x05,
    DXL2_STATUS_ERROR_DATA_LIMIT = 0x06,
    DXL2_STATUS_ERROR_ACCESS = 0x07
} Dxl2StatusError;

#define DXL2_STATUS_ALERT_MASK 0x80U /**< Hardware alert bit. / 硬件告警位。 */

typedef enum
{
    DXL2_DECODE_OK = 0,       /**< One complete valid packet. / 已得到一个完整有效包。 */
    DXL2_DECODE_INCOMPLETE,   /**< Keep buffered bytes and wait. / 保留缓冲数据并继续等待。 */
    DXL2_DECODE_NO_HEADER,    /**< Noise precedes or replaces the header. / 包头前存在噪声或无包头。 */
    DXL2_DECODE_BAD_LENGTH,   /**< Length field is outside local limits. / Length 超出本机限制。 */
    DXL2_DECODE_BAD_ID,       /**< ID is neither unicast nor broadcast. / ID 既非单播也非广播。 */
    DXL2_DECODE_BAD_CRC,      /**< CRC-16 validation failed. / CRC-16 校验失败。 */
    DXL2_DECODE_BAD_BODY      /**< Stuffed body cannot be decoded. / 填充后的包体无法解码。 */
} Dxl2DecodeStatus;

typedef struct
{
    uint8_t id;                               /**< Packet ID. / 数据包 ID。 */
    uint8_t instruction;                      /**< Instruction or Status (0x55). / 指令码或状态码 0x55。 */
    uint8_t error;                            /**< Status error byte; zero for instructions. / 状态错误字节；指令包为零。 */
    uint16_t parameter_length;                /**< Unstuffed parameter bytes. / 去填充后的参数字节数。 */
    uint8_t parameters[DXL2_MAX_PARAMETERS];  /**< Unstuffed parameters. / 去填充后的参数数据。 */
} Dxl2Packet;

typedef struct
{
    Dxl2DecodeStatus status;  /**< Decode result. / 解码结果。 */
    uint16_t consumed;        /**< Bytes safe to remove from the stream. / 可从流缓冲区移除的字节数。 */
    Dxl2Packet packet;        /**< Valid only when status is OK. / 仅在状态为 OK 时有效。 */
} Dxl2DecodeResult;

/**
 * @brief Validated view of one TK Sync Control request. / 一个已校验 TK 同步控制请求的只读视图。
 * @note record points into the decoded packet and is valid only while that packet lives.
 * @note record 指向已解码数据包，其有效期不超过该数据包。
 */
typedef struct
{
    uint16_t sequence;
    uint8_t execute_mode;
    uint32_t execute_value;
    uint16_t ack_mask;
    uint8_t node_count;
    uint8_t reply_index;
    const uint8_t *record;
} Dxl2TkSyncControlView;

typedef enum
{
    DXL2_TK_SYNC_INVALID = 0, /**< Global structure error: discard without reply. / 全局结构错误：丢弃且不回复。 */
    DXL2_TK_SYNC_NOT_TARGETED, /**< Valid request without this node. / 请求有效但不含本节点。 */
    DXL2_TK_SYNC_TARGETED /**< Valid request containing this node. / 请求有效且包含本节点。 */
} Dxl2TkSyncParseStatus;

/**
 * @brief Validate an A0 request and locate one node record. / 校验 A0 请求并定位一个节点记录。
 * @note Duplicate IDs, unsupported ACK mask bits, and length mismatches invalidate the whole request.
 * @note ID 重复、ACK Mask 保留位置位或长度不匹配都会使整包无效。
 */
Dxl2TkSyncParseStatus Dxl2_ParseTkSyncControl(const Dxl2Packet *packet,
                                               uint8_t node_id,
                                               Dxl2TkSyncControlView *view);

/**
 * @brief Return whether this firmware must emit a Status Packet for an instruction.
 * @brief 判断本固件是否必须为该指令发送状态包。
 *
 * Supported unicast request/response instructions reply. One-way stream
 * configuration instructions do not reply. Broadcast replies are limited
 * to Ping and Sync Read; unsupported Bulk Read is intentionally not included.
 * 受支持单播指令始终回复；广播仅 Ping 和 Sync Read 回复，未实现的 Bulk Read 不计入。
 */
bool Dxl2_ShouldReturnStatus(uint8_t packet_id, uint8_t instruction);

/**
 * @brief Update CRC-16 using DYNAMIXEL polynomial 0x8005. / 使用 DYNAMIXEL 多项式 0x8005 更新 CRC-16。
 * @param accumulator Previous CRC; pass zero for a new packet. / 上一段 CRC；新数据包传零。
 * @param data Input bytes; NULL leaves the accumulator unchanged. / 输入字节；NULL 时保持原 CRC。
 * @param length Input byte count. / 输入字节数。
 * @return Updated CRC value. / 更新后的 CRC 值。
 */
uint16_t Dxl2_UpdateCrc(uint16_t accumulator, const uint8_t *data, uint16_t length);

/**
 * @brief Encode one instruction packet with Byte Stuffing. / 编码一个带 Byte Stuffing 的指令包。
 * @param output Caller-owned encoded packet buffer. / 调用者持有的编码输出缓冲区。
 * @param capacity Output buffer capacity. / 输出缓冲区容量。
 * @param id Target unicast ID or 0xFE broadcast ID. / 目标单播 ID 或 0xFE 广播 ID。
 * @param instruction Standard instruction code; Status is rejected. / 标准指令码；拒绝 Status。
 * @param parameters Unstuffed parameters, or NULL when length is zero. / 未填充参数；长度为零时可为 NULL。
 * @param parameter_length Parameter byte count. / 参数字节数。
 * @return Encoded length, or zero on invalid input/capacity. / 编码长度；输入或容量无效时返回零。
 */
uint16_t Dxl2_EncodeInstruction(uint8_t *output, uint16_t capacity, uint8_t id,
                                uint8_t instruction, const uint8_t *parameters,
                                uint16_t parameter_length);

/**
 * @brief Encode one status packet with Byte Stuffing. / 编码一个带 Byte Stuffing 的状态包。
 * @param output Caller-owned encoded packet buffer. / 调用者持有的编码输出缓冲区。
 * @param capacity Output buffer capacity. / 输出缓冲区容量。
 * @param id Responding device ID. / 响应设备 ID。
 * @param error DYNAMIXEL status error byte. / DYNAMIXEL 状态错误字节。
 * @param parameters Unstuffed response data. / 未填充的响应数据。
 * @param parameter_length Response data byte count. / 响应数据字节数。
 * @return Encoded length, or zero on invalid input/capacity. / 编码长度；输入或容量无效时返回零。
 */
uint16_t Dxl2_EncodeStatus(uint8_t *output, uint16_t capacity, uint8_t id,
                           uint8_t error, const uint8_t *parameters,
                           uint16_t parameter_length);

/**
 * @brief Decode at most one packet from a stream buffer. / 从流缓冲区最多解码一个数据包。
 * @param input Current stream bytes; ownership remains with the caller. / 当前流数据；所有权仍归调用者。
 * @param length Available byte count. / 当前可用字节数。
 * @return Decode status, removable byte count, and packet when complete. / 返回状态、可移除字节数及完整包。
 */
Dxl2DecodeResult Dxl2_DecodePacket(const uint8_t *input, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* TK_DYNAMIXEL2_CODEC_H */
