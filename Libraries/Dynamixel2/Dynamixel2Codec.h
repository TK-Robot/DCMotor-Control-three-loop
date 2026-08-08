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
#define DXL2_MAX_PARAMETERS     96U    /**< Unstuffed parameter capacity. / 去填充后的参数容量。 */
#define DXL2_MAX_PACKET_SIZE    128U   /**< Maximum encoded packet size. / 最大编码包长度。 */

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
