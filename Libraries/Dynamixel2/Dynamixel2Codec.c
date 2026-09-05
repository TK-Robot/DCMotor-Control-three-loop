/**
 * @file Dynamixel2Codec.c
 * @brief HAL-independent DYNAMIXEL Protocol 2.0 packet codec.
 * @brief 与 HAL 无关的 DYNAMIXEL Protocol 2.0 数据包编解码器。
 */

#include "Dynamixel2Codec.h"

#include <string.h>

static const uint8_t Dxl2_Header[4] = {0xFFU, 0xFFU, 0xFDU, 0x00U};

static uint16_t Dxl2_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void Dxl2_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8);
}

static uint32_t Dxl2_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8)
           | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

Dxl2TkSyncParseStatus Dxl2_ParseTkSyncControl(const Dxl2Packet *packet,
                                               uint8_t node_id,
                                               Dxl2TkSyncControlView *view)
{
    uint8_t count;
    uint8_t index;
    uint8_t other;
    const uint8_t *records;

    if (packet == NULL || view == NULL || packet->id != DXL2_BROADCAST_ID
        || packet->instruction != DXL2_INST_TK_SYNC_CONTROL
        || packet->parameter_length < DXL2_TK_SYNC_HEADER_SIZE)
    {
        return DXL2_TK_SYNC_INVALID;
    }
    count = packet->parameters[9];
    if (count == 0U || count > DXL2_TK_SYNC_MAX_NODES
        || packet->parameter_length
               != (uint16_t)(DXL2_TK_SYNC_HEADER_SIZE
                             + (uint16_t)count * DXL2_TK_SYNC_RECORD_SIZE)
        || (Dxl2_ReadU16(&packet->parameters[7])
            & (uint16_t)~DXL2_TK_ACK_SUPPORTED_MASK) != 0U)
    {
        return DXL2_TK_SYNC_INVALID;
    }

    records = &packet->parameters[DXL2_TK_SYNC_HEADER_SIZE];
    for (index = 0U; index < count; ++index)
    {
        uint8_t id = records[(uint16_t)index * DXL2_TK_SYNC_RECORD_SIZE];
        if (id == 0U || id > 0xFCU)
        {
            return DXL2_TK_SYNC_INVALID;
        }
        for (other = 0U; other < index; ++other)
        {
            if (id == records[(uint16_t)other * DXL2_TK_SYNC_RECORD_SIZE])
            {
                return DXL2_TK_SYNC_INVALID;
            }
        }
    }

    memset(view, 0, sizeof(*view));
    view->sequence = Dxl2_ReadU16(&packet->parameters[0]);
    view->execute_mode = packet->parameters[2];
    view->execute_value = Dxl2_ReadU32(&packet->parameters[3]);
    view->ack_mask = Dxl2_ReadU16(&packet->parameters[7]);
    view->node_count = count;
    for (index = 0U; index < count; ++index)
    {
        const uint8_t *record = &records[(uint16_t)index * DXL2_TK_SYNC_RECORD_SIZE];
        if (record[0] == node_id)
        {
            view->reply_index = index;
            view->record = record;
            return DXL2_TK_SYNC_TARGETED;
        }
    }
    return DXL2_TK_SYNC_NOT_TARGETED;
}

bool Dxl2_ShouldReturnStatus(uint8_t packet_id, uint8_t instruction)
{
    if (packet_id == DXL2_BROADCAST_ID)
    {
        return instruction == DXL2_INST_PING || instruction == DXL2_INST_SYNC_READ;
    }
    return packet_id <= 0xFCU
           && instruction != DXL2_INST_STATUS
           && instruction != DXL2_INST_TK_STREAM_SYNC
           && instruction != DXL2_INST_TK_STREAM_READ;
}

uint16_t Dxl2_UpdateCrc(uint16_t accumulator, const uint8_t *data, uint16_t length)
{
    uint16_t index;
    uint8_t bit;

    if (data == NULL)
    {
        return accumulator;
    }
    for (index = 0U; index < length; ++index)
    {
        accumulator ^= (uint16_t)data[index] << 8;
        for (bit = 0U; bit < 8U; ++bit)
        {
            accumulator = ((accumulator & 0x8000U) != 0U)
                              ? (uint16_t)((accumulator << 1) ^ 0x8005U)
                              : (uint16_t)(accumulator << 1);
        }
    }
    return accumulator;
}

static uint16_t Dxl2_StuffBody(uint8_t *output, uint16_t capacity,
                               const uint8_t *body, uint16_t body_length)
{
    uint16_t source;
    uint16_t destination = 0U;

    /* Stuff only the instruction/error/parameter body; header, ID, Length, and CRC are excluded. */
    /* 仅填充指令、错误字节和参数包体；包头、ID、Length 与 CRC 不参与填充。 */
    for (source = 0U; source < body_length; ++source)
    {
        if (destination >= capacity)
        {
            return 0U;
        }
        output[destination++] = body[source];
        if (destination >= 3U && output[destination - 3U] == 0xFFU
            && output[destination - 2U] == 0xFFU
            && output[destination - 1U] == 0xFDU)
        {
            if (destination >= capacity)
            {
                return 0U;
            }
            output[destination++] = 0xFDU;
        }
    }
    return destination;
}

static uint16_t Dxl2_UnstuffBody(uint8_t *output, uint16_t capacity,
                                 const uint8_t *body, uint16_t body_length)
{
    uint16_t source;
    uint16_t destination = 0U;

    /* Remove exactly one inserted 0xFD after each FF FF FD sequence. */
    /* 每遇到 FF FF FD 序列，仅移除其后协议插入的一个 0xFD。 */
    for (source = 0U; source < body_length; ++source)
    {
        if (destination >= capacity)
        {
            return 0U;
        }
        output[destination++] = body[source];
        if (destination >= 3U && source + 1U < body_length
            && output[destination - 3U] == 0xFFU
            && output[destination - 2U] == 0xFFU
            && output[destination - 1U] == 0xFDU
            && body[source + 1U] == 0xFDU)
        {
            ++source;
        }
    }
    return destination;
}

static uint16_t Dxl2_Encode(uint8_t *output, uint16_t capacity, uint8_t id,
                            const uint8_t *body, uint16_t body_length)
{
    uint16_t stuffed_length;
    uint16_t total_length;
    uint16_t crc;

    if (output == NULL || body == NULL || body_length == 0U
        || (id > 0xFCU && id != DXL2_BROADCAST_ID) || capacity < 10U)
    {
        return 0U;
    }

    memcpy(output, Dxl2_Header, sizeof(Dxl2_Header));
    output[4] = id;
    stuffed_length = Dxl2_StuffBody(&output[7], (uint16_t)(capacity - 9U),
                                    body, body_length);
    if (stuffed_length == 0U)
    {
        return 0U;
    }
    total_length = (uint16_t)(7U + stuffed_length + 2U);
    Dxl2_WriteU16(&output[5], (uint16_t)(stuffed_length + 2U));
    crc = Dxl2_UpdateCrc(0U, output, (uint16_t)(total_length - 2U));
    Dxl2_WriteU16(&output[total_length - 2U], crc);
    return total_length;
}

uint16_t Dxl2_EncodeInstruction(uint8_t *output, uint16_t capacity, uint8_t id,
                                uint8_t instruction, const uint8_t *parameters,
                                uint16_t parameter_length)
{
    uint8_t body[DXL2_MAX_PARAMETERS + 1U];

    if (instruction == DXL2_INST_STATUS || parameter_length > DXL2_MAX_PARAMETERS
        || (parameter_length != 0U && parameters == NULL))
    {
        return 0U;
    }
    body[0] = instruction;
    if (parameter_length != 0U)
    {
        memcpy(&body[1], parameters, parameter_length);
    }
    return Dxl2_Encode(output, capacity, id, body, (uint16_t)(parameter_length + 1U));
}

uint16_t Dxl2_EncodeStatus(uint8_t *output, uint16_t capacity, uint8_t id,
                           uint8_t error, const uint8_t *parameters,
                           uint16_t parameter_length)
{
    uint8_t body[DXL2_MAX_PARAMETERS + 2U];

    if (parameter_length > DXL2_MAX_PARAMETERS
        || (parameter_length != 0U && parameters == NULL))
    {
        return 0U;
    }
    body[0] = DXL2_INST_STATUS;
    body[1] = error;
    if (parameter_length != 0U)
    {
        memcpy(&body[2], parameters, parameter_length);
    }
    return Dxl2_Encode(output, capacity, id, body, (uint16_t)(parameter_length + 2U));
}

Dxl2DecodeResult Dxl2_DecodePacket(const uint8_t *input, uint16_t length)
{
    Dxl2DecodeResult result;
    uint16_t offset;
    uint16_t packet_length;
    uint16_t total_length;
    uint16_t expected_crc;
    uint16_t actual_crc;
    uint8_t body[DXL2_MAX_PARAMETERS + 2U];
    uint16_t body_length;

    memset(&result, 0, sizeof(result));
    result.status = DXL2_DECODE_INCOMPLETE;
    if (input == NULL || length < 4U)
    {
        return result;
    }

    /* Report removable leading noise instead of discarding the final three possible header bytes. */
    /* 报告可移除的前导噪声，同时保留末尾三个可能属于下一包包头的字节。 */
    for (offset = 0U; offset + 4U <= length; ++offset)
    {
        if (input[offset] == Dxl2_Header[0]
            && input[offset + 1U] == Dxl2_Header[1]
            && input[offset + 2U] == Dxl2_Header[2]
            && input[offset + 3U] == Dxl2_Header[3])
        {
            break;
        }
    }
    if (offset + 4U > length)
    {
        result.status = DXL2_DECODE_NO_HEADER;
        result.consumed = (length > 3U) ? (uint16_t)(length - 3U) : 0U;
        return result;
    }
    if (offset != 0U)
    {
        result.status = DXL2_DECODE_NO_HEADER;
        result.consumed = offset;
        return result;
    }
    if (length < 7U)
    {
        return result;
    }
    if (input[4] > 0xFCU && input[4] != DXL2_BROADCAST_ID)
    {
        result.status = DXL2_DECODE_BAD_ID;
        result.consumed = 4U;
        return result;
    }

    packet_length = Dxl2_ReadU16(&input[5]);
    total_length = (uint16_t)(7U + packet_length);
    if (packet_length < 3U || total_length > DXL2_MAX_PACKET_SIZE)
    {
        result.status = DXL2_DECODE_BAD_LENGTH;
        result.consumed = 4U;
        return result;
    }
    if (length < total_length)
    {
        return result;
    }

    /* CRC covers the stuffed on-wire bytes from the first header byte through parameters. */
    /* CRC 覆盖线上填充后的数据，从包头首字节一直到参数末尾。 */
    expected_crc = Dxl2_ReadU16(&input[total_length - 2U]);
    actual_crc = Dxl2_UpdateCrc(0U, input, (uint16_t)(total_length - 2U));
    if (expected_crc != actual_crc)
    {
        result.status = DXL2_DECODE_BAD_CRC;
        result.consumed = total_length;
        return result;
    }

    body_length = Dxl2_UnstuffBody(body, sizeof(body), &input[7],
                                   (uint16_t)(packet_length - 2U));
    if (body_length == 0U)
    {
        result.status = DXL2_DECODE_BAD_BODY;
        result.consumed = total_length;
        return result;
    }
    result.packet.id = input[4];
    result.packet.instruction = body[0];
    if (body[0] == DXL2_INST_STATUS)
    {
        if (body_length < 2U)
        {
            result.status = DXL2_DECODE_BAD_BODY;
            result.consumed = total_length;
            return result;
        }
        result.packet.error = body[1];
        result.packet.parameter_length = (uint16_t)(body_length - 2U);
        memcpy(result.packet.parameters, &body[2], result.packet.parameter_length);
    }
    else
    {
        result.packet.parameter_length = (uint16_t)(body_length - 1U);
        memcpy(result.packet.parameters, &body[1], result.packet.parameter_length);
    }
    result.status = DXL2_DECODE_OK;
    result.consumed = total_length;
    return result;
}
