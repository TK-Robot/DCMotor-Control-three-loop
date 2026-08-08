#include "Dynamixel2Codec.h"
#include "Dynamixel2ControlTable.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

static void check(bool condition, const char *message)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

static void test_official_ping(void)
{
    static const uint8_t expected[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01,
                                       0x03, 0x00, 0x01, 0x19, 0x4E};
    uint8_t encoded[32];
    uint16_t length = Dxl2_EncodeInstruction(encoded, sizeof(encoded), 1U,
                                             DXL2_INST_PING, NULL, 0U);
    Dxl2DecodeResult decoded = Dxl2_DecodePacket(expected, sizeof(expected));
    check(length == sizeof(expected) && memcmp(encoded, expected, sizeof(expected)) == 0,
          "official ID 1 Ping vector");
    check(decoded.status == DXL2_DECODE_OK && decoded.packet.id == 1U
              && decoded.packet.instruction == DXL2_INST_PING,
          "official Ping decode");
}

static void test_stuffing_and_stream(void)
{
    const uint8_t parameters[] = {20U, 0U, 0xFFU, 0xFFU, 0xFDU, 1U};
    uint8_t encoded[64];
    uint16_t length = Dxl2_EncodeInstruction(encoded, sizeof(encoded), 2U,
                                             DXL2_INST_WRITE, parameters,
                                             sizeof(parameters));
    Dxl2DecodeResult decoded = Dxl2_DecodePacket(encoded, length);
    check(decoded.status == DXL2_DECODE_OK
              && decoded.packet.parameter_length == sizeof(parameters)
              && memcmp(decoded.packet.parameters, parameters, sizeof(parameters)) == 0,
          "Byte Stuffing round trip");
    check(Dxl2_DecodePacket(encoded, (uint16_t)(length - 1U)).status
              == DXL2_DECODE_INCOMPLETE,
          "partial packet waits");
    encoded[length - 1U] ^= 1U;
    check(Dxl2_DecodePacket(encoded, length).status == DXL2_DECODE_BAD_CRC,
          "bad CRC rejected");
}

static void test_status_and_sync_instructions(void)
{
    const uint8_t status_parameters[] = {0x34U, 0x12U, 0x78U, 0x56U};
    const uint8_t sync_read_parameters[] = {40U, 0U, 14U, 0U, 1U, 2U, 3U};
    const uint8_t sync_write_parameters[] = {16U, 0U, 14U, 0U,
                                              1U, 1U, 2U, 0U, 0U, 0U, 0U,
                                              0U, 0U, 0U, 0U, 0U, 0U, 0U,
                                              0U, 0U};
    uint8_t encoded[96];
    uint16_t length;
    Dxl2DecodeResult decoded;

    length = Dxl2_EncodeStatus(encoded, sizeof(encoded), 3U,
                              DXL2_STATUS_ERROR_DATA_RANGE,
                              status_parameters, sizeof(status_parameters));
    decoded = Dxl2_DecodePacket(encoded, length);
    check(decoded.status == DXL2_DECODE_OK
              && decoded.packet.instruction == DXL2_INST_STATUS
              && decoded.packet.id == 3U
              && decoded.packet.error == DXL2_STATUS_ERROR_DATA_RANGE
              && decoded.packet.parameter_length == sizeof(status_parameters)
              && memcmp(decoded.packet.parameters, status_parameters,
                        sizeof(status_parameters)) == 0,
          "status packet round trip");

    length = Dxl2_EncodeInstruction(encoded, sizeof(encoded), DXL2_BROADCAST_ID,
                                    DXL2_INST_SYNC_READ, sync_read_parameters,
                                    sizeof(sync_read_parameters));
    decoded = Dxl2_DecodePacket(encoded, length);
    check(decoded.status == DXL2_DECODE_OK
              && decoded.packet.instruction == DXL2_INST_SYNC_READ
              && decoded.packet.id == DXL2_BROADCAST_ID
              && decoded.packet.parameter_length == sizeof(sync_read_parameters),
          "Sync Read instruction round trip");

    length = Dxl2_EncodeInstruction(encoded, sizeof(encoded), DXL2_BROADCAST_ID,
                                    DXL2_INST_SYNC_WRITE, sync_write_parameters,
                                    sizeof(sync_write_parameters));
    decoded = Dxl2_DecodePacket(encoded, length);
    check(decoded.status == DXL2_DECODE_OK
              && decoded.packet.instruction == DXL2_INST_SYNC_WRITE
              && decoded.packet.parameter_length == sizeof(sync_write_parameters),
          "Sync Write instruction round trip");
}

static void test_control_table_contract(void)
{
    check(DXL2_CONTROL_TABLE_SIZE == 153U, "Control Table size");
    check(DXL2_FIXED_STATUS_RETURN_LEVEL == 2U,
          "fixed Status Return Level");
    check(DXL2_STATUS_ERROR_NONE == 0x00U
              && DXL2_STATUS_ERROR_RESULT_FAIL == 0x01U
              && DXL2_STATUS_ERROR_INSTRUCTION == 0x02U
              && DXL2_STATUS_ERROR_CRC == 0x03U
              && DXL2_STATUS_ERROR_DATA_RANGE == 0x04U
              && DXL2_STATUS_ERROR_DATA_LENGTH == 0x05U
              && DXL2_STATUS_ERROR_DATA_LIMIT == 0x06U
              && DXL2_STATUS_ERROR_ACCESS == 0x07U
              && DXL2_STATUS_ALERT_MASK == 0x80U,
          "standard Status Packet error values");
    check(DXL2_ADDR_COMMAND_SEQUENCE == 34U
              && DXL2_ADDR_APPLIED_SEQUENCE == 36U
              && DXL2_ADDR_LAST_COMMAND_RESULT == 38U
              && DXL2_ADDR_RESERVED_39 == 39U,
          "periodic command acknowledgement addresses");
    check(DXL2_LEGACY_COMMAND_IMAGE_SIZE == 14U
              && DXL2_FULL_COMMAND_IMAGE_SIZE == 20U
              && DXL2_ACK_FEEDBACK_IMAGE_SIZE == 27U,
          "command and acknowledgement image sizes");
    check(DXL2_STATUS_PACKET_OVERHEAD_BYTES == 11U
              && DXL2_UART_BITS_PER_BYTE == 10U
              && DXL2_REPLY_GUARD_US == 50U
              && DXL2_REPLY_SLOT_MIN_US == 50U,
          "reply slot wire-time contract");
    check(DXL2_ADDR_CURRENT_TICK_MS == 122U, "Current Tick address");
    check(DXL2_ADDR_PWM_INPUT_LOW_US == 126U, "PWM input observation address");
    check(DXL2_STATUS_READY == 0x0001U
              && DXL2_STATUS_PWM_INPUT_VALID == 0x0002U
              && DXL2_STATUS_OUTPUT_ENABLED == 0x0004U
              && DXL2_STATUS_FAULT_PRESENT == 0x0008U
              && DXL2_STATUS_PROTECTION_INHIBIT == 0x0010U
              && DXL2_STATUS_UNDERVOLTAGE == 0x0020U
              && DXL2_STATUS_OVERTEMPERATURE == 0x0040U,
          "Status Word low bits");
    check(DXL2_STATUS_PWM_SOURCE == 0x0100U
              && DXL2_STATUS_SERIAL_SOURCE == 0x0200U
              && DXL2_STATUS_FAULT_FREE == 0x0800U
              && DXL2_STATUS_PROTOCOL_ACTIVE == 0x1000U,
          "Status Word source and health bits");
    check(DXL2_FAULT_NONE == 0x0000U
              && DXL2_FAULT_SERIAL_WATCHDOG == 0x000AU,
          "Fault Code values");
    check(DXL2_DIAG_WATCHDOG == 5U, "Last Diagnostic values");
    check(DXL2_ENCODER_COUNTS_PER_REV == 16384L
              && DXL2_PID_GAIN_SCALE == 1000L
              && DXL2_VELOCITY_PID_I_SCALE == 10000L
              && DXL2_DRIVE_OUTPUT_FULL_SCALE == 1000L,
          "published engineering scales");
    check(DXL2_CONTROL_ENABLE == 0x0001U
              && DXL2_CONTROL_USE_EXECUTE_TICK == 0x0002U
              && DXL2_CONTROL_CLEAR_FAULT == 0x0004U,
          "Control Word bits");
    check(!Dynamixel2_IsExecuteTickDue(0xFFFFFFF0U, 0U),
          "zero Execute Tick remains a future wrapped instant");
    check(Dynamixel2_IsExecuteTickDue(0U, 0U),
          "zero Execute Tick is due only at the wrapped deadline");
    check(!Dynamixel2_IsExecuteTickDue(0xFFFFFFF0U, 10U),
          "future Execute Tick across wrap waits");
    check(Dynamixel2_IsExecuteTickDue(10U, 10U),
          "Execute Tick at wrapped deadline is due");
    check(Dynamixel2_IsExecuteTickDue(20U, 10U),
          "past Execute Tick is due");
}

static void test_response_policy(void)
{
    check(Dxl2_ShouldReturnStatus(1U, DXL2_INST_PING)
              && Dxl2_ShouldReturnStatus(1U, DXL2_INST_READ)
              && Dxl2_ShouldReturnStatus(1U, DXL2_INST_WRITE)
              && Dxl2_ShouldReturnStatus(1U, DXL2_INST_REG_WRITE)
              && Dxl2_ShouldReturnStatus(1U, DXL2_INST_ACTION),
          "supported unicast instructions return Status");
    check(Dxl2_ShouldReturnStatus(DXL2_BROADCAST_ID, DXL2_INST_PING)
              && Dxl2_ShouldReturnStatus(DXL2_BROADCAST_ID, DXL2_INST_SYNC_READ),
          "implemented broadcast read operations return Status");
    check(!Dxl2_ShouldReturnStatus(DXL2_BROADCAST_ID, DXL2_INST_WRITE)
              && !Dxl2_ShouldReturnStatus(DXL2_BROADCAST_ID, DXL2_INST_REG_WRITE)
              && !Dxl2_ShouldReturnStatus(DXL2_BROADCAST_ID, DXL2_INST_ACTION)
              && !Dxl2_ShouldReturnStatus(DXL2_BROADCAST_ID, DXL2_INST_SYNC_WRITE)
              && !Dxl2_ShouldReturnStatus(DXL2_BROADCAST_ID,
                                           DXL2_INST_TK_SYNC_CONTROL),
          "broadcast write operations never return Status");
    check(!Dxl2_ShouldReturnStatus(1U, DXL2_INST_STATUS),
          "a received Status Packet is never acknowledged");
}

static void test_tk_sync_control_parser(void)
{
    uint8_t parameters[DXL2_TK_SYNC_HEADER_SIZE
                       + 2U * DXL2_TK_SYNC_RECORD_SIZE] = {0};
    uint8_t encoded[DXL2_MAX_PACKET_SIZE];
    Dxl2DecodeResult decoded;
    Dxl2TkSyncControlView view;
    uint16_t length;

    parameters[0] = 0x34U;
    parameters[1] = 0x12U;
    parameters[2] = DXL2_TK_EXECUTE_NEXT_UPDATE;
    parameters[7] = 0x1FU;
    parameters[9] = 2U;
    parameters[10] = 1U;
    parameters[25] = 3U;
    length = Dxl2_EncodeInstruction(encoded, sizeof(encoded), DXL2_BROADCAST_ID,
                                    DXL2_INST_TK_SYNC_CONTROL, parameters,
                                    sizeof(parameters));
    decoded = Dxl2_DecodePacket(encoded, length);
    check(decoded.status == DXL2_DECODE_OK
              && Dxl2_ParseTkSyncControl(&decoded.packet, 3U, &view)
                     == DXL2_TK_SYNC_TARGETED
              && view.sequence == 0x1234U && view.ack_mask == 0x001FU
              && view.reply_index == 1U && view.record == &decoded.packet.parameters[25],
          "TK Sync Control valid targeted record");
    check(Dxl2_ParseTkSyncControl(&decoded.packet, 2U, &view)
              == DXL2_TK_SYNC_NOT_TARGETED,
          "TK Sync Control valid non-targeted node");

    decoded.packet.parameters[25] = 1U;
    check(Dxl2_ParseTkSyncControl(&decoded.packet, 1U, &view)
              == DXL2_TK_SYNC_INVALID,
          "TK Sync Control duplicate ID invalidates whole packet");
    decoded.packet.parameters[25] = 3U;
    decoded.packet.parameters[8] = 0x80U;
    check(Dxl2_ParseTkSyncControl(&decoded.packet, 1U, &view)
              == DXL2_TK_SYNC_INVALID,
          "TK Sync Control reserved ACK mask invalidates whole packet");
    decoded.packet.parameters[8] = 0U;
    --decoded.packet.parameter_length;
    check(Dxl2_ParseTkSyncControl(&decoded.packet, 1U, &view)
              == DXL2_TK_SYNC_INVALID,
          "TK Sync Control record length mismatch invalidates whole packet");
}

static void test_tk_sync_control_eight_nodes(void)
{
    uint8_t parameters[DXL2_TK_SYNC_HEADER_SIZE
                       + DXL2_TK_SYNC_MAX_NODES * DXL2_TK_SYNC_RECORD_SIZE] = {0};
    uint8_t encoded[DXL2_MAX_PACKET_SIZE];
    Dxl2DecodeResult decoded;
    Dxl2TkSyncControlView view;
    uint16_t length;
    uint8_t index;

    parameters[7] = 0xFFU;
    parameters[8] = 0x03U;
    parameters[9] = DXL2_TK_SYNC_MAX_NODES;
    for (index = 0U; index < DXL2_TK_SYNC_MAX_NODES; ++index)
    {
        parameters[DXL2_TK_SYNC_HEADER_SIZE
                   + (uint16_t)index * DXL2_TK_SYNC_RECORD_SIZE] =
            (uint8_t)(index + 1U);
    }
    length = Dxl2_EncodeInstruction(encoded, sizeof(encoded), DXL2_BROADCAST_ID,
                                    DXL2_INST_TK_SYNC_CONTROL, parameters,
                                    sizeof(parameters));
    decoded = Dxl2_DecodePacket(encoded, length);
    check(sizeof(parameters) == 130U && length >= 140U
              && decoded.status == DXL2_DECODE_OK
              && Dxl2_ParseTkSyncControl(&decoded.packet, 8U, &view)
                     == DXL2_TK_SYNC_TARGETED
              && view.node_count == 8U && view.reply_index == 7U,
          "eight-node TK Sync Control fits protocol buffers");
}

int main(void)
{
    test_official_ping();
    test_stuffing_and_stream();
    test_status_and_sync_instructions();
    test_control_table_contract();
    test_response_policy();
    test_tk_sync_control_parser();
    test_tk_sync_control_eight_nodes();
    return failures == 0 ? 0 : 1;
}
