#include "Dynamixel2Codec.h"

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

    length = Dxl2_EncodeStatus(encoded, sizeof(encoded), 3U, 0x04U,
                              status_parameters, sizeof(status_parameters));
    decoded = Dxl2_DecodePacket(encoded, length);
    check(decoded.status == DXL2_DECODE_OK
              && decoded.packet.instruction == DXL2_INST_STATUS
              && decoded.packet.id == 3U && decoded.packet.error == 0x04U
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

int main(void)
{
    test_official_ping();
    test_stuffing_and_stream();
    test_status_and_sync_instructions();
    return failures == 0 ? 0 : 1;
}
