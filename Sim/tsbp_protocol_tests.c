/**
 * @file tsbp_protocol_tests.c
 * @brief PC-side protocol tests for TSBP frame basics.
 * @brief TSBP 基础帧格式的 PC 端协议测试。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(expr) do { if (!(expr)) { \
    printf("FAIL: %s:%d: %s\n", __FILE__, __LINE__, #expr); return 1; } \
} while (0)

#define TSBP_SOF0 0xA5U
#define TSBP_SOF1 0x5AU
#define TSBP_FAST_SOF 0xA6U
#define TSBP_VERSION 0x01U
#define TSBP_RX_PDO_SIZE 13U
#define TSBP_TX_PDO_SIZE 17U
#define TSBP_MAX_REPLY_DELAY_US 1000U

static uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8U; b++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int test_crc_vector(void)
{
    static const uint8_t text[] = "123456789";

    CHECK(crc16_ccitt_false(text, 9) == 0x29B1U);
    return 0;
}

static int test_sdo_read_frame_crc(void)
{
    uint8_t frame[16] = {0};
    uint16_t crc;

    frame[0] = TSBP_SOF0;
    frame[1] = TSBP_SOF1;
    frame[2] = TSBP_VERSION;
    frame[3] = 0x10;
    frame[4] = 0;
    frame[5] = 7;
    frame[6] = 1;
    frame[7] = 0;
    wr16(&frame[8], 4);
    wr16(&frame[10], 0x1001);
    frame[12] = 0;
    frame[13] = 0;
    crc = crc16_ccitt_false(&frame[2], 12);
    wr16(&frame[14], crc);

    CHECK(frame[0] == TSBP_SOF0 && frame[1] == TSBP_SOF1);
    CHECK(rd16(&frame[8]) == 4);
    CHECK(rd16(&frame[14]) == crc16_ccitt_false(&frame[2], 12));
    return 0;
}

static int test_fast_pdo_crc(void)
{
    uint8_t frame[19] = {0};
    uint16_t crc;

    frame[0] = TSBP_FAST_SOF;
    frame[1] = 3;
    wr16(&frame[2], 100);
    wr16(&frame[4], 1);
    frame[6] = 1;
    wr16(&frame[7], 120);
    crc = crc16_ccitt_false(&frame[1], 16);
    wr16(&frame[17], crc);

    CHECK(frame[0] == TSBP_FAST_SOF);
    CHECK(rd16(&frame[17]) == crc16_ccitt_false(&frame[1], 16));
    frame[7] ^= 0x55U;
    CHECK(rd16(&frame[17]) != crc16_ccitt_false(&frame[1], 16));
    return 0;
}

static int test_baud_policy(void)
{
    const uint32_t supported[] = {115200UL, 500000UL, 1000000UL, 2000000UL};
    uint32_t invalid = 921600UL;
    uint8_t found = 0;

    for (uint32_t i = 0; i < sizeof(supported) / sizeof(supported[0]); i++) {
        if (supported[i] == 1000000UL) found = 1;
        CHECK(supported[i] != invalid);
    }
    CHECK(found != 0U);
    return 0;
}

static int test_parallel_slot_math(void)
{
    uint16_t reply_slot_us = 120U;

    for (uint8_t pos = 1; pos <= 4U; pos++) {
        uint16_t rx_offset = (uint16_t)(pos - 1U) * TSBP_RX_PDO_SIZE;
        uint16_t reply_delay = (uint16_t)(pos - 1U) * reply_slot_us;

        CHECK(rx_offset == (uint16_t)((pos - 1U) * 13U));
        CHECK(reply_delay <= TSBP_MAX_REPLY_DELAY_US);
    }
    return 0;
}

static void fill_rx_slot(uint8_t *p, uint8_t slot_id)
{
    for (uint8_t i = 0; i < TSBP_RX_PDO_SIZE; i++) {
        p[i] = (uint8_t)(0x10U * slot_id + i);
    }
}

static void fill_tx_slot(uint8_t *p, uint8_t slot_id)
{
    for (uint8_t i = 0; i < TSBP_TX_PDO_SIZE; i++) {
        p[i] = (uint8_t)(0x80U + (0x10U * slot_id) + i);
    }
}

static uint16_t chain_step(uint8_t node_count, uint8_t position,
                           const uint8_t *in, uint8_t *out)
{
    uint8_t remaining_rx = (uint8_t)(node_count - position + 1U);
    uint8_t collected_tx = (uint8_t)(position - 1U);
    uint16_t keep_rx_len = (uint16_t)(remaining_rx - 1U) * TSBP_RX_PDO_SIZE;
    uint16_t keep_tx_len = (uint16_t)collected_tx * TSBP_TX_PDO_SIZE;

    if (keep_rx_len != 0U) {
        memcpy(out, &in[TSBP_RX_PDO_SIZE], keep_rx_len);
    }
    if (keep_tx_len != 0U) {
        memcpy(&out[keep_rx_len], &in[(uint16_t)remaining_rx * TSBP_RX_PDO_SIZE], keep_tx_len);
    }
    fill_tx_slot(&out[keep_rx_len + keep_tx_len], position);
    return (uint16_t)(keep_rx_len + keep_tx_len + TSBP_TX_PDO_SIZE);
}

static int test_chain_store_forward_roll(void)
{
    uint8_t image_a[96] = {0};
    uint8_t image_b[96] = {0};
    uint16_t len;

    fill_rx_slot(&image_a[0], 1);
    fill_rx_slot(&image_a[TSBP_RX_PDO_SIZE], 2);
    fill_rx_slot(&image_a[2U * TSBP_RX_PDO_SIZE], 3);
    fill_rx_slot(&image_a[3U * TSBP_RX_PDO_SIZE], 4);

    len = chain_step(4, 1, image_a, image_b);
    CHECK(len == (uint16_t)(3U * TSBP_RX_PDO_SIZE + TSBP_TX_PDO_SIZE));
    CHECK(memcmp(&image_b[0], &image_a[TSBP_RX_PDO_SIZE], TSBP_RX_PDO_SIZE) == 0);

    len = chain_step(4, 2, image_b, image_a);
    CHECK(len == (uint16_t)(2U * TSBP_RX_PDO_SIZE + 2U * TSBP_TX_PDO_SIZE));
    CHECK(memcmp(&image_a[0], &image_b[TSBP_RX_PDO_SIZE], TSBP_RX_PDO_SIZE) == 0);

    len = chain_step(4, 3, image_a, image_b);
    CHECK(len == (uint16_t)(TSBP_RX_PDO_SIZE + 3U * TSBP_TX_PDO_SIZE));
    CHECK(memcmp(&image_b[0], &image_a[TSBP_RX_PDO_SIZE], TSBP_RX_PDO_SIZE) == 0);

    len = chain_step(4, 4, image_b, image_a);
    CHECK(len == (uint16_t)(4U * TSBP_TX_PDO_SIZE));
    for (uint8_t pos = 1; pos <= 4U; pos++) {
        uint8_t expect[TSBP_TX_PDO_SIZE];
        fill_tx_slot(expect, pos);
        CHECK(memcmp(&image_a[(uint16_t)(pos - 1U) * TSBP_TX_PDO_SIZE],
                     expect, TSBP_TX_PDO_SIZE) == 0);
    }
    return 0;
}

int main(void)
{
    CHECK(test_crc_vector() == 0);
    CHECK(test_sdo_read_frame_crc() == 0);
    CHECK(test_fast_pdo_crc() == 0);
    CHECK(test_baud_policy() == 0);
    CHECK(test_parallel_slot_math() == 0);
    CHECK(test_chain_store_forward_roll() == 0);
    printf("PASS\n");
    return 0;
}
