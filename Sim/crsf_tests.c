#include "Crsf.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void pack_channels(const uint16_t channels[CRSF_CHANNEL_COUNT],
                          uint8_t payload[CRSF_RC_PAYLOAD_SIZE])
{
    uint32_t accumulator = 0U;
    uint8_t accumulator_bits = 0U;
    uint8_t output = 0U;
    uint8_t channel;

    memset(payload, 0, CRSF_RC_PAYLOAD_SIZE);
    for (channel = 0U; channel < CRSF_CHANNEL_COUNT; ++channel)
    {
        accumulator |= (uint32_t)(channels[channel] & 0x07FFU) << accumulator_bits;
        accumulator_bits = (uint8_t)(accumulator_bits + CRSF_CHANNEL_BITS);
        while (accumulator_bits >= 8U)
        {
            payload[output++] = (uint8_t)accumulator;
            accumulator >>= 8U;
            accumulator_bits = (uint8_t)(accumulator_bits - 8U);
        }
    }
}

static uint8_t build_frame(uint8_t frame[26],
                           const uint16_t channels[CRSF_CHANNEL_COUNT])
{
    frame[0] = CRSF_DEVICE_ADDRESS_FC;
    frame[1] = CRSF_RC_FRAME_LENGTH;
    frame[2] = CRSF_FRAME_TYPE_RC_CHANNELS;
    pack_channels(channels, &frame[3]);
    frame[25] = Crsf_Crc8(&frame[2], 23U);
    return 26U;
}

static void init_control_param(Param *param)
{
    memset(param, 0, sizeof(*param));
    param->ControlSource = CONTROL_SOURCE_CRSF;
    param->CrsfPositionChannel = 1U;
    param->CrsfEnableChannel = 2U;
    param->CrsfChannelMin = 172U;
    param->CrsfChannelCenter = 992U;
    param->CrsfChannelMax = 1811U;
    param->CrsfCenterTrigger = 1200U;
    param->CrsfEnableThreshold = 1200U;
    param->CrsfAutoEnable = true;
    param->CrsfArmCurrentLimit_mA = 300U;
    param->CrsfArmSpeed_cps = 5000U;
    param->CrsfArmFollowError = 100U;
    param->CrsfArmTimeoutMs = 1000U;
    param->CrsfWatchdogMs = 100U;
    param->CrsfNegativePositionLimit = 0;
    param->CrsfPositivePositionLimit = 16383;
    param->CrsfCenterReference = 8191;
    param->EncoderMultiTurnValue = 5000;
}

static void test_split_frame_decodes_all_channels(void)
{
    CrsfContext context;
    uint8_t frame[26];
    uint16_t channels[CRSF_CHANNEL_COUNT] = {
        172U, 992U, 1811U, 0U, 2047U, 300U, 400U, 500U,
        600U, 700U, 800U, 900U, 1000U, 1100U, 1200U, 1300U
    };
    uint8_t index;

    Crsf_Init(&context, NULL);
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, 7U);
    assert(!context.channels_valid);
    Crsf_ProcessBytes(&context, &frame[7], 19U);
    assert(context.channels_valid);
    assert(context.valid_frame_count == 1U);
    for (index = 0U; index < CRSF_CHANNEL_COUNT; ++index)
    {
        assert(Crsf_GetChannel(&context, index) == channels[index]);
    }
}

static void test_noise_and_bad_crc_are_rejected(void)
{
    CrsfContext context;
    uint8_t frame[26];
    uint8_t stream[40] = {0xFFU, 0xFFU, 0xFDU, 0x00U, 0x01U, 0x07U};
    uint16_t channels[CRSF_CHANNEL_COUNT] = {0U};

    channels[0] = 992U;
    Crsf_Init(&context, NULL);
    (void)build_frame(frame, channels);
    frame[25] ^= 0x01U;
    memcpy(&stream[6], frame, sizeof(frame));
    Crsf_ProcessBytes(&context, stream, 32U);
    assert(!context.channels_valid);
    assert(context.crc_error_count == 1U);

    frame[25] ^= 0x01U;
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    assert(context.channels_valid);
    assert(Crsf_GetChannel(&context, 0U) == 992U);
    assert(context.valid_frame_count == 1U);
}

static void test_enable_channel_arms_and_watchdog_disables(void)
{
    CrsfContext context;
    Param param;
    uint8_t frame[26];
    uint16_t channels[CRSF_CHANNEL_COUNT] = {0U};
    uint16_t tick;

    init_control_param(&param);
    param.EncoderMultiTurnValue = 8191;
    channels[0] = 992U;
    channels[1] = 1300U;
    Crsf_Init(&context, &param);
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(Crsf_GetActiveCommand(&context)->enable);
    assert(context.control_state == CRSF_CONTROL_ACTIVE);
    assert(Crsf_GetActiveCommand(&context)->target_position == 8191);
    assert(Crsf_GetActiveCommand(&context)->current_limit_mA == 300U);
    assert(Crsf_GetActiveCommand(&context)->speed_limit_cps == 5000U);
    assert((param.CrsfStatus & CRSF_STATUS_ACTIVE) != 0U);

    for (tick = 0U; tick <= param.CrsfWatchdogMs; ++tick)
    {
        Crsf_1msTick(&context);
    }
    assert(!Crsf_GetActiveCommand(&context)->enable);
    assert((param.CrsfStatus & CRSF_STATUS_TIMEOUT) != 0U);
}

static void test_arm_timeout_latches_failure(void)
{
    CrsfContext context;
    Param param;
    uint8_t frame[26];
    uint16_t channels[CRSF_CHANNEL_COUNT] = {0U};
    uint8_t tick;

    init_control_param(&param);
    param.CrsfArmTimeoutMs = 5U;
    param.CrsfCenterReference = 8192;
    channels[0] = 1811U;
    channels[1] = 1300U;
    Crsf_Init(&context, &param);
    context.center_reference_valid = true;
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    for (tick = 0U; tick < 7U; ++tick)
    {
        Crsf_1msTick(&context);
    }
    assert(context.control_state == CRSF_CONTROL_ARM_FAILED);
    assert(!Crsf_GetActiveCommand(&context)->enable);
    assert((param.CrsfStatus & CRSF_STATUS_ARM_FAILED) != 0U);
}

static void test_manual_enable_and_source_change_disarm(void)
{
    CrsfContext context;
    Param param;
    uint8_t frame[26];
    uint16_t channels[CRSF_CHANNEL_COUNT] = {0U};

    init_control_param(&param);
    param.CrsfEnableChannel = 0U;
    param.CrsfManualEnable = true;
    channels[0] = 992U;
    Crsf_Init(&context, &param);
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(Crsf_GetActiveCommand(&context)->enable);

    param.ControlSource = CONTROL_SOURCE_SERIAL;
    Crsf_1msTick(&context);
    assert(!Crsf_GetActiveCommand(&context)->enable);
    assert(context.control_state == CRSF_CONTROL_DISABLED);
}

static void test_position_endpoints_use_signed_multiturn_limits(void)
{
    CrsfContext context;
    Param param;
    uint8_t frame[26];
    uint16_t channels[CRSF_CHANNEL_COUNT] = {0U};

    init_control_param(&param);
    param.CrsfNegativePositionLimit = -16384;
    param.CrsfPositivePositionLimit = 16384;
    param.CrsfCenterReference = 0;
    param.EncoderMultiTurnValue = 0;
    channels[0] = 172U;
    channels[1] = 1300U;
    Crsf_Init(&context, &param);
    context.center_reference_valid = true;
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(Crsf_GetActiveCommand(&context)->target_position == -16384);
    param.EncoderMultiTurnValue = -16384;
    Crsf_1msTick(&context);
    assert(context.control_state == CRSF_CONTROL_ACTIVE);

    channels[0] = 1811U;
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(Crsf_GetActiveCommand(&context)->target_position == 16384);
}

static void test_auto_enable_false_requires_low_then_rising_edge(void)
{
    CrsfContext context;
    Param param;
    uint8_t frame[26];
    uint16_t channels[CRSF_CHANNEL_COUNT] = {0U};

    init_control_param(&param);
    param.EncoderMultiTurnValue = 8191;
    param.CrsfAutoEnable = false;
    channels[0] = 992U;
    channels[1] = 1200U;
    Crsf_Init(&context, &param);

    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(!Crsf_GetActiveCommand(&context)->enable);

    channels[1] = 1199U;
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(!Crsf_GetActiveCommand(&context)->enable);

    channels[1] = 1200U;
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(Crsf_GetActiveCommand(&context)->enable);
    assert(context.control_state == CRSF_CONTROL_ACTIVE);

    channels[1] = 1199U;
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(!Crsf_GetActiveCommand(&context)->enable);
}

static void test_position_channel_zero_disables_control(void)
{
    CrsfContext context;
    Param param;
    uint8_t frame[26];
    uint16_t channels[CRSF_CHANNEL_COUNT] = {0U};

    init_control_param(&param);
    param.CrsfPositionChannel = 0U;
    channels[1] = 1300U;
    Crsf_Init(&context, &param);
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);

    assert(!Crsf_GetActiveCommand(&context)->enable);
    assert((param.CrsfStatus & CRSF_STATUS_POSITION_CHANNEL_VALID) == 0U);
}

static void test_center_trigger_requires_low_to_high_edge(void)
{
    CrsfContext context;
    Param param;
    uint8_t frame[26];
    uint16_t channels[CRSF_CHANNEL_COUNT] = {0U};

    init_control_param(&param);
    param.CrsfCenterChannel = 3U;
    channels[0] = 992U;
    channels[1] = 0U;
    channels[2] = 1300U;
    Crsf_Init(&context, &param);

    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(!context.center_reference_valid);

    channels[2] = 1199U;
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(param.CrsfRawCenter == 1199U);
    assert(context.center_channel_seen_low);
    param.EncoderMultiTurnValue = 6789;

    channels[2] = 1200U;
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(context.center_reference_valid);
    assert(param.CrsfCenterReference == 6789);
}

static void test_arm_target_is_frozen_until_active(void)
{
    CrsfContext context;
    Param param;
    uint8_t frame[26];
    uint16_t channels[CRSF_CHANNEL_COUNT] = {0U};

    init_control_param(&param);
    param.CrsfNegativePositionLimit = -10000;
    param.CrsfPositivePositionLimit = 10000;
    param.CrsfCenterReference = 0;
    param.EncoderMultiTurnValue = 0;
    channels[0] = 1811U;
    channels[1] = 1300U;
    Crsf_Init(&context, &param);
    context.center_reference_valid = true;

    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(context.control_state == CRSF_CONTROL_ARM_TRACK);
    assert(Crsf_GetActiveCommand(&context)->target_position == 10000);

    channels[0] = 172U;
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(context.control_state == CRSF_CONTROL_ARM_TRACK);
    assert(Crsf_GetActiveCommand(&context)->target_position == 10000);
}

static void test_configured_midpoint_maps_full_position_range(void)
{
    CrsfContext context;
    Param param;
    uint8_t frame[26];
    uint16_t channels[CRSF_CHANNEL_COUNT] = {0U};

    init_control_param(&param);
    param.CrsfNegativePositionLimit = 500;
    param.CrsfPositivePositionLimit = 16200;
    param.CrsfCenterReference = 8350;
    param.EncoderMultiTurnValue = 8350;
    channels[0] = 992U;
    channels[1] = 1457U;
    Crsf_Init(&context, &param);
    context.center_reference_valid = false;

    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);

    assert(context.control_state == CRSF_CONTROL_ACTIVE);
    assert(Crsf_GetActiveCommand(&context)->enable);
    assert(Crsf_GetActiveCommand(&context)->target_position == 8350);
    assert(Crsf_GetActiveCommand(&context)->current_limit_mA == 300U);
    assert(Crsf_GetActiveCommand(&context)->speed_limit_cps == 5000U);

    channels[0] = 1811U;
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(Crsf_GetActiveCommand(&context)->target_position == 16200);

    channels[0] = 172U;
    (void)build_frame(frame, channels);
    Crsf_ProcessBytes(&context, frame, sizeof(frame));
    Crsf_1msTick(&context);
    assert(Crsf_GetActiveCommand(&context)->target_position == 500);
}

int main(void)
{
    test_split_frame_decodes_all_channels();
    test_noise_and_bad_crc_are_rejected();
    test_enable_channel_arms_and_watchdog_disables();
    test_arm_timeout_latches_failure();
    test_manual_enable_and_source_change_disarm();
    test_position_endpoints_use_signed_multiturn_limits();
    test_auto_enable_false_requires_low_then_rising_edge();
    test_position_channel_zero_disables_control();
    test_center_trigger_requires_low_to_high_edge();
    test_arm_target_is_frozen_until_active();
    test_configured_midpoint_maps_full_position_range();
    puts("crsf_tests: PASS");
    return 0;
}
