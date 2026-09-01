/**
 * @file Crsf.c
 * @brief CRSF RC Channels Packed decoder implementation.
 * @brief CRSF 紧凑遥控通道解码实现。
 */

#include "Crsf.h"
#include "FixedPointMath.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

uint8_t Crsf_Crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;

    if (data == NULL)
    {
        return 0U;
    }
    while (length-- != 0U)
    {
        uint8_t bit;
        crc ^= *data++;
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 0x80U) != 0U
                      ? (uint8_t)((crc << 1U) ^ 0xD5U)
                      : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static void Crsf_DecodeChannels(uint16_t channels[CRSF_CHANNEL_COUNT],
                                const uint8_t *payload)
{
    uint32_t accumulator = 0U;
    uint8_t accumulator_bits = 0U;
    uint8_t payload_index = 0U;
    uint8_t channel;

    for (channel = 0U; channel < CRSF_CHANNEL_COUNT; ++channel)
    {
        while (accumulator_bits < CRSF_CHANNEL_BITS)
        {
            accumulator |= (uint32_t)payload[payload_index++] << accumulator_bits;
            accumulator_bits = (uint8_t)(accumulator_bits + 8U);
        }
        channels[channel] = (uint16_t)(accumulator & 0x07FFU);
        accumulator >>= CRSF_CHANNEL_BITS;
        accumulator_bits = (uint8_t)(accumulator_bits - CRSF_CHANNEL_BITS);
    }
}

static void Crsf_ResetCandidate(CrsfContext *context)
{
    context->frame_size = 0U;
    context->expected_size = 0U;
}

static void Crsf_FinishCandidate(CrsfContext *context)
{
    const uint8_t frame_length = context->frame[1];
    const uint8_t expected_crc = context->frame[context->expected_size - 1U];
    const uint8_t actual_crc = Crsf_Crc8(&context->frame[2],
                                         (uint8_t)(frame_length - 1U));

    if (expected_crc != actual_crc)
    {
        if (context->crc_error_count < UINT32_MAX) ++context->crc_error_count;
    }
    else if (frame_length == CRSF_RC_FRAME_LENGTH
             && context->frame[2] == CRSF_FRAME_TYPE_RC_CHANNELS)
    {
        uint8_t next = (uint8_t)(context->active_channel_buffer ^ 1U);
        Crsf_DecodeChannels(context->channel_buffers[next], &context->frame[3]);
        context->active_channel_buffer = next;
        context->channels_valid = true;
        context->frame_ever_received = true;
        ++context->frame_generation;
        if (context->valid_frame_count < UINT32_MAX) ++context->valid_frame_count;
    }
    else
    {
        if (context->malformed_frame_count < UINT32_MAX)
            ++context->malformed_frame_count;
    }
    Crsf_ResetCandidate(context);
}

static void Crsf_ProcessByte(CrsfContext *context, uint8_t byte)
{
    if (context->frame_size == 0U)
    {
        if (byte == CRSF_DEVICE_ADDRESS_FC)
        {
            context->frame[0] = byte;
            context->frame_size = 1U;
        }
        return;
    }

    if (context->frame_size == 1U)
    {
        if (byte != CRSF_RC_FRAME_LENGTH)
        {
            Crsf_ResetCandidate(context);
            if (byte == CRSF_DEVICE_ADDRESS_FC)
            {
                context->frame[0] = byte;
                context->frame_size = 1U;
            }
            return;
        }
        context->frame[1] = byte;
        context->frame_size = 2U;
        context->expected_size = (uint8_t)(byte + 2U);
        return;
    }

    if (context->frame_size == 2U && byte != CRSF_FRAME_TYPE_RC_CHANNELS)
    {
        Crsf_ResetCandidate(context);
        if (byte == CRSF_DEVICE_ADDRESS_FC)
        {
            context->frame[0] = byte;
            context->frame_size = 1U;
        }
        return;
    }

    context->frame[context->frame_size++] = byte;
    if (context->frame_size == context->expected_size)
    {
        Crsf_FinishCandidate(context);
    }
}

void Crsf_Init(CrsfContext *context, Param *param)
{
    if (context != NULL)
    {
        memset(context, 0, sizeof(*context));
        context->param = param;
        context->command.mode = SERVO_MODE_POSITION;
        context->command.position_multi_turn = true;
    }
}

void Crsf_ProcessBytes(CrsfContext *context, const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if (context == NULL || (data == NULL && length != 0U))
    {
        return;
    }
    for (index = 0U; index < length; ++index)
    {
        Crsf_ProcessByte(context, data[index]);
    }
}

uint16_t Crsf_GetChannel(const CrsfContext *context, uint8_t channel_index)
{
    if (context == NULL || channel_index >= CRSF_CHANNEL_COUNT)
    {
        return 0U;
    }
    return context->channel_buffers[context->active_channel_buffer][channel_index];
}

static int32_t Crsf_AbsI32(int32_t value)
{
    return value < 0 ? -value : value;
}

static int32_t Crsf_MapPosition(const Param *param, uint16_t raw)
{
    const int32_t negative_limit = param->CrsfNegativePositionLimit;
    const int32_t positive_limit = param->CrsfPositivePositionLimit;
    int32_t center_reference = param->CrsfCenterReference;
    uint16_t clamped = raw;

    if (center_reference < negative_limit || center_reference > positive_limit)
        return param->EncoderMultiTurnValue;
    if (clamped < param->CrsfChannelMin) clamped = param->CrsfChannelMin;
    if (clamped > param->CrsfChannelMax) clamped = param->CrsfChannelMax;

    int32_t target;

    if (clamped <= param->CrsfChannelCenter)
    {
        const uint32_t denominator = (uint32_t)(param->CrsfChannelCenter
                                                - param->CrsfChannelMin);
        const uint32_t numerator = (uint32_t)(param->CrsfChannelCenter - clamped);
        const int64_t travel = (int64_t)center_reference - negative_limit;
        target = center_reference - FixedPoint_DivideS64ByU32(
            travel * numerator, denominator);
    }
    else
    {
        const uint32_t denominator = (uint32_t)(param->CrsfChannelMax
                                                - param->CrsfChannelCenter);
        const uint32_t numerator = (uint32_t)(clamped - param->CrsfChannelCenter);
        const int64_t travel = (int64_t)positive_limit - center_reference;
        target = center_reference + FixedPoint_DivideS64ByU32(
            travel * numerator, denominator);
    }
    if (target < negative_limit) target = negative_limit;
    if (target > positive_limit) target = positive_limit;
    return target;
}

static void Crsf_DisableCommand(CrsfContext *context)
{
    context->command.mode = SERVO_MODE_POSITION;
    context->command.enable = false;
    context->command.target_current_mA = 0;
    context->command.target_speed = 0;
    context->command.target_position = context->param != NULL
                                           ? context->param->EncoderMultiTurnValue : 0;
    context->command.current_limit_mA = 0U;
    context->command.speed_limit_cps = 0U;
}

static bool Crsf_ChannelConfigured(uint8_t channel)
{
    return channel >= 1U && channel <= CRSF_CHANNEL_COUNT;
}

static void Crsf_PublishRawChannels(CrsfContext *context)
{
    Param *param = context->param;
    const uint8_t snapshot = context->active_channel_buffer;
    const uint16_t *channels = context->channel_buffers[snapshot];

    param->CrsfRawPosition = Crsf_ChannelConfigured(param->CrsfPositionChannel)
                                 ? channels[param->CrsfPositionChannel - 1U]
                                 : 0U;
    param->CrsfRawEnable = Crsf_ChannelConfigured(param->CrsfEnableChannel)
                               ? channels[param->CrsfEnableChannel - 1U]
                               : 0U;
    param->CrsfRawCenter = Crsf_ChannelConfigured(param->CrsfCenterChannel)
                               ? channels[param->CrsfCenterChannel - 1U]
                               : 0U;
}

static bool Crsf_UpdateEnableRequest(CrsfContext *context)
{
    const Param *param = context->param;

    if (Crsf_ChannelConfigured(param->CrsfEnableChannel))
    {
        if (param->CrsfRawEnable < param->CrsfEnableThreshold)
        {
            context->enable_latched = false;
            context->enable_channel_seen_low = true;
        }
        else if (param->CrsfAutoEnable || context->enable_channel_seen_low)
        {
            context->enable_latched = true;
        }
        return context->enable_latched;
    }
    context->enable_latched = false;
    context->enable_channel_seen_low = false;
    return param->CrsfAutoEnable || param->CrsfManualEnable;
}

static void Crsf_UpdateCenterTrigger(CrsfContext *context, bool frame_valid)
{
    Param *param = context->param;

    if (!frame_valid || !Crsf_ChannelConfigured(param->CrsfCenterChannel))
    {
        context->center_latched = false;
        context->center_channel_seen_low = false;
        return;
    }
    if (param->CrsfRawCenter < param->CrsfCenterTrigger)
    {
        context->center_latched = false;
        context->center_channel_seen_low = true;
    }
    else if (!context->center_latched && context->center_channel_seen_low)
    {
        param->CrsfCenterReference = param->EncoderMultiTurnValue;
        context->center_reference_valid = true;
        context->center_latched = true;
    }
}

static void Crsf_StartArmTracking(CrsfContext *context)
{
    Param *param = context->param;

    if (!context->center_reference_valid)
    {
        context->center_reference_valid = true;
    }
    /* Freeze one target for ARM validation; live channel tracking starts only after ACTIVE. */
    /* ARM 验证期间锁定单一目标；只有进入 ACTIVE 后才跟随实时通道。 */
    context->arm_target_position = Crsf_MapPosition(param, param->CrsfRawPosition);
    context->control_state = CRSF_CONTROL_ARM_TRACK;
    context->arm_elapsed_ms = 0U;
}

static void Crsf_RunArmMonitor(CrsfContext *context)
{
    Param *param = context->param;
    const int32_t error = context->arm_target_position - param->EncoderMultiTurnValue;
    const int32_t error_abs = Crsf_AbsI32(error);

    if (error_abs <= param->CrsfArmFollowError)
    {
        context->control_state = CRSF_CONTROL_ACTIVE;
    }
}

void Crsf_1msTick(CrsfContext *context)
{
    Param *param;
    uint16_t status = 0U;
    bool frame_valid;
    bool enable_request;
    int32_t target;

    if (context == NULL || context->param == NULL)
    {
        return;
    }
    param = context->param;
    if (context->frame_generation != context->processed_frame_generation)
    {
        context->processed_frame_generation = context->frame_generation;
        context->frame_age_ms = 0U;
    }
    else if (context->channels_valid && context->frame_age_ms < UINT16_MAX)
    {
        ++context->frame_age_ms;
    }
    frame_valid = context->channels_valid
                  && context->frame_age_ms <= param->CrsfWatchdogMs;
    if (frame_valid) status |= CRSF_STATUS_FRAME_VALID;
    else if (context->frame_ever_received) status |= CRSF_STATUS_TIMEOUT;
    if (Crsf_ChannelConfigured(param->CrsfPositionChannel))
        status |= CRSF_STATUS_POSITION_CHANNEL_VALID;
    if (Crsf_ChannelConfigured(param->CrsfEnableChannel))
        status |= CRSF_STATUS_ENABLE_CHANNEL_VALID;

    Crsf_PublishRawChannels(context);
    Crsf_UpdateCenterTrigger(context, frame_valid);
    enable_request = frame_valid && Crsf_UpdateEnableRequest(context);
    if (enable_request) status |= CRSF_STATUS_ENABLE_REQUEST;

    if (param->ControlSource != CONTROL_SOURCE_CRSF || !frame_valid
        || !Crsf_ChannelConfigured(param->CrsfPositionChannel)
        || param->FaultCode != 0U)
    {
        context->control_state = CRSF_CONTROL_DISABLED;
        context->enable_latched = false;
        context->enable_channel_seen_low = false;
        Crsf_DisableCommand(context);
    }
    else if (!enable_request)
    {
        context->control_state = CRSF_CONTROL_DISABLED;
        Crsf_DisableCommand(context);
    }
    else if (context->control_state == CRSF_CONTROL_ARM_FAILED)
    {
        Crsf_DisableCommand(context);
    }
    else
    {
        if (context->control_state == CRSF_CONTROL_DISABLED)
            Crsf_StartArmTracking(context);
        target = context->control_state == CRSF_CONTROL_ARM_TRACK
                     ? context->arm_target_position
                     : Crsf_MapPosition(param, param->CrsfRawPosition);
        context->command.mode = SERVO_MODE_POSITION;
        context->command.enable = true;
        context->command.target_position = target;
        context->command.target_current_mA = 0;
        context->command.target_speed = 0;
        if (context->control_state == CRSF_CONTROL_ARM_TRACK)
        {
            context->command.current_limit_mA = param->CrsfArmCurrentLimit_mA;
            context->command.speed_limit_cps = param->CrsfArmSpeed_cps;
            if (context->arm_elapsed_ms < UINT16_MAX) ++context->arm_elapsed_ms;
            Crsf_RunArmMonitor(context);
            if (context->control_state == CRSF_CONTROL_ARM_TRACK
                && context->arm_elapsed_ms >= param->CrsfArmTimeoutMs)
            {
                context->control_state = CRSF_CONTROL_ARM_FAILED;
                Crsf_DisableCommand(context);
            }
        }
        else
        {
            /* Keep the configured CRSF safety envelope after arming. */
            context->command.current_limit_mA = param->CrsfArmCurrentLimit_mA;
            context->command.speed_limit_cps = param->CrsfArmSpeed_cps;
        }
    }

    if (context->center_reference_valid)
        status |= CRSF_STATUS_CENTER_REFERENCE_VALID;
    if (context->control_state == CRSF_CONTROL_ARM_TRACK)
        status |= CRSF_STATUS_ARM_TRACKING;
    else if (context->control_state == CRSF_CONTROL_ACTIVE)
        status |= CRSF_STATUS_ACTIVE;
    else if (context->control_state == CRSF_CONTROL_ARM_FAILED)
        status |= CRSF_STATUS_ARM_FAILED;
    param->CrsfStatus = status;
}

const ServoCommand *Crsf_GetActiveCommand(const CrsfContext *context)
{
    static const ServoCommand disabled = {0};
    return context != NULL ? &context->command : &disabled;
}
