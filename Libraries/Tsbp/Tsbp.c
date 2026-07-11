/**
 * @file Tsbp.c
 * @brief TK Servo Bus Protocol slave core implementation.
 * @brief TK Servo Bus Protocol 从站核心实现。
 */

#include "Tsbp.h"

#include <string.h>

extern DMA_HandleTypeDef hdma_usart2_rx;
extern uint32_t SystemCoreClock;

enum
{
    TSBP_DATA_U8 = 0x01,
    TSBP_DATA_I8 = 0x02,
    TSBP_DATA_U16 = 0x03,
    TSBP_DATA_I16 = 0x04,
    TSBP_DATA_U32 = 0x05,
    TSBP_DATA_I32 = 0x06,
    TSBP_DATA_BOOL = 0x07,
};

enum
{
    SDO_STATUS_OK = 0x00,
    SDO_STATUS_NOT_FOUND = 0x01,
    SDO_STATUS_ACCESS = 0x02,
    SDO_STATUS_TYPE = 0x03,
    SDO_STATUS_RANGE = 0x04,
    SDO_STATUS_STATE = 0x05,
};

static uint16_t Tsbp_ReadU16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t Tsbp_ReadU32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void Tsbp_WriteU16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)(value >> 8);
}

static void Tsbp_WriteU32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8) & 0xFFU);
    p[2] = (uint8_t)((value >> 16) & 0xFFU);
    p[3] = (uint8_t)(value >> 24);
}

static uint16_t Tsbp_Crc16Update(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)data << 8;
    for (uint8_t i = 0; i < 8U; i++)
    {
        crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint16_t Tsbp_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0; i < len; i++)
    {
        crc = Tsbp_Crc16Update(crc, data[i]);
    }
    return crc;
}

static void Tsbp_StartRx(TsbpContext *ctx)
{
    (void)HAL_UARTEx_ReceiveToIdle_DMA(ctx->huart, ctx->param->RxBuf, sizeof(ctx->param->RxBuf));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

static void Tsbp_InitReplyTimer(void)
{
    uint32_t prescaler = (SystemCoreClock / 1000000UL);

    if (prescaler == 0UL)
    {
        prescaler = 1UL;
    }
    TIM1->PSC = (uint16_t)(prescaler - 1UL);
    TIM1->ARR = 0xFFFFU;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;
}

static void Tsbp_DelayUs(uint16_t delay_us)
{
    uint16_t start;

    if (delay_us == 0U)
    {
        return;
    }

    start = (uint16_t)TIM1->CNT;
    while ((uint16_t)((uint16_t)TIM1->CNT - start) < delay_us)
    {
    }
}

static void Tsbp_SendManagement(TsbpContext *ctx, uint8_t type, uint8_t seq,
                                const uint8_t *payload, uint16_t payload_len)
{
    uint8_t *tx = ctx->param->TxBuf;
    uint16_t frame_len;
    uint16_t crc;

    if (ctx->tx_busy || payload_len > TSBP_MAX_PAYLOAD)
    {
        return;
    }

    tx[0] = TSBP_SOF0;
    tx[1] = TSBP_SOF1;
    tx[2] = TSBP_VERSION;
    tx[3] = type;
    tx[4] = 0;
    tx[5] = seq;
    tx[6] = TSBP_MASTER_ID;
    tx[7] = ctx->node_id;
    Tsbp_WriteU16(&tx[8], payload_len);
    if (payload_len != 0U && payload != NULL)
    {
        memcpy(&tx[10], payload, payload_len);
    }

    crc = Tsbp_Crc16(&tx[2], (uint16_t)(8U + payload_len));
    Tsbp_WriteU16(&tx[10U + payload_len], crc);
    frame_len = (uint16_t)(12U + payload_len);

    ctx->tx_busy = true;
    if (HAL_UART_Transmit_DMA(ctx->huart, tx, frame_len) != HAL_OK)
    {
        ctx->tx_busy = false;
    }
}

static void Tsbp_SendError(TsbpContext *ctx, uint8_t seq, uint8_t error,
                           uint16_t index, uint8_t subindex)
{
    uint8_t payload[8] = {0};

    payload[0] = error;
    payload[1] = 0;
    Tsbp_WriteU16(&payload[2], index);
    payload[4] = subindex;
    payload[5] = (uint8_t)ctx->state;
    Tsbp_SendManagement(ctx, TSBP_TYPE_ERROR, seq, payload, sizeof(payload));
}

static void Tsbp_SendSdoReply(TsbpContext *ctx, uint8_t seq, uint16_t index,
                              uint8_t subindex, uint8_t type, uint8_t size,
                              uint8_t status, const uint8_t *data)
{
    uint8_t payload[14] = {0};

    Tsbp_WriteU16(&payload[0], index);
    payload[2] = subindex;
    payload[3] = type;
    payload[4] = size;
    payload[5] = status;
    if (data != NULL && size <= 8U)
    {
        memcpy(&payload[6], data, size);
    }

    Tsbp_SendManagement(ctx, TSBP_TYPE_SDO_REPLY, seq, payload, sizeof(payload));
}

static uint16_t Tsbp_StatusWord(const TsbpContext *ctx)
{
    uint16_t status = 0;

    if (ctx->state >= TSBP_STATE_INIT) status |= (1U << 0);
    if (ctx->state == TSBP_STATE_OP) status |= (1U << 2);
    if (ctx->state == TSBP_STATE_FAULT) status |= (1U << 3);
    if (ctx->param->ControlSource == CONTROL_SOURCE_PWM_INPUT) status |= (1U << 8);
    if (ctx->param->ControlSource == CONTROL_SOURCE_SERIAL_PDO ||
        ctx->param->ControlSource == CONTROL_SOURCE_SERIAL_SDO) status |= (1U << 9);
    if (ctx->param->FaultCode == 0U) status |= (1U << 11);
    status |= (1U << 12);
    return status;
}

static uint8_t Tsbp_ReadObject(TsbpContext *ctx, uint16_t index, uint8_t subindex,
                               uint8_t *type, uint8_t *size, uint8_t *data)
{
    memset(data, 0, 8);

    switch (index)
    {
    case 0x1000:
        *type = TSBP_DATA_U32;
        *size = 4;
        Tsbp_WriteU32(data, 0x00000001UL);
        return SDO_STATUS_OK;
    case 0x1001:
        *type = TSBP_DATA_U8;
        *size = 1;
        data[0] = TSBP_VERSION;
        return SDO_STATUS_OK;
    case 0x1002:
        *type = TSBP_DATA_U16;
        *size = 2;
        Tsbp_WriteU16(data, 0x0001U);
        return SDO_STATUS_OK;
    case 0x1100:
        *type = TSBP_DATA_U8;
        *size = 1;
        data[0] = ctx->node_id;
        return SDO_STATUS_OK;
    case 0x1101:
        *type = TSBP_DATA_U8;
        *size = 1;
        data[0] = ctx->param->Topology;
        return SDO_STATUS_OK;
    case 0x1102:
        *type = TSBP_DATA_U32;
        *size = 4;
        Tsbp_WriteU32(data, ctx->param->BaudRate);
        return SDO_STATUS_OK;
    case 0x1103:
        *type = TSBP_DATA_U16;
        *size = 2;
        Tsbp_WriteU16(data, ctx->param->SerialWatchdogMs);
        return SDO_STATUS_OK;
    case 0x1105:
        *type = TSBP_DATA_U8;
        *size = 1;
        data[0] = ctx->param->PdoMissLimit;
        return SDO_STATUS_OK;
    case 0x1108:
        *type = TSBP_DATA_U8;
        *size = 1;
        data[0] = ctx->node_id;
        return SDO_STATUS_OK;
    case 0x1109:
        *type = TSBP_DATA_U8;
        *size = 1;
        data[0] = ctx->param->NodeCount;
        return SDO_STATUS_OK;
    case 0x110A:
        *type = TSBP_DATA_U8;
        *size = 1;
        data[0] = ctx->param->NodePosition;
        return SDO_STATUS_OK;
    case 0x110B:
        *type = TSBP_DATA_U16;
        *size = 2;
        Tsbp_WriteU16(data, ctx->param->ReplySlotUs);
        return SDO_STATUS_OK;
    case 0x1201:
        *type = TSBP_DATA_U8;
        *size = 1;
        data[0] = (uint8_t)ctx->state;
        return SDO_STATUS_OK;
    case 0x2000:
        *type = TSBP_DATA_U8;
        *size = 1;
        data[0] = ctx->param->ControlSource;
        return SDO_STATUS_OK;
    case 0x2001:
        *type = TSBP_DATA_U8;
        *size = 1;
        data[0] = (uint8_t)ctx->pending_command.mode;
        return SDO_STATUS_OK;
    case 0x2100:
        *type = TSBP_DATA_U16;
        *size = 2;
        Tsbp_WriteU16(data, Tsbp_StatusWord(ctx));
        return SDO_STATUS_OK;
    case 0x2101:
        *type = TSBP_DATA_I16;
        *size = 2;
        Tsbp_WriteU16(data, (uint16_t)ctx->param->INA181_mA);
        return SDO_STATUS_OK;
    case 0x2102:
        *type = TSBP_DATA_I32;
        *size = 4;
        Tsbp_WriteU32(data, (uint32_t)ctx->param->EncoderSpeed);
        return SDO_STATUS_OK;
    case 0x2103:
        *type = TSBP_DATA_I32;
        *size = 4;
        Tsbp_WriteU32(data, (uint32_t)ctx->param->EncoderMultiTurnValue);
        return SDO_STATUS_OK;
    case 0x2104:
        *type = TSBP_DATA_U16;
        *size = 2;
        Tsbp_WriteU16(data, ctx->param->FaultCode);
        return SDO_STATUS_OK;
    case 0x2105:
        *type = TSBP_DATA_I16;
        *size = 2;
        Tsbp_WriteU16(data, (uint16_t)ctx->param->DrivePower);
        return SDO_STATUS_OK;
    case 0x2601:
        *type = TSBP_DATA_U16;
        *size = 2;
        Tsbp_WriteU16(data, ctx->param->VCC_mV);
        return SDO_STATUS_OK;
    case 0x2602:
        *type = TSBP_DATA_I8;
        *size = 1;
        data[0] = (uint8_t)ctx->param->Temp;
        return SDO_STATUS_OK;
    case 0x2608:
        *type = TSBP_DATA_U8;
        *size = 1;
        data[0] = ctx->param->FailSafePolicy;
        return SDO_STATUS_OK;
    default:
        (void)subindex;
        return SDO_STATUS_NOT_FOUND;
    }
}

static bool Tsbp_IsSupportedBaud(uint32_t baud)
{
    return (baud == 115200UL) || (baud == 500000UL) ||
           (baud == 1000000UL) || (baud == 2000000UL);
}

static uint8_t Tsbp_ClampNodeCount(uint8_t node_count)
{
    if (node_count < 1U || node_count > TSBP_MAX_NODES)
    {
        return 1U;
    }
    return node_count;
}

static uint8_t Tsbp_ClampNodePosition(uint8_t node_position, uint8_t node_count)
{
    if (node_position < 1U || node_position > node_count)
    {
        return 1U;
    }
    return node_position;
}

static void Tsbp_NormalizeBusConfig(Param *param)
{
    if (param->Topology > TSBP_TOPOLOGY_RING_CHAIN)
    {
        param->Topology = TSBP_TOPOLOGY_PARALLEL_BUS;
    }
    param->NodeCount = Tsbp_ClampNodeCount(param->NodeCount);
    param->NodePosition = Tsbp_ClampNodePosition(param->NodePosition, param->NodeCount);
    if (param->ReplySlotUs < 50U || param->ReplySlotUs > TSBP_MAX_REPLY_DELAY_US)
    {
        param->ReplySlotUs = TSBP_DEFAULT_REPLY_SLOT_US;
    }
}

static void Tsbp_ApplyFailSafe(TsbpContext *ctx)
{
    if (ctx->param->FailSafePolicy == FAILSAFE_BRAKE)
    {
        ctx->param->DriveRunMode = 1;
        ctx->param->DrivePower = 0;
    }
    else if (ctx->param->FailSafePolicy == FAILSAFE_FALLBACK_PWM)
    {
        ctx->param->ControlSource = CONTROL_SOURCE_PWM_INPUT;
        ctx->active_command.enable = false;
        ctx->pending_command.enable = false;
    }
    else
    {
        ctx->param->DriveRunMode = 0;
        ctx->param->DrivePower = 0;
        ctx->active_command.enable = false;
        ctx->pending_command.enable = false;
    }
}

static uint8_t Tsbp_WriteObject(TsbpContext *ctx, uint16_t index, uint8_t subindex,
                                uint8_t type, uint8_t size, const uint8_t *data)
{
    (void)subindex;

    switch (index)
    {
    case 0x1101:
        if (type != TSBP_DATA_U8 || size != 1U) return SDO_STATUS_TYPE;
        if (data[0] > TSBP_TOPOLOGY_RING_CHAIN) return SDO_STATUS_RANGE;
        ctx->param->Topology = data[0];
        return SDO_STATUS_OK;
    case 0x1102:
    {
        uint32_t baud;
        if (type != TSBP_DATA_U32 || size != 4U) return SDO_STATUS_TYPE;
        baud = Tsbp_ReadU32(data);
        if (!Tsbp_IsSupportedBaud(baud)) return SDO_STATUS_RANGE;
        ctx->param->BaudRate = baud;
        return SDO_STATUS_OK;
    }
    case 0x1103:
        if (type != TSBP_DATA_U16 || size != 2U) return SDO_STATUS_TYPE;
        ctx->param->SerialWatchdogMs = Tsbp_ReadU16(data);
        return (ctx->param->SerialWatchdogMs != 0U) ? SDO_STATUS_OK : SDO_STATUS_RANGE;
    case 0x1105:
        if (type != TSBP_DATA_U8 || size != 1U) return SDO_STATUS_TYPE;
        ctx->param->PdoMissLimit = data[0];
        return (ctx->param->PdoMissLimit != 0U) ? SDO_STATUS_OK : SDO_STATUS_RANGE;
    case 0x1108:
        if (type != TSBP_DATA_U8 || size != 1U) return SDO_STATUS_TYPE;
        if (data[0] == 0U || data[0] >= TSBP_BROADCAST_ID) return SDO_STATUS_RANGE;
        ctx->node_id = data[0];
        ctx->param->NodeId = data[0];
        return SDO_STATUS_OK;
    case 0x1109:
        if (type != TSBP_DATA_U8 || size != 1U) return SDO_STATUS_TYPE;
        if (data[0] < 1U || data[0] > TSBP_MAX_NODES) return SDO_STATUS_RANGE;
        ctx->param->NodeCount = data[0];
        ctx->param->NodePosition = Tsbp_ClampNodePosition(ctx->param->NodePosition, ctx->param->NodeCount);
        return SDO_STATUS_OK;
    case 0x110A:
        if (type != TSBP_DATA_U8 || size != 1U) return SDO_STATUS_TYPE;
        if (data[0] < 1U || data[0] > ctx->param->NodeCount) return SDO_STATUS_RANGE;
        ctx->param->NodePosition = data[0];
        return SDO_STATUS_OK;
    case 0x110B:
    {
        uint16_t slot_us;
        if (type != TSBP_DATA_U16 || size != 2U) return SDO_STATUS_TYPE;
        slot_us = Tsbp_ReadU16(data);
        if (slot_us < 50U || slot_us > TSBP_MAX_REPLY_DELAY_US) return SDO_STATUS_RANGE;
        ctx->param->ReplySlotUs = slot_us;
        return SDO_STATUS_OK;
    }
    case 0x1200:
        if (type != TSBP_DATA_U8 || size != 1U) return SDO_STATUS_TYPE;
        if (data[0] >= 1U && data[0] <= 4U)
        {
            ctx->state = (TsbpState)data[0];
            return SDO_STATUS_OK;
        }
        if (data[0] == 5U)
        {
            ctx->active_command.enable = false;
            ctx->pending_command.enable = false;
            ctx->state = TSBP_STATE_SAFEOP;
            return SDO_STATUS_OK;
        }
        if (data[0] == 6U)
        {
            ctx->param->FaultCode = 0;
            ctx->state = TSBP_STATE_INIT;
            return SDO_STATUS_OK;
        }
        return SDO_STATUS_RANGE;
    case 0x2000:
        if (type != TSBP_DATA_U8 || size != 1U) return SDO_STATUS_TYPE;
        if (data[0] > CONTROL_SOURCE_SERIAL_SDO) return SDO_STATUS_RANGE;
        ctx->param->ControlSource = data[0];
        ctx->active_command.enable = false;
        ctx->pending_command.enable = false;
        ctx->serial_watchdog_count_ms = 0;
        return SDO_STATUS_OK;
    case 0x2001:
        if (type != TSBP_DATA_U8 || size != 1U) return SDO_STATUS_TYPE;
        if (data[0] > SERVO_MODE_POSITION) return SDO_STATUS_RANGE;
        ctx->pending_command.mode = (ServoMode)data[0];
        ctx->pending_valid = true;
        ctx->serial_watchdog_count_ms = 0;
        return SDO_STATUS_OK;
    case 0x2002:
    {
        uint16_t control;
        if (type != TSBP_DATA_U16 || size != 2U) return SDO_STATUS_TYPE;
        control = Tsbp_ReadU16(data);
        ctx->pending_command.enable = ((control & 0x0001U) != 0U) &&
                                      (ctx->state == TSBP_STATE_OP) &&
                                      (ctx->param->ControlSource == CONTROL_SOURCE_SERIAL_SDO);
        if ((control & 0x0004U) != 0U)
        {
            ctx->param->FaultCode = 0;
            if (ctx->state == TSBP_STATE_FAULT) ctx->state = TSBP_STATE_INIT;
        }
        ctx->pending_valid = true;
        ctx->serial_watchdog_count_ms = 0;
        return SDO_STATUS_OK;
    }
    case 0x2003:
        if (type != TSBP_DATA_I16 || size != 2U) return SDO_STATUS_TYPE;
        ctx->pending_command.target_current_mA = (int16_t)Tsbp_ReadU16(data);
        ctx->pending_valid = true;
        ctx->serial_watchdog_count_ms = 0;
        return SDO_STATUS_OK;
    case 0x2004:
        if (type != TSBP_DATA_I32 || size != 4U) return SDO_STATUS_TYPE;
        ctx->pending_command.target_speed = (int32_t)Tsbp_ReadU32(data);
        ctx->pending_valid = true;
        ctx->serial_watchdog_count_ms = 0;
        return SDO_STATUS_OK;
    case 0x2005:
        if (type != TSBP_DATA_I32 || size != 4U) return SDO_STATUS_TYPE;
        ctx->pending_command.target_position = (int32_t)Tsbp_ReadU32(data);
        ctx->pending_valid = true;
        ctx->serial_watchdog_count_ms = 0;
        return SDO_STATUS_OK;
    case 0x2608:
        if (type != TSBP_DATA_U8 || size != 1U) return SDO_STATUS_TYPE;
        if (data[0] > FAILSAFE_FALLBACK_PWM) return SDO_STATUS_RANGE;
        ctx->param->FailSafePolicy = data[0];
        return SDO_STATUS_OK;
    case 0x2800:
        if (type != TSBP_DATA_U8 || size != 1U) return SDO_STATUS_TYPE;
        if (data[0] == 1U)
        {
            ctx->save_request = true;
            return SDO_STATUS_OK;
        }
        return SDO_STATUS_RANGE;
    default:
        return SDO_STATUS_NOT_FOUND;
    }
}

static void Tsbp_HandleSdoRead(TsbpContext *ctx, uint8_t seq, const uint8_t *payload, uint16_t len)
{
    uint8_t data[8];
    uint8_t type = 0;
    uint8_t size = 0;
    uint16_t index;
    uint8_t subindex;
    uint8_t status;

    if (len != 4U)
    {
        Tsbp_SendError(ctx, seq, TSBP_ERR_BAD_LENGTH, 0, 0);
        return;
    }

    index = Tsbp_ReadU16(&payload[0]);
    subindex = payload[2];
    status = Tsbp_ReadObject(ctx, index, subindex, &type, &size, data);
    Tsbp_SendSdoReply(ctx, seq, index, subindex, type, size, status, data);
}

static void Tsbp_HandleSdoWrite(TsbpContext *ctx, uint8_t seq, const uint8_t *payload, uint16_t len)
{
    uint16_t index;
    uint8_t subindex;
    uint8_t type;
    uint8_t size;
    uint8_t status;

    if (len != 13U)
    {
        Tsbp_SendError(ctx, seq, TSBP_ERR_BAD_LENGTH, 0, 0);
        return;
    }

    index = Tsbp_ReadU16(&payload[0]);
    subindex = payload[2];
    type = payload[3];
    size = payload[4];
    if (size > 8U)
    {
        Tsbp_SendSdoReply(ctx, seq, index, subindex, type, size, SDO_STATUS_RANGE, NULL);
        return;
    }

    status = Tsbp_WriteObject(ctx, index, subindex, type, size, &payload[5]);
    Tsbp_SendSdoReply(ctx, seq, index, subindex, type, size, status, &payload[5]);
}

static void Tsbp_HandleManagementFrame(TsbpContext *ctx, const uint8_t *rx, uint16_t size)
{
    uint8_t type;
    uint8_t seq;
    uint8_t dst;
    uint16_t len;
    uint16_t rx_crc;
    uint16_t calc_crc;

    if (size < 12U || rx[0] != TSBP_SOF0 || rx[1] != TSBP_SOF1)
    {
        return;
    }

    len = Tsbp_ReadU16(&rx[8]);
    if (len > TSBP_MAX_PAYLOAD || size != (uint16_t)(12U + len))
    {
        return;
    }

    rx_crc = Tsbp_ReadU16(&rx[10U + len]);
    calc_crc = Tsbp_Crc16(&rx[2], (uint16_t)(8U + len));
    if (rx_crc != calc_crc)
    {
        return;
    }

    if (rx[2] != TSBP_VERSION)
    {
        Tsbp_SendError(ctx, rx[5], TSBP_ERR_UNSUPPORTED_VERSION, 0, 0);
        return;
    }

    type = rx[3];
    seq = rx[5];
    dst = rx[6];
    if (dst != ctx->node_id && dst != TSBP_BROADCAST_ID)
    {
        return;
    }

    if (type == TSBP_TYPE_SDO_READ)
    {
        Tsbp_HandleSdoRead(ctx, seq, &rx[10], len);
    }
    else if (type == TSBP_TYPE_SDO_WRITE)
    {
        Tsbp_HandleSdoWrite(ctx, seq, &rx[10], len);
    }
    else if (type == TSBP_TYPE_PING)
    {
        Tsbp_SendManagement(ctx, TSBP_TYPE_PING, seq, NULL, 0);
    }
    else
    {
        Tsbp_SendError(ctx, seq, TSBP_ERR_UNSUPPORTED_TYPE, 0, 0);
    }
}

static void Tsbp_WriteTxPdo(TsbpContext *ctx, uint8_t *payload)
{
    Tsbp_WriteU16(&payload[0], Tsbp_StatusWord(ctx));
    Tsbp_WriteU16(&payload[2], ctx->param->FaultCode);
    Tsbp_WriteU16(&payload[4], (uint16_t)ctx->param->INA181_mA);
    Tsbp_WriteU32(&payload[6], (uint32_t)ctx->param->EncoderSpeed);
    Tsbp_WriteU32(&payload[10], (uint32_t)ctx->param->EncoderMultiTurnValue);
    Tsbp_WriteU16(&payload[14], ctx->param->VCC_mV);
    payload[16] = (uint8_t)ctx->param->Temp;
}

static void Tsbp_ApplyRxPdo(TsbpContext *ctx, const uint8_t *pdo)
{
    uint16_t control = Tsbp_ReadU16(&pdo[0]);

    ctx->pending_command.mode = (pdo[2] <= SERVO_MODE_POSITION) ?
                                (ServoMode)pdo[2] : SERVO_MODE_CURRENT;
    ctx->pending_command.target_current_mA = (int16_t)Tsbp_ReadU16(&pdo[3]);
    ctx->pending_command.target_speed = (int32_t)Tsbp_ReadU32(&pdo[5]);
    ctx->pending_command.target_position = (int32_t)Tsbp_ReadU32(&pdo[9]);
    ctx->pending_command.enable = ((control & 0x0001U) != 0U) &&
                                  (ctx->state == TSBP_STATE_OP) &&
                                  (ctx->param->ControlSource == CONTROL_SOURCE_SERIAL_PDO);
    ctx->pending_valid = true;
    ctx->serial_watchdog_count_ms = 0;
    ctx->pdo_miss_count = 0;
}

static void Tsbp_SendFastPdo(TsbpContext *ctx, uint8_t seq, uint16_t cycle_id)
{
    uint8_t *tx = ctx->param->TxBuf;
    uint16_t crc;

    if (ctx->tx_busy)
    {
        return;
    }

    tx[0] = TSBP_FAST_SOF;
    tx[1] = seq;
    Tsbp_WriteU16(&tx[2], cycle_id);
    Tsbp_WriteTxPdo(ctx, &tx[4]);
    crc = Tsbp_Crc16(&tx[1], (uint16_t)(3U + TSBP_TX_PDO_SIZE));
    Tsbp_WriteU16(&tx[4U + TSBP_TX_PDO_SIZE], crc);

    ctx->tx_busy = true;
    if (HAL_UART_Transmit_DMA(ctx->huart, tx, (uint16_t)(6U + TSBP_TX_PDO_SIZE)) != HAL_OK)
    {
        ctx->tx_busy = false;
    }
}

static void Tsbp_SendFastProcess(TsbpContext *ctx, uint8_t seq, uint16_t cycle_id, uint16_t process_len)
{
    uint8_t *tx = ctx->param->TxBuf;
    uint16_t crc;

    if (ctx->tx_busy || process_len > (uint16_t)(sizeof(ctx->param->TxBuf) - 6U))
    {
        return;
    }

    tx[0] = TSBP_FAST_SOF;
    tx[1] = seq;
    Tsbp_WriteU16(&tx[2], cycle_id);
    crc = Tsbp_Crc16(&tx[1], (uint16_t)(3U + process_len));
    Tsbp_WriteU16(&tx[4U + process_len], crc);

    ctx->tx_busy = true;
    if (HAL_UART_Transmit_DMA(ctx->huart, tx, (uint16_t)(6U + process_len)) != HAL_OK)
    {
        ctx->tx_busy = false;
    }
}

static bool Tsbp_IsDuplicatePdoSeq(TsbpContext *ctx, uint8_t seq)
{
    if (ctx->pdo_seq_valid && ctx->last_seq == seq)
    {
        return true;
    }
    ctx->last_seq = seq;
    ctx->pdo_seq_valid = true;
    return false;
}

static void Tsbp_HandleParallelPdo(TsbpContext *ctx, const uint8_t *rx, uint16_t size)
{
    uint8_t seq;
    uint16_t cycle_id;
    uint16_t process_len;
    uint16_t pdo_offset;
    uint16_t rx_crc;
    uint16_t calc_crc;
    uint16_t reply_delay;

    process_len = (uint16_t)ctx->param->NodeCount * TSBP_RX_PDO_SIZE;
    if (size != (uint16_t)(6U + process_len))
    {
        return;
    }

    seq = rx[1];
    cycle_id = Tsbp_ReadU16(&rx[2]);
    rx_crc = Tsbp_ReadU16(&rx[4U + process_len]);
    calc_crc = Tsbp_Crc16(&rx[1], (uint16_t)(3U + process_len));
    if (rx_crc != calc_crc)
    {
        return;
    }

    if (!Tsbp_IsDuplicatePdoSeq(ctx, seq))
    {
        pdo_offset = (uint16_t)(ctx->param->NodePosition - 1U) * TSBP_RX_PDO_SIZE;
        Tsbp_ApplyRxPdo(ctx, &rx[4U + pdo_offset]);
    }

    reply_delay = (uint16_t)(ctx->param->NodePosition - 1U) * ctx->param->ReplySlotUs;
    if (reply_delay > TSBP_MAX_REPLY_DELAY_US)
    {
        reply_delay = TSBP_MAX_REPLY_DELAY_US;
    }
    Tsbp_DelayUs(reply_delay);
    Tsbp_SendFastPdo(ctx, seq, cycle_id);
}

static void Tsbp_HandleChainPdo(TsbpContext *ctx, const uint8_t *rx, uint16_t size)
{
    uint8_t seq;
    uint16_t cycle_id;
    uint8_t remaining_rx_count;
    uint8_t collected_tx_count;
    uint16_t in_process_len;
    uint16_t out_process_len;
    uint16_t keep_rx_len;
    uint16_t keep_tx_len;
    uint16_t rx_crc;
    uint16_t calc_crc;
    uint8_t *out;

    remaining_rx_count = (uint8_t)(ctx->param->NodeCount - ctx->param->NodePosition + 1U);
    collected_tx_count = (uint8_t)(ctx->param->NodePosition - 1U);
    in_process_len = (uint16_t)remaining_rx_count * TSBP_RX_PDO_SIZE +
                     (uint16_t)collected_tx_count * TSBP_TX_PDO_SIZE;

    if (size != (uint16_t)(6U + in_process_len))
    {
        return;
    }

    seq = rx[1];
    cycle_id = Tsbp_ReadU16(&rx[2]);
    rx_crc = Tsbp_ReadU16(&rx[4U + in_process_len]);
    calc_crc = Tsbp_Crc16(&rx[1], (uint16_t)(3U + in_process_len));
    if (rx_crc != calc_crc || ctx->tx_busy)
    {
        return;
    }

    if (!Tsbp_IsDuplicatePdoSeq(ctx, seq))
    {
        Tsbp_ApplyRxPdo(ctx, &rx[4]);
    }

    keep_rx_len = (uint16_t)(remaining_rx_count - 1U) * TSBP_RX_PDO_SIZE;
    keep_tx_len = (uint16_t)collected_tx_count * TSBP_TX_PDO_SIZE;
    out_process_len = (uint16_t)(keep_rx_len + keep_tx_len + TSBP_TX_PDO_SIZE);
    if (out_process_len > (uint16_t)(sizeof(ctx->param->TxBuf) - 6U))
    {
        return;
    }

    out = &ctx->param->TxBuf[4];
    if (keep_rx_len != 0U)
    {
        memcpy(out, &rx[4U + TSBP_RX_PDO_SIZE], keep_rx_len);
    }
    if (keep_tx_len != 0U)
    {
        memcpy(&out[keep_rx_len], &rx[4U + ((uint16_t)remaining_rx_count * TSBP_RX_PDO_SIZE)], keep_tx_len);
    }
    Tsbp_WriteTxPdo(ctx, &out[keep_rx_len + keep_tx_len]);
    Tsbp_SendFastProcess(ctx, seq, cycle_id, out_process_len);
}

static void Tsbp_HandleFastPdo(TsbpContext *ctx, const uint8_t *rx, uint16_t size)
{
    if (ctx->param->Topology == TSBP_TOPOLOGY_RING_CHAIN)
    {
        Tsbp_HandleChainPdo(ctx, rx, size);
    }
    else
    {
        Tsbp_HandleParallelPdo(ctx, rx, size);
    }
}

void Tsbp_Init(TsbpContext *ctx, UART_HandleTypeDef *huart, Param *param)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->huart = huart;
    ctx->param = param;
    ctx->state = TSBP_STATE_INIT;
    Tsbp_NormalizeBusConfig(param);
    ctx->node_id = (param->NodeId != 0U) ? param->NodeId : TSBP_DEFAULT_NODE_ID;
    ctx->active_command.mode = SERVO_MODE_CURRENT;
    ctx->active_command.enable = false;
    ctx->pending_command = ctx->active_command;
    param->ControlSource = CONTROL_SOURCE_PWM_INPUT;
    param->NodeId = ctx->node_id;
    Tsbp_InitReplyTimer();
    Tsbp_StartRx(ctx);
}

void Tsbp_RxEventCallback(TsbpContext *ctx, const UART_HandleTypeDef *huart, uint16_t size)
{
    uint8_t local_rx[sizeof(((Param *)0)->RxBuf)];

    if ((ctx == NULL) || (huart != ctx->huart))
    {
        return;
    }

    if (size <= sizeof(local_rx))
    {
        memcpy(local_rx, ctx->param->RxBuf, size);
    }
    Tsbp_StartRx(ctx);

    if (size > sizeof(local_rx))
    {
        return;
    }

    if (size >= 2U && local_rx[0] == TSBP_SOF0 && local_rx[1] == TSBP_SOF1)
    {
        Tsbp_HandleManagementFrame(ctx, local_rx, size);
    }
    else if (size >= 1U && local_rx[0] == TSBP_FAST_SOF)
    {
        Tsbp_HandleFastPdo(ctx, local_rx, size);
    }
}

void Tsbp_TxCpltCallback(TsbpContext *ctx, const UART_HandleTypeDef *huart)
{
    if ((ctx != NULL) && (huart == ctx->huart))
    {
        ctx->tx_busy = false;
    }
}

void Tsbp_1msTick(TsbpContext *ctx)
{
    uint16_t watchdog_ms;

    if (ctx == NULL)
    {
        return;
    }

    watchdog_ms = (ctx->param->SerialWatchdogMs != 0U) ? ctx->param->SerialWatchdogMs : 100U;

    if (ctx->param->ControlSource == CONTROL_SOURCE_SERIAL_SDO ||
        ctx->param->ControlSource == CONTROL_SOURCE_SERIAL_PDO)
    {
        if (ctx->serial_watchdog_count_ms < UINT16_MAX)
        {
            ctx->serial_watchdog_count_ms++;
        }
        if (ctx->serial_watchdog_count_ms > watchdog_ms)
        {
            ctx->param->FaultCode = TSBP_ERR_WATCHDOG;
            Tsbp_ApplyFailSafe(ctx);
            if (ctx->state == TSBP_STATE_OP)
            {
                ctx->state = TSBP_STATE_FAULT;
            }
        }
    }
    else
    {
        ctx->active_command.enable = false;
        ctx->pending_command.enable = false;
    }

    if (ctx->pending_valid)
    {
        ctx->active_command = ctx->pending_command;
        ctx->pending_valid = false;
    }
}

const ServoCommand *Tsbp_GetActiveCommand(const TsbpContext *ctx)
{
    static const ServoCommand disabled = {SERVO_MODE_CURRENT, false, 0, 0, 0};

    if (ctx == NULL)
    {
        return &disabled;
    }
    return &ctx->active_command;
}

bool Tsbp_ConsumeSaveRequest(TsbpContext *ctx)
{
    bool request = ctx->save_request;

    ctx->save_request = false;
    return request;
}
