/**
 * @file Dynamixel2.c
 * @brief TK Servo DYNAMIXEL Protocol 2.0 slave implementation.
 * @brief TK Servo DYNAMIXEL Protocol 2.0 从站实现。
 */

#include "Dynamixel2.h"
#include "PID.h"

#include <limits.h>
#include <string.h>

extern DMA_HandleTypeDef hdma_usart2_rx;
extern uint32_t SystemCoreClock;

enum
{
    DXL2_ERROR_NONE = 0x00,
    DXL2_ERROR_RESULT_FAIL = 0x01,
    DXL2_ERROR_INSTRUCTION = 0x02,
    DXL2_ERROR_DATA_RANGE = 0x04,
    DXL2_ERROR_DATA_LENGTH = 0x05,
    DXL2_ERROR_ACCESS = 0x07,
    DXL2_FAULT_WATCHDOG = 0x000AU
};

/*
 * Control Table layout (little-endian multi-byte values):
 * 控制表布局（多字节值均为小端序）：
 *   0..9    identity and persistent bus configuration / 身份与持久化总线配置
 *   16..33  runtime command image / 运行命令映像
 *   40..62  read-only feedback image / 只读反馈映像
 *   64..110 current, velocity, and position PID blocks / 电流、速度、位置 PID 块
 *   112..120 persistent protection and direction settings / 持久化保护与方向配置
 *   128..152 diagnostics, clear, and NVM-save commands / 诊断、清零与 NVM 保存命令
 * The exact public contract is documented in docs/dynamixel2/control-table.md.
 * 完整公开契约见 docs/dynamixel2/control-table.md。
 */

static uint16_t Dynamixel2_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t Dynamixel2_ReadU32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8)
           | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void Dynamixel2_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8);
}

static void Dynamixel2_WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
    data[2] = (uint8_t)((value >> 16) & 0xFFU);
    data[3] = (uint8_t)(value >> 24);
}

static void Dynamixel2_RecordDiagnostic(Dynamixel2Context *context, uint16_t code)
{
    if (context == NULL)
    {
        return;
    }
    context->last_diag_error = code;
    if (context->diagnostic_error_count < UINT32_MAX)
    {
        ++context->diagnostic_error_count;
    }
}

static void Dynamixel2_ResetWatchdog(Dynamixel2Context *context)
{
    context->serial_watchdog_count_ms = 0U;
    context->watchdog_latched = false;
}

static void Dynamixel2_ApplyFailSafe(Dynamixel2Context *context)
{
    /* The fail-safe changes the control source/output state, never a target alone. */
    /* 失联保护必须改变控制源或输出状态，不能只把目标值清零后继续保持使能。 */
    if (context->param->FailSafePolicy == FAILSAFE_BRAKE)
    {
        context->param->DriveRunMode = 1U;
        context->param->DrivePower = 0;
    }
    else if (context->param->FailSafePolicy == FAILSAFE_FALLBACK_PWM)
    {
        context->param->ControlSource = CONTROL_SOURCE_PWM_INPUT;
    }
    else
    {
        context->param->DriveRunMode = 0U;
        context->param->DrivePower = 0;
    }
    context->active_command.enable = false;
    context->pending_command.enable = false;
}

static uint16_t Dynamixel2_StatusWord(const Dynamixel2Context *context)
{
    /*
     * Bits: 0 ready, 2 enabled, 3 fault, 8 PWM source, 9 serial source,
     *       11 fault-free, 12 communication active.
     * 位定义：0 就绪，2 已使能，3 故障，8 PWM 源，9 串口源，11 无故障，12 通信活动。
     */
    uint16_t status = 1U;

    if (context->active_command.enable) status |= (1U << 2);
    if (context->param->FaultCode != 0U) status |= (1U << 3);
    if (context->param->ControlSource == CONTROL_SOURCE_PWM_INPUT) status |= (1U << 8);
    if (context->param->ControlSource == CONTROL_SOURCE_SERIAL) status |= (1U << 9);
    if (context->param->FaultCode == 0U) status |= (1U << 11);
    status |= (1U << 12);
    return status;
}

static uint8_t Dynamixel2_BaudCode(uint32_t baud)
{
    switch (baud)
    {
    case 115200UL: return 2U;
    case 1000000UL: return 3U;
    case 2000000UL: return 4U;
    default: return 2U;
    }
}

static bool Dynamixel2_BaudFromCode(uint8_t code, uint32_t *baud)
{
    switch (code)
    {
    case 2U: *baud = 115200UL; return true;
    case 3U: *baud = 1000000UL; return true;
    case 4U: *baud = 2000000UL; return true;
    default: return false;
    }
}

static PID_Int *Dynamixel2_PidForBase(Dynamixel2Context *context, uint16_t base)
{
    switch (base)
    {
    case 64U: return &context->param->Pid_PosEle;
    case 80U: return &context->param->Pid_PosVel;
    case 96U: return &context->param->Pid_Pos;
    default: return NULL;
    }
}

static void Dynamixel2_WritePidSnapshot(uint8_t *table, uint16_t base, const PID_Int *pid)
{
    Dynamixel2_WriteU16(&table[base], pid->Kp);
    Dynamixel2_WriteU16(&table[base + 2U], pid->Ki);
    Dynamixel2_WriteU16(&table[base + 4U], pid->Kd);
    Dynamixel2_WriteU32(&table[base + 6U], (uint32_t)pid->integral_max);
    Dynamixel2_WriteU16(&table[base + 10U], pid->out_max);
    Dynamixel2_WriteU16(&table[base + 12U], pid->out_min);
}

static void Dynamixel2_BuildControlTable(const Dynamixel2Context *context,
                                         uint8_t table[DXL2_CONTROL_TABLE_SIZE])
{
    /* Build one coherent snapshot so a multi-byte Read cannot mix update moments. */
    /* 每次读取先生成一致快照，避免多字节 Read 混入不同更新时刻的数据。 */
    memset(table, 0, DXL2_CONTROL_TABLE_SIZE);
    Dynamixel2_WriteU16(&table[0], DXL2_MODEL_NUMBER);
    table[2] = DXL2_FIRMWARE_VERSION;
    table[3] = 2U;
    table[4] = context->node_id;
    table[5] = Dynamixel2_BaudCode(context->param->BaudRate);
    Dynamixel2_WriteU16(&table[6], context->param->SerialWatchdogMs);
    table[8] = context->param->NodePosition;
    Dynamixel2_WriteU16(&table[9], context->param->ReplySlotUs);

    table[16] = context->param->ControlSource;
    table[17] = (uint8_t)context->pending_command.mode;
    Dynamixel2_WriteU16(&table[18], context->pending_command.enable ? 1U : 0U);
    Dynamixel2_WriteU16(&table[20], (uint16_t)context->pending_command.target_current_mA);
    Dynamixel2_WriteU32(&table[22], (uint32_t)context->pending_command.target_speed);
    Dynamixel2_WriteU32(&table[26], (uint32_t)context->pending_command.target_position);
    Dynamixel2_WriteU32(&table[30], context->execute_tick);

    Dynamixel2_WriteU16(&table[40], Dynamixel2_StatusWord(context));
    Dynamixel2_WriteU16(&table[42], context->param->FaultCode);
    Dynamixel2_WriteU16(&table[44], (uint16_t)context->param->CurrentLogical_mA);
    Dynamixel2_WriteU32(&table[46], (uint32_t)context->param->EncoderSpeed);
    Dynamixel2_WriteU32(&table[50], (uint32_t)context->param->EncoderValue);
    Dynamixel2_WriteU32(&table[54], (uint32_t)context->param->EncoderMultiTurnValue);
    Dynamixel2_WriteU16(&table[58], (uint16_t)context->param->DrivePower);
    Dynamixel2_WriteU16(&table[60], context->param->VCC_mV);
    table[62] = (uint8_t)context->param->Temp;

    Dynamixel2_WritePidSnapshot(table, 64U, &context->param->Pid_PosEle);
    Dynamixel2_WritePidSnapshot(table, 80U, &context->param->Pid_PosVel);
    Dynamixel2_WritePidSnapshot(table, 96U, &context->param->Pid_Pos);
    table[112] = (uint8_t)context->param->TempLimit;
    Dynamixel2_WriteU16(&table[114], context->param->SpeedMax);
    table[116] = context->param->DrivePwmMode;
    table[117] = context->param->EncoderVeer ? 1U : 0U;
    Dynamixel2_WriteU16(&table[118], context->param->EncoderOffset);
    table[120] = context->param->FailSafePolicy;

    Dynamixel2_WriteU16(&table[128], context->last_diag_error);
    Dynamixel2_WriteU32(&table[130], context->diagnostic_error_count);
    Dynamixel2_WriteU32(&table[134], context->uart_error_count);
    Dynamixel2_WriteU32(&table[138], context->rx_crc_error_count);
    Dynamixel2_WriteU32(&table[142], context->rx_bad_packet_count);
    Dynamixel2_WriteU32(&table[146], context->rx_packet_count);
}

static uint8_t Dynamixel2_ReadTable(Dynamixel2Context *context, uint16_t address,
                                    uint16_t length, uint8_t *output)
{
    uint8_t table[DXL2_CONTROL_TABLE_SIZE];

    if (length == 0U || output == NULL || address >= DXL2_CONTROL_TABLE_SIZE
        || length > (uint16_t)(DXL2_CONTROL_TABLE_SIZE - address))
    {
        return DXL2_ERROR_DATA_RANGE;
    }
    Dynamixel2_BuildControlTable(context, table);
    memcpy(output, &table[address], length);
    return DXL2_ERROR_NONE;
}

static uint8_t Dynamixel2_WritePid(Dynamixel2Context *context, uint16_t address,
                                   const uint8_t *data, uint16_t length, bool apply)
{
    uint16_t base = (uint16_t)(address & 0xFFF0U);
    uint16_t offset = (uint16_t)(address - base);
    PID_Int *pid = Dynamixel2_PidForBase(context, base);

    /* Reg Write validates with apply=false; Action later commits the same checked bytes. */
    /* Reg Write 以 apply=false 预校验；Action 随后提交同一份已检查数据。 */
    if (pid == NULL)
    {
        return DXL2_ERROR_ACCESS;
    }
    if ((offset == 0U || offset == 2U || offset == 4U
         || offset == 10U || offset == 12U) && length == 2U)
    {
        if (apply)
        {
            uint16_t value = Dynamixel2_ReadU16(data);
            if (offset == 0U) pid->Kp = value;
            else if (offset == 2U) pid->Ki = value;
            else if (offset == 4U) pid->Kd = value;
            else if (offset == 10U) pid->out_max = value;
            else pid->out_min = value;
        }
        return DXL2_ERROR_NONE;
    }
    if (offset == 6U && length == 4U)
    {
        int32_t value = (int32_t)Dynamixel2_ReadU32(data);
        if (value < 0)
        {
            return DXL2_ERROR_DATA_RANGE;
        }
        if (apply) pid->integral_max = value;
        return DXL2_ERROR_NONE;
    }
    if (offset == 14U && length == 1U)
    {
        if (data[0] != 1U) return DXL2_ERROR_DATA_RANGE;
        if (apply) PID_Reset(pid);
        return DXL2_ERROR_NONE;
    }
    return DXL2_ERROR_DATA_LENGTH;
}

static bool Dynamixel2_IsCommandAddress(uint16_t address)
{
    return address == 16U || address == 17U || address == 18U || address == 20U
           || address == 22U || address == 26U || address == 30U;
}

static uint8_t Dynamixel2_WriteCommandBlock(Dynamixel2Context *context,
                                            const uint8_t *data, bool apply)
{
    uint8_t source = data[0];
    uint8_t mode = data[1];
    uint16_t control_word = Dynamixel2_ReadU16(&data[2]);
    bool clear_fault = (control_word & 0x0004U) != 0U;

    if (source > CONTROL_SOURCE_PWM_INPUT || mode > SERVO_MODE_POSITION)
    {
        return DXL2_ERROR_DATA_RANGE;
    }
    if (!apply)
    {
        return DXL2_ERROR_NONE;
    }

    /* A 14-byte block is the atomic command image used by Sync Write. */
    /* 14 字节块是供 Sync Write 使用的原子命令映像。 */
    if (context->param->ControlSource != source)
    {
        /* Source changes are always disarmed; a later Control Word must enable output. */
        /* 切换控制源时始终撤销使能；必须随后再次写控制字才能使能输出。 */
        context->active_command.enable = false;
        context->pending_command.enable = false;
    }
    context->param->ControlSource = source;
    context->pending_command.mode = (ServoMode)mode;
    context->pending_command.target_current_mA = (int16_t)Dynamixel2_ReadU16(&data[4]);
    context->pending_command.target_speed = (int32_t)Dynamixel2_ReadU32(&data[6]);
    context->pending_command.target_position = (int32_t)Dynamixel2_ReadU32(&data[10]);
    if (clear_fault)
    {
        context->param->FaultCode = 0U;
    }
    /* Fault clear is always disarmed; enabling requires a later explicit write. */
    /* 清故障写入始终保持未使能；使能必须由后续独立写入显式触发。 */
    context->pending_command.enable = !clear_fault
                                      && ((control_word & 0x0001U) != 0U)
                                      && source == CONTROL_SOURCE_SERIAL
                                      && context->param->FaultCode == 0U;
    context->pending_valid = true;
    Dynamixel2_ResetWatchdog(context);
    return DXL2_ERROR_NONE;
}

static uint8_t Dynamixel2_WriteCommand(Dynamixel2Context *context, uint16_t address,
                                       const uint8_t *data, uint16_t length, bool apply)
{
    if (address == 16U && length == 14U)
    {
        return Dynamixel2_WriteCommandBlock(context, data, apply);
    }
    if (!Dynamixel2_IsCommandAddress(address))
    {
        return DXL2_ERROR_ACCESS;
    }

    switch (address)
    {
    case 16U:
        if (length != 1U) return DXL2_ERROR_DATA_LENGTH;
        if (data[0] > CONTROL_SOURCE_PWM_INPUT) return DXL2_ERROR_DATA_RANGE;
        if (apply)
        {
            if (context->param->ControlSource != data[0])
            {
                context->active_command.enable = false;
                context->pending_command.enable = false;
            }
            context->param->ControlSource = data[0];
            Dynamixel2_ResetWatchdog(context);
        }
        return DXL2_ERROR_NONE;
    case 17U:
        if (length != 1U) return DXL2_ERROR_DATA_LENGTH;
        if (data[0] > SERVO_MODE_POSITION) return DXL2_ERROR_DATA_RANGE;
        if (apply)
        {
            context->pending_command.mode = (ServoMode)data[0];
            context->pending_valid = true;
            Dynamixel2_ResetWatchdog(context);
        }
        return DXL2_ERROR_NONE;
    case 18U:
    {
        uint16_t control_word;
        bool clear_fault;
        if (length != 2U) return DXL2_ERROR_DATA_LENGTH;
        control_word = Dynamixel2_ReadU16(data);
        clear_fault = (control_word & 0x0004U) != 0U;
        if (apply)
        {
            if (clear_fault)
            {
                context->param->FaultCode = 0U;
            }
            context->pending_command.enable = !clear_fault
                                              && ((control_word & 0x0001U) != 0U)
                                              && context->param->ControlSource == CONTROL_SOURCE_SERIAL
                                              && context->param->FaultCode == 0U;
            context->pending_valid = true;
            Dynamixel2_ResetWatchdog(context);
        }
        return DXL2_ERROR_NONE;
    }
    case 20U:
        if (length != 2U) return DXL2_ERROR_DATA_LENGTH;
        if (apply)
        {
            context->pending_command.target_current_mA = (int16_t)Dynamixel2_ReadU16(data);
            context->pending_valid = true;
            Dynamixel2_ResetWatchdog(context);
        }
        return DXL2_ERROR_NONE;
    case 22U:
        if (length != 4U) return DXL2_ERROR_DATA_LENGTH;
        if (apply)
        {
            context->pending_command.target_speed = (int32_t)Dynamixel2_ReadU32(data);
            context->pending_valid = true;
            Dynamixel2_ResetWatchdog(context);
        }
        return DXL2_ERROR_NONE;
    case 26U:
        if (length != 4U) return DXL2_ERROR_DATA_LENGTH;
        if (apply)
        {
            context->pending_command.target_position = (int32_t)Dynamixel2_ReadU32(data);
            context->pending_valid = true;
            Dynamixel2_ResetWatchdog(context);
        }
        return DXL2_ERROR_NONE;
    case 30U:
        if (length != 4U) return DXL2_ERROR_DATA_LENGTH;
        if (apply) context->execute_tick = Dynamixel2_ReadU32(data);
        return DXL2_ERROR_NONE;
    default:
        return DXL2_ERROR_ACCESS;
    }
}

static uint8_t Dynamixel2_WriteTable(Dynamixel2Context *context, uint16_t address,
                                     const uint8_t *data, uint16_t length, bool apply)
{
    uint32_t baud;

    if (data == NULL || length == 0U)
    {
        return DXL2_ERROR_DATA_LENGTH;
    }
    if (Dynamixel2_IsCommandAddress(address))
    {
        return Dynamixel2_WriteCommand(context, address, data, length, apply);
    }
    if (address >= 64U && address < 111U)
    {
        return Dynamixel2_WritePid(context, address, data, length, apply);
    }

    switch (address)
    {
    case 4U:
        if (length != 1U) return DXL2_ERROR_DATA_LENGTH;
        if (data[0] == 0U || data[0] > 0xFCU) return DXL2_ERROR_DATA_RANGE;
        if (apply)
        {
            context->node_id = data[0];
            context->param->NodeId = data[0];
        }
        return DXL2_ERROR_NONE;
    case 5U:
        if (length != 1U) return DXL2_ERROR_DATA_LENGTH;
        if (!Dynamixel2_BaudFromCode(data[0], &baud)) return DXL2_ERROR_DATA_RANGE;
        if (apply) context->param->BaudRate = baud;
        return DXL2_ERROR_NONE;
    case 6U:
        if (length != 2U) return DXL2_ERROR_DATA_LENGTH;
        if (apply) context->param->SerialWatchdogMs = Dynamixel2_ReadU16(data);
        return DXL2_ERROR_NONE;
    case 8U:
        if (length != 1U) return DXL2_ERROR_DATA_LENGTH;
        if (data[0] == 0U) return DXL2_ERROR_DATA_RANGE;
        if (apply) context->param->NodePosition = data[0];
        return DXL2_ERROR_NONE;
    case 9U:
        if (length != 2U) return DXL2_ERROR_DATA_LENGTH;
        if (Dynamixel2_ReadU16(data) < 50U) return DXL2_ERROR_DATA_RANGE;
        if (apply) context->param->ReplySlotUs = Dynamixel2_ReadU16(data);
        return DXL2_ERROR_NONE;
    case 112U:
        if (length != 1U) return DXL2_ERROR_DATA_LENGTH;
        if ((int8_t)data[0] < 20 || (int8_t)data[0] > 85) return DXL2_ERROR_DATA_RANGE;
        if (apply) context->param->TempLimit = (int8_t)data[0];
        return DXL2_ERROR_NONE;
    case 114U:
        if (length != 2U) return DXL2_ERROR_DATA_LENGTH;
        if (Dynamixel2_ReadU16(data) < 1000U) return DXL2_ERROR_DATA_RANGE;
        if (apply) context->param->SpeedMax = Dynamixel2_ReadU16(data);
        return DXL2_ERROR_NONE;
    case 116U:
        if (length != 1U) return DXL2_ERROR_DATA_LENGTH;
        if (data[0] != 2U && data[0] != 3U && data[0] != 4U)
            return DXL2_ERROR_DATA_RANGE;
        if (apply) context->param->DrivePwmMode = data[0];
        return DXL2_ERROR_NONE;
    case 117U:
        if (length != 1U) return DXL2_ERROR_DATA_LENGTH;
        if (data[0] > 1U) return DXL2_ERROR_DATA_RANGE;
        if (context->active_command.enable || context->pending_command.enable)
            return DXL2_ERROR_ACCESS;
        if (apply)
        {
            context->param->EncoderVeer = data[0] != 0U;
            context->param->EncoderRebaseline = true;
        }
        return DXL2_ERROR_NONE;
    case 118U:
        if (length != 2U) return DXL2_ERROR_DATA_LENGTH;
        if (Dynamixel2_ReadU16(data) >= 16384U) return DXL2_ERROR_DATA_RANGE;
        if (apply)
        {
            context->param->EncoderOffset = Dynamixel2_ReadU16(data);
            context->param->EncoderRebaseline = true;
        }
        return DXL2_ERROR_NONE;
    case 120U:
        if (length != 1U) return DXL2_ERROR_DATA_LENGTH;
        if (data[0] > FAILSAFE_FALLBACK_PWM) return DXL2_ERROR_DATA_RANGE;
        if (apply) context->param->FailSafePolicy = data[0];
        return DXL2_ERROR_NONE;
    case 150U:
        if (length != 1U) return DXL2_ERROR_DATA_LENGTH;
        if (data[0] != 1U) return DXL2_ERROR_DATA_RANGE;
        if (apply)
        {
            context->last_diag_error = DXL2_DIAG_NONE;
            context->diagnostic_error_count = 0U;
            context->last_uart_error = 0U;
            context->uart_error_count = 0U;
            context->rx_packet_count = 0U;
            context->rx_crc_error_count = 0U;
            context->rx_bad_packet_count = 0U;
            context->tx_drop_count = 0U;
        }
        return DXL2_ERROR_NONE;
    case 152U:
        if (length != 1U) return DXL2_ERROR_DATA_LENGTH;
        if (data[0] != 1U) return DXL2_ERROR_DATA_RANGE;
        /* Flash programming is deferred to the main loop, never performed in UART callback context. */
        /* Flash 写入延迟到主循环执行，绝不在 UART 回调上下文中编程。 */
        if (apply) context->save_request = true;
        return DXL2_ERROR_NONE;
    default:
        return DXL2_ERROR_ACCESS;
    }
}

static bool Dynamixel2_StartRx(Dynamixel2Context *context)
{
    if (context == NULL || context->huart == NULL || context->param == NULL)
    {
        return false;
    }
    /* Param.RxBuf belongs to DMA until an idle/error callback snapshots it. */
    /* Param.RxBuf 在空闲或错误回调取得快照前由 DMA 独占。 */
    context->rx_active = false;
    if (HAL_UARTEx_ReceiveToIdle_DMA(context->huart, context->param->RxBuf,
                                     sizeof(context->param->RxBuf)) != HAL_OK)
    {
        Dynamixel2_RecordDiagnostic(context, DXL2_DIAG_UART_ERROR);
        return false;
    }
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    context->rx_active = true;
    return true;
}

static void Dynamixel2_InitReplyTimer(void)
{
    /* TIM1 is reserved here as a free-running 1 MHz reply-slot clock. */
    /* TIM1 在此专用于 1 MHz 自由运行的回复时隙时钟。 */
    uint32_t prescaler = SystemCoreClock / 1000000UL;
    if (prescaler == 0UL) prescaler = 1UL;
    TIM1->PSC = (uint16_t)(prescaler - 1UL);
    TIM1->ARR = 0xFFFFU;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;
}

static void Dynamixel2_DelayUs(uint32_t delay_us)
{
    /* Used only in request-triggered response slots, never from the 1 ms control path. */
    /* 仅用于请求触发的响应时隙，不在 1 ms 控制路径中调用。 */
    while (delay_us != 0UL)
    {
        uint16_t chunk = (delay_us > 60000UL) ? 60000U : (uint16_t)delay_us;
        uint16_t start = (uint16_t)TIM1->CNT;
        while ((uint16_t)((uint16_t)TIM1->CNT - start) < chunk)
        {
        }
        delay_us -= chunk;
    }
}

static bool Dynamixel2_StartTx(Dynamixel2Context *context, uint8_t *data,
                               uint16_t length)
{
    if (context == NULL || data == NULL || length == 0U || context->huart == NULL)
    {
        return false;
    }
    /* DMA borrows the buffer until Dynamixel2_TxCpltCallback releases tx_busy. */
    /* DMA 借用该缓冲区，直到 Dynamixel2_TxCpltCallback 释放 tx_busy。 */
    context->tx_busy = true;
    if (HAL_UART_Transmit_DMA(context->huart, data, length) != HAL_OK)
    {
        context->tx_busy = false;
        if (context->tx_drop_count < UINT32_MAX) ++context->tx_drop_count;
        Dynamixel2_RecordDiagnostic(context, DXL2_DIAG_TX_DROP);
        return false;
    }
    return true;
}

static void Dynamixel2_TrySendPending(Dynamixel2Context *context)
{
    if (context == NULL || context->tx_busy || !context->pending_tx_valid
        || context->pending_tx_length == 0U)
    {
        return;
    }
    memcpy(context->param->TxBuf, context->pending_tx, context->pending_tx_length);
    if (Dynamixel2_StartTx(context, context->param->TxBuf,
                           context->pending_tx_length))
    {
        context->pending_tx_valid = false;
        context->pending_tx_length = 0U;
    }
}

static void Dynamixel2_SendStatus(Dynamixel2Context *context, uint8_t error,
                                  const uint8_t *parameters, uint16_t length,
                                  uint32_t delay_us)
{
    uint8_t packet[DXL2_MAX_PACKET_SIZE];
    uint16_t packet_length = Dxl2_EncodeStatus(packet, sizeof(packet), context->node_id,
                                               error, parameters, length);
    if (packet_length == 0U)
    {
        return;
    }
    if (delay_us != 0UL)
    {
        Dynamixel2_DelayUs(delay_us);
    }
    Dynamixel2_TrySendPending(context);
    /* One bounded pending slot prevents callbacks from blocking on an active DMA TX. */
    /* 单级有界待发槽避免回调在 DMA 发送期间阻塞。 */
    if (context->tx_busy)
    {
        if (context->pending_tx_valid)
        {
            if (context->tx_drop_count < UINT32_MAX) ++context->tx_drop_count;
            Dynamixel2_RecordDiagnostic(context, DXL2_DIAG_TX_DROP);
            return;
        }
        memcpy(context->pending_tx, packet, packet_length);
        context->pending_tx_length = packet_length;
        context->pending_tx_valid = true;
        return;
    }
    memcpy(context->param->TxBuf, packet, packet_length);
    (void)Dynamixel2_StartTx(context, context->param->TxBuf, packet_length);
}

static uint32_t Dynamixel2_ReplyDelay(const Dynamixel2Context *context,
                                      uint16_t order, uint16_t data_length)
{
    uint32_t baud = context->param->BaudRate != 0UL
                        ? context->param->BaudRate : 115200UL;
    /* Include start/stop bits and a guard; a configured slot can only be enlarged. */
    /* 计入起止位和保护间隔；配置的时隙只允许自动增大，不能压缩到包时长以下。 */
    uint32_t packet_time_us = (((uint32_t)(11U + data_length) * 10UL * 1000000UL)
                               + baud - 1UL) / baud + 50UL;
    uint32_t slot_us = context->param->ReplySlotUs;
    if (slot_us < packet_time_us) slot_us = packet_time_us;
    return (uint32_t)order * slot_us;
}

static void Dynamixel2_HandleRead(Dynamixel2Context *context,
                                  const Dxl2Packet *packet)
{
    uint8_t data[DXL2_MAX_PARAMETERS];
    uint16_t address;
    uint16_t length;
    uint8_t error;

    if (packet->parameter_length != 4U)
    {
        Dynamixel2_SendStatus(context, DXL2_ERROR_DATA_LENGTH, NULL, 0U, 0U);
        return;
    }
    address = Dynamixel2_ReadU16(&packet->parameters[0]);
    length = Dynamixel2_ReadU16(&packet->parameters[2]);
    if (length > sizeof(data))
    {
        Dynamixel2_SendStatus(context, DXL2_ERROR_DATA_LENGTH, NULL, 0U, 0U);
        return;
    }
    error = Dynamixel2_ReadTable(context, address, length, data);
    Dynamixel2_SendStatus(context, error, error == 0U ? data : NULL,
                          error == 0U ? length : 0U, 0U);
}

static void Dynamixel2_HandleWrite(Dynamixel2Context *context,
                                   const Dxl2Packet *packet, bool registered)
{
    uint16_t address;
    uint16_t data_length;
    uint8_t error;

    if (packet->parameter_length < 3U)
    {
        Dynamixel2_SendStatus(context, DXL2_ERROR_DATA_LENGTH, NULL, 0U, 0U);
        return;
    }
    address = Dynamixel2_ReadU16(packet->parameters);
    data_length = (uint16_t)(packet->parameter_length - 2U);
    error = Dynamixel2_WriteTable(context, address, &packet->parameters[2],
                                  data_length, !registered);
    if (error == 0U && registered)
    {
        context->registered_address = address;
        context->registered_length = data_length;
        memcpy(context->registered_data, &packet->parameters[2], data_length);
        context->registered_write_valid = true;
    }
    if (packet->id != DXL2_BROADCAST_ID)
    {
        Dynamixel2_SendStatus(context, error, NULL, 0U, 0U);
    }
}

static void Dynamixel2_HandleAction(Dynamixel2Context *context,
                                    const Dxl2Packet *packet)
{
    uint8_t error = DXL2_ERROR_NONE;
    if (context->registered_write_valid)
    {
        error = Dynamixel2_WriteTable(context, context->registered_address,
                                      context->registered_data,
                                      context->registered_length, true);
        context->registered_write_valid = false;
    }
    if (packet->id != DXL2_BROADCAST_ID)
    {
        Dynamixel2_SendStatus(context, error, NULL, 0U, 0U);
    }
}

static void Dynamixel2_HandleSyncWrite(Dynamixel2Context *context,
                                       const Dxl2Packet *packet)
{
    uint16_t address;
    uint16_t data_length;
    uint16_t offset;

    if (packet->parameter_length < 5U)
    {
        return;
    }
    address = Dynamixel2_ReadU16(&packet->parameters[0]);
    data_length = Dynamixel2_ReadU16(&packet->parameters[2]);
    if (data_length == 0U
        || ((uint16_t)(packet->parameter_length - 4U)
            % (uint16_t)(data_length + 1U)) != 0U)
    {
        return;
    }
    /* Every item is ID followed by one fixed-width data block; broadcast writes never reply. */
    /* 每项由 ID 和固定宽度数据块组成；广播写入绝不回复。 */
    for (offset = 4U; offset + 1U + data_length <= packet->parameter_length;
         offset = (uint16_t)(offset + 1U + data_length))
    {
        if (packet->parameters[offset] == context->node_id)
        {
            (void)Dynamixel2_WriteTable(context, address,
                                        &packet->parameters[offset + 1U],
                                        data_length, true);
            return;
        }
    }
}

static void Dynamixel2_HandleSyncRead(Dynamixel2Context *context,
                                      const Dxl2Packet *packet)
{
    uint8_t data[DXL2_MAX_PARAMETERS];
    uint16_t address;
    uint16_t data_length;
    uint16_t index;
    uint8_t error;

    if (packet->parameter_length < 5U)
    {
        return;
    }
    address = Dynamixel2_ReadU16(&packet->parameters[0]);
    data_length = Dynamixel2_ReadU16(&packet->parameters[2]);
    if (data_length == 0U || data_length > sizeof(data))
    {
        return;
    }
    /* The request's ID-list order defines collision-free response order on the diode-OR bus. */
    /* 在二极管汇线总线上，请求中的 ID 列表顺序决定无冲突的回复顺序。 */
    for (index = 4U; index < packet->parameter_length; ++index)
    {
        if (packet->parameters[index] == context->node_id)
        {
            error = Dynamixel2_ReadTable(context, address, data_length, data);
            Dynamixel2_SendStatus(context, error, error == 0U ? data : NULL,
                                  error == 0U ? data_length : 0U,
                                  Dynamixel2_ReplyDelay(context,
                                                        (uint16_t)(index - 4U),
                                                        data_length));
            return;
        }
    }
}

static void Dynamixel2_HandlePacket(Dynamixel2Context *context,
                                    const Dxl2Packet *packet)
{
    bool addressed = packet->id == context->node_id
                     || packet->id == DXL2_BROADCAST_ID;
    if (!addressed || packet->instruction == DXL2_INST_STATUS)
    {
        return;
    }

    switch (packet->instruction)
    {
    case DXL2_INST_PING:
    {
        uint8_t identity[3];
        Dynamixel2_WriteU16(identity, DXL2_MODEL_NUMBER);
        identity[2] = DXL2_FIRMWARE_VERSION;
        Dynamixel2_SendStatus(context, DXL2_ERROR_NONE, identity, sizeof(identity),
                              packet->id == DXL2_BROADCAST_ID
                                  ? Dynamixel2_ReplyDelay(context,
                                        context->param->NodePosition > 0U
                                            ? (uint16_t)(context->param->NodePosition - 1U) : 0U,
                                        sizeof(identity))
                                  : 0U);
        break;
    }
    case DXL2_INST_READ:
        if (packet->id != DXL2_BROADCAST_ID) Dynamixel2_HandleRead(context, packet);
        break;
    case DXL2_INST_WRITE:
        Dynamixel2_HandleWrite(context, packet, false);
        break;
    case DXL2_INST_REG_WRITE:
        Dynamixel2_HandleWrite(context, packet, true);
        break;
    case DXL2_INST_ACTION:
        Dynamixel2_HandleAction(context, packet);
        break;
    case DXL2_INST_SYNC_READ:
        if (packet->id == DXL2_BROADCAST_ID) Dynamixel2_HandleSyncRead(context, packet);
        break;
    case DXL2_INST_SYNC_WRITE:
        if (packet->id == DXL2_BROADCAST_ID) Dynamixel2_HandleSyncWrite(context, packet);
        break;
    default:
        if (packet->id != DXL2_BROADCAST_ID)
        {
            Dynamixel2_SendStatus(context, DXL2_ERROR_INSTRUCTION, NULL, 0U, 0U);
        }
        break;
    }
}

static void Dynamixel2_DiscardStreamPrefix(Dynamixel2Context *context,
                                           uint16_t count)
{
    if (count >= context->rx_stream_length)
    {
        context->rx_stream_length = 0U;
        return;
    }
    memmove(context->rx_stream, &context->rx_stream[count],
            (uint16_t)(context->rx_stream_length - count));
    context->rx_stream_length = (uint16_t)(context->rx_stream_length - count);
}

static void Dynamixel2_ConsumeStream(Dynamixel2Context *context)
{
    /* Decode is bounded by DXL2_RX_STREAM_SIZE and consumes at most the buffered bytes. */
    /* 解码工作受 DXL2_RX_STREAM_SIZE 限制，最多处理当前已缓冲数据。 */
    while (context->rx_stream_length != 0U)
    {
        Dxl2DecodeResult result = Dxl2_DecodePacket(context->rx_stream,
                                                     context->rx_stream_length);
        if (result.status == DXL2_DECODE_INCOMPLETE)
        {
            return;
        }
        if (result.status == DXL2_DECODE_OK)
        {
            if (context->rx_packet_count < UINT32_MAX) ++context->rx_packet_count;
            Dynamixel2_HandlePacket(context, &result.packet);
        }
        else
        {
            if (result.status == DXL2_DECODE_BAD_CRC)
            {
                if (context->rx_crc_error_count < UINT32_MAX) ++context->rx_crc_error_count;
                Dynamixel2_RecordDiagnostic(context, DXL2_DIAG_RX_CRC);
            }
            else
            {
                if (context->rx_bad_packet_count < UINT32_MAX) ++context->rx_bad_packet_count;
                Dynamixel2_RecordDiagnostic(context, DXL2_DIAG_RX_BAD_PACKET);
            }
        }
        if (result.consumed == 0U)
        {
            return;
        }
        Dynamixel2_DiscardStreamPrefix(context, result.consumed);
    }
}

void Dynamixel2_Init(Dynamixel2Context *context, UART_HandleTypeDef *huart, Param *param)
{
    if (context == NULL || param == NULL)
    {
        return;
    }
    memset(context, 0, sizeof(*context));
    context->huart = huart;
    context->param = param;
    context->node_id = (param->NodeId >= 1U && param->NodeId <= 0xFCU)
                           ? param->NodeId : 1U;
    context->active_command.mode = SERVO_MODE_CURRENT;
    context->active_command.enable = false;
    context->pending_command = context->active_command;
    /* NVM never restores a live serial command: startup is disarmed and returns to PWM input. */
    /* NVM 绝不恢复活动串口命令：上电保持未使能，并回到 PWM 输入。 */
    param->ControlSource = CONTROL_SOURCE_PWM_INPUT;
    param->NodeId = context->node_id;
    if (param->NodePosition == 0U) param->NodePosition = 1U;
    if (param->ReplySlotUs < 50U) param->ReplySlotUs = 120U;
    Dynamixel2_InitReplyTimer();
    (void)Dynamixel2_StartRx(context);
}

bool Dynamixel2_RestartRx(Dynamixel2Context *context)
{
    return Dynamixel2_StartRx(context);
}

void Dynamixel2_RecordUartError(Dynamixel2Context *context, uint32_t error_code)
{
    if (context == NULL)
    {
        return;
    }
    context->last_uart_error = error_code;
    if (context->uart_error_count < UINT32_MAX) ++context->uart_error_count;
    Dynamixel2_RecordDiagnostic(context, DXL2_DIAG_UART_ERROR);
}

void Dynamixel2_RxEventCallback(Dynamixel2Context *context,
                                const UART_HandleTypeDef *huart, uint16_t size)
{
    uint8_t received[sizeof(((Param *)0)->RxBuf)];

    if (context == NULL || huart != context->huart || context->param == NULL)
    {
        return;
    }
    /* Snapshot DMA bytes before re-arming because the next transfer reuses Param.RxBuf. */
    /* 重启 DMA 会复用 Param.RxBuf，因此必须先复制本次接收数据。 */
    if (size <= sizeof(received))
    {
        memcpy(received, context->param->RxBuf, size);
    }
    (void)Dynamixel2_StartRx(context);
    if (size > sizeof(received)
        || size > (uint16_t)(sizeof(context->rx_stream) - context->rx_stream_length))
    {
        context->rx_stream_length = 0U;
        if (context->rx_bad_packet_count < UINT32_MAX) ++context->rx_bad_packet_count;
        Dynamixel2_RecordDiagnostic(context, DXL2_DIAG_RX_BAD_PACKET);
        return;
    }
    memcpy(&context->rx_stream[context->rx_stream_length], received, size);
    context->rx_stream_length = (uint16_t)(context->rx_stream_length + size);
    Dynamixel2_ConsumeStream(context);
}

void Dynamixel2_TxCpltCallback(Dynamixel2Context *context,
                               const UART_HandleTypeDef *huart)
{
    if (context != NULL && huart == context->huart)
    {
        context->tx_busy = false;
        Dynamixel2_TrySendPending(context);
    }
}

void Dynamixel2_1msTick(Dynamixel2Context *context)
{
    uint16_t watchdog_ms;
    if (context == NULL || context->param == NULL)
    {
        return;
    }
    ++context->tick_ms;
    if (!context->rx_active) (void)Dynamixel2_StartRx(context);
    Dynamixel2_TrySendPending(context);

    /* Only an explicitly enabled serial owner must refresh the watchdog. */
    /* 只有显式使能的串口控制所有者需要刷新看门狗。 */
    watchdog_ms = context->param->SerialWatchdogMs;
    if (context->param->ControlSource == CONTROL_SOURCE_SERIAL
        && (context->active_command.enable || context->pending_command.enable)
        && watchdog_ms != 0U)
    {
        if (context->serial_watchdog_count_ms < UINT16_MAX)
            ++context->serial_watchdog_count_ms;
        if (context->serial_watchdog_count_ms > watchdog_ms && !context->watchdog_latched)
        {
            context->watchdog_latched = true;
            context->param->FaultCode = DXL2_FAULT_WATCHDOG;
            Dynamixel2_RecordDiagnostic(context, DXL2_DIAG_WATCHDOG);
            Dynamixel2_ApplyFailSafe(context);
        }
    }
    else if (context->param->ControlSource != CONTROL_SOURCE_SERIAL)
    {
        Dynamixel2_ResetWatchdog(context);
        context->active_command.enable = false;
        context->pending_command.enable = false;
    }

    /* Signed subtraction preserves due-time ordering across the 32-bit tick wrap. */
    /* 使用有符号差值，在 32 位 tick 回绕时仍保持到期判断正确。 */
    if (context->pending_valid
        && (context->execute_tick == 0UL
            || (int32_t)(context->tick_ms - context->execute_tick) >= 0))
    {
        context->active_command = context->pending_command;
        context->pending_valid = false;
        context->execute_tick = 0UL;
    }
}

const ServoCommand *Dynamixel2_GetActiveCommand(const Dynamixel2Context *context)
{
    static const ServoCommand disabled = {SERVO_MODE_CURRENT, false, 0, 0, 0};
    return context != NULL ? &context->active_command : &disabled;
}

bool Dynamixel2_ConsumeSaveRequest(Dynamixel2Context *context)
{
    bool request;
    if (context == NULL)
    {
        return false;
    }
    request = context->save_request;
    context->save_request = false;
    return request;
}
