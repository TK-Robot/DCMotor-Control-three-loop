/**
 * @file Dynamixel2.c
 * @brief TK Servo DYNAMIXEL Protocol 2.0 slave implementation.
 * @brief TK Servo DYNAMIXEL Protocol 2.0 从站实现。
 */

#include "Dynamixel2.h"
#include "PID.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

extern DMA_HandleTypeDef hdma_usart2_rx;

static Dynamixel2Context *Dynamixel2_ReplyContext;
extern uint32_t SystemCoreClock;

enum
{
    DXL2_ERROR_NONE = DXL2_STATUS_ERROR_NONE,
    DXL2_ERROR_RESULT_FAIL = DXL2_STATUS_ERROR_RESULT_FAIL,
    DXL2_ERROR_INSTRUCTION = DXL2_STATUS_ERROR_INSTRUCTION,
    DXL2_ERROR_DATA_RANGE = DXL2_STATUS_ERROR_DATA_RANGE,
    DXL2_ERROR_DATA_LENGTH = DXL2_STATUS_ERROR_DATA_LENGTH,
    DXL2_ERROR_ACCESS = DXL2_STATUS_ERROR_ACCESS
};

/*
 * Control Table layout (little-endian multi-byte values):
 * 控制表布局（多字节值均为小端序）：
 *   0..9    identity and persistent bus configuration / 身份与持久化总线配置
 *   16..39  runtime command and acknowledgement image / 运行命令与确认映像
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
     * Bits: 0 ready, 1 PWM input valid, 2 enabled, 3 fault, 8 PWM source, 9 serial source,
     *       11 fault-free, 12 communication active.
     * 位定义：0 就绪，1 PWM 输入有效，2 已使能，3 故障，8 PWM 源，9 串口源，
     *         11 无故障，12 通信活动。
     */
    uint16_t status = DXL2_STATUS_READY;

    if (context->param->PwmInputValid) status |= DXL2_STATUS_PWM_INPUT_VALID;
    if (context->param->OutputEnabled) status |= DXL2_STATUS_OUTPUT_ENABLED;
    if (context->param->FaultCode != DXL2_FAULT_NONE) status |= DXL2_STATUS_FAULT_PRESENT;
    if (context->param->ProtectionFlags != PROTECTION_NONE)
        status |= DXL2_STATUS_PROTECTION_INHIBIT;
    if ((context->param->ProtectionFlags & PROTECTION_UNDERVOLTAGE) != 0U)
        status |= DXL2_STATUS_UNDERVOLTAGE;
    if ((context->param->ProtectionFlags & PROTECTION_OVERTEMPERATURE) != 0U)
        status |= DXL2_STATUS_OVERTEMPERATURE;
    if (context->param->ControlSource == CONTROL_SOURCE_PWM_INPUT) status |= DXL2_STATUS_PWM_SOURCE;
    if (context->param->ControlSource == CONTROL_SOURCE_SERIAL) status |= DXL2_STATUS_SERIAL_SOURCE;
    if (context->param->FaultCode == DXL2_FAULT_NONE) status |= DXL2_STATUS_FAULT_FREE;
    status |= DXL2_STATUS_PROTOCOL_ACTIVE;
    return status;
}

static void Dynamixel2_PublishStatusSnapshot(Dynamixel2Context *context)
{
    uint8_t next = (uint8_t)(context->status_snapshot_index ^ 1U);
    Dynamixel2StatusSnapshot *snapshot = &context->status_snapshot[next];
    snapshot->actual_velocity = context->param->EncoderSpeed;
    snapshot->actual_position = context->param->EncoderValue;
    snapshot->multi_turn_position = context->param->EncoderMultiTurnValue;
    snapshot->current_tick = context->tick_ms;
    snapshot->status_word = Dynamixel2_StatusWord(context);
    snapshot->fault_code = context->param->FaultCode;
    snapshot->actual_current = context->param->CurrentLogical_mA;
    snapshot->drive_output = context->param->DrivePower;
    snapshot->supply_voltage = context->param->VCC_mV;
    snapshot->temperature = context->param->Temp;
    __DMB();
    context->status_snapshot_index = next;
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
    Dynamixel2_WriteU16(&table[DXL2_ADDR_MODEL_NUMBER], DXL2_MODEL_NUMBER);
    table[DXL2_ADDR_FIRMWARE_VERSION] = DXL2_FIRMWARE_VERSION;
    table[DXL2_ADDR_PROTOCOL_VERSION] = 2U;
    table[DXL2_ADDR_NODE_ID] = context->node_id;
    table[DXL2_ADDR_BAUD_CODE] = Dynamixel2_BaudCode(context->param->BaudRate);
    Dynamixel2_WriteU16(&table[DXL2_ADDR_SERIAL_WATCHDOG_MS], context->param->SerialWatchdogMs);
    table[DXL2_ADDR_NODE_POSITION] = context->param->NodePosition;
    Dynamixel2_WriteU16(&table[DXL2_ADDR_REPLY_SLOT_US], context->param->ReplySlotUs);

    table[16] = context->param->ControlSource;
    table[17] = (uint8_t)context->pending_command.mode;
    Dynamixel2_WriteU16(&table[18], context->pending_command.enable ? 1U : 0U);
    Dynamixel2_WriteU16(&table[20], (uint16_t)context->pending_command.target_current_mA);
    Dynamixel2_WriteU32(&table[22], (uint32_t)context->pending_command.target_speed);
    Dynamixel2_WriteU32(&table[26], (uint32_t)context->pending_command.target_position);
    Dynamixel2_WriteU32(&table[30], context->execute_tick);
    Dynamixel2_WriteU16(&table[DXL2_ADDR_COMMAND_SEQUENCE], context->accepted_sequence);
    Dynamixel2_WriteU16(&table[DXL2_ADDR_APPLIED_SEQUENCE], context->applied_sequence);
    table[DXL2_ADDR_LAST_COMMAND_RESULT] = context->last_command_result;

    Dynamixel2_WriteU16(&table[DXL2_ADDR_STATUS_WORD], Dynamixel2_StatusWord(context));
    Dynamixel2_WriteU16(&table[DXL2_ADDR_FAULT_CODE], context->param->FaultCode);
    Dynamixel2_WriteU16(&table[DXL2_ADDR_ACTUAL_CURRENT_MA], (uint16_t)context->param->CurrentLogical_mA);
    Dynamixel2_WriteU32(&table[DXL2_ADDR_ACTUAL_VELOCITY_CPS], (uint32_t)context->param->EncoderSpeed);
    Dynamixel2_WriteU32(&table[DXL2_ADDR_ACTUAL_POSITION_COUNT], (uint32_t)context->param->EncoderValue);
    Dynamixel2_WriteU32(&table[DXL2_ADDR_MULTI_TURN_POSITION_COUNT], (uint32_t)context->param->EncoderMultiTurnValue);
    Dynamixel2_WriteU16(&table[DXL2_ADDR_DRIVE_OUTPUT_PERMILLE], (uint16_t)context->param->DrivePower);
    Dynamixel2_WriteU16(&table[DXL2_ADDR_SUPPLY_VOLTAGE_MV], context->param->VCC_mV);
    table[DXL2_ADDR_TEMPERATURE_C] = (uint8_t)context->param->Temp;

    Dynamixel2_WritePidSnapshot(table, DXL2_ADDR_CURRENT_PID, &context->param->Pid_PosEle);
    Dynamixel2_WritePidSnapshot(table, DXL2_ADDR_VELOCITY_PID, &context->param->Pid_PosVel);
    Dynamixel2_WritePidSnapshot(table, DXL2_ADDR_POSITION_PID, &context->param->Pid_Pos);
    table[DXL2_ADDR_TEMPERATURE_LIMIT_C] = (uint8_t)context->param->TempLimit;
    Dynamixel2_WriteU16(&table[DXL2_ADDR_SPEED_LIMIT_CPS], context->param->SpeedMax);
    table[DXL2_ADDR_PWM_MODE] = context->param->DrivePwmMode;
    table[DXL2_ADDR_ENCODER_DIRECTION] = context->param->EncoderVeer ? 1U : 0U;
    Dynamixel2_WriteU16(&table[DXL2_ADDR_ENCODER_OFFSET_COUNT], context->param->EncoderOffset);
    table[DXL2_ADDR_FAIL_SAFE_POLICY] = context->param->FailSafePolicy;
    Dynamixel2_WriteU32(&table[DXL2_ADDR_CURRENT_TICK_MS], context->tick_ms);
    Dynamixel2_WriteU16(&table[DXL2_ADDR_PWM_INPUT_LOW_US], context->param->DutyRatio);

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
           || address == 22U || address == 26U || address == 30U || address == 34U;
}

static uint8_t Dynamixel2_WriteCommandBlock(Dynamixel2Context *context,
                                            const uint8_t *data, uint16_t length,
                                            bool apply)
{
    uint8_t source = data[0];
    uint8_t mode = data[1];
    uint16_t control_word = Dynamixel2_ReadU16(&data[2]);
    bool clear_fault = (control_word & DXL2_CONTROL_CLEAR_FAULT) != 0U;
    bool scheduled = (control_word & DXL2_CONTROL_USE_EXECUTE_TICK) != 0U;
    uint16_t sequence = length == DXL2_FULL_COMMAND_IMAGE_SIZE
                            ? Dynamixel2_ReadU16(&data[18]) : 0U;

    if (source > CONTROL_SOURCE_PWM_INPUT || mode > SERVO_MODE_POSITION
        || (control_word & (uint16_t)~(DXL2_CONTROL_ENABLE
                                      | DXL2_CONTROL_USE_EXECUTE_TICK
                                      | DXL2_CONTROL_CLEAR_FAULT)) != 0U)
    {
        return DXL2_ERROR_DATA_RANGE;
    }
    if (scheduled && length != DXL2_FULL_COMMAND_IMAGE_SIZE)
    {
        return DXL2_ERROR_DATA_LENGTH;
    }
    if (!apply)
    {
        return DXL2_ERROR_NONE;
    }

    if (length == DXL2_FULL_COMMAND_IMAGE_SIZE)
    {
        if (context->pending_valid)
        {
            if (context->pending_sequence_valid
                && sequence == context->pending_sequence)
            {
                /* Exact sequence retransmission is idempotent and is not a watchdog refresh. */
                /* 相同序号重传按幂等处理，并且不刷新看门狗。 */
                return DXL2_ERROR_NONE;
            }
            return DXL2_ERROR_RESULT_FAIL;
        }
        if (context->applied_sequence_valid
            && sequence == context->applied_sequence)
        {
            return DXL2_ERROR_NONE;
        }
    }

    /* Length 14 is the legacy immediate image; length 20 also carries tick and sequence. */
    /* 长度 14 是兼容的立即命令映像；长度 20 还原子携带时刻和序号。 */
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
    if (length == DXL2_FULL_COMMAND_IMAGE_SIZE)
    {
        context->execute_tick = Dynamixel2_ReadU32(&data[14]);
        context->execute_scheduled = scheduled;
        context->pending_sequence = sequence;
        context->pending_sequence_valid = true;
        context->accepted_sequence = sequence;
        context->accepted_sequence_valid = true;
    }
    else
    {
        context->execute_scheduled = false;
        context->pending_sequence_valid = false;
    }
    if (clear_fault)
    {
        context->param->FaultCode = 0U;
    }
    /* Fault clear is always disarmed; enabling requires a later explicit write. */
    /* 清故障写入始终保持未使能；使能必须由后续独立写入显式触发。 */
    context->pending_command.enable = !clear_fault
                                      && ((control_word & DXL2_CONTROL_ENABLE) != 0U)
                                      && source == CONTROL_SOURCE_SERIAL
                                      && context->param->FaultCode == 0U;
    context->pending_valid = true;
    Dynamixel2_ResetWatchdog(context);
    return DXL2_ERROR_NONE;
}

static uint8_t Dynamixel2_WriteCommand(Dynamixel2Context *context, uint16_t address,
                                       const uint8_t *data, uint16_t length, bool apply)
{
    if (address == DXL2_ADDR_CONTROL_SOURCE
        && (length == DXL2_LEGACY_COMMAND_IMAGE_SIZE
            || length == DXL2_FULL_COMMAND_IMAGE_SIZE))
    {
        return Dynamixel2_WriteCommandBlock(context, data, length, apply);
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
        if ((control_word & (uint16_t)~(DXL2_CONTROL_ENABLE
                                       | DXL2_CONTROL_USE_EXECUTE_TICK
                                       | DXL2_CONTROL_CLEAR_FAULT)) != 0U)
            return DXL2_ERROR_DATA_RANGE;
        clear_fault = (control_word & DXL2_CONTROL_CLEAR_FAULT) != 0U;
        if (apply)
        {
            if (clear_fault)
            {
                context->param->FaultCode = 0U;
            }
            context->execute_scheduled = (control_word
                                          & DXL2_CONTROL_USE_EXECUTE_TICK) != 0U;
            context->pending_command.enable = !clear_fault
                                              && ((control_word & DXL2_CONTROL_ENABLE) != 0U)
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
    case 34U:
        if (length != 2U) return DXL2_ERROR_DATA_LENGTH;
        if (apply)
        {
            context->pending_sequence = Dynamixel2_ReadU16(data);
            context->pending_sequence_valid = true;
        }
        return DXL2_ERROR_NONE;
    default:
        return DXL2_ERROR_ACCESS;
    }
}

typedef enum
{
    DXL2_WRITE_NODE_ID = 1U,
    DXL2_WRITE_BAUD_CODE,
    DXL2_WRITE_ENCODER_DIRECTION,
    DXL2_WRITE_ENCODER_OFFSET,
    DXL2_WRITE_CLEAR_DIAGNOSTICS,
    DXL2_WRITE_SAVE_NVM
} Dynamixel2WriteAction;

typedef struct
{
    uint8_t address;
    uint8_t length;
    uint16_t target;
    uint16_t minimum;
    uint16_t maximum;
} Dynamixel2WriteRule;

#define DXL2_WRITE_ACTION_FLAG 0x8000U
#define DXL2_WRITE_FIELD(field_) ((uint16_t)offsetof(Param, field_))
#define DXL2_WRITE_ACTION(action_) \
    ((uint16_t)(DXL2_WRITE_ACTION_FLAG | (uint16_t)(action_)))

/* Metadata offsets must never overlap the encoded action flag. / 元数据偏移不得与动作标志重叠。 */
_Static_assert(sizeof(Param) < DXL2_WRITE_ACTION_FLAG,
               "Param offsets exceed compact write metadata");
_Static_assert(sizeof(((Param *)0)->SerialWatchdogMs) == 2U
                   && sizeof(((Param *)0)->ReplySlotUs) == 2U
                   && sizeof(((Param *)0)->SpeedMax) == 2U,
               "16-bit write metadata field width mismatch");
_Static_assert(sizeof(((Param *)0)->NodePosition) == 1U
                   && sizeof(((Param *)0)->TempLimit) == 1U
                   && sizeof(((Param *)0)->DrivePwmMode) == 1U
                   && sizeof(((Param *)0)->FailSafePolicy) == 1U,
               "8-bit write metadata field width mismatch");

/*
 * One compact rule owns address, width, access and range validation. Only
 * entries carrying an action flag need behavior beyond a direct Param write.
 * 每条紧凑规则统一负责地址、宽度、权限与范围校验；只有带动作标志的条目
 * 才需要执行普通 Param 字段写入之外的副作用。
 */
static const Dynamixel2WriteRule Dynamixel2_WriteRules[] = {
    {DXL2_ADDR_NODE_ID, 1U, DXL2_WRITE_ACTION(DXL2_WRITE_NODE_ID), 1U, 0xFCU},
    {DXL2_ADDR_BAUD_CODE, 1U, DXL2_WRITE_ACTION(DXL2_WRITE_BAUD_CODE), 2U, 4U},
    {DXL2_ADDR_SERIAL_WATCHDOG_MS, 2U, DXL2_WRITE_FIELD(SerialWatchdogMs), 0U, UINT16_MAX},
    {DXL2_ADDR_NODE_POSITION, 1U, DXL2_WRITE_FIELD(NodePosition), 1U, UINT8_MAX},
    {DXL2_ADDR_REPLY_SLOT_US, 2U, DXL2_WRITE_FIELD(ReplySlotUs),
     DXL2_REPLY_SLOT_MIN_US, DXL2_REPLY_SLOT_MAX_US},
    {DXL2_ADDR_TEMPERATURE_LIMIT_C, 1U, DXL2_WRITE_FIELD(TempLimit), 20U, 85U},
    {DXL2_ADDR_SPEED_LIMIT_CPS, 2U, DXL2_WRITE_FIELD(SpeedMax), 1000U, UINT16_MAX},
    {DXL2_ADDR_PWM_MODE, 1U, DXL2_WRITE_FIELD(DrivePwmMode), 2U, 4U},
    {DXL2_ADDR_ENCODER_DIRECTION, 1U,
     DXL2_WRITE_ACTION(DXL2_WRITE_ENCODER_DIRECTION), 0U, 1U},
    {DXL2_ADDR_ENCODER_OFFSET_COUNT, 2U,
     DXL2_WRITE_ACTION(DXL2_WRITE_ENCODER_OFFSET), 0U, 16383U},
    {DXL2_ADDR_FAIL_SAFE_POLICY, 1U, DXL2_WRITE_FIELD(FailSafePolicy),
     0U, FAILSAFE_FALLBACK_PWM},
    {DXL2_ADDR_CLEAR_DIAGNOSTICS, 1U,
     DXL2_WRITE_ACTION(DXL2_WRITE_CLEAR_DIAGNOSTICS), 1U, 1U},
    {DXL2_ADDR_SAVE_NVM, 1U, DXL2_WRITE_ACTION(DXL2_WRITE_SAVE_NVM), 1U, 1U}
};

static const Dynamixel2WriteRule *Dynamixel2_FindWriteRule(uint16_t address)
{
    uint8_t index;
    for (index = 0U;
         index < (uint8_t)(sizeof(Dynamixel2_WriteRules)
                           / sizeof(Dynamixel2_WriteRules[0]));
         ++index)
    {
        if (Dynamixel2_WriteRules[index].address == address)
        {
            return &Dynamixel2_WriteRules[index];
        }
    }
    return NULL;
}

static uint8_t Dynamixel2_ApplyWriteAction(Dynamixel2Context *context,
                                            Dynamixel2WriteAction action,
                                            const uint8_t *data, bool apply)
{
    uint32_t baud;

    switch (action)
    {
    case DXL2_WRITE_NODE_ID:
        if (apply)
        {
            context->node_id = data[0];
            context->param->NodeId = data[0];
        }
        break;
    case DXL2_WRITE_BAUD_CODE:
        if (apply)
        {
            if (context->baud_change_after_tx || context->baud_change_ready
                || context->baud_change_in_progress)
            {
                return DXL2_ERROR_RESULT_FAIL;
            }
            (void)Dynamixel2_BaudFromCode(data[0], &baud);
            context->pending_baud_rate = baud;
            context->baud_change_ready = true;
        }
        break;
    case DXL2_WRITE_ENCODER_DIRECTION:
        if (context->active_command.enable || context->pending_command.enable)
        {
            return DXL2_ERROR_ACCESS;
        }
        if (apply)
        {
            context->param->EncoderVeer = data[0] != 0U;
            context->param->EncoderRebaseline = true;
        }
        break;
    case DXL2_WRITE_ENCODER_OFFSET:
        if (apply)
        {
            context->param->EncoderOffset = Dynamixel2_ReadU16(data);
            context->param->EncoderRebaseline = true;
        }
        break;
    case DXL2_WRITE_CLEAR_DIAGNOSTICS:
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
        break;
    case DXL2_WRITE_SAVE_NVM:
        if (context->param->OutputEnabled || context->active_command.enable
            || context->pending_command.enable)
        {
            return DXL2_ERROR_ACCESS;
        }
        /* Flash work remains deferred to main context. / Flash 操作仍延迟到主循环上下文。 */
        if (apply) context->save_request = true;
        break;
    default:
        return DXL2_ERROR_ACCESS;
    }
    return DXL2_ERROR_NONE;
}

static uint8_t Dynamixel2_WriteTable(Dynamixel2Context *context, uint16_t address,
                                     const uint8_t *data, uint16_t length, bool apply)
{
    const Dynamixel2WriteRule *rule;
    uint16_t value;

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

    rule = Dynamixel2_FindWriteRule(address);
    if (rule == NULL)
    {
        return DXL2_ERROR_ACCESS;
    }
    if (length != rule->length)
    {
        return DXL2_ERROR_DATA_LENGTH;
    }
    value = length == 1U ? data[0] : Dynamixel2_ReadU16(data);
    if (value < rule->minimum || value > rule->maximum)
    {
        return DXL2_ERROR_DATA_RANGE;
    }
    if ((rule->target & DXL2_WRITE_ACTION_FLAG) != 0U)
    {
        return Dynamixel2_ApplyWriteAction(
            context, (Dynamixel2WriteAction)(rule->target & ~DXL2_WRITE_ACTION_FLAG),
            data, apply);
    }
    if (apply)
    {
        /* STM32 and the public table are both little-endian. / STM32 与公开控制表均为小端序。 */
        memcpy((uint8_t *)context->param + rule->target, data, length);
    }
    return DXL2_ERROR_NONE;
}

#undef DXL2_WRITE_ACTION
#undef DXL2_WRITE_FIELD
#undef DXL2_WRITE_ACTION_FLAG

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
    NVIC_EnableIRQ(TIM1_CC_IRQn);
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
    if (context == NULL || context->tx_busy || context->baud_change_ready
        || context->baud_change_in_progress || !context->pending_tx_valid
        || context->pending_tx_delayed
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

void TIM1_CC_IRQHandler(void)
{
    if ((TIM1->SR & TIM_SR_CC1IF) != 0U && (TIM1->DIER & TIM_DIER_CC1IE) != 0U)
    {
        TIM1->DIER &= ~TIM_DIER_CC1IE;
        TIM1->SR &= ~TIM_SR_CC1IF;
        if (Dynamixel2_ReplyContext != NULL)
        {
            Dynamixel2_ReplyContext->pending_tx_delayed = false;
            Dynamixel2_TrySendPending(Dynamixel2_ReplyContext);
        }
    }
}

static bool Dynamixel2_ArmDelayedStatus(Dynamixel2Context *context,
                                         uint8_t error,
                                         const uint8_t *parameters,
                                         uint16_t length,
                                         uint32_t delay_us)
{
    uint16_t now;
    uint16_t elapsed;
    if (context->tx_busy || context->pending_tx_valid || delay_us > 60000UL)
    {
        if (context->tx_drop_count < UINT32_MAX) ++context->tx_drop_count;
        Dynamixel2_RecordDiagnostic(context, DXL2_DIAG_TX_DROP);
        return false;
    }
    context->pending_tx_length = Dxl2_EncodeStatus(
        context->pending_tx, sizeof(context->pending_tx), context->node_id,
        error, parameters, length);
    if (context->pending_tx_length == 0U)
    {
        return false;
    }
    context->pending_tx_valid = true;
    context->pending_tx_delayed = true;
    now = (uint16_t)TIM1->CNT;
    elapsed = (uint16_t)(now - context->rx_packet_end_us);
    if (elapsed >= (uint16_t)delay_us)
    {
        context->pending_tx_delayed = false;
        Dynamixel2_TrySendPending(context);
        return true;
    }
    TIM1->CCR1 = (uint16_t)(context->rx_packet_end_us + (uint16_t)delay_us);
    TIM1->SR &= ~TIM_SR_CC1IF;
    TIM1->DIER |= TIM_DIER_CC1IE;
    return true;
}

static void Dynamixel2_SendStatus(Dynamixel2Context *context, uint8_t error,
                                  const uint8_t *parameters, uint16_t length,
                                  uint32_t delay_us)
{
    uint8_t packet[DXL2_MAX_PACKET_SIZE];
    uint8_t status_error = error;
    if (context->param->FaultCode != DXL2_FAULT_NONE)
    {
        status_error |= DXL2_STATUS_ALERT_MASK;
    }
    if (delay_us != 0UL)
    {
        (void)Dynamixel2_ArmDelayedStatus(context, status_error, parameters,
                                          length, delay_us);
        return;
    }
    uint16_t packet_length = Dxl2_EncodeStatus(packet, sizeof(packet), context->node_id,
                                               status_error, parameters, length);
    if (packet_length == 0U)
    {
        return;
    }
    Dynamixel2_TrySendPending(context);
    if (context->baud_change_ready || context->baud_change_in_progress)
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
    uint32_t wire_bytes = DXL2_STATUS_PACKET_OVERHEAD_BYTES + data_length
                          + ((uint32_t)data_length + 2UL) / 3UL;
    uint32_t packet_time_us = ((wire_bytes
                                * DXL2_UART_BITS_PER_BYTE * 1000000UL)
                               + baud - 1UL) / baud + DXL2_REPLY_GUARD_US;
    uint32_t slot_us = context->param->ReplySlotUs;
    if (slot_us < packet_time_us) slot_us = packet_time_us;
    return DXL2_REPLY_GUARD_US + (uint32_t)order * slot_us;
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
    bool unicast = Dxl2_ShouldReturnStatus(packet->id, packet->instruction);

    if (packet->parameter_length < 3U)
    {
        if (unicast)
        {
            Dynamixel2_SendStatus(context, DXL2_ERROR_DATA_LENGTH, NULL, 0U, 0U);
        }
        return;
    }
    address = Dynamixel2_ReadU16(packet->parameters);
    data_length = (uint16_t)(packet->parameter_length - 2U);
    if (unicast && !registered
        && (address == DXL2_ADDR_NODE_ID || address == DXL2_ADDR_BAUD_CODE))
    {
        error = Dynamixel2_WriteTable(context, address, &packet->parameters[2],
                                      data_length, false);
        if (error == DXL2_ERROR_NONE
            && (context->tx_busy || context->pending_tx_valid
                || context->node_id_change_after_tx
                || context->baud_change_after_tx
                || context->baud_change_ready
                || context->baud_change_in_progress))
        {
            error = DXL2_ERROR_RESULT_FAIL;
        }
        Dynamixel2_SendStatus(context, error, NULL, 0U, 0U);
        if (error == DXL2_ERROR_NONE && context->tx_busy)
        {
            if (address == DXL2_ADDR_NODE_ID)
            {
                context->pending_node_id = packet->parameters[2];
                context->node_id_change_after_tx = true;
            }
            else
            {
                (void)Dynamixel2_BaudFromCode(packet->parameters[2],
                                               &context->pending_baud_rate);
                context->baud_change_after_tx = true;
            }
        }
        return;
    }
    if (unicast && !registered && address == DXL2_ADDR_SAVE_NVM
        && (context->save_request || context->save_status_pending))
    {
        error = DXL2_ERROR_RESULT_FAIL;
    }
    else
    {
        error = Dynamixel2_WriteTable(context, address, &packet->parameters[2],
                                      data_length, !registered);
    }
    if (error == 0U && registered)
    {
        context->registered_address = address;
        context->registered_length = data_length;
        memcpy(context->registered_data, &packet->parameters[2], data_length);
        context->registered_write_valid = true;
    }
    if (unicast && !registered && address == DXL2_ADDR_SAVE_NVM
        && error == DXL2_ERROR_NONE)
    {
        /* The Status Packet is sent only after main confirms durable Flash completion. */
        /* 只有主循环确认 Flash 持久化完成后才发送状态包。 */
        context->save_status_pending = true;
        return;
    }
    if (unicast)
    {
        Dynamixel2_SendStatus(context, error, NULL, 0U, 0U);
    }
}

static void Dynamixel2_HandleAction(Dynamixel2Context *context,
                                    const Dxl2Packet *packet)
{
    uint8_t error = DXL2_ERROR_NONE;
    uint16_t address = context->registered_address;
    bool save_action = context->registered_write_valid
                       && address == DXL2_ADDR_SAVE_NVM;
    bool node_id_action = context->registered_write_valid
                          && address == DXL2_ADDR_NODE_ID;
    bool baud_action = context->registered_write_valid
                       && address == DXL2_ADDR_BAUD_CODE;
    if (context->registered_write_valid)
    {
        if ((node_id_action || baud_action)
            && Dxl2_ShouldReturnStatus(packet->id, packet->instruction))
        {
            error = Dynamixel2_WriteTable(context, context->registered_address,
                                          context->registered_data,
                                          context->registered_length, false);
            if (error == DXL2_ERROR_NONE
                && (context->tx_busy || context->pending_tx_valid
                    || context->node_id_change_after_tx
                    || context->baud_change_after_tx
                    || context->baud_change_ready
                    || context->baud_change_in_progress))
            {
                error = DXL2_ERROR_RESULT_FAIL;
            }
        }
        else if (save_action && Dxl2_ShouldReturnStatus(packet->id, packet->instruction)
            && (context->save_request || context->save_status_pending))
        {
            error = DXL2_ERROR_RESULT_FAIL;
        }
        else
        {
            error = Dynamixel2_WriteTable(context, context->registered_address,
                                      context->registered_data,
                                      context->registered_length, true);
        }
        context->registered_write_valid = false;
    }
    if (Dxl2_ShouldReturnStatus(packet->id, packet->instruction))
    {
        if (node_id_action || baud_action)
        {
            Dynamixel2_SendStatus(context, error, NULL, 0U, 0U);
            if (error == DXL2_ERROR_NONE && context->tx_busy)
            {
                if (node_id_action)
                {
                    context->pending_node_id = context->registered_data[0];
                    context->node_id_change_after_tx = true;
                }
                else
                {
                    (void)Dynamixel2_BaudFromCode(context->registered_data[0],
                                                   &context->pending_baud_rate);
                    context->baud_change_after_tx = true;
                }
            }
            return;
        }
        if (save_action && error == DXL2_ERROR_NONE)
        {
            context->save_status_pending = true;
            return;
        }
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

static uint16_t Dynamixel2_BuildTkAck(const Dynamixel2Context *context,
                                      uint16_t mask, bool duplicate,
                                      uint8_t *output)
{
    Dynamixel2StatusSnapshot snapshot =
        context->status_snapshot[context->status_snapshot_index];
    uint16_t offset = 0U;
    uint8_t state = 0U;

    if (context->accepted_sequence_valid) state |= DXL2_ACK_ACCEPTED_VALID;
    if (context->applied_sequence_valid) state |= DXL2_ACK_APPLIED_VALID;
    if (context->pending_valid) state |= DXL2_ACK_PENDING;
    if (duplicate) state |= DXL2_ACK_DUPLICATE;
    Dynamixel2_WriteU16(&output[offset], context->accepted_sequence); offset += 2U;
    Dynamixel2_WriteU16(&output[offset], context->applied_sequence); offset += 2U;
    output[offset++] = context->last_command_result;
    output[offset++] = state;

#define DXL2_ACK_U16(bit_, value_) do { if ((mask & (bit_)) != 0U) { \
    Dynamixel2_WriteU16(&output[offset], (uint16_t)(value_)); offset += 2U; } } while (0)
#define DXL2_ACK_U32(bit_, value_) do { if ((mask & (bit_)) != 0U) { \
    Dynamixel2_WriteU32(&output[offset], (uint32_t)(value_)); offset += 4U; } } while (0)
    DXL2_ACK_U16(DXL2_ACK_STATUS_WORD, snapshot.status_word);
    DXL2_ACK_U16(DXL2_ACK_FAULT_CODE, snapshot.fault_code);
    DXL2_ACK_U16(DXL2_ACK_ACTUAL_CURRENT, snapshot.actual_current);
    DXL2_ACK_U32(DXL2_ACK_ACTUAL_VELOCITY, snapshot.actual_velocity);
    DXL2_ACK_U32(DXL2_ACK_ACTUAL_POSITION, snapshot.actual_position);
    DXL2_ACK_U32(DXL2_ACK_MULTI_TURN_POSITION, snapshot.multi_turn_position);
    DXL2_ACK_U16(DXL2_ACK_DRIVE_OUTPUT, snapshot.drive_output);
    DXL2_ACK_U16(DXL2_ACK_SUPPLY_VOLTAGE, snapshot.supply_voltage);
    if ((mask & DXL2_ACK_TEMPERATURE) != 0U)
        output[offset++] = (uint8_t)snapshot.temperature;
    DXL2_ACK_U32(DXL2_ACK_CURRENT_TICK, snapshot.current_tick);
#undef DXL2_ACK_U16
#undef DXL2_ACK_U32
    return offset;
}

static void Dynamixel2_HandleTkSyncControl(Dynamixel2Context *context,
                                            const Dxl2Packet *packet)
{
    Dxl2TkSyncControlView view;
    Dxl2TkSyncParseStatus parse =
        Dxl2_ParseTkSyncControl(packet, context->node_id, &view);
    uint8_t command[DXL2_FULL_COMMAND_IMAGE_SIZE];
    uint8_t ack[33];
    uint8_t error = DXL2_ERROR_NONE;
    bool duplicate;
    uint16_t ack_length;

    if (parse == DXL2_TK_SYNC_INVALID)
    {
        if (context->rx_bad_packet_count < UINT32_MAX) ++context->rx_bad_packet_count;
        Dynamixel2_RecordDiagnostic(context, DXL2_DIAG_RX_BAD_PACKET);
        return;
    }
    if (parse != DXL2_TK_SYNC_TARGETED)
    {
        return;
    }
    duplicate = (context->pending_valid && context->pending_sequence_valid
                 && view.sequence == context->pending_sequence)
                || (!context->pending_valid && context->applied_sequence_valid
                    && view.sequence == context->applied_sequence);
    if (view.execute_mode != DXL2_TK_EXECUTE_NEXT_UPDATE
        || (Dynamixel2_ReadU16(&view.record[3])
            & DXL2_CONTROL_USE_EXECUTE_TICK) != 0U)
    {
        error = DXL2_ERROR_DATA_RANGE;
    }
    else
    {
        memcpy(command, &view.record[1], DXL2_LEGACY_COMMAND_IMAGE_SIZE);
        Dynamixel2_WriteU32(&command[14], view.execute_value);
        Dynamixel2_WriteU16(&command[18], view.sequence);
        error = Dynamixel2_WriteCommandBlock(context, command,
                                              sizeof(command), true);
    }
    ack_length = Dynamixel2_BuildTkAck(context, view.ack_mask, duplicate, ack);
    Dynamixel2_SendStatus(context, error, ack, ack_length,
                          Dynamixel2_ReplyDelay(context, view.reply_index,
                                                ack_length));
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
    case DXL2_INST_TK_SYNC_CONTROL:
        if (packet->id == DXL2_BROADCAST_ID)
            Dynamixel2_HandleTkSyncControl(context, packet);
        else
            Dynamixel2_SendStatus(context, DXL2_ERROR_INSTRUCTION, NULL, 0U, 0U);
        break;
    default:
        if (Dxl2_ShouldReturnStatus(packet->id, packet->instruction))
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
    Dynamixel2_ReplyContext = context;
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
    if (param->ReplySlotUs < DXL2_REPLY_SLOT_MIN_US
        || param->ReplySlotUs > DXL2_REPLY_SLOT_MAX_US)
        param->ReplySlotUs = 120U;
    Dynamixel2_InitReplyTimer();
    Dynamixel2_PublishStatusSnapshot(context);
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
    context->rx_packet_end_us = (uint16_t)TIM1->CNT;
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
        if (context->node_id_change_after_tx)
        {
            context->node_id = context->pending_node_id;
            context->param->NodeId = context->pending_node_id;
            context->node_id_change_after_tx = false;
        }
        if (context->baud_change_after_tx)
        {
            context->baud_change_after_tx = false;
            context->baud_change_ready = true;
        }
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
            context->param->FaultCode = DXL2_FAULT_SERIAL_WATCHDOG;
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
        && (!context->execute_scheduled
            || Dynamixel2_IsExecuteTickDue(context->tick_ms, context->execute_tick)))
    {
        context->active_command = context->pending_command;
        if (context->pending_sequence_valid)
        {
            context->applied_sequence = context->pending_sequence;
            context->applied_sequence_valid = true;
            context->last_command_result = DXL2_ERROR_NONE;
        }
        context->pending_valid = false;
        context->pending_sequence_valid = false;
        context->execute_scheduled = false;
    }
    Dynamixel2_PublishStatusSnapshot(context);
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

void Dynamixel2_CompleteSaveRequest(Dynamixel2Context *context, bool success)
{
    if (context == NULL || !context->save_status_pending)
    {
        return;
    }
    context->save_status_pending = false;
    Dynamixel2_SendStatus(context,
                          success ? DXL2_ERROR_NONE : DXL2_ERROR_RESULT_FAIL,
                          NULL, 0U, 0U);
}

bool Dynamixel2_ConsumeBaudRateChange(Dynamixel2Context *context,
                                      uint32_t *baud)
{
    if (context == NULL || baud == NULL || !context->baud_change_ready)
    {
        return false;
    }
    *baud = context->pending_baud_rate;
    context->baud_change_ready = false;
    context->baud_change_in_progress = true;
    return true;
}

void Dynamixel2_CompleteBaudRateChange(Dynamixel2Context *context,
                                       bool success)
{
    if (context == NULL || !context->baud_change_in_progress)
    {
        return;
    }
    if (success)
    {
        context->param->BaudRate = context->pending_baud_rate;
    }
    context->baud_change_in_progress = false;
}
