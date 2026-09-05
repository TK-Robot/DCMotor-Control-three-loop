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

static Dynamixel2Context *Dynamixel2_ReplyContext;
extern uint32_t SystemCoreClock;

static void Dynamixel2_TrySendStream(Dynamixel2Context *context);

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
    uint16_t protection = context->param->ProtectionFlags;

    if (context->param->PwmInputValid) status |= DXL2_STATUS_PWM_INPUT_VALID;
    if (context->param->OutputEnabled) status |= DXL2_STATUS_OUTPUT_ENABLED;
    if (context->param->FaultCode != DXL2_FAULT_NONE) status |= DXL2_STATUS_FAULT_PRESENT;
    if (protection != PROTECTION_NONE)
        status |= DXL2_STATUS_PROTECTION_INHIBIT;
    status |= (uint16_t)((protection & (PROTECTION_UNDERVOLTAGE
                                       | PROTECTION_OVERTEMPERATURE)) << 5U);
    status |= (uint16_t)((protection & PROTECTION_OVERCURRENT) << 4U);
    if (context->param->ControlSource == CONTROL_SOURCE_PWM_INPUT) status |= DXL2_STATUS_PWM_SOURCE;
    if (context->param->ControlSource == CONTROL_SOURCE_SERIAL) status |= DXL2_STATUS_SERIAL_SOURCE;
    if (context->param->ControlSource == CONTROL_SOURCE_CRSF) status |= DXL2_STATUS_CRSF_SOURCE;
    if (context->param->FaultCode == DXL2_FAULT_NONE) status |= DXL2_STATUS_FAULT_FREE;
    status |= DXL2_STATUS_PROTOCOL_ACTIVE;
    return status;
}

static uint16_t Dynamixel2_TorqueModelStatus(const Dynamixel2Context *context)
{
    const Param *param = context->param;
    uint16_t status = 0U;

    if (MotorTorqueModel_IsTorqueValid(&param->MotorTorqueParams))
        status |= DXL2_TORQUE_STATUS_TORQUE_MODEL_VALID;
    if (param->MotorTorqueResult.electrical_model_valid)
        status |= DXL2_TORQUE_STATUS_ELECTRICAL_MODEL_VALID;
    if (param->MechanicalResult.valid)
        status |= DXL2_TORQUE_STATUS_MECHANICAL_MODEL_VALID;
    if (param->TorqueCommandVoltageLimited)
        status |= DXL2_TORQUE_STATUS_COMMAND_VOLTAGE_LIMITED;
    if (param->MotorTorqueResult.voltage_limited)
        status |= DXL2_TORQUE_STATUS_OPERATING_VOLTAGE_LIMITED;
    if ((param->ProtectionFlags & PROTECTION_TORQUE_MODEL_INVALID) != 0U)
        status |= DXL2_TORQUE_STATUS_CONFIGURATION_FAULT;
    if (param->CurrentSampleValid)
        status |= DXL2_TORQUE_STATUS_CURRENT_SAMPLE_VALID;
    if (param->CurrentEstimated)
        status |= DXL2_TORQUE_STATUS_CURRENT_ESTIMATED;
    if (!param->CurrentSampleValid &&
        (param->DriveRunMode == 2U || param->DriveRunMode == 3U) &&
        param->DrivePower != 0)
        status |= DXL2_TORQUE_STATUS_CURRENT_SAMPLE_UNQUALIFIED;
    return status;
}

static const uint32_t Dynamixel2_BaudRates[] =
{
    115200UL, 1000000UL, 2000000UL, 230400UL, 420000UL
};

enum
{
    DXL2_BAUD_CODE_FIRST = 2U,
    DXL2_BAUD_CODE_LAST = DXL2_BAUD_CODE_FIRST
                           + sizeof(Dynamixel2_BaudRates)
                                 / sizeof(Dynamixel2_BaudRates[0]) - 1U
};

static uint8_t Dynamixel2_BaudCode(uint32_t baud)
{
    uint8_t index;

    for (index = 0U; index < sizeof(Dynamixel2_BaudRates) / sizeof(Dynamixel2_BaudRates[0]); index++)
    {
        if (Dynamixel2_BaudRates[index] == baud)
            return (uint8_t)(index + DXL2_BAUD_CODE_FIRST);
    }
    return DXL2_BAUD_CODE_FIRST;
}

static bool Dynamixel2_BaudFromCode(uint8_t code, uint32_t *baud)
{
    if (code < DXL2_BAUD_CODE_FIRST || code > DXL2_BAUD_CODE_LAST) return false;
    *baud = Dynamixel2_BaudRates[code - DXL2_BAUD_CODE_FIRST];
    return true;
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

typedef struct
{
    uint16_t address;
    uint16_t offset;
} Dynamixel2ReadField;

#define DXL2_PARAM_FIELD(address_, field_) \
    {(address_), (uint16_t)offsetof(Param, field_)}

static const Dynamixel2ReadField Dynamixel2_U8ReadFields[] = {
    DXL2_PARAM_FIELD(DXL2_ADDR_NODE_POSITION, NodePosition),
    DXL2_PARAM_FIELD(DXL2_ADDR_POSITION_DEADBAND_COUNT, PositionDeadbandCounts),
    DXL2_PARAM_FIELD(DXL2_ADDR_TEMPERATURE_C, Temp),
    DXL2_PARAM_FIELD(DXL2_ADDR_TEMPERATURE_LIMIT_C, TempLimit),
    DXL2_PARAM_FIELD(DXL2_ADDR_PWM_MODE, DrivePwmMode),
    DXL2_PARAM_FIELD(DXL2_ADDR_ENCODER_DIRECTION, EncoderVeer),
    DXL2_PARAM_FIELD(DXL2_ADDR_FAIL_SAFE_POLICY, FailSafePolicy),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_POSITION_CHANNEL, CrsfPositionChannel),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_CENTER_CHANNEL, CrsfCenterChannel),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_ENABLE_CHANNEL, CrsfEnableChannel),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_AUTO_ENABLE, CrsfAutoEnable)
};

static const Dynamixel2ReadField Dynamixel2_U16ReadFields[] = {
    DXL2_PARAM_FIELD(DXL2_ADDR_SERIAL_WATCHDOG_MS, SerialWatchdogMs),
    DXL2_PARAM_FIELD(DXL2_ADDR_REPLY_SLOT_US, ReplySlotUs),
    DXL2_PARAM_FIELD(DXL2_ADDR_ACCEL_LIMIT_CPS2, AccelMax),
    DXL2_PARAM_FIELD(DXL2_ADDR_ACTUAL_CURRENT_MA, CurrentLogical_mA),
    DXL2_PARAM_FIELD(DXL2_ADDR_DRIVE_OUTPUT_PERMILLE, DrivePower),
    DXL2_PARAM_FIELD(DXL2_ADDR_SUPPLY_VOLTAGE_MV, VCC_mV),
    DXL2_PARAM_FIELD(DXL2_ADDR_SPEED_LIMIT_CPS, SpeedMax),
    DXL2_PARAM_FIELD(DXL2_ADDR_ENCODER_OFFSET_COUNT, EncoderOffset),
    DXL2_PARAM_FIELD(DXL2_ADDR_PWM_INPUT_PULSE_US, DutyRatio),
    DXL2_PARAM_FIELD(DXL2_ADDR_DECEL_LIMIT_CPS2, DecelMax),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_CHANNEL_MIN, CrsfChannelMin),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_CHANNEL_CENTER, CrsfChannelCenter),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_CHANNEL_MAX, CrsfChannelMax),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_CENTER_TRIGGER, CrsfCenterTrigger),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_ENABLE_THRESHOLD, CrsfEnableThreshold),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_ARM_CURRENT_LIMIT_MA, CrsfArmCurrentLimit_mA),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_ARM_FOLLOW_ERROR, CrsfArmFollowError),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_ARM_TIMEOUT_MS, CrsfArmTimeoutMs),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_WATCHDOG_MS, CrsfWatchdogMs),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_STATUS, CrsfStatus),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_RAW_POSITION, CrsfRawPosition),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_RAW_ENABLE, CrsfRawEnable),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_RAW_CENTER, CrsfRawCenter),
    DXL2_PARAM_FIELD(DXL2_ADDR_MOTOR_WINDING_TEMPERATURE_C, MotorWindingTemperature_C),
    DXL2_PARAM_FIELD(DXL2_ADDR_TORQUE_ENCODER_COUNTS_PER_REV, TorqueEncoderCountsPerRev),
    DXL2_PARAM_FIELD(DXL2_ADDR_TORQUE_CURRENT_LIMIT_MA, TorqueCurrentLimit_mA),
    DXL2_PARAM_FIELD(DXL2_ADDR_MOTOR_REFERENCE_TEMPERATURE_C, MotorTorqueParams.reference_temperature_C),
    DXL2_PARAM_FIELD(DXL2_ADDR_RESISTANCE_TEMP_COEFFICIENT_PPM_PER_C, MotorTorqueParams.resistance_temp_coefficient_ppm_per_C),
    DXL2_PARAM_FIELD(DXL2_ADDR_BRUSH_DROP_MV, MotorTorqueParams.brush_drop_mV),
    DXL2_PARAM_FIELD(DXL2_ADDR_FRICTION_DEADBAND_CPS, MechanicalParams.friction_deadband_cps),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_ADC_RAW, CurrentAdcRaw),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_ADC_OFFSET, CurrentAdcOffset),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_INSTANT_MA, CurrentInstant_mA),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_FILTERED_MA, INA181_mA),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_LOGICAL_MA, CurrentLogical_mA),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_IREF_MA, ExpectMA),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_VFF_PWM, CurrentModelPwm),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_VPI_PWM, CurrentCorrectionPwm),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_LOOP_STATUS, CurrentLoopStatus),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_BRIDGE_STATUS, CurrentBridgeStatus),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_SAMPLE_AGE_MS, CurrentSampleAgeMs),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_WINDOW_VALID, CurrentWindowValid),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_WINDOW_MIN_MA, CurrentWindowMin_mA),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_WINDOW_MAX_MA, CurrentWindowMax_mA),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_WINDOW_AVG_MA, CurrentWindowAvg_mA),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_WINDOW_INVALID, CurrentWindowInvalid),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_HARD_LIMIT_TRIPS, CurrentHardLimitTrips),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_PI_FROZEN, CurrentPiFrozenCount),
    DXL2_PARAM_FIELD(DXL2_ADDR_MOTOR_INDUCTANCE_UH, MotorInductance_uH),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_PEAK_LIMIT_MA, CurrentPeakLimit_mA),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_ABSOLUTE_LIMIT_MA, CurrentAbsoluteLimit_mA),
    DXL2_PARAM_FIELD(DXL2_ADDR_STALL_CURRENT_THRESHOLD_MA, StallCurrentThreshold_mA),
    DXL2_PARAM_FIELD(DXL2_ADDR_STALL_SPEED_THRESHOLD_CPS, StallSpeedThreshold_cps),
    DXL2_PARAM_FIELD(DXL2_ADDR_STALL_CONFIRM_TIME_MS, StallConfirmTimeMs),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_AVERAGE_MA, CurrentAverage_mA),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_PEAK_CHOP_EVENTS, CurrentPeakChopEvents),
    DXL2_PARAM_FIELD(DXL2_ADDR_STALL_ELAPSED_MS, StallElapsedMs),
    DXL2_PARAM_FIELD(DXL2_ADDR_LOW_SPEED_COMP_MAX_SPEED_CPS, LowSpeedCompMaxSpeed_cps)
};

static const Dynamixel2ReadField Dynamixel2_U32ReadFields[] = {
    DXL2_PARAM_FIELD(DXL2_ADDR_ACTUAL_VELOCITY_CPS, EncoderSpeed),
    DXL2_PARAM_FIELD(DXL2_ADDR_MULTI_TURN_POSITION_COUNT, EncoderMultiTurnValue),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_ARM_SPEED_CPS, CrsfArmSpeed_cps),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_NEGATIVE_POSITION_LIMIT, CrsfNegativePositionLimit),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_POSITIVE_POSITION_LIMIT, CrsfPositivePositionLimit),
    DXL2_PARAM_FIELD(DXL2_ADDR_CRSF_CENTER_REFERENCE, CrsfCenterReference),
    DXL2_PARAM_FIELD(DXL2_ADDR_TARGET_ELECTROMAGNETIC_TORQUE_UNM, TargetElectromagneticTorque_uNm),
    DXL2_PARAM_FIELD(DXL2_ADDR_ACTUAL_ELECTROMAGNETIC_TORQUE_UNM, MotorTorqueResult.electromagnetic_torque_uNm),
    DXL2_PARAM_FIELD(DXL2_ADDR_ESTIMATED_SHAFT_LOAD_TORQUE_UNM, MechanicalResult.shaft_load_torque_uNm),
    DXL2_PARAM_FIELD(DXL2_ADDR_INERTIA_TORQUE_UNM, MechanicalResult.inertia_torque_uNm),
    DXL2_PARAM_FIELD(DXL2_ADDR_INTERNAL_LOSS_TORQUE_UNM, MechanicalResult.internal_loss_torque_uNm),
    DXL2_PARAM_FIELD(DXL2_ADDR_REQUIRED_MOTOR_VOLTAGE_MV, MotorTorqueResult.required_voltage_mV),
    DXL2_PARAM_FIELD(DXL2_ADDR_BACK_EMF_MV, MotorTorqueResult.back_emf_mV),
    DXL2_PARAM_FIELD(DXL2_ADDR_AVAILABLE_CURRENT_MA, MotorTorqueResult.available_current_mA),
    DXL2_PARAM_FIELD(DXL2_ADDR_TORQUE_CONSTANT_UNM_PER_A, MotorTorqueParams.torque_constant_uNm_per_A),
    DXL2_PARAM_FIELD(DXL2_ADDR_TORQUE_TEMP_COEFFICIENT_PPM_PER_C, MotorTorqueParams.torque_temp_coefficient_ppm_per_C),
    DXL2_PARAM_FIELD(DXL2_ADDR_BACK_EMF_UV_PER_RPM, MotorTorqueParams.back_emf_uV_per_rpm),
    DXL2_PARAM_FIELD(DXL2_ADDR_TERMINAL_RESISTANCE_MOHM, MotorTorqueParams.terminal_resistance_mOhm),
    DXL2_PARAM_FIELD(DXL2_ADDR_TOTAL_INERTIA_UG_CM2, MechanicalParams.total_inertia_ug_cm2),
    DXL2_PARAM_FIELD(DXL2_ADDR_COULOMB_FRICTION_UNM, MechanicalParams.coulomb_friction_uNm),
    DXL2_PARAM_FIELD(DXL2_ADDR_VISCOUS_FRICTION_NNM_PER_RPM, MechanicalParams.viscous_friction_nNm_per_rpm),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_VALID_TOTAL, CurrentValidTotal),
    DXL2_PARAM_FIELD(DXL2_ADDR_CURRENT_INVALID_TOTAL, CurrentInvalidTotal)
};

static void Dynamixel2_CopyParamFields(uint8_t *table, const Param *param,
                                        const Dynamixel2ReadField *fields,
                                        uint8_t count, uint8_t width)
{
    uint8_t index;
    for (index = 0U; index < count; ++index)
    {
        memcpy(&table[fields[index].address],
               (const uint8_t *)param + fields[index].offset, width);
    }
}

static void Dynamixel2_StoreParamField(Param *param, uint16_t address,
                                        const uint8_t *data, uint8_t width)
{
    const Dynamixel2ReadField *fields;
    uint8_t count;
    uint8_t index;
    if (width == 1U)
    {
        fields = Dynamixel2_U8ReadFields;
        count = (uint8_t)(sizeof(Dynamixel2_U8ReadFields)
                          / sizeof(Dynamixel2_U8ReadFields[0]));
    }
    else if (width == 2U)
    {
        fields = Dynamixel2_U16ReadFields;
        count = (uint8_t)(sizeof(Dynamixel2_U16ReadFields)
                          / sizeof(Dynamixel2_U16ReadFields[0]));
    }
    else
    {
        fields = Dynamixel2_U32ReadFields;
        count = (uint8_t)(sizeof(Dynamixel2_U32ReadFields)
                          / sizeof(Dynamixel2_U32ReadFields[0]));
    }
    for (index = 0U; index < count; ++index)
    {
        if (fields[index].address == address)
        {
            memcpy((uint8_t *)param + fields[index].offset, data, width);
            return;
        }
    }
}

#undef DXL2_PARAM_FIELD

static void Dynamixel2_BuildControlTable(const Dynamixel2Context *context,
                                         uint8_t table[DXL2_CONTROL_TABLE_SIZE])
{
    /* Build one coherent snapshot so a multi-byte Read cannot mix update moments. */
    /* 每次读取先生成一致快照，避免多字节 Read 混入不同更新时刻的数据。 */
    memset(table, 0, DXL2_CONTROL_TABLE_SIZE);
    Dynamixel2_CopyParamFields(table, context->param, Dynamixel2_U8ReadFields,
        (uint8_t)(sizeof(Dynamixel2_U8ReadFields) / sizeof(Dynamixel2_U8ReadFields[0])), 1U);
    Dynamixel2_CopyParamFields(table, context->param, Dynamixel2_U16ReadFields,
        (uint8_t)(sizeof(Dynamixel2_U16ReadFields) / sizeof(Dynamixel2_U16ReadFields[0])), 2U);
    Dynamixel2_CopyParamFields(table, context->param, Dynamixel2_U32ReadFields,
        (uint8_t)(sizeof(Dynamixel2_U32ReadFields) / sizeof(Dynamixel2_U32ReadFields[0])), 4U);
    Dynamixel2_WriteU16(&table[DXL2_ADDR_MODEL_NUMBER], DXL2_MODEL_NUMBER);
    table[DXL2_ADDR_FIRMWARE_VERSION] = DXL2_FIRMWARE_VERSION;
    table[DXL2_ADDR_PROTOCOL_VERSION] = 2U;
    table[DXL2_ADDR_NODE_ID] = context->node_id;
    table[DXL2_ADDR_BAUD_CODE] = Dynamixel2_BaudCode(context->param->BaudRate);

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
    /* EncoderValue is 16-bit internally, while the public position register is I32. */
    Dynamixel2_WriteU32(&table[DXL2_ADDR_ACTUAL_POSITION_COUNT],
                        context->param->EncoderValue);

    Dynamixel2_WritePidSnapshot(table, DXL2_ADDR_CURRENT_PID, &context->param->Pid_PosEle);
    Dynamixel2_WritePidSnapshot(table, DXL2_ADDR_VELOCITY_PID, &context->param->Pid_PosVel);
    Dynamixel2_WritePidSnapshot(table, DXL2_ADDR_POSITION_PID, &context->param->Pid_Pos);
    Dynamixel2_WriteU32(&table[DXL2_ADDR_CURRENT_TICK_MS], context->tick_ms);

    Dynamixel2_WriteU16(&table[128], context->last_diag_error);
    Dynamixel2_WriteU32(&table[130], context->diagnostic_error_count);
    Dynamixel2_WriteU32(&table[134], context->uart_error_count);
    Dynamixel2_WriteU32(&table[138], context->rx_crc_error_count);
    Dynamixel2_WriteU32(&table[142], context->rx_bad_packet_count);
    Dynamixel2_WriteU32(&table[146], context->rx_packet_count);

    Dynamixel2_WriteU32(&table[DXL2_ADDR_TARGET_SHAFT_TORQUE_UNM],
                        (uint32_t)context->pending_command.target_torque_uNm);
    Dynamixel2_WriteU16(&table[DXL2_ADDR_TORQUE_MODEL_STATUS],
                        Dynamixel2_TorqueModelStatus(context));
    memcpy(&table[DXL2_ADDR_LOW_SPEED_COMP_FORWARD_MAP],
           context->param->LowSpeedCompMap_mA,
           sizeof(context->param->LowSpeedCompMap_mA));
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
        uint16_t value = Dynamixel2_ReadU16(data);
        if (base == DXL2_ADDR_CURRENT_PID &&
            ((offset == 0U && value > CURRENT_PID_KP_MAX) ||
             (offset == 2U && value > CURRENT_PID_KI_MAX) ||
             (offset == 4U && value != 0U) ||
             (offset == 10U &&
              (value > CURRENT_PID_VOLTAGE_MAX_MV ||
               value < pid->out_min)) ||
             (offset == 12U && value > pid->out_max)))
        {
            return DXL2_ERROR_DATA_RANGE;
        }
        if (base == DXL2_ADDR_POSITION_PID && offset == 4U
            && value > POSITION_PID_KD_MAX)
        {
            return DXL2_ERROR_DATA_RANGE;
        }
        if (apply)
        {
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
        if (value < 0 ||
            (base == DXL2_ADDR_CURRENT_PID &&
             value > CURRENT_PID_INTEGRAL_MAX))
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
           || address == 22U || address == 26U || address == 30U || address == 34U
           || address == DXL2_ADDR_TARGET_SHAFT_TORQUE_UNM;
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

    if (source > CONTROL_SOURCE_CRSF || mode > SERVO_MODE_PWM_DUTY
        || (control_word & (uint16_t)~(DXL2_CONTROL_ENABLE
                                      | DXL2_CONTROL_USE_EXECUTE_TICK
                                      | DXL2_CONTROL_CLEAR_FAULT
                                      | DXL2_CONTROL_POSITION_MULTI_TURN)) != 0U)
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

    /* Fault clear is a safety command, not an ordinary latest-value update.
     * It must preempt an armed/scheduled pending image and must not be lost
     * when a newly connected host happens to reuse the last 16-bit sequence. */
    if (clear_fault)
    {
        context->pending_valid = false;
        context->pending_sequence_valid = false;
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
        if (!clear_fault && context->applied_sequence_valid
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
        context->param->CrsfManualEnable = false;
    }
    context->param->ControlSource = source;
    context->pending_command.mode = (ServoMode)mode;
    context->pending_command.position_multi_turn =
        (control_word & DXL2_CONTROL_POSITION_MULTI_TURN) != 0U;
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
    context->param->CrsfManualEnable = !clear_fault
                                       && ((control_word & DXL2_CONTROL_ENABLE) != 0U)
                                       && source == CONTROL_SOURCE_CRSF
                                       && context->param->FaultCode == 0U;
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
        if (data[0] > CONTROL_SOURCE_CRSF) return DXL2_ERROR_DATA_RANGE;
        if (apply)
        {
            if (context->param->ControlSource != data[0])
            {
                context->active_command.enable = false;
                context->pending_command.enable = false;
                context->param->CrsfManualEnable = false;
            }
            context->param->ControlSource = data[0];
            Dynamixel2_ResetWatchdog(context);
        }
        return DXL2_ERROR_NONE;
    case 17U:
        if (length != 1U) return DXL2_ERROR_DATA_LENGTH;
        if (data[0] > SERVO_MODE_PWM_DUTY) return DXL2_ERROR_DATA_RANGE;
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
                                       | DXL2_CONTROL_CLEAR_FAULT
                                       | DXL2_CONTROL_POSITION_MULTI_TURN)) != 0U)
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
            context->pending_command.position_multi_turn =
                (control_word & DXL2_CONTROL_POSITION_MULTI_TURN) != 0U;
            context->param->CrsfManualEnable = !clear_fault
                                               && ((control_word & DXL2_CONTROL_ENABLE) != 0U)
                                               && context->param->ControlSource == CONTROL_SOURCE_CRSF
                                               && context->param->FaultCode == 0U;
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
    case DXL2_ADDR_TARGET_SHAFT_TORQUE_UNM:
        if (length != 4U) return DXL2_ERROR_DATA_LENGTH;
        if (apply)
        {
            context->pending_command.target_torque_uNm =
                (int32_t)Dynamixel2_ReadU32(data);
            context->pending_valid = true;
            Dynamixel2_ResetWatchdog(context);
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
                   && sizeof(((Param *)0)->SpeedMax) == 2U
                   && sizeof(((Param *)0)->AccelMax) == 2U
                   && sizeof(((Param *)0)->DecelMax) == 2U,
               "16-bit write metadata field width mismatch");
_Static_assert(sizeof(((Param *)0)->NodePosition) == 1U
                   && sizeof(((Param *)0)->TempLimit) == 1U
                   && sizeof(((Param *)0)->DrivePwmMode) == 1U
                   && sizeof(((Param *)0)->FailSafePolicy) == 1U
                   && sizeof(((Param *)0)->PositionDeadbandCounts) == 1U,
               "8-bit write metadata field width mismatch");

/*
 * One compact rule owns address, width, access and range validation. Only
 * entries carrying an action flag need behavior beyond a direct Param write.
 * 每条紧凑规则统一负责地址、宽度、权限与范围校验；只有带动作标志的条目
 * 才需要执行普通 Param 字段写入之外的副作用。
 */
static const Dynamixel2WriteRule Dynamixel2_WriteRules[] = {
    {DXL2_ADDR_NODE_ID, 1U, DXL2_WRITE_ACTION(DXL2_WRITE_NODE_ID), 1U, 0xFCU},
    {DXL2_ADDR_BAUD_CODE, 1U, DXL2_WRITE_ACTION(DXL2_WRITE_BAUD_CODE),
     DXL2_BAUD_CODE_FIRST, DXL2_BAUD_CODE_LAST},
    {DXL2_ADDR_SERIAL_WATCHDOG_MS, 2U, DXL2_WRITE_FIELD(SerialWatchdogMs), 0U, UINT16_MAX},
    {DXL2_ADDR_NODE_POSITION, 1U, DXL2_WRITE_FIELD(NodePosition), 1U, UINT8_MAX},
    {DXL2_ADDR_REPLY_SLOT_US, 2U, DXL2_WRITE_FIELD(ReplySlotUs),
     DXL2_REPLY_SLOT_MIN_US, DXL2_REPLY_SLOT_MAX_US},
    {DXL2_ADDR_ACCEL_LIMIT_CPS2, 2U, DXL2_WRITE_FIELD(AccelMax), 1U, UINT16_MAX},
    {DXL2_ADDR_POSITION_DEADBAND_COUNT, 1U,
     DXL2_WRITE_FIELD(PositionDeadbandCounts), 1U,
     PID_POSITION_DEADBAND_MAX_COUNTS},
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
    {DXL2_ADDR_SAVE_NVM, 1U, DXL2_WRITE_ACTION(DXL2_WRITE_SAVE_NVM), 1U, 1U},
    {DXL2_ADDR_DECEL_LIMIT_CPS2, 2U, DXL2_WRITE_FIELD(DecelMax), 1U, UINT16_MAX},
    {DXL2_ADDR_CRSF_POSITION_CHANNEL, 1U, DXL2_WRITE_FIELD(CrsfPositionChannel), 0U, 16U},
    {DXL2_ADDR_CRSF_CENTER_CHANNEL, 1U, DXL2_WRITE_FIELD(CrsfCenterChannel), 0U, 16U},
    {DXL2_ADDR_CRSF_ENABLE_CHANNEL, 1U, DXL2_WRITE_FIELD(CrsfEnableChannel), 0U, 16U},
    {DXL2_ADDR_CRSF_AUTO_ENABLE, 1U, DXL2_WRITE_FIELD(CrsfAutoEnable), 0U, 1U},
    {DXL2_ADDR_CRSF_CHANNEL_MIN, 2U, DXL2_WRITE_FIELD(CrsfChannelMin), 0U, 2047U},
    {DXL2_ADDR_CRSF_CHANNEL_CENTER, 2U, DXL2_WRITE_FIELD(CrsfChannelCenter), 0U, 2047U},
    {DXL2_ADDR_CRSF_CHANNEL_MAX, 2U, DXL2_WRITE_FIELD(CrsfChannelMax), 0U, 2047U},
    {DXL2_ADDR_CRSF_CENTER_TRIGGER, 2U, DXL2_WRITE_FIELD(CrsfCenterTrigger), 0U, 2047U},
    {DXL2_ADDR_CRSF_ENABLE_THRESHOLD, 2U, DXL2_WRITE_FIELD(CrsfEnableThreshold), 0U, 2047U},
    {DXL2_ADDR_CRSF_ARM_CURRENT_LIMIT_MA, 2U, DXL2_WRITE_FIELD(CrsfArmCurrentLimit_mA), 1U, 30000U},
    {DXL2_ADDR_CRSF_ARM_FOLLOW_ERROR, 2U, DXL2_WRITE_FIELD(CrsfArmFollowError), 1U, UINT16_MAX},
    {DXL2_ADDR_CRSF_ARM_TIMEOUT_MS, 2U, DXL2_WRITE_FIELD(CrsfArmTimeoutMs), 100U, 10000U},
    {DXL2_ADDR_CRSF_WATCHDOG_MS, 2U, DXL2_WRITE_FIELD(CrsfWatchdogMs), 20U, 2000U}
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

static uint8_t Dynamixel2_WriteCrsfConfig(Dynamixel2Context *context,
                                           uint16_t address,
                                           const uint8_t *data,
                                           uint16_t length, bool apply)
{
    Param *param = context->param;
    int32_t signed_value;

    switch (address)
    {
    case DXL2_ADDR_CRSF_ARM_SPEED_CPS:
        if (length != 4U) return DXL2_ERROR_DATA_LENGTH;
        if (Dynamixel2_ReadU32(data) == 0U
            || Dynamixel2_ReadU32(data) > 1000000UL)
            return DXL2_ERROR_DATA_RANGE;
        if (apply) Dynamixel2_StoreParamField(param, address, data, 4U);
        break;
    case DXL2_ADDR_CRSF_NEGATIVE_POSITION_LIMIT:
    case DXL2_ADDR_CRSF_POSITIVE_POSITION_LIMIT:
        if (length != 4U) return DXL2_ERROR_DATA_LENGTH;
        signed_value = (int32_t)Dynamixel2_ReadU32(data);
        if (signed_value < -1000000000L || signed_value > 1000000000L
            || (address == DXL2_ADDR_CRSF_NEGATIVE_POSITION_LIMIT
                && signed_value >= param->CrsfPositivePositionLimit)
            || (address == DXL2_ADDR_CRSF_POSITIVE_POSITION_LIMIT
                && signed_value <= param->CrsfNegativePositionLimit))
        {
            return DXL2_ERROR_DATA_RANGE;
        }
        if (apply)
        {
            Dynamixel2_StoreParamField(param, address, data, 4U);
            param->CrsfCenterReference = (param->CrsfNegativePositionLimit
                                          + param->CrsfPositivePositionLimit) >> 1;
        }
        break;
    default:
        return DXL2_ERROR_ACCESS;
    }
    return DXL2_ERROR_NONE;
}

static uint8_t Dynamixel2_WriteTorqueConfig(Dynamixel2Context *context,
                                            uint16_t address,
                                            const uint8_t *data,
                                            uint16_t length, bool apply)
{
    Param *param = context->param;
    uint32_t value_u32;
    int32_t value_i32;
    int16_t value_i16;

    /* Live winding temperature may be refreshed while armed; model constants may not. */
    /* 运行期可刷新绕组温度；其余模型常数仅允许停机修改。 */
    if (address != DXL2_ADDR_MOTOR_WINDING_TEMPERATURE_C
        && (param->OutputEnabled || context->active_command.enable
            || context->pending_command.enable))
    {
        return DXL2_ERROR_ACCESS;
    }

    if (address >= DXL2_ADDR_LOW_SPEED_COMP_FORWARD_MAP
        && address < DXL2_CONTROL_TABLE_SIZE)
    {
        uint16_t offset = address - DXL2_ADDR_LOW_SPEED_COMP_FORWARD_MAP;
        uint8_t *destination = (uint8_t *)param->LowSpeedCompMap_mA;

        if (length > (uint16_t)(sizeof(param->LowSpeedCompMap_mA) - offset))
            return DXL2_ERROR_DATA_LENGTH;
        for (uint16_t i = 0U; i < length; ++i)
        {
            int8_t correction = (int8_t)data[i];
            if (correction < -LOW_SPEED_COMP_MAX_CORRECTION_MA
                || correction > LOW_SPEED_COMP_MAX_CORRECTION_MA)
                return DXL2_ERROR_DATA_RANGE;
        }
        if (apply)
        {
            memcpy(&destination[offset], data, length);
        }
        return DXL2_ERROR_NONE;
    }

    switch (address)
    {
    case DXL2_ADDR_LOW_SPEED_COMP_MAX_SPEED_CPS:
        if (length != 2U) return DXL2_ERROR_DATA_LENGTH;
        value_u32 = Dynamixel2_ReadU16(data);
        if (value_u32 != 0U && (value_u32 < 500U || value_u32 > 5000U))
            return DXL2_ERROR_DATA_RANGE;
        if (apply) Dynamixel2_StoreParamField(param, address, data, 2U);
        break;
    case DXL2_ADDR_MOTOR_WINDING_TEMPERATURE_C:
    case DXL2_ADDR_MOTOR_REFERENCE_TEMPERATURE_C:
        if (length != 2U) return DXL2_ERROR_DATA_LENGTH;
        value_i16 = (int16_t)Dynamixel2_ReadU16(data);
        if (value_i16 < -40 || value_i16 > 200) return DXL2_ERROR_DATA_RANGE;
        if (apply) Dynamixel2_StoreParamField(param, address, data, 2U);
        break;
    case DXL2_ADDR_TORQUE_ENCODER_COUNTS_PER_REV:
    case DXL2_ADDR_TORQUE_CURRENT_LIMIT_MA:
    case DXL2_ADDR_RESISTANCE_TEMP_COEFFICIENT_PPM_PER_C:
    case DXL2_ADDR_BRUSH_DROP_MV:
    case DXL2_ADDR_FRICTION_DEADBAND_CPS:
    case DXL2_ADDR_MOTOR_INDUCTANCE_UH:
    case DXL2_ADDR_CURRENT_PEAK_LIMIT_MA:
    case DXL2_ADDR_CURRENT_ABSOLUTE_LIMIT_MA:
    case DXL2_ADDR_STALL_CURRENT_THRESHOLD_MA:
    case DXL2_ADDR_STALL_SPEED_THRESHOLD_CPS:
    case DXL2_ADDR_STALL_CONFIRM_TIME_MS:
        if (length != 2U) return DXL2_ERROR_DATA_LENGTH;
        value_u32 = Dynamixel2_ReadU16(data);
        if ((address == DXL2_ADDR_TORQUE_ENCODER_COUNTS_PER_REV && value_u32 == 0U)
            || (address == DXL2_ADDR_TORQUE_CURRENT_LIMIT_MA
                && (value_u32 == 0U || value_u32 > 30000U))
            || (address == DXL2_ADDR_RESISTANCE_TEMP_COEFFICIENT_PPM_PER_C
                && value_u32 > 10000U)
            || (address == DXL2_ADDR_BRUSH_DROP_MV && value_u32 > 5000U)
            || (address == DXL2_ADDR_MOTOR_INDUCTANCE_UH
                && (value_u32 == 0U || value_u32 > 10000U))
            || (address == DXL2_ADDR_CURRENT_PEAK_LIMIT_MA
                && (value_u32 < 100U
                    || value_u32 >= param->CurrentAbsoluteLimit_mA))
            || (address == DXL2_ADDR_CURRENT_ABSOLUTE_LIMIT_MA
                && (value_u32 <= param->CurrentPeakLimit_mA
                    || value_u32 > 1830U))
            || (address == DXL2_ADDR_STALL_CURRENT_THRESHOLD_MA
                && (value_u32 < 50U || value_u32 > 1500U))
            || (address == DXL2_ADDR_STALL_SPEED_THRESHOLD_CPS
                && value_u32 < 10U)
            || (address == DXL2_ADDR_STALL_CONFIRM_TIME_MS
                && (value_u32 < 500U || value_u32 > 10000U)))
        {
            return DXL2_ERROR_DATA_RANGE;
        }
        if (apply) Dynamixel2_StoreParamField(param, address, data, 2U);
        break;
    case DXL2_ADDR_TORQUE_TEMP_COEFFICIENT_PPM_PER_C:
        if (length != 4U) return DXL2_ERROR_DATA_LENGTH;
        value_i32 = (int32_t)Dynamixel2_ReadU32(data);
        if (value_i32 < -10000L || value_i32 > 10000L)
            return DXL2_ERROR_DATA_RANGE;
        if (apply) Dynamixel2_StoreParamField(param, address, data, 4U);
        break;
    case DXL2_ADDR_TORQUE_CONSTANT_UNM_PER_A:
    case DXL2_ADDR_BACK_EMF_UV_PER_RPM:
    case DXL2_ADDR_TERMINAL_RESISTANCE_MOHM:
    case DXL2_ADDR_TOTAL_INERTIA_UG_CM2:
    case DXL2_ADDR_COULOMB_FRICTION_UNM:
    case DXL2_ADDR_VISCOUS_FRICTION_NNM_PER_RPM:
        if (length != 4U) return DXL2_ERROR_DATA_LENGTH;
        value_u32 = Dynamixel2_ReadU32(data);
        if ((address == DXL2_ADDR_TORQUE_CONSTANT_UNM_PER_A
             && (value_u32 == 0U || value_u32 > 1000000UL))
            || (address == DXL2_ADDR_BACK_EMF_UV_PER_RPM
                && (value_u32 == 0U || value_u32 > 10000000UL))
            || (address == DXL2_ADDR_TERMINAL_RESISTANCE_MOHM
                && (value_u32 == 0U || value_u32 > 10000000UL))
            || (address == DXL2_ADDR_TOTAL_INERTIA_UG_CM2
                && value_u32 > 100000000UL)
            || ((address == DXL2_ADDR_COULOMB_FRICTION_UNM
                  || address == DXL2_ADDR_VISCOUS_FRICTION_NNM_PER_RPM)
                 && value_u32 > 10000000UL))
        {
            return DXL2_ERROR_DATA_RANGE;
        }
        if (apply) Dynamixel2_StoreParamField(param, address, data, 4U);
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
    if (address >= DXL2_ADDR_CRSF_POSITION_CHANNEL
        && address < DXL2_ADDR_CRSF_STATUS)
    {
        uint16_t crsf_value;
        if (context->param->OutputEnabled || context->param->CrsfManualEnable
            || (context->param->CrsfStatus
                & (CRSF_STATUS_ARM_TRACKING | CRSF_STATUS_ACTIVE)) != 0U)
        {
            return DXL2_ERROR_ACCESS;
        }
        if (address == DXL2_ADDR_CRSF_ARM_SPEED_CPS
            || address == DXL2_ADDR_CRSF_NEGATIVE_POSITION_LIMIT
            || address == DXL2_ADDR_CRSF_POSITIVE_POSITION_LIMIT)
        {
            return Dynamixel2_WriteCrsfConfig(context, address, data, length, apply);
        }
        if (length == 2U)
        {
            crsf_value = Dynamixel2_ReadU16(data);
            if ((address == DXL2_ADDR_CRSF_CHANNEL_MIN
                 && crsf_value >= context->param->CrsfChannelCenter)
                || (address == DXL2_ADDR_CRSF_CHANNEL_CENTER
                    && (crsf_value <= context->param->CrsfChannelMin
                        || crsf_value >= context->param->CrsfChannelMax))
                || (address == DXL2_ADDR_CRSF_CHANNEL_MAX
                    && crsf_value <= context->param->CrsfChannelCenter))
            {
                return DXL2_ERROR_DATA_RANGE;
            }
        }
    }
    if (address >= DXL2_ADDR_MOTOR_WINDING_TEMPERATURE_C
        && address < DXL2_CONTROL_TABLE_SIZE)
    {
        return Dynamixel2_WriteTorqueConfig(context, address, data, length, apply);
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
    if (context == NULL || context->uart != USART2 || context->param == NULL)
    {
        return false;
    }
    /* Param.RxBuf belongs to DMA until an idle/error callback snapshots it. */
    /* Param.RxBuf 在空闲或错误回调取得快照前由 DMA 独占。 */
    context->rx_active = false;
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
    LL_DMA_ClearFlag_GI4(DMA1);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4,
                            LL_USART_DMA_GetRegAddr(context->uart,
                                                    LL_USART_DMA_REG_DATA_RECEIVE));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4,
                            (uint32_t)context->param->RxBuf);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4,
                         sizeof(context->param->RxBuf));
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4);
    LL_USART_ClearFlag_IDLE(context->uart);
    LL_USART_ClearFlag_ORE(context->uart);
    LL_USART_EnableIT_IDLE(context->uart);
    LL_USART_EnableIT_ERROR(context->uart);
    LL_USART_EnableDMAReq_RX(context->uart);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
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
    if (context == NULL || data == NULL || length == 0U || context->uart != USART2)
    {
        return false;
    }
    if (context->tx_busy)
    {
        return false;
    }
    /* DMA borrows the buffer until Dynamixel2_TxCpltCallback releases tx_busy. */
    /* DMA 借用该缓冲区，直到 Dynamixel2_TxCpltCallback 释放 tx_busy。 */
    context->tx_busy = true;
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
    LL_DMA_ClearFlag_GI5(DMA1);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5,
                            LL_USART_DMA_GetRegAddr(context->uart,
                                                    LL_USART_DMA_REG_DATA_TRANSMIT));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)data);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, length);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);
    LL_USART_DisableIT_TC(context->uart);
    LL_USART_ClearFlag_TC(context->uart);
    LL_USART_EnableDMAReq_TX(context->uart);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
    return true;
}

static bool Dynamixel2_StartPacket(Dynamixel2Context *context,
                                   const uint8_t *packet, uint16_t length)
{
    uint32_t primask;
    bool started = false;
    if (context == NULL || context->param == NULL || packet == NULL || length == 0U
        || length > sizeof(context->param->TxBuf))
    {
        return false;
    }

    /* The 1 ms task and USART IRQ are both TX producers. Claim the shared DMA
     * buffer and start DMA in one critical section, otherwise an IRQ can start
     * an ACK between memcpy and tx_busy=true and the resumed task corrupts it. */
    primask = __get_PRIMASK();
    __disable_irq();
    if (!context->tx_busy)
    {
        memcpy(context->param->TxBuf, packet, length);
        started = Dynamixel2_StartTx(context, context->param->TxBuf, length);
    }
    __set_PRIMASK(primask);
    return started;
}

static bool Dynamixel2_QueuePacket(Dynamixel2Context *context,
                                   const uint8_t *packet, uint16_t length)
{
    uint32_t primask;
    bool queued = false;
    if (context == NULL || packet == NULL || length == 0U
        || length > sizeof(context->pending_tx))
    {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (!context->pending_tx_valid)
    {
        memcpy(context->pending_tx, packet, length);
        context->pending_tx_length = length;
        context->pending_tx_delayed = false;
        context->pending_tx_valid = true;
        queued = true;
    }
    __set_PRIMASK(primask);
    return queued;
}

static void Dynamixel2_TrySendPending(Dynamixel2Context *context)
{
    uint32_t primask;
    if (context == NULL)
    {
        return;
    }

    /* Transfer queue ownership to DMA atomically. Clearing the queue after
     * re-enabling IRQs would let an IRQ observe a stale occupied slot. */
    primask = __get_PRIMASK();
    __disable_irq();
    if (!context->tx_busy && !context->baud_change_ready
        && !context->baud_change_in_progress && context->pending_tx_valid
        && !context->pending_tx_delayed && context->pending_tx_length != 0U)
    {
        memcpy(context->param->TxBuf, context->pending_tx,
               context->pending_tx_length);
        if (Dynamixel2_StartTx(context, context->param->TxBuf,
                               context->pending_tx_length))
        {
            context->pending_tx_valid = false;
            context->pending_tx_length = 0U;
        }
    }
    __set_PRIMASK(primask);
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
            Dynamixel2_TrySendStream(Dynamixel2_ReplyContext);
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
        Dynamixel2_TrySendStream(context);
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
        if (!Dynamixel2_QueuePacket(context, packet, packet_length))
        {
            if (context->tx_drop_count < UINT32_MAX) ++context->tx_drop_count;
            Dynamixel2_RecordDiagnostic(context, DXL2_DIAG_TX_DROP);
        }
        return;
    }
    /* One bounded pending slot prevents callbacks from blocking on an active DMA TX. */
    /* 单级有界待发槽避免回调在 DMA 发送期间阻塞。 */
    if (context->tx_busy)
    {
        if (!Dynamixel2_QueuePacket(context, packet, packet_length))
        {
            if (context->tx_drop_count < UINT32_MAX) ++context->tx_drop_count;
            Dynamixel2_RecordDiagnostic(context, DXL2_DIAG_TX_DROP);
        }
        return;
    }
    if (!Dynamixel2_StartPacket(context, packet, packet_length))
    {
        if (!Dynamixel2_QueuePacket(context, packet, packet_length))
        {
            if (context->tx_drop_count < UINT32_MAX) ++context->tx_drop_count;
            Dynamixel2_RecordDiagnostic(context, DXL2_DIAG_TX_DROP);
        }
    }
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

static void Dynamixel2_StopStream(Dynamixel2Context *context)
{
    context->stream_active = false;
    context->stream_frame_ready = false;
    context->stream_node_count = 0U;
}

static void Dynamixel2_HandleStreamSync(Dynamixel2Context *context,
                                         const Dxl2Packet *packet)
{
    uint8_t count;
    uint8_t index;
    uint16_t period;
    uint16_t slot;
    uint16_t session;
    uint32_t master;

    if (packet->id != DXL2_BROADCAST_ID
        || packet->parameter_length < DXL2_TK_STREAM_SYNC_HEADER_SIZE
        || packet->parameters[0] != DXL2_TK_STREAM_VERSION)
    {
        return;
    }
    if ((packet->parameters[1] & DXL2_TK_STREAM_SYNC_ENABLE) == 0U)
    {
        Dynamixel2_StopStream(context);
        return;
    }
    count = packet->parameters[12];
    if (count == 0U || count > DXL2_TK_STREAM_MAX_NODES
        || packet->parameter_length
               != (uint16_t)(DXL2_TK_STREAM_SYNC_HEADER_SIZE + count))
    {
        return;
    }
    for (index = 0U; index < count; ++index)
    {
        if (packet->parameters[DXL2_TK_STREAM_SYNC_HEADER_SIZE + index]
            == context->node_id)
        {
            break;
        }
    }
    if (index == count)
    {
        Dynamixel2_StopStream(context);
        return;
    }

    session = Dynamixel2_ReadU16(&packet->parameters[2]);
    master = Dynamixel2_ReadU32(&packet->parameters[4]);
    period = Dynamixel2_ReadU16(&packet->parameters[8]);
    slot = Dynamixel2_ReadU16(&packet->parameters[10]);
    if (period == 0U || slot < 100U
        || period < (uint16_t)((uint16_t)count
                    * (uint16_t)(((uint32_t)slot + 999UL) / 1000UL)))
    {
        Dynamixel2_StopStream(context);
        return;
    }

    /* A periodic A1 with the same session is a lease renewal, not a new
     * clock measurement. The master's timestamp is captured before queued
     * USB/UART transmission; applying it again would turn variable transport
     * latency into a visible clock jump and cancel an otherwise valid frame. */
    if (context->stream_active && session == context->stream_session
        && (packet->parameters[1] & DXL2_TK_STREAM_SYNC_CLEAR) == 0U
        && period == context->stream_period_ms
        && slot == context->stream_slot_us
        && index == context->stream_reply_index
        && count == context->stream_node_count)
    {
        context->stream_lease_deadline = context->tick_ms + 3000U;
        return;
    }
    if (session != context->stream_session
        || (packet->parameters[1] & DXL2_TK_STREAM_SYNC_CLEAR) != 0U)
    {
        context->stream_range_count = 0U;
        context->stream_sequence = 0U;
    }
    context->stream_session = session;
    context->stream_period_ms = period;
    context->stream_slot_us = slot;
    context->stream_reply_index = index;
    context->stream_node_count = count;
    context->stream_clock_offset = master - context->tick_ms;
    context->stream_lease_deadline = context->tick_ms + 3000U;
    context->stream_next_sample_tick = context->tick_ms
        + period - (uint16_t)(master % period);
    context->stream_frame_ready = false;
    context->stream_active = true;
}

static void Dynamixel2_HandleStreamRead(Dynamixel2Context *context,
                                         const Dxl2Packet *packet)
{
    uint16_t session;
    uint16_t address;
    uint16_t frame_length = DXL2_TK_STREAM_FRAME_HEADER_SIZE;
    uint8_t length;
    uint8_t flags;
    uint8_t index;

    if (packet->id == DXL2_BROADCAST_ID || packet->parameter_length != 7U
        || packet->parameters[0] != DXL2_TK_STREAM_VERSION
        || !context->stream_active)
    {
        return;
    }
    session = Dynamixel2_ReadU16(&packet->parameters[1]);
    flags = packet->parameters[3];
    address = Dynamixel2_ReadU16(&packet->parameters[4]);
    length = packet->parameters[6];
    if (session != context->stream_session || length == 0U
        || (uint32_t)address + length > DXL2_CONTROL_TABLE_SIZE)
    {
        return;
    }
    if ((flags & DXL2_TK_STREAM_READ_REPLACE) != 0U)
    {
        context->stream_range_count = 0U;
    }
    for (index = 0U; index < context->stream_range_count; ++index)
    {
        if (context->stream_ranges[index].address == address
            && context->stream_ranges[index].length == length)
        {
            return;
        }
    }
    if (context->stream_range_count >= DXL2_TK_STREAM_MAX_RANGES)
    {
        return;
    }
    for (index = 0U; index < context->stream_range_count; ++index)
    {
        frame_length = (uint16_t)(frame_length + 3U
                       + context->stream_ranges[index].length);
    }
    if ((uint16_t)(frame_length + 3U + length) > DXL2_MAX_PARAMETERS)
    {
        return;
    }
    context->stream_ranges[context->stream_range_count].address = address;
    context->stream_ranges[context->stream_range_count].length = length;
    ++context->stream_range_count;
}

static void Dynamixel2_CaptureStreamFrame(Dynamixel2Context *context)
{
    uint8_t table[DXL2_CONTROL_TABLE_SIZE];
    uint16_t offset = 0U;
    uint8_t index;
    uint8_t flags = 0U;
    uint32_t slot_ms = ((uint32_t)context->stream_slot_us + 999UL) / 1000UL;
    uint32_t primask;

    /* Make the shared snapshot unavailable before rewriting it. An IRQ may
     * otherwise encode a half-old, half-new A3 payload. */
    primask = __get_PRIMASK();
    __disable_irq();
    if (context->stream_frame_ready)
    {
        context->stream_frame_overwritten = true;
    }
    context->stream_frame_ready = false;
    __set_PRIMASK(primask);
    if (context->stream_frame_overwritten)
    {
        flags |= DXL2_TK_STREAM_FLAG_OVERWRITE;
        context->stream_frame_overwritten = false;
    }
    Dynamixel2_BuildControlTable(context, table);
    context->stream_frame[offset++] = DXL2_TK_STREAM_STATUS_MARKER;
    context->stream_frame[offset++] = DXL2_TK_STREAM_VERSION;
    Dynamixel2_WriteU16(&context->stream_frame[offset], context->stream_session);
    offset += 2U;
    ++context->stream_sequence;
    Dynamixel2_WriteU16(&context->stream_frame[offset], context->stream_sequence);
    offset += 2U;
    Dynamixel2_WriteU32(
        &context->stream_frame[offset], context->tick_ms
                                        + context->stream_clock_offset);
    offset += 4U;
    context->stream_frame[offset++] = flags;
    context->stream_frame[offset++] = context->stream_range_count;
    for (index = 0U; index < context->stream_range_count; ++index)
    {
        const Dynamixel2StreamRange *range = &context->stream_ranges[index];
        Dynamixel2_WriteU16(&context->stream_frame[offset], range->address);
        offset += 2U;
        context->stream_frame[offset++] = range->length;
        memcpy(&context->stream_frame[offset], &table[range->address], range->length);
        offset = (uint16_t)(offset + range->length);
    }
    primask = __get_PRIMASK();
    __disable_irq();
    context->stream_frame_length = offset;
    context->stream_tx_due_tick = context->tick_ms
                                  + (uint32_t)context->stream_reply_index * slot_ms;
    context->stream_tx_expire_tick = context->tick_ms
                                     + (uint32_t)(context->stream_reply_index + 1U)
                                           * slot_ms - 1UL;
    context->stream_frame_ready = true;
    __set_PRIMASK(primask);
}

static void Dynamixel2_TrySendStream(Dynamixel2Context *context)
{
    uint8_t packet[DXL2_MAX_PACKET_SIZE];
    uint16_t packet_length;

    if (context == NULL || !context->stream_active
        || !context->stream_frame_ready)
    {
        return;
    }
    if (context->stream_node_count > 1U
        && Dynamixel2_IsExecuteTickDue(context->tick_ms,
                                       context->stream_tx_expire_tick + 1UL))
    {
        context->stream_frame_ready = false;
        context->stream_frame_overwritten = true;
        return;
    }
    if (!Dynamixel2_IsExecuteTickDue(context->tick_ms,
                                     context->stream_tx_due_tick)
        || context->tx_busy
        || context->baud_change_ready || context->baud_change_in_progress)
    {
        return;
    }
    packet_length = Dxl2_EncodeStatus(
        packet, sizeof(packet), context->node_id,
        context->param->FaultCode == DXL2_FAULT_NONE
            ? DXL2_ERROR_NONE : DXL2_STATUS_ALERT_MASK,
        context->stream_frame, context->stream_frame_length);
    if (packet_length == 0U)
    {
        context->stream_frame_ready = false;
        return;
    }
    if (Dynamixel2_StartPacket(context, packet, packet_length))
    {
        context->stream_frame_ready = false;
    }
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

static void Dynamixel2_HandleTimedRead(Dynamixel2Context *context,
                                       const Dxl2Packet *packet)
{
    union
    {
        uint32_t tick;
        uint8_t bytes[DXL2_MAX_PARAMETERS];
    } response;
    uint16_t address;
    uint8_t length;
    uint8_t error;

    if (packet->parameter_length != DXL2_TK_TIMED_READ_REQUEST_SIZE)
    {
        Dynamixel2_SendStatus(context, DXL2_ERROR_DATA_LENGTH, NULL, 0U, 0U);
        return;
    }
    address = Dynamixel2_ReadU16(&packet->parameters[0]);
    length = packet->parameters[2] & 0x7FU;
    response.tick = context->tick_ms;
    error = Dynamixel2_ReadTable(context, address, length,
                                 &response.bytes[DXL2_TK_TIMED_READ_TICK_SIZE]);
    Dynamixel2_SendStatus(
        context, error, error == 0U ? response.bytes : NULL,
        error == 0U ? (uint16_t)(length + DXL2_TK_TIMED_READ_TICK_SIZE) : 0U, 0U);
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
    DXL2_ACK_U16(DXL2_ACK_STATUS_WORD, Dynamixel2_StatusWord(context));
    DXL2_ACK_U16(DXL2_ACK_FAULT_CODE, context->param->FaultCode);
    DXL2_ACK_U16(DXL2_ACK_ACTUAL_CURRENT, context->param->CurrentLogical_mA);
    DXL2_ACK_U32(DXL2_ACK_ACTUAL_VELOCITY, context->param->EncoderSpeed);
    DXL2_ACK_U32(DXL2_ACK_ACTUAL_POSITION, context->param->EncoderValue);
    DXL2_ACK_U32(DXL2_ACK_MULTI_TURN_POSITION, context->param->EncoderMultiTurnValue);
    DXL2_ACK_U16(DXL2_ACK_DRIVE_OUTPUT, context->param->DrivePower);
    DXL2_ACK_U16(DXL2_ACK_SUPPLY_VOLTAGE, context->param->VCC_mV);
    if ((mask & DXL2_ACK_TEMPERATURE) != 0U)
        output[offset++] = (uint8_t)context->param->Temp;
    DXL2_ACK_U32(DXL2_ACK_CURRENT_TICK, context->tick_ms);
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
        if (packet->id != DXL2_BROADCAST_ID)
            Dynamixel2_HandleRead(context, packet);
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
            Dynamixel2_HandleTimedRead(context, packet);
        break;
    case DXL2_INST_TK_STREAM_SYNC:
        Dynamixel2_HandleStreamSync(context, packet);
        break;
    case DXL2_INST_TK_STREAM_READ:
        Dynamixel2_HandleStreamRead(context, packet);
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
    context->rx_stream_length = (uint16_t)(context->rx_stream_length - count);
    for (uint16_t index = 0U; index < context->rx_stream_length; ++index)
        context->rx_stream[index] = context->rx_stream[count + index];
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
        else if (result.status != DXL2_DECODE_NO_HEADER)
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

void Dynamixel2_Init(Dynamixel2Context *context, USART_TypeDef *uart, Param *param)
{
    if (context == NULL || param == NULL)
    {
        return;
    }
    memset(context, 0, sizeof(*context));
    context->uart = uart;
    context->param = param;
    Dynamixel2_ReplyContext = context;
    context->node_id = (param->NodeId >= 1U && param->NodeId <= 0xFCU)
                           ? param->NodeId : 1U;
    context->active_command.mode = SERVO_MODE_CURRENT;
    context->active_command.enable = false;
    context->pending_command = context->active_command;
    /* NVM may restore the selected source, but no live command or enable state. */
    /* NVM 可恢复控制源选择，但绝不恢复活动命令或使能状态。 */
    if (param->ControlSource > CONTROL_SOURCE_CRSF)
        param->ControlSource = CONTROL_SOURCE_PWM_INPUT;
    param->CrsfManualEnable = false;
    param->NodeId = context->node_id;
    if (param->NodePosition == 0U) param->NodePosition = 1U;
    if (param->ReplySlotUs < DXL2_REPLY_SLOT_MIN_US
        || param->ReplySlotUs > DXL2_REPLY_SLOT_MAX_US)
        param->ReplySlotUs = 120U;
    Dynamixel2_InitReplyTimer();
    (void)Dynamixel2_StartRx(context);
}

void Dynamixel2_SetRxObserver(Dynamixel2Context *context,
                              Dynamixel2RxObserver observer, void *user)
{
    if (context != NULL)
    {
        context->rx_observer = observer;
        context->rx_observer_user = user;
    }
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

void Dynamixel2_RxEventCallback(Dynamixel2Context *context, uint16_t size)
{
    uint8_t received[sizeof(((Param *)0)->RxBuf)];

    if (context == NULL || context->uart != USART2 || context->param == NULL)
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
    if (size <= sizeof(received) && context->rx_observer != NULL)
    {
        context->rx_observer(context->rx_observer_user, received, size);
    }
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

void Dynamixel2_TxCpltCallback(Dynamixel2Context *context)
{
    if (context != NULL && context->uart == USART2)
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
        Dynamixel2_TrySendStream(context);
        Dynamixel2_TrySendPending(context);
    }
}

void Dynamixel2_UartIrqHandler(Dynamixel2Context *context)
{
    uint32_t errors;

    if (context == NULL || context->uart != USART2)
    {
        return;
    }

    errors = context->uart->ISR & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE);
    if (errors != 0U)
    {
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
        LL_USART_DisableDMAReq_RX(context->uart);
        context->uart->ICR = USART_ICR_PECF | USART_ICR_FECF
                           | USART_ICR_NECF | USART_ICR_ORECF;
        context->rx_active = false;
        Dynamixel2_RecordUartError(context, errors);
        (void)Dynamixel2_StartRx(context);
    }

    if (LL_USART_IsEnabledIT_IDLE(context->uart)
        && LL_USART_IsActiveFlag_IDLE(context->uart))
    {
        uint16_t size;
        LL_USART_ClearFlag_IDLE(context->uart);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
        LL_USART_DisableDMAReq_RX(context->uart);
        size = (uint16_t)(sizeof(context->param->RxBuf)
               - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_4));
        context->rx_active = false;
        if (size != 0U)
        {
            Dynamixel2_RxEventCallback(context, size);
        }
        else
        {
            (void)Dynamixel2_StartRx(context);
        }
    }

    if (LL_USART_IsEnabledIT_TC(context->uart)
        && LL_USART_IsActiveFlag_TC(context->uart))
    {
        LL_USART_DisableIT_TC(context->uart);
        LL_USART_ClearFlag_TC(context->uart);
        Dynamixel2_TxCpltCallback(context);
    }
}

void Dynamixel2_DmaIrqHandler(Dynamixel2Context *context)
{
    if (context == NULL || context->uart != USART2)
    {
        return;
    }

    if (LL_DMA_IsActiveFlag_TE4(DMA1))
    {
        LL_DMA_ClearFlag_GI4(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
        LL_USART_DisableDMAReq_RX(context->uart);
        context->rx_active = false;
        Dynamixel2_RecordUartError(context, DMA_ISR_TEIF4);
        (void)Dynamixel2_StartRx(context);
    }
    else if (LL_DMA_IsActiveFlag_TC4(DMA1))
    {
        LL_DMA_ClearFlag_GI4(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
        LL_USART_DisableDMAReq_RX(context->uart);
        context->rx_active = false;
        Dynamixel2_RxEventCallback(context,
                                   (uint16_t)sizeof(context->param->RxBuf));
    }

    if (LL_DMA_IsActiveFlag_TE5(DMA1))
    {
        LL_DMA_ClearFlag_GI5(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
        LL_USART_DisableDMAReq_TX(context->uart);
        context->tx_busy = false;
        context->node_id_change_after_tx = false;
        context->baud_change_after_tx = false;
        Dynamixel2_RecordUartError(context, DMA_ISR_TEIF5);
        Dynamixel2_TrySendStream(context);
        Dynamixel2_TrySendPending(context);
    }
    else if (LL_DMA_IsActiveFlag_TC5(DMA1))
    {
        /* DMA completion only means TDR is fed; commit changes at UART TC. */
        /* DMA 完成仅表示 TDR 已填充；节点配置必须等 UART TC 后提交。 */
        LL_DMA_ClearFlag_GI5(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
        LL_USART_DisableDMAReq_TX(context->uart);
        LL_USART_EnableIT_TC(context->uart);
        if (LL_USART_IsActiveFlag_TC(context->uart))
        {
            LL_USART_DisableIT_TC(context->uart);
            LL_USART_ClearFlag_TC(context->uart);
            Dynamixel2_TxCpltCallback(context);
        }
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
    if (context->stream_active)
    {
        if (Dynamixel2_IsExecuteTickDue(context->tick_ms,
                                        context->stream_lease_deadline))
        {
            Dynamixel2_StopStream(context);
        }
        else if (Dynamixel2_IsExecuteTickDue(context->tick_ms,
                                             context->stream_next_sample_tick))
        {
            Dynamixel2_CaptureStreamFrame(context);
            context->stream_next_sample_tick += context->stream_period_ms;
            if (Dynamixel2_IsExecuteTickDue(context->tick_ms,
                                            context->stream_next_sample_tick))
            {
                context->stream_next_sample_tick = context->tick_ms
                                                   + context->stream_period_ms;
            }
        }
        Dynamixel2_TrySendStream(context);
    }
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
}

const ServoCommand *Dynamixel2_GetActiveCommand(const Dynamixel2Context *context)
{
    static const ServoCommand disabled = {0};
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
