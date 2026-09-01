#include "NvmParam.h"

#include <stddef.h>
#include <string.h>

#include "PID.h"
#include "stm32g0xx.h"

/*
 * Architecture note:
 * One Flash page is reserved as an append-only parameter log. Each save writes
 * one complete record into the next empty slot. The page is erased only when
 * all slots are used, so normal parameter updates do not erase Flash every time.
 *
 * 架构说明：
 * 最后一页 Flash 被保留为追加式参数日志。每次保存只向下一个空槽写入一条完整记录；
 * 只有所有槽位写满后才擦除整页，从而减少 Flash 擦写次数。
 */
#define NVM_PARAM_FLASH_SIZE_BYTES   (32U * 1024U)
#define NVM_PARAM_FLASH_PAGE_BYTES   (2U * 1024U)
#define NVM_PARAM_FLASH_ADDR         (FLASH_BASE + NVM_PARAM_FLASH_SIZE_BYTES - NVM_PARAM_FLASH_PAGE_BYTES)
#define NVM_PARAM_MAGIC              (0x56504B54UL)
#define NVM_PARAM_COMMIT_MAGIC       (0x54494D43UL)
#define NVM_PARAM_LOW_SPEED_MARKER8  (0x4CU)
#define NVM_PARAM_LOW_SPEED_MARKER16 (0x5343U)
#define NVM_PARAM_VERSION            (8U)
#define NVM_PARAM_CRC_INIT           (0xFFFFFFFFUL)
#define NVM_PARAM_CRC_POLY           (0xEDB88320UL)
#define NVM_FLASH_KEY1               (0x45670123UL)
#define NVM_FLASH_KEY2               (0xCDEF89ABUL)
#define NVM_FLASH_ERROR_MASK         (FLASH_SR_OPERR | FLASH_SR_PROGERR | \
                                      FLASH_SR_WRPERR | FLASH_SR_PGAERR | \
                                      FLASH_SR_SIZERR | FLASH_SR_PGSERR | \
                                      FLASH_SR_MISERR | FLASH_SR_FASTERR | \
                                      FLASH_SR_OPTVERR)

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t data_size;
    uint32_t sequence;
    uint32_t crc32;
} NvmParamRecordHeader;

typedef struct
{
    uint32_t magic;          ///< Record header magic. / 记录头标记。
    uint16_t version;        ///< Save format version. / 保存格式版本。
    uint16_t data_size;      ///< Payload size in bytes. / 有效载荷字节数。
    uint32_t sequence;       ///< Monotonic record sequence. / 单调递增记录序号。
    uint32_t crc32;          ///< CRC32 over metadata and payload. / 元数据和载荷 CRC32。
    Param_SaveData data;     ///< Saved parameter payload. / 保存参数载荷。
    uint32_t commit_magic;   ///< Written last to mark a complete record. / 最后写入，用于标记记录完整。
} NvmParamRecord;

enum
{
    NVM_PARAM_SLOT_SIZE = (int)((sizeof(NvmParamRecord) + 7U) & ~7U),
    NVM_PARAM_SLOT_COUNT = (int)(NVM_PARAM_FLASH_PAGE_BYTES / ((sizeof(NvmParamRecord) + 7U) & ~7U))
};

_Static_assert(sizeof(NvmParamRecord) <= NVM_PARAM_FLASH_PAGE_BYTES, "NVM record is larger than one Flash page");
_Static_assert(NVM_PARAM_SLOT_COUNT > 0, "NVM Flash page cannot hold one record");
_Static_assert(sizeof(Param_SaveData) == 252U,
               "v8 parameter payload layout changed without a version bump");
_Static_assert(sizeof(NvmParamRecordHeader) == 16U, "NVM record header layout changed");

static uint32_t NvmParam_Crc32Update(uint32_t crc, const void *data, uint32_t size)
{
    const uint8_t *bytes = (const uint8_t *)data;

    while (size-- > 0U)
    {
        crc ^= *bytes++;
        for (uint8_t bit = 0; bit < 8U; bit++)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (crc >> 1U) ^ NVM_PARAM_CRC_POLY;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static uint32_t NvmParam_CalcRawCrc(const NvmParamRecordHeader *header,
                                    const void *data)
{
    uint32_t crc = NVM_PARAM_CRC_INIT;

    crc = NvmParam_Crc32Update(crc, &header->version, sizeof(header->version));
    crc = NvmParam_Crc32Update(crc, &header->data_size, sizeof(header->data_size));
    crc = NvmParam_Crc32Update(crc, &header->sequence, sizeof(header->sequence));
    crc = NvmParam_Crc32Update(crc, data, header->data_size);

    return ~crc;
}

static bool NvmParam_IsSlotErased(uint32_t address)
{
    const uint64_t *slot = (const uint64_t *)address;

    return *slot == UINT64_MAX;
}

static bool NvmParam_IsRecordValid(const NvmParamRecordHeader *record)
{
    const uint8_t *data = (const uint8_t *)record + sizeof(*record);
    const uint32_t *commit = (const uint32_t *)(data + sizeof(Param_SaveData));

    return record->magic == NVM_PARAM_MAGIC
        && record->version == NVM_PARAM_VERSION
        && record->data_size == sizeof(Param_SaveData)
        && *commit == NVM_PARAM_COMMIT_MAGIC
        && record->crc32 == NvmParam_CalcRawCrc(record, data);
}

static void NvmParam_CopyPidToSave(PID_SaveParam *dst, const PID_Int *src)
{
    dst->Kp = src->Kp;
    dst->Ki = src->Ki;
    dst->Kd = src->Kd;
    dst->integral_max = src->integral_max;
    dst->out_max = src->out_max;
    dst->out_min = src->out_min;
}

static void NvmParam_CopyPidFromSave(PID_Int *dst, const PID_SaveParam *src)
{
    dst->Kp = src->Kp;
    dst->Ki = src->Ki;
    dst->Kd = src->Kd;
    dst->integral_max = src->integral_max;
    dst->out_max = src->out_max;
    dst->out_min = src->out_min;

    dst->integral = 0;
    dst->prev_error = 0;
    dst->prev_prev_error = 0;
    dst->prev_feedback = 0;
    dst->prev_out = 0;
}


static bool NvmParam_IsSupportedBaud(uint32_t baud)
{
    return baud >= 115200UL && baud <= 2000000UL;
}

static uint8_t NvmParam_ClampNodeCount(uint8_t node_count)
{
    if (node_count < 1U || node_count > 4U)
    {
        return 1U;
    }
    return node_count;
}

static uint8_t NvmParam_ClampNodePosition(uint8_t node_position, uint8_t node_count)
{
    if (node_position < 1U || node_position > node_count)
    {
        return 1U;
    }
    return node_position;
}

static void NvmParam_FillSaveData(Param_SaveData *data, const Param *param)
{
    memset(data, 0, sizeof(*data));

    NvmParam_CopyPidToSave(&data->Pid_Pos, &param->Pid_Pos);
    NvmParam_CopyPidToSave(&data->Pid_PosVel, &param->Pid_PosVel);
    NvmParam_CopyPidToSave(&data->Pid_PosEle, &param->Pid_PosEle);
    memcpy(data->LowSpeedCompMap_mA, param->LowSpeedCompMap_mA,
           sizeof(data->LowSpeedCompMap_mA));
    data->CycleTimeMs = param->CycleTimeMs;
    data->TempLimit = param->TempLimit;

    data->EncoderOffset = param->EncoderOffset;
    data->PositionDeadbandCounts = param->PositionDeadbandCounts;
    data->LowSpeedCompMaxSpeed_cps = param->LowSpeedCompMaxSpeed_cps;
    data->SpeedMax = param->SpeedMax;
    data->AccelMax = param->AccelMax;
    data->DecelMax = param->DecelMax;
    data->EncoderVeer = param->EncoderVeer;

    data->DriveRunMode = param->DrivePwmMode;
    data->DrivePwmMode = param->DrivePwmMode;
    data->DriveVeerFlag = param->DriveVeerFlag;
    data->ExpectMA = param->ExpectMA;
    data->PowerSaveVoltage_mV = param->PowerSaveVoltage_mV;
    data->BaudRate = param->BaudRate;
    data->SerialWatchdogMs = param->SerialWatchdogMs;
    data->PdoMissLimit = param->PdoMissLimit;
    data->FailSafePolicy = param->FailSafePolicy;
    data->NodeId = param->NodeId;
    data->Topology = param->Topology;
    data->NodeCount = param->NodeCount;
    data->NodePosition = param->NodePosition;
    data->ReplySlotUs = param->ReplySlotUs;
    data->ControlSource = param->ControlSource;
    data->CrsfPositionChannel = param->CrsfPositionChannel;
    data->CrsfCenterChannel = param->CrsfCenterChannel;
    data->CrsfEnableChannel = param->CrsfEnableChannel;
    data->CrsfAutoEnable = param->CrsfAutoEnable;
    data->CrsfReserved164 = NVM_PARAM_LOW_SPEED_MARKER8;
    data->CrsfChannelMin = param->CrsfChannelMin;
    data->CrsfChannelCenter = param->CrsfChannelCenter;
    data->CrsfChannelMax = param->CrsfChannelMax;
    data->CrsfCenterTrigger = param->CrsfCenterTrigger;
    data->CrsfEnableThreshold = param->CrsfEnableThreshold;
    data->CrsfReserved176 = NVM_PARAM_LOW_SPEED_MARKER16;
    data->CrsfArmCurrentLimit_mA = param->CrsfArmCurrentLimit_mA;
    data->CrsfArmSpeed_cps = param->CrsfArmSpeed_cps;
    data->CrsfArmFollowError = param->CrsfArmFollowError;
    data->CrsfArmTimeoutMs = param->CrsfArmTimeoutMs;
    data->CrsfWatchdogMs = param->CrsfWatchdogMs;
    data->CrsfNegativePositionLimit = param->CrsfNegativePositionLimit;
    data->CrsfPositivePositionLimit = param->CrsfPositivePositionLimit;
    data->TorqueEncoderCountsPerRev = param->TorqueEncoderCountsPerRev;
    data->TorqueCurrentLimit_mA = param->TorqueCurrentLimit_mA;
    data->MotorTorqueParams = param->MotorTorqueParams;
    data->MechanicalParams = param->MechanicalParams;
    data->MotorInductance_uH = param->MotorInductance_uH;
    data->CurrentPeakLimit_mA = param->CurrentPeakLimit_mA;
    data->CurrentAbsoluteLimit_mA = param->CurrentAbsoluteLimit_mA;
    data->StallCurrentThreshold_mA = param->StallCurrentThreshold_mA;
    data->StallSpeedThreshold_cps = param->StallSpeedThreshold_cps;
    data->StallConfirmTimeMs = param->StallConfirmTimeMs;
}

static void NvmParam_ApplySaveData(Param *param, const Param_SaveData *data)
{
    NvmParam_CopyPidFromSave(&param->Pid_Pos, &data->Pid_Pos);
    NvmParam_CopyPidFromSave(&param->Pid_PosVel, &data->Pid_PosVel);
    NvmParam_CopyPidFromSave(&param->Pid_PosEle, &data->Pid_PosEle);

    param->CycleTimeMs = data->CycleTimeMs;
    param->TempLimit = (data->TempLimit >= 20 && data->TempLimit <= 85) ?
                       data->TempLimit : 40;

    param->EncoderOffset = data->EncoderOffset;
    param->SpeedMax = data->SpeedMax;
    param->PositionDeadbandCounts = data->PositionDeadbandCounts != 0U
                                        ? data->PositionDeadbandCounts
                                        : PID_POSITION_DEADBAND_DEFAULT_COUNTS;
    param->AccelMax = data->AccelMax;
    param->DecelMax = data->DecelMax;
    param->EncoderVeer = data->EncoderVeer;

    param->DriveRunMode = 0U;
    param->DrivePwmMode = (data->DrivePwmMode == 4U || data->DriveRunMode == 4U) ? 4U :
                          ((data->DrivePwmMode == 3U || data->DriveRunMode == 3U) ? 3U : 2U);
    param->DriveVeerFlag = data->DriveVeerFlag;
    param->ExpectMA = data->ExpectMA;
    param->PowerSaveVoltage_mV = data->PowerSaveVoltage_mV;
    param->BaudRate = NvmParam_IsSupportedBaud(data->BaudRate) ? data->BaudRate : 115200UL;
    /* Migrate the former factory default without overwriting an explicit custom value. */
    /* 仅迁移旧的出厂默认值，不覆盖用户主动保存的自定义值。 */
    param->SerialWatchdogMs = (data->SerialWatchdogMs == 100U || data->SerialWatchdogMs == 0U) ?
                              500U : data->SerialWatchdogMs;
    param->PdoMissLimit = (data->PdoMissLimit != 0U) ? data->PdoMissLimit : 3U;
    param->FailSafePolicy = (data->FailSafePolicy <= FAILSAFE_FALLBACK_PWM) ?
                            data->FailSafePolicy : FAILSAFE_DISABLE_OUTPUT;
    param->NodeId = (data->NodeId != 0U && data->NodeId < 0x7FU) ? data->NodeId : 1U;
    param->Topology = (data->Topology <= BUS_TOPOLOGY_CHAIN) ?
                      data->Topology : BUS_TOPOLOGY_PARALLEL;
    param->NodeCount = NvmParam_ClampNodeCount(data->NodeCount);
    param->NodePosition = NvmParam_ClampNodePosition(data->NodePosition, param->NodeCount);
    param->ReplySlotUs = (data->ReplySlotUs >= 50U && data->ReplySlotUs <= 8000U) ?
                         data->ReplySlotUs : 120U;
    param->ControlSource = data->ControlSource <= CONTROL_SOURCE_CRSF
                               ? data->ControlSource : CONTROL_SOURCE_PWM_INPUT;
    param->CrsfPositionChannel = data->CrsfPositionChannel <= 16U
                                     ? data->CrsfPositionChannel : 1U;
    param->CrsfCenterChannel = data->CrsfCenterChannel <= 16U
                                   ? data->CrsfCenterChannel : 0U;
    param->CrsfEnableChannel = data->CrsfEnableChannel <= 16U
                                    ? data->CrsfEnableChannel : 0U;
    param->CrsfAutoEnable = data->CrsfAutoEnable;
    if (data->CrsfChannelMin < data->CrsfChannelCenter
        && data->CrsfChannelCenter < data->CrsfChannelMax
        && data->CrsfChannelMax <= 2047U)
    {
        param->CrsfChannelMin = data->CrsfChannelMin;
        param->CrsfChannelCenter = data->CrsfChannelCenter;
        param->CrsfChannelMax = data->CrsfChannelMax;
    }
    param->CrsfCenterTrigger = data->CrsfCenterTrigger <= 2047U
                                   ? data->CrsfCenterTrigger : 1200U;
    param->CrsfEnableThreshold = data->CrsfEnableThreshold <= 2047U
                                    ? data->CrsfEnableThreshold : 1200U;
    param->CrsfArmCurrentLimit_mA = (data->CrsfArmCurrentLimit_mA >= 1U
                                     && data->CrsfArmCurrentLimit_mA <= 30000U)
                                        ? data->CrsfArmCurrentLimit_mA : 300U;
    param->CrsfArmSpeed_cps = (data->CrsfArmSpeed_cps >= 1U
                               && data->CrsfArmSpeed_cps <= 1000000UL)
                                  ? data->CrsfArmSpeed_cps : 5000U;
    param->CrsfArmFollowError = data->CrsfArmFollowError != 0U
                                    ? data->CrsfArmFollowError : 100U;
    param->CrsfArmTimeoutMs = (data->CrsfArmTimeoutMs >= 100U
                               && data->CrsfArmTimeoutMs <= 10000U)
                                  ? data->CrsfArmTimeoutMs : 2000U;
    param->CrsfWatchdogMs = (data->CrsfWatchdogMs >= 20U
                             && data->CrsfWatchdogMs <= 2000U)
                                ? data->CrsfWatchdogMs : 100U;
    if (data->CrsfNegativePositionLimit < data->CrsfPositivePositionLimit
        && data->CrsfNegativePositionLimit >= -1000000000L
        && data->CrsfPositivePositionLimit <= 1000000000L)
    {
        param->CrsfNegativePositionLimit = data->CrsfNegativePositionLimit;
        param->CrsfPositivePositionLimit = data->CrsfPositivePositionLimit;
        param->CrsfCenterReference = (data->CrsfNegativePositionLimit
                                      + data->CrsfPositivePositionLimit) >> 1;
    }
    param->TorqueEncoderCountsPerRev = data->TorqueEncoderCountsPerRev != 0U
                                           ? data->TorqueEncoderCountsPerRev : 16384U;
    param->TorqueCurrentLimit_mA = data->TorqueCurrentLimit_mA <= 30000U
                                       ? data->TorqueCurrentLimit_mA : 0U;
    param->MotorTorqueParams.torque_constant_uNm_per_A =
        data->MotorTorqueParams.torque_constant_uNm_per_A <= 1000000UL
            ? data->MotorTorqueParams.torque_constant_uNm_per_A : 0U;
    param->MotorTorqueParams.torque_temp_coefficient_ppm_per_C =
        (data->MotorTorqueParams.torque_temp_coefficient_ppm_per_C >= -10000L
         && data->MotorTorqueParams.torque_temp_coefficient_ppm_per_C <= 10000L)
            ? data->MotorTorqueParams.torque_temp_coefficient_ppm_per_C : 0;
    param->MotorTorqueParams.back_emf_uV_per_rpm =
        data->MotorTorqueParams.back_emf_uV_per_rpm <= 10000000UL
            ? data->MotorTorqueParams.back_emf_uV_per_rpm : 0U;
    param->MotorTorqueParams.terminal_resistance_mOhm =
        data->MotorTorqueParams.terminal_resistance_mOhm <= 10000000UL
            ? data->MotorTorqueParams.terminal_resistance_mOhm : 0U;
    param->MotorTorqueParams.resistance_temp_coefficient_ppm_per_C =
        data->MotorTorqueParams.resistance_temp_coefficient_ppm_per_C <= 10000U
            ? data->MotorTorqueParams.resistance_temp_coefficient_ppm_per_C : 4000U;
    param->MotorTorqueParams.brush_drop_mV =
        data->MotorTorqueParams.brush_drop_mV <= 5000U
            ? data->MotorTorqueParams.brush_drop_mV : 0U;
    param->MotorTorqueParams.reference_temperature_C =
        (data->MotorTorqueParams.reference_temperature_C >= -40
         && data->MotorTorqueParams.reference_temperature_C <= 200)
            ? data->MotorTorqueParams.reference_temperature_C : 25;
    param->MechanicalParams.total_inertia_ug_cm2 =
        data->MechanicalParams.total_inertia_ug_cm2 <= 100000000UL
            ? data->MechanicalParams.total_inertia_ug_cm2 : 0U;
    param->MechanicalParams.coulomb_friction_uNm =
        data->MechanicalParams.coulomb_friction_uNm <= 10000000UL
            ? data->MechanicalParams.coulomb_friction_uNm : 0U;
    param->MechanicalParams.viscous_friction_nNm_per_rpm =
        data->MechanicalParams.viscous_friction_nNm_per_rpm <= 10000000UL
            ? data->MechanicalParams.viscous_friction_nNm_per_rpm : 0U;
    param->MechanicalParams.friction_deadband_cps =
        data->MechanicalParams.friction_deadband_cps;
    param->MotorInductance_uH = (data->MotorInductance_uH >= 1U
                                 && data->MotorInductance_uH <= 10000U)
                                    ? data->MotorInductance_uH : 10U;
    if (data->CurrentPeakLimit_mA >= 100U
        && data->CurrentAbsoluteLimit_mA > data->CurrentPeakLimit_mA
        && data->CurrentAbsoluteLimit_mA <= 1830U)
    {
        param->CurrentPeakLimit_mA = data->CurrentPeakLimit_mA;
        param->CurrentAbsoluteLimit_mA = data->CurrentAbsoluteLimit_mA;
    }
    else
    {
        param->CurrentPeakLimit_mA = 1500U;
        param->CurrentAbsoluteLimit_mA = 1800U;
    }
    param->StallCurrentThreshold_mA =
        (data->StallCurrentThreshold_mA >= 50U
         && data->StallCurrentThreshold_mA <= 1500U)
            ? data->StallCurrentThreshold_mA : 300U;
    param->StallSpeedThreshold_cps =
        (data->StallSpeedThreshold_cps >= 10U)
            ? data->StallSpeedThreshold_cps : 300U;
    param->StallConfirmTimeMs =
        (data->StallConfirmTimeMs >= 500U
         && data->StallConfirmTimeMs <= 10000U)
            ? data->StallConfirmTimeMs : 3000U;
    if (data->CrsfReserved164 == NVM_PARAM_LOW_SPEED_MARKER8
        && data->CrsfReserved176 == NVM_PARAM_LOW_SPEED_MARKER16)
    {
        param->LowSpeedCompMaxSpeed_cps =
            (data->LowSpeedCompMaxSpeed_cps == 0U
             || (data->LowSpeedCompMaxSpeed_cps >= 500U
             && data->LowSpeedCompMaxSpeed_cps <= 5000U)
            )
                ? data->LowSpeedCompMaxSpeed_cps
                : 0U;
        memcpy(param->LowSpeedCompMap_mA, data->LowSpeedCompMap_mA,
               sizeof(param->LowSpeedCompMap_mA));
    }
    else
    {
        param->LowSpeedCompMaxSpeed_cps = 0U;
        memset(param->LowSpeedCompMap_mA, 0,
               sizeof(param->LowSpeedCompMap_mA));
    }
    param->MotorWindingTemperature_C =
        param->MotorTorqueParams.reference_temperature_C;
    /* Runtime arm state is never restored from Flash. / 运行期软使能状态绝不从 Flash 恢复。 */
    param->CrsfManualEnable = false;
    param->CrsfStatus = 0U;
}

static bool NvmParam_FindLatest(const NvmParamRecordHeader **latest,
                                uint32_t *next_slot)
{
    const NvmParamRecordHeader *best = NULL;
    uint32_t first_free = NVM_PARAM_SLOT_COUNT;

    for (uint32_t i = 0; i < (uint32_t)NVM_PARAM_SLOT_COUNT; i++)
    {
        uint32_t address = NVM_PARAM_FLASH_ADDR
                         + i * (uint32_t)NVM_PARAM_SLOT_SIZE;
        if (NvmParam_IsSlotErased(address))
        {
            first_free = i;
            break;
        }

        const NvmParamRecordHeader *record =
            (const NvmParamRecordHeader *)address;
        if (NvmParam_IsRecordValid(record)
            && (best == NULL || record->sequence >= best->sequence))
        {
            best = record;
        }
        first_free = i + 1U;
    }
    if (latest != NULL) *latest = best;
    if (next_slot != NULL) *next_slot = first_free;
    return best != NULL;
}

static bool NvmParam_FlashWaitReady(void)
{
    uint32_t timeout = 10000000UL;

    while ((FLASH->SR & (FLASH_SR_BSY1 | FLASH_SR_CFGBSY)) != 0U)
    {
        if (timeout-- == 0U)
        {
            return false;
        }
    }
    return true;
}

static bool NvmParam_FlashUnlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0U)
    {
        FLASH->KEYR = NVM_FLASH_KEY1;
        FLASH->KEYR = NVM_FLASH_KEY2;
    }
    return (FLASH->CR & FLASH_CR_LOCK) == 0U;
}

static void NvmParam_FlashLock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

static void NvmParam_FlashClearStatus(void)
{
    FLASH->SR = FLASH_SR_EOP | NVM_FLASH_ERROR_MASK;
}

static NvmParamStatus NvmParam_ErasePage(void)
{
    uint32_t page = (NVM_PARAM_FLASH_ADDR - FLASH_BASE) / NVM_PARAM_FLASH_PAGE_BYTES;
    bool success;

    if (!NvmParam_FlashWaitReady() || !NvmParam_FlashUnlock())
    {
        return NVM_PARAM_FLASH_ERROR;
    }
    NvmParam_FlashClearStatus();
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PNB)
              | FLASH_CR_PER | (page << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    success = NvmParam_FlashWaitReady()
              && ((FLASH->SR & NVM_FLASH_ERROR_MASK) == 0U);
    FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_PNB);
    NvmParam_FlashClearStatus();
    NvmParam_FlashLock();

    return success ? NVM_PARAM_OK : NVM_PARAM_FLASH_ERROR;
}

static NvmParamStatus NvmParam_WriteSlot(uint32_t slot_index, const NvmParamRecord *record)
{
    uint8_t write_buf[NVM_PARAM_SLOT_SIZE];
    bool success = true;

    /* Flash programming is 64-bit aligned on STM32G0, so pad the slot with 0xFF. */
    /* STM32G0 按 64 位双字写 Flash，因此槽位尾部用 0xFF 补齐。 */
    memset(write_buf, 0xFF, sizeof(write_buf));
    memcpy(write_buf, record, sizeof(*record));

    if (!NvmParam_FlashWaitReady() || !NvmParam_FlashUnlock())
    {
        return NVM_PARAM_FLASH_ERROR;
    }

    for (uint32_t offset = 0; offset < (uint32_t)NVM_PARAM_SLOT_SIZE; offset += 8U)
    {
        uint64_t double_word;

        memcpy(&double_word, &write_buf[offset], sizeof(double_word));
        uint32_t address = NVM_PARAM_FLASH_ADDR
                         + (slot_index * (uint32_t)NVM_PARAM_SLOT_SIZE) + offset;

        NvmParam_FlashClearStatus();
        FLASH->CR |= FLASH_CR_PG;
        *(volatile uint32_t *)address = (uint32_t)double_word;
        __ISB();
        *(volatile uint32_t *)(address + 4U) = (uint32_t)(double_word >> 32U);
        success = NvmParam_FlashWaitReady()
                  && ((FLASH->SR & NVM_FLASH_ERROR_MASK) == 0U);
        FLASH->CR &= ~FLASH_CR_PG;
        if (!success)
        {
            break;
        }
    }

    NvmParam_FlashClearStatus();
    NvmParam_FlashLock();

    return success ? NVM_PARAM_OK : NVM_PARAM_FLASH_ERROR;
}

NvmParamStatus NvmParam_Load(Param *param)
{
    const NvmParamRecordHeader *latest = NULL;

    if (param == NULL)
    {
        return NVM_PARAM_BAD_ARG;
    }

    if (NvmParam_FindLatest(&latest, NULL))
    {
        const Param_SaveData *data = (const Param_SaveData *)(latest + 1);
        NvmParam_ApplySaveData(param, data);
        return NVM_PARAM_OK;
    }

    return NVM_PARAM_EMPTY;
}

NvmParamStatus NvmParam_Save(const Param *param)
{
    const NvmParamRecordHeader *latest = NULL;
    NvmParamRecord record;
    Param_SaveData new_data;
    uint32_t next_slot = 0;
    uint32_t next_sequence = 1;

    if (param == NULL)
    {
        return NVM_PARAM_BAD_ARG;
    }

    NvmParam_FillSaveData(&new_data, param);
    (void)NvmParam_FindLatest(&latest, &next_slot);

    uint32_t unchanged = 0U;
    if (latest != NULL)
    {
        while (unchanged < sizeof(new_data)
               && ((const uint8_t *)(latest + 1))[unchanged]
                  == ((const uint8_t *)&new_data)[unchanged])
            ++unchanged;
    }
    if (unchanged == sizeof(new_data))
    {
        return NVM_PARAM_OK;
    }

    if (latest != NULL)
    {
        next_sequence = latest->sequence + 1U;
    }

    /* A record from an older layout can overlap the new slot boundaries. */
    /* 旧版记录可能跨越新版槽边界；首次保存新版格式前必须整页擦除。 */
    if ((latest == NULL && !NvmParam_IsSlotErased(NVM_PARAM_FLASH_ADDR))
        || next_slot >= (uint32_t)NVM_PARAM_SLOT_COUNT)
    {
        NvmParamStatus erase_status = NvmParam_ErasePage();
        if (erase_status != NVM_PARAM_OK)
        {
            return erase_status;
        }
        next_slot = 0;
    }

    memset(&record, 0xFF, sizeof(record));
    record.magic = NVM_PARAM_MAGIC;
    record.version = NVM_PARAM_VERSION;
    record.data_size = sizeof(Param_SaveData);
    record.sequence = next_sequence;
    record.data = new_data;
    record.crc32 = NvmParam_CalcRawCrc(
        (const NvmParamRecordHeader *)&record, &record.data);
    record.commit_magic = NVM_PARAM_COMMIT_MAGIC;

    return NvmParam_WriteSlot(next_slot, &record);
}

NvmParamStatus NvmParam_EraseAll(void)
{
    return NvmParam_ErasePage();
}

bool NvmParam_HasValidData(void)
{
    return NvmParam_FindLatest(NULL, NULL);
}

uint32_t NvmParam_GetLatestSequence(void)
{
    const NvmParamRecordHeader *latest = NULL;

    if (NvmParam_FindLatest(&latest, NULL))
    {
        return latest->sequence;
    }
    return 0U;
}
