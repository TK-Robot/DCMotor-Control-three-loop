#include "NvmParam.h"

#include <string.h>

#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_flash_ex.h"

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
#define NVM_PARAM_FLASH_ADDR         (FLASH_BASE + NVM_PARAM_FLASH_SIZE_BYTES - FLASH_PAGE_SIZE)
#define NVM_PARAM_MAGIC              (0x56504B54UL)
#define NVM_PARAM_COMMIT_MAGIC       (0x54494D43UL)
#define NVM_PARAM_VERSION            (1U)
#define NVM_PARAM_CRC_INIT           (0xFFFFFFFFUL)
#define NVM_PARAM_CRC_POLY           (0xEDB88320UL)

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
    NVM_PARAM_SLOT_COUNT = (int)(FLASH_PAGE_SIZE / ((sizeof(NvmParamRecord) + 7U) & ~7U))
};

_Static_assert(sizeof(NvmParamRecord) <= FLASH_PAGE_SIZE, "NVM record is larger than one Flash page");
_Static_assert(NVM_PARAM_SLOT_COUNT > 0, "NVM Flash page cannot hold one record");

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

static uint32_t NvmParam_CalcCrc(const NvmParamRecord *record)
{
    uint32_t crc = NVM_PARAM_CRC_INIT;

    crc = NvmParam_Crc32Update(crc, &record->version, sizeof(record->version));
    crc = NvmParam_Crc32Update(crc, &record->data_size, sizeof(record->data_size));
    crc = NvmParam_Crc32Update(crc, &record->sequence, sizeof(record->sequence));
    crc = NvmParam_Crc32Update(crc, &record->data, sizeof(record->data));

    return ~crc;
}

static bool NvmParam_IsSlotErased(uint32_t address)
{
    const uint64_t *slot = (const uint64_t *)address;

    return *slot == UINT64_MAX;
}

static const NvmParamRecord *NvmParam_RecordAt(uint32_t slot_index)
{
    return (const NvmParamRecord *)(NVM_PARAM_FLASH_ADDR + (slot_index * (uint32_t)NVM_PARAM_SLOT_SIZE));
}

static bool NvmParam_IsRecordValid(const NvmParamRecord *record)
{
    if (record->magic != NVM_PARAM_MAGIC)
    {
        return false;
    }

    if (record->commit_magic != NVM_PARAM_COMMIT_MAGIC)
    {
        return false;
    }

    if (record->version != NVM_PARAM_VERSION)
    {
        return false;
    }

    if (record->data_size != sizeof(Param_SaveData))
    {
        return false;
    }

    return record->crc32 == NvmParam_CalcCrc(record);
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

static void NvmParam_FillSaveData(Param_SaveData *data, const Param *param)
{
    memset(data, 0, sizeof(*data));

    NvmParam_CopyPidToSave(&data->Pid_Pos, &param->Pid_Pos);
    NvmParam_CopyPidToSave(&data->Pid_PosVel, &param->Pid_PosVel);
    NvmParam_CopyPidToSave(&data->Pid_PosEle, &param->Pid_PosEle);
    NvmParam_CopyPidToSave(&data->Pid_Vel, &param->Pid_Vel);
    NvmParam_CopyPidToSave(&data->Pid_Ele, &param->Pid_Ele);

    data->CycleTimeMs = param->CycleTimeMs;
    data->TempLimit = param->TempLimit;

    data->EncoderOffset = param->EncoderOffset;
    data->EncoderExpect = param->EncoderExpect;
    data->EncoderSpeedExpect = param->EncoderSpeedExpect;
    data->SpeedMax = param->SpeedMax;
    data->AccelMax = param->AccelMax;
    data->DecelMax = param->DecelMax;
    data->EncoderVeer = param->EncoderVeer;

    data->DriveRunMode = param->DriveRunMode;
    data->DriveVeerFlag = param->DriveVeerFlag;
    data->ExpectMA = param->ExpectMA;
    data->PowerSaveVoltage_mV = param->PowerSaveVoltage_mV;
}

static void NvmParam_ApplySaveData(Param *param, const Param_SaveData *data)
{
    NvmParam_CopyPidFromSave(&param->Pid_Pos, &data->Pid_Pos);
    NvmParam_CopyPidFromSave(&param->Pid_PosVel, &data->Pid_PosVel);
    NvmParam_CopyPidFromSave(&param->Pid_PosEle, &data->Pid_PosEle);
    NvmParam_CopyPidFromSave(&param->Pid_Vel, &data->Pid_Vel);
    NvmParam_CopyPidFromSave(&param->Pid_Ele, &data->Pid_Ele);

    param->CycleTimeMs = data->CycleTimeMs;
    param->TempLimit = data->TempLimit;

    param->EncoderOffset = data->EncoderOffset;
    param->EncoderExpect = data->EncoderExpect;
    param->EncoderSpeedExpect = data->EncoderSpeedExpect;
    param->SpeedMax = data->SpeedMax;
    param->AccelMax = data->AccelMax;
    param->DecelMax = data->DecelMax;
    param->EncoderVeer = data->EncoderVeer;

    param->DriveRunMode = data->DriveRunMode;
    param->DriveVeerFlag = data->DriveVeerFlag;
    param->ExpectMA = data->ExpectMA;
    param->PowerSaveVoltage_mV = data->PowerSaveVoltage_mV;
}

static bool NvmParam_FindLatest(const NvmParamRecord **latest, uint32_t *next_slot)
{
    const NvmParamRecord *best = NULL;
    uint32_t first_free = NVM_PARAM_SLOT_COUNT;

    for (uint32_t i = 0; i < (uint32_t)NVM_PARAM_SLOT_COUNT; i++)
    {
        uint32_t address = NVM_PARAM_FLASH_ADDR + (i * (uint32_t)NVM_PARAM_SLOT_SIZE);

        if (NvmParam_IsSlotErased(address))
        {
            first_free = i;
            break;
        }

        const NvmParamRecord *record = NvmParam_RecordAt(i);
        if (NvmParam_IsRecordValid(record))
        {
            if ((best == NULL) || (record->sequence >= best->sequence))
            {
                best = record;
            }
        }

        first_free = i + 1U;
    }

    if (latest != NULL)
    {
        *latest = best;
    }

    if (next_slot != NULL)
    {
        *next_slot = first_free;
    }

    return best != NULL;
}

static NvmParamStatus NvmParam_ErasePage(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0;
    HAL_StatusTypeDef status;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (NVM_PARAM_FLASH_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
    erase.NbPages = 1;

    status = HAL_FLASH_Unlock();
    if (status == HAL_OK)
    {
        status = HAL_FLASHEx_Erase(&erase, &page_error);
        (void)HAL_FLASH_Lock();
    }

    return (status == HAL_OK) ? NVM_PARAM_OK : NVM_PARAM_FLASH_ERROR;
}

static NvmParamStatus NvmParam_WriteSlot(uint32_t slot_index, const NvmParamRecord *record)
{
    uint8_t write_buf[NVM_PARAM_SLOT_SIZE];
    HAL_StatusTypeDef status;

    /* Flash programming is 64-bit aligned on STM32G0, so pad the slot with 0xFF. */
    /* STM32G0 按 64 位双字写 Flash，因此槽位尾部用 0xFF 补齐。 */
    memset(write_buf, 0xFF, sizeof(write_buf));
    memcpy(write_buf, record, sizeof(*record));

    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        return NVM_PARAM_FLASH_ERROR;
    }

    for (uint32_t offset = 0; offset < (uint32_t)NVM_PARAM_SLOT_SIZE; offset += 8U)
    {
        uint64_t double_word;

        memcpy(&double_word, &write_buf[offset], sizeof(double_word));
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   NVM_PARAM_FLASH_ADDR + (slot_index * (uint32_t)NVM_PARAM_SLOT_SIZE) + offset,
                                   double_word);
        if (status != HAL_OK)
        {
            break;
        }
    }

    (void)HAL_FLASH_Lock();

    return (status == HAL_OK) ? NVM_PARAM_OK : NVM_PARAM_FLASH_ERROR;
}

NvmParamStatus NvmParam_Load(Param *param)
{
    const NvmParamRecord *latest = NULL;

    if (param == NULL)
    {
        return NVM_PARAM_BAD_ARG;
    }

    if (!NvmParam_FindLatest(&latest, NULL))
    {
        return NVM_PARAM_EMPTY;
    }

    if (!NvmParam_IsRecordValid(latest))
    {
        return NVM_PARAM_CRC_ERROR;
    }

    NvmParam_ApplySaveData(param, &latest->data);
    return NVM_PARAM_OK;
}

NvmParamStatus NvmParam_Save(const Param *param)
{
    const NvmParamRecord *latest = NULL;
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

    if ((latest != NULL) && (memcmp(&latest->data, &new_data, sizeof(new_data)) == 0))
    {
        return NVM_PARAM_OK;
    }

    if (latest != NULL)
    {
        next_sequence = latest->sequence + 1U;
    }

    if (next_slot >= (uint32_t)NVM_PARAM_SLOT_COUNT)
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
    record.crc32 = NvmParam_CalcCrc(&record);
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
    const NvmParamRecord *latest = NULL;

    if (!NvmParam_FindLatest(&latest, NULL))
    {
        return 0;
    }

    return latest->sequence;
}
