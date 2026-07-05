/**
 * @file NvmParam.h
 * @brief Non-volatile parameter storage interface.
 * @brief 掉电参数保存接口。
 *
 * The module stores only Param_SaveData in the reserved Flash page. Runtime
 * state is copied in and out through Param so application modules do not need
 * to know the Flash layout.
 *
 * 该模块只把 Param_SaveData 写入预留 Flash 页。应用层通过 Param 传入和取出数据，
 * 不直接感知 Flash 地址布局。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_NVMPARAM_H
#define TRIPLE_CASCADECONTROLDCMOTOR_NVMPARAM_H

#include <stdbool.h>
#include <stdint.h>

#include "TypeDefine.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    NVM_PARAM_OK = 0,       ///< Operation succeeded. / 操作成功。
    NVM_PARAM_EMPTY,        ///< No valid record found. / 未找到有效记录。
    NVM_PARAM_BAD_ARG,      ///< Invalid function argument. / 函数参数无效。
    NVM_PARAM_FLASH_ERROR,  ///< Flash erase or program failed. / Flash 擦写失败。
    NVM_PARAM_CRC_ERROR     ///< Record CRC check failed. / 记录 CRC 校验失败。
} NvmParamStatus;

/**
 * @brief Load the latest valid saved parameters into the runtime Param block.
 * @brief 将最新有效保存参数加载到运行参数块。
 *
 * @param param Runtime parameter block. / 运行参数块。
 * @return Load status. / 加载状态。
 */
NvmParamStatus NvmParam_Load(Param *param);

/**
 * @brief Save current configurable parameters if they changed.
 * @brief 配置参数发生变化时保存当前参数。
 *
 * @param param Runtime parameter block. / 运行参数块。
 * @return Save status. / 保存状态。
 */
NvmParamStatus NvmParam_Save(const Param *param);

/**
 * @brief Erase all saved parameter records.
 * @brief 清除所有已保存参数记录。
 *
 * @return Erase status. / 擦除状态。
 */
NvmParamStatus NvmParam_EraseAll(void);

/**
 * @brief Check whether Flash contains a valid parameter record.
 * @brief 检查 Flash 中是否存在有效参数记录。
 *
 * @return true if a valid record exists. / 存在有效记录时返回 true。
 */
bool NvmParam_HasValidData(void);

/**
 * @brief Get the sequence number of the latest valid record.
 * @brief 获取最新有效记录的序号。
 *
 * @return Latest sequence number, or 0 when empty. / 最新序号；无记录时返回 0。
 */
uint32_t NvmParam_GetLatestSequence(void);

#ifdef __cplusplus
}
#endif

#endif
