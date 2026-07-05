/**
 * @file MT6701.h
 * @brief MT6701 magnetic encoder interface.
 * @brief MT6701 磁编码器接口。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_MT6701_H
#define TRIPLE_CASCADECONTROLDCMOTOR_MT6701_H

#include <stdbool.h>

#include "i2c.h"
#include "TypeDefine.h"
#include "Filter.h"

#define MT6701_ADDR_7BIT        0x06U              ///< 7-bit I2C address. / 7 位 I2C 地址。
#define MT6701_ADDRESS          (MT6701_ADDR_7BIT << 1U) ///< HAL 8-bit address. / HAL 使用的 8 位地址。
#define MT6701_REG_ANGLE_H      0x03U              ///< Angle high-byte register. / 角度高字节寄存器。
#define ReadAddressH            MT6701_REG_ANGLE_H ///< Backward-compatible register name. / 兼容旧代码的寄存器名。
#define MT6701_I2C_TIMEOUT_MS   2U                 ///< Blocking read timeout. / 阻塞读取超时时间。

/**
 * @brief Runtime handle for MT6701 encoder sampling.
 * @brief MT6701 编码器采样运行句柄。
 */
typedef struct
{
    I2C_HandleTypeDef* iic;     ///< I2C bus handle. / I2C 总线句柄。
    Param* param;               ///< Shared runtime parameters. / 共享运行参数。
    bool dma_busy;              ///< I2C DMA busy flag. / I2C DMA 忙标志。
    LPF_Filter SpeedFilter;     ///< Speed low-pass filter. / 速度低通滤波器。
    LPF_Filter AccDecSpeedFilter; ///< Acceleration low-pass filter. / 加减速度低通滤波器。
} MT6701;

/**
 * @brief Initialize encoder handle and speed filters.
 * @brief 初始化编码器句柄和速度滤波器。
 */
void MT6701_init(MT6701* MT,I2C_HandleTypeDef *hi2c1,Param* params);

/**
 * @brief Read encoder angle and update multi-turn position.
 * @brief 读取编码器角度并更新多圈位置。
 */
void MT6701_Update(MT6701* MT);

/**
 * @brief Decode raw MT6701 bytes to corrected single-turn angle.
 * @brief 将 MT6701 原始字节解码为校正后的单圈角度。
 */
void MT6701_CodedManage(MT6701* MT);

/**
 * @brief Calculate encoder speed and acceleration from angle delta.
 * @brief 根据角度变化计算编码器速度和加速度。
 */
void MT6701_SpeedUpdate(MT6701* MT);

#endif // TRIPLE_CASCADECONTROLDCMOTOR_MT6701_H
