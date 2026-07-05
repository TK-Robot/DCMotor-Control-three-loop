/**
 * @file Filter.h
 * @brief Integer filters used by control and measurement modules.
 * @brief 控制和测量模块使用的整数滤波器。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_FILTER_H
#define TRIPLE_CASCADECONTROLDCMOTOR_FILTER_H

#include "TypeDefine.h"

/**
 * @brief First-order low-pass filter.
 * @brief 一阶低通滤波器。
 */
typedef struct
{
    int32_t prev_output; ///< Previous output value. / 上一次输出值。
    uint16_t alpha;      ///< Filter coefficient, 0..256. / 滤波系数，范围 0..256。
} LPF_Filter;

/**
 * @brief Initialize first-order low-pass filter.
 * @brief 初始化一阶低通滤波器。
 */
void LPF_Filter_Init(LPF_Filter* Filter,uint16_t Alpha);

/**
 * @brief Update low-pass filter with a new sample.
 * @brief 使用新采样值更新低通滤波器。
 */
int32_t LPF_Filter_Update(LPF_Filter* Filter, int32_t value);

/**
 * @brief Moving-average filter.
 * @brief 滑动平均滤波器。
 */
typedef struct
{
    int32_t* buf; ///< External history buffer. / 外部历史数据缓冲区。
    uint8_t idx;  ///< Current write index. / 当前写入索引。
    int32_t sum;  ///< Running sum. / 窗口累计和。
    uint8_t size; ///< Window length. / 窗口长度。
} MA_Filter;

/**
 * @brief Initialize moving-average filter.
 * @brief 初始化滑动平均滤波器。
 */
void MA_Filter_Init(MA_Filter* Filter,uint8_t Size);

/**
 * @brief Update moving-average filter with a new sample.
 * @brief 使用新采样值更新滑动平均滤波器。
 */
int32_t MA_Filter_Update(MA_Filter* Filter, int32_t value);

/**
 * @brief Simple integer one-dimensional Kalman filter.
 * @brief 简单的一维整数卡尔曼滤波器。
 */
typedef struct
{
    int32_t x; ///< Estimated value. / 估计值。
    int32_t p; ///< Estimate covariance. / 估计误差协方差。
    int32_t q; ///< Process noise. / 过程噪声。
    int32_t r; ///< Measurement noise. / 测量噪声。
} Kalman_Filter;

/**
 * @brief Initialize integer Kalman filter.
 * @brief 初始化整数卡尔曼滤波器。
 */
void Kalman_Filter_Init(Kalman_Filter* Filter, int32_t init_x, int32_t init_p, int32_t q, int32_t r);

/**
 * @brief Update Kalman filter with a new measurement.
 * @brief 使用新测量值更新卡尔曼滤波器。
 */
int32_t Kalman_Filter_Update(Kalman_Filter* Filter, int32_t measurement);

#endif // TRIPLE_CASCADECONTROLDCMOTOR_FILTER_H
