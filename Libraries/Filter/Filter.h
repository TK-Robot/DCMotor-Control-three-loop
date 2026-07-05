//
// Created by Administrator on 2026/1/1.
//

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_FILTER_H
#define TRIPLE_CASCADECONTROLDCMOTOR_FILTER_H

#include "TypeDefine.h"

// ======================== 一阶低通滤波器 ========================
typedef struct
{
    // 属性
    int32_t prev_output;  // 上一次输出
    uint16_t alpha;       // 滤波系数，0~256（比例 alpha/256）
} LPF_Filter; //一阶低通滤波器

void LPF_Filter_Init(LPF_Filter* Filter,uint16_t Alpha);
int32_t LPF_Filter_Update(LPF_Filter* Filter, int32_t value);


// ======================== 滑动平均滤波器 ========================
typedef struct
{
    // 属性
    int32_t* buf; // 存储历史速度值
    uint8_t idx; // 当前写入索引
    int32_t sum; // 历史值累加和
    uint8_t size; //窗口长度
} MA_Filter; //滑动平均滤波器

void MA_Filter_Init(MA_Filter* Filter,uint8_t Size);
int32_t MA_Filter_Update(MA_Filter* Filter, int32_t value);


// ======================== 整数卡尔曼滤波器 ========================
// 简单一维卡尔曼滤波，速度或位置
typedef struct
{
    int32_t x;      // 滤波值
    int32_t p;      // 估计误差协方差
    int32_t q;      // 过程噪声（固定）
    int32_t r;      // 测量噪声（固定）
} Kalman_Filter;//整数卡尔曼滤波器

void Kalman_Filter_Init(Kalman_Filter* Filter, int32_t init_x, int32_t init_p, int32_t q, int32_t r);
int32_t Kalman_Filter_Update(Kalman_Filter* Filter, int32_t measurement);

#endif //TRIPLE_CASCADECONTROLDCMOTOR_FILTER_H