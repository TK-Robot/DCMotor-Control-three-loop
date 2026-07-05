//
// Created by Administrator on 2026/1/1.
//

#include "Filter.h"

// ======================== 一阶低通滤波器 ========================
void LPF_Filter_Init(LPF_Filter* Filter,uint16_t Alpha)
{
    Filter->prev_output = 0;
    Filter->alpha = Alpha; // 推荐 alpha = 64~128（低通系数越大，响应越慢）
}

int32_t LPF_Filter_Update(LPF_Filter* Filter, int32_t value)
{
    // y = y_prev + alpha * (x - y_prev) / 256
    int32_t diff = value - Filter->prev_output;
    Filter->prev_output += (diff * Filter->alpha) >> 8; // >>8 等价除以256
    return Filter->prev_output;
}

// ======================== 滑动平均滤波器 ========================
void MA_Filter_Init(MA_Filter* Filter,uint8_t Size)
{
    Filter->idx = 0;
    Filter->sum = 0;
    Filter->size = Size;
    for(uint8_t i=0;i<Filter->size;i++)
        Filter->buf[i] = 0;
}

int32_t MA_Filter_Update(MA_Filter* Filter, int32_t value)
{
    // 减去旧值
    Filter->sum -= Filter->buf[Filter->idx];

    // 写入新值
    Filter->buf[Filter->idx] = value;

    // 累加
    Filter->sum += value;

    // 更新索引
    Filter->idx++;
    if(Filter->idx >= Filter->size) Filter->idx = 0;

    // 返回平均值
    return Filter->sum / Filter->size;
}

// ======================== 整数卡尔曼滤波器 ========================
void Kalman_Filter_Init(Kalman_Filter* Filter, int32_t init_x, int32_t init_p, int32_t q, int32_t r)
{
    Filter->x = init_x;
    Filter->p = init_p;
    Filter->q = q;
    Filter->r = r;
}

int32_t Kalman_Filter_Update(Kalman_Filter* Filter, int32_t measurement)
{
    // 预测更新
    Filter->p += Filter->q;

    // 卡尔曼增益
    int32_t k = (Filter->p << 8) / (Filter->p + Filter->r); // >>8之后与alpha类似缩放
    Filter->x += ((k * (measurement - Filter->x)) >> 8);
    Filter->p = ((256 - k) * Filter->p) >> 8;

    return Filter->x;
}