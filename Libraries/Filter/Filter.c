/**
 * @file Filter.c
 * @brief Integer filter implementations.
 * @brief 整数滤波器实现。
 */

#include "Filter.h"

void LPF_Filter_Init(LPF_Filter* Filter,uint16_t Alpha)
{
    Filter->prev_output = 0;
    Filter->alpha = Alpha;
}

int32_t LPF_Filter_Update(LPF_Filter* Filter, int32_t value)
{
    /* y = y_prev + alpha * (x - y_prev) / 256. */
    /* 一阶低通：alpha 越大，响应越快；alpha 越小，滤波越强。 */
    int32_t diff = value - Filter->prev_output;
    Filter->prev_output += (diff * Filter->alpha) >> 8;
    return Filter->prev_output;
}

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
    /* Replace the oldest sample and keep a running sum for O(1) update. */
    /* 替换最旧采样并维护累计和，使每次更新为 O(1)。 */
    Filter->sum -= Filter->buf[Filter->idx];
    Filter->buf[Filter->idx] = value;
    Filter->sum += value;

    Filter->idx++;
    if(Filter->idx >= Filter->size) Filter->idx = 0;

    return Filter->sum / Filter->size;
}

void Kalman_Filter_Init(Kalman_Filter* Filter, int32_t init_x, int32_t init_p, int32_t q, int32_t r)
{
    Filter->x = init_x;
    Filter->p = init_p;
    Filter->q = q;
    Filter->r = r;
}

int32_t Kalman_Filter_Update(Kalman_Filter* Filter, int32_t measurement)
{
    /* 8-bit fixed-point gain keeps the filter lightweight on Cortex-M0+. */
    /* 使用 8 位定点增益，适合 Cortex-M0+ 上的轻量计算。 */
    Filter->p += Filter->q;

    int32_t k = (Filter->p << 8) / (Filter->p + Filter->r);
    Filter->x += ((k * (measurement - Filter->x)) >> 8);
    Filter->p = ((256 - k) * Filter->p) >> 8;

    return Filter->x;
}
