/**
 * @file FixedPointMath.c
 * @brief Compact signed 64-by-32 division for fixed-point model results.
 * @brief 用于定点模型结果的紧凑型有符号 64 除 32 实现。
 */

#include "FixedPointMath.h"

#include <stdbool.h>
#include <limits.h>

int64_t FixedPoint_DivideS64ByU32ToS64(int64_t numerator, uint32_t divisor)
{
    bool negative = numerator < 0;
    uint64_t magnitude;
    uint32_t high;
    uint32_t low;
    uint32_t quotient_high;
    uint32_t quotient_low;
    uint64_t result;

    if (divisor == 0U) return negative ? INT64_MIN : INT64_MAX;

    /* Avoid signed overflow for INT64_MIN. */
    magnitude = negative ? (uint64_t)(-(numerator + 1)) + 1U
                         : (uint64_t)numerator;
    high = (uint32_t)(magnitude >> 32);
    low = (uint32_t)magnitude;
    if (high == 0U)
    {
        quotient_high = 0U;
        quotient_low = low / divisor;
    }
    else
    {
        quotient_high = high / divisor;
        uint64_t remainder = high % divisor;
        uint32_t mask = 0x80000000UL;

        quotient_low = 0U;
        while (mask != 0U)
        {
            remainder = (remainder << 1) | ((low & mask) != 0U);
            if (remainder >= divisor)
            {
                remainder -= divisor;
                quotient_low |= mask;
            }
            mask >>= 1;
        }
    }
    result = ((uint64_t)quotient_high << 32) | quotient_low;

    if (!negative) return (int64_t)result;
    if (result == ((uint64_t)INT64_MAX + 1U)) return INT64_MIN;
    return -(int64_t)result;
}

int32_t FixedPoint_DivideS64ByU32(int64_t numerator, uint32_t divisor)
{
    int64_t result = FixedPoint_DivideS64ByU32ToS64(numerator, divisor);

    if (result > INT32_MAX) return INT32_MAX;
    if (result < INT32_MIN) return INT32_MIN;
    return (int32_t)result;
}
