/**
 * @file FixedPointMath.h
 * @brief Small fixed-point helpers for Cortex-M0+.
 * @brief 面向 Cortex-M0+ 的轻量定点运算辅助函数。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_FIXEDPOINTMATH_H
#define TRIPLE_CASCADECONTROLDCMOTOR_FIXEDPOINTMATH_H

#include <stdint.h>

/** Divide a signed 64-bit numerator by a positive 32-bit divisor and saturate. */
int32_t FixedPoint_DivideS64ByU32(int64_t numerator, uint32_t divisor);

/** Divide a signed 64-bit numerator by a positive 32-bit divisor exactly. */
int64_t FixedPoint_DivideS64ByU32ToS64(int64_t numerator, uint32_t divisor);

#endif /* TRIPLE_CASCADECONTROLDCMOTOR_FIXEDPOINTMATH_H */
