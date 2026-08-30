/**
 * @file VelocityObserver.h
 * @brief Sliding-window encoder velocity estimator.
 * @brief 定点 alpha-beta 位置速度观测器。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_VELOCITYOBSERVER_H
#define TRIPLE_CASCADECONTROLDCMOTOR_VELOCITYOBSERVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t last_position;
    int32_t velocity_cps;
    int32_t velocity_accumulator;
    bool initialized;
} VelocityObserver;

void VelocityObserver_Init(VelocityObserver *observer);
void VelocityObserver_Reset(VelocityObserver *observer,
                            int32_t measured_position);
int32_t VelocityObserver_Update(VelocityObserver *observer,
                                int32_t measured_position,
                                uint16_t sample_period_us);

#endif /* TRIPLE_CASCADECONTROLDCMOTOR_VELOCITYOBSERVER_H */
