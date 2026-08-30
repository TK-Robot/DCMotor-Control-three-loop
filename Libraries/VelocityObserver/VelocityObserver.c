/**
 * @file VelocityObserver.c
 * @brief Sliding-window encoder velocity estimator implementation.
 * @brief 定点 alpha-beta 位置速度观测器实现。
 */

#include "VelocityObserver.h"

#include <stddef.h>

#define OBSERVER_RESET_ERROR_COUNTS 512L
#define OBSERVER_VELOCITY_FILTER_SHIFT 4U

void VelocityObserver_Init(VelocityObserver *observer)
{
    if (observer == NULL) return;
    observer->last_position = 0;
    observer->velocity_cps = 0;
    observer->velocity_accumulator = 0;
    observer->initialized = false;
}

void VelocityObserver_Reset(VelocityObserver *observer,
                            int32_t measured_position)
{
    if (observer == NULL) return;
    observer->last_position = measured_position;
    observer->velocity_cps = 0;
    observer->velocity_accumulator = 0;
    observer->initialized = true;
}

int32_t VelocityObserver_Update(VelocityObserver *observer,
                                int32_t measured_position,
                                uint16_t sample_period_us)
{
    int32_t sample_delta;
    int32_t instantaneous_velocity_cps;

    if (observer == NULL || sample_period_us == 0U) return 0;
    if (!observer->initialized)
    {
        VelocityObserver_Reset(observer, measured_position);
        return 0;
    }

    sample_delta = measured_position - observer->last_position;
    if (sample_delta > OBSERVER_RESET_ERROR_COUNTS ||
        sample_delta < -OBSERVER_RESET_ERROR_COUNTS)
    {
        VelocityObserver_Reset(observer, measured_position);
        return 0;
    }
    observer->last_position = measured_position;
    instantaneous_velocity_cps = sample_delta * 1000000L
                               / (int32_t)sample_period_us;
    observer->velocity_accumulator +=
        instantaneous_velocity_cps - observer->velocity_cps;
    observer->velocity_cps = observer->velocity_accumulator
                           / (1L << OBSERVER_VELOCITY_FILTER_SHIFT);
    return observer->velocity_cps;
}
