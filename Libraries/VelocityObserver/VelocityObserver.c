/**
 * @file VelocityObserver.c
 * @brief Hybrid encoder edge-period velocity estimator implementation.
 * @brief 定点 alpha-beta 位置速度观测器实现。
 */

#include "VelocityObserver.h"

#include <stddef.h>

#define OBSERVER_RESET_ERROR_COUNTS 512L
#define OBSERVER_FILTER_TIME_US 16384U
#define OBSERVER_STOP_TIMEOUT_US 100000U

void VelocityObserver_Init(VelocityObserver *observer)
{
    if (observer == NULL) return;
    observer->last_position = 0;
    observer->velocity_cps = 0;
    observer->edge_elapsed_us = 0U;
    observer->initialized = false;
}

void VelocityObserver_Reset(VelocityObserver *observer,
                            int32_t measured_position)
{
    if (observer == NULL) return;
    observer->last_position = measured_position;
    observer->velocity_cps = 0;
    observer->edge_elapsed_us = 0U;
    observer->initialized = true;
}

int32_t VelocityObserver_Update(VelocityObserver *observer,
                                int32_t measured_position,
                                uint16_t sample_period_us)
{
    int32_t sample_delta;
    uint32_t elapsed_us;

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
    elapsed_us = observer->edge_elapsed_us + sample_period_us;
    observer->edge_elapsed_us = elapsed_us;

    if (sample_delta != 0)
    {
        int32_t measured_velocity = sample_delta * 1000000L
                                  / (int32_t)elapsed_us;
        uint32_t gain_q8 = elapsed_us >= OBSERVER_FILTER_TIME_US
                         ? 256U : (elapsed_us + 32U) >> 6;
        int32_t correction;

        correction = (measured_velocity - observer->velocity_cps)
                   * (int32_t)gain_q8;
        observer->velocity_cps += correction / 256L;
        observer->edge_elapsed_us = 0U;
    }
    else if (elapsed_us >= OBSERVER_STOP_TIMEOUT_US)
    {
        observer->velocity_cps = 0;
    }
    else
    {
        int32_t observable_limit = 1000000L / (int32_t)elapsed_us;

        /* With no new encoder edge, retain the reciprocal-period estimate but
         * reduce it once the elapsed time disproves the previous speed. */
        if (observer->velocity_cps > observable_limit)
            observer->velocity_cps = observable_limit;
        else if (observer->velocity_cps < -observable_limit)
            observer->velocity_cps = -observable_limit;
    }
    return observer->velocity_cps;
}
