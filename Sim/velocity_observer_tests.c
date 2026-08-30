#include <stdio.h>
#include <stdlib.h>

#include "VelocityObserver.h"

static int failures;

#define CHECK(condition) do { \
    if (!(condition)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        failures++; \
    } \
} while (0)

static void test_stationary_is_stable(void)
{
    VelocityObserver observer;

    VelocityObserver_Init(&observer);
    for (int sample = 0; sample < 200; ++sample)
    {
        CHECK(VelocityObserver_Update(&observer, 1234, 1000U) == 0);
    }
}

static void test_quantized_constant_velocity_converges(void)
{
    VelocityObserver observer;
    int32_t velocity = 0;

    VelocityObserver_Init(&observer);
    for (int sample = 0; sample < 1000; ++sample)
    {
        int32_t position = sample * 3 / 2;
        velocity = VelocityObserver_Update(&observer, position, 1000U);
    }
    CHECK(abs(velocity - 1500) < 30);
}

static void test_quantized_velocity_converges_within_40ms(void)
{
    VelocityObserver observer;
    int32_t velocity = 0;

    VelocityObserver_Init(&observer);
    for (int sample = 0; sample <= 40; ++sample)
    {
        int32_t position = sample * 3 / 2;
        velocity = VelocityObserver_Update(&observer, position, 1000U);
    }
    CHECK(abs(velocity - 1500) < 150);
}

static void test_sliding_window_updates_each_sample(void)
{
    VelocityObserver observer;
    int32_t previous_velocity = 0;
    int32_t velocity = 0;

    VelocityObserver_Init(&observer);
    for (int sample = 0; sample <= 128; ++sample)
        previous_velocity = VelocityObserver_Update(&observer, sample, 1000U);
    velocity = VelocityObserver_Update(&observer, 130, 1000U);
    CHECK(abs(previous_velocity - 1000) < 10);
    CHECK(velocity > previous_velocity);
}

static void test_one_count_stationary_jitter_is_bounded(void)
{
    VelocityObserver observer;
    int32_t maximum_velocity = 0;

    VelocityObserver_Init(&observer);
    for (int sample = 0; sample < 500; ++sample)
    {
        int32_t velocity = VelocityObserver_Update(
            &observer, 1234 + (sample & 1), 1000U);
        if (abs(velocity) > maximum_velocity) maximum_velocity = abs(velocity);
    }
    CHECK(maximum_velocity < 100);
}

static void test_large_position_jump_rebaselines(void)
{
    VelocityObserver observer;

    VelocityObserver_Init(&observer);
    (void)VelocityObserver_Update(&observer, 0, 1000U);
    (void)VelocityObserver_Update(&observer, 10, 1000U);
    CHECK(VelocityObserver_Update(&observer, 10000, 1000U) == 0);
}

static void test_variable_sample_period_uses_measured_time(void)
{
    VelocityObserver observer;
    int32_t position = 0;
    int32_t velocity = 0;

    VelocityObserver_Init(&observer);
    for (int sample = 0; sample < 400; ++sample)
    {
        const uint16_t period_us = (sample & 1) ? 2000U : 1000U;
        position += (period_us == 1000U) ? 2 : 4;
        velocity = VelocityObserver_Update(
            &observer, position, period_us);
    }
    CHECK(abs(velocity - 2000) < 50);
}

static void test_abrupt_sample_period_change_does_not_spike(void)
{
    VelocityObserver observer;
    int32_t position = 0;
    int32_t velocity = 0;

    VelocityObserver_Init(&observer);
    for (int sample = 0; sample < 64; ++sample)
    {
        position += 2;
        velocity = VelocityObserver_Update(&observer, position, 1000U);
    }
    for (int sample = 0; sample < 32; ++sample)
    {
        position += 20;
        velocity = VelocityObserver_Update(&observer, position, 10000U);
        CHECK(abs(velocity - 2000) < 50);
    }
}

int main(void)
{
    test_stationary_is_stable();
    test_quantized_constant_velocity_converges();
    test_quantized_velocity_converges_within_40ms();
    test_sliding_window_updates_each_sample();
    test_one_count_stationary_jitter_is_bounded();
    test_large_position_jump_rebaselines();
    test_variable_sample_period_uses_measured_time();
    test_abrupt_sample_period_change_does_not_spike();

    if (failures != 0)
    {
        printf("%d velocity-observer test(s) failed\n", failures);
        return 1;
    }
    puts("velocity-observer tests passed");
    return 0;
}
