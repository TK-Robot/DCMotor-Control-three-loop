#include <stdio.h>
#include <stdlib.h>

#include "CurrentControl.h"
#include "Filter.h"

static int failures;

static const CurrentControlElectrical electrical = {
    .absolute_limit_mA = 1750U
};

#define CHECK(condition) do { \
    if (!(condition)) { \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        failures++; \
    } \
} while (0)

static void test_disabled_output(void)
{
    CurrentControl control;
    CurrentControlOutput output;

    CurrentControl_Init(&control);
    output = CurrentControl_Step(&control, 0, false);
    CHECK(output.power_permille == 0);
    CHECK(output.drive_mode == 0U);
}

static void test_low_duty_preserves_requested_voltage(void)
{
    CurrentControl control;
    CurrentControlOutput output;
    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, 100, 32, 4U, &electrical);
    for (int i = 0; i < 200; ++i)
    {
        output = CurrentControl_Step(&control, 0, false);
        CHECK(output.power_permille == 32);
        CHECK(output.drive_mode == 2U);
    }
}

static void test_uncalibrated_inductance_does_not_limit_voltage(void)
{
    CurrentControl control;
    CurrentControlOutput output;

    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, 400, 200, 4U, &electrical);
    output = CurrentControl_Step(&control, 0, false);
    CHECK(output.power_permille == 200);
    CHECK(output.drive_mode == 2U);

    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, -400, -200, 4U, &electrical);
    output = CurrentControl_Step(&control, 0, false);
    CHECK(output.power_permille == -200);
    CHECK(output.drive_mode == 2U);
}

static void test_peak_sample_does_not_become_cycle_average(void)
{
    CurrentControl control;
    CurrentControlOutput output;
    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, 200, 100, 4U, &electrical);
    (void)CurrentControl_Step(&control, 0, false);
    output = CurrentControl_Step(&control, 100, true);
    CHECK(output.power_permille == 100);
    CHECK(output.drive_mode == 2U);

    output = CurrentControl_Step(&control, 300, true);
    CHECK(output.power_permille == 100);
    CHECK(output.drive_mode == 2U);
}

static void test_unqualified_samples_do_not_change_voltage_command(void)
{
    CurrentControl control;
    CurrentControlOutput output = {0};
    int32_t accumulated_pwm = 0;
    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, 100, 32, 4U, &electrical);
    for (int cycle = 0; cycle < 200; ++cycle)
    {
        output = CurrentControl_Step(&control, 0, false);
        accumulated_pwm += output.power_permille;
    }
    CHECK(accumulated_pwm == 32 * 200);
}

static void test_reversal_forces_decay(void)
{
    CurrentControl control;
    CurrentControlOutput output;

    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, 200, 100, 4U, &electrical);
    (void)CurrentControl_Step(&control, 150, true);
    CurrentControl_SetCommand(&control, true, -200, -100, 4U, &electrical);
    output = CurrentControl_Step(&control, 1600, true);
    CHECK(output.power_permille == 0);
    CHECK(output.drive_mode == 0U);
    CHECK(control.peak_chop_events == 0U);
    for (uint8_t cycle = 1U;
         cycle < CURRENT_CONTROL_REVERSAL_COAST_CYCLES; ++cycle)
    {
        output = CurrentControl_Step(&control, 1600, true);
        CHECK(output.power_permille == 0);
        CHECK(control.peak_chop_events == 0U);
    }
    output = CurrentControl_Step(&control, 0, false);
    CHECK(output.power_permille == -100);
    CHECK(output.drive_mode == 3U);

    /* Switching to the opposite bridge direction produces a short INA181/ADC
     * settling transient. The target-transient hold must ignore it for soft
     * chopping in both directions while absolute protection stays armed. */
    for (uint8_t cycle = 1U;
         cycle < CURRENT_CONTROL_AUTO_FAST_HOLD_CYCLES; ++cycle)
    {
        output = CurrentControl_Step(&control, 1600, true);
        CHECK(control.peak_chop_events == 0U);
    }
    output = CurrentControl_Step(&control, 1600, true);
    CHECK(control.peak_chop_events == 0U);

    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, -200, -100, 4U, &electrical);
    (void)CurrentControl_Step(&control, 150, true);
    CurrentControl_SetCommand(&control, true, 200, 100, 4U, &electrical);
    for (uint8_t cycle = 0U;
         cycle < CURRENT_CONTROL_REVERSAL_COAST_CYCLES; ++cycle)
    {
        output = CurrentControl_Step(&control, 1600, true);
        CHECK(output.power_permille == 0);
        CHECK(control.peak_chop_events == 0U);
    }
    for (uint8_t cycle = 0U;
         cycle < CURRENT_CONTROL_AUTO_FAST_HOLD_CYCLES; ++cycle)
    {
        output = CurrentControl_Step(&control, 1600, true);
        CHECK(control.peak_chop_events == 0U);
    }
    output = CurrentControl_Step(&control, 1600, true);
    CHECK(control.peak_chop_events == 0U);

    /* Absolute protection remains active while reversal coast owns output. */
    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, 200, 100, 4U, &electrical);
    (void)CurrentControl_Step(&control, 150, true);
    CurrentControl_SetCommand(&control, true, -200, -100, 4U, &electrical);
    output = CurrentControl_Step(&control, 1750, true);
    CHECK(output.hard_limit_active);
    CHECK(output.power_permille == 0);
}

static void test_auto_decay_does_not_compare_peak_with_average_target(void)
{
    CurrentControl control;
    CurrentControlOutput output;

    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, 200, 300, 4U, &electrical);
    (void)CurrentControl_Step(&control, 0, false);
    for (uint8_t cycle = 0U; cycle < 16U; ++cycle)
        output = CurrentControl_Step(&control, 800, true);
    CHECK(output.drive_mode == 2U);
}

static void test_low_duty_auto_decay_starts_slow(void)
{
    CurrentControl control;
    CurrentControlOutput output;

    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, 100, 32, 4U, &electrical);

    output = CurrentControl_Step(&control, 0, false);
    CHECK(output.power_permille == 32);
    CHECK(output.drive_mode == 2U);
}

static void test_auto_decay_reacts_to_target_drop(void)
{
    CurrentControl control;
    CurrentControlOutput output;

    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, 300, 300, 4U, &electrical);
    output = CurrentControl_Step(&control, 300, true);
    CHECK(output.drive_mode == 2U);

    CurrentControl_SetCommand(&control, true, 200, 200, 4U, &electrical);
    output = CurrentControl_Step(&control, 0, false);
    CHECK(output.power_permille == 200);
    CHECK(output.drive_mode == 3U);
}

static void test_explicit_decay_modes_are_not_automatic(void)
{
    CurrentControl control;
    CurrentControlOutput output;

    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, 200, 300, 2U, &electrical);
    output = CurrentControl_Step(&control, 400, true);
    CHECK(output.drive_mode == 2U);

    CurrentControl_SetCommand(&control, true, 100, 100, 3U, &electrical);
    output = CurrentControl_Step(&control, 0, false);
    CHECK(output.drive_mode == 3U);
}

static void test_model_brake_uses_brake_coast_mode(void)
{
    CurrentControl control;
    CurrentControlOutput output;

    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, -100, -120, 1U,
                              &electrical);
    output = CurrentControl_Step(&control, 0, false);
    CHECK(output.power_permille == -120);
    CHECK(output.drive_mode == 1U);
}

static void test_absolute_guard_is_non_latching(void)
{
    CurrentControl control;
    CurrentControlOutput output;

    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, 500, 300, 4U, &electrical);
    (void)CurrentControl_Step(&control, 0, false);
    output = CurrentControl_Step(&control, 1501, true);
    CHECK(output.power_permille == 300);
    CHECK(output.drive_mode == 2U);
    CHECK(!output.hard_limit_active);
    CHECK(control.peak_chop_events == 0U);

    output = CurrentControl_Step(&control, 1750, true);
    CHECK(output.power_permille == 0);
    CHECK(output.drive_mode == 0U);
    CHECK(output.hard_limit_active);
    CHECK(control.peak_chop_events == 1U);

    output = CurrentControl_Step(&control, 0, false);
    CHECK(output.power_permille == 300);
    CHECK(output.drive_mode == 2U);
    CHECK(!output.hard_limit_active);
}

static void test_single_filtered_spike_does_not_reach_guard(void)
{
    CurrentControl control;
    CurrentControlOutput output;
    LPF_Filter filter;

    CurrentControl_Init(&control);
    CurrentControl_SetCommand(&control, true, 100, 300, 4U, &electrical);
    LPF_Filter_Init(&filter, 128U);
    filter.prev_output = 400;

    output = CurrentControl_Step(
        &control, (int16_t)LPF_Filter_Update(&filter, 1754), true);
    CHECK(!output.hard_limit_active);
    output = CurrentControl_Step(
        &control, (int16_t)LPF_Filter_Update(&filter, 400), true);
    CHECK(!output.hard_limit_active);
}

int main(void)
{
    test_disabled_output();
    test_low_duty_preserves_requested_voltage();
    test_uncalibrated_inductance_does_not_limit_voltage();
    test_peak_sample_does_not_become_cycle_average();
    test_unqualified_samples_do_not_change_voltage_command();
    test_reversal_forces_decay();
    test_auto_decay_does_not_compare_peak_with_average_target();
    test_low_duty_auto_decay_starts_slow();
    test_auto_decay_reacts_to_target_drop();
    test_explicit_decay_modes_are_not_automatic();
    test_model_brake_uses_brake_coast_mode();
    test_absolute_guard_is_non_latching();
    test_single_filtered_spike_does_not_reach_guard();

    if (failures != 0)
    {
        printf("%d current-control test(s) failed\n", failures);
        return 1;
    }
    puts("current-control tests passed");
    return 0;
}
