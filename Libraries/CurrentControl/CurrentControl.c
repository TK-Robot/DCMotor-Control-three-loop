/**
 * @file CurrentControl.c
 * @brief Model-assisted PWM actuator with synchronous peak-current protection.
 * @brief 模型辅助 PWM 执行器与同步峰值电流保护实现。
 */

#include "CurrentControl.h"
#include "CurrentSenseModel.h"

#include <stddef.h>

static int32_t CurrentControl_AbsI16(int16_t value)
{
    return value < 0 ? -(int32_t)value : (int32_t)value;
}

static int8_t CurrentControl_Sign(int16_t value)
{
    if (value > 0) return 1;
    if (value < 0) return -1;
    return 0;
}

static int16_t CurrentControl_ClampPwm(int16_t value)
{
    if (value < -1000) return -1000;
    if (value > 1000) return 1000;
    return value;
}

static void CurrentControl_ResetRuntime(CurrentControl *control)
{
    control->output_power = 0;
    control->output_mode = 0U;
    control->reversal_coast_cycles = 0U;
    control->auto_fast_hold_cycles = 0U;
    control->last_voltage_sign = 0;
    control->last_target_sign = 0;
    control->last_target_magnitude_mA = 0U;
}

void CurrentControl_Init(CurrentControl *control)
{
    if (control == NULL) return;

    control->target_current_mA = 0;
    control->feedforward_pwm = 0;
    control->configured_pwm_mode = 4U;
    control->electrical = (CurrentControlElectrical){0};
    control->peak_chop_events = 0U;
    control->enabled = false;
    CurrentControl_ResetRuntime(control);
}

void CurrentControl_SetCommand(CurrentControl *control,
                               bool enabled,
                               int16_t physical_target_current_mA,
                               int16_t physical_feedforward_pwm,
                               uint8_t configured_pwm_mode,
                               const CurrentControlElectrical *electrical)
{
    int32_t target_magnitude;
    int8_t target_sign;

    if (control == NULL) return;

    if (!enabled || physical_target_current_mA == 0)
    {
        control->enabled = false;
        control->target_current_mA = 0;
        control->feedforward_pwm = 0;
        CurrentControl_ResetRuntime(control);
        return;
    }

    target_magnitude = CurrentControl_AbsI16(physical_target_current_mA);
    target_sign = CurrentControl_Sign(physical_target_current_mA);

    if (configured_pwm_mode == 4U && control->enabled)
    {
        if ((control->last_target_sign != 0 &&
             target_sign != control->last_target_sign) ||
            ((int32_t)control->last_target_magnitude_mA >=
             target_magnitude + CURRENT_CONTROL_AUTO_TARGET_DROP_MA))
        {
            control->auto_fast_hold_cycles =
                CURRENT_CONTROL_AUTO_FAST_HOLD_CYCLES;
        }
    }
    else if (configured_pwm_mode != 4U)
    {
        control->auto_fast_hold_cycles = 0U;
    }

    control->feedforward_pwm = physical_feedforward_pwm;
    control->target_current_mA = physical_target_current_mA;
    control->configured_pwm_mode = configured_pwm_mode;
    if (electrical != NULL) control->electrical = *electrical;
    control->last_target_sign = target_sign;
    control->last_target_magnitude_mA = (uint16_t)target_magnitude;
    control->enabled = true;
}

CurrentControlOutput CurrentControl_Step(CurrentControl *control,
                                         int16_t measured_current_mA,
                                         bool sample_valid)
{
    CurrentControlOutput output = {0, 0U, false};
    int32_t measured_magnitude;
    int16_t requested_pwm;
    int8_t voltage_sign;
    uint8_t decay_mode;

    if (control == NULL || !control->enabled ||
        control->target_current_mA == 0)
    {
        if (control != NULL) CurrentControl_ResetRuntime(control);
        return output;
    }

    requested_pwm = CurrentControl_ClampPwm(control->feedforward_pwm);
    voltage_sign = CurrentControl_Sign(requested_pwm);
    measured_magnitude = CurrentControl_AbsI16(measured_current_mA);

    if (control->last_voltage_sign != 0 && voltage_sign != 0 &&
        voltage_sign != control->last_voltage_sign)
    {
        control->reversal_coast_cycles = CURRENT_CONTROL_REVERSAL_COAST_CYCLES;
    }
    if (voltage_sign != 0) control->last_voltage_sign = voltage_sign;

    if (sample_valid)
    {
        /* This is an active-window peak sample, not cycle-average winding
         * current. Use it only for peak and absolute protection. */
        if (control->electrical.absolute_limit_mA != 0U &&
            measured_magnitude >= control->electrical.absolute_limit_mA)
        {
            /* The INA181 is at the edge of its measurable range here. Coast
             * this PWM cycle immediately, but let the 1 ms supervisor decide
             * whether the overload persisted long enough to latch a fault. */
            if (control->peak_chop_events != UINT16_MAX)
                control->peak_chop_events++;
            output.hard_limit_active = true;
            return output;
        }
    }

    /* Target-transient fast hold and reversal coast let the new bridge context
     * settle. The absolute rail guard remains active throughout. */
    if (control->reversal_coast_cycles > 0U)
    {
        control->reversal_coast_cycles--;
        return output;
    }

    if (control->configured_pwm_mode == 1U)
    {
        decay_mode = 1U;
    }
    else if (control->configured_pwm_mode == 3U)
    {
        decay_mode = 3U;
    }
    else if (control->configured_pwm_mode == 2U)
    {
        decay_mode = 2U;
    }
    else if (control->configured_pwm_mode == 4U)
    {
        if (control->auto_fast_hold_cycles > 0U)
        {
            control->auto_fast_hold_cycles--;
            decay_mode = 3U;
        }
        else decay_mode = 2U;
    }
    else decay_mode = 2U;

    /* The common low-side shunt cannot observe winding current throughout
     * recirculation, so its peak sample must never close an average-current PI
     * loop. Automatic decay uses only a bounded fast hold after target drops
     * or reversals, then returns to efficient slow decay. */
    control->output_power = requested_pwm;
    if (control->output_power == 0) decay_mode = 0U;
    control->output_mode = decay_mode;

    output.power_permille = control->output_power;
    output.drive_mode = decay_mode;
    return output;
}
