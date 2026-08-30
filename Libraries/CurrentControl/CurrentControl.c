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
    control->peak_coast_cycles = 0U;
    control->average_current_mA = 0;
    control->hard_limit_active = false;
    control->peak_limit_active = false;
}

void CurrentControl_Init(CurrentControl *control)
{
    if (control == NULL) return;

    control->target_current_mA = 0;
    control->feedforward_pwm = 0;
    control->last_measured_mA = 0;
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

static void CurrentControl_UpdateAverage(CurrentControl *control,
                                         int16_t cycle_average_mA)
{
    int32_t difference = (int32_t)cycle_average_mA
                       - control->average_current_mA;
    control->average_current_mA += (int16_t)(
        difference * CURRENT_CONTROL_AVERAGE_FILTER_ALPHA / 256L);
}

static uint8_t CurrentControl_SelectAutoDecay(CurrentControl *control,
                                               int32_t target_magnitude,
                                               int32_t measured_magnitude,
                                               int32_t duty_magnitude,
                                               bool sample_valid)
{
    bool sample_can_control_decay =
        sample_valid &&
        duty_magnitude >= CURRENT_SENSE_CONTROL_MIN_DUTY_PERMILLE;

    if (control->auto_fast_hold_cycles > 0U)
    {
        control->auto_fast_hold_cycles--;
        return 3U;
    }

    if (control->output_mode == 3U)
    {
        if (!sample_can_control_decay ||
            measured_magnitude <=
                target_magnitude + CURRENT_CONTROL_AUTO_SLOW_RETURN_ERROR_MA)
        {
            return 2U;
        }
        return 3U;
    }

    if (sample_can_control_decay &&
        measured_magnitude >
            target_magnitude + CURRENT_CONTROL_AUTO_FAST_ENTER_ERROR_MA)
    {
        control->auto_fast_hold_cycles =
            CURRENT_CONTROL_AUTO_FAST_HOLD_CYCLES;
        return 3U;
    }

    return 2U;
}

CurrentControlOutput CurrentControl_Step(CurrentControl *control,
                                         int16_t measured_current_mA,
                                         bool sample_valid)
{
    CurrentControlOutput output = {0, 0, 0U, false, false};
    int32_t measured_magnitude;
    int16_t requested_pwm;
    int16_t unconstrained_pwm;
    int8_t voltage_sign;
    uint8_t decay_mode;

    if (control == NULL || !control->enabled ||
        control->target_current_mA == 0)
    {
        if (control != NULL) CurrentControl_ResetRuntime(control);
        return output;
    }

    unconstrained_pwm = CurrentControl_ClampPwm(control->feedforward_pwm);
    requested_pwm = unconstrained_pwm;
    voltage_sign = CurrentControl_Sign(requested_pwm);
    measured_magnitude = CurrentControl_AbsI16(measured_current_mA);

    if (control->last_voltage_sign != 0 && voltage_sign != 0 &&
        voltage_sign != control->last_voltage_sign)
    {
        control->reversal_coast_cycles = CURRENT_CONTROL_REVERSAL_COAST_CYCLES;
        control->peak_coast_cycles = 0U;
    }
    if (voltage_sign != 0) control->last_voltage_sign = voltage_sign;

    if (sample_valid)
    {
        control->last_measured_mA = (int16_t)measured_magnitude;
        /* PA0 is already a 16-conversion hardware time average. Do not apply
         * duty or an uncalibrated R/L model a second time. */
        CurrentControl_UpdateAverage(control, (int16_t)measured_magnitude);
        if (control->electrical.absolute_limit_mA != 0U &&
            measured_magnitude >= control->electrical.absolute_limit_mA)
        {
            control->hard_limit_active = true;
        }
        else if (control->reversal_coast_cycles == 0U &&
                 control->auto_fast_hold_cycles == 0U &&
                 control->peak_coast_cycles == 0U &&
                 control->electrical.peak_limit_mA != 0U &&
                 measured_magnitude >= control->electrical.peak_limit_mA)
        {
            control->peak_coast_cycles = CURRENT_CONTROL_PEAK_COAST_CYCLES;
            if (control->peak_chop_events != UINT16_MAX)
                control->peak_chop_events++;
        }
    }
    else if (control->output_power == 0)
    {
        CurrentControl_UpdateAverage(control, 0);
    }

    if (control->hard_limit_active)
    {
        control->output_power = 0;
        control->output_mode = 0U;
        output.drive_mode = 0U;
        output.hard_limit_active = true;
        output.average_current_mA = control->average_current_mA;
        return output;
    }

    control->peak_limit_active = requested_pwm != unconstrained_pwm;
    /* Target-transient fast hold and reversal coast keep the soft peak chopper
     * disarmed until the new bridge context has settled. Samples remain
     * available for diagnostics and absolute protection throughout. */
    if (control->reversal_coast_cycles > 0U)
    {
        control->reversal_coast_cycles--;
        control->output_power = 0;
        control->output_mode = 0U;
        output.drive_mode = 0U;
        output.hard_limit_active = control->hard_limit_active;
        output.average_current_mA = control->average_current_mA;
        return output;
    }

    if (control->peak_coast_cycles > 0U)
    {
        control->peak_coast_cycles--;
        control->output_power = 0;
        control->output_mode = 0U;
        control->peak_limit_active = true;
        output.average_current_mA = control->average_current_mA;
        output.peak_limit_active = true;
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
        decay_mode = CurrentControl_SelectAutoDecay(
            control,
            CurrentControl_AbsI16(control->target_current_mA),
            measured_magnitude,
            CurrentControl_AbsI16(control->output_power),
            sample_valid);
    }
    else decay_mode = 2U;

    /* The common low-side shunt cannot observe winding current throughout
     * recirculation, so it must never close an average-current PI loop. Auto
     * decay only uses qualified active-window samples at useful duty, with
     * hysteresis and a bounded fast-decay hold after target transients. */
    control->output_power = requested_pwm;
    if (control->output_power == 0) decay_mode = 0U;
    control->output_mode = decay_mode;

    output.power_permille = control->output_power;
    output.average_current_mA = control->average_current_mA;
    output.drive_mode = decay_mode;
    output.hard_limit_active = false;
    output.peak_limit_active = control->peak_limit_active;
    return output;
}
