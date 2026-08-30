/**
 * @file CurrentSenseModel.c
 * @brief Low-side shunt sampling model for the DRV8837 PWM bridge.
 * @brief DRV8837 PWM H 桥低侧分流采样模型。
 */

#include "CurrentSenseModel.h"

#include <limits.h>

static uint16_t CurrentSenseModel_AbsPower(int16_t drive_power_permille)
{
    int32_t magnitude = drive_power_permille;

    if (magnitude < 0) magnitude = -magnitude;
    if (magnitude > 1000) magnitude = 1000;
    return (uint16_t)magnitude;
}

uint32_t CurrentSenseModel_DutyTicks(uint16_t timer_arr,
                                     int16_t drive_power_permille)
{
    uint32_t period_ticks = (uint32_t)timer_arr + 1U;
    uint32_t duty_ticks = period_ticks
                        * CurrentSenseModel_AbsPower(drive_power_permille)
                        / 1000U;

    if (duty_ticks > period_ticks) duty_ticks = period_ticks;
    return duty_ticks;
}

uint16_t CurrentSenseModel_TriggerCompare(uint16_t timer_arr,
                                          int16_t drive_power_permille,
                                          uint8_t drive_run_mode)
{
    uint32_t period_ticks = (uint32_t)timer_arr + 1U;
    uint32_t compare = period_ticks / 2U;

    (void)drive_power_permille;
    (void)drive_run_mode;

    /* The 16x hardware oversampling window spans roughly 26 us. Keeping its
     * trigger fixed at the PWM center averages the narrow bridge pulse instead
     * of turning switching-edge timing into a false instantaneous current. */

    /* PWM2 must have an in-period edge or TIM3 will stop triggering the ADC. */
    if (compare == 0U) compare = 1U;
    if (compare >= period_ticks) compare = period_ticks - 1U;
    return (uint16_t)compare;
}

bool CurrentSenseModel_IsSampleValid(int16_t drive_power_permille,
                                     uint8_t drive_run_mode)
{
    return (drive_run_mode == 2U || drive_run_mode == 3U)
        && CurrentSenseModel_AbsPower(drive_power_permille)
           >= CURRENT_SENSE_MIN_VALID_DUTY_PERMILLE;
}

bool CurrentSenseModel_CanEstimateFromAverageDuty(uint8_t drive_run_mode)
{
    /* Drive/brake PWM has a defined average terminal voltage of duty * Vbus.
     * Drive/coast PWM depends on winding inductance and current extinction, so
     * the same steady-state R/Ke equation is not a truthful fallback. */
    return drive_run_mode == 2U;
}

int16_t CurrentSenseModel_AdcToMilliamp(uint16_t current_adc,
                                        uint16_t current_offset_adc,
                                        uint16_t vref_adc)
{
    uint32_t delta_adc;
    uint32_t numerator;
    uint32_t denominator;
    uint32_t current_mA;

    if (vref_adc == 0U || current_adc <= current_offset_adc)
    {
        return 0;
    }

    delta_adc = (uint32_t)current_adc - current_offset_adc;
    /* 90 mOhm x gain 20 permits reducing 1000/1800 to 5/9. This
     * equivalent 32-bit form avoids a software 64-bit divide in the ISR. */
    numerator = CURRENT_SENSE_VREFINT_MV * delta_adc * 5U;
    denominator = (uint32_t)vref_adc * 9U;
    current_mA = (numerator + denominator / 2U) / denominator;
    if (current_mA > INT16_MAX) current_mA = INT16_MAX;
    return (int16_t)current_mA;
}
