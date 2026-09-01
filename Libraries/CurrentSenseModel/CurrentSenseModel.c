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

    return duty_ticks;
}

uint16_t CurrentSenseModel_TriggerCompare(uint16_t timer_arr,
                                          uint32_t duty_ticks,
                                          uint8_t drive_run_mode)
{
    uint32_t period_ticks = (uint32_t)timer_arr + 1U;
    uint32_t compare = period_ticks / 2U;

    if (duty_ticks >= CURRENT_SENSE_MIN_ACTIVE_TICKS &&
        (drive_run_mode == 2U || drive_run_mode == 3U))
    {
        /* Sample near the end of the active interval so protection observes
         * the pulse peak. At minimum duty this is also exactly one blanking
         * interval after the leading bridge edge. */
        compare = drive_run_mode == 2U
            ? period_ticks - CURRENT_SENSE_CURRENT_ACQUISITION_TICKS
                           - CURRENT_SENSE_TRAILING_GUARD_TICKS
            : duty_ticks - CURRENT_SENSE_CURRENT_ACQUISITION_TICKS
                         - CURRENT_SENSE_TRAILING_GUARD_TICKS;
    }

    /* The proven TIM3 configuration (ARR=1249) keeps every result strictly
     * inside the PWM period; IsSampleValid rejects any other ARR. */
    return (uint16_t)compare;
}

bool CurrentSenseModel_IsSampleValid(uint16_t timer_arr,
                                     int16_t drive_power_permille,
                                     uint8_t drive_run_mode)
{
    return timer_arr == CURRENT_SENSE_TIMER_ARR
        && (drive_run_mode == 2U || drive_run_mode == 3U)
        && CurrentSenseModel_AbsPower(drive_power_permille)
           >= CURRENT_SENSE_MIN_VALID_DUTY_PERMILLE;
}

bool CurrentSenseModel_CanEstimateFromAverageDuty(uint8_t drive_run_mode,
                                                   bool regenerative_braking)
{
    /* Drive/brake PWM has a defined average terminal voltage of duty * Vbus.
     * Drive/coast PWM depends on winding inductance and current extinction, so
     * the same steady-state R/Ke equation is not a truthful fallback.  The
     * supervisory loop can request regenerative brake/coast one main-loop tick
     * before the PWM ISR publishes that bridge mode; reject the stale mode too. */
    return drive_run_mode == 2U && !regenerative_braking;
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
