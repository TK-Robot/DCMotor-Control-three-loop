/**
 * @file CurrentSenseModel.h
 * @brief Low-side shunt sampling model for the DRV8837 PWM bridge.
 * @brief DRV8837 PWM H 桥低侧分流采样模型。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_CURRENTSENSEMODEL_H
#define TRIPLE_CASCADECONTROLDCMOTOR_CURRENTSENSEMODEL_H

#include <stdbool.h>
#include <stdint.h>

#define CURRENT_SENSE_SHUNT_MILLIOHM 90U
#define CURRENT_SENSE_GAIN 20U
#define CURRENT_SENSE_VREFINT_MV 1212U
#define CURRENT_SENSE_HARD_LIMIT_MA 1500U

/* TIM3 and ADC both run at 32 MHz. With 2x oversampling and 1.5-cycle
 * acquisition, the current rank takes 28 ticks. Both sample-and-hold phases
 * finish after 16 ticks; the final SAR conversion may safely continue after
 * the bridge interval. Keep 2 us edge blanking and a 0.25 us trailing guard. */
#define CURRENT_SENSE_EDGE_BLANKING_TICKS 64U
#define CURRENT_SENSE_CURRENT_RANK_TICKS 28U
#define CURRENT_SENSE_CURRENT_ACQUISITION_TICKS 16U
#define CURRENT_SENSE_TRAILING_GUARD_TICKS 8U
#define CURRENT_SENSE_LONG_RANK_TICKS 346U
#define CURRENT_SENSE_ADC_SCAN_TICKS \
    (2U * CURRENT_SENSE_CURRENT_RANK_TICKS \
     + 2U * CURRENT_SENSE_LONG_RANK_TICKS)
#define CURRENT_SENSE_MIN_ACTIVE_TICKS \
    (CURRENT_SENSE_EDGE_BLANKING_TICKS \
     + CURRENT_SENSE_CURRENT_ACQUISITION_TICKS \
     + CURRENT_SENSE_TRAILING_GUARD_TICKS)
#define CURRENT_SENSE_TIMER_ARR 1249U
#define CURRENT_SENSE_MIN_VALID_DUTY_PERMILLE 71U

uint32_t CurrentSenseModel_DutyTicks(uint16_t timer_arr,
                                     int16_t drive_power_permille);

uint16_t CurrentSenseModel_TriggerCompare(uint16_t timer_arr,
                                          uint32_t duty_ticks,
                                          uint8_t drive_run_mode);

bool CurrentSenseModel_IsSampleValid(uint16_t timer_arr,
                                     int16_t drive_power_permille,
                                     uint8_t drive_run_mode);

bool CurrentSenseModel_CanEstimateFromAverageDuty(uint8_t drive_run_mode,
                                                   bool regenerative_braking);

int16_t CurrentSenseModel_AdcToMilliamp(uint16_t current_adc,
                                        uint16_t current_offset_adc,
                                        uint16_t vref_adc);

#endif /* TRIPLE_CASCADECONTROLDCMOTOR_CURRENTSENSEMODEL_H */
