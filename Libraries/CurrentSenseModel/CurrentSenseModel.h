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
#define CURRENT_SENSE_MIN_VALID_DUTY_PERMILLE 1U
#define CURRENT_SENSE_CONTROL_MIN_DUTY_PERMILLE 1U
#define CURRENT_SENSE_HARD_LIMIT_MA 1500U

uint32_t CurrentSenseModel_DutyTicks(uint16_t timer_arr,
                                     int16_t drive_power_permille);

uint16_t CurrentSenseModel_TriggerCompare(uint16_t timer_arr,
                                          int16_t drive_power_permille,
                                          uint8_t drive_run_mode);

bool CurrentSenseModel_IsSampleValid(int16_t drive_power_permille,
                                     uint8_t drive_run_mode);

bool CurrentSenseModel_CanEstimateFromAverageDuty(uint8_t drive_run_mode);

int16_t CurrentSenseModel_AdcToMilliamp(uint16_t current_adc,
                                        uint16_t current_offset_adc,
                                        uint16_t vref_adc);

#endif /* TRIPLE_CASCADECONTROLDCMOTOR_CURRENTSENSEMODEL_H */
