/**
 * @file CurrentControl.h
 * @brief Model-assisted PWM actuator with synchronous peak-current protection.
 * @brief 模型辅助 PWM 执行器与同步峰值电流保护。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_CURRENTCONTROL_H
#define TRIPLE_CASCADECONTROLDCMOTOR_CURRENTCONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define CURRENT_CONTROL_REVERSAL_COAST_CYCLES 32U
#define CURRENT_CONTROL_AUTO_FAST_HOLD_CYCLES 64U
#define CURRENT_CONTROL_AUTO_TARGET_DROP_MA 30L

typedef struct
{
    uint16_t absolute_limit_mA;
} CurrentControlElectrical;

typedef struct
{
    int16_t target_current_mA;
    int16_t feedforward_pwm;
    int16_t output_power;
    uint8_t configured_pwm_mode;
    uint8_t output_mode;
    uint8_t reversal_coast_cycles;
    uint8_t auto_fast_hold_cycles;
    int8_t last_voltage_sign;
    int8_t last_target_sign;
    uint16_t last_target_magnitude_mA;
    uint16_t peak_chop_events;
    CurrentControlElectrical electrical;
    bool enabled;
} CurrentControl;

typedef struct
{
    int16_t power_permille;
    uint8_t drive_mode;
    bool hard_limit_active;
} CurrentControlOutput;

void CurrentControl_Init(CurrentControl *control);

/**
 * @brief Publish the slow-loop target and electrical-model voltage command.
 * @brief 发布慢速外环目标和电气模型电压指令。
 */
void CurrentControl_SetCommand(CurrentControl *control,
                               bool enabled,
                               int16_t physical_target_current_mA,
                               int16_t physical_feedforward_pwm,
                               uint8_t configured_pwm_mode,
                               const CurrentControlElectrical *electrical);

/**
 * @brief Apply the model voltage command and supervise a qualified current sample.
 * @brief 执行模型电压指令并监控合格的电流样本。
 */
CurrentControlOutput CurrentControl_Step(CurrentControl *control,
                                         int16_t measured_current_mA,
                                         bool sample_valid);

#endif /* TRIPLE_CASCADECONTROLDCMOTOR_CURRENTCONTROL_H */
