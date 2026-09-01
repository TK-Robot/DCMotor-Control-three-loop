/**
 * @file PWMCapture.h
 * @brief PWM input capture helper.
 * @brief PWM 输入捕获辅助模块。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_PWMCAPTURE_H
#define TRIPLE_CASCADECONTROLDCMOTOR_PWMCAPTURE_H

#include <stdbool.h>
#include <stdint.h>

#include "tim.h"

#define PWM_INPUT_VALID_MIN_US  900U  ///< Accepted active-high pulse lower bound. / 可接受高电平脉宽下限。
#define PWM_INPUT_VALID_MAX_US 2100U  ///< Accepted active-high pulse upper bound. / 可接受高电平脉宽上限。
#define PWM_INPUT_TIMEOUT_MS      15U ///< About five missing 330 Hz frames. / 约五帧 330 Hz 丢失后撤销使能。

/**
 * @brief Runtime state for one PWM input capture channel.
 * @brief 单路 PWM 输入捕获运行状态。
 */
typedef struct
{
    uint16_t CaptureOneUpTime;   ///< First rising-edge timestamp. / 第一次上升沿时间戳。
    uint16_t CaptureOneDownTime; ///< Falling-edge timestamp. / 下降沿时间戳。
    volatile uint16_t DutyRatio; ///< Captured high-level width in microseconds. / 捕获的高电平宽度，单位微秒。
    volatile uint16_t SignalAgeMs; ///< Time since the latest complete pulse. / 最近完整脉冲后的毫秒数。
    uint8_t EdgeNumber;          ///< Capture state machine step. / 捕获状态机步骤。
    TIM_TypeDef *timer;          ///< Timer input-capture instance. / 输入捕获定时器实例。
    volatile bool SignalValid;   ///< Pulse is in range and has not timed out. / 脉宽有效且尚未超时。
} CaptureData;

/**
 * @brief Start PWM input capture interrupt.
 * @brief 启动 PWM 输入捕获中断。
 */
void PWMCapture_Init(CaptureData *Data, TIM_TypeDef *timer);

/**
 * @brief Update capture state and return the latest measured value.
 * @brief 更新捕获状态并返回最新测量值。
 */
uint16_t PWMCapture_Calculate(CaptureData *Data, TIM_TypeDef *timer);

/**
 * @brief Age the captured signal by one millisecond and invalidate stale input.
 * @brief 递增一次 PWM 输入年龄，并使超时输入失效。
 */
void PWMCapture_1msTick(CaptureData *Data);

/**
 * @brief Configure capture polarity to rising edge.
 * @brief 配置为上升沿捕获。
 */
void SetCaptureToRising(CaptureData* Data);

/**
 * @brief Configure capture polarity to falling edge.
 * @brief 配置为下降沿捕获。
 */
void SetCaptureToFalling(CaptureData* Data);

#endif // TRIPLE_CASCADECONTROLDCMOTOR_PWMCAPTURE_H
