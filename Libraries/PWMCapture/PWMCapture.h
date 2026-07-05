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

/**
 * @brief Runtime state for one PWM input capture channel.
 * @brief 单路 PWM 输入捕获运行状态。
 */
typedef struct
{
    uint16_t CaptureOneUpTime;   ///< First rising-edge timestamp. / 第一次上升沿时间戳。
    uint16_t CaptureOneDownTime; ///< Falling-edge timestamp. / 下降沿时间戳。
    uint16_t CaptureTwoUpTime;   ///< Second rising-edge timestamp. / 第二次上升沿时间戳。
    uint16_t DutyRatio;          ///< Calculated low-width value. / 计算得到的低电平宽度值。
    uint8_t EdgeNumber;          ///< Capture state machine step. / 捕获状态机步骤。
    TIM_HandleTypeDef *htim;     ///< Timer input-capture handle. / 输入捕获定时器句柄。
    bool CaptureState;           ///< Reserved capture state flag. / 预留捕获状态标志。
} CaptureData;

/**
 * @brief Start PWM input capture interrupt.
 * @brief 启动 PWM 输入捕获中断。
 */
void PWMCapture_Init(CaptureData* Data,TIM_HandleTypeDef *htim);

/**
 * @brief Update capture state and return the latest measured value.
 * @brief 更新捕获状态并返回最新测量值。
 */
uint16_t PWMCapture_Calculate(CaptureData* Data,TIM_HandleTypeDef *htim);

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
