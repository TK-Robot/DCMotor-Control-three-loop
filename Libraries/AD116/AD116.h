/**
 * @file AD116.h
 * @brief AD116 H-bridge motor driver control interface.
 * @brief AD116 H 桥电机驱动控制接口。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_AD116_H
#define TRIPLE_CASCADECONTROLDCMOTOR_AD116_H

#include <stdbool.h>

#include "tim.h"
#include "TypeDefine.h"

#define PowerMin 10 ///< Minimum command before output is forced to zero. / 小于该输出时强制归零。

/**
 * @brief Runtime handle for two-channel PWM motor drive.
 * @brief 双通道 PWM 电机驱动运行句柄。
 */
typedef struct
{
    TIM_HandleTypeDef* htim; ///< PWM timer handle. / PWM 定时器句柄。
    Param *param;            ///< Shared runtime parameters. / 共享运行参数。
    uint32_t channel1;       ///< PWM channel 1. / PWM 通道 1。
    uint32_t channel2;       ///< PWM channel 2. / PWM 通道 2。
} AD116;

/**
 * @brief Initialize PWM outputs for the motor driver.
 * @brief 初始化电机驱动 PWM 输出。
 */
void AD116_init(AD116* ad116,TIM_HandleTypeDef* htim, uint32_t Channel1, uint32_t Channel2,Param *params);

/**
 * @brief Update timer prescaler and auto-reload value.
 * @brief 更新定时器预分频和自动重装值。
 */
void AD116_setTimerFrequency(const AD116* ad116, uint32_t psc, uint32_t arr);

/**
 * @brief Apply DriveRunMode and DrivePower to PWM outputs.
 * @brief 根据 DriveRunMode 和 DrivePower 更新 PWM 输出。
 */
void AD116_Update(AD116* ad116, Param* param);

/**
 * @brief Start loop timing measurement.
 * @brief 开始控制循环计时。
 */
void CycleStart(AD116* ad116,TIM_HandleTypeDef* htim);

/**
 * @brief Block until the configured control period has elapsed.
 * @brief 阻塞等待到配置的控制周期结束。
 */
void CycleBlockingTimer(AD116* ad116,TIM_HandleTypeDef* htim);

#endif // TRIPLE_CASCADECONTROLDCMOTOR_AD116_H
