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

#define PowerMin 1 ///< Timer resolution is the only low-duty clamp. / 仅按定时器分辨率限制低占空比。

/**
 * @brief Runtime handle for two-channel PWM motor drive.
 * @brief 双通道 PWM 电机驱动运行句柄。
 */
typedef struct
{
    TIM_TypeDef *timer;      ///< PWM timer instance. / PWM 定时器实例。
    Param *param;            ///< Shared runtime parameters. / 共享运行参数。
    uint32_t channel1;       ///< PWM channel 1. / PWM 通道 1。
    uint32_t channel2;       ///< PWM channel 2. / PWM 通道 2。
} AD116;

/**
 * @brief Initialize PWM outputs for the motor driver.
 * @brief 初始化电机驱动 PWM 输出。
 */
void AD116_init(AD116 *ad116, TIM_TypeDef *timer, uint32_t channel1,
                uint32_t channel2, Param *params);

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
 * @brief Apply one model-voltage PWM actuator output.
 * @brief 应用一次模型电压 PWM 执行器输出。
 */
void AD116_ApplyPwm(AD116 *ad116, int16_t power_permille,
                    uint8_t drive_mode);

/**
 * @brief Start loop timing measurement.
 * @brief 开始控制循环计时。
 */
void CycleStart(AD116 *ad116, TIM_TypeDef *timer);

/**
 * @brief Block until the configured control period has elapsed.
 * @brief 阻塞等待到配置的控制周期结束。
 */
void CycleBlockingTimer(AD116 *ad116, TIM_TypeDef *timer);

#endif // TRIPLE_CASCADECONTROLDCMOTOR_AD116_H
