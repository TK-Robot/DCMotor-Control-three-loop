/**
 * @file ServoControl.h
 * @brief Three-mode cascaded servo control scheduler.
 * @brief 三模式级联伺服控制调度器。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_SERVOCONTROL_H
#define TRIPLE_CASCADECONTROLDCMOTOR_SERVOCONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "TypeDefine.h"

#define SERVO_SPEED_PERIOD_MS        5U   ///< Speed loop period. / 速度环周期。
#define SERVO_POSITION_PERIOD_MS     10U  ///< Position loop period. / 位置环周期。
#define SERVO_TELEMETRY_PERIOD_MS    20U  ///< Telemetry period. / 遥测周期。
#define SERVO_POWER_RECOVER_HYST_MV  500U ///< Power recovery hysteresis. / 电源恢复回差。

/**
 * @brief Runtime context for multi-rate servo control.
 * @brief 多速率伺服控制运行上下文。
 */
typedef struct
{
    Param *param;              ///< Shared runtime parameters. / 共享运行参数。
    ServoCommand command;      ///< Active servo command. / 当前伺服指令。
    ServoMode last_mode;       ///< Previous mode for transition handling. / 上一次模式，用于切换处理。
    uint8_t speed_count;       ///< Speed-loop divider counter. / 速度环分频计数。
    uint8_t position_count;    ///< Position-loop divider counter. / 位置环分频计数。
    uint8_t telemetry_count;   ///< Telemetry divider counter. / 遥测分频计数。
    bool speed_due;            ///< Speed loop should run this tick. / 本周期需要运行速度环。
    bool position_due;         ///< Position loop should run this tick. / 本周期需要运行位置环。
    bool telemetry_due;        ///< Telemetry should run this tick. / 本周期需要发送遥测。
    bool save_request;         ///< One-shot NVM save request. / 单次掉电保存请求。
    bool power_low_latched;    ///< Latched low-power protection state. / 低压保护锁存状态。
} ServoControl;

/**
 * @brief Initialize the servo scheduler and keep output disabled by default.
 * @brief 初始化伺服调度器，默认保持输出关闭。
 */
void ServoControl_Init(ServoControl *servo, Param *param);

/**
 * @brief Update the active runtime command.
 * @brief 更新当前运行指令。
 */
void ServoControl_SetCommand(ServoControl *servo, const ServoCommand *command);

/**
 * @brief Start a 1 ms scheduler tick and update loop divider flags.
 * @brief 开始 1 ms 调度周期并更新各环分频标志。
 */
void ServoControl_Begin1ms(ServoControl *servo);

/**
 * @brief Run protection checks and cascaded control for this 1 ms tick.
 * @brief 执行本 1 ms 周期的保护判断和级联控制。
 */
void ServoControl_Run1ms(ServoControl *servo);

/**
 * @brief Query whether encoder/speed feedback should be refreshed.
 * @brief 查询是否需要刷新编码器和速度反馈。
 */
bool ServoControl_IsSpeedDue(const ServoControl *servo);

/**
 * @brief Query whether telemetry should be sent.
 * @brief 查询是否需要发送遥测。
 */
bool ServoControl_IsTelemetryDue(const ServoControl *servo);

/**
 * @brief Consume and clear the one-shot NVM save request.
 * @brief 读取并清除单次掉电保存请求。
 */
bool ServoControl_ConsumeSaveRequest(ServoControl *servo);

/**
 * @brief Clear PID history and speed planner state.
 * @brief 清空 PID 历史状态和速度规划状态。
 */
void ServoControl_ResetLoops(ServoControl *servo);

#endif // TRIPLE_CASCADECONTROLDCMOTOR_SERVOCONTROL_H
