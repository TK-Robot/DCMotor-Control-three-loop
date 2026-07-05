/**
 * @file PID.h
 * @brief Fixed-point PID control helpers.
 * @brief 定点 PID 控制辅助函数。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_PID_H
#define TRIPLE_CASCADECONTROLDCMOTOR_PID_H

#include <stdint.h>

#include "TypeDefine.h"

#define PID_SCALE        1000
#define PID_AbsDEADBAND 2
#define PID_IncDEADBAND 24

#define SPEED_LOW        8000
#define SPEED_MID        15000
#define FF_SCALE         1000
#define KV_Q             24
#define PWM_DEAD         110
#define PWM_MAX          750

static const int16_t speed_lut[] = {1250, 6000, 7500, 9000};
static const int16_t pwm_lut[] = {110, 142, 174, 206};
#define LUT_SIZE (sizeof(speed_lut)/sizeof(speed_lut[0]))

void PID_Init(Param *param);

/**
 * @brief Clear PID runtime history while keeping gains and limits.
 * @brief 清空 PID 运行历史状态，保留增益和限幅参数。
 */
void PID_Reset(PID_Int *pid);

int32_t PID_AbsCalculate(PID_Int* pid,int32_t setValue,int32_t CurrentValue);
int16_t PID_Vel_Calc(PID_Int* pid, int32_t target, int32_t feedback);

/**
 * @brief Position loop: convert position target to speed target.
 * @brief 位置环：将位置目标转换为速度目标。
 */
int32_t PID_PositionLoop(Param *param, int32_t target_position);

/**
 * @brief Speed loop: convert speed target to current target.
 * @brief 速度环：将速度目标转换为电流目标。
 */
int16_t PID_SpeedLoop(Param *param, int32_t target_speed);

/**
 * @brief Current loop: convert current target to PWM command.
 * @brief 电流环：将电流目标转换为 PWM 输出指令。
 */
int16_t PID_CurrentLoop(Param *param, int16_t target_current_mA);

int16_t PID_Pos(Param *param);
int16_t FeedForward_LUT(int32_t speed_abs);
int16_t Speed_FeedForward(int32_t speed);
int32_t SpeedPlan_Update(Param *param,int32_t SpeedCmd);

#endif // TRIPLE_CASCADECONTROLDCMOTOR_PID_H
