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
#define PID_POSITION_DEADBAND_DEFAULT_COUNTS 16U
#define PID_POSITION_DEADBAND_MAX_COUNTS     255U
#define PID_POSITION_UPDATE_PERIOD_MS 5L
#define POSITION_PID_KD_MAX 1000U
#define SPEED_PID_OUTPUT_MAX_MA 1800U
#define SPEED_LOW_CURRENT_PULSE_MA 400U
#define SPEED_PULSE_DENSITY_MAX_CPS 3000L
#define SPEED_CRAWL_COAST_OVERSPEED_CPS 750L

#define CURRENT_PID_KP_MAX 2000U
#define CURRENT_PID_KI_MAX 500U
#define CURRENT_PID_INTEGRAL_MAX 30000L
#define CURRENT_PID_VOLTAGE_MAX_MV 2000U

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
void PID_MigrateLegacyCurrentTuning(Param *param);

/**
 * @brief Clear PID runtime history while keeping gains and limits.
 * @brief 清空 PID 运行历史状态，保留增益和限幅参数。
 */
void PID_Reset(PID_Int *pid);

int32_t PID_AbsCalculate(PID_Int* pid,int32_t setValue,int32_t CurrentValue);
/**
 * @brief Position loop: convert position target to speed target.
 * @brief 位置环：将位置目标转换为速度目标。
 */
int32_t PID_PositionLoop(Param *param, int32_t target_position);

/**
 * @brief Speed loop: convert speed target to current target.
 * @brief 速度环：将速度目标转换为电流目标。
 */
int16_t PID_SpeedLoop(Param *param, int32_t target_speed,
                      uint16_t update_period_ms);

/**
 * @brief Speed loop torque boundary used by the model-assisted cascade.
 * @brief 模型辅助级联使用的速度环力矩边界。
 *
 * Existing velocity PID gains keep their current-equivalent mA scaling for
 * NVM/control-table compatibility. The returned value is converted through
 * the temperature-corrected torque constant and is therefore in uN*m.
 * 为兼容 NVM 和控制表，现有速度 PID 增益继续使用 mA 等效量缩放；返回值
 * 通过温度修正后的力矩常数换算，单位为 uN*m。
 */
int32_t PID_SpeedTorqueLoop(Param *param, int32_t target_speed,
                            uint16_t update_period_ms,
                            uint16_t current_limit_mA,
                            int32_t *reference_acceleration_cps2);

/**
 * @brief Combine R/Ke feed-forward with observable current PID voltage trim.
 * @brief 将 R/Ke 前馈与可观测电流 PID 电压修正组合为 PWM 指令。
 */
int16_t PID_CurrentLoop(Param *param, int16_t target_current_mA);

int16_t PID_Pos(Param *param);
int16_t FeedForward_LUT(int32_t speed_abs);
int16_t Speed_FeedForward(int32_t speed);
int32_t SpeedPlan_Update(Param *param, int32_t SpeedCmd,
                         uint16_t update_period_ms);

#endif // TRIPLE_CASCADECONTROLDCMOTOR_PID_H
