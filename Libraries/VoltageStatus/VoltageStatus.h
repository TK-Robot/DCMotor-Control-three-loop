/**
 * @file VoltageStatus.h
 * @brief ADC voltage, current, temperature, and power-loss status interface.
 * @brief ADC 电压、电流、温度和掉电状态检测接口。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_VOLTAGESTATUS_H
#define TRIPLE_CASCADECONTROLDCMOTOR_VOLTAGESTATUS_H

#include <stdbool.h>

#include "adc.h"
#include "AD116.h"
#include "CurrentControl.h"
#include "TypeDefine.h"
#include "Filter.h"

#define MaxVoltageBit 8191 ///< 2x oversampling sum full scale. / 2 倍过采样累加满量程。
#define VrefInt 1212       ///< Internal reference voltage in mV. / 内部参考电压，单位 mV。
#define VREF 3300          ///< Nominal MCU supply in mV. / MCU 标称供电，单位 mV。
#define SamplingMR 90      ///< Current sampling resistor parameter. / 电流采样电阻参数。
#define SamplingGainV 20   ///< Current amplifier gain. / 电流采样放大倍数。

/* External power divider: VCC -> 100k -> ADC pin -> 47k -> GND. */
/* 外部电源分压：VCC -> 100k -> ADC 引脚 -> 47k -> GND。 */
#define VCC_DIVIDER_HIGH_KOHM 100U
#define VCC_DIVIDER_LOW_KOHM 47U

#define Sampling 2 ///< Current conversion divisor. / 电流换算除数。

#define TS_CAL1   (*(uint16_t*)0x1FFF75A8)  ///< Temperature ADC calibration at 30 C. / 30 摄氏度温度校准值。
#define TS_CAL2   (*(uint16_t*)0x1FFF75CA)  ///< Temperature ADC calibration at 130 C. / 130 摄氏度温度校准值。
#define VREFINT_CAL (*(uint16_t*)0x1FFF75AA) ///< Factory VREFINT calibration. / 出厂 VREFINT 校准值。

/**
 * @brief Runtime handle for ADC-based status measurement.
 * @brief 基于 ADC 的状态检测运行句柄。
 */
typedef struct
{
    ADC_TypeDef *adc;        ///< ADC peripheral instance. / ADC 外设实例。
    Param* param;            ///< Shared runtime parameters. / 共享运行参数。
    LPF_Filter SampMaFilter; ///< Current measurement low-pass filter. / 电流测量低通滤波器。
    uint16_t current_offset_adc; ///< Zero-current ADC offset. / 零电流 ADC 偏置。
    uint8_t sample_drive_mode; ///< Bridge mode that produced the filtered sample. / 产生滤波样本的桥模式。
    int8_t sample_drive_sign; ///< Drive direction that produced the filtered sample. / 产生滤波样本的驱动方向。
    bool sample_valid;       ///< Previous active-window sample state. / 上次有效导通区采样状态。
    bool sample_filter_initialized; ///< Filter context survives brief PDM/coast gaps. / 滤波上下文跨越短暂 PDM/滑行间隔。
    /* Rolling statistics window (folded into Param every 20 ms). */
    /* 滚动统计窗口，每 20 ms 折叠进 Param 供诊断上传。 */
    int32_t window_sum_mA;
    uint16_t window_valid;
    uint16_t window_invalid;
    int16_t window_min_mA;
    int16_t window_max_mA;
    uint16_t window_tick_ms;
    bool last_hard_limit;
    volatile uint16_t adc_snapshot[ADC_STATUS_CONVERSION_COUNT];
    volatile uint32_t adc_snapshot_sequence;
} VoltageStatus;

/**
 * @brief Calibrate ADC and start DMA sampling.
 * @brief 校准 ADC 并启动 DMA 采样。
 */
void VoltageStatus_init(VoltageStatus *status, ADC_TypeDef *adc, Param *params);

/**
 * @brief Convert ADC sample to millivolts using VREFINT correction.
 * @brief 使用 VREFINT 校正把 ADC 采样值换算成毫伏。
 */
uint16_t ADC_to_mV(uint16_t Adc, uint16_t Vref);

/**
 * @brief Convert the divided VCC ADC sample to real input voltage.
 * @brief 将 VCC 分压后的 ADC 采样值换算成真实外部电源电压。
 */
uint16_t VoltageStatus_VccAdcToPower_mV(uint16_t adc, uint16_t Vref);

/**
 * @brief Check whether external power is below the configured save threshold.
 * @brief 检查外部电源是否低于配置的掉电保存阈值。
 */
bool VoltageStatus_IsPowerLow(const VoltageStatus* VoltageStatus);

/**
 * @brief Analyze DMA ADC samples and update Param measurements.
 * @brief 分析 DMA ADC 采样并更新 Param 中的测量值。
 */
void VoltageStatus_AnalyzeData(VoltageStatus* VoltageStatus);

void VoltageStatus_DmaIrqHandler(VoltageStatus *status,
                                 CurrentControl *current_control,
                                 AD116 *drive);

/**
 * @brief Derive logical signed current from measured magnitude and drive direction.
 * @brief 根据电流幅值、PWM 物理方向和编码器方向推导逻辑有符号电流。
 */
void VoltageStatus_UpdateLogicalCurrent(VoltageStatus* VoltageStatus);

/**
 * @brief Disable motor output when MCU temperature exceeds the limit.
 * @brief MCU 温度超过限制时关闭电机输出。
 */
void TempLimit(const VoltageStatus* VoltageStatus);

#endif // TRIPLE_CASCADECONTROLDCMOTOR_VOLTAGESTATUS_H
