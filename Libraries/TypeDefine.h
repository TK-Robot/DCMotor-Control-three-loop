//
// Created by Administrator on 2025/12/13.
//

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_TYPEDEFINE_H
#define TRIPLE_CASCADECONTROLDCMOTOR_TYPEDEFINE_H

#include <stdbool.h>
#include "stdint.h"

#define MAP(x, in_min, in_max, out_min, out_max) \
(((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

typedef struct
{
    // PID参数
    uint16_t Kp;           // 比例系数
    uint16_t Ki;           // 积分系数
    uint16_t Kd;           // 微分系数

    // 积分项相关
    int32_t integral;     // 累计误差
    int32_t integral_max; // 累计误差最大值

    // 微分项相关
    int32_t prev_error;   // 上一次误差
    int32_t prev_prev_error;   // 上上次误差

    int32_t prev_feedback;

    // 输出限幅
    uint16_t out_max;      // 最大输出
    uint16_t out_min;      // 最小输出（绝对值）

    // 上次控制量
    int16_t prev_out;


} PID_Int;

typedef struct
{
    //UART参数
    uint8_t RxBuf[32];//UART接收缓存区
    bool ReturnEn;//UART接收返回标志

    //PWM输入捕获
    uint16_t DutyRatio;

    //ADC参数
    uint16_t VoltageBuf[5];//电压检测缓存区
    uint16_t INA181_mV;//INA181检测电压
    int16_t INA181_mA;//INA181检测电流
    int16_t ExpectMA;//预期电流值
    uint16_t INA181REF_mV;
    uint16_t VCC_mV;
    uint16_t Temp_mV;
    int8_t Temp;//温度
    int8_t TempLimit ;//温度限制

    //编码器
    volatile uint8_t EncoderReadData[2];//14位编码器缓存区
    uint16_t EncoderValue;//14位编码值
    uint16_t EncoderOffset;//编码器偏移值
    uint16_t LastEncoderValue;//上次编码值
    int16_t EncoderExpect;//预期编码器位置
    int32_t EncoderSpeed;//编码器速度
    int32_t EncoderSpeedExpect;//预期编码器速度
    int32_t LastEncoderSpeed;//上次编码器速度
    int16_t EncoderMultiTurn;//多圈计数
    int32_t EncoderMultiTurnValue;//包含多圈绝对值
    int32_t LastEncoderMultiTurnValue;//上次编码值
    uint16_t SpeedMax;//速度最大设定值
    int32_t AccDec;//加减速度
    uint16_t AccelMax;//加速度
    uint16_t DecelMax;//减速度
    int32_t SpeedRef;//规划速度
    int32_t target_speed;
    bool EncoderVeer;

    //驱动参数
    bool DriveVeerFlag;//旋转标志位
    uint8_t DriveRunMode; // 0：滑行 1：刹车 2：慢衰减 正反转&刹车 3：快衰减 正反转&滑行
    int16_t DrivePower; //0至1000

    //PID
    PID_Int Pid_Pos;//位置环
    PID_Int Pid_PosVel;//位置速度环
    PID_Int Pid_PosEle;//电流环
    PID_Int Pid_Vel;//速度环
    PID_Int Pid_Ele;//电流环
    uint8_t CycleTimeMs;//循环周期
    uint16_t ProcessTimeUs;//程序运行时间



} Param;



#endif //TRIPLE_CASCADECONTROLDCMOTOR_TYPEDEFINE_H