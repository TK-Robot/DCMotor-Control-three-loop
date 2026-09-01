/**
 * @file TypeDefine.h
 * @brief Project-wide data types and parameter definitions.
 * @brief 项目通用数据类型与参数定义。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_TYPEDEFINE_H
#define TRIPLE_CASCADECONTROLDCMOTOR_TYPEDEFINE_H

#include <stdbool.h>
#include <stdint.h>

#include "MechanicalModel/MechanicalModel.h"
#include "MotorTorqueModel/MotorTorqueModel.h"

#define LOW_SPEED_COMP_BIN_COUNT             32U
#define LOW_SPEED_COMP_MAX_CORRECTION_MA      100
#define LOW_SPEED_COMP_DEFAULT_MAX_SPEED_CPS 3000U

#define MAP(x, in_min, in_max, out_min, out_max) \
(((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

/**
 * @brief Servo outer-loop control mode.
 * @brief 伺服外环控制模式。
 */
typedef enum
{
    SERVO_MODE_CURRENT = 0, ///< Hybrid model/observable-current command. / 模型与可观测电流混合指令。
    SERVO_MODE_SPEED,       ///< Velocity PI plus model voltage actuation. / 速度 PI 加模型电压执行。
    SERVO_MODE_POSITION,    ///< Position P plus velocity PI. / 位置 P 加速度 PI。
    SERVO_MODE_TORQUE,      ///< Estimated torque-to-current model. / 估算力矩到电流模型。
    SERVO_MODE_PWM_DUTY     ///< Diagnostic direct duty in target_current_mA, -1000..1000 permille. / 诊断直通占空比，复用目标电流字段，范围 -1000..1000 千分比。
} ServoMode;

/**
 * @brief Runtime servo command written by the future command protocol.
 * @brief 后续命令协议写入的运行期伺服指令。
 */
typedef struct
{
    ServoMode mode;           ///< Active servo mode. / 当前伺服模式。
    bool enable;              ///< Output enable command. / 输出使能指令。
    bool position_multi_turn; ///< Position target is accumulated when true; otherwise it stays inside the current turn. / true 为多圈累计目标，否则限制在当前单圈内且不跨零。
    int16_t target_current_mA; ///< Current target in mA, or diagnostic PWM permille in mode 4. / 电流目标（mA）；模式4时为诊断 PWM 千分比。
    int32_t target_torque_uNm; ///< Estimated shaft load torque target. / 轴端负载力矩目标，单位 uN·m。
    int32_t target_speed;     ///< Speed target. / 速度目标。
    int32_t target_position;  ///< Position target. / 位置目标。
    uint16_t current_limit_mA; ///< Temporary command current limit; zero uses configured limit. / 临时命令电流限制；零表示使用配置限制。
    uint32_t speed_limit_cps;  ///< Temporary command speed limit; zero uses SpeedMax. / 临时命令速度限制；零表示使用 SpeedMax。
} ServoCommand;

/**
 * @brief Runtime control source selected by the command interface.
 * @brief 命令接口选择的运行控制源。
 */
typedef enum
{
    CONTROL_SOURCE_DISABLED = 0,   ///< Output disabled. / 输出关闭。
    CONTROL_SOURCE_SERIAL = 1,     ///< DYNAMIXEL serial control. / DYNAMIXEL 串口控制。
    CONTROL_SOURCE_PWM_INPUT = 2,  ///< PWM input control. / PWM 输入控制。
    CONTROL_SOURCE_CRSF = 3        ///< CRSF RC-channel control. / CRSF 遥控通道控制。
} ControlSource;

/** CRSF runtime state exposed through the Control Table. / 通过控制表公开的 CRSF 运行状态。 */
typedef enum
{
    CRSF_STATUS_FRAME_VALID = (1U << 0),
    CRSF_STATUS_POSITION_CHANNEL_VALID = (1U << 1),
    CRSF_STATUS_ENABLE_CHANNEL_VALID = (1U << 2),
    CRSF_STATUS_ENABLE_REQUEST = (1U << 3),
    CRSF_STATUS_ARM_TRACKING = (1U << 4),
    CRSF_STATUS_ACTIVE = (1U << 5),
    CRSF_STATUS_ARM_FAILED = (1U << 6),
    CRSF_STATUS_TIMEOUT = (1U << 7),
    CRSF_STATUS_CENTER_REFERENCE_VALID = (1U << 8)
} CrsfStatusMask;

/**
 * @brief Fail-safe action used when serial control times out.
 * @brief 串口控制超时时的失效保护动作。
 */
typedef enum
{
    FAILSAFE_DISABLE_OUTPUT = 0, ///< Coast/disable output. / 关闭输出。
    FAILSAFE_BRAKE = 1,          ///< Brake output. / 刹车输出。
    FAILSAFE_FALLBACK_PWM = 2    ///< Return to PWM input. / 回退到 PWM 输入。
} FailSafePolicy;

/**
 * @brief Runtime output-inhibit reasons reported through Status Word.
 * @brief 通过 Status Word 上报的运行期输出禁止原因。
 */
typedef enum
{
    PROTECTION_NONE = 0,
    PROTECTION_UNDERVOLTAGE = (1U << 0),
    PROTECTION_OVERTEMPERATURE = (1U << 1),
    PROTECTION_TORQUE_MODEL_INVALID = (1U << 2),
    PROTECTION_OVERCURRENT = (1U << 3),
    PROTECTION_STALL = (1U << 4),
    PROTECTION_ENCODER = (1U << 5)
} ProtectionFlag;

typedef enum
{
    BUS_TOPOLOGY_PARALLEL = 0,
    BUS_TOPOLOGY_CHAIN = 1
} BusTopology;

typedef struct
{
    uint16_t Kp;
    uint16_t Ki;
    uint16_t Kd;
    int32_t integral;
    int32_t integral_max;
    int32_t prev_error;
    int32_t prev_prev_error;
    int32_t prev_feedback;
    uint16_t out_max;
    uint16_t out_min;
    int16_t prev_out;
} PID_Int;

typedef struct
{
    uint16_t Kp;
    uint16_t Ki;
    uint16_t Kd;
    int32_t integral_max;
    uint16_t out_max;
    uint16_t out_min;
} PID_SaveParam;

typedef struct
{
    PID_SaveParam Pid_Pos;
    PID_SaveParam Pid_PosVel;
    PID_SaveParam Pid_PosEle;
    int8_t LowSpeedCompMap_mA[2][LOW_SPEED_COMP_BIN_COUNT];

    uint8_t CycleTimeMs;
    int8_t TempLimit;

    uint16_t EncoderOffset;
    uint8_t LowSpeedCompEnable;
    uint8_t PositionDeadbandCounts;
    uint16_t LowSpeedCompMaxSpeed_cps;
    uint32_t LowSpeedCompReserved2;
    uint16_t SpeedMax;
    uint16_t AccelMax;
    uint16_t DecelMax;
    bool EncoderVeer;

    uint8_t DriveRunMode;
    bool DriveVeerFlag;
    int16_t ExpectMA;
    uint16_t PowerSaveVoltage_mV;
    uint32_t BaudRate;
    uint16_t SerialWatchdogMs;
    uint8_t PdoMissLimit;
    uint8_t FailSafePolicy;
    uint8_t NodeId;
    uint8_t Topology;
    uint8_t NodeCount;
    uint8_t NodePosition;
    uint16_t ReplySlotUs;

    uint8_t DrivePwmMode;
    uint8_t ControlSource;

    uint8_t CrsfPositionChannel;
    uint8_t CrsfCenterChannel;
    uint8_t CrsfEnableChannel;
    bool CrsfAutoEnable;
    uint8_t CrsfReserved164;
    uint16_t CrsfChannelMin;
    uint16_t CrsfChannelCenter;
    uint16_t CrsfChannelMax;
    uint16_t CrsfCenterTrigger;
    uint16_t CrsfEnableThreshold;
    uint16_t CrsfReserved176;
    uint16_t CrsfArmCurrentLimit_mA;
    uint32_t CrsfArmSpeed_cps;
    uint16_t CrsfArmFollowError;
    uint16_t CrsfArmTimeoutMs;
    uint16_t CrsfWatchdogMs;
    int32_t CrsfNegativePositionLimit;
    int32_t CrsfPositivePositionLimit;

    uint16_t TorqueEncoderCountsPerRev;
    uint16_t TorqueCurrentLimit_mA;
    MotorTorqueModelParams MotorTorqueParams;
    MechanicalModelParams MechanicalParams;
    uint16_t MotorInductance_uH;
    uint16_t CurrentPeakLimit_mA;
    uint16_t CurrentAbsoluteLimit_mA;
    uint16_t StallCurrentThreshold_mA;
    uint16_t StallSpeedThreshold_cps;
    uint16_t StallConfirmTimeMs;
} Param_SaveData;

typedef struct
{
    /* One DMA transaction holds the complete maximum stuffed protocol packet. */
    /* 一次 DMA 事务容纳完整的最大填充协议数据包。 */
    uint8_t RxBuf[256];
    uint8_t TxBuf[256];
    bool ReturnEn;

    uint16_t DutyRatio;
    bool PwmInputValid;       ///< A recent valid external PWM command is present. / 存在近期有效的外部 PWM 命令。
    bool OutputEnabled;       ///< Servo output is armed after protection checks. / 保护检查后伺服输出处于使能状态。

    volatile uint16_t VoltageBuf[5];
    uint16_t INA181_mV;
    int16_t INA181_mA;
    int16_t CurrentLogical_mA; ///< Model-assisted signed current; not bidirectional hardware feedback. / 模型辅助带符号电流，非硬件双向反馈。
    bool CurrentSampleValid; ///< Current magnitude sample is inside an active bridge interval. / 电流幅值采样位于 H 桥有效导通区。
    bool CurrentEstimated;   ///< Sign or magnitude uses the command/electrical model. / 符号或幅值使用指令/电气模型估算。
    bool CurrentHardLimitActive; ///< Cycle current exceeded the hardware-safe limit. / 周期电流超过硬件安全上限。
    int16_t ExpectMA;
    uint16_t INA181REF_mV;

    /* Current-sense & current-loop diagnostics, exposed read-only at 290..333. */
    /* 电流采样与电流环诊断，只读暴露于地址 290..333；不写入 NVM。 */
    uint16_t CurrentAdcRaw;         ///< Latest shunt ADC counts. / 最新分流 ADC 原始码值。
    uint16_t CurrentAdcOffset;      ///< Zero-current ADC offset. / 零电流 ADC 偏置。
    int16_t CurrentInstant_mA;      ///< Unfiltered qualified sample. / 未滤波的合格瞬时采样。
    int16_t CurrentModelPwm;        ///< Pure electrical-model feedforward PWM. / 纯电气模型前馈 PWM。
    int16_t CurrentCorrectionPwm;   ///< Current-loop PI correction PWM. / 电流环 PI 修正 PWM。
    uint16_t CurrentLoopStatus;     ///< bit0 PI running, bit1 sample valid, bit2 estimate, bit3 frozen, bit4 direction reset, bit5 voltage limited, bit6 peak chop, bit7 model brake. / 电流环状态位。
    uint16_t CurrentBridgeStatus;   ///< Low byte DriveRunMode, high byte last sample bridge mode. / 低字节桥模式，高字节采样桥模式。
    uint16_t CurrentSampleAgeMs;    ///< Age of the newest qualified sample. / 最新合格样本的年龄。
    uint16_t CurrentWindowValid;    ///< Valid samples in the statistics window. / 统计窗口内有效样本数。
    uint16_t CurrentWindowInvalid;  ///< Invalid samples in the statistics window. / 统计窗口内无效样本数。
    int16_t CurrentWindowMin_mA;    ///< Window minimum current. / 窗口最小电流。
    int16_t CurrentWindowMax_mA;    ///< Window maximum current. / 窗口最大电流。
    int16_t CurrentWindowAvg_mA;    ///< Window average of valid samples. / 窗口有效样本平均。
    uint16_t CurrentHardLimitTrips; ///< Hard current-limit events. / 硬限流事件次数。
    uint16_t CurrentPiFrozenCount;  ///< PI frozen (left observable region) events. / PI 冻结事件次数。
    int16_t CurrentAverage_mA;      ///< R/Ke model estimate of cycle-average winding current. / R/Ke 模型估算的周期平均绕组电流。
    uint16_t CurrentPeakChopEvents; ///< Non-latching peak-current chop events. / 非锁存峰值削波事件次数。
    bool CurrentPeakLimitActive;    ///< The actuator is suppressing a peak-current pulse. / 执行器正在抑制峰值电流脉冲。
    bool CurrentPiWasRunning;       ///< Previous-update PI running state. / 上一拍 PI 运行状态。
    uint32_t CurrentValidTotal;     ///< Lifetime qualified samples. / 累计有效样本。
    uint32_t CurrentInvalidTotal;   ///< Lifetime invalid samples. / 累计无效样本。
    uint16_t VCC_mV;
    uint16_t PowerSaveVoltage_mV;
    uint16_t Temp_mV;
    int8_t Temp;
    int8_t TempLimit;

    uint16_t TorqueEncoderCountsPerRev;
    uint16_t TorqueCurrentLimit_mA;
    uint16_t MotorInductance_uH;
    uint16_t CurrentPeakLimit_mA;
    uint16_t CurrentAbsoluteLimit_mA;
    uint16_t StallCurrentThreshold_mA;
    uint16_t StallSpeedThreshold_cps;
    uint16_t StallConfirmTimeMs;
    uint16_t StallElapsedMs;
    int16_t MotorWindingTemperature_C;
    MotorTorqueModelParams MotorTorqueParams;
    MechanicalModelParams MechanicalParams;
    MotorTorqueModelResult MotorTorqueResult;
    MechanicalModelResult MechanicalResult;
    int32_t TargetTorque_uNm;
    int32_t TargetElectromagneticTorque_uNm;
    bool TorqueCommandVoltageLimited;
    uint16_t LowSpeedCompMaxSpeed_cps; ///< Zero disables; otherwise compensation fades out here. / 零表示关闭，否则补偿在该速度处衰减为零。
    int8_t LowSpeedCompMap_mA[2][LOW_SPEED_COMP_BIN_COUNT]; ///< [0] forward, [1] reverse current-deviation maps. / [0] 正转、[1] 反转电流偏差表。

    volatile uint8_t EncoderReadData[2];
    uint16_t EncoderSampleAgeMs;
    bool EncoderFeedbackValid;
    uint16_t EncoderValue;
    uint16_t EncoderOffset;
    uint16_t LastEncoderValue;
    bool EncoderRebaseline;
    uint8_t PositionDeadbandCounts; ///< Position-loop stop band in encoder counts. / 位置环停止死区，单位 count。
    int16_t EncoderExpect;
    int32_t EncoderSpeed;
    int32_t EncoderSpeedExpect;
    int32_t LastEncoderSpeed;
    int16_t EncoderMultiTurn;
    int32_t EncoderMultiTurnValue;
    int32_t LastEncoderMultiTurnValue;
    uint16_t SpeedMax;
    int32_t AccDec;
    uint16_t AccelMax;       ///< Acceleration in 1000 count/s^2 units. / 加速度，单位 1000 count/s²。
    uint16_t DecelMax;       ///< Deceleration in 1000 count/s^2 units. / 减速度，单位 1000 count/s²。
    int32_t SpeedRef;
    int32_t target_speed;
    bool EncoderVeer;

    bool DriveVeerFlag;
    uint8_t DriveRunMode;
    uint8_t DrivePwmMode;
    int16_t DrivePower;
    int16_t CurrentFeedforwardPwm; ///< Model PWM plus observable-current voltage trim. / 模型PWM与可观测电流电压修正之和。

    PID_Int Pid_Pos;
    PID_Int Pid_PosVel;
    PID_Int Pid_PosEle;
    PID_Int Pid_Vel;
    PID_Int Pid_Ele;
    uint8_t CycleTimeMs;
    uint16_t ProcessTimeUs;
    uint32_t BaudRate;
    uint16_t SerialWatchdogMs;
    uint8_t PdoMissLimit;
    uint8_t FailSafePolicy;
    uint8_t ControlSource;
    uint8_t NodeId;
    uint8_t Topology;
    uint8_t NodeCount;
    uint8_t NodePosition;
    uint16_t ReplySlotUs;
    uint16_t FaultCode;
    uint8_t ProtectionFlags;

    uint8_t CrsfPositionChannel;
    uint8_t CrsfCenterChannel;
    uint8_t CrsfEnableChannel;
    bool CrsfAutoEnable;
    bool CrsfManualEnable;
    uint16_t CrsfChannelMin;
    uint16_t CrsfChannelCenter;
    uint16_t CrsfChannelMax;
    uint16_t CrsfCenterTrigger;
    uint16_t CrsfEnableThreshold;
    uint16_t CrsfArmCurrentLimit_mA;
    uint32_t CrsfArmSpeed_cps;
    uint16_t CrsfArmFollowError;
    uint16_t CrsfArmTimeoutMs;
    uint16_t CrsfWatchdogMs;
    int32_t CrsfNegativePositionLimit;
    int32_t CrsfPositivePositionLimit;
    uint16_t CrsfStatus;
    uint16_t CrsfRawPosition;
    uint16_t CrsfRawEnable;
    uint16_t CrsfRawCenter;
    int32_t CrsfCenterReference;
} Param;

#endif // TRIPLE_CASCADECONTROLDCMOTOR_TYPEDEFINE_H
