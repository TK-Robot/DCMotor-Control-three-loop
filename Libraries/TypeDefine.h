/**
 * @file TypeDefine.h
 * @brief Project-wide data types and parameter definitions.
 * @brief 项目通用数据类型与参数定义。
 */

#ifndef TRIPLE_CASCADECONTROLDCMOTOR_TYPEDEFINE_H
#define TRIPLE_CASCADECONTROLDCMOTOR_TYPEDEFINE_H

#include <stdbool.h>
#include <stdint.h>

#define MAP(x, in_min, in_max, out_min, out_max) \
(((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))

/**
 * @brief Servo outer-loop control mode.
 * @brief 伺服外环控制模式。
 */
typedef enum
{
    SERVO_MODE_CURRENT = 0, ///< Current loop only. / 仅电流环。
    SERVO_MODE_SPEED,       ///< Speed loop plus current loop. / 速度环加电流环。
    SERVO_MODE_POSITION     ///< Position, speed, and current loops. / 位置、速度、电流三环。
} ServoMode;

/**
 * @brief Runtime servo command written by the future command protocol.
 * @brief 后续命令协议写入的运行期伺服指令。
 */
typedef struct
{
    ServoMode mode;           ///< Active servo mode. / 当前伺服模式。
    bool enable;              ///< Output enable command. / 输出使能指令。
    int16_t target_current_mA; ///< Current target in mA. / 电流目标，单位 mA。
    int32_t target_speed;     ///< Speed target. / 速度目标。
    int32_t target_position;  ///< Position target. / 位置目标。
} ServoCommand;

/**
 * @brief Runtime control source selected by the command interface.
 * @brief 命令接口选择的运行控制源。
 */
typedef enum
{
    CONTROL_SOURCE_DISABLED = 0,   ///< Output disabled. / 输出关闭。
    CONTROL_SOURCE_SERIAL = 1,     ///< DYNAMIXEL serial control. / DYNAMIXEL 串口控制。
    CONTROL_SOURCE_PWM_INPUT = 2   ///< PWM input control. / PWM 输入控制。
} ControlSource;

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
    PROTECTION_OVERTEMPERATURE = (1U << 1)
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
    PID_SaveParam Pid_Vel;
    PID_SaveParam Pid_Ele;

    uint8_t CycleTimeMs;
    int8_t TempLimit;

    uint16_t EncoderOffset;
    int16_t EncoderExpect;
    int32_t EncoderSpeedExpect;
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
    uint8_t reserved[3];
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

    uint16_t VoltageBuf[5];
    uint16_t INA181_mV;
    int16_t INA181_mA;
    int16_t CurrentLogical_mA;
    int16_t ExpectMA;
    uint16_t INA181REF_mV;
    uint16_t VCC_mV;
    uint16_t PowerSaveVoltage_mV;
    uint16_t Temp_mV;
    int8_t Temp;
    int8_t TempLimit;

    volatile uint8_t EncoderReadData[2];
    uint16_t EncoderValue;
    uint16_t EncoderOffset;
    uint16_t LastEncoderValue;
    bool EncoderRebaseline;
    int16_t EncoderExpect;
    int32_t EncoderSpeed;
    int32_t EncoderSpeedExpect;
    int32_t LastEncoderSpeed;
    int16_t EncoderMultiTurn;
    int32_t EncoderMultiTurnValue;
    int32_t LastEncoderMultiTurnValue;
    uint16_t SpeedMax;
    int32_t AccDec;
    uint16_t AccelMax;
    uint16_t DecelMax;
    int32_t SpeedRef;
    int32_t target_speed;
    bool EncoderVeer;

    bool DriveVeerFlag;
    uint8_t DriveRunMode;
    uint8_t DrivePwmMode;
    int16_t DrivePower;

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
} Param;

#endif // TRIPLE_CASCADECONTROLDCMOTOR_TYPEDEFINE_H
