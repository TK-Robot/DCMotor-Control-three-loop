# DCMotor-Control-three-loop

## TK Servo Bus 协议 / TK Servo Bus Protocol

本项目的串口伺服总线协议文档已按模块拆分到 [docs/tsbp](docs/tsbp/README.md)，包含中英文双语说明、主站规则、两种拓扑、PDO/SDO、对象字典、故障重连、分布式时钟和 XML 类从站配置文件。

The UART servo bus protocol documentation is split into modules under [docs/tsbp](docs/tsbp/README.md). It includes bilingual protocol rules, master requirements, two supported topologies, PDO/SDO, object dictionary, fault reconnect, distributed clock sync, and an XML-like slave description file.

## 项目简介 / Overview

这是一个基于 STM32G030 的超小体积有刷直流电机伺服项目。受 DRV8837 公共低侧单向电流采样限制，系统采用位置 P、速度 PI、R/Ke 电压前馈、硬件时间平均电流修正和同步保护，而不是伪造四象限绕组电流反馈。项目包含 MT6701 磁编码器反馈、PWM 输出、INA181 电流采样、电源电压/温度监测、DYNAMIXEL/CRSF/PWM 输入、掉电参数保存和 PC 端控制逻辑仿真。

This compact STM32G030 brushed-DC servo uses position P, velocity PI, R/Ke voltage feed-forward, observable-region current PI trim, and synchronous peak-current protection. The DRV8837 common low-side unidirectional shunt cannot support a true four-quadrant average-current loop.

## 最新更新 / Recent Updates

- 新增三种伺服控制模式：电流模式、速度模式、位置模式。  
  Added three servo control modes: current, speed, and position.
- 新增 `ServoControl` 调度层，实现 1 kHz 速度观测/PI、200 Hz 位置环和 50 Hz 遥测。
  Added 1 kHz velocity observation/PI, a 200 Hz position loop, and 50 Hz telemetry.
- 目标电流通过 `R·I + Ke·ω` 模型转换为 PWM 前馈；PA0 恢复 16 次硬件过采样，在约 26 µs 窗口内平均窄 PWM 脉冲与 INA181 响应，再经低通供 1 kHz PI 修正。未实测的电感参数不参与 PWM 限幅；单向分流无法观测制动/续流回路，因此反馈明确表示桥侧时间平均值，不冒充四象限绕组电流。
  Model current becomes PWM feed-forward through `R·I + Ke·ω`; synchronized active-window samples plus R/L decay reconstruct cycle-average winding current for the 1 kHz PI, with separate peak chopping and absolute overcurrent latching.
- 新增掉电保存模块 `NvmParam`，使用 Flash 尾页保存配置参数。  
  Added `NvmParam` to store configuration data in the reserved last Flash page.
- 新增 `Sim` 目录，提供 PC 端逻辑仿真和单元测试源码。  
  Added the `Sim` directory for PC-side logic simulation and unit-test sources.
- Release 默认启用 `-Oz`、LTO 和纯 C 最小启动；当前固件为 30,648 B，低于 30 KiB 程序区限制。
  Release enables `-Oz`, LTO, and a minimal pure-C startup; the current firmware is 30,648 B and fits the 30 KiB application region.

## 运动控制架构 / Motion Control Architecture

系统采用工业伺服常见的级联控制结构：

The system uses a common industrial-servo cascaded control structure:

```text
Position P -> Velocity PI -> Current Ref -> Model FF + Current PI -> PWM -> Motor
位置 P     -> 速度 PI     -> 电流目标    -> 模型前馈 + 电流PI     -> PWM -> 电机
```

三种控制模式按启用的外环数量区分：

The three control modes differ by how many outer loops are enabled:

```text
Hybrid Current Mode:
target_current -> R/Ke Voltage FF + observable current PI -> PWM
混合电流模式：
目标电流       -> R/Ke 电压前馈 + 可观测区电流PI          -> PWM

Speed Mode:
target_speed -> Velocity PI -> target_current -> Model FF + Current PI -> PWM
速度模式：
目标速度     -> 速度 PI  -> 目标电流       -> 模型前馈 + 电流PI -> PWM

Position Mode:
target_position -> Position P -> target_speed -> Velocity PI -> target_current -> Model FF + Current PI -> PWM
位置模式：
目标位置        -> 位置 P      -> 目标速度     -> 速度 PI  -> 目标电流       -> 模型前馈 + 电流PI -> PWM
```

控制调度由 `ServoControl` 完成：

Control scheduling is handled by `ServoControl`:

- PWM 周期：同步 ADC 幅值采样、硬峰值保护和 PWM 更新。
  PWM cycle: synchronous magnitude sampling, hard peak protection, and PWM update.
- 1 ms：读取 MT6701，运行速度观测器、速度 PI 和保护调度。
  1 ms: update MT6701, velocity observer, velocity PI, and protection scheduling.
- 5 ms：位置模式下运行位置 P 环。
  5 ms: run the position P loop in position mode.
- 20 ms：发送 UART 遥测数据。  
  20 ms: send UART telemetry.

`DriveRunMode` 只表示 H 桥功率模式，不表示伺服控制模式：

`DriveRunMode` only represents the H-bridge power stage mode, not the servo control mode:

```text
0 = coast
1 = brake
2 = slow decay
3 = fast decay
```

伺服控制模式由 `ServoMode` 表示：

Servo control mode is represented by `ServoMode`:

```c
typedef enum {
    SERVO_MODE_CURRENT = 0,
    SERVO_MODE_SPEED,
    SERVO_MODE_POSITION
} ServoMode;
```

串口协议后续只需要写入运行命令结构：

The UART protocol can later update this runtime command structure:

```c
typedef struct {
    ServoMode mode;
    bool enable;
    int16_t target_current_mA;
    int32_t target_speed;
    int32_t target_position;
} ServoCommand;
```

## 主要模块 / Main Modules

```text
Core/                  STM32CubeMX generated startup, peripheral init, and main loop
Drivers/               STM32 HAL and CMSIS drivers
Libraries/AD116        AD116 H-bridge PWM driver
Libraries/MT6701       MT6701 magnetic encoder readout and speed calculation
Libraries/PID          Integer PID loops and speed planning
Libraries/ServoControl Three-mode servo scheduler
Libraries/NvmParam     Flash-backed non-volatile parameter storage
Libraries/VoltageStatus ADC current, voltage, temperature, and power-loss detection
Libraries/UartProto    UART receive and telemetry helpers
Libraries/Filter       Integer low-pass, moving-average, and Kalman filters
Libraries/PWMCapture   PWM input capture helper
Sim/                   PC-side simulation and unit-test sources
```

## 掉电保存 / Non-Volatile Storage

项目将 STM32G030 最后 2 KB Flash 页预留为参数保存区，程序链接区限制为 30 KB。`NvmParam` 使用追加式记录保存配置，并通过 magic、version、sequence 和 CRC32 校验数据完整性。默认低于 4 V 时触发一次保存请求，并关闭输出。

The last 2 KB Flash page is reserved for parameter storage, leaving 30 KB for application code. `NvmParam` stores configuration data as append-only records and validates them with magic, version, sequence, and CRC32. By default, the system requests one save below 4 V and disables motor output.

## 构建 / Build

推荐使用 Release 构建：

Release build is recommended:

```powershell
cmake --preset Release
cmake --build --preset Release
arm-none-eabi-size build\Release\Triple-CascadeControlDCMotor.elf
```

当前 Release 默认启用 `-Oz` 和 LTO，实测占用 30,648 B（30 KiB 程序区的 99.77%）。

Release enables `-Oz` and LTO by default and currently uses 30,648 B (99.77% of the 30 KiB application region).

## 仿真与测试 / Simulation and Tests

`Sim` 目录提供两个 PC 端目标：

The `Sim` directory provides two PC-side targets:

```text
servo_sim    Outputs CSV simulation data for current/speed/position modes
servo_tests  Runs basic control-logic unit tests
```

示例构建方式：

Example build:

```powershell
cmake -S Sim -B build\Sim -G Ninja
cmake --build build\Sim
build\Sim\servo_tests.exe
build\Sim\servo_sim.exe > sim.csv
```

注意：PC 仿真需要本机安装桌面 C 编译器，例如 MinGW、Clang 或 Visual Studio Build Tools。

Note: PC simulation requires a host C compiler such as MinGW, Clang, or Visual Studio Build Tools.

## 许可证 / License

本项目基于 Apache License 2.0 开源。

This project is open-sourced under the Apache License 2.0.
