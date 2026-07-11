# DCMotor-Control-three-loop

## TK Servo Bus 协议 / TK Servo Bus Protocol

本项目的串口伺服总线协议文档已按模块拆分到 [docs/tsbp](docs/tsbp/README.md)，包含中英文双语说明、主站规则、两种拓扑、PDO/SDO、对象字典、故障重连、分布式时钟和 XML 类从站配置文件。

The UART servo bus protocol documentation is split into modules under [docs/tsbp](docs/tsbp/README.md). It includes bilingual protocol rules, master requirements, two supported topologies, PDO/SDO, object dictionary, fault reconnect, distributed clock sync, and an XML-like slave description file.

## 项目简介 / Overview

这是一个基于 STM32G030 的超小体积直流电机伺服控制项目，面向电流环、速度环和位置环的级联三环控制。项目包含 MT6701 磁编码器反馈、AD116 电机驱动 PWM 输出、INA181 电流采样、电源电压/温度监测、UART 遥测、PWM 输入捕获、掉电参数保存和 PC 端控制逻辑仿真。

This is a compact STM32G030-based DC motor servo control project implementing cascaded current, speed, and position loops. It includes MT6701 magnetic encoder feedback, AD116 motor-driver PWM output, INA181 current sampling, power-voltage/temperature monitoring, UART telemetry, PWM input capture, non-volatile parameter storage, and PC-side control simulation.

## 最新更新 / Recent Updates

- 新增三种伺服控制模式：电流模式、速度模式、位置模式。  
  Added three servo control modes: current, speed, and position.
- 新增 `ServoControl` 调度层，实现 1 ms / 5 ms / 10 ms / 20 ms 合作式多速率调度。  
  Added the `ServoControl` scheduler with cooperative 1 ms / 5 ms / 10 ms / 20 ms multi-rate timing.
- PID 拆分为位置环、速度环、电流环接口，并增加 `PID_Reset()`。  
  Split PID control into position, speed, and current loop APIs, with `PID_Reset()`.
- 新增掉电保存模块 `NvmParam`，使用 Flash 尾页保存配置参数。  
  Added `NvmParam` to store configuration data in the reserved last Flash page.
- 新增 `Sim` 目录，提供 PC 端逻辑仿真和单元测试源码。  
  Added the `Sim` directory for PC-side logic simulation and unit-test sources.
- Release 固件体积当前约 22 KB，仍低于 30 KB 程序区限制。  
  The current Release firmware is about 22 KB, below the 30 KB application Flash region.

## 运动控制架构 / Motion Control Architecture

系统采用工业伺服常见的级联控制结构：

The system uses a common industrial-servo cascaded control structure:

```text
Position Loop -> Speed Loop -> Current Loop -> PWM Driver -> Motor
位置环        -> 速度环     -> 电流环      -> PWM 驱动   -> 电机
```

三种控制模式按启用的外环数量区分：

The three control modes differ by how many outer loops are enabled:

```text
Current Mode:
target_current -> Current Loop -> PWM
电流模式：
目标电流       -> 电流环       -> PWM

Speed Mode:
target_speed -> Speed Loop -> target_current -> Current Loop -> PWM
速度模式：
目标速度     -> 速度环    -> 目标电流       -> 电流环       -> PWM

Position Mode:
target_position -> Position Loop -> target_speed -> Speed Loop -> target_current -> Current Loop -> PWM
位置模式：
目标位置        -> 位置环       -> 目标速度     -> 速度环    -> 目标电流       -> 电流环       -> PWM
```

控制调度由 `ServoControl` 完成：

Control scheduling is handled by `ServoControl`:

- 1 ms：ADC 状态分析、保护判断、电流环、PWM 输出。  
  1 ms: ADC status analysis, protection checks, current loop, and PWM output.
- 5 ms：读取 MT6701，更新速度反馈，运行速度环。  
  5 ms: read MT6701, update speed feedback, and run the speed loop.
- 10 ms：位置模式下运行位置环。  
  10 ms: run the position loop in position mode.
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

当前 Release 目标控制在 22 KiB 左右，适合 STM32G030 的小 Flash 空间。

The current Release target is kept around 22 KiB, suitable for the small Flash size of STM32G030.

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
