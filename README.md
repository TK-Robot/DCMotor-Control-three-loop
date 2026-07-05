# DCMotor-Control-three-loop

基于 STM32G030 的超小体积直流电机控制项目，面向电流环、速度环和位置环级联控制。

A compact STM32G030-based DC motor control project for cascaded current, speed, and position control.

## Features

- STM32G030 Cortex-M0+ firmware based on STM32 HAL and CMake
- MT6701 magnetic encoder position and speed feedback
- AD116 motor driver PWM output control
- INA181 current sampling and voltage/temperature monitoring
- Integer PID control for current, speed, and position loops
- UART DMA telemetry for runtime data observation
- PWM input capture support for external command input

## Project Structure

```text
Core/        STM32CubeMX generated application and peripheral code
Drivers/     STM32 HAL and CMSIS drivers
Libraries/   Motor control, encoder, PID, filters, UART protocol, and sampling modules
cmake/       CMake toolchain and STM32CubeMX source integration
```

## Main Modules

- `Libraries/AD116`: motor driver PWM output and run mode control
- `Libraries/MT6701`: 14-bit magnetic encoder readout, multi-turn position, and speed calculation
- `Libraries/PID`: integer PID loops, speed planning, and feed-forward helpers
- `Libraries/VoltageStatus`: ADC-based current, voltage, and temperature processing
- `Libraries/UartProto`: UART DMA communication helpers
- `Libraries/Filter`: low-pass, moving-average, and Kalman filters
- `Libraries/PWMCapture`: PWM duty-cycle input capture

## Build

This project uses the ARM GNU toolchain and CMake presets.

```powershell
cmake --preset Release
cmake --build --preset Release
```

The build output is generated under `build/` and is intentionally ignored by Git. The `Debug` preset keeps optimization disabled and may exceed the 32 KB flash size of STM32G030.

## License

This project is licensed under the Apache License 2.0.
