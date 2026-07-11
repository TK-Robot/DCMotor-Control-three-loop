# Project Agent Instructions

## Scope

- These instructions apply to the entire repository.
- Follow a more specific `AGENTS.md` when working under a directory that contains one.
- Keep changes minimal, focused, and consistent with the existing codebase.
- Do not modify agent instruction files unless explicitly requested.

## Project

- This is a C11 firmware project for an STM32G030 DC motor controller.
- The controller uses cascaded current, speed, and position loops.
- Firmware is built with CMake, Ninja, and the GNU Arm Embedded toolchain.
- `Sim/` provides host-side simulations and logic tests.

## Repository Layout

- `Core/`: STM32CubeMX-generated initialization, interrupts, and main loop.
- `Libraries/`: project-owned control, driver, protocol, filter, and storage modules.
- `Drivers/`: STM32 HAL and CMSIS vendor code.
- `Sim/`: host-side simulations and tests.
- `docs/tsbp/`: TSBP protocol documentation.
- `Triple-CascadeControlDCMotor.ioc`: STM32CubeMX project configuration.

## Critical Constraints

- Preserve STM32CubeMX `USER CODE BEGIN` and `USER CODE END` sections.
- Do not edit CubeMX-generated code outside user sections unless explicitly required and confirmed.
- Treat the `.ioc` file as the source of truth for CubeMX-managed pins, clocks, and peripherals.
- Do not modify files under `Drivers/` unless explicitly requested.
- Ask before changing the linker script, startup code, Flash layout, interrupt priorities, pin assignments, or clock configuration.
- The STM32G030 target has no hardware FPU; avoid floating-point work in real-time control paths.
- Preserve the 30 KiB firmware region and the final 2 KiB Flash page reserved for non-volatile parameters.
- Do not add blocking operations or unbounded work to interrupts or the 1 ms control path.
- Preserve UTF-8 encoding in bilingual source files and documentation.

## Code Style

- Use C11 and the existing formatting style.
- Use four-space indentation and braces on separate lines.
- Prefer fixed-width integer types for hardware, protocol, and control values.
- Keep module APIs in matching `.h` and `.c` files, and keep internal helpers `static`.
- Use explicit bounds, saturation, and error handling for control and hardware values.
- Add comments only for hardware constraints, timing requirements, state transitions, or other non-obvious intent.

## Behavior Changes

- Add or update host-side tests for logic that can run without STM32 hardware.
- Protocol changes must update the relevant files under `docs/tsbp/` and `Sim/tsbp_protocol_tests.c`.
- Servo or PID behavior changes must update `Sim/servo_tests.c`.
- NVM format changes must preserve versioning, CRC validation, and backward-compatibility considerations.
- For hardware-only behavior, provide a concise manual verification procedure when host testing is not possible.

## Firmware Build

Configure and build the Release firmware:

```powershell
cmake --preset Release
cmake --build --preset Release
```

Check firmware size:

```powershell
arm-none-eabi-size build\Release\Triple-CascadeControlDCMotor.elf
```

## Host Tests

Configure and build the host-side targets:

```powershell
cmake -S Sim -B build\Sim -G Ninja
cmake --build build\Sim
```

Run the relevant tests:

```powershell
build\Sim\servo_tests.exe
build\Sim\tsbp_protocol_tests.exe
```

- Run the narrowest relevant test first.
- Report commands that could not be run and explain why.

## Change Boundaries

- Do not modify generated build output, `.idea/`, or unrelated files.
- Do not reformat vendor or generated files.
- Do not add dependencies, telemetry, analytics, or network calls without approval.
- Do not commit, push, create branches, or perform destructive Git operations unless explicitly requested.
- Do not fix unrelated warnings, bugs, or style issues.

## Completion Report

Report a concise summary, changed files, verification results, and anything not verified.
