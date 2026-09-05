cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Triple-CascadeControlDCMotor)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Libraries/MT6701/MT6701.c
        Libraries/MT6701/MT6701.h
        Libraries/VelocityObserver/VelocityObserver.c
        Libraries/VelocityObserver/VelocityObserver.h
        Libraries/AD116/AD116.c
        Libraries/AD116/AD116.h
        Libraries/VoltageStatus/VoltageStatus.c
        Libraries/VoltageStatus/VoltageStatus.h
        Libraries/CurrentSenseModel/CurrentSenseModel.c
        Libraries/CurrentSenseModel/CurrentSenseModel.h
        Libraries/CurrentControl/CurrentControl.c
        Libraries/CurrentControl/CurrentControl.h
        Libraries/FixedPointMath/FixedPointMath.c
        Libraries/FixedPointMath/FixedPointMath.h
        Libraries/TypeDefine.h
        Libraries/PID/PID.h
        Libraries/PID/PID.c
        Libraries/Filter/Filter.c
        Libraries/Filter/Filter.h
        Libraries/PWMCapture/PWMCapture.c
        Libraries/PWMCapture/PWMCapture.h
        Libraries/NvmParam/NvmParam.c
        Libraries/NvmParam/NvmParam.h
        Libraries/ServoControl/ServoControl.c
        Libraries/ServoControl/ServoControl.h
        Libraries/MotorTorqueModel/MotorTorqueModel.c
        Libraries/MotorTorqueModel/MotorTorqueModel.h
        Libraries/MechanicalModel/MechanicalModel.c
        Libraries/MechanicalModel/MechanicalModel.h
        Libraries/Dynamixel2/Dynamixel2Codec.c
        Libraries/Dynamixel2/Dynamixel2Codec.h
        Libraries/Dynamixel2/Dynamixel2.c
        Libraries/Dynamixel2/Dynamixel2.h
        Libraries/Crsf/Crsf.c
        Libraries/Crsf/Crsf.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        Libraries/MT6701
        Libraries/VelocityObserver
        Libraries/AD116
        Libraries/VoltageStatus
        Libraries/CurrentSenseModel
        Libraries/CurrentControl
        Libraries/FixedPointMath
        Libraries
        Libraries/PID
        Libraries/Filter
        Libraries/PWMCapture
        Libraries/NvmParam
        Libraries/ServoControl
        Libraries/MotorTorqueModel
        Libraries/MechanicalModel
        Libraries/Dynamixel2
        Libraries/Crsf
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)

# Keep the images commonly selected by flashing tools in lockstep with the ELF.
# Without this post-build step CMake can relink a new ELF while leaving an old
# .hex/.bin in the build directory, which makes a successful rebuild appear to
# have no effect on the target.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    VERBATIM
)

# Release LTO is configured toolchain-wide so the application and STM32 driver
# library are optimized together. Debug keeps normal object boundaries and
# symbols, but uses size optimization because the MCU exposes only 30 KiB Flash.
# Release 的 LTO 在工具链中统一配置，使应用与 STM32 驱动库共同优化；Debug 保留
# 普通对象边界和调试符号，但受限于 30 KiB Flash，使用体积优化。
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Os)
endif()

# STM32G030 uses a Cortex-M0+ core without hardware FPU.
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM0PLUS
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)
