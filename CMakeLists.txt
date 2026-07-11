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
        Libraries/AD116/AD116.c
        Libraries/AD116/AD116.h
        Libraries/VoltageStatus/VoltageStatus.c
        Libraries/VoltageStatus/VoltageStatus.h
        Libraries/TypeDefine.h
        Libraries/PID/PID.h
        Libraries/PID/PID.c
        Libraries/PID/PID.h
        Libraries/Filter/Filter.c
        Libraries/Filter/Filter.h
        Libraries/PWMCapture/PWMCapture.c
        Libraries/PWMCapture/PWMCapture.h
        Libraries/NvmParam/NvmParam.c
        Libraries/NvmParam/NvmParam.h
        Libraries/ServoControl/ServoControl.c
        Libraries/ServoControl/ServoControl.h
        Libraries/Tsbp/Tsbp.c
        Libraries/Tsbp/Tsbp.h
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
        Libraries/AD116
        Libraries/VoltageStatus
        Libraries
        Libraries/PID
        Libraries/Filter
        Libraries/PWMCapture
        Libraries/NvmParam
        Libraries/ServoControl
        Libraries/Tsbp
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

# STM32G030 uses a Cortex-M0+ core without hardware FPU.
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM0PLUS
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)
