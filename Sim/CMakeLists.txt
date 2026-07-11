cmake_minimum_required(VERSION 3.16)
project(TripleCascadeServoSim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(servo_sim
    servo_sim.c
    ../Libraries/PID/PID.c
    ../Libraries/ServoControl/ServoControl.c
)

target_include_directories(servo_sim PRIVATE
    ../Libraries
    ../Libraries/PID
    ../Libraries/ServoControl
)

add_executable(servo_tests
    servo_tests.c
    ../Libraries/PID/PID.c
    ../Libraries/ServoControl/ServoControl.c
)

target_include_directories(servo_tests PRIVATE
    ../Libraries
    ../Libraries/PID
    ../Libraries/ServoControl
)

add_executable(tsbp_bus_sim
    tsbp_bus_sim.c
)

add_executable(tsbp_protocol_tests
    tsbp_protocol_tests.c
)
