cmake_minimum_required(VERSION 3.16)
project(TripleCascadeServoSim C)

enable_testing()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(servo_sim
    servo_sim.c
    ../Libraries/PID/PID.c
    ../Libraries/ServoControl/ServoControl.c
    ../Libraries/MotorTorqueModel/MotorTorqueModel.c
    ../Libraries/MechanicalModel/MechanicalModel.c
    ../Libraries/FixedPointMath/FixedPointMath.c
)

target_include_directories(servo_sim PRIVATE
    ../Libraries
    ../Libraries/PID
    ../Libraries/ServoControl
    ../Libraries/MotorTorqueModel
    ../Libraries/MechanicalModel
    ../Libraries/CurrentSenseModel
    ../Libraries/FixedPointMath
)

add_executable(servo_tests
    servo_tests.c
    ../Libraries/PID/PID.c
    ../Libraries/ServoControl/ServoControl.c
    ../Libraries/MotorTorqueModel/MotorTorqueModel.c
    ../Libraries/MechanicalModel/MechanicalModel.c
    ../Libraries/FixedPointMath/FixedPointMath.c
)

target_include_directories(servo_tests PRIVATE
    ../Libraries
    ../Libraries/PID
    ../Libraries/ServoControl
    ../Libraries/MotorTorqueModel
    ../Libraries/MechanicalModel
    ../Libraries/CurrentSenseModel
    ../Libraries/FixedPointMath
)

add_test(NAME servo_tests COMMAND servo_tests)

add_executable(torque_model_tests
    torque_model_tests.c
    ../Libraries/MotorTorqueModel/MotorTorqueModel.c
    ../Libraries/MechanicalModel/MechanicalModel.c
    ../Libraries/CurrentSenseModel/CurrentSenseModel.c
    ../Libraries/FixedPointMath/FixedPointMath.c
)

target_include_directories(torque_model_tests PRIVATE
    ../Libraries/MotorTorqueModel
    ../Libraries/MechanicalModel
    ../Libraries/CurrentSenseModel
    ../Libraries/FixedPointMath
)

add_test(NAME torque_model_tests COMMAND torque_model_tests)

add_executable(current_control_tests
    current_control_tests.c
    ../Libraries/CurrentControl/CurrentControl.c
    ../Libraries/CurrentSenseModel/CurrentSenseModel.c
    ../Libraries/Filter/Filter.c
)

target_include_directories(current_control_tests PRIVATE
    ../Libraries
    ../Libraries/CurrentControl
    ../Libraries/CurrentSenseModel
    ../Libraries/Filter
)

add_test(NAME current_control_tests COMMAND current_control_tests)

add_executable(velocity_observer_tests
    velocity_observer_tests.c
    ../Libraries/VelocityObserver/VelocityObserver.c
)

target_include_directories(velocity_observer_tests PRIVATE
    ../Libraries/VelocityObserver
)

add_test(NAME velocity_observer_tests COMMAND velocity_observer_tests)

add_executable(dynamixel2_protocol_tests
    dynamixel2_protocol_tests.c
    ../Libraries/Dynamixel2/Dynamixel2Codec.c
)

target_include_directories(dynamixel2_protocol_tests PRIVATE
    ../Libraries
    ../Libraries/Dynamixel2
)

add_test(NAME dynamixel2_protocol_tests COMMAND dynamixel2_protocol_tests)

add_executable(crsf_tests
    crsf_tests.c
    ../Libraries/Crsf/Crsf.c
    ../Libraries/FixedPointMath/FixedPointMath.c
)

target_include_directories(crsf_tests PRIVATE
    ../Libraries
    ../Libraries/Crsf
    ../Libraries/FixedPointMath
)

add_test(NAME crsf_tests COMMAND crsf_tests)
