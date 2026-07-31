cmake_minimum_required(VERSION 3.14)
project(pika_sensor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Expect: source /opt/ros/humble/setup.bash before cmake/build
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

if(NOT TARGET yaml-cpp::yaml-cpp AND TARGET yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

if(NOT TARGET kdl_parser::kdl_parser)
  # Older kdl-parser packages (e.g. Debian libkdl-parser-dev) only export plain
  # *_LIBRARIES / *_INCLUDE_DIRS variables, not a modern imported target.
  add_library(kdl_parser::kdl_parser INTERFACE IMPORTED)
  target_link_libraries(kdl_parser::kdl_parser INTERFACE ${kdl_parser_LIBRARIES})
  target_include_directories(kdl_parser::kdl_parser INTERFACE ${kdl_parser_INCLUDE_DIRS})
endif()

# -----------------------------------------------------------------------------
# openarm_can_local — vendored OpenArm CAN driver (third_party/openarm_can)
# -----------------------------------------------------------------------------
set(OPENARM_CAN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/openarm_can)

add_library(openarm_can_local STATIC
    ${OPENARM_CAN_DIR}/src/openarm/can/socket/arm_component.cpp
    ${OPENARM_CAN_DIR}/src/openarm/can/socket/gripper_component.cpp
    ${OPENARM_CAN_DIR}/src/openarm/can/socket/openarm.cpp
    ${OPENARM_CAN_DIR}/src/openarm/canbus/can_device_collection.cpp
    ${OPENARM_CAN_DIR}/src/openarm/canbus/can_socket.cpp
    ${OPENARM_CAN_DIR}/src/openarm/damiao_motor/dm_motor.cpp
    ${OPENARM_CAN_DIR}/src/openarm/damiao_motor/dm_motor_control.cpp
    ${OPENARM_CAN_DIR}/src/openarm/damiao_motor/dm_motor_device.cpp
    ${OPENARM_CAN_DIR}/src/openarm/damiao_motor/dm_motor_device_collection.cpp
)

set_target_properties(openarm_can_local PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(openarm_can_local PUBLIC
    ${OPENARM_CAN_DIR}/include
)

# -----------------------------------------------------------------------------
# openarm_sensor_lib — controller / IK / interpolator / safety / hardware / io
# -----------------------------------------------------------------------------
add_library(openarm_sensor_lib STATIC
    src/controller/cartesian_mapper.cpp
    src/controller/control.cpp
    src/controller/dynamics.cpp
    src/ik/ik_solver.cpp
    src/ik/dls_ik.cpp
    src/interpolator/joint_interpolator.cpp
    src/interpolator/linear_joint_interpolator.cpp
    src/safety/safety_filter.cpp
    src/hardware/teleop_hardware.cpp
    src/openarm_port/openarm_init.cpp
    src/io/pika_pose_ros.cpp
    src/io/tcp_relative_csv_writer.cpp
    src/health_check.cpp
    src/pose_health_monitor.cpp
)

target_include_directories(openarm_sensor_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${orocos_kdl_INCLUDE_DIRS}
)

target_link_libraries(openarm_sensor_lib PUBLIC
    openarm_can_local
    Eigen3::Eigen
    ${orocos_kdl_LIBRARIES}
    kdl_parser::kdl_parser
    urdfdom::urdfdom_model
    yaml-cpp::yaml-cpp
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    Threads::Threads
)

target_compile_options(openarm_sensor_lib PRIVATE -O2 -Wall -Wextra)

# -----------------------------------------------------------------------------
# pika_control — Pika sensor -> OpenArm real-hardware Cartesian teleop
# (120Hz IK/admin thread + 1000Hz linear-interpolated follower/CAN thread)
# NOTE: no MuJoCo dependency; real hardware only.
# -----------------------------------------------------------------------------
add_executable(pika_control
    src/app/openarm_pika_control.cpp
)

target_include_directories(pika_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(pika_control PRIVATE
    openarm_sensor_lib
)

target_compile_options(pika_control PRIVATE -O2 -Wall -Wextra)

install(TARGETS pika_control RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/pika_sensor)
