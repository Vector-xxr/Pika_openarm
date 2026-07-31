cmake_minimum_required(VERSION 3.14)
project(all_telop LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# Camera / gripper deps (from utils/all)
# ---------------------------------------------------------------------------
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json CONFIG QUIET)
if(NOT nlohmann_json_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(NLOHMANN_JSON QUIET nlohmann_json)
  endif()
endif()
if(NOT nlohmann_json_FOUND AND NOT NLOHMANN_JSON_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    URL_HASH SHA256=d6c65aca6b1ed68e7a184f80ff21ecab1d6ba11c2b7fe5cf83b100ed61f76c2
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  FetchContent_MakeAvailable(json)
endif()

# ---------------------------------------------------------------------------
# Teleop deps (from utils/sensor) — expect: source /opt/ros/humble/setup.bash
# ---------------------------------------------------------------------------
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(CURL REQUIRED)

if(NOT TARGET yaml-cpp::yaml-cpp AND TARGET yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()

if(NOT TARGET kdl_parser::kdl_parser)
  add_library(kdl_parser::kdl_parser INTERFACE IMPORTED)
  target_link_libraries(kdl_parser::kdl_parser INTERFACE ${kdl_parser_LIBRARIES})
  target_include_directories(kdl_parser::kdl_parser INTERFACE ${kdl_parser_INCLUDE_DIRS})
endif()

# ---------------------------------------------------------------------------
# openarm_can_local
# ---------------------------------------------------------------------------
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
target_include_directories(openarm_can_local PUBLIC ${OPENARM_CAN_DIR}/include)

# ---------------------------------------------------------------------------
# openarm_sensor_lib
# ---------------------------------------------------------------------------
add_library(openarm_sensor_lib STATIC
    src_sensor/controller/cartesian_mapper.cpp
    src_sensor/controller/control.cpp
    src_sensor/controller/dynamics.cpp
    src_sensor/ik/ik_solver.cpp
    src_sensor/ik/dls_ik.cpp
    src_sensor/interpolator/joint_interpolator.cpp
    src_sensor/interpolator/linear_joint_interpolator.cpp
    src_sensor/safety/safety_filter.cpp
    src_sensor/hardware/teleop_hardware.cpp
    src_sensor/openarm_port/openarm_init.cpp
    src_sensor/io/pika_pose_ros.cpp
    src_sensor/io/tcp_relative_csv_writer.cpp
    src_sensor/health_check.cpp
    src_sensor/pose_health_monitor.cpp
)
target_include_directories(openarm_sensor_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include_sensor
    ${orocos_kdl_INCLUDE_DIRS}
)
target_link_libraries(openarm_sensor_lib PUBLIC
    openarm_can_local
    Eigen3::Eigen
    ${orocos_kdl_LIBRARIES}
    kdl_parser::kdl_parser
    urdf::urdf
    urdfdom::urdfdom_model
    yaml-cpp::yaml-cpp
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    Threads::Threads
)
target_compile_options(openarm_sensor_lib PRIVATE -O2 -Wall -Wextra)

# ---------------------------------------------------------------------------
# all_recorder_lib — cameras + gripper
# ---------------------------------------------------------------------------
add_library(all_recorder_lib STATIC
    src_all/config.cpp
    src_all/realsense_camera.cpp
    src_all/fisheye_camera.cpp
    src_all/recorder.cpp
    src_all/saver.cpp
    src_all/statistics.cpp
    src_all/utils.cpp
    src_all/gripper_protocol.cpp
    src_all/gripper_serial_io.cpp
    src_all/gripper_controller.cpp
    src_all/gripper_subsystem.cpp
)
target_include_directories(all_recorder_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include_all
)
target_link_libraries(all_recorder_lib PUBLIC
    realsense2::realsense2
    ${OpenCV_LIBS}
    yaml-cpp
    Boost::system
    Threads::Threads
)
if(nlohmann_json_FOUND)
  target_link_libraries(all_recorder_lib PUBLIC nlohmann_json::nlohmann_json)
elseif(NLOHMANN_JSON_FOUND)
  target_include_directories(all_recorder_lib PUBLIC ${NLOHMANN_JSON_INCLUDE_DIRS})
  target_link_libraries(all_recorder_lib PUBLIC ${NLOHMANN_JSON_LIBRARIES})
else()
  target_link_libraries(all_recorder_lib PUBLIC nlohmann_json::nlohmann_json)
endif()

# ---------------------------------------------------------------------------
# all_telop executable
# ---------------------------------------------------------------------------
add_executable(all_telop
    src/main.cpp
    src_sensor/app/teleop_session.cpp
)

target_include_directories(all_telop PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include_all
    ${CMAKE_CURRENT_SOURCE_DIR}/include_sensor
)

target_link_libraries(all_telop PRIVATE
    all_recorder_lib
    openarm_sensor_lib
    CURL::libcurl
)
# Ensure linker resolves curl from system dirs before any conda shadowing.
target_link_options(all_telop PRIVATE
    "LINKER:-rpath-link,/usr/lib/x86_64-linux-gnu"
)
# Explicit system curl when Conda puts a mismatched libcurl earlier on the path.
if(EXISTS "/usr/lib/x86_64-linux-gnu/libcurl.so")
  target_link_libraries(all_telop PRIVATE /usr/lib/x86_64-linux-gnu/libcurl.so)
endif()

target_compile_options(all_telop PRIVATE -O2 -Wall -Wextra)

install(TARGETS all_telop RUNTIME DESTINATION bin)
install(DIRECTORY config DESTINATION share/all_telop)
install(DIRECTORY assets DESTINATION share/all_telop)
