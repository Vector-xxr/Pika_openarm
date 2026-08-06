cmake_minimum_required(VERSION 3.14)
project(all_recorder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
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
  )
  FetchContent_MakeAvailable(json)
endif()

include_directories(include)

add_executable(all_recorder
    src/main.cpp
    src/config.cpp
    src/realsense_camera.cpp
    src/fisheye_camera.cpp
    src/recorder.cpp
    src/saver.cpp
    src/statistics.cpp
    src/utils.cpp
    src/gripper_protocol.cpp
    src/gripper_serial_io.cpp
    src/gripper_controller.cpp
    src/gripper_subsystem.cpp
)

target_link_libraries(all_recorder
    realsense2::realsense2
    ${OpenCV_LIBS}
    yaml-cpp
    Boost::system
    pthread
)

if(nlohmann_json_FOUND)
  target_link_libraries(all_recorder nlohmann_json::nlohmann_json)
elseif(NLOHMANN_JSON_FOUND)
  target_include_directories(all_recorder PRIVATE ${NLOHMANN_JSON_INCLUDE_DIRS})
  target_link_libraries(all_recorder ${NLOHMANN_JSON_LIBRARIES})
else()
  target_link_libraries(all_recorder nlohmann_json::nlohmann_json)
endif()

install(TARGETS all_recorder DESTINATION bin)
