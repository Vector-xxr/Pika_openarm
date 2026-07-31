cmake_minimum_required(VERSION 3.14)
project(data_recorder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_executable(data_recorder
    src/main.cpp
    src/config.cpp
    src/realsense_camera.cpp
    src/fisheye_camera.cpp
    src/recorder.cpp
    src/saver.cpp
    src/statistics.cpp
    src/utils.cpp
)

target_link_libraries(data_recorder
    realsense2::realsense2
    ${OpenCV_LIBS}
    yaml-cpp
)

# 可选安装
install(TARGETS data_recorder DESTINATION bin)