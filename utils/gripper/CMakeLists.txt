cmake_minimum_required(VERSION 3.14)
project(read_gripper_angle)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost REQUIRED COMPONENTS system)

# 自动查找或下载 nlohmann_json
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

add_executable(read_gripper_angle read_gripper_angle.cpp)
target_link_libraries(read_gripper_angle Boost::system)

if(nlohmann_json_FOUND)
    target_link_libraries(read_gripper_angle nlohmann_json::nlohmann_json)
elseif(NLOHMANN_JSON_FOUND)
    target_include_directories(read_gripper_angle PRIVATE ${NLOHMANN_JSON_INCLUDE_DIRS})
    target_link_libraries(read_gripper_angle ${NLOHMANN_JSON_LIBRARIES})
else()
    target_link_libraries(read_gripper_angle nlohmann_json::nlohmann_json)
endif()

target_compile_options(read_gripper_angle PRIVATE -O2 -Wall)