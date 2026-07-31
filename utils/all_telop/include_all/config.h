#pragma once
#include <string>
#include <yaml-cpp/yaml.h>

struct CameraParams {
    int width;
    int height;
    int fps;
};

struct RealsenseParams {
    CameraParams color;
    CameraParams depth;
    std::string serial;    // 设备序列号，空则自动选择第一个
};

struct FisheyeParams {
    std::string device;
    int width;
    int height;
    int fps;
    std::string fourcc;      // "MJPG" 或 "YUYV"
};

struct SaverParams {
    std::string color_format;
    std::string depth_format;
    std::string fisheye_format;
    int color_quality;
    int depth_quality;
    int fisheye_quality;
};

struct GripperParams {
    bool enabled = true;
    std::string sensor_port = "/dev/ttyUSB50";
    std::string gripper_port = "/dev/ttyUSB60";
    unsigned int baud_rate = 460800;
    float effort_mA = 1000.0f;
    float velocity = 20.0f;
    bool mit_mode = true;
    float ctrl_rate = 50.0f;
    float capture_rate = 50.0f;  // CSV/统计入队 Hz；0=不限频（有包即记）
};

struct GeneralParams {
    std::string base_data_dir;
    std::string episode;
    int duration_seconds;
    int warmup_frames;
    int fps_target;
};

struct Config {
    GeneralParams general;
    RealsenseParams realsense;
    FisheyeParams fisheye;
    SaverParams saver;
    GripperParams gripper;
};

Config loadConfig(const std::string& filename);