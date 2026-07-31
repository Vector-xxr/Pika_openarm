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
};

Config loadConfig(const std::string& filename);