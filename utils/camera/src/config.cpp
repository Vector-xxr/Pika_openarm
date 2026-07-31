#include "config.h"
#include <iostream>
#include <cstdlib>

Config loadConfig(const std::string& filename) {
    YAML::Node node = YAML::LoadFile(filename);
    Config cfg;

    // General
    cfg.general.base_data_dir = node["general"]["base_data_dir"].as<std::string>();
    cfg.general.episode = node["general"]["episode"].as<std::string>();
    cfg.general.duration_seconds = node["general"]["duration_seconds"].as<int>();
    cfg.general.warmup_frames = node["general"]["warmup_frames"].as<int>();
    cfg.general.fps_target = node["general"]["fps_target"].as<int>();

    // Realsense
    cfg.realsense.serial = node["realsense"]["serial"].as<std::string>("");
    cfg.realsense.color.width = node["realsense"]["color"]["width"].as<int>();
    cfg.realsense.color.height = node["realsense"]["color"]["height"].as<int>();
    cfg.realsense.color.fps = node["realsense"]["color"]["fps"].as<int>();
    cfg.realsense.depth.width = node["realsense"]["depth"]["width"].as<int>();
    cfg.realsense.depth.height = node["realsense"]["depth"]["height"].as<int>();
    cfg.realsense.depth.fps = node["realsense"]["depth"]["fps"].as<int>();

    // Fisheye
    cfg.fisheye.device = node["fisheye"]["device"].as<std::string>();
    cfg.fisheye.width = node["fisheye"]["width"].as<int>();
    cfg.fisheye.height = node["fisheye"]["height"].as<int>();
    cfg.fisheye.fps = node["fisheye"]["fps"].as<int>();
    cfg.fisheye.fourcc = node["fisheye"]["fourcc"].as<std::string>("MJPG");

    // Saver
    cfg.saver.color_format = node["saver"]["color_format"].as<std::string>("png");
    cfg.saver.depth_format = node["saver"]["depth_format"].as<std::string>("png");
    cfg.saver.fisheye_format = node["saver"]["fisheye_format"].as<std::string>("png");
    cfg.saver.color_quality = node["saver"]["color_quality"].as<int>(90);
    cfg.saver.depth_quality = node["saver"]["depth_quality"].as<int>(100);
    cfg.saver.fisheye_quality = node["saver"]["fisheye_quality"].as<int>(90);

    // 展开 ~
    std::string& home = cfg.general.base_data_dir;
    if (home.find("~") == 0) {
        const char* home_env = std::getenv("HOME");
        if (home_env) home.replace(0, 1, home_env);
    }
    return cfg;
}