#include "realsense_camera.h"
#include "utils.h"
#include <iostream>

RealSenseCamera::RealSenseCamera(int color_width, int color_height, int color_fps,
                                 int depth_width, int depth_height, int depth_fps,
                                 const std::string& serial)
    : serial_(serial) {
    cfg_.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_BGR8, color_fps);
    cfg_.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, depth_fps);
}

bool RealSenseCamera::start() {
    try {
        if (!serial_.empty()) {
            rs2::context ctx;
            auto devices = ctx.query_devices();
            bool found = false;
            for (auto&& dev : devices) {
                if (dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == serial_) {
                    cfg_.enable_device(serial_);
                    found = true;
                    break;
                }
            }
            if (!found) {
                std::cerr << "未找到序列号为 " << serial_ << " 的 RealSense 设备" << std::endl;
                return false;
            }
        }
        pipe_.start(cfg_);
        started_ = true;
        return true;
    } catch (const rs2::error& e) {
        std::cerr << "RealSense start error: " << e.what() << std::endl;
        return false;
    }
}

bool RealSenseCamera::getFrames(cv::Mat& color, cv::Mat& depth, double& timestamp) {
    if (!started_) return false;
    try {
        auto frames = pipe_.wait_for_frames();
        timestamp = get_system_timestamp_seconds();

        auto color_frame = frames.get_color_frame();
        auto depth_frame = frames.get_depth_frame();
        if (!color_frame || !depth_frame) return false;

        color = cv::Mat(color_frame.get_height(), color_frame.get_width(),
                        CV_8UC3, (void*)color_frame.get_data()).clone();
        depth = cv::Mat(depth_frame.get_height(), depth_frame.get_width(),
                        CV_16UC1, (void*)depth_frame.get_data()).clone();
        return true;
    } catch (const rs2::error& e) {
        std::cerr << "RealSense getFrames error: " << e.what() << std::endl;
        return false;
    }
}

void RealSenseCamera::stop() {
    if (started_) {
        pipe_.stop();
        started_ = false;
    }
}

RealSenseCamera::~RealSenseCamera() {
    stop();
}
