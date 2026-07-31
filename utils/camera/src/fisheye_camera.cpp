#include "fisheye_camera.h"
#include "utils.h"
#include <iostream>

FisheyeCamera::FisheyeCamera(const std::string& device, int width, int height, int fps, const std::string& fourcc)
    : device_(device), width_(width), height_(height), fps_(fps), fourcc_(fourcc) {}

bool FisheyeCamera::start() {
    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        std::cerr << "无法打开鱼眼摄像头: " << device_ << std::endl;
        return false;
    }

    // 设置像素格式（如果配置了）
    if (!fourcc_.empty()) {
        int fourcc_code = 0;
        if (fourcc_ == "MJPG") {
            fourcc_code = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        } else if (fourcc_ == "YUYV") {
            fourcc_code = cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V');
        } else {
            std::cerr << "不支持的 FOURCC 格式: " << fourcc_ << "，忽略" << std::endl;
        }
        if (fourcc_code != 0) {
            cap_.set(cv::CAP_PROP_FOURCC, fourcc_code);
        }
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);
    opened_ = true;
    return true;
}

bool FisheyeCamera::getFrame(cv::Mat& frame, double& timestamp) {
    if (!opened_) return false;
    if (!cap_.read(frame)) return false;
    timestamp = get_system_timestamp_seconds();
    return true;
}

void FisheyeCamera::stop() {
    if (opened_) {
        cap_.release();
        opened_ = false;
    }
}

FisheyeCamera::~FisheyeCamera() {
    stop();
}
