#pragma once
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class RealSenseCamera {
public:
    RealSenseCamera(int color_width, int color_height, int color_fps,
                    int depth_width, int depth_height, int depth_fps,
                    const std::string& serial = "");
    bool start();
    bool getFrames(cv::Mat& color, cv::Mat& depth, double& timestamp);
    void stop();
    ~RealSenseCamera();

private:
    rs2::pipeline pipe_;
    rs2::config cfg_;
    std::string serial_;
    bool started_ = false;
};
