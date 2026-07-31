#pragma once
#include <opencv2/opencv.hpp>
#include <string>

class FisheyeCamera {
public:
    FisheyeCamera(const std::string& device, int width, int height, int fps, const std::string& fourcc);
    bool start();
    bool getFrame(cv::Mat& frame, double& timestamp);
    void stop();
    ~FisheyeCamera();

private:
    cv::VideoCapture cap_;
    std::string device_;
    int width_, height_, fps_;
    std::string fourcc_;
    bool opened_ = false;
};
