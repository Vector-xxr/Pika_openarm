#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <opencv2/opencv.hpp>

struct FrameData {
    double timestamp;
    cv::Mat image;
    FrameData() : timestamp(0) {}
    FrameData(double ts, cv::Mat img) : timestamp(ts), image(img.clone()) {}
};

class FrameQueue {
public:
    void push(const FrameData& frame) {
        std::unique_lock<std::mutex> lock(mtx_);
        queue_.push(frame);
        cond_.notify_one();
    }

    bool pop(FrameData& frame) {
        std::unique_lock<std::mutex> lock(mtx_);
        cond_.wait(lock, [this] { return !queue_.empty() || done_; });
        if (queue_.empty()) return false;
        frame = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    void setDone() {
        std::lock_guard<std::mutex> lock(mtx_);
        done_ = true;
        cond_.notify_all();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.size();
    }

private:
    mutable std::mutex mtx_;
    std::condition_variable cond_;
    std::queue<FrameData> queue_;
    bool done_ = false;
};