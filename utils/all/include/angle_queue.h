#pragma once
#include <condition_variable>
#include <mutex>
#include <queue>
#include <atomic>

struct AngleSample {
    double timestamp = 0.0;
    float angle = 0.0f;
    AngleSample() = default;
    AngleSample(double ts, float a) : timestamp(ts), angle(a) {}
};

class AngleQueue {
public:
    void push(const AngleSample& sample) {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            queue_.push(sample);
        }
        cv_.notify_one();
    }

    // 返回 false 表示结束且队列空
    bool pop(AngleSample& sample) {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this] { return done_.load() || !queue_.empty(); });
        if (queue_.empty()) return false;
        sample = queue_.front();
        queue_.pop();
        return true;
    }

    void setDone() {
        done_.store(true);
        cv_.notify_all();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.size();
    }

private:
    mutable std::mutex mtx_;
    std::condition_variable cv_;
    std::queue<AngleSample> queue_;
    std::atomic<bool> done_{false};
};
