#pragma once

#include <array>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

struct PoseSample {
    double t = 0.0;                 // seconds (prefer header stamp)
    std::array<double, 3> pos{};    // xyz meters
    std::array<double, 4> rot{};    // wxyz quaternion
};

// Bounded thread-safe FIFO for pose samples.
class PoseQueue {
public:
    explicit PoseQueue(std::size_t capacity = 2048) : capacity_(capacity) {}

    void push(const PoseSample& s) {
        std::lock_guard<std::mutex> lock(mu_);
        if (capacity_ > 0 && q_.size() >= capacity_) {
            q_.pop_front();
            ++dropped_;
        }
        q_.push_back(s);
    }

    std::optional<PoseSample> try_pop() {
        std::lock_guard<std::mutex> lock(mu_);
        if (q_.empty()) {
            return std::nullopt;
        }
        PoseSample s = q_.front();
        q_.pop_front();
        return s;
    }

    std::deque<PoseSample> snapshot() const {
        std::lock_guard<std::mutex> lock(mu_);
        return q_;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mu_);
        q_.clear();
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mu_);
        return q_.size();
    }

    std::size_t dropped() const {
        std::lock_guard<std::mutex> lock(mu_);
        return dropped_;
    }

private:
    mutable std::mutex mu_;
    std::deque<PoseSample> q_;
    std::size_t capacity_;
    std::size_t dropped_ = 0;
};
