#pragma once

#include "health_check.h"
#include "pose_queue.h"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

// Pose health for pika_control:
//   1) Startup stationary check (settle + warmup + analyze_stationary)
//   2) After first 'p', frame-to-frame jump gate (continues after 'q', stops on exit)
class PoseHealthMonitor {
public:
    struct StaticOptions {
        double wait_first_sec = 15.0;
        double settle_sec = 1.0;
        double warmup_sec = 1.0;
        HealthThresholds thresholds;
    };

    PoseHealthMonitor() = default;

    // Called from the ROS pose callback thread. Returns false when an enabled
    // dynamic jump check rejects this sample for control.
    bool on_sample(const PoseSample& sample);

    // Block until stationary check finishes (or keep_running becomes false).
    // Returns true if stationary stats pass. On timeout / abort, returns false.
    bool run_static_check(std::atomic<bool>& keep_running, const StaticOptions& opt);

    // Enable jump-gate monitoring (call on first teleop 'p'; leave on after 'q').
    void enable_dynamic(bool on);

    bool dynamic_enabled() const { return dynamic_enabled_.load(); }
    bool static_finished() const { return static_finished_.load(); }
    bool static_ok() const { return static_ok_.load(); }

    void set_jump_thresholds(const JumpThresholds& th);

    static void print_static_fail_hint();

private:
    enum class StaticPhase {
        WaitingFirst,
        Settling,
        Collecting,
        Done,
    };

    bool accept_dynamic_sample(const PoseSample& sample);

    StaticOptions opt_{};
    JumpThresholds jump_th_{};

    std::mutex mu_;
    StaticPhase phase_{StaticPhase::WaitingFirst};
    std::chrono::steady_clock::time_point first_deadline_{};
    std::chrono::steady_clock::time_point settle_deadline_{};
    std::chrono::steady_clock::time_point collect_deadline_{};
    std::vector<PoseSample> collect_;

    std::atomic<bool> static_finished_{false};
    std::atomic<bool> static_ok_{false};
    std::atomic<bool> dynamic_enabled_{false};

    bool have_prev_jump_{false};
    PoseSample prev_jump_{};
    std::chrono::steady_clock::time_point last_jump_warn_{};
};
