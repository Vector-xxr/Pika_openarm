#pragma once

#include "pose_queue.h"

#include <vector>

struct HealthStats {
    bool ok = false;
    std::size_t n = 0;
    double drift_m = 0.0;
    double drift_rad = 0.0;
    double max_speed = 0.0;
    double max_ang_speed = 0.0;
    double max_acc = 0.0;
};

struct HealthThresholds {
    double max_drift_m = 0.005;     // 5 mm
    double max_drift_rad = 0.03;    // ~1.7 deg
    double max_speed = 0.15;        // m/s
    double max_ang_speed = 1.0;     // rad/s
    double max_acc = 100.0;         // m/s^2 (relaxed for ~120Hz + stamp jitter)
    std::size_t min_samples = 20;
    double min_dt_sec = 0.005;
};

HealthStats analyze_stationary(const std::vector<PoseSample>& samples,
                               const HealthThresholds& th);

// Frame-to-frame jump / fly-point gate used during teleop (dynamic monitoring).
struct JumpThresholds {
    double max_delta_m = 0.12;      // consecutive pose distance [m]
    double max_speed = 3.0;         // m/s
    double max_ang_speed = 8.0;     // rad/s
    double max_gap_sec = 0.50;      // tracking stall / droptime [s]
    double min_dt_sec = 0.001;
};

struct JumpStats {
    bool ok = true;
    double delta_m = 0.0;
    double speed = 0.0;
    double ang_speed = 0.0;
    double dt = 0.0;
    bool gap = false;
};

JumpStats analyze_jump(const PoseSample& prev, const PoseSample& curr,
                       const JumpThresholds& th);
