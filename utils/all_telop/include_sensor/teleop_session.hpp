#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <pose_health_monitor.h>

// Encapsulates OpenArm Pika teleop (pose IO, static health, admin/follower threads).
// Keyboard is owned by the orchestrator; pass shared session atomics.
class TeleopSession {
public:
    TeleopSession();
    ~TeleopSession();

    TeleopSession(const TeleopSession&) = delete;
    TeleopSession& operator=(const TeleopSession&) = delete;

    // Initialize OpenArm, ROS pose subscriber, CSV writer. Does NOT run static check.
    // vive_csv_path: absolute or ~/... path for relative TCP CSV (episode/sensor/vive.csv).
    bool init(int argc, char** argv, const std::string& config_path,
              const std::string& vive_csv_path);

    // Blocking stationary check (failure prints hint, still returns true unless aborted).
    bool runStaticCheck();

    // Start admin + follower threads using shared session atomics.
    // session_start ↔ teleop_rezero, session_stop ↔ teleop_stop_req, session_active ↔ teleop_active
    void startControl(std::atomic<bool>* session_active, std::atomic<bool>* session_start,
                      std::atomic<bool>* session_stop);

    void stop();

    PoseHealthMonitor* poseHealth() { return pose_health_.get(); }
    bool ready() const { return ready_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    std::unique_ptr<PoseHealthMonitor> pose_health_;
    bool ready_ = false;
    bool ros_initialized_ = false;
};
