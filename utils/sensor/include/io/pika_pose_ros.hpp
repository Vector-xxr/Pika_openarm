#pragma once

#include <pose_queue.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

class PoseHealthMonitor;

// Owns a dedicated rclcpp node that subscribes to a geometry_msgs/PoseStamped
// topic and pushes every message into a caller-provided PoseQueue as a
// PoseSample (position + wxyz quaternion). The node spins on its own
// background thread (MultiThreadedExecutor) so it can coexist with the
// non-ROS realtime control loops in the same process.
//
// Requires rclcpp::init() to have been called by the caller before start().
class PikaPoseRosIO {
public:
    PikaPoseRosIO() = default;
    ~PikaPoseRosIO();

    PikaPoseRosIO(const PikaPoseRosIO&) = delete;
    PikaPoseRosIO& operator=(const PikaPoseRosIO&) = delete;

    // Starts (or restarts, if already running) subscribing to `topic`, pushing
    // samples into `queue` (not owned, must outlive this object or stop()).
    // Optional `health` receives every sample for static/dynamic pose checks.
    bool start(const std::string& topic, PoseQueue* queue,
               PoseHealthMonitor* health = nullptr);

    // Stops the spin thread and destroys the node. Safe to call repeatedly.
    void stop();

    bool running() const { return running_.load(); }

private:
    class Impl;
    std::shared_ptr<Impl> impl_;
    std::thread spin_thread_;
    std::atomic<bool> running_{false};
};
