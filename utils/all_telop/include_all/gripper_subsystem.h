#pragma once
#include "config.h"
#include "angle_queue.h"
#include "gripper_serial_io.h"
#include "gripper_controller.h"
#include <boost/asio.hpp>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// 门面：采集 / 控制 / 回执 三线程；对外提供 AngleQueue 供 Saver。
// session_active_ 非空时：跟随与 CSV 入队仅在 active==true 时进行。
class GripperSubsystem {
public:
    explicit GripperSubsystem(const GripperParams& params);
    ~GripperSubsystem();

    bool start();
    void stop();

    // Optional session gate (camera/teleop p/q). Null = always follow & enqueue.
    void setSessionActive(std::atomic<bool>* session_active) { session_active_ = session_active; }

    AngleQueue& sampleQueue() { return sample_queue_; }
    const std::vector<double>& timestamps() const { return timestamps_; }
    std::mutex& timestampsMutex() { return ts_mtx_; }

    int samplesPushed() const { return samples_pushed_.load(); }

private:
    void captureLoop();
    void controlLoop();
    void feedbackLoop();

    bool sessionIsActive() const {
        return session_active_ == nullptr || session_active_->load();
    }

    GripperParams params_;
    std::shared_ptr<GripperSerialIO> gripper_io_;
    std::unique_ptr<GripperController> controller_;

    boost::asio::io_context sensor_io_;
    std::unique_ptr<boost::asio::serial_port> sensor_port_;
    std::mutex sensor_mtx_;

    AngleQueue sample_queue_;
    std::mutex latest_mtx_;
    float latest_angle_ = 0.0f;
    bool has_latest_ = false;

    std::vector<double> timestamps_;
    std::mutex ts_mtx_;
    std::atomic<int> samples_pushed_{0};

    std::atomic<bool> running_{false};
    std::atomic<bool>* session_active_{nullptr};
    std::thread capture_thread_;
    std::thread control_thread_;
    std::thread feedback_thread_;
};
