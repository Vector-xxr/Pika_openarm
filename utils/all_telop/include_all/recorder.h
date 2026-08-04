#pragma once
#include "config.h"
#include "frame_queue.h"
#include "gripper_subsystem.h"
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Session-gated multi-camera recorder + optional gripper follow.
// Flow: initialize() → startSession() (warmup + barrier → cameras_ready)
//       → while keep_running: record only when session_active
//       → shutdown joins / closes.
class Recorder {
public:
    Recorder(const Config& cfg);
    bool initialize();

    // Shared with teleop / keyboard. Must be set before startSession().
    void setSessionActive(std::atomic<bool>* session_active) { session_active_ = session_active; }
    void setKeepRunning(std::atomic<bool>* keep_running) { keep_running_ = keep_running; }

    // Launch capture threads (blocking on caller until shutdown):
    // warmup → barrier → start gripper → idle/record until !*keep_running_.
    void startSession();

    bool camerasReady() const { return cameras_ready_.load(); }
    bool camerasFailed() const { return capture_abort_.load(); }

    const std::string& baseDir() const { return base_dir_; }
    const std::string& episodeName() const { return episode_name_; }
    std::string sensorDir() const { return base_dir_ + "/sensor"; }
    std::string viveCsvPath() const { return sensorDir() + "/vive.csv"; }

    // active_seconds: wall time spent with session_active (p..q), excluding pauses.
    void printStatistics(double active_seconds = 0.0);
    ~Recorder();

private:
    Config cfg_;
    int max_frames_;  // 0 = unlimited (Ctrl+C / keep_running)
    std::string episode_name_;

    FrameQueue color_queue_, depth_queue_, fisheye_queue_;

    std::thread color_saver_, depth_saver_, fisheye_saver_, gripper_saver_;
    std::atomic<int> color_saved_{0}, depth_saved_{0}, fisheye_saved_{0}, gripper_saved_{0};
    std::atomic<bool> color_finished_{false}, depth_finished_{false},
        fisheye_finished_{false}, gripper_finished_{false};

    std::thread fisheye_cap_thread_;
    std::atomic<bool> fisheye_done_{false};

    static constexpr int kSyncParticipants = 2;
    std::mutex sync_mu_;
    std::condition_variable sync_cv_;
    std::atomic<int> sync_ready_count_{0};
    std::atomic<bool> capture_abort_{false};
    std::atomic<bool> sync_time_logged_{false};
    std::atomic<bool> cameras_ready_{false};
    double sync_start_ts_{0.0};
    std::atomic<bool> gripper_started_{false};

    std::atomic<bool>* session_active_{nullptr};
    std::atomic<bool>* keep_running_{nullptr};

    std::vector<double> color_timestamps_, depth_timestamps_, fisheye_timestamps_;
    std::mutex ts_mutex_color_, ts_mutex_depth_, ts_mutex_fisheye_;

    std::ofstream csv_color_, csv_depth_, csv_fisheye_, csv_gripper_;
    std::mutex csv_mu_color_, csv_mu_depth_, csv_mu_fisheye_;

    std::string base_dir_, color_dir_, depth_dir_, fisheye_dir_, csv_dir_, gripper_dir_,
        sensor_dir_;

    std::unique_ptr<GripperSubsystem> gripper_;

    bool createDirectories();
    void openCSVFiles();
    void closeCSVFiles();
    void writeCSVHeader(std::ofstream& csv);
    void writeCSVRow(std::ofstream& csv, int idx, double ts, double prev_ts);
    void writeGripperCSVHeader(std::ofstream& csv);

    bool syncAfterWarmup(const char* side);
    void abortCaptureStart(const char* reason);
    void tryStartGripperAfterSync();
    void stopGripper();

    bool stillRunning() const {
        return keep_running_ == nullptr || keep_running_->load();
    }
    bool sessionIsActive() const {
        return session_active_ != nullptr && session_active_->load();
    }

    void startSaverThreads();
    void startFisheyeCapture();
    void waitForAll();
    void runRealSenseCapture();
};
