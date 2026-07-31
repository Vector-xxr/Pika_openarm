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

class Recorder {
public:
    Recorder(const Config& cfg);
    bool initialize();
    void start();
    void printStatistics();
    ~Recorder();

private:
    Config cfg_;
    int total_frames_;

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
    double sync_start_ts_{0.0};
    std::atomic<bool> gripper_started_{false};

    std::vector<double> color_timestamps_, depth_timestamps_, fisheye_timestamps_;
    std::mutex ts_mutex_color_, ts_mutex_depth_, ts_mutex_fisheye_;

    std::ofstream csv_color_, csv_depth_, csv_fisheye_, csv_gripper_;

    std::string base_dir_, color_dir_, depth_dir_, fisheye_dir_, csv_dir_, gripper_dir_;

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

    void startSaverThreads();
    void startFisheyeCapture();
    void waitForAll();
    void runRealSenseCapture();
};
