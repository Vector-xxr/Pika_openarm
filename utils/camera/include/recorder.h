#pragma once
#include "config.h"
#include "frame_queue.h"
#include <atomic>
#include <condition_variable>
#include <thread>
#include <mutex>
#include <fstream>

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

    // 队列
    FrameQueue color_queue_, depth_queue_, fisheye_queue_;

    // 保存线程相关
    std::thread color_saver_, depth_saver_, fisheye_saver_;
    std::atomic<int> color_saved_{0}, depth_saved_{0}, fisheye_saved_{0};
    std::atomic<bool> color_finished_{false}, depth_finished_{false}, fisheye_finished_{false};

    // 鱼眼采集线程
    std::thread fisheye_cap_thread_;
    std::atomic<bool> fisheye_done_{false};

    // 采集侧同步启动（C++17：atomic + condition_variable）
    static constexpr int kSyncParticipants = 2;
    std::mutex sync_mu_;
    std::condition_variable sync_cv_;
    std::atomic<int> sync_ready_count_{0};
    std::atomic<bool> capture_abort_{false};
    std::atomic<bool> sync_time_logged_{false};
    double sync_start_ts_{0.0};

    // 时间戳向量和互斥量
    std::vector<double> color_timestamps_, depth_timestamps_, fisheye_timestamps_;
    std::mutex ts_mutex_color_, ts_mutex_depth_, ts_mutex_fisheye_;

    // CSV 文件
    std::ofstream csv_color_, csv_depth_, csv_fisheye_;

    // 目录
    std::string base_dir_, color_dir_, depth_dir_, fisheye_dir_, csv_dir_;

    // 辅助函数
    bool createDirectories();
    void openCSVFiles();
    void closeCSVFiles();
    void writeCSVHeader(std::ofstream& csv);
    void writeCSVRow(std::ofstream& csv, int idx, double ts, double prev_ts);

    // 两边 warm-up 完成后同步；失败返回 false
    bool syncAfterWarmup(const char* side);
    void abortCaptureStart(const char* reason);

    // 保存线程启动
    void startSaverThreads();
    void startFisheyeCapture();
    void waitForAll();

    // 实际采集主循环（在 start() 中调用）
    void runRealSenseCapture();
};