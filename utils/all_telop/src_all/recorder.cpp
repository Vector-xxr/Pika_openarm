#include "recorder.h"
#include "realsense_camera.h"
#include "fisheye_camera.h"
#include "saver.h"
#include "statistics.h"
#include "utils.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <unistd.h>

Recorder::Recorder(const Config& cfg) : cfg_(cfg) {
    // duration_seconds <= 0 → unlimited until keep_running clears
    if (cfg.general.duration_seconds > 0) {
        max_frames_ = cfg.general.duration_seconds * cfg.general.fps_target;
    } else {
        max_frames_ = 0;
    }
}

bool Recorder::initialize() {
    episode_name_ = cfg_.general.episode;
    if (episode_name_ == "auto") {
        episode_name_ = get_next_episode(cfg_.general.base_data_dir);
        std::cout << "自动生成 episode: " << episode_name_ << std::endl;
    } else {
        std::cout << "使用指定 episode: " << episode_name_ << std::endl;
    }

    base_dir_ = cfg_.general.base_data_dir + "/" + episode_name_;
    color_dir_ = base_dir_ + "/color";
    depth_dir_ = base_dir_ + "/depth";
    fisheye_dir_ = base_dir_ + "/fisheye";
    csv_dir_ = base_dir_ + "/csv";
    gripper_dir_ = base_dir_ + "/gripper";
    sensor_dir_ = base_dir_ + "/sensor";

    if (!createDirectories()) return false;

    std::cout << "保存根目录: " << base_dir_ << std::endl;
    std::cout << "CSV 目录: " << csv_dir_ << std::endl;
    std::cout << "Sensor 目录: " << sensor_dir_ << std::endl;
    if (cfg_.gripper.enabled) {
        std::cout << "Gripper 目录: " << gripper_dir_ << std::endl;
    }
    if (max_frames_ > 0) {
        std::cout << "最大采集帧数: " << max_frames_ << " (duration*fps)" << std::endl;
    } else {
        std::cout << "采集帧数: 无上限（按 p/q 启停，Ctrl+C 结束）" << std::endl;
    }

    openCSVFiles();
    return true;
}

bool Recorder::createDirectories() {
    std::vector<std::string> dirs = {base_dir_, color_dir_, depth_dir_, fisheye_dir_, csv_dir_,
                                     sensor_dir_};
    if (cfg_.gripper.enabled) dirs.push_back(gripper_dir_);
    for (const auto& d : dirs) {
        if (system(("mkdir -p " + d).c_str()) != 0) {
            std::cerr << "创建目录失败: " << d << std::endl;
            return false;
        }
    }
    return true;
}

void Recorder::openCSVFiles() {
    csv_color_.open(csv_dir_ + "/color.csv");
    csv_depth_.open(csv_dir_ + "/depth.csv");
    csv_fisheye_.open(csv_dir_ + "/fisheye.csv");
    writeCSVHeader(csv_color_);
    writeCSVHeader(csv_depth_);
    writeCSVHeader(csv_fisheye_);
    if (cfg_.gripper.enabled) {
        csv_gripper_.open(gripper_dir_ + "/angle.csv");
        writeGripperCSVHeader(csv_gripper_);
    }
}

void Recorder::writeCSVHeader(std::ofstream& csv) {
    csv << "Frame_Index,Timestamp(s),Delay(s)\n";
}

void Recorder::writeGripperCSVHeader(std::ofstream& csv) {
    csv << "Frame_Index,Timestamp(s),Delay(s),Angle(rad)\n";
}

void Recorder::writeCSVRow(std::ofstream& csv, int idx, double ts, double prev_ts) {
    if (idx == 0)
        csv << idx << "," << std::fixed << std::setprecision(9) << ts << ",N/A\n";
    else
        csv << idx << "," << ts << "," << (ts - prev_ts) << "\n";
}

void Recorder::closeCSVFiles() {
    if (csv_color_.is_open()) csv_color_.close();
    if (csv_depth_.is_open()) csv_depth_.close();
    if (csv_fisheye_.is_open()) csv_fisheye_.close();
    if (csv_gripper_.is_open()) csv_gripper_.close();
}

void Recorder::abortCaptureStart(const char* reason) {
    std::cerr << "采集同步中止: " << reason << std::endl;
    capture_abort_.store(true);
    cameras_ready_.store(false);
    sync_cv_.notify_all();
}

void Recorder::tryStartGripperAfterSync() {
    if (!cfg_.gripper.enabled) return;

    bool expected = false;
    if (!gripper_started_.compare_exchange_strong(expected, true)) return;

    gripper_ = std::make_unique<GripperSubsystem>(cfg_.gripper);
    gripper_->setSessionActive(session_active_);
    if (!gripper_->start()) {
        std::cerr << "Gripper 启动失败，继续相机采集（无夹爪跟随）" << std::endl;
        gripper_.reset();
        gripper_finished_ = true;
        return;
    }

    gripper_saver_ = std::thread(save_gripper_thread_func,
                                 std::ref(gripper_->sampleQueue()),
                                 std::ref(csv_gripper_),
                                 std::ref(gripper_saved_),
                                 std::ref(gripper_finished_));
}

void Recorder::stopGripper() {
    if (!gripper_) return;
    gripper_->stop();
}

bool Recorder::syncAfterWarmup(const char* side) {
    if (capture_abort_.load()) return false;

    const int n = sync_ready_count_.fetch_add(1) + 1;
    std::cout << "[" << side << "] 预热完成 (" << n << "/" << kSyncParticipants
              << ")，等待同步..." << std::endl;
    sync_cv_.notify_all();

    {
        std::unique_lock<std::mutex> lock(sync_mu_);
        sync_cv_.wait(lock, [this] {
            return capture_abort_.load() ||
                   sync_ready_count_.load() >= kSyncParticipants ||
                   !stillRunning();
        });
    }

    if (!stillRunning() || capture_abort_.load()) {
        std::cerr << "[" << side << "] 同步启动失败（中止）" << std::endl;
        return false;
    }

    bool expected = false;
    if (sync_time_logged_.compare_exchange_strong(expected, true)) {
        sync_start_ts_ = get_system_timestamp_seconds();
        std::cout << "两侧预热完成，等待键盘 p 同步开启采集 t_ready=" << std::fixed
                  << std::setprecision(9) << sync_start_ts_ << std::endl;
        tryStartGripperAfterSync();
        cameras_ready_.store(true);
        std::cout << "[recorder] cameras_ready=true（夹爪已 open/enable；跟随/写盘需 session_active）"
                  << std::endl;
    }
    return true;
}

void Recorder::startSession() {
    if (!cfg_.gripper.enabled) {
        gripper_finished_ = true;
    }

    startSaverThreads();
    startFisheyeCapture();
    runRealSenseCapture();

    while (!fisheye_done_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (fisheye_cap_thread_.joinable()) fisheye_cap_thread_.join();

    stopGripper();
    if (!gripper_saver_.joinable()) {
        gripper_finished_ = true;
    }

    color_queue_.setDone();
    depth_queue_.setDone();
    fisheye_queue_.setDone();

    waitForAll();
    closeCSVFiles();
}

void Recorder::startSaverThreads() {
    color_saver_ = std::thread(save_thread_func, std::ref(color_queue_), color_dir_, "color",
                               cfg_.saver.color_format, cfg_.saver.color_quality,
                               std::ref(color_saved_), std::ref(color_finished_));
    depth_saver_ = std::thread(save_thread_func, std::ref(depth_queue_), depth_dir_, "depth",
                               cfg_.saver.depth_format, cfg_.saver.depth_quality,
                               std::ref(depth_saved_), std::ref(depth_finished_));
    fisheye_saver_ = std::thread(save_thread_func, std::ref(fisheye_queue_), fisheye_dir_, "fisheye",
                                 cfg_.saver.fisheye_format, cfg_.saver.fisheye_quality,
                                 std::ref(fisheye_saved_), std::ref(fisheye_finished_));
}

void Recorder::startFisheyeCapture() {
    fisheye_cap_thread_ = std::thread([this]() {
        const int WARMUP = cfg_.general.warmup_frames;
        FisheyeCamera fisheye(cfg_.fisheye.device, cfg_.fisheye.width, cfg_.fisheye.height,
                              cfg_.fisheye.fps, cfg_.fisheye.fourcc);
        if (!fisheye.start()) {
            abortCaptureStart("鱼眼摄像头启动失败");
            fisheye_done_ = true;
            return;
        }
        for (int i = 0; i < WARMUP && stillRunning(); ++i) {
            cv::Mat dummy;
            double ts;
            fisheye.getFrame(dummy, ts);
        }

        if (!syncAfterWarmup("鱼眼")) {
            fisheye.stop();
            fisheye_done_ = true;
            return;
        }

        {
            cv::Mat dummy;
            double ts;
            fisheye.getFrame(dummy, ts);
        }

        std::cout << "鱼眼待命：session 激活时写入，pause 时丢弃帧" << std::endl;

        int idx = 0;
        double prev_ts = 0.0;
        while (stillRunning()) {
            if (max_frames_ > 0 && idx >= max_frames_) break;

            cv::Mat frame;
            double ts;
            if (!fisheye.getFrame(frame, ts)) {
                std::cerr << "鱼眼读取失败，停止采集" << std::endl;
                break;
            }

            if (!sessionIsActive()) {
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(ts_mutex_fisheye_);
                fisheye_timestamps_.push_back(ts);
            }
            {
                std::lock_guard<std::mutex> lock(csv_mu_fisheye_);
                writeCSVRow(csv_fisheye_, idx, ts, prev_ts);
            }
            prev_ts = ts;
            fisheye_queue_.push(FrameData(ts, frame));
            ++idx;
            if (idx % 30 == 0) {
                std::cout << "鱼眼已采集 " << idx << " 帧" << std::endl;
            }
        }
        fisheye.stop();
        fisheye_done_ = true;
    });
}

void Recorder::runRealSenseCapture() {
    const int WARMUP = cfg_.general.warmup_frames;
    RealSenseCamera realsense(cfg_.realsense.color.width, cfg_.realsense.color.height,
                              cfg_.realsense.color.fps, cfg_.realsense.depth.width,
                              cfg_.realsense.depth.height, cfg_.realsense.depth.fps,
                              cfg_.realsense.serial);
    if (!realsense.start()) {
        abortCaptureStart("RealSense 启动失败");
        return;
    }

    std::cout << "RealSense 预热中，丢弃前 " << WARMUP << " 帧..." << std::endl;
    for (int i = 0; i < WARMUP && stillRunning(); ++i) {
        cv::Mat color, depth;
        double ts;
        realsense.getFrames(color, depth, ts);
    }

    if (!syncAfterWarmup("RealSense")) {
        realsense.stop();
        return;
    }

    {
        cv::Mat color, depth;
        double ts;
        realsense.getFrames(color, depth, ts);
    }

    std::cout << "RealSense 待命：session 激活时写入，pause 时丢弃帧" << std::endl;
    auto start_time = std::chrono::steady_clock::now();

    int frame_idx = 0;
    double prev_color_ts = 0.0;
    double prev_depth_ts = 0.0;

    while (stillRunning()) {
        if (max_frames_ > 0 && frame_idx >= max_frames_) {
            std::cout << "已达最大帧数限制，结束 RealSense 采集" << std::endl;
            break;
        }

        cv::Mat color, depth;
        double ts;
        if (!realsense.getFrames(color, depth, ts)) {
            std::cerr << "RealSense 采集帧失败" << std::endl;
            break;
        }

        if (!sessionIsActive()) {
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(ts_mutex_color_);
            color_timestamps_.push_back(ts);
        }
        {
            std::lock_guard<std::mutex> lock(csv_mu_color_);
            writeCSVRow(csv_color_, frame_idx, ts, prev_color_ts);
        }
        prev_color_ts = ts;
        color_queue_.push(FrameData(ts, color));

        {
            std::lock_guard<std::mutex> lock(ts_mutex_depth_);
            depth_timestamps_.push_back(ts);
        }
        {
            std::lock_guard<std::mutex> lock(csv_mu_depth_);
            writeCSVRow(csv_depth_, frame_idx, ts, prev_depth_ts);
        }
        prev_depth_ts = ts;
        depth_queue_.push(FrameData(ts, depth));

        ++frame_idx;
        if (frame_idx % 30 == 0) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time).count();
            std::cout << "已采集 " << frame_idx
                      << " 帧, 彩色队列: " << color_queue_.size()
                      << ", 深度: " << depth_queue_.size()
                      << ", 鱼眼: " << fisheye_queue_.size()
                      << ", 已保存彩色: " << color_saved_
                      << ", 深度: " << depth_saved_
                      << ", 鱼眼: " << fisheye_saved_;
            if (gripper_) {
                std::cout << ", 夹爪样本: " << gripper_->samplesPushed()
                          << ", 已写 angle: " << gripper_saved_;
            }
            std::cout << ", 耗时: " << std::fixed << std::setprecision(1) << elapsed << "s"
                      << std::endl;
        }
    }
    realsense.stop();
}

void Recorder::waitForAll() {
    std::cout << "采集结束，等待所有帧保存完毕..." << std::endl;
    while (!color_finished_.load() || !depth_finished_.load() || !fisheye_finished_.load() ||
           !gripper_finished_.load() ||
           color_queue_.size() > 0 || depth_queue_.size() > 0 || fisheye_queue_.size() > 0 ||
           (gripper_ && gripper_->sampleQueue().size() > 0)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "彩色队列剩余: " << color_queue_.size()
                  << ", 深度队列剩余: " << depth_queue_.size()
                  << ", 鱼眼队列剩余: " << fisheye_queue_.size()
                  << "  已保存彩色: " << color_saved_
                  << ", 深度: " << depth_saved_
                  << ", 鱼眼: " << fisheye_saved_;
        if (cfg_.gripper.enabled) {
            std::cout << ", 夹爪: " << gripper_saved_;
        }
        std::cout << "\r" << std::flush;
    }
    std::cout << std::endl;

    if (color_saver_.joinable()) color_saver_.join();
    if (depth_saver_.joinable()) depth_saver_.join();
    if (fisheye_saver_.joinable()) fisheye_saver_.join();
    if (gripper_saver_.joinable()) gripper_saver_.join();
}

void Recorder::printStatistics() {
    {
        std::lock_guard<std::mutex> lock(ts_mutex_color_);
        ::print_statistics(color_timestamps_, "彩色");
    }
    {
        std::lock_guard<std::mutex> lock(ts_mutex_depth_);
        ::print_statistics(depth_timestamps_, "深度");
    }
    {
        std::lock_guard<std::mutex> lock(ts_mutex_fisheye_);
        ::print_statistics(fisheye_timestamps_, "鱼眼");
    }
    if (gripper_) {
        std::lock_guard<std::mutex> lock(gripper_->timestampsMutex());
        ::print_statistics(gripper_->timestamps(), "gripper");
    }

    std::cout << "\n所有数据已保存至: " << base_dir_ << std::endl;
    std::cout << "彩色图像: " << color_dir_ << "\n深度图像: " << depth_dir_
              << "\n鱼眼图像: " << fisheye_dir_ << std::endl;
    std::cout << "CSV 时间戳文件: " << csv_dir_ << " (color.csv, depth.csv, fisheye.csv)"
              << std::endl;
    if (cfg_.gripper.enabled) {
        std::cout << "Gripper 角度: " << gripper_dir_ << "/angle.csv" << std::endl;
    }
    std::cout << "Sensor TCP: " << viveCsvPath() << std::endl;
    std::cout << "成功保存彩色: " << color_saved_ << std::endl;
    std::cout << "成功保存深度: " << depth_saved_ << std::endl;
    std::cout << "成功保存鱼眼: " << fisheye_saved_ << std::endl;
    if (cfg_.gripper.enabled) {
        std::cout << "成功保存夹爪角度样本: " << gripper_saved_ << std::endl;
    }
}

Recorder::~Recorder() {
    stopGripper();
    if (fisheye_cap_thread_.joinable()) fisheye_cap_thread_.join();
    if (color_saver_.joinable()) color_saver_.join();
    if (depth_saver_.joinable()) depth_saver_.join();
    if (fisheye_saver_.joinable()) fisheye_saver_.join();
    if (gripper_saver_.joinable()) gripper_saver_.join();
    closeCSVFiles();
}
