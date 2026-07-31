#include "gripper_subsystem.h"
#include "gripper_protocol.h"
#include "utils.h"
#include <nlohmann/json.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <chrono>
#include <cmath>

GripperSubsystem::GripperSubsystem(const GripperParams& params) : params_(params) {}

GripperSubsystem::~GripperSubsystem() {
    stop();
}

bool GripperSubsystem::start() {
    if (running_.load()) return true;

    gripper_io_ = std::make_shared<GripperSerialIO>();
    if (!gripper_io_->open(params_.gripper_port, params_.baud_rate)) {
        std::cerr << "GripperSubsystem: open gripper port failed" << std::endl;
        return false;
    }

    try {
        sensor_port_ = std::make_unique<boost::asio::serial_port>(sensor_io_, params_.sensor_port);
        using boost::asio::serial_port;
        sensor_port_->set_option(serial_port::baud_rate(params_.baud_rate));
        sensor_port_->set_option(serial_port::character_size(8));
        sensor_port_->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sensor_port_->set_option(serial_port::parity(serial_port::parity::none));
        sensor_port_->set_option(serial_port::flow_control(serial_port::flow_control::none));
    } catch (const std::exception& e) {
        std::cerr << "GripperSubsystem: open sensor port failed: " << e.what() << std::endl;
        gripper_io_->close();
        return false;
    }

    controller_ = std::make_unique<GripperController>(gripper_io_);
    controller_->initEffort(params_.effort_mA);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    controller_->setVelocity(params_.velocity);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    controller_->enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    running_.store(true);
    feedback_thread_ = std::thread(&GripperSubsystem::feedbackLoop, this);
    capture_thread_ = std::thread(&GripperSubsystem::captureLoop, this);
    control_thread_ = std::thread(&GripperSubsystem::controlLoop, this);

    std::cout << "GripperSubsystem started (sensor=" << params_.sensor_port
              << ", gripper=" << params_.gripper_port
              << ", mit=" << (params_.mit_mode ? "22" : "23")
              << ", ctrl_rate=" << params_.ctrl_rate << " Hz"
              << ", capture_rate=" << params_.capture_rate << " Hz)" << std::endl;
    return true;
}

void GripperSubsystem::stop() {
    running_.store(false);

    // 先关 sensor 串口，解除 capture 线程上阻塞的 read_some
    if (sensor_port_) {
        std::lock_guard<std::mutex> lock(sensor_mtx_);
        boost::system::error_code ec;
        if (sensor_port_->is_open()) sensor_port_->close(ec);
    }

    if (control_thread_.joinable()) control_thread_.join();
    if (capture_thread_.joinable()) capture_thread_.join();
    if (feedback_thread_.joinable()) feedback_thread_.join();

    if (controller_) {
        try {
            controller_->disable();
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        } catch (...) {
        }
    }

    sample_queue_.setDone();

    if (sensor_port_) {
        std::lock_guard<std::mutex> lock(sensor_mtx_);
        sensor_port_.reset();
    }
    if (gripper_io_) gripper_io_->close();
    controller_.reset();
}

void GripperSubsystem::feedbackLoop() {
    while (running_.load()) {
        gripper_io_->pollAndParseFeedback();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void GripperSubsystem::captureLoop() {
    std::string buffer;
    char read_buf[2048];
    const double period =
        (params_.capture_rate > 0.0f)
            ? (1.0 / static_cast<double>(params_.capture_rate))
            : 0.0;
    double next_due = -1.0;  // 绝对节拍：入队后 next_due += period

    while (running_.load()) {
        size_t n = 0;
        try {
            std::lock_guard<std::mutex> lock(sensor_mtx_);
            if (!sensor_port_ || !sensor_port_->is_open()) break;
            n = sensor_port_->read_some(boost::asio::buffer(read_buf, sizeof(read_buf)));
        } catch (const boost::system::system_error& e) {
            if (!running_.load() || e.code() == boost::asio::error::interrupted) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        if (!running_.load()) break;
        if (n == 0) continue;

        buffer.append(read_buf, n);
        if (buffer.size() > 4096) buffer.erase(0, buffer.size() - 2048);

        int start = -1, end = -1;
        while (find_json(buffer, start, end)) {
            std::string json_str = buffer.substr(start, end - start + 1);
            buffer.erase(0, end + 1);
            try {
                auto json = nlohmann::json::parse(json_str);
                if (!json.contains("AS5047") || !json["AS5047"].contains("rad")) {
                    start = end = -1;
                    continue;
                }
                if (json["AS5047"].contains("error")) {
                    start = end = -1;
                    continue;
                }
                double angle = json["AS5047"]["rad"].get<double>();
                if (angle < 0.0) angle = 0.0;
                else if (angle > 1.67) angle = 1.67;

                const double ts = get_system_timestamp_seconds();
                // 每包更新最新角，供 Control 跟随（不受 capture_rate 限制）
                {
                    std::lock_guard<std::mutex> lk(latest_mtx_);
                    latest_angle_ = static_cast<float>(angle);
                    has_latest_ = true;
                }

                // 按绝对节拍入队：长期平均 ≈ capture_rate（源频率需更高）
                bool allow_enqueue = (period <= 0.0);
                if (!allow_enqueue) {
                    if (next_due < 0.0) {
                        allow_enqueue = true;
                        next_due = ts + period;
                    } else if (ts >= next_due) {
                        allow_enqueue = true;
                        // 追上节拍，避免落后堆积后连发多包
                        do {
                            next_due += period;
                        } while (next_due <= ts);
                    }
                }
                if (allow_enqueue) {
                    {
                        std::lock_guard<std::mutex> lk(ts_mtx_);
                        timestamps_.push_back(ts);
                    }
                    sample_queue_.push(AngleSample(ts, static_cast<float>(angle)));
                    samples_pushed_.fetch_add(1);
                }
            } catch (...) {
            }
            start = end = -1;
        }
    }
}

void GripperSubsystem::controlLoop() {
    const double period = 1.0 / std::max(1.0, static_cast<double>(params_.ctrl_rate));
    while (running_.load()) {
        auto fb = gripper_io_->snapshot();
        if (!fb.enabled) {
            controller_->enable();
        }

        float angle = 0.0f;
        bool has = false;
        {
            std::lock_guard<std::mutex> lk(latest_mtx_);
            has = has_latest_;
            angle = latest_angle_;
        }
        if (has) {
            controller_->sendPosition(angle, params_.mit_mode);
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(period));
    }
}
