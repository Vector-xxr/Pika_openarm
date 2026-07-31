#pragma once
#include <boost/asio.hpp>
#include <mutex>
#include <string>
#include <vector>
#include <atomic>

// 夹爪串口 IO：打开/读写 + 解析 gripper 侧 motor/motorstatus 回执
class GripperSerialIO {
public:
    struct FeedbackState {
        bool enabled = false;
        float motor_angle = -1.0f;
        float motor_current = 0.0f;
        std::string status_str = "????";
    };

    GripperSerialIO();
    ~GripperSerialIO();

    bool open(const std::string& port, unsigned int baud_rate);
    void close();
    bool isOpen() const;

    void writeBytes(const std::vector<uint8_t>& data);

    // 非阻塞读一批字节并解析 JSON 回执；返回读到的字节数
    size_t pollAndParseFeedback();

    FeedbackState snapshot() const;

private:
    void configure(unsigned int baud_rate);
    void appendAndParse(const char* data, size_t n);

    boost::asio::io_context io_;
    std::unique_ptr<boost::asio::serial_port> port_;
    mutable std::mutex mtx_;
    std::string rx_buffer_;
    FeedbackState state_;
};
