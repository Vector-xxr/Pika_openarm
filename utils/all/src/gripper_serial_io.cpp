#include "gripper_serial_io.h"
#include "gripper_protocol.h"
#include <nlohmann/json.hpp>
#include <sys/ioctl.h>
#include <iostream>
#include <cstring>

GripperSerialIO::GripperSerialIO() = default;

GripperSerialIO::~GripperSerialIO() {
    close();
}

void GripperSerialIO::configure(unsigned int baud_rate) {
    using boost::asio::serial_port;
    port_->set_option(serial_port::baud_rate(baud_rate));
    port_->set_option(serial_port::character_size(8));
    port_->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    port_->set_option(serial_port::parity(serial_port::parity::none));
    port_->set_option(serial_port::flow_control(serial_port::flow_control::none));
}

bool GripperSerialIO::open(const std::string& port, unsigned int baud_rate) {
    std::lock_guard<std::mutex> lock(mtx_);
    try {
        port_ = std::make_unique<boost::asio::serial_port>(io_, port);
        configure(baud_rate);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "GripperSerialIO open failed (" << port << "): " << e.what() << std::endl;
        port_.reset();
        return false;
    }
}

void GripperSerialIO::close() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (port_ && port_->is_open()) {
        boost::system::error_code ec;
        port_->close(ec);
    }
    port_.reset();
}

bool GripperSerialIO::isOpen() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return port_ && port_->is_open();
}

void GripperSerialIO::writeBytes(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!port_ || !port_->is_open()) return;
    boost::asio::write(*port_, boost::asio::buffer(data));
}

void GripperSerialIO::appendAndParse(const char* data, size_t n) {
    rx_buffer_.append(data, n);
    if (rx_buffer_.size() > 8192) {
        size_t pos = rx_buffer_.find('{');
        if (pos != std::string::npos) rx_buffer_.erase(0, pos);
        else rx_buffer_.clear();
    }

    int start = -1, end = -1;
    while (find_json(rx_buffer_, start, end)) {
        std::string json_str = rx_buffer_.substr(start, end - start + 1);
        rx_buffer_.erase(0, end + 1);
        try {
            auto json = nlohmann::json::parse(json_str);
            if (json.contains("motor") && json["motor"].contains("Position")) {
                state_.motor_angle = json["motor"]["Position"].get<float>();
                if (json["motor"].contains("Current")) {
                    state_.motor_current = json["motor"]["Current"].get<float>();
                }
            }
            if (json.contains("motorstatus") && json["motorstatus"].contains("Status")) {
                std::string st = json["motorstatus"]["Status"].get<std::string>();
                int status = hex2dec(st);
                state_.enabled = (status & 0b01000000) != 0;
                state_.status_str = st;
            }
        } catch (...) {
        }
        start = end = -1;
    }
}

size_t GripperSerialIO::pollAndParseFeedback() {
    char read_buf[2048];
    size_t n = 0;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!port_ || !port_->is_open()) return 0;
        try {
            int bytes_available = 0;
            if (ioctl(port_->native_handle(), FIONREAD, &bytes_available) == 0 &&
                bytes_available > 0) {
                n = port_->read_some(boost::asio::buffer(read_buf, sizeof(read_buf)));
                if (n > 0) appendAndParse(read_buf, n);
            }
        } catch (const boost::system::system_error&) {
            return 0;
        }
    }
    return n;
}

GripperSerialIO::FeedbackState GripperSerialIO::snapshot() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return state_;
}
