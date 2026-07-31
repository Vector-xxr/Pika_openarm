// read_gripper_angle.cpp
// 从 sensor 读 AS5047 角度，控制 gripper 跟随；可选保存 sensor 角度 CSV。
// 协议对齐 serial_gripper_imu.cpp：EFFORT + VELOCITY + 读反馈，Status 无 0x40 时重发 ENABLE，位置照发。
#include <iostream>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <cstring>
#include <chrono>
#include <cmath>
#include <atomic>
#include <csignal>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdlib>
#include <ctime>

using boost::asio::serial_port;
using boost::asio::io_context;

enum SendFlag {
    DISABLE = 10,
    ENABLE = 11,
    EFFORT_CTRL = 15,
    POSITION_CTRL_MIT = 22,
    POSITION_CTRL_POS_VEL = 23,
    VELOCITY_CTRL = 13   // 新增，用于速度限制
};

static std::atomic<bool> g_running{true};

void onSignal(int) {
    g_running = false;
}

bool find_json(const std::string& msg, int& start, int& end) {
    std::vector<int> stack;
    for (size_t i = 0; i < msg.size(); ++i) {
        char ch = msg[i];
        if (ch == '{') {
            stack.push_back(static_cast<int>(i));
        } else if (ch == '}') {
            if (!stack.empty()) {
                int index = stack.back();
                stack.pop_back();
                if (stack.empty() || (index > 0 && msg[index - 1] != ':')) {
                    start = index;
                    end = static_cast<int>(i);
                    return true;
                }
            }
        }
    }
    return false;
}

int hex2dec(const std::string& str) {
    std::string s = str;
    if (s.size() >= 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        s = s.substr(2);
    }
    int num = 0;
    for (size_t i = 0; i < s.size(); ++i) {
        char ch = s[i];
        int temp = 0;
        if (ch >= '0' && ch <= '9') temp = ch - '0';
        else if (ch >= 'A' && ch <= 'F') temp = ch - 'A' + 10;
        else if (ch >= 'a' && ch <= 'f') temp = ch - 'a' + 10;
        else continue;
        num = (num << 4) | temp;
    }
    return num;
}

template <typename T>
std::vector<uint8_t> createBinaryCommand(uint8_t cmd, const std::vector<T>& values) {
    std::vector<uint8_t> binaryCmd;
    binaryCmd.push_back(cmd);
    for (size_t i = 0; i < values.size(); ++i) {
        T value = values[i];
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
        for (size_t b = 0; b < sizeof(T); ++b) {
            binaryCmd.push_back(bytes[b]);
        }
    }
    binaryCmd.push_back('\r');
    binaryCmd.push_back('\n');
    return binaryCmd;
}

void configureSerial(serial_port& sp) {
    sp.set_option(serial_port::baud_rate(460800));
    sp.set_option(serial_port::character_size(8));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
}

struct GripperState {
    std::mutex mtx;
    bool enabled = false;
    float motor_angle = -1.0f;
    float motor_current = 0.0f;
    std::string status_str = "????";
};

struct SensorData {
    double timestamp;
    float angle;
};

std::queue<SensorData> sensor_queue;
std::mutex queue_mtx;
std::condition_variable queue_cv;

void saveSensorDataThread(const std::string& save_dir) {
    std::string cmd = "mkdir -p \"" + save_dir + "\"";
    if (system(cmd.c_str()) != 0) {
        std::cerr << "Warning: mkdir failed for " << save_dir << std::endl;
    }

    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm_info{};
    localtime_r(&now_c, &tm_info);
    std::ostringstream oss;
    oss << std::put_time(&tm_info, "%Y%m%d_%H%M%S");
    std::string filename = save_dir + "/sensor_" + oss.str() + ".csv";

    std::ofstream csv(filename);
    if (!csv) {
        std::cerr << "Warning: cannot open " << filename << " for writing\n";
        return;
    }
    csv << "Frame_Index,Timestamp(s),Delay(s),Angle(rad)\n";

    int frame_idx = 0;
    double last_timestamp = 0.0;

    while (g_running || !sensor_queue.empty()) {
        SensorData data;
        {
            std::unique_lock<std::mutex> lock(queue_mtx);
            if (sensor_queue.empty() && g_running) {
                queue_cv.wait_for(lock, std::chrono::milliseconds(100));
                continue;
            }
            if (sensor_queue.empty()) break;
            data = sensor_queue.front();
            sensor_queue.pop();
        }

        double delay = (frame_idx == 0) ? 0.0 : (data.timestamp - last_timestamp);
        csv << frame_idx << ","
            << std::fixed << std::setprecision(9) << data.timestamp << ","
            << delay << ","
            << std::setprecision(6) << data.angle << "\n";
        last_timestamp = data.timestamp;
        frame_idx++;
    }
    csv.close();
    std::cout << "Sensor data saved to " << filename << std::endl;
}

void writeCommand(serial_port& sp, std::mutex& serial_mtx, const std::vector<uint8_t>& cmd) {
    std::lock_guard<std::mutex> lock(serial_mtx);
    boost::asio::write(sp, boost::asio::buffer(cmd));
}

void gripperReceiveLoop(serial_port& sp, std::mutex& serial_mtx, GripperState& state) {
    std::string buffer;
    char read_buf[2048];
    while (g_running) {
        size_t n = 0;
        try {
            std::lock_guard<std::mutex> lock(serial_mtx);
            int bytes_available = 0;
            if (ioctl(sp.native_handle(), FIONREAD, &bytes_available) == 0 &&
                bytes_available > 0) {
                n = sp.read_some(boost::asio::buffer(read_buf, sizeof(read_buf)));
            }
        } catch (const boost::system::system_error& e) {
            if (!g_running || e.code() == boost::asio::error::interrupted) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        if (n == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        buffer.append(read_buf, n);
        if (buffer.size() > 8192) {
            size_t pos = buffer.find('{');
            if (pos != std::string::npos) buffer.erase(0, pos);
            else buffer.clear();
        }

        int start = -1, end = -1;
        while (find_json(buffer, start, end)) {
            std::string json_str = buffer.substr(start, end - start + 1);
            buffer.erase(0, end + 1);
            try {
                auto json = nlohmann::json::parse(json_str);
                if (json.contains("motor") && json["motor"].contains("Position")) {
                    std::lock_guard<std::mutex> lk(state.mtx);
                    state.motor_angle = json["motor"]["Position"].get<float>();
                    if (json["motor"].contains("Current")) {
                        state.motor_current = json["motor"]["Current"].get<float>();
                    }
                }
                if (json.contains("motorstatus") && json["motorstatus"].contains("Status")) {
                    std::string st = json["motorstatus"]["Status"].get<std::string>();
                    int status = hex2dec(st);
                    bool en = (status & 0b01000000) != 0;
                    std::lock_guard<std::mutex> lk(state.mtx);
                    state.enabled = en;
                    state.status_str = st;
                }
            } catch (...) {
            }
            start = end = -1;
        }
    }
}

void printUsage(const char* prog) {
    std::cerr << "Usage: " << prog
              << " [sensor_port] [gripper_port] [options]\n"
              << "  defaults: sensor=/dev/ttyUSB50 gripper=/dev/ttyUSB60\n"
              << "  --effort mA     current limit in mA, default 1000\n"
              << "  --velocity rad/s speed limit, default 20.0\n"   // 新增速度参数
              << "  --rate Hz       control rate, default 1000\n"
              << "  --mit           POSITION_CTRL_MIT(22) [default]\n"
              << "  --pos-vel       POSITION_CTRL_POS_VEL(23)\n"
              << "  --verbose\n";
}

int main(int argc, char* argv[]) {
    std::string sensor_port = "/dev/ttyUSB50";
    std::string gripper_port = "/dev/ttyUSB60";
    float motor_current_limit_mA = 1000.0f;
    float velocity_limit = 20.0f;    // 新增速度限制变量，默认 1.0 rad/s
    float ctrl_rate = 100.0f;
    bool mit_mode = true;
    bool verbose = false;

    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--effort" && i + 1 < argc) {
            motor_current_limit_mA = std::stof(argv[++i]);
        } else if (arg == "--velocity" && i + 1 < argc) {
            velocity_limit = std::stof(argv[++i]);
        } else if (arg == "--rate" && i + 1 < argc) {
            ctrl_rate = std::stof(argv[++i]);
        } else if (arg == "--mit") {
            mit_mode = true;
        } else if (arg == "--pos-vel") {
            mit_mode = false;
        } else if (arg == "--verbose" || arg == "-v") {
            verbose = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (!arg.empty() && arg[0] == '-') {
            std::cerr << "Unknown option: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        } else {
            positional.push_back(arg);
        }
    }
    if (positional.size() >= 1) sensor_port = positional[0];
    if (positional.size() >= 2) gripper_port = positional[1];
    if (ctrl_rate <= 0.0f) {
        std::cerr << "ctrl rate must be > 0" << std::endl;
        return 1;
    }
    const double ctrl_period = 1.0 / static_cast<double>(ctrl_rate);
    const uint8_t pos_cmd = mit_mode ? POSITION_CTRL_MIT : POSITION_CTRL_POS_VEL;

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    try {
        io_context io;
        std::cout << "Opening sensor : " << sensor_port << std::endl;
        std::cout << "Opening gripper: " << gripper_port << std::endl;
        serial_port sensor_sp(io, sensor_port);
        serial_port gripper_sp(io, gripper_port);
        configureSerial(sensor_sp);
        configureSerial(gripper_sp);

        std::mutex gripper_serial_mtx;
        GripperState gripper_state;
        std::thread rx_thread(gripperReceiveLoop,
                              std::ref(gripper_sp),
                              std::ref(gripper_serial_mtx),
                              std::ref(gripper_state));

        // ========== 初始化：先 EFFORT_CTRL，再 VELOCITY_CTRL，再 ENABLE ==========
        const float effort = motor_current_limit_mA / 1000.0f;
        writeCommand(gripper_sp, gripper_serial_mtx,
                     createBinaryCommand<float>(EFFORT_CTRL, {effort}));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // 新增：发送速度限制命令 (两个参数均为 velocity_limit)
        writeCommand(gripper_sp, gripper_serial_mtx,
                     createBinaryCommand<float>(VELOCITY_CTRL, {velocity_limit, velocity_limit}));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        writeCommand(gripper_sp, gripper_serial_mtx,
                     createBinaryCommand<float>(ENABLE, {0.0f}));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 等待使能确认（最多 1 秒）
        for (int i = 0; i < 50 && g_running; ++i) {
            bool en = false;
            std::string st;
            {
                std::lock_guard<std::mutex> lk(gripper_state.mtx);
                en = gripper_state.enabled;
                st = gripper_state.status_str;
            }
            if (en) {
                std::cout << "Gripper ENABLED (Status=" << st << ")" << std::endl;
                break;
            }
            if (i == 20) {
                writeCommand(gripper_sp, gripper_serial_mtx,
                             createBinaryCommand<float>(ENABLE, {0.0f}));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            if (i == 49) {
                std::cerr << "Warning: gripper not enabled yet (Status=" << st
                          << "). Will keep retrying ENABLE while commanding.\n";
            }
        }

        const char* home = std::getenv("HOME");
        std::string save_dir = (home ? std::string(home) : std::string("."))
                               + "/pika_ros/data/007/gripper";
        std::thread save_thread(saveSensorDataThread, save_dir);

        std::cout << "Mode=" << (mit_mode ? "MIT(22)" : "POS_VEL(23)")
                  << " rate=" << ctrl_rate << " Hz"
                  << " effort=" << motor_current_limit_mA << " mA"
                  << " velocity=" << velocity_limit << " rad/s"
                  << std::endl;
        std::cout << "Following sensor angle -> gripper. Ctrl+C to stop." << std::endl;

        std::string buffer;
        char read_buf[2048];
        double last_cmd_time = -1.0;

        while (g_running) {
            size_t n = 0;
            try {
                n = sensor_sp.read_some(boost::asio::buffer(read_buf, sizeof(read_buf)));
            } catch (const boost::system::system_error& e) {
                if (!g_running || e.code() == boost::asio::error::interrupted) break;
                throw;
            }
            if (!g_running) break;
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
                        continue;
                    }
                    if (json["AS5047"].contains("error")) {
                        continue;
                    }

                    double sensor_angle = json["AS5047"]["rad"].get<double>();
                    if (sensor_angle < 0.0) sensor_angle = 0.0;
                    else if (sensor_angle > 1.67) sensor_angle = 1.67;
                    float cmd_angle = static_cast<float>(sensor_angle);

                    auto now = std::chrono::steady_clock::now();
                    double now_s = std::chrono::duration<double>(now.time_since_epoch()).count();

                    // 频率限制：只有满足时间间隔才执行后续操作（数据存储 + 夹爪控制）
                    if (last_cmd_time >= 0.0 && (now_s - last_cmd_time) < ctrl_period) {
                        start = end = -1;
                        continue;   // 跳过本次所有操作，不存储数据也不发送命令
                    }

                    // ---------- 在此处存储数据（与控制同频率） ----------
                    {
                        std::lock_guard<std::mutex> lock(queue_mtx);
                        sensor_queue.push(SensorData{now_s, cmd_angle});
                        queue_cv.notify_one();
                    }

                    // 检查使能状态，未使能则重发 ENABLE
                    bool enabled = false;
                    float motor_angle = -1.0f;
                    std::string status_str;
                    {
                        std::lock_guard<std::mutex> lk(gripper_state.mtx);
                        enabled = gripper_state.enabled;
                        motor_angle = gripper_state.motor_angle;
                        status_str = gripper_state.status_str;
                    }
                    if (!enabled) {
                        writeCommand(gripper_sp, gripper_serial_mtx,
                                     createBinaryCommand<float>(ENABLE, {0.0f}));
                    }

                    // 发送位置命令
                    writeCommand(gripper_sp, gripper_serial_mtx,
                                 createBinaryCommand<float>(pos_cmd, {cmd_angle}));
                    last_cmd_time = now_s;

                    if (verbose) {
                        std::cout << "sensor=" << sensor_angle
                                  << " cmd=" << static_cast<int>(pos_cmd)
                                  << " motor=" << motor_angle
                                  << " Status=" << status_str
                                  << (enabled ? " EN" : " DIS")
                                  << std::endl;
                    }
                } catch (const nlohmann::json::parse_error&) {
                }
                start = end = -1;
            }
        }

        g_running = false;
        queue_cv.notify_all();

        // 退出前发送 DISABLE
        try {
            writeCommand(gripper_sp, gripper_serial_mtx,
                         createBinaryCommand<float>(DISABLE, {0.0f}));
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            std::cout << "\nGripper DISABLED." << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "\nWarning: failed to send DISABLE: " << e.what() << std::endl;
        }

        if (rx_thread.joinable()) rx_thread.join();
        if (save_thread.joinable()) save_thread.join();
        std::cout << "Stopped." << std::endl;
    } catch (const std::exception& e) {
        g_running = false;
        queue_cv.notify_all();
        std::cerr << "Error: " << e.what() << std::endl;
        printUsage(argv[0]);
        return 1;
    }
    return 0;
}