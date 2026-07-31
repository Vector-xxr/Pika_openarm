// all_telop — orchestrates camera capture + serial gripper + OpenArm teleop
// under a single keyboard p/q session gate.

#include "config.h"
#include "recorder.h"

#include <common.hpp>
#include <teleop_session.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

std::string findConfigPath() {
    const std::vector<std::string> candidates = {
        "config/default.yaml",
        "../config/default.yaml",
        "utils/all_telop/config/default.yaml",
    };
    for (const auto& p : candidates) {
        std::ifstream f(p);
        if (f.good()) return p;
    }
    return "../config/default.yaml";
}

class TerminalRawMode {
public:
    bool enable() {
        if (!isatty(STDIN_FILENO)) return false;
        if (tcgetattr(STDIN_FILENO, &orig_) != 0) return false;
        termios raw = orig_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return false;
        active_ = true;
        return true;
    }

    void disable() {
        if (active_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
            active_ = false;
        }
    }

    ~TerminalRawMode() { disable(); }

private:
    termios orig_{};
    bool active_ = false;
};

// Sets session_active immediately (camera + gripper gate) and start/stop edges for teleop Admin.
void keyboard_loop(std::atomic<bool>* cameras_ready, std::atomic<bool>* session_active,
                   std::atomic<bool>* session_start, std::atomic<bool>* session_stop,
                   PoseHealthMonitor* health) {
    TerminalRawMode terminal;
    if (!terminal.enable()) {
        std::cerr << "[all_telop] WARN: stdin is not a TTY; p/q unavailable." << std::endl;
        return;
    }

    bool expect_p = true;
    while (keep_running) {
        char key = 0;
        const ssize_t n = ::read(STDIN_FILENO, &key, 1);
        if (n != 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        if ((key == 'p' || key == 'P') && expect_p) {
            if (cameras_ready == nullptr || !cameras_ready->load()) {
                std::cout << "[all_telop] 相机尚未就绪，忽略 p（等待预热 barrier）" << std::endl;
                continue;
            }
            expect_p = false;
            if (session_active != nullptr) session_active->store(true);
            if (session_start != nullptr) session_start->store(true);
            if (health != nullptr && !health->dynamic_enabled()) {
                health->enable_dynamic(true);
            }
            std::cout << "[all_telop] SESSION START — 相机采集 + 夹爪跟随 + 机械臂遥操 (rezero)"
                      << std::endl;
        } else if ((key == 'q' || key == 'Q') && !expect_p) {
            expect_p = true;
            if (session_active != nullptr) session_active->store(false);
            if (session_stop != nullptr) session_stop->store(true);
            std::cout << "[all_telop] SESSION STOP — 暂停采集/夹爪跟随，臂保持姿态" << std::endl;
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::string episode_override;
    std::string config_arg;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_arg = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: " << argv[0] << " [episode] [--config PATH]\n"
                << "  episode         optional episode name (default: auto from yaml)\n"
                << "  --config PATH   YAML (default: ../config/default.yaml)\n"
                << "Keys:\n"
                << "  p    start/resume: cameras + gripper follow + arm teleop (rezero)\n"
                << "  q    pause capture/gripper; arm holds pose (same episode)\n"
                << "  Ctrl+C  exit and flush\n"
                << "Prerequisite: ros2 launch pika_locator; CAN-FD up.\n";
            return 0;
        } else if (!arg.empty() && arg[0] != '-') {
            episode_override = arg;
        }
    }

    try {
        const std::string config_path = config_arg.empty() ? findConfigPath() : config_arg;
        std::cout << "[all_telop] config: " << config_path << std::endl;

        Config cfg = loadConfig(config_path);
        if (!episode_override.empty()) {
            cfg.general.episode = episode_override;
        }

        std::atomic<bool> session_active{false};
        std::atomic<bool> session_start{false};
        std::atomic<bool> session_stop{false};

        Recorder recorder(cfg);
        if (!recorder.initialize()) {
            std::cerr << "[all_telop] Recorder 初始化失败" << std::endl;
            return EXIT_FAILURE;
        }
        recorder.setSessionActive(&session_active);
        recorder.setKeepRunning(&keep_running);

        TeleopSession teleop;
        if (!teleop.init(argc, argv, config_path, recorder.viveCsvPath())) {
            std::cerr << "[all_telop] Teleop 初始化失败" << std::endl;
            keep_running = false;
            return EXIT_FAILURE;
        }

        // Parallel: camera warmup + Vive static check
        std::thread recorder_thread([&]() { recorder.startSession(); });
        std::thread static_thread([&]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (!teleop.runStaticCheck()) {
                std::cerr << "[all_telop] 静态校验中止 (shutdown)" << std::endl;
            }
        });

        static_thread.join();
        if (!keep_running.load()) {
            recorder_thread.join();
            teleop.stop();
            return 0;
        }

        while (keep_running.load() && !recorder.camerasReady() && !recorder.camerasFailed()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (recorder.camerasFailed() || !recorder.camerasReady()) {
            std::cerr << "[all_telop] 相机未就绪，退出" << std::endl;
            keep_running = false;
            recorder_thread.join();
            teleop.stop();
            return EXIT_FAILURE;
        }

        // Follower holds at home until first p (session_active=false)
        teleop.startControl(&session_active, &session_start, &session_stop);

        std::cout << "[all_telop] Ready — cameras warmed, static check done, session OFF."
                  << std::endl;
        std::cout << "[all_telop] Press 'p' to START (capture+gripper+teleop), "
                     "'q' to PAUSE, Ctrl+C to exit."
                  << std::endl;
        std::cout << "[all_telop] Episode dir: " << recorder.baseDir() << std::endl;

        std::atomic<bool> cameras_ready_flag{true};
        std::thread keyboard_thread(keyboard_loop, &cameras_ready_flag, &session_active,
                                    &session_start, &session_stop, teleop.poseHealth());

        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        session_active.store(false);
        session_stop.store(true);

        if (keyboard_thread.joinable()) keyboard_thread.join();
        teleop.stop();
        recorder_thread.join();
        recorder.printStatistics();

        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "[all_telop] 异常: " << e.what() << std::endl;
        keep_running = false;
        return EXIT_FAILURE;
    }
}
