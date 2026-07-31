#include "config.h"
#include "recorder.h"
#include "utils.h"
#include <iostream>
#include <cstdlib>

int main(int argc, char* argv[]) {
    try {
        // 1. 加载配置
        std::string config_path = "../config/default.yaml"; // 可调整路径
        Config cfg = loadConfig(config_path);

        // 2. 命令行覆盖 episode
        if (argc >= 2) {
            cfg.general.episode = argv[1];
        }

        // 3. 创建 Recorder
        Recorder recorder(cfg);
        if (!recorder.initialize()) {
            std::cerr << "Recorder 初始化失败" << std::endl;
            return EXIT_FAILURE;
        }

        // 4. 开始采集（阻塞）
        recorder.start();

        // 5. 打印统计
        recorder.printStatistics();

        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}