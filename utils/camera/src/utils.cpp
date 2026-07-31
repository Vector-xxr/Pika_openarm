#include "utils.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <dirent.h>
#include <cctype>
#include <cstdlib>
#include <iostream>

double get_system_timestamp_seconds() {
    auto now = std::chrono::system_clock::now();
    auto elapsed = now.time_since_epoch();
    return std::chrono::duration<double>(elapsed).count();
}

std::string format_timestamp(double timestamp_sec) {
    time_t sec_part = static_cast<time_t>(timestamp_sec);
    double frac_part = timestamp_sec - static_cast<double>(sec_part);
    int microsec = static_cast<int>(frac_part * 1000000);
    struct tm timeinfo;
    localtime_r(&sec_part, &timeinfo);
    std::stringstream ss;
    ss << std::put_time(&timeinfo, "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(6) << microsec;
    return ss.str();
}

std::string get_next_episode(const std::string& base_dir) {
    system(("mkdir -p " + base_dir).c_str());
    DIR* dir = opendir(base_dir.c_str());
    if (!dir) {
        std::cerr << "无法打开目录 " << base_dir << "，使用默认 001" << std::endl;
        return "001";
    }
    int max_num = 0;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type != DT_DIR) continue;
        std::string name = entry->d_name;
        bool all_digits = true;
        for (char c : name) {
            if (!std::isdigit(c)) { all_digits = false; break; }
        }
        if (!all_digits || name.empty()) continue;
        int num = std::stoi(name);
        if (num > max_num) max_num = num;
    }
    closedir(dir);
    int new_num = max_num + 1;
    std::ostringstream oss;
    oss << std::setw(3) << std::setfill('0') << new_num;
    return oss.str();
}