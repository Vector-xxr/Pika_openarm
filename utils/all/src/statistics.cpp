#include "statistics.h"
#include <iostream>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <iomanip>

void print_statistics(const std::vector<double>& timestamps, const std::string& stream_name) {
    if (timestamps.size() < 2) {
        std::cout << stream_name << " 未采集到有效帧，无法统计" << std::endl;
        return;
    }
    std::vector<double> diffs;
    diffs.reserve(timestamps.size() - 1);
    for (size_t i = 1; i < timestamps.size(); ++i)
        diffs.push_back(timestamps[i] - timestamps[i-1]);

    double sum = std::accumulate(diffs.begin(), diffs.end(), 0.0);
    double avg = sum / diffs.size();
    double min_val = *std::min_element(diffs.begin(), diffs.end());
    double max_val = *std::max_element(diffs.begin(), diffs.end());
    double sq_sum = std::inner_product(diffs.begin(), diffs.end(), diffs.begin(), 0.0);
    double stddev = std::sqrt(sq_sum / diffs.size() - avg * avg);
    double total_time = timestamps.back() - timestamps.front();
    double avg_fps = (timestamps.size() - 1) / total_time;

    std::cout << "\n======= " << stream_name << " 统计 =======" << std::endl;
    std::cout << "average rate: " << std::fixed << std::setprecision(3) << avg_fps << " fps" << std::endl;
    std::cout << "min: " << std::setprecision(3) << min_val << "s max: " << max_val << "s "
              << "std dev: " << std::setprecision(5) << stddev << "s window: " << diffs.size() << std::endl;
    std::cout << "总耗时: " << total_time << " 秒" << std::endl;
    std::cout << "===============================" << std::endl;
}