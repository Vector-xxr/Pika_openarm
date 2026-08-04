#include "statistics.h"
#include <iostream>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <iomanip>

namespace {

// Gaps larger than this are treated as q→p pause and excluded from interval stats.
constexpr double kMaxInSessionDt = 0.2;

}  // namespace

void print_statistics(const std::vector<double>& timestamps, const std::string& stream_name,
                      double active_seconds) {
    if (timestamps.size() < 2) {
        std::cout << stream_name << " 未采集到有效帧，无法统计" << std::endl;
        return;
    }

    std::vector<double> diffs;
    diffs.reserve(timestamps.size() - 1);
    for (size_t i = 1; i < timestamps.size(); ++i) {
        const double dt = timestamps[i] - timestamps[i - 1];
        if (dt > 0.0 && dt <= kMaxInSessionDt) {
            diffs.push_back(dt);
        }
    }

    if (diffs.empty()) {
        std::cout << stream_name << " 有效帧间隔不足（均超过暂停判定阈值 "
                  << kMaxInSessionDt << "s），无法统计" << std::endl;
        return;
    }

    const double sum = std::accumulate(diffs.begin(), diffs.end(), 0.0);
    const double avg = sum / static_cast<double>(diffs.size());
    const double min_val = *std::min_element(diffs.begin(), diffs.end());
    const double max_val = *std::max_element(diffs.begin(), diffs.end());
    const double sq_sum = std::inner_product(diffs.begin(), diffs.end(), diffs.begin(), 0.0);
    const double var = sq_sum / static_cast<double>(diffs.size()) - avg * avg;
    const double stddev = std::sqrt(std::max(0.0, var));

    // Prefer keyboard-tracked active session time; fall back to filtered sum.
    const double total_time =
        (active_seconds > 0.0) ? active_seconds : sum;
    const double avg_fps =
        (total_time > 0.0) ? (static_cast<double>(timestamps.size() - 1) / total_time) : 0.0;

    std::cout << "\n======= " << stream_name << " 统计 =======" << std::endl;
    std::cout << "average rate: " << std::fixed << std::setprecision(3) << avg_fps << " fps"
              << std::endl;
    std::cout << "min: " << std::setprecision(3) << min_val << "s max: " << max_val << "s "
              << "std dev: " << std::setprecision(5) << stddev << "s window: " << diffs.size()
              << std::endl;
    std::cout << "总耗时: " << std::setprecision(5) << total_time << " 秒"
              << " (仅 session 激活累计；间隔统计已排除 >" << kMaxInSessionDt << "s 的暂停间隙)"
              << std::endl;
    std::cout << "===============================" << std::endl;
}
