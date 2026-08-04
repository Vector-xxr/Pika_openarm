#pragma once
#include <vector>
#include <string>

// active_seconds: wall time while session was active (p..q segments).
// Used for average fps and 总耗时. Inter-frame min/max/stddev skip gaps > 0.2s
// (pause between q and p), matching Capture_SOP.md.
void print_statistics(const std::vector<double>& timestamps, const std::string& stream_name,
                      double active_seconds);
