#pragma once
#include <string>
#include <vector>

double get_system_timestamp_seconds();
std::string format_timestamp(double timestamp_sec);
std::string get_next_episode(const std::string& base_dir);