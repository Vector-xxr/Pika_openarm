#pragma once
#include "frame_queue.h"
#include <string>
#include <atomic>

void save_thread_func(FrameQueue& queue, const std::string& save_dir,
                      const std::string& prefix, const std::string& format, int quality,
                      std::atomic<int>& saved_count, std::atomic<bool>& save_finished);