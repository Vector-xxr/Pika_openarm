#pragma once
#include "frame_queue.h"
#include "angle_queue.h"
#include <string>
#include <atomic>
#include <fstream>

void save_thread_func(FrameQueue& queue, const std::string& save_dir,
                      const std::string& prefix, const std::string& format, int quality,
                      std::atomic<int>& saved_count, std::atomic<bool>& save_finished);

// 写 gripper/angle.csv：Frame_Index,Timestamp(s),Delay(s),Angle(rad)
void save_gripper_thread_func(AngleQueue& queue, std::ofstream& csv,
                              std::atomic<int>& saved_count,
                              std::atomic<bool>& save_finished);
