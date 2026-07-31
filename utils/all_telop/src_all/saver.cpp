#include "saver.h"
#include "utils.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>

void save_thread_func(FrameQueue& queue, const std::string& save_dir,
                      const std::string& prefix, const std::string& format, int quality,
                      std::atomic<int>& saved_count, std::atomic<bool>& save_finished) {
    while (true) {
        FrameData frame;
        if (!queue.pop(frame)) break;

        std::string time_str = format_timestamp(frame.timestamp);
        std::string ext = (format == "png") ? ".png" : ".jpg";
        std::string filename = save_dir + "/" + prefix + "_" + time_str + ext;

        if (format == "png") {
            cv::imwrite(filename, frame.image);
        } else {
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};
            cv::imwrite(filename, frame.image, params);
        }
        saved_count++;
    }
    save_finished = true;
}

void save_gripper_thread_func(AngleQueue& queue, std::ofstream& csv,
                              std::atomic<int>& saved_count,
                              std::atomic<bool>& save_finished) {
    int frame_idx = 0;
    double last_ts = 0.0;
    while (true) {
        AngleSample sample;
        if (!queue.pop(sample)) break;

        if (!csv.is_open()) {
            saved_count++;
            continue;
        }

        if (frame_idx == 0) {
            csv << frame_idx << "," << std::fixed << std::setprecision(9)
                << sample.timestamp << ",N/A," << std::setprecision(6)
                << sample.angle << "\n";
        } else {
            csv << frame_idx << "," << std::fixed << std::setprecision(9)
                << sample.timestamp << "," << (sample.timestamp - last_ts) << ","
                << std::setprecision(6) << sample.angle << "\n";
        }
        last_ts = sample.timestamp;
        frame_idx++;
        saved_count++;
    }
    if (csv.is_open()) csv.flush();
    save_finished = true;
}
