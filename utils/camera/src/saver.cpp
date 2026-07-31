#include "saver.h"
#include "utils.h"
#include <opencv2/opencv.hpp>
#include <iostream>

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
            // PNG 编码：可选压缩级别（默认无损），可设置 IMWRITE_PNG_COMPRESSION 但这里不设
            cv::imwrite(filename, frame.image);
        } else { // jpg
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};
            cv::imwrite(filename, frame.image, params);
        }
        saved_count++;
    }
    save_finished = true;
}