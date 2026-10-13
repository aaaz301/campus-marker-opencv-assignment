#include "armor_detector/video_writer.hpp"

#include <iostream>

namespace armor_detector {

VideoWriter::~VideoWriter() {
    release();
}

bool VideoWriter::open(const std::string& path, double fps, const cv::Size& frame_size) {
    release();

    if (frame_size.width <= 0 || frame_size.height <= 0 || fps <= 0.0) {
        std::cerr << "[ERROR] Invalid video writer parameters." << std::endl;
        return false;
    }

    path_ = path;
    fps_ = fps;
    frame_size_ = frame_size;

    // 默认使用 mp4v 编码，Linux 下通常可用
    writer_.open(path_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps_, frame_size_, true);
    initialized_ = writer_.isOpened();

    if (!initialized_) {
        std::cerr << "[ERROR] Failed to open video writer: " << path_ << std::endl;
    }

    return initialized_;
}

bool VideoWriter::open(const std::string& path, double fps) {
    release();
    path_ = path;
    fps_ = fps;
    initialized_ = false;
    return true;
}

bool VideoWriter::write(const cv::Mat& frame) {
    if (frame.empty()) {
        return false;
    }

    // 延迟初始化
    if (!initialized_) {
        frame_size_ = frame.size();
        writer_.open(path_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps_, frame_size_, true);
        initialized_ = writer_.isOpened();
        if (!initialized_) {
            std::cerr << "[ERROR] Failed to open video writer: " << path_ << std::endl;
            return false;
        }
    }

    writer_ << frame;
    return true;
}

bool VideoWriter::isOpened() const {
    return initialized_ && writer_.isOpened();
}

void VideoWriter::release() {
    writer_.release();
    initialized_ = false;
}

} // namespace armor_detector
