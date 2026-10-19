#include "armor_detector/capture.hpp"

#include <algorithm>
#include <cctype>
#include <iostream>

#include <opencv2/imgcodecs.hpp>

namespace armor_detector {

Capture::~Capture() {
    release();
}

bool Capture::open(const std::string& source) {
    release();

    if (source.empty()) {
        std::cerr << "[ERROR] Empty source." << std::endl;
        return false;
    }

    // 判断是否为纯数字（摄像头索引）
    bool is_number = !source.empty() && std::all_of(source.begin(), source.end(), [](unsigned char c) {
        return std::isdigit(c);
    });

    if (is_number) {
        // 摄像头
        type_ = InputType::CAMERA;
        int index = std::stoi(source);
        if (!cap_.open(index, cv::CAP_V4L2)) {
            // 回退到默认后端
            cap_.open(index);
        }
        return cap_.isOpened();
    }

    // 尝试作为图片打开
    image_ = cv::imread(source, cv::IMREAD_COLOR);
    if (!image_.empty()) {
        type_ = InputType::IMAGE;
        image_consumed_ = false;
        return true;
    }

    // 尝试作为视频打开
    cap_.open(source);
    if (cap_.isOpened()) {
        type_ = InputType::VIDEO;
        return true;
    }

    std::cerr << "[ERROR] Failed to open source: " << source << std::endl;
    return false;
}

bool Capture::read(cv::Mat& frame) {
    switch (type_) {
        case InputType::IMAGE:
            if (!image_consumed_ && !image_.empty()) {
                frame = image_.clone();
                image_consumed_ = true;
                return true;
            }
            return false;

        case InputType::VIDEO:
        case InputType::CAMERA:
            return cap_.read(frame);

        default:
            return false;
    }
}

void Capture::release() {
    cap_.release();
    image_.release();
    image_consumed_ = false;
    type_ = InputType::IMAGE;
}

bool Capture::isOpened() const {
    if (type_ == InputType::IMAGE) {
        return !image_.empty() && !image_consumed_;
    }
    return cap_.isOpened();
}

InputType Capture::type() const {
    return type_;
}

double Capture::fps() const {
    if (type_ == InputType::IMAGE) {
        return 0.0;
    }
    return cap_.get(cv::CAP_PROP_FPS);
}

} // namespace armor_detector
