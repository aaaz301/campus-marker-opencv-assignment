#pragma once

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace armor_detector {

/**
 * @brief 检测结果视频写入封装。
 */
class VideoWriter {
public:
    VideoWriter() = default;
    ~VideoWriter();

    /**
     * @brief 指定路径、帧率、分辨率打开。
     */
    bool open(const std::string& path, double fps, const cv::Size& frame_size);

    /**
     * @brief 延迟打开：第一帧写入时自动根据帧大小初始化。
     */
    bool open(const std::string& path, double fps);

    /**
     * @brief 写入一帧。
     * @return 是否成功。
     */
    bool write(const cv::Mat& frame);

    /**
     * @brief 是否已打开。
     */
    bool isOpened() const;

    /**
     * @brief 释放资源。
     */
    void release();

private:
    cv::VideoWriter writer_;
    std::string path_;
    double fps_{30.0};
    cv::Size frame_size_{0, 0};
    bool initialized_{false};
};

} // namespace armor_detector
