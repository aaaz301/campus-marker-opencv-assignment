#pragma once

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace armor_detector {

/**
 * @brief 输入源类型。
 */
enum class InputType {
    IMAGE,   // 单张图片
    VIDEO,   // 视频文件
    CAMERA   // 摄像头
};

/**
 * @brief 统一输入源封装。
 * 对图片：只返回一帧。
 * 对视频/摄像头：循环返回帧。
 */
class Capture {
public:
    Capture() = default;
    ~Capture();

    /**
     * @brief 打开输入源。
     * @param source 图片路径、视频路径，或摄像头索引（"0", "1" 等）。
     * @return 是否成功。
     */
    bool open(const std::string& source);

    /**
     * @brief 读取一帧。
     * @param frame 输出帧。
     * @return 是否成功。
     */
    bool read(cv::Mat& frame);

    /**
     * @brief 释放资源。
     */
    void release();

    /**
     * @brief 是否已打开。
     */
    bool isOpened() const;

    /**
     * @brief 获取当前输入源类型。
     */
    InputType type() const;

    /**
     * @brief 获取视频/摄像头帧率（FPS）。
     */
    double fps() const;

private:
    InputType type_{InputType::IMAGE};
    cv::VideoCapture cap_;
    cv::Mat image_;
    bool image_consumed_{false};
};

} // namespace armor_detector
