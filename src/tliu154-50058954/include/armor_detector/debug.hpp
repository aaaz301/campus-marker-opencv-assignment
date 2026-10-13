#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "armor_detector/types.hpp"

namespace armor_detector {

/**
 * @brief 调试中间结果。
 */
struct DebugInfo {
    cv::Mat binary_image;
    cv::Mat light_bar_image;
    cv::Mat armor_image;
};

/**
 * @brief 保存调试图像到输出目录。
 * @param info 调试信息。
 * @param output_dir 输出目录。
 */
void saveDebugImages(const DebugInfo& info, const std::string& output_dir);

} // namespace armor_detector
