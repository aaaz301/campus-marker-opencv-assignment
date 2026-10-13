#pragma once

#include <array>
#include <opencv2/core.hpp>

namespace armor_detector {

/**
 * @brief 目标颜色类型。
 */
enum class ColorType {
    RED,
    BLUE,
    UNKNOWN
};

/**
 * @brief 候选灯条。
 */
struct LightBar {
    cv::RotatedRect rotated_rect;           // 最小外接旋转矩形
    float area{};                           // 轮廓面积
    float aspect_ratio{};                   // 长宽比
    float angle{};                          // 归一化后的角度（度）
    cv::Point2f center{};                   // 中心点
    ColorType color{ColorType::UNKNOWN};    // 颜色
};

/**
 * @brief 候选装甲板。
 */
struct Armor {
    LightBar left_bar;                          // 左侧灯条
    LightBar right_bar;                         // 右侧灯条
    std::array<cv::Point2f, 4> vertices{};      // 左上、右上、右下、左下
    ColorType color{ColorType::UNKNOWN};        // 颜色
    float confidence{};                         // 置信度
};

} // namespace armor_detector
