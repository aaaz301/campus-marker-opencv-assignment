#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "armor_detector/params.hpp"
#include "armor_detector/types.hpp"

namespace armor_detector {

/**
 * @brief 获取归一化角，范围[0~180)
 */
float getNormalizedAngle(const cv::RotatedRect &rect);

/**
 * @brief 获得RECT的长宽比
 */
float getRatio(const cv::RotatedRect &rect);

/**
 * @brief 从二值化图像中检测候选灯条。
 * @param binary 二值化图像。
 * @param params 检测参数。
 * @return 候选灯条列表，建议按面积从大到小排序。
 */
std::vector<LightBar> detectLightBars(const cv::Mat& binary, const DetectorParams& params, ColorType color = ColorType::UNKNOWN);

/**
 * @brief 在图像上绘制灯条。
 * @param image 输入/输出图像。
 * @param bars 灯条列表。
 */
void drawLightBars(cv::Mat& image, const std::vector<LightBar>& bars);

} // namespace armor_detector
