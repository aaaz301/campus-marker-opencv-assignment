#pragma once

#include <opencv2/core.hpp>

#include "armor_detector/params.hpp"
#include "armor_detector/types.hpp"

namespace armor_detector {

/**
 * @brief 对输入 BGR 图像进行预处理，得到二值化掩码。
 * @param bgr 输入图像。
 * @param color 目标颜色。
 * @param params 检测参数。
 * @return 二值化掩码。
 */
cv::Mat preprocess(const cv::Mat& bgr, ColorType color, const DetectorParams& params);

} // namespace armor_detector
