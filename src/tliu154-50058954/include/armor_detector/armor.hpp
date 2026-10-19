#pragma once

#include <vector>

#include "armor_detector/light_bar.hpp"
#include "armor_detector/params.hpp"
#include "armor_detector/types.hpp"

namespace armor_detector {

/**
 * @brief 将候选灯条配对成装甲板。
 * @param bars 灯条列表。
 * @param params 检测参数。
 * @return 候选装甲板列表。
 */
std::vector<Armor> matchArmors(const std::vector<LightBar>& bars, const DetectorParams& params);

/**
 * @brief 在图像上绘制装甲板。
 * @param image 输入/输出图像。
 * @param armors 装甲板列表。
 */
void drawArmors(cv::Mat& image, const std::vector<Armor>& armors);

} // namespace armor_detector
