#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "armor_detector/debug.hpp"
#include "armor_detector/params.hpp"
#include "armor_detector/types.hpp"

namespace armor_detector {

/**
 * @brief 装甲板检测器。
 */
class ArmorDetector {
public:
    /**
     * @brief 构造函数。
     * @param params 检测参数。
     */
    explicit ArmorDetector(const DetectorParams& params);

    /**
     * @brief 对单帧图像进行检测。
     * @param image 输入图像（BGR）。
     * @return 检测到的装甲板列表。
     */
    std::vector<Armor> detect(const cv::Mat& image);

    /**
     * @brief 设置目标颜色。
     * @param color 目标颜色。
     */
    void setTargetColor(ColorType color);

    /**
     * @brief 更新检测参数（热重载时使用）。
     * @param params 新的检测参数。
     */
    void setParams(const DetectorParams& params);

    /**
     * @brief 获取上一次检测的中间结果。
     * @return 调试信息。
     */
    const DebugInfo& getDebugInfo() const;

private:
    DetectorParams params_;
    ColorType target_color_{ColorType::RED};
    DebugInfo debug_info_;
};

} // namespace armor_detector
