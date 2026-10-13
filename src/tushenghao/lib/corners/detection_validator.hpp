#pragma once

#include "corners/corner_types.hpp"

namespace mark
{

    /**
     * @brief 最终几何质检接口
     *
     * 负责检查：
     *
     * - 四角测量结果是否满足几何约束；
     * - 屏幕排序映射是否有效；
     * - 当前角点是否可以作为最终检测结果依据。
     *
     * 输入：
     *
     * CornerMeasurement
     *      +
     * ScreenOrder
     *
     * 输出：
     *
     * DetectionValidation
     *
     * 注意：
     *
     * 这里只做验证。
     *
     * 不负责：
     *
     * - 从图像重新检测角点；
     * - 修改物理角身份；
     * - 重新排序屏幕点；
     * - 创建 Detection 对象；
     * - 调用 decode。
     */
    DetectionValidation validateDetectionGeometry(
        const CornerMeasurement &measurement,
        const ScreenOrder &order,
        cv::Size original_size,
        const CornerConfig &config);

} // namespace mark