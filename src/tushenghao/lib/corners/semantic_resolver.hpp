#pragma once

#include <vector>

#include "corners/corner_types.hpp"

namespace mark
{

    /**
     * @brief 语义归并接口
     *
     * 负责处理多个 CornerMeasurement 之间的解释关系。
     *
     * 它回答：
     *
     * 1. 多个几何测量是否描述同一个目标；
     * 2. 当前方向是否可以唯一确定；
     * 3. 哪些测量假设应该保留。
     *
     * 它不负责：
     *
     * - 从图像中检测角点；
     * - 拟合边和计算交点；
     * - 屏幕顺序排序。
     *
     * 输入：
     *
     * CornerMeasurement
     *      ↓
     * 语义归并
     *      ↓
     * SemanticResolution
     */
    SemanticResolution resolveSemantics(
        const std::vector<CornerMeasurement> &measurements,
        bool search_truncated,
        const CornerConfig &config);

} // namespace mark