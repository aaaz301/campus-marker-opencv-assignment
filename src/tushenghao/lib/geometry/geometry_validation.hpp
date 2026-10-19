// step7:独立验证（输入输出接口:验证+输出）
#pragma once

#include "core/geometry_types.hpp"
#include "core/marker_geometry.hpp"
#include "mark/detector_config.hpp"

#include <vector>

namespace mark
{
    GeometryBatch validateGeometryBatch(
        const GeometryBatch &batch,
        const MarkerGeometry &geometry,
        const std::vector<WhiteComponent> &components,
        const GeometryConfig &config);

} // namespace mark