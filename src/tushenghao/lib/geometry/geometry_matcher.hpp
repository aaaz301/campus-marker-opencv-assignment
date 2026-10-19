#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "core/geometry_types.hpp"
#include "core/marker_geometry.hpp"
#include "mark/detector_config.hpp"

namespace mark
{

        GeometryBatch generateGeometryHypotheses(
            const std::vector<ShapeObservation> &observations,
            const MarkerGeometry &model_geometry,
            const GeometryConfig &config);

        std::vector<std::vector<ComponentAssignment>>
        generateSixComponentCombinations(
            const std::vector<ShapeObservation> &observations,
            const MarkerGeometry &model_geometry,
            const GeometryConfig &config,
            bool &resource_truncated);

        cv::Mat fitModelToImageAffine(
            const std::vector<ComponentAssignment> &assignment,
            const std::vector<ShapeObservation> &observations,
            const MarkerGeometry &model_geometry);

        GeometryHypothesis buildGeometryHypothesis(
            const std::vector<ComponentAssignment> &assignment,
            const cv::Mat &affine_transform,
            const std::vector<ShapeObservation> &observations,
            const MarkerGeometry &model_geometry);

} // namespace mark