#include "armor_detector/light_bar.hpp"

#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace armor_detector {

float getNormalizedAngle(const cv::RotatedRect &rect){
    // OpenCV RotatedRect:
    //   angle ∈ [-90, 0)，表示 width 边与 x 轴的夹角。
    //   height 边与 x 轴的夹角 = angle + 90。
    // 目标是：返回长边与竖直方向的夹角，范围 [0, 90]。
    // 竖直长条 -> 0°，水平长条 -> 90°。
    float angle = rect.angle;
    float width = rect.size.width;
    float height = rect.size.height;

    // 长边与 x 轴的夹角
    float long_side_angle = (width > height) ? angle : (angle + 90.0f);

    // 归一化到 [0, 180)
    while (long_side_angle < 0.0f) {
        long_side_angle += 180.0f;
    }
    while (long_side_angle >= 180.0f) {
        long_side_angle -= 180.0f;
    }

    // 长边与竖直方向（90°）的夹角
    float vertical_angle = std::fabs(long_side_angle - 90.0f);
    if (vertical_angle > 90.0f) {
        vertical_angle = 180.0f - vertical_angle;
    }
    return vertical_angle;
}

float getRatio(const cv::RotatedRect &rect){
    float ratio;
    float long_side,short_side;
    long_side=rect.size.height>rect.size.width? rect.size.height:rect.size.width;
    short_side=rect.size.height<rect.size.width? rect.size.height:rect.size.width;

    ratio=long_side/short_side;
    return ratio;
}


std::vector<LightBar> detectLightBars(const cv::Mat& binary, const DetectorParams& params, ColorType color) {
    std::vector<LightBar> lightbars;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary.clone(), contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::cout << "[DEBUG] Found " << contours.size() << " contour(s)" << std::endl;

    for (auto& cnt : contours) {
        float area = cv::contourArea(cnt);
        if (area < params.min_light_bar_area || area > params.max_light_bar_area) {
            std::cout << "[DEBUG] Reject contour: area=" << area
                      << " not in [" << params.min_light_bar_area << ", "
                      << params.max_light_bar_area << "]" << std::endl;
            continue;
        }

        cv::RotatedRect rect = cv::minAreaRect(cnt);
        float normalized_angle = getNormalizedAngle(rect);
        float raw_ratio = rect.size.height / rect.size.width;
        if (normalized_angle > params.max_light_bar_angle) {
            std::cout << "[DEBUG] Reject contour: angle=" << normalized_angle
                      << " > " << params.max_light_bar_angle
                      << " (raw angle=" << rect.angle
                      << ", size=" << rect.size.width << "x" << rect.size.height
                      << ", raw_ratio=" << raw_ratio
                      << ", area=" << area << ")" << std::endl;
            continue;
        }

        float ratio = getRatio(rect);
        if (ratio < params.min_light_bar_ratio || ratio > params.max_light_bar_ratio) {
            std::cout << "[DEBUG] Reject contour: ratio=" << ratio
                      << " not in [" << params.min_light_bar_ratio << ", "
                      << params.max_light_bar_ratio << "]"
                      << " (raw angle=" << rect.angle
                      << ", size=" << rect.size.width << "x" << rect.size.height
                      << ", area=" << area << ")" << std::endl;
            continue;
        }

        LightBar bar;
        bar.rotated_rect = rect;
        bar.area = area;
        bar.angle = normalized_angle;
        bar.aspect_ratio = ratio;
        bar.center = rect.center;
        bar.color = color;
        lightbars.push_back(bar);
        std::cout << "[DEBUG] Light bar kept: area=" << bar.area
                  << ", ratio=" << bar.aspect_ratio
                  << ", angle=" << bar.angle
                  << ", raw_angle=" << rect.angle
                  << ", size=" << rect.size.width << "x" << rect.size.height
                  << ", center=(" << bar.center.x << ", " << bar.center.y << ")"
                  << std::endl;
    }

    std::sort(lightbars.begin(), lightbars.end(),
              [](const LightBar& a, const LightBar& b) { return a.area > b.area; });

    std::cout << "[DEBUG] Kept " << lightbars.size() << " light bar(s)" << std::endl;
    return lightbars;
}

void drawLightBars(cv::Mat& image, const std::vector<LightBar>& bars) {
   
    
    for(auto &b:bars){
        cv::Point2f vertices[4];
        b.rotated_rect.points(vertices);

        std::vector<std::vector<cv::Point>> polys(1);
        polys[0].assign(vertices, vertices + 4);

        // 画矩形
        cv::polylines(image, polys, true, cv::Scalar(0, 255, 0), 2);

        //画十字
        cv::line(image,
            cv::Point(b.rotated_rect.center.x-3,b.rotated_rect.center.y),
            cv::Point(b.rotated_rect.center.x+3,b.rotated_rect.center.y),
            cv::Scalar(0,255,0),
            2);
        cv::line(image,
            cv::Point(b.rotated_rect.center.x,b.rotated_rect.center.y-3),
            cv::Point(b.rotated_rect.center.x,b.rotated_rect.center.y+3),
            cv::Scalar(0,255,0),
            2);
        
        
    }
}

} // namespace armor_detector
