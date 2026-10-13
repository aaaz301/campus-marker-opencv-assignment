#include "armor_detector/image_utils.hpp"

#include <opencv2/imgproc.hpp>

namespace armor_detector {

cv::Mat preprocess(const cv::Mat& bgr, ColorType color, const DetectorParams& params) {
    // 1. BGR -> HSV
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // 2. 根据目标颜色生成掩码
    cv::Mat final_mask;
    if (color == ColorType::RED) {
        cv::Mat lower_mask, upper_mask;
        cv::inRange(hsv,
                    cv::Scalar(params.red_hue_low_min, params.saturation_min, params.value_min),
                    cv::Scalar(params.red_hue_low_max, 255, 255),
                    lower_mask);
        cv::inRange(hsv,
                    cv::Scalar(params.red_hue_high_min, params.saturation_min, params.value_min),
                    cv::Scalar(params.red_hue_high_max, 255, 255),
                    upper_mask);
        cv::bitwise_or(lower_mask, upper_mask, final_mask);
    } else if (color == ColorType::BLUE) {
        cv::inRange(hsv,
                    cv::Scalar(params.blue_hue_min, params.saturation_min, params.value_min),
                    cv::Scalar(params.blue_hue_max, 255, 255),
                    final_mask);
    } else {
        return cv::Mat();
    }

    // 3. 形态学：只做闭运算（dilate 后 erode）连接细条内部的小空洞，
    //    避免开运算把细灯条腐蚀断。
    if (params.morph_kernel_size > 0) {
        const int k = std::max(1, params.morph_kernel_size);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k));
        cv::morphologyEx(final_mask, final_mask, cv::MORPH_CLOSE, kernel,
                         cv::Point(-1, -1), std::max(1, params.morph_iterations));
    }

    // 4. 填充 mask 中的空洞：把过曝形成的"甜甜圈"填成实心。
    //    对视频/实际场景中的灯条尤其重要。
    if (params.fill_holes) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(final_mask.clone(), contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat filled = cv::Mat::zeros(final_mask.size(), CV_8UC1);
        for (std::size_t i = 0; i < contours.size(); ++i) {
            cv::drawContours(filled, contours, static_cast<int>(i), cv::Scalar(255), -1);
        }
        final_mask = filled;
    }

    return final_mask;
}

} // namespace armor_detector
