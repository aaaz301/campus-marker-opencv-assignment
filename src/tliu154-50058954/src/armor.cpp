#include "armor_detector/armor.hpp"

#include <cmath>

#include <opencv2/imgproc.hpp>

namespace armor_detector {

namespace {

/**
 * @brief 从旋转矩形四个顶点中取出 y 最小（最上）和 y 最大（最下）的两个点。
 * 用于估算灯条的真实上下端点，而不是用可能不准确的 size.width/height。
 */
std::pair<cv::Point2f, cv::Point2f> getTopAndBottomPoints(const cv::RotatedRect& rect) {
    cv::Point2f pts[4];
    rect.points(pts);

    cv::Point2f top = pts[0];
    cv::Point2f bottom = pts[0];
    for (int i = 1; i < 4; ++i) {
        if (pts[i].y < top.y) {
            top = pts[i];
        }
        if (pts[i].y > bottom.y) {
            bottom = pts[i];
        }
    }
    return {top, bottom};
}

} // namespace

std::vector<Armor> matchArmors(const std::vector<LightBar>& bars, const DetectorParams& params) {
    std::vector<Armor> armors;
    const std::size_t bar_count = bars.size();
    if (bar_count < 2U) {
        std::cout << "[DEBUG] matchArmors: only " << bar_count << " bar(s), skip pairing" << std::endl;
        return armors;
    }

    std::cout << "[DEBUG] matchArmors: trying to pair " << bar_count << " bars" << std::endl;

    for (std::size_t i = 0; i + 1U < bar_count; ++i) {
        for (std::size_t j = i + 1U; j < bar_count; ++j) {
            // 高度差
            const float height_i = bars[i].rotated_rect.size.height;
            const float height_j = bars[j].rotated_rect.size.height;
            const float avg_height = (height_i + height_j) * 0.5f;
            const float y_diff = std::fabs(bars[i].center.y - bars[j].center.y);
            if (y_diff > params.max_bar_height_diff_ratio * avg_height) {
                std::cout << "[DEBUG] Pair " << i << "," << j << " rejected: y_diff=" << y_diff
                          << " > " << params.max_bar_height_diff_ratio * avg_height << std::endl;
                continue;
            }

            // 角度差
            const float angle_diff = std::fabs(bars[i].angle - bars[j].angle);
            if (angle_diff > params.max_bar_angle_diff) {
                std::cout << "[DEBUG] Pair " << i << "," << j << " rejected: angle_diff=" << angle_diff
                          << " > " << params.max_bar_angle_diff << std::endl;
                continue;
            }

            // 确定左右
            const LightBar& left_bar = bars[i].center.x < bars[j].center.x ? bars[i] : bars[j];
            const LightBar& right_bar = bars[j].center.x > bars[i].center.x ? bars[j] : bars[i];

            // 装甲板长宽比
            const float armor_width = std::fabs(right_bar.center.x - left_bar.center.x);
            const float armor_height = (left_bar.rotated_rect.size.height + right_bar.rotated_rect.size.height) * 0.5f;
            if (armor_height < 1.0f) {
                continue;
            }
            const float armor_aspect = armor_width / armor_height;
            if (armor_aspect < params.min_armor_aspect_ratio ||
                armor_aspect > params.max_armor_aspect_ratio) {
                std::cout << "[DEBUG] Pair " << i << "," << j << " rejected: aspect=" << armor_aspect
                          << " not in [" << params.min_armor_aspect_ratio << ", "
                          << params.max_armor_aspect_ratio << "]" << std::endl;
                continue;
            }

            Armor armor;
            armor.left_bar = left_bar;
            armor.right_bar = right_bar;
            armor.color = left_bar.color;

            // 用灯条上下端点构造装甲板四角
            const auto left_points = getTopAndBottomPoints(left_bar.rotated_rect);
            const auto right_points = getTopAndBottomPoints(right_bar.rotated_rect);

            armor.vertices[0] = left_points.first;   // 左上
            armor.vertices[1] = right_points.first;  // 右上
            armor.vertices[2] = right_points.second; // 右下
            armor.vertices[3] = left_points.second;  // 左下

            armors.push_back(armor);
            std::cout << "[DEBUG] Pair " << i << "," << j << " accepted: aspect=" << armor_aspect << std::endl;
        }
    }

    std::cout << "[DEBUG] matchArmors: found " << armors.size() << " armor(s)" << std::endl;
    return armors;
}

void drawArmors(cv::Mat& image, const std::vector<Armor>& armors) {
    for (std::size_t i = 0; i < armors.size(); ++i) {
        const auto& armor = armors[i];

        // 根据装甲板颜色选画框颜色：红色用红，蓝色用蓝
        const cv::Scalar color = (armor.color == ColorType::BLUE)
                                     ? cv::Scalar(255, 0, 0)
                                     : cv::Scalar(0, 0, 255);

        // 画四边形
        cv::line(image, armor.vertices[0], armor.vertices[1], color, 2);
        cv::line(image, armor.vertices[1], armor.vertices[2], color, 2);
        cv::line(image, armor.vertices[2], armor.vertices[3], color, 2);
        cv::line(image, armor.vertices[3], armor.vertices[0], color, 2);

        // 标序号，放在左灯条中心上方
        const auto label = std::to_string(i);
        const cv::Point text_pos(
            static_cast<int>(armor.left_bar.center.x) - 5,
            static_cast<int>(armor.left_bar.center.y) - 10);
        cv::putText(image, label, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 0), 2);
    }
}

} // namespace armor_detector
