#pragma once

#include <algorithm>
#include <iostream>
#include <string>

namespace armor_detector {

/**
 * @brief 装甲板检测器的所有可调参数。
 */
struct DetectorParams {
    // HSV 阈值
    int red_hue_low_min{0};
    int red_hue_low_max{20};
    int red_hue_high_min{160};
    int red_hue_high_max{179};
    int blue_hue_min{100};
    int blue_hue_max{140};
    int saturation_min{80};
    int value_min{80};

    // 形态学
    int morph_kernel_size{3};
    int morph_iterations{1};
    bool fill_holes{true};  // 填充过曝灯条形成的"甜甜圈"空洞

    // 灯条筛选
    float min_light_bar_area{20.0f};
    float max_light_bar_area{5000.0f};
    float min_light_bar_ratio{1.5f};
    float max_light_bar_ratio{20.0f};
    float max_light_bar_angle{45.0f};

    // 装甲板配对
    float max_bar_height_diff_ratio{0.5f};
    float max_bar_angle_diff{15.0f};
    float min_armor_aspect_ratio{0.3f};
    float max_armor_aspect_ratio{4.0f};
    float max_armor_vertical_angle{20.0f};

    /**
     * @brief 验证并修正参数，确保 min <= max 且数值在合理范围内。
     * 只修复明显非法值，不限制正常调参范围。
     */
    void validate() {
        // HSV：hue 必须在 [0, 179]，sat/value 在 [0, 255]
        red_hue_low_min = clamp(red_hue_low_min, 0, 179);
        red_hue_low_max = clamp(red_hue_low_max, 0, 179);
        red_hue_high_min = clamp(red_hue_high_min, 0, 179);
        red_hue_high_max = clamp(red_hue_high_max, 0, 179);
        blue_hue_min = clamp(blue_hue_min, 0, 179);
        blue_hue_max = clamp(blue_hue_max, 0, 179);
        saturation_min = clamp(saturation_min, 0, 255);
        value_min = clamp(value_min, 0, 255);

        // 形态学：核大小必须为正奇数，迭代次数至少 1
        morph_kernel_size = std::max(1, morph_kernel_size);
        if (morph_kernel_size % 2 == 0) {
            ++morph_kernel_size;
        }
        morph_iterations = std::max(1, morph_iterations);

        // 灯条面积：确保 min <= max，且非负
        min_light_bar_area = std::max(0.0f, min_light_bar_area);
        max_light_bar_area = std::max(0.0f, max_light_bar_area);
        ensureMinMax(min_light_bar_area, max_light_bar_area);

        // 灯条长宽比：min <= max，最小为 1
        min_light_bar_ratio = std::max(1.0f, min_light_bar_ratio);
        max_light_bar_ratio = std::max(1.0f, max_light_bar_ratio);
        ensureMinMax(min_light_bar_ratio, max_light_bar_ratio);

        max_light_bar_angle = clamp(max_light_bar_angle, 0.0f, 90.0f);

        // 配对参数
        max_bar_height_diff_ratio = clamp(max_bar_height_diff_ratio, 0.0f, 5.0f);
        max_bar_angle_diff = clamp(max_bar_angle_diff, 0.0f, 90.0f);

        min_armor_aspect_ratio = std::max(0.1f, min_armor_aspect_ratio);
        max_armor_aspect_ratio = std::max(0.1f, max_armor_aspect_ratio);
        ensureMinMax(min_armor_aspect_ratio, max_armor_aspect_ratio);

        max_armor_vertical_angle = clamp(max_armor_vertical_angle, 0.0f, 90.0f);
    }

    // 调试
    bool save_images{true};
    std::string output_dir{"data/output"};

    /**
     * @brief 将当前参数保存到 YAML 文件。
     */
    void saveToYaml(const std::string& yaml_path) const;

private:
    template <typename T>
    static T clamp(T val, T min_val, T max_val) {
        if (val < min_val) {
            std::cerr << "[WARN] " << val << " is below " << min_val
                      << ". Clamping to " << min_val << "." << std::endl;
            return min_val;
        }
        if (val > max_val) {
            std::cerr << "[WARN] " << val << " is above " << max_val
                      << ". Clamping to " << max_val << "." << std::endl;
            return max_val;
        }
        return val;
    }

    template <typename T>
    static void ensureMinMax(T& min_val, T& max_val) {
        if (min_val > max_val) {
            std::cerr << "[WARN] min (" << min_val << ") > max (" << max_val
                      << "). Swapping." << std::endl;
            std::swap(min_val, max_val);
        }
    }
};

/**
 * @brief 从 YAML 文件加载参数。
 * @param yaml_path YAML 文件路径。
 * @return 加载后的参数。
 * @throw std::runtime_error 文件不存在或解析失败。
 */
DetectorParams loadParams(const std::string& yaml_path);



} // namespace armor_detector
