#include "armor_detector/debug.hpp"

#include <filesystem>
#include <iostream>

#include <opencv2/imgcodecs.hpp>

namespace armor_detector {

namespace fs = std::filesystem;

void saveDebugImages(const DebugInfo& info, const std::string& output_dir) {
    if (info.binary_image.empty() && info.light_bar_image.empty() && info.armor_image.empty()) {
        return;
    }

    fs::create_directories(output_dir);

    const std::string binary_path = output_dir + "/01_binary.jpg";
    const std::string light_bar_path = output_dir + "/02_light_bars.jpg";
    const std::string armor_path = output_dir + "/03_armors.jpg";

    if (!info.binary_image.empty()) {
        if (!cv::imwrite(binary_path, info.binary_image)) {
            std::cerr << "[WARN] Failed to save: " << binary_path << std::endl;
        }
    }
    if (!info.light_bar_image.empty()) {
        if (!cv::imwrite(light_bar_path, info.light_bar_image)) {
            std::cerr << "[WARN] Failed to save: " << light_bar_path << std::endl;
        }
    }
    if (!info.armor_image.empty()) {
        if (!cv::imwrite(armor_path, info.armor_image)) {
            std::cerr << "[WARN] Failed to save: " << armor_path << std::endl;
        }
    }
}

} // namespace armor_detector
