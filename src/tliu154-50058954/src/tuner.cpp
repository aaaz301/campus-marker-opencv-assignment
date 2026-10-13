#include "armor_detector/tuner.hpp"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <map>

#ifdef USE_TRACKBAR
#include <opencv2/highgui.hpp>
#endif

namespace armor_detector {

namespace fs = std::filesystem;

// ==================== trackbar 支持（可选） ====================
#ifdef USE_TRACKBAR

#include <memory>

struct TrackbarState {
    enum class Type { INT, FLOAT } type;
    void* ptr{nullptr};
    float scale{1.0f};
    int int_value{0};
};

static std::map<std::string, std::unique_ptr<TrackbarState>> g_states;

static void onTrackbar(int value, void* userdata) {
    const std::string& name = *static_cast<const std::string*>(userdata);
    auto it = g_states.find(name);
    if (it == g_states.end() || it->second == nullptr) {
        return;
    }

    it->second->int_value = value;
    if (it->second->type == TrackbarState::Type::INT) {
        *static_cast<int*>(it->second->ptr) = value;
    } else {
        *static_cast<float*>(it->second->ptr) = static_cast<float>(value) / it->second->scale;
    }
}

static void addIntTrackbar(const std::string& window, const std::string& name, int& value, int max_value) {
    auto state = std::make_unique<TrackbarState>();
    state->type = TrackbarState::Type::INT;
    state->ptr = &value;
    state->scale = 1.0f;
    state->int_value = value;

    auto [it, inserted] = g_states.emplace(name, std::move(state));
    cv::createTrackbar(name, window, &it->second->int_value, max_value,
                       onTrackbar, const_cast<std::string*>(&it->first));
}

static void addFloatTrackbar(const std::string& window, const std::string& name, float& value,
                             float max_value, float scale) {
    auto state = std::make_unique<TrackbarState>();
    state->type = TrackbarState::Type::FLOAT;
    state->ptr = &value;
    state->scale = scale;
    state->int_value = static_cast<int>(value * scale);

    const int max_int = static_cast<int>(max_value * scale);
    auto [it, inserted] = g_states.emplace(name, std::move(state));
    cv::createTrackbar(name, window, &it->second->int_value, max_int,
                       onTrackbar, const_cast<std::string*>(&it->first));
}

#endif // USE_TRACKBAR

// ==================== ParamTuner 实现 ====================

ParamTuner::ParamTuner(DetectorParams& params, const std::string& window_name)
    : params_(&params), window_name_(window_name) {}

void ParamTuner::createTrackbars() {
#ifdef USE_TRACKBAR
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);

    addIntTrackbar(window_name_, "red_hue_low_min", params_->red_hue_low_min, 180);
    addIntTrackbar(window_name_, "red_hue_low_max", params_->red_hue_low_max, 180);
    addIntTrackbar(window_name_, "red_hue_high_min", params_->red_hue_high_min, 180);
    addIntTrackbar(window_name_, "red_hue_high_max", params_->red_hue_high_max, 180);
    addIntTrackbar(window_name_, "blue_hue_min", params_->blue_hue_min, 180);
    addIntTrackbar(window_name_, "blue_hue_max", params_->blue_hue_max, 180);
    addIntTrackbar(window_name_, "saturation_min", params_->saturation_min, 255);
    addIntTrackbar(window_name_, "value_min", params_->value_min, 255);

    addIntTrackbar(window_name_, "morph_kernel_size", params_->morph_kernel_size, 21);
    addIntTrackbar(window_name_, "morph_iterations", params_->morph_iterations, 10);

    addFloatTrackbar(window_name_, "min_light_bar_area", params_->min_light_bar_area, 5000.0f, 1.0f);
    addFloatTrackbar(window_name_, "max_light_bar_area", params_->max_light_bar_area, 15000.0f, 1.0f);
    addFloatTrackbar(window_name_, "min_light_bar_ratio", params_->min_light_bar_ratio, 20.0f, 10.0f);
    addFloatTrackbar(window_name_, "max_light_bar_ratio", params_->max_light_bar_ratio, 50.0f, 10.0f);
    addFloatTrackbar(window_name_, "max_light_bar_angle", params_->max_light_bar_angle, 90.0f, 1.0f);

    addFloatTrackbar(window_name_, "max_bar_height_diff_ratio", params_->max_bar_height_diff_ratio, 1.0f, 100.0f);
    addFloatTrackbar(window_name_, "max_bar_angle_diff", params_->max_bar_angle_diff, 90.0f, 1.0f);
    addFloatTrackbar(window_name_, "min_armor_aspect_ratio", params_->min_armor_aspect_ratio, 10.0f, 10.0f);
    addFloatTrackbar(window_name_, "max_armor_aspect_ratio", params_->max_armor_aspect_ratio, 10.0f, 10.0f);
    addFloatTrackbar(window_name_, "max_armor_vertical_angle", params_->max_armor_vertical_angle, 90.0f, 1.0f);
#else
    std::cout << "[INFO] Trackbar disabled. Use YAML hot-reload instead (modify params and save)."
              << std::endl;
#endif
}

bool ParamTuner::checkAndReload(const std::string& yaml_path) {
    if (params_ == nullptr || !fs::exists(yaml_path)) {
        return false;
    }

    const auto mtime = fs::last_write_time(yaml_path).time_since_epoch().count();
    if (mtime == last_mtime_) {
        return false;
    }

    last_mtime_ = mtime;
    reload(yaml_path);
    return true;
}

void ParamTuner::reload(const std::string& yaml_path) {
    if (params_ == nullptr) {
        return;
    }

    try {
        *params_ = loadParams(yaml_path);
        std::cout << "[INFO] Params reloaded from " << yaml_path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[WARN] Failed to reload params: " << e.what() << std::endl;
    }
}

void ParamTuner::save(const std::string& yaml_path) const {
    if (params_ == nullptr) {
        return;
    }

    params_->saveToYaml(yaml_path);
    std::cout << "[INFO] Params saved to " << yaml_path << std::endl;
}

} // namespace armor_detector
