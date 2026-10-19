#pragma once

#include <string>

#include "armor_detector/params.hpp"

namespace armor_detector {

/**
 * @brief 参数调参器。
 *
 * 支持两种方式：
 * 1. YAML 热重载：修改 YAML 文件后程序自动加载，WSL 下最可靠。
 * 2. OpenCV trackbar：需要 GUI 环境，编译时定义 USE_TRACKBAR 宏启用。
 */
class ParamTuner {
public:
    /**
     * @brief 构造函数。
     * @param params 被调参数（指针，会被实时修改）。
     * @param window_name 窗口名（trackbar 使用）。
     */
    ParamTuner(DetectorParams& params, const std::string& window_name = "Tuner");

    /**
     * @brief 创建 OpenCV trackbar（仅在定义 USE_TRACKBAR 时生效）。
     */
    void createTrackbars();

    /**
     * @brief 检查 YAML 文件是否被修改，如果是则热重载参数。
     * @param yaml_path YAML 文件路径。
     * @return 是否实际执行了重载。
     */
    bool checkAndReload(const std::string& yaml_path);

    /**
     * @brief 立即从 YAML 重新加载参数。
     * @param yaml_path YAML 文件路径。
     */
    void reload(const std::string& yaml_path);

    /**
     * @brief 保存当前参数到 YAML。
     * @param yaml_path YAML 文件路径。
     */
    void save(const std::string& yaml_path) const;

private:
    DetectorParams* params_{nullptr};
    std::string window_name_;
    long long last_mtime_{0};
};

} // namespace armor_detector
