#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "armor_detector/capture.hpp"
#include "armor_detector/detector.hpp"
#include "armor_detector/params.hpp"
#include "armor_detector/tuner.hpp"
#include "armor_detector/video_writer.hpp"

namespace fs = std::filesystem;
using namespace armor_detector;

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name
              << " <params.yaml> <source> [red|blue] [output.mp4]" << std::endl;
    std::cout << "  source: image path, video path, or camera index (e.g. 0)" << std::endl;
    std::cout << "  output.mp4: optional, save processed video" << std::endl;
}

ColorType parseColor(const std::string& color_arg) {
    if (color_arg == "blue" || color_arg == "Blue" || color_arg == "BLUE") {
        return ColorType::BLUE;
    }
    return ColorType::RED;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    const std::string params_path = argv[1];
    const std::string source = argv[2];
    const ColorType target_color = (argc > 3) ? parseColor(argv[3]) : ColorType::RED;
    const std::string output_video = (argc > 4) ? argv[4] : "";

    try {
        DetectorParams params = loadParams(params_path);
        params.validate();

        // 如果 output_dir 是相对路径，以 YAML 文件所在目录为基准解析为绝对路径。
        // 若 YAML 放在 config/ 子目录下，则退到项目根目录，避免输出到 config/data/output。
        if (!params.output_dir.empty() && !fs::path(params.output_dir).is_absolute()) {
            fs::path yaml_dir = fs::path(params_path).parent_path();
            if (yaml_dir.filename() == "config") {
                yaml_dir = yaml_dir.parent_path();
            }
            const fs::path output_full = yaml_dir / params.output_dir;
            fs::create_directories(output_full);
            params.output_dir = fs::canonical(output_full).string();
        }

        Capture capture;
        if (!capture.open(source)) {
            std::cerr << "[ERROR] Failed to open source: " << source << std::endl;
            return 1;
        }

        ArmorDetector detector(params);
        detector.setTargetColor(target_color);

        ParamTuner tuner(params);
        tuner.reload(params_path);  // 初始化 mtime
        tuner.createTrackbars();

        VideoWriter writer;
        if (!output_video.empty()) {
            writer.open(output_video, capture.fps() > 0.0 ? capture.fps() : 30.0);
        }

        if (params.save_images) {
            std::cout << "[INFO] Debug images will be saved to: "
                      << fs::absolute(params.output_dir) << std::endl;
        }

        std::cout << "[INFO] Press 'q' to quit, 's' to save params, 'r' to reload params." << std::endl;

        cv::Mat frame;
        int frame_count = 0;
        auto last_fps_time = std::chrono::steady_clock::now();
        const bool is_image = (capture.type() == InputType::IMAGE);

        while (capture.read(frame)) {
            if (frame.empty()) {
                std::cerr << "[WARN] Empty frame, skipping." << std::endl;
                continue;
            }

            // YAML 热重载
            if (tuner.checkAndReload(params_path)) {
                detector.setParams(params);
            }
            detector.setTargetColor(target_color);

            // 检测
            std::vector<Armor> armors = detector.detect(frame);
            ++frame_count;

            const DebugInfo& debug = detector.getDebugInfo();

            // 保存调试图
            if (params.save_images) {
                saveDebugImages(debug, params.output_dir);
            }

            // 写入输出视频
            if (!output_video.empty() && !debug.armor_image.empty()) {
                writer.write(debug.armor_image);
            }

            // 统计 FPS
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time).count();
            if (elapsed >= 1000) {
                double fps = frame_count * 1000.0 / elapsed;
                std::cout << "[INFO] FPS: " << fps << "  Armors: " << armors.size() << std::endl;
                frame_count = 0;
                last_fps_time = now;
            }

            // 图片输入：处理完一帧直接退出，不需要 waitKey
            if (is_image) {
                std::cout << "[INFO] Image processed. Armors: " << armors.size() << std::endl;
                break;
            }

            // 键盘事件（仅视频/摄像头）
            const int key = cv::waitKey(1);
            if (key == 'q' || key == 27) {  // q 或 ESC
                break;
            } else if (key == 's') {
                tuner.save(params_path);
            } else if (key == 'r') {
                tuner.reload(params_path);
                detector.setParams(params);
            }
        }

        std::cout << "[INFO] Done." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
