#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "Detector.h"

namespace fs = std::filesystem;

// ======================= 每帧处理流程 =======================
// 输入一帧图像，完成二值化、灯条提取、装甲板匹配和结果绘制。
static cv::Mat processFrame(const cv::Mat& frame, const Params& p, cv::Mat* debugMask = nullptr) {
    cv::Mat binary = makeBinary(frame, p);
    if (debugMask) *debugMask = binary;

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    auto bars = findLightBars(binary, p);
    auto armors = matchArmors(bars, gray, p);

    cv::Mat result = frame.clone();
    drawResult(result, bars, armors);
    return result;
}

// ======================= 视频处理 =======================
// 读取视频文件并逐帧处理，输出结果视频，必要时显示调试窗口。
static void processVideo(const std::string& path, const Params& p, bool gui) {
    cv::VideoCapture cap(path);
    if (!cap.isOpened()) {
        std::cerr << "Cannot open: " << path << std::endl;
        return;
    }

    int w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (!(fps > 1.0)) fps = 30.0;

    fs::create_directories("output");
    std::string outPath = "output/" + fs::path(path).stem().string() + "_out.mp4";
    cv::VideoWriter writer(outPath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, cv::Size(w, h));
    if (!writer.isOpened()) {
        std::cerr << "Cannot create output: " << outPath << std::endl;
        return;
    }

    cv::Mat frame, mask;
    int idx = 0;
    while (cap.read(frame)) {
        cv::Mat result = processFrame(frame, p, gui ? &mask : nullptr);
        writer.write(result);

        if (gui) {
            cv::imshow("result", result);
            cv::imshow("binary", mask);
            if (cv::waitKey(1) == 27) break;  // ESC：退出当前视频处理
        }
        ++idx;
    }

    std::cout << path << " done, " << idx << " frames -> " << outPath << std::endl;
}

// ======================= 程序入口 =======================
// 支持命令行参数指定视频文件和 GUI 开关，并加载不同阈值参数。
int main(int argc, char** argv) {
    std::string pattern = "example1.mp4";
    bool gui = true;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        if (s == "--nogui") gui = false;
        else pattern = s;
    }

    std::vector<cv::String> videos;
    cv::glob(pattern, videos);
    if (videos.empty()) {
        std::cerr << "No video found for: " << pattern << std::endl;
        return 1;
    }

    Params p;
    if (gui) {
        cv::namedWindow("tune", cv::WINDOW_NORMAL);
        cv::createTrackbar("diff(B-R)", "tune", &p.diffThresh, 255);
        cv::createTrackbar("blueMin", "tune", &p.blueMin, 255);
        cv::createTrackbar("digitGray", "tune", &p.digitGray, 255);
    }

    for (const auto& v : videos) processVideo(v, p, gui);

    cv::destroyAllWindows();
    return 0;
}
