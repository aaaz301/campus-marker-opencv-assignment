#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <iostream>
#include <filesystem>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <climits>

namespace fs=std::filesystem;


struct FrameResult {
    cv::Mat annotated;
    bool marker_detected=false;
    std::vector<cv::Point> board_corners;
};

void drawBoard(cv::Mat& image, const std::vector<cv::Point>& corners) {
    for (int i = 0; i < 4; ++i) {
        cv::line(image, corners[i], corners[(i + 1) % 4],
                 cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
    }

    for (int i = 0; i < 4; ++i) {
        cv::circle(image, corners[i], 6,
                   cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
        cv::putText(image, "P" + std::to_string(i + 1),
                    corners[i] + cv::Point(8, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }
}

void drawStatus(cv::Mat& image, bool detected) {
    cv::putText(image,
                detected ? "Detected" : "Not Detected",
                cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.0,
                detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
                2, cv::LINE_AA);
}

void openVideoWriter(cv::VideoWriter& writer, const fs::path& path,
                     double fps, const cv::Size& size) {
    writer.open(path.string(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                fps, size, true);
    if (!writer.isOpened()) {
        throw std::runtime_error("无法创建视频: " + path.string());
    }
}

FrameResult process_frame(const cv::Mat& frame){
    FrameResult result;

    cv::Mat image_progress_denoise;
    cv::GaussianBlur(frame,image_progress_denoise,cv::Size{5,5},1.2);

    cv::Mat gray;
    cv::cvtColor(image_progress_denoise, gray, cv::COLOR_BGR2GRAY);
    cv::Mat image_progress_binary;
    cv::threshold(gray, image_progress_binary, 128, 255, cv::THRESH_BINARY);

    const cv::Mat open_kernel =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat image_progress_open_eroded;
    cv::erode(image_progress_binary, image_progress_open_eroded, open_kernel);
    cv::Mat image_progress_opened;
    cv::dilate(image_progress_open_eroded, image_progress_opened, open_kernel);
    const cv::Mat close_kernel =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat image_progress_close_dilated;
    cv::dilate(image_progress_opened, image_progress_close_dilated, close_kernel);
    cv::Mat image_progress_morphology;
    cv::erode(image_progress_close_dilated, image_progress_morphology, close_kernel);

    cv::Mat contour_input = image_progress_morphology.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(contour_input, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    cv::Mat image_progress_contours = frame.clone();
    cv::drawContours(image_progress_contours, contours, -1,
                     cv::Scalar(255, 255, 0), 1);

    std::vector<std::vector<cv::Point>> candidates;
    for (const auto& contour : contours){
        double area = cv::contourArea(contour);
        if (area<250.0) {
            continue;
        }
        std::vector<cv::Point> approx;
        double eps=0.03*cv::arcLength(contour,true);
        cv::approxPolyDP(contour,approx,eps,true);

        if (approx.size()<=8&&approx.size()>=4){       
            candidates.push_back(approx);
        }
    }
    
    if (candidates.size()>4){
        std::sort(candidates.begin(), candidates.end(),
        [](const auto& a, const auto& b){
                return cv::contourArea(a)>cv::contourArea(b);
            }
        );
        candidates.resize(4);
    }
    if (candidates.size() != 4){
        result.annotated=frame.clone();
        drawStatus(result.annotated,false); 
        return result;
    }

    std::vector<cv::Point> corners;
    for (const auto& poly : candidates){
        std::vector<cv::Point> hull;
        cv::convexHull(poly, hull);

        cv::Point concave_pt;
        bool found = false;
        for (const auto& p : poly){
            bool on_hull = false;
            for (const auto& h : hull){
                if (p==h){ 
                    on_hull=true; 
                    break; 
                }
            }
            if (!on_hull){
                concave_pt=p;
                found=true;
                break;
            }
        }
        if (found) corners.push_back(concave_pt);
    }

    cv::Point2f center(0, 0);
    for (const auto& p : corners) center += cv::Point2f(p);
    center *= 0.25f;

    std::sort(corners.begin(), corners.end(),
        [&](const cv::Point& a, const cv::Point& b) {
            return std::atan2(a.y - center.y, a.x - center.x)<std::atan2(b.y - center.y, b.x - center.x);
            }
        );
    
    result.annotated=frame.clone();
    if (corners.size() == 4) {
        cv::RotatedRect board = cv::minAreaRect(corners);
        float ratio = board.size.width / board.size.height;
        if (ratio > 0.6f && ratio < 1.6f) {
            result.marker_detected = true;
            result.board_corners=corners;
            drawBoard(result.annotated, corners);
        }
    }
    drawStatus(result.annotated, result.marker_detected);
    return result;
}

int main(int argc,char** argv){
    if (argc != 3) {
        std::cerr << "用法: " << argv[0] << " <输入视频.mp4> <输出目录>\n";
        return 1;
    }

    const fs::path input_path(argv[1]);
    const fs::path output_directory(argv[2]);
    if (!fs::is_regular_file(input_path)) {
            std::cout<<"输入视频不存在: " + input_path.string();
            return 1;
    }

    cv::VideoCapture capture(input_path.string());
    if(!capture.isOpened()){
        std::cout<<"无法读取输入视频: " + input_path.string();
    }
    fs::create_directories(output_directory);
    const std::string stem = input_path.stem().string();
    const fs::path video_path = output_directory / (stem + "_annotated.mp4");

    double fps = capture.get(cv::CAP_PROP_FPS);
    if (!std::isfinite(fps) || fps <= 0.0) {
        fps = 30.0;
    }

    cv::Mat frame;
    if (!capture.read(frame)) {
        std::cout<<"无法读取第一帧"<<std::endl;
        return 1;
    }
    cv::VideoWriter result_writer;
    openVideoWriter(result_writer, video_path, fps, frame.size());
    result_writer.write(frame);
    int total_fps=0;
    while(capture.read(frame)){
        ++total_fps;
        FrameResult result=process_frame(frame);
        result_writer.write(result.annotated);
    }

    result_writer.release();
    capture.release();
    return 0;
}