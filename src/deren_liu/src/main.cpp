#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

constexpr int kThreshold = 200;
constexpr double kContourArea = 1000.0;//这两个暂时没用，最终可能删
constexpr double kRectArea = 4000.0;
constexpr float kRatio = 1.2;
constexpr double kMinFill = 0.3;
constexpr double kMaxFill = 0.8;
constexpr float kMinAngle = 20.0;

namespace{

enum class Orientation {
    TopLeft,
    TopRight,
    BottomRight,
    BottomLeft
};

std::string orientationToString(Orientation o) {
    switch (o) {
        case Orientation::TopLeft:     return "LT";
        case Orientation::TopRight:    return "RT";
        case Orientation::BottomRight: return "RB";
        case Orientation::BottomLeft:  return "LB";
    }
    return "?";
}

const cv::Point2f kDirectionVec[4] = {
    {-1, -1},  //左上
    { 1, -1},  //右上
    { 1,  1},  //右下
    {-1,  1}   //左下
};

struct LightBar{
    std::vector<cv::Point> contour;
    cv::RotatedRect rectangle;
    float angle_degrees = 0.0F;
    Orientation orientation;
    cv::Point big_rect_vertex;
};

cv::Point pixelPoint(const cv::Point2f& point){
    return cv::Point{cvRound(point.x), cvRound(point.y)};
}

//矩形筛选
//条件为 1.长宽比 2.填充率 3.角度
bool makeLightBar(const std::vector<cv::Point>& contour, LightBar& bar){
    const cv::RotatedRect rectangle = cv::minAreaRect(contour);
    const float length = std::max(rectangle.size.height, rectangle.size.width);
    const float width = std::min(rectangle.size.height, rectangle.size.width);
    const float ratio = length / width;
    if(width < 0.0F || length < 0.0F || ratio > kRatio){
        return false;
    }

    const double fill = cv::contourArea(contour) / (length * width);
    if(fill < kMinFill || fill > kMaxFill){
        return false;
    }

    cv::Point2f vertices[4];
    rectangle.points(vertices);
    const cv::Point2f edge = vertices[1] - vertices[0];
    float angle_degrees =std::atan2(edge.y , edge.x) * 180.0F / CV_PI;
    //把角度范围转为 -45 ~ 45
    while(angle_degrees < -45.0F) angle_degrees += 90.0F;
    while(angle_degrees > 45.0F) angle_degrees -= 90.0F;
    if(std::abs(angle_degrees) > kMinAngle){
        return false;
    }

    bar.contour = contour;
    bar.rectangle = rectangle;
    bar.angle_degrees = angle_degrees;
    return true;
} 

//函数里同时判断L形朝向和所需的矩形顶点
void classifyOrientation(LightBar& bar){
    //凸包
    std::vector<int> hull_indices;
    cv::convexHull(bar.contour, hull_indices, false);
    std::vector<cv::Vec4i> defects;
    cv::convexityDefects(bar.contour, hull_indices, defects);

    //deepest_point 即为最凸点，
    //之后要用矩形中心到最凸点的向量判断L形的朝向
    float max_depth = 0.0F;
    cv::Point deepest_point;
    for(const auto& d : defects){
        float depth = d[3] / 256.0F;
        if(depth > max_depth){
            max_depth = depth;
            deepest_point = bar.contour[d[2]];
        }
    }

    cv::Point2f deepest_point2f = deepest_point;
    cv::Point2f unit_direction =  deepest_point2f - bar.rectangle.center;
    float len = std::sqrt(unit_direction.dot(unit_direction));
    int direction_index;
    if (len < 1e-6F){
        direction_index = 0;   //防除零，随便返回一个
    } 
    else{
        unit_direction /= len;
        float best= -1000.0F; //-INF
        for(int i = 0; i < 4; ++i){
            float temp = unit_direction.dot(kDirectionVec[i]);
            if(temp > best){
                best = temp;
                direction_index = i;
            }
        }
    }
    bar.orientation = static_cast<Orientation>(direction_index);

    cv::Point2f vertices[4];
    bar.rectangle.points(vertices);
    float best= -1000.0F; //-INF
    for(int i = 0; i < 4; ++i){
        cv::Point2f unit_direction =  vertices[i] - bar.rectangle.center;
        float len = std::sqrt(unit_direction.dot(unit_direction));
        unit_direction /= len;

        float temp = unit_direction.dot(kDirectionVec[direction_index]);
        if(temp > best){
            best = temp;
            bar.big_rect_vertex = vertices[i];
        }
    }
    
}

void drawRotatedRect(cv::Mat image, const cv::RotatedRect& rectangle,
                     const cv::Scalar& color, int thickness = 2){
    cv::Point2f vertices[4];
    rectangle.points(vertices);
    for(int i = 0; i < 4; ++i){
        cv::line(image, pixelPoint(vertices[i]), pixelPoint(vertices[(i + 1) % 4]),
                 color, thickness, cv::LINE_AA);
    }
}   

cv::Mat processFrame(const cv::Mat& frame){
    //降噪
    cv::Mat image_denoise;
    cv::medianBlur(frame, image_denoise, 3);

    //分离通道 + 二值化
    //这里通过取bgr的最小值来筛选
    std::vector<cv::Mat> bgr_channels;
    cv::split(image_denoise, bgr_channels);
    cv::Mat min_bg, min_all;
    cv::min(bgr_channels[0], bgr_channels[1], min_bg);
    cv::min(bgr_channels[2], min_bg, min_all);
    cv::Mat image_binary;
    cv::threshold(min_all, image_binary, kThreshold, 255, cv::THRESH_BINARY);

    //开运算
    const cv::Mat open_kernel = 
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::Mat image_opened;
    cv::morphologyEx(image_binary, image_opened, cv::MORPH_OPEN, open_kernel);
    //闭运算
    //目前想法是先把右上角的断点闭合，使灯板变成四个相似的L形
    const cv::Mat close_kernel = 
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25,25));
    cv::Mat image_closed;
    cv::morphologyEx(image_opened, image_closed, cv::MORPH_CLOSE, close_kernel);

    //轮廓提取
    cv::Mat contour_input = image_closed.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(contour_input, contours, cv::RETR_EXTERNAL, 
                     cv::CHAIN_APPROX_SIMPLE);
    
    //旋转矩形拟合
    //这一步同时进行矩形筛选
    //条件为 1.长宽比 2.填充率 3.角度
    std::vector<LightBar> bars;
    cv::Mat result = frame.clone();
    for(const auto& contour : contours){
        LightBar bar;
        if(makeLightBar(contour, bar)){
            drawRotatedRect(result, cv::minAreaRect(contour),
                            cv::Scalar(255, 255, 0), 1);
            bars.push_back(bar);
        }
    }

    //判断L形朝向
    for(LightBar& bar : bars){
        classifyOrientation(bar);
    }

    //生成大矩形轮廓，顶点与顶点标识
    //前面画的小矩形轮廓也保留了
    if(bars.size() == 4){
        cv::Point2f vertices[4];
        for(const LightBar& bar : bars){
            //顶点标识： LT、RT、RB、LB
            cv::Point lable_position;
            if(static_cast<int>(bar.orientation) <= 1){
                //Top
                lable_position = pixelPoint(bar.big_rect_vertex) + cv::Point(-10, -10);
            }else{
                //Bottom
                lable_position = pixelPoint(bar.big_rect_vertex) + cv::Point(-10, 20);
            }
            cv::putText(result, orientationToString(bar.orientation),
                        lable_position, 
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            
            //顶点
            cv::circle(result, bar.big_rect_vertex,
                       5, cv::Scalar(0, 0, 255), -1);
            
            vertices[static_cast<int>(bar.orientation)] = bar.big_rect_vertex;
        }
        for(int i = 0; i < 4; ++i){
            //大矩形的边
            cv::line(result, vertices[i], vertices[(i+1)%4],
                     cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        }
    }
    else{
        const int center_x = result.cols / 2;
        const int center_y = result.rows / 2;
        cv::putText(result, "Not Found", cv::Point(center_x, center_y),
                    cv::FONT_HERSHEY_SIMPLEX, 1, 
                    cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
    return result;
}

void openVideoWriter(cv::VideoWriter& writer, const fs::path& path,
                     double fps, const cv::Size& size){
    writer.open(path.string(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                fps, size, true);
    if(!writer.isOpened()){
        throw std::runtime_error("无法创建视频： " + path.string());
    }
}

} //namespace

int main(int argc, char** argv){
    if(argc != 3){
        std::cerr << "用法： " << argv[0] << " <输入视频> <输出目录>\n";
        return 1;
    }
    try{
        const fs::path input_path = argv[1];
        const fs::path output_directory = argv[2]; 
        if(!fs::is_regular_file(input_path)){
            throw std::runtime_error("输入视频不存在： " + input_path.string());
        }
        cv::VideoCapture capture(input_path.string());
        if(!capture.isOpened()){
            throw std::runtime_error("无法读取输入视频： " + input_path.string());
        }
        fs::create_directories(output_directory);
        fs::path output_path = output_directory / "demo.mp4";

        double fps = capture.get(cv::CAP_PROP_FPS);
        if(!std::isfinite(fps) || fps <= 0.0){
            fps = 30.0;
        }
        
        int frame_index = 0;
        cv::VideoWriter writer;
        cv::Mat frame;
        while(capture.read(frame)){
            if(frame.empty()){
                throw std::runtime_error("视频中出现空帧");
            }
            cv::Mat result_bgr = processFrame(frame);
            // cv::Mat result_bgr;
            // cv::cvtColor(result, result_bgr, cv::COLOR_GRAY2BGR);

            if(!writer.isOpened()){
                openVideoWriter(writer, output_path, fps, frame.size());
            }
            writer.write(result_bgr);
            ++frame_index;
        }
        if(frame_index == 0){
            throw std::runtime_error("视频没有可读取的帧");
        }
        writer.release();
        capture.release();
        return 0;
    }catch(const std::exception& error){
        std::cerr << "错误： " << error.what() << "\n";
        return 1;
    }
}
