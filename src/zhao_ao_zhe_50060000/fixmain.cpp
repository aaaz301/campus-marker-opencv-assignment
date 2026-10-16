#include <filesystem>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>


struct FrameResult{
bool detected=false;
std::vector<cv::Point2f> keypoints;
cv::Mat debugimage;
cv::RotatedRect boundingBox; 
};
namespace{

FrameResult processFrame(const cv::Mat& frame){
FrameResult result;
result.debugimage = frame.clone();
cv::Mat image_processed= frame.clone();
cv::Mat gray;
cv::cvtColor(image_processed,gray,cv::COLOR_BGR2GRAY);
cv::medianBlur(gray,gray,5);

cv::Mat binary;
cv::threshold(gray,binary,200,255,cv::THRESH_BINARY);


cv::Mat image_processed_morphology;
cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
cv::morphologyEx(binary, image_processed_morphology, cv::MORPH_CLOSE, close_kernel);
cv::morphologyEx(image_processed_morphology, image_processed_morphology, cv::MORPH_OPEN, open_kernel);


std::vector<std::vector<cv::Point>>counters;
cv::findContours(image_processed_morphology,counters,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);


std::vector<cv::Point2f> all_L_corners;
for(const auto& contour:counters){if (cv::contourArea(contour) >= 50.0){
   cv::RotatedRect rotRect = cv::minAreaRect(contour);
   cv::Point2f pts[4];
   rotRect.points(pts);
   for (int i = 0; i < 4; i++) {
            all_L_corners.push_back(pts[i]);
        }
    }
}
if(all_L_corners.size()>=12){
    result.detected=true;
    cv::RotatedRect bigRect=cv::minAreaRect(all_L_corners);
    result.boundingBox = bigRect;
    cv::Point2f vertices[4];
    bigRect.points(vertices);
    std::vector<float>sum(4),diff(4);
    for(int i=0;i<4;i++){
        sum[i]=vertices[i].x+vertices[i].y;
        diff[i]=vertices[i].x-vertices[i].y;
    }

    int id_t1=std::distance(sum.begin(),std::min_element(sum.begin(),sum.end()));
    int id_t2=std::distance(diff.begin(),std::max_element(diff.begin(),diff.end()));
    int id_t3=std::distance(sum.begin(),std::max_element(sum.begin(),sum.end()));
    int id_t4=std::distance(diff.begin(),std::min_element(diff.begin(),diff.end()));

    result.keypoints={vertices[id_t1],vertices[id_t2],vertices[id_t3],vertices[id_t4]};
    result.debugimage=frame.clone();
    cv::Mat &output_image=result.debugimage;

    for(int i=0;i<4;i++){
        cv::line(output_image,result.keypoints[i],result.keypoints[(i+1)%4],cv::Scalar(0,255,0),2);
        std::string label=std::to_string(i+1);
        cv::putText(output_image,label,result.keypoints[i],cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(255,0,0),2);

    }
    
    }
return result;  
}
}
int main(int argc,char* argv[]){
if (argc != 3) {
    std::cerr << "用法: " << argv[0] << " <输入视频.avi> <输出目录>\n";
    return 1;
}
try{
if(!std::filesystem::is_regular_file(argv[1])){
    std::cerr << "错误:输入视频文件不存在。" << std::endl;
    return -1;}

cv::VideoCapture cap(argv[1]);
if (!cap.isOpened()) {
    std::cerr << "错误:打不开视频文件。" << std::endl;
    return -1;}


double fps =cap.get(cv::CAP_PROP_FPS);
int width =static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
int height =static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
if(!(fps>0&&fps<=60.0)){
        fps =30.0;
    }
std::filesystem::path output_dir(argv[2]);
if (!std::filesystem::exists(output_dir)) {
    std::filesystem::create_directories(output_dir);
    }
std::string right_path = (output_dir / "right.avi").string();
std::string wrong_path = (output_dir / "wrong.avi").string();
int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
cv::VideoWriter right(right_path, fourcc, fps, cv::Size(width, height));
cv::VideoWriter wrong(wrong_path, fourcc, fps, cv::Size(width, height));
if (!right.isOpened() || !wrong.isOpened()) {
            std::cerr << "错误:无法创建输出视频文件，请检查输出目录权限。" << std::endl;
            return -1;
        }

cv::Mat frame;
int frame_count = 0;
while (cap.read(frame)) {
            frame_count++;
            FrameResult result = processFrame(frame);
            if (result.detected) {
                right.write(result.debugimage); // 识别成功，写入 right.avi
                std::cout << "第 " << frame_count << " 帧：检测到标志物，已写入 right.avi" << std::endl;
            } else {
                wrong.write(result.debugimage); // 未识别/识别错误，写入 wrong.avi
                std::cout << "第 " << frame_count << " 帧：未检测到目标，已写入 wrong.avi" << std::endl;
            }
             if (!result.debugimage.empty()) {
                cv::imshow("Result", result.debugimage);
            }
            if (cv::waitKey(static_cast<int>(1000 / fps)) == 27) {
                break;
            }
        }
cap.release();
right.release();
wrong.release();
cv::destroyAllWindows();
std::cout << "处理完成！共处理 " << frame_count << " 帧。" << std::endl;
    }
catch (const std::exception& e) {
    std::cerr << "异常: " << e.what() << "\n";
    return -1;
    }
return 0;
}