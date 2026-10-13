#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

int debug=1;

namespace fs = std::filesystem;

namespace {

double calc_distance(std::vector<cv::Point> &contour1,std::vector<cv::Point> &contour2)
{
    double Min=1e9;
    for(auto i:contour1){
        for(auto j:contour2){
            Min=std::min(Min,1.0*(i.x-j.x)*(i.x-j.x)+(i.y-j.y)*(i.y-j.y));
        }
    }
    return std::sqrt(Min);
}

struct DSU;
std::vector<DSU> dsu;

struct DSU
{
    std::vector<cv::Point> contour;
    double sum=0;
    double squaresum=0;
    int fa,size=1;
    DSU(std::vector<cv::Point> Contour,int Fa):contour(Contour),fa(Fa){
    }
    bool operator<(const DSU &a)const{
        return this->fa<a.fa;
    }
    static double calcCV(int i){
        if(dsu[i].contour.empty()) return -1;
        double bar=dsu[i].sum/dsu[i].contour.size();
        return std::sqrt(dsu[i].squaresum/dsu[i].contour.size()-bar*bar)/bar;
    }
    static void merge(int i,int j){
        i=find(i),j=find(j);
        if(i==j) return;
        if(dsu[i].size<dsu[j].size) std::swap(i,j);
        dsu[j].fa=i;
        dsu[i].size+=dsu[j].size;
        dsu[i].sum+=dsu[i].squaresum;
        dsu[i].squaresum+=dsu[j].squaresum;
        dsu[i].contour.insert(dsu[i].contour.end(),dsu[j].contour.begin(),dsu[j].contour.end());
    }
    static int find(int x){
        return x==dsu[x].fa?x:dsu[x].fa=find(dsu[x].fa);
    }
};

struct DIS
{
    int i,j;
    double dis;
    bool operator<(const DIS &a)const{
        return this->dis<a.dis;
    }
};

cv::Mat processFrame(const cv::Mat& frame) {
    //第零步：BGR 原图。OpenCV 读取的视频帧默认按蓝、绿、红排列；
    //保留一份原图，方便与后续每个处理阶段直接对照。
    cv::Mat image_progress_bgr = frame.clone();

    //第一步：滤波去噪。中值滤波用邻域的中间值替换孤立噪点，
    cv::Mat image_progress_denoise;
    cv::medianBlur(image_progress_bgr, image_progress_denoise, 3);

    //第二步：二值化。由于标志物和背景是黑白，故转换为灰度图。
    cv::Mat image_progress_binary,image_progress_gray;
    cv::cvtColor(image_progress_denoise,image_progress_gray,cv::COLOR_BGR2GRAY);
    cv::threshold(image_progress_gray,image_progress_binary,200,255,cv::THRESH_BINARY);

    //第三步：形态学处理。用开运算，删除小亮点。
    cv::Mat image_progress_open;
    const cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(image_progress_binary, image_progress_open, cv::MORPH_OPEN, open_kernel);

    //第四步：轮廓提取。每个白色连通区域给出一条外轮廓，得到点集序列contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image_progress_open, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //对每个点集进行多边形近似，以减少计算量。
    for(auto &i:contours){
        std::vector<cv::Point> approx;
        double epsilon=0.02*cv::arcLength(i,true);
        cv::approxPolyDP(i,approx,epsilon,true);
        i=approx;
    }

    //第五步：找到构成标志物的若干点集。
    //为实现识别多个标志物，对两两之间点集距离排序，距离从小到大排序。
    std::vector<DIS> distances;
    for(int i=0;i<contours.size();i++){
        for(int j=i+1;j<contours.size();j++){
            distances.push_back({i,j,calc_distance(contours[i],contours[j])});
        }
    }
    std::sort(distances.begin(),distances.end());
    //依次合并点集，控制合并后的变异系数(CV)不超过阈值，以保证标志物识别准确性。
    //合并用并查集实现。
    for(auto i:contours) dsu.push_back(DSU(i,dsu.size()));
    for(auto i:distances){
        int fai=DSU::find(i.i),faj=DSU::find(i.j);
        if(fai==faj) continue;
        double dis=calc_distance(dsu[fai].contour,dsu[faj].contour);
        double Sum=dsu[fai].sum+dsu[faj].sum+dis;
        double Squaresum=dsu[fai].squaresum+dsu[faj].squaresum+dis*dis;
        int n=dsu[fai].size+dsu[faj].size-1;
        double Bar=Sum/n;
        double CV=sqrt(Squaresum/n-Bar*Bar)/Bar;
        if(CV<=2.1){//CV阈值
            DSU::merge(fai,faj);
        }
    }

    //第六步，计算合并后点集的凸包，这个凸包就是标志物标识。描边后返回。
    //找到合并后的若干点集
    std::vector<int> contour_index;
    for(int i=0;i<dsu.size();i++) contour_index.push_back(DSU::find(i));
    sort(contour_index.begin(),contour_index.end());
    contour_index.erase(std::unique(contour_index.begin(),contour_index.end()),contour_index.end());
    //找凸包，进行多边形近似
    std::vector<std::vector<cv::Point>> hulls;
    for(auto i:contour_index){
        hulls.emplace_back();
        cv::convexHull(dsu[i].contour,hulls.back());
        std::vector<cv::Point> approx;
        double epsilon=0.02*cv::arcLength(hulls.back(),true);
        cv::approxPolyDP(hulls.back(),approx,epsilon,true);
        hulls.pop_back();
        if(cv::contourArea(approx)<=20) continue;
        if(approx.size()==4){
            hulls.push_back(approx);
        }
        else{//如果不是四边形，就拟合出一个矩形
            cv::RotatedRect rect = cv::minAreaRect(approx);
            cv::Point2f vertice[4];
            rect.points(vertice);
            hulls.emplace_back();
            for(int j=0;j<4;j++){
                hulls.back().push_back(cv::Point(vertice[j].x,vertice[j].y));
            }
        }
    }
    //描边标点
    cv::Mat result = frame.clone();
    // return result;
    cv::drawContours(result, hulls, -1, cv::Scalar(0, 255, 0), 3);
    for(auto i:hulls){
        for(int j=0;j<4;j++){
            cv::circle(result,i[j],5,cv::Scalar(0,0,255),-1);
        }
    }
    //标记是否检测到标志物
    if(hulls.empty()){
        cv::putText(result,"Not Detected",cv::Point(20,40),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0,0,255),2,cv::LINE_AA);
    }
    else{
        cv::putText(result,"Marker Detected",cv::Point(20,40),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0,255,0),2,cv::LINE_AA);
    }
    dsu.clear();
    return result;
}

void openVideoWriter(cv::VideoWriter& writer, const fs::path& path,
                     double fps, const cv::Size& size) {
    // 直接用 OpenCV 写入 mp4v 编码的 MP4，帧率和尺寸由调用方传入。
    writer.open(path.string(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                fps, size, true);
    if (!writer.isOpened()) {
        throw std::runtime_error("无法创建视频: " + path.string());
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "用法: " << argv[0] << " <输入视频.mp4> <输出目录>\n";
        return 1;
    }

    try {
        const fs::path input_path(argv[1]);
        const fs::path output_directory(argv[2]);
        if (!fs::is_regular_file(input_path)) {
            throw std::runtime_error("输入视频不存在: " + input_path.string());
        }

        cv::VideoCapture capture(input_path.string());
        if (!capture.isOpened()) {
            throw std::runtime_error("无法读取输入视频: " + input_path.string());
        }
        fs::create_directories(output_directory);
        const std::string stem = input_path.stem().string();
        const fs::path video_path = output_directory / (stem + "_annotated.mp4");

        double fps = capture.get(cv::CAP_PROP_FPS);
        if (!std::isfinite(fps) || fps <= 0.0) {
            fps = 30.0;
        }
        cv::VideoWriter annotated_writer;
        cv::Mat frame;
        int frame_index = 0;
        while (capture.read(frame)) {
            if (frame.empty()) {
                throw std::runtime_error("视频中出现空帧");
            }

            cv::Mat result = processFrame(frame);
            if (!annotated_writer.isOpened()) {
                openVideoWriter(annotated_writer, video_path, fps, frame.size());
            }
            annotated_writer.write(result);
            ++frame_index;
        }
        if (frame_index == 0) {
            throw std::runtime_error("视频没有可读取的帧");
        }
        annotated_writer.release();
        capture.release();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "错误: " << error.what() << '\n';
        return 1;
    }
}
/*
cmake -S src/chenzinan_50059063 -B build/chenzinan_50059063 -DCMAKE_BUILD_TYPE=Release

cmake --build build/chenzinan_50059063 --parallel

./build/chenzinan_50059063/PJ ./example.mp4 ./
*/