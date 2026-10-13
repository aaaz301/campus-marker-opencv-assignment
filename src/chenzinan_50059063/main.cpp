#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

int debug=1;

namespace fs = std::filesystem;

namespace Detector{

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
    // std::cout<<contour_index.size()<<std::endl;
    //找凸包，进行多边形近似，描边标点
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
    writer.open(path.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                fps, size, true);
    // std::cout<<"aaa";
    if (!writer.isOpened()) {
        throw std::runtime_error("无法创建视频: " + path.string());
    }
    // std::cout<<" a";
}

void write_video(cv::VideoCapture &capture,const fs::path video_path,double &fps)
{
    cv::VideoWriter annotated_writer;
    cv::Mat frame;
    int frame_index = 0;
    while (capture.read(frame)) {
        if (frame.empty()) {
            throw std::runtime_error("视频中出现空帧");
        }

        cv::Mat result = Detector::processFrame(frame);
        if (!annotated_writer.isOpened()) {
            Detector::openVideoWriter(annotated_writer, video_path, fps, frame.size());
        }
        annotated_writer.write(result);
        ++frame_index;
    }
    if (frame_index == 0) {
        throw std::runtime_error("视频没有可读取的帧");
    }
    annotated_writer.release();
    capture.release();
    std::cout<<"标志物识别完成"<<std::endl;
}

}  // namespace

namespace Camera_calibration_and_Pose_estimation
{

const int gap=80;//每80帧取一帧用于位姿检测。
const cv::Size grid_size(7,7);//7*7圆点格
const double circle_spacing(40.0);//从视频推测相邻圆心距离是40mm，如果不是也不要紧。
cv::Mat camera_matrix,dist_coeffs;//保存内参矩阵和畸变系数
std::vector<cv::Mat> rvecs,tvecs;
std::vector<std::vector<cv::Point3f>> all_object_points;//关键帧世界坐标
std::vector<std::vector<cv::Point2f>> all_image_points;//关键帧图像坐标

std::vector<cv::Point3f> object_points;//构建世界坐标

void camera_calibration(cv::VideoCapture &capture,const fs::path &yaml_path,double &fps)
{
    const int width=(int)capture.get(cv::CAP_PROP_FRAME_WIDTH);
    const int height=(int)capture.get(cv::CAP_PROP_FRAME_HEIGHT);
    const cv::Size image_size(width,height);

    //建立标定板坐标
    for(int i=0;i<grid_size.height;i++){
        for(int j=0;j<grid_size.width;j++){
            object_points.emplace_back(j*circle_spacing,i*circle_spacing,0.0);
        }
    }
    
    cv::SimpleBlobDetector::Params blobParams;//判定为圆点的参数
    blobParams.minArea=10.0f;
    blobParams.maxArea=5000.0f;
    blobParams.minCircularity=0.6f;
    blobParams.minConvexity=0.8f;
    blobParams.minInertiaRatio=0.4f;
    cv::Ptr<cv::SimpleBlobDetector> blobDetector=cv::SimpleBlobDetector::create(blobParams);

    int frame_index=0;
    cv::Mat frame,gray;
    std::vector<cv::Point2f> centers;
    while(capture.read(frame)){
        if (frame.empty()) {
            throw std::runtime_error("视频中出现空帧");
        }
        if(frame_index%gap==0){
            cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);//灰度化
            bool isfound=cv::findCirclesGrid(gray,grid_size,centers,cv::CALIB_CB_SYMMETRIC_GRID|cv::CALIB_CB_CLUSTERING,blobDetector);//找标定板上的圆点
            if(isfound){
                //亚像素精化
                cv::cornerSubPix(gray,centers,cv::Size(5,5),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::EPS|cv::TermCriteria::COUNT,30,0.01));
                //作为一组数据放入vector
                all_object_points.push_back(object_points);
                all_image_points.push_back(centers);
            }
        }
        ++frame_index;
    }
    // std::cout<<frame_index<<std::endl;
    if (frame_index == 0) {
        throw std::runtime_error("视频没有可读取的帧");
    }
    capture.release();

    std::cout<<"成功检测帧数: "<<all_image_points.size()<<std::endl;
    if(all_image_points.empty()){
        std::cout<<"没有检测到任何标定板！"<<std::endl;
        return;
    }
    //计算出内参矩阵和畸变系数
    double rms=cv::calibrateCamera(all_object_points,all_image_points,image_size,camera_matrix,dist_coeffs,rvecs,tvecs);//相机标定

    std::cout << "\n========== 标定完成 ==========\n";
    std::cout << "RMS 重投影误差: " << rms << " px\n";
    std::cout << "有效帧数: " << all_image_points.size() << "\n\n";

    std::cout << "内参矩阵:\n" << camera_matrix << "\n\n";
    std::cout << "畸变系数:\n" << dist_coeffs << "\n\n";

    // fx, fy, cx, cy
    std::cout << "fx = " << camera_matrix.at<double>(0, 0)
              << ", fy = " << camera_matrix.at<double>(1, 1)
              << ", cx = " << camera_matrix.at<double>(0, 2)
              << ", cy = " << camera_matrix.at<double>(1, 2) << "\n";

    // // 保存到文件
    // cv::FileStorage fs_out(yaml_path.string(), cv::FileStorage::WRITE);
    // fs_out << "image_width"    << width;
    // fs_out << "image_height"   << height;
    // fs_out << "camera_matrix"  << camera_matrix;
    // fs_out << "dist_coeffs"    << dist_coeffs;
    // fs_out << "rms_error"      << rms;
    // fs_out.release();
    // std::cout << "\n标定结果已保存: " << yaml_path << std::endl;
}

cv::Vec3f rotate(const cv::Mat &rvec)//计算旋转
{
    cv::Mat R;
    cv::Rodrigues(rvec,R);
    double sy=std::sqrt(R.at<double>(0,0)*R.at<double>(0,0)+R.at<double>(1,0)*R.at<double>(1,0));
    bool singular=(sy<1e-6);
    double rx,ry,rz;
    if (!singular){
        rx=std::atan2(R.at<double>(2,1),R.at<double>(2,2));
        ry=std::atan2(-R.at<double>(2,0),sy);
        rz=std::atan2(R.at<double>(1,0),R.at<double>(0,0));
    }
    else{
        rx=std::atan2(-R.at<double>(1,2),R.at<double>(1,1));
        ry=std::atan2(-R.at<double>(2,0),sy);
        rz=0;
    }
    return cv::Vec3f(rx*180.0/CV_PI,ry*180.0/CV_PI,rz*180.0/CV_PI);
}

void pose_estimation(cv::VideoCapture &capture,const fs::path &video_path,const double &fps)
{
    cv::Mat map1,map2;
    //去畸变
    cv::initUndistortRectifyMap(camera_matrix,dist_coeffs,cv::Mat(),camera_matrix,cv::Size((int)capture.get(cv::CAP_PROP_FRAME_WIDTH),(int)capture.get(cv::CAP_PROP_FRAME_HEIGHT)),CV_16SC2,map1,map2);
    
    cv::Mat frame,undistorted,rvec,tvec;
    cv::VideoWriter video;
    std::vector<cv::Point2f> centers;
    int frame_index=0;
    bool isfound,pnp;
    cv::Vec3f euler;
    while(capture.read(frame)){
        if (frame.empty()) {
            throw std::runtime_error("视频中出现空帧");
        }
        if(frame_index%70==0){//减少计算量
            cv::remap(frame,undistorted,map1,map2,cv::INTER_LINEAR);//得到修正后图像undistorted
            cv::Mat gray;
            cv::cvtColor(undistorted,gray,cv::COLOR_BGR2GRAY);//灰度化
            // cv::Mat enhanced;
            // gray.convertTo(enhanced,-1,1.5,20);//增强
            //检测圆点
            isfound=cv::findCirclesGrid(gray,grid_size,centers,cv::CALIB_CB_SYMMETRIC_GRID|cv::CALIB_CB_CLUSTERING);
            if(isfound){
                //亚像素精化
                cv::cornerSubPix(gray,centers,cv::Size(5,5),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::EPS|cv::TermCriteria::COUNT,30,0.01));
                //PnP
                pnp=cv::solvePnP(object_points,centers,camera_matrix,dist_coeffs,rvec,tvec,false,cv::SOLVEPNP_ITERATIVE);
                if(pnp){
                    euler=rotate(rvec);
                    std::cout << "帧 " << frame_index << " — "
                            << "平移(mm): "
                            << "X=" << std::setw(8) << tvec.at<double>(0)
                            << " Y=" << std::setw(8) << tvec.at<double>(1)
                            << " Z=" << std::setw(8) << tvec.at<double>(2)
                            << "  |  旋转(deg): "
                            << "RX=" << std::setw(7) << euler[0]
                            << " RY=" << std::setw(7) << euler[1]
                            << " RZ=" << std::setw(7) << euler[2]
                            << std::endl;
                    //画检测到的圆心
                    cv::drawChessboardCorners(undistorted,grid_size,centers,pnp);
                    //画坐标轴
                    cv::drawFrameAxes(undistorted,camera_matrix,dist_coeffs,rvec,tvec,3*circle_spacing,3);
                    //左上角标记偏移和旋转
                    cv::putText(undistorted,
                        "X="+std::to_string((int)tvec.at<double>(0))+"mm "+
                        "Y="+std::to_string((int)tvec.at<double>(1))+"mm "+
                        "Z="+std::to_string((int)tvec.at<double>(2))+"mm; "+
                        "RX="+std::to_string((int)euler[0])+"deg "+
                        "RY="+std::to_string((int)euler[1])+"deg "+
                        "RZ="+std::to_string((int)euler[2])+"deg ",
                        cv::Point(30,50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0,255,0),2);
                }
                else{
                    // std::cout<<"帧 "<<frame_index<<" 未检测到圆点格"<<std::endl;
                    cv::putText(undistorted,"Not detected",cv::Point(30,50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0,0,255),2);
                }
            }
            else{
                // std::cout<<"帧 "<<frame_index<<" 未检测到圆点格"<<std::endl;
                cv::putText(undistorted,"Not detected",cv::Point(30,50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0,0,255),2);
            }
        }
        if (!video.isOpened()) {
            Detector::openVideoWriter(video, video_path, fps, frame.size());
        }
        video.write(undistorted);
        ++frame_index;
    }
    // std::cout<<frame_index<<std::endl;
    if (frame_index == 0) {
        throw std::runtime_error("视频没有可读取的帧");
    }
    capture.release();
}

}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "用法: " << argv[0] << " <标志物识别输入视频.avi> <相机标定输入视频.avi> <输出目录>\n";
        return 1;
    }

    try {
        const fs::path input_path1(argv[1]);
        const fs::path input_path2(argv[2]);
        const fs::path output_directory(argv[3]);
        if (!fs::is_regular_file(input_path1)) {
            throw std::runtime_error("标志物识别输入视频不存在: " + input_path1.string());
        }
        if (!fs::is_regular_file(input_path2)) {
            throw std::runtime_error("相机标定输入视频不存在: " + input_path2.string());
        }

        cv::VideoCapture capture1(input_path1.string());
        cv::VideoCapture capture2(input_path2.string());
        if (!capture1.isOpened()) {
            throw std::runtime_error("无法读取标志物识别输入视频: " + input_path1.string());
        }
        if (!capture2.isOpened()) {
            throw std::runtime_error("无法读取相机标定输入视频: " + input_path2.string());
        }

        fs::create_directories(output_directory);
        const std::string stem1 = input_path1.stem().string();
        const std::string stem2 = input_path2.stem().string();
        const fs::path video_path = output_directory / (stem1 + "_annotated.avi");
        const fs::path yaml_path = output_directory / (stem2 + "_camera_calibration.yaml");
        const fs::path video_path2 = output_directory / (stem2 + "_demo.avi");

        double fps1 = capture1.get(cv::CAP_PROP_FPS),fps2=capture2.get(cv::CAP_PROP_FPS);
        Detector::write_video(capture1,video_path,fps1);
        Camera_calibration_and_Pose_estimation::camera_calibration(capture2,yaml_path,fps2);
        capture2.open(input_path2.string());
        Camera_calibration_and_Pose_estimation::pose_estimation(capture2,video_path2,fps2);
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

./build/chenzinan_50059063/PJ ./example.mp4 ./data/raw/calibration_video.avi ./
*/