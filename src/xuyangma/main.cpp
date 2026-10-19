#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include "detector.hpp"
#include "calibration.hpp"
// print commands
void printUsage(){
    std::cout<<"Usage:"<<std::endl;
    std::cout<<"Detection:"<<std::endl;
    std::cout<<"./marker_detector [input_video] [output_video]"<<std::endl;
    std::cout<<"Calibration:"<<std::endl;
    std::cout<<"./marker_detector --calibrate <video> <cols> <rows> <point_spacing_m> <output_yaml>"<<std::endl;
    std::cout<<"Pose:"<<std::endl;
    std::cout<<"./marker_detector --pose <video> <calibration_yaml> <marker_size_m> [output_video]"<<std::endl;
}
// open video or camera
bool openInput(const std::string& inputPath,cv::VideoCapture& cap){
    if(inputPath=="0"||inputPath=="camera"){
        cap.open(0);
    }else{
        cap.open(inputPath);
    }
    if(!cap.isOpened()){
        std::cerr<<"Cannot open input: "<<inputPath<<std::endl;
        return false;
    }
    return true;
}
// run detector
int runDetection(const std::string& inputPath,const std::string& outputPath,bool usePose,const CalibrationData& calibration,double markerSize){
    cv::VideoCapture cap;
    if(!openInput(inputPath,cap)){
        return 1;
    }
    double fps=cap.get(cv::CAP_PROP_FPS);
    if(fps<=1.0||fps>240.0){
        fps=30.0;
    }
    int width=static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int height=static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    std::cout<<"Input: "<<width<<" x "<<height<<", FPS = "<<fps<<std::endl;
    if(usePose){
        if(calibration.imageSize.width>0&&calibration.imageSize.height>0){
            if(width!=calibration.imageSize.width||height!=calibration.imageSize.height){
                std::cerr<<"Video size does not match calibration size."<<std::endl;
                std::cerr<<"Calibration: "<<calibration.imageSize.width<<" x "<<calibration.imageSize.height<<std::endl;
                std::cerr<<"Current video: "<<width<<" x "<<height<<std::endl;
                return 1;
            }
        }
        std::cout<<"Pose unit: meter"<<std::endl;
    }
    cv::VideoWriter writer;
    if(!outputPath.empty()){
        writer.open(
            outputPath,
            cv::VideoWriter::fourcc('M','J','P','G'),
            fps,
            cv::Size(width,height)
        );
        if(!writer.isOpened()){
            std::cerr<<"Cannot open output video."<<std::endl;
            return 1;
        }
    }
    Quad previousCorners;
    bool hasPrevious=false;
    int frameIndex=0;
    cv::Mat frame;
    while(cap.read(frame)){
        Quad corners;
        bool detected=detectMarker(frame,corners);
        if(detected&&hasPrevious){
            if(!isTemporallyConsistent(corners,previousCorners,frame.size())){
                detected=false;
            }
        }
        if(detected){
            if(hasPrevious){
                smoothCorners(corners,previousCorners,frame.size());
            }
            drawDetection(frame,corners);
            std::cout<<"Frame "<<frameIndex<<": DETECTED ";
            const char* labels[4]={"LT","RT","RB","LB"};
            for(int i=0;i<4;i++){
                std::cout<<labels[i]<<"=("<<cvRound(corners[i].x)<<","<<cvRound(corners[i].y)<<") ";
            }
            std::cout<<std::endl;
            if(usePose){
                cv::Mat rvec;
                cv::Mat tvec;
                double reprojectionError=0.0;
                bool poseSuccess=estimatePose(
                    corners,
                    calibration.cameraMatrix,
                    calibration.distCoeffs,
                    markerSize,
                    rvec,
                    tvec,
                    reprojectionError
                );
                if(poseSuccess){
                    drawPoseAxes(
                        frame,
                        calibration.cameraMatrix,
                        calibration.distCoeffs,
                        rvec,
                        tvec,
                        markerSize*0.5
                    );
                    double x=tvec.at<double>(0,0);
                    double y=tvec.at<double>(1,0);
                    double z=tvec.at<double>(2,0);
                    double distance=cv::norm(tvec);
                    std::ostringstream poseText;
                    poseText<<std::fixed<<std::setprecision(3);
                    poseText<<"X="<<x<<"m Y="<<y<<"m Z="<<z<<"m";
                    cv::putText(frame,poseText.str(),cv::Point(30,90),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(255,255,255),2);
                    std::ostringstream distanceText;
                    distanceText<<std::fixed<<std::setprecision(3);
                    distanceText<<"Distance="<<distance<<"m";
                    cv::putText(frame,distanceText.str(),cv::Point(30,120),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(255,255,255),2);
                    std::ostringstream errorText;
                    errorText<<std::fixed<<std::setprecision(2);
                    errorText<<"Reprojection error="<<reprojectionError<<" px";
                    cv::putText(frame,errorText.str(),cv::Point(30,150),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(255,255,255),2);
                    std::cout<<std::fixed<<std::setprecision(3);
                    std::cout<<"Pose X="<<x<<"m Y="<<y<<"m Z="<<z<<"m Distance="<<distance<<"m ";
                    std::cout<<std::setprecision(2)<<"Error="<<reprojectionError<<" px"<<std::endl;
                }else{
                    cv::putText(frame,"POSE FAILED",cv::Point(30,90),cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,0,255),2);
                }
            }
            previousCorners=corners;
            hasPrevious=true;
        }else{
            cv::putText(frame,"NOT DETECTED",cv::Point(30,50),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0,0,255),2);
            std::cout<<"Frame "<<frameIndex<<": NOT DETECTED"<<std::endl;
            hasPrevious=false;
        }
        if(writer.isOpened()){
            writer.write(frame);
        }
        cv::imshow("Campus Marker Detection",frame);
        int delay=std::max(1,static_cast<int>(1000.0/fps));
        int key=cv::waitKey(delay);
        if(key==27||key=='q'){
            break;
        }
        frameIndex++;
    }
    cap.release();
    if(writer.isOpened()){
        writer.release();
    }
    cv::destroyAllWindows();
    return 0;
}
int main(int argc,char** argv){
    if(argc>=2&&std::string(argv[1])=="--calibrate"){
        if(argc!=7){
            printUsage();
            return 1;
        }
        std::string videoPath=argv[2];
        int boardCols=std::stoi(argv[3]);
        int boardRows=std::stoi(argv[4]);
        double pointSpacing=std::stod(argv[5]);
        std::string outputPath=argv[6];
        std::cout<<"Calibration pattern: symmetric circles grid"<<std::endl;
        std::cout<<"Grid size: "<<boardCols<<" x "<<boardRows<<std::endl;
        std::cout<<"Point spacing: "<<pointSpacing<<" m"<<std::endl;
        bool success=calibrateCameraFromVideo(
            videoPath,
            boardCols,
            boardRows,
            pointSpacing,
            outputPath
        );
        return success?0:1;
    }
    if(argc>=2&&std::string(argv[1])=="--pose"){
        if(argc!=5&&argc!=6){
            printUsage();
            return 1;
        }
        std::string videoPath=argv[2];
        std::string calibrationPath=argv[3];
        double markerSize=std::stod(argv[4]);
        std::string outputPath="";
        if(argc==6){
            outputPath=argv[5];
        }
        CalibrationData calibration;
        if(!loadCalibration(calibrationPath,calibration)){
            return 1;
        }
        return runDetection(
            videoPath,
            outputPath,
            true,
            calibration,
            markerSize
        );
    }
    std::string inputPath="data/raw/marker_video.avi";
    std::string outputPath="";
    if(argc>=2){
        inputPath=argv[1];
    }
    if(argc>=3){
        outputPath=argv[2];
    }
    if(argc>3){
        printUsage();
        return 1;
    }
    CalibrationData emptyCalibration;
    return runDetection(
        inputPath,
        outputPath,
        false,
        emptyCalibration,
        0.0
    );
}