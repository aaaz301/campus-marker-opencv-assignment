#include "calibration.hpp"
#include <iostream>
#include <vector>
// calibrate using symmetric circle grid
bool calibrateCameraFromVideo(const std::string& videoPath,int boardCols,int boardRows,double pointSpacing,const std::string& outputPath){
    if(boardCols<=0||boardRows<=0||pointSpacing<=0){
        std::cerr<<"Invalid calibration parameters."<<std::endl;
        return false;
    }
    cv::VideoCapture cap(videoPath);
    if(!cap.isOpened()){
        std::cerr<<"Cannot open calibration video: "<<videoPath<<std::endl;
        return false;
    }
    cv::Size boardSize(boardCols,boardRows);
    std::vector<cv::Point3f> boardPoints;
    for(int y=0;y<boardRows;y++){
        for(int x=0;x<boardCols;x++){
            boardPoints.emplace_back(
                static_cast<float>(x*pointSpacing),
                static_cast<float>(y*pointSpacing),
                0.0f
            );
        }
    }
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<int> usedFrames;
    cv::Mat frame;
    cv::Mat gray;
    cv::Size imageSize;
    int frameIndex=0;
    int sampleStep=30;
    int maxFrames=30;
    while(cap.read(frame)){
        if(frameIndex%sampleStep!=0){
            frameIndex++;
            continue;
        }
        imageSize=frame.size();
        cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> centers;
        bool found=cv::findCirclesGrid(
            gray,
            boardSize,
            centers,
            cv::CALIB_CB_SYMMETRIC_GRID|cv::CALIB_CB_CLUSTERING
        );
        if(found){
            imagePoints.push_back(centers);
            objectPoints.push_back(boardPoints);
            usedFrames.push_back(frameIndex);
            cv::drawChessboardCorners(frame,boardSize,centers,true);
            std::cout<<"Accepted frame: "<<frameIndex<<std::endl;
        }
        cv::imshow("Calibration",frame);
        int key=cv::waitKey(1);
        if(key==27||key=='q'){
            break;
        }
        if(static_cast<int>(imagePoints.size())>=maxFrames){
            break;
        }
        frameIndex++;
    }
    cap.release();
    cv::destroyWindow("Calibration");
    if(imagePoints.size()<10){
        std::cerr<<"Not enough calibration frames: "<<imagePoints.size()<<std::endl;
        return false;
    }
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    double rms=cv::calibrateCamera(
        objectPoints,
        imagePoints,
        imageSize,
        cameraMatrix,
        distCoeffs,
        rvecs,
        tvecs
    );
    double meanReprojectionError=0.0;
    for(size_t i=0;i<objectPoints.size();i++){
        std::vector<cv::Point2f> projected;
        cv::projectPoints(
            objectPoints[i],
            rvecs[i],
            tvecs[i],
            cameraMatrix,
            distCoeffs,
            projected
        );
        double error=cv::norm(imagePoints[i],projected,cv::NORM_L2);
        error/=std::sqrt(static_cast<double>(projected.size()));
        meanReprojectionError+=error;
    }
    meanReprojectionError/=objectPoints.size();
    std::cout<<"Calibration frames: "<<imagePoints.size()<<std::endl;
    std::cout<<"RMS error: "<<rms<<std::endl;
    std::cout<<"Mean reprojection error: "<<meanReprojectionError<<" px"<<std::endl;
    std::cout<<"Camera matrix:"<<std::endl;
    std::cout<<cameraMatrix<<std::endl;
    std::cout<<"Distortion coefficients:"<<std::endl;
    std::cout<<distCoeffs<<std::endl;
    cv::FileStorage fs(outputPath,cv::FileStorage::WRITE);
    if(!fs.isOpened()){
        std::cerr<<"Cannot save calibration file."<<std::endl;
        return false;
    }
    fs<<"camera_matrix"<<cameraMatrix;
    fs<<"distortion_coefficients"<<distCoeffs;
    fs<<"image_width"<<imageSize.width;
    fs<<"image_height"<<imageSize.height;
    fs<<"board_cols"<<boardCols;
    fs<<"board_rows"<<boardRows;
    fs<<"point_spacing"<<pointSpacing;
    fs<<"pattern"<<"symmetric_circles";
    fs<<"rms_error"<<rms;
    fs<<"mean_reprojection_error"<<meanReprojectionError;
    fs<<"used_frames"<<usedFrames;
    fs.release();
    std::cout<<"Calibration saved to: "<<outputPath<<std::endl;
    return true;
}
// load calibration
bool loadCalibration(const std::string& path,CalibrationData& data){
    cv::FileStorage fs(path,cv::FileStorage::READ);
    if(!fs.isOpened()){
        std::cerr<<"Cannot open calibration file: "<<path<<std::endl;
        return false;
    }
    fs["camera_matrix"]>>data.cameraMatrix;
    fs["distortion_coefficients"]>>data.distCoeffs;
    int width=0;
    int height=0;
    fs["image_width"]>>width;
    fs["image_height"]>>height;
    data.imageSize=cv::Size(width,height);
    fs.release();
    if(data.cameraMatrix.empty()||data.distCoeffs.empty()){
        std::cerr<<"Calibration file is incomplete."<<std::endl;
        return false;
    }
    return true;
}
// estimate pose
bool estimatePose(const Quad& corners,const cv::Mat& cameraMatrix,const cv::Mat& distCoeffs,double markerSize,cv::Mat& rvec,cv::Mat& tvec,double& reprojectionError){
    if(markerSize<=0){
        return false;
    }
    double half=markerSize/2.0;
    std::vector<cv::Point3f> objectPoints={
        cv::Point3f(static_cast<float>(-half),static_cast<float>(half),0.0f),
        cv::Point3f(static_cast<float>(half),static_cast<float>(half),0.0f),
        cv::Point3f(static_cast<float>(half),static_cast<float>(-half),0.0f),
        cv::Point3f(static_cast<float>(-half),static_cast<float>(-half),0.0f)
    };
    std::vector<cv::Point2f> imagePoints(corners.begin(),corners.end());
    bool success=cv::solvePnP(
        objectPoints,
        imagePoints,
        cameraMatrix,
        distCoeffs,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE_SQUARE
    );
    if(!success){
        success=cv::solvePnP(
            objectPoints,
            imagePoints,
            cameraMatrix,
            distCoeffs,
            rvec,
            tvec,
            false,
            cv::SOLVEPNP_ITERATIVE
        );
    }
    if(!success){
        return false;
    }
    std::vector<cv::Point2f> projected;
    cv::projectPoints(
        objectPoints,
        rvec,
        tvec,
        cameraMatrix,
        distCoeffs,
        projected
    );
    reprojectionError=0.0;
    for(int i=0;i<4;i++){
        reprojectionError+=cv::norm(imagePoints[i]-projected[i]);
    }
    reprojectionError/=4.0;
    return true;
}
// draw XYZ axes
void drawPoseAxes(cv::Mat& frame,const cv::Mat& cameraMatrix,const cv::Mat& distCoeffs,const cv::Mat& rvec,const cv::Mat& tvec,double axisLength){
    std::vector<cv::Point3f> axes={
        cv::Point3f(0,0,0),
        cv::Point3f(static_cast<float>(axisLength),0,0),
        cv::Point3f(0,static_cast<float>(axisLength),0),
        cv::Point3f(0,0,static_cast<float>(axisLength))
    };
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(
        axes,
        rvec,
        tvec,
        cameraMatrix,
        distCoeffs,
        imagePoints
    );
    if(imagePoints.size()!=4){
        return;
    }
    cv::Point origin(cvRound(imagePoints[0].x),cvRound(imagePoints[0].y));
    cv::Point xPoint(cvRound(imagePoints[1].x),cvRound(imagePoints[1].y));
    cv::Point yPoint(cvRound(imagePoints[2].x),cvRound(imagePoints[2].y));
    cv::Point zPoint(cvRound(imagePoints[3].x),cvRound(imagePoints[3].y));
    cv::line(frame,origin,xPoint,cv::Scalar(0,0,255),3);
    cv::line(frame,origin,yPoint,cv::Scalar(0,255,0),3);
    cv::line(frame,origin,zPoint,cv::Scalar(255,0,0),3);
    cv::putText(frame,"X",xPoint,cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,0,255),2);
    cv::putText(frame,"Y",yPoint,cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(0,255,0),2);
    cv::putText(frame,"Z",zPoint,cv::FONT_HERSHEY_SIMPLEX,0.7,cv::Scalar(255,0,0),2);
}