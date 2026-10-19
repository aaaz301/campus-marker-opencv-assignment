#pragma once
#include <opencv2/opencv.hpp>
#include <array>
using Quad=std::array<cv::Point2f,4>;
// four marker corners
bool detectMarker(const cv::Mat& frame,Quad& bestCorners);
// check motion between frames
bool isTemporallyConsistent(const Quad& current,const Quad& previous,const cv::Size& frameSize);
// smooth corner positions
void smoothCorners(Quad& current,const Quad& previous,const cv::Size& frameSize);
// draw result
void drawDetection(cv::Mat& frame,const Quad& corners);