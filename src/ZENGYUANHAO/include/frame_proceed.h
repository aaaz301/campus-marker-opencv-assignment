#include<opencv2/opencv.hpp>
#include<vector>
struct img{
    cv::Mat frame;
    bool is_grey;
    int armor_number;
};
struct LightBar{
    cv::RotatedRect rect;
    cv::Point2f top;
    cv::Point2f bottom;
    float length;
    float angle;
};
struct BarPair{
    LightBar former;
    LightBar latter;
    int numleft;
    int numright;
    float score;
    cv::Point2f apex[4];
};
img frame_proceed(const cv::Mat& frame,int x);
std::vector<BarPair> proceed_pair(const std::vector<LightBar>& bars,
                                  const cv::Mat& frame,float max_distance);