#ifndef DETECTER_H
#define DETECTER_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <array>
#include <vector>

// ======================= 参数配置 =======================
// 这些参数控制灯条分割、筛选和装甲板匹配逻辑。
struct Params {
    int diffThresh = 30;      // B - R 差值阈值，用于提取蓝色灯条
    int blueMin = 80;          // B 通道最低阈值，过滤暗噪声
    int digitGray = 75;        // 数字区域灰度门限

    double minBarArea = 15.0;     // 单个灯条最小面积
    double minBarLength = 10.0;   // 单个灯条最小长度
    double minLenWidthRatio = 2.0;
    double maxLenWidthRatio = 20.0;
    double maxBarTilt = 40.0;     // 灯条相对竖直方向的最大倾斜角
    double minFillRatio = 0.4;    // 轮廓填充度下限

    double minLenSimilarity = 0.4;   // 两灯条长度相似度下限
    double maxAngleDiff = 15.0;       // 两灯条方向差异允许上限
    double minDistRatio = 0.5;        // 中心距离与平均长度比下限
    double maxDistRatio = 3.2;        // 中心距离与平均长度比上限
    double maxCosCenterLine = 0.5;    // 中心线与灯条轴夹角余弦上限
    double minDigitRatio = 0.10;      // 中间区域亮像素占比下限
};

// ======================= 数据结构 =======================
// 单个灯条：通常为装甲板的发光条带，保存其旋转矩形和几何特征。
struct LightBar {
    cv::RotatedRect rect;
    cv::Point2f center, top, bottom;  // top.y <= bottom.y
    float length = 0.f, width = 0.f;
    float angle = 0.f;                // 相对竖直方向的角度，范围 [-90, 90]
};

// 一对灯条对应一个装甲板候选，按 LT、RT、RB、LB 顺序存储四个角点。
struct Armor {
    LightBar left, right;
    std::array<cv::Point2f, 4> corners;
    cv::Point2f center;
};

// ======================= 核心算法接口 =======================
// 把图像转换为蓝灯条二值图。
cv::Mat makeBinary(const cv::Mat& frame, const Params& p);

// 根据旋转矩形提取灯条几何参数。
LightBar makeLightBar(const cv::RotatedRect& r);

// 从二值图中提取所有候选灯条，并做几何筛选。
std::vector<LightBar> findLightBars(const cv::Mat& binary, const Params& p);

// 计算两灯条之间四边形区域内白色数字的亮度占比。
double digitRatio(const cv::Mat& gray, const std::array<cv::Point2f, 4>& q, int grayThr);

// 用灯条之间的长度、角度、距离和数字区域特征匹配装甲板。
std::vector<Armor> matchArmors(const std::vector<LightBar>& bars,
                              const cv::Mat& gray,
                              const Params& p);

// 将识别结果绘制到图像中。
void drawResult(cv::Mat& img, const std::vector<LightBar>& bars,
               const std::vector<Armor>& armors);

#endif

