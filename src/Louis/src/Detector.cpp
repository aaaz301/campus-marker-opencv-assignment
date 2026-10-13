#include "Detector.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

// ======================= 第 1 步：二值化图像 =======================
// 这里的灯条并不是过曝区域，而是偏蓝且亮度较低的目标，
// 因此使用 B 通道减去 R 通道来突出蓝色灯条。
cv::Mat makeBinary(const cv::Mat& frame, const Params& p) {
    std::vector<cv::Mat> ch;
    cv::split(frame, ch);

    cv::Mat diff, diffMask, blueMask, binary;
    cv::subtract(ch[0], ch[2], diff);
    cv::threshold(diff, diffMask, p.diffThresh, 255, cv::THRESH_BINARY);
    cv::threshold(ch[0], blueMask, p.blueMin, 255, cv::THRESH_BINARY);
    cv::bitwise_and(diffMask, blueMask, binary);

    // 使用垂直结构元素补偿灯条之间的小间隙，同时不明显拉宽目标。
    static const cv::Mat kClose = cv::getStructuringElement(cv::MORPH_RECT, {3, 5});
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kClose);
    return binary;
}

// ======================= 第 2 步：提取单个灯条 =======================
// 从旋转矩形中恢复灯条的长轴、短轴和顶点位置，用于后续筛选和匹配。
LightBar makeLightBar(const cv::RotatedRect& r) {
    cv::Point2f p[4];
    r.points(p);

    double e01 = cv::norm(p[0] - p[1]);
    double e12 = cv::norm(p[1] - p[2]);

    LightBar bar;
    cv::Point2f a, b;
    if (e01 >= e12) {
        bar.length = static_cast<float>(e01);
        bar.width = static_cast<float>(e12);
        a = (p[1] + p[2]) * 0.5f;
        b = (p[3] + p[0]) * 0.5f;
    } else {
        bar.length = static_cast<float>(e12);
        bar.width = static_cast<float>(e01);
        a = (p[0] + p[1]) * 0.5f;
        b = (p[2] + p[3]) * 0.5f;
    }

    bar.top = (a.y < b.y) ? a : b;
    bar.bottom = (a.y < b.y) ? b : a;
    bar.center = r.center;
    bar.rect = r;

    cv::Point2f axis = bar.bottom - bar.top;
    bar.angle = static_cast<float>(std::atan2(axis.x, axis.y) * 180.0 / CV_PI);
    return bar;
}

// 从二值图中提取所有可能的灯条，并按照长宽比、面积和角度做过滤。
std::vector<LightBar> findLightBars(const cv::Mat& binary, const Params& p) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBar> bars;
    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < p.minBarArea) continue;

        cv::RotatedRect r = cv::minAreaRect(c);
        if (r.size.area() < 1.f) continue;

        LightBar bar = makeLightBar(r);
        if (bar.width < 1.f || bar.length < p.minBarLength) continue;

        double ratio = bar.length / bar.width;
        if (ratio < p.minLenWidthRatio || ratio > p.maxLenWidthRatio) continue;
        if (area / r.size.area() < p.minFillRatio) continue;
        if (std::abs(bar.angle) > p.maxBarTilt) continue;

        bars.push_back(bar);
    }

    // 按横坐标排序，方便后续判断中间是否存在另一条灯条。
    std::sort(bars.begin(), bars.end(), [](const LightBar& a, const LightBar& b) {
        return a.center.x < b.center.x;
    });
    return bars;
}

// ======================= 第 3 步：灯条配对 -> 装甲板 =======================
// 通过计算四边形内部白色数字的分布密度来判断该区域是否真的是装甲板。
double digitRatio(const cv::Mat& gray, const std::array<cv::Point2f, 4>& q, int grayThr) {
    cv::Point2f c = (q[0] + q[1] + q[2] + q[3]) * 0.25f;
    std::vector<cv::Point> poly;
    for (const auto& pt : q) {
        cv::Point2f s = c + (pt - c) * 0.7f;
        poly.emplace_back(cvRound(s.x), cvRound(s.y));
    }

    cv::Rect box = cv::boundingRect(poly) & cv::Rect(0, 0, gray.cols, gray.rows);
    if (box.area() <= 0) return 0.0;

    std::vector<cv::Point> shifted;
    for (const auto& pt : poly) shifted.push_back(pt - box.tl());

    cv::Mat mask = cv::Mat::zeros(box.size(), CV_8UC1);
    cv::fillConvexPoly(mask, shifted, 255);
    int total = cv::countNonZero(mask);
    if (total == 0) return 0.0;

    cv::Mat bright;
    cv::threshold(gray(box), bright, grayThr, 255, cv::THRESH_BINARY);
    cv::bitwise_and(bright, mask, bright);
    return static_cast<double>(cv::countNonZero(bright)) / total;
}

// 按照长度相似、平行度、距离和数字区域特征筛选装甲板候选。
std::vector<Armor> matchArmors(const std::vector<LightBar>& bars,
                              const cv::Mat& gray,
                              const Params& p) {
    struct Cand { int i, j; double score; };
    std::vector<Cand> cands;
    const int n = static_cast<int>(bars.size());

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const LightBar& a = bars[i];
            const LightBar& b = bars[j];

            double lenSim = std::min(a.length, b.length) / std::max(a.length, b.length);
            if (lenSim < p.minLenSimilarity) continue;

            double angleDiff = std::abs(a.angle - b.angle);
            if (angleDiff > p.maxAngleDiff) continue;

            cv::Point2f v = b.center - a.center;
            double dist = cv::norm(v);
            if (dist < 1e-3) continue;
            double avgLen = 0.5 * (a.length + b.length);
            double distRatio = dist / avgLen;
            if (distRatio < p.minDistRatio || distRatio > p.maxDistRatio) continue;

            cv::Point2f axisA = a.bottom - a.top;
            cv::Point2f axisB = b.bottom - b.top;
            cv::Point2f axis = axisA * static_cast<float>(1.0 / cv::norm(axisA)) +
                               axisB * static_cast<float>(1.0 / cv::norm(axisB));
            double cosv = std::abs(axis.dot(v)) / (cv::norm(axis) * dist);
            if (cosv > p.maxCosCenterLine) continue;

            // 若中间还有相近高度的灯条，说明这两条形状更像穿越当前目标的干扰项。
            bool crossed = false;
            double midY = 0.5 * (a.center.y + b.center.y);
            for (int k = i + 1; k < j; ++k) {
                const LightBar& o = bars[k];
                if (std::abs(o.center.y - midY) < 0.5 * avgLen &&
                    o.length >= 0.5 * std::min(a.length, b.length)) {
                    crossed = true;
                    break;
                }
            }
            if (crossed) continue;

            const LightBar& L = (a.center.x < b.center.x) ? a : b;
            const LightBar& R = (a.center.x < b.center.x) ? b : a;
            std::array<cv::Point2f, 4> quad = {L.top, R.top, R.bottom, L.bottom};
            double digit = digitRatio(gray, quad, p.digitGray);
            if (digit < p.minDigitRatio) continue;

            double score = angleDiff / p.maxAngleDiff + (1.0 - lenSim) + cosv - digit;
            cands.push_back({i, j, score});
        }
    }

    // 以得分排序，优先保留更可能是有效装甲板的组合，并保证每条灯条最多使用一次。
    std::sort(cands.begin(), cands.end(),
              [](const Cand& x, const Cand& y) { return x.score < y.score; });
    std::vector<bool> used(bars.size(), false);

    std::vector<Armor> armors;
    for (const auto& c : cands) {
        if (used[c.i] || used[c.j]) continue;
        used[c.i] = used[c.j] = true;

        Armor ar;
        if (bars[c.i].center.x < bars[c.j].center.x) {
            ar.left = bars[c.i];
            ar.right = bars[c.j];
        } else {
            ar.left = bars[c.j];
            ar.right = bars[c.i];
        }

        ar.corners = {ar.left.top, ar.right.top, ar.right.bottom, ar.left.bottom};
        ar.center = (ar.corners[0] + ar.corners[1] + ar.corners[2] + ar.corners[3]) * 0.25f;
        armors.push_back(ar);
    }
    return armors;
}

// ======================= 第 4 步：绘制结果 =======================
// 将识别结果直接叠加在原图上，显示灯条和装甲板轮廓。
void drawResult(cv::Mat& img, const std::vector<LightBar>& bars,
                const std::vector<Armor>& armors) {
    for (const auto& b : bars) {
        cv::Point2f pts[4];
        b.rect.points(pts);
        for (int i = 0; i < 4; ++i)
            cv::line(img, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 0), 1);
    }

    for (const auto& a : armors) {
        for (int i = 0; i < 4; ++i) {
            cv::line(img, a.corners[i], a.corners[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
        }
        cv::line(img, a.corners[0], a.corners[2], cv::Scalar(0, 255, 255), 1);
        cv::line(img, a.corners[1], a.corners[3], cv::Scalar(0, 255, 255), 1);
        cv::line(img, a.left.center, a.right.center, cv::Scalar(255, 255, 0), 1);
        cv::circle(img, a.center, 4, cv::Scalar(255, 0, 255), -1);
        cv::putText(img, "armor", a.corners[0] + cv::Point2f(0, -6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    }
}
