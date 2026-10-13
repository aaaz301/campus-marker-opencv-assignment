#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

int main(int argc, char* argv[]) {
    // 1. 检查输入并打开视频
    if (argc != 2) {
        std::cerr << "请提供一个视频路径\n";
        return 1;
    }

    cv::VideoCapture cap(argv[1]);

    if (!cap.isOpened()) {
        std::cerr << "视频打开失败\n";
        return 1;
    }

    cv::Mat frame;
    double fps = cap.get(cv::CAP_PROP_FPS);

    if (!std::isfinite(fps) || fps <= 0) {
        fps = 30.0;
    }
    cv::VideoWriter writer;
    int frameCount = 0;

    while (cap.read(frame)) {
        // 第一次读到画面时，创建结果视频
        ++frameCount;
        if (!writer.isOpened()) {
            writer.open(
                "build/yuhang/result.avi",
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                fps,
                frame.size()
            );

            if (!writer.isOpened()) {
                std::cerr << "无法创建结果视频\n";
                return 1;
            }
        }
        // 2. 去噪、灰度转换、亮度分割
        cv::Mat denoised;
        cv::medianBlur(frame, denoised, 3);

        cv::Mat gray;
        cv::cvtColor(denoised, gray, cv::COLOR_BGR2GRAY);

        cv::Mat binary;
        cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);

        // 3. 提取轮廓，保存大 L 形候选
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(
            binary, contours,
            cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE
        );

        std::vector<cv::Rect> candidates;
        std::vector<std::vector<cv::Point>> candidateContours;

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            cv::Rect box = cv::boundingRect(contour);

            double fillRatio = area / box.area();
            double aspectRatio =
                static_cast<double>(box.width) / box.height;

            if (fillRatio > 0.35 && fillRatio < 0.60 &&
                aspectRatio > 0.7 && aspectRatio < 1.3) {
                candidates.push_back(box);
                candidateContours.push_back(contour);
            }
        }

        // 4. 计算候选中心
        std::vector<cv::Point2f> centers;

        for (const auto& box : candidates) {
            centers.emplace_back(
                box.x + box.width / 2.0F,
                box.y + box.height / 2.0F
            );
        }

        cv::Mat result = frame.clone();
        bool detected = false;

        // 5. 检查三个候选的布局
        if (centers.size() == 3) {
            int topIndex = 0;

            for (int i = 1; i < 3; ++i) {
                if (centers[i].y < centers[topIndex].y) {
                    topIndex = i;
                }
            }

            std::vector<int> bottomIndices;

            for (int i = 0; i < 3; ++i) {
                if (i != topIndex) {
                    bottomIndices.push_back(i);
                }
            }

            int leftIndex = bottomIndices[0];
            int rightIndex = bottomIndices[1];

            if (centers[leftIndex].x > centers[rightIndex].x) {
                int temp = leftIndex;
                leftIndex = rightIndex;
                rightIndex = temp;
            }

            cv::Point2f top = centers[topIndex];
            cv::Point2f left = centers[leftIndex];
            cv::Point2f right = centers[rightIndex];

            float horizontalGap = right.x - left.x;
            float heightDifference = std::abs(right.y - left.y);
            float leftOffset = std::abs(top.x - left.x);

            bool belowTop = left.y > top.y && right.y > top.y;

            bool bottomAligned =
                horizontalGap > 0 &&
                heightDifference < 0.2F * horizontalGap;

            bool leftAligned =
                leftOffset < 0.2F * horizontalGap;

            double topSize = candidates[topIndex].area();
            double leftSize = candidates[leftIndex].area();
            double rightSize = candidates[rightIndex].area();

            double largestSize =
                std::max({topSize, leftSize, rightSize});
            double smallestSize =
                std::min({topSize, leftSize, rightSize});

            bool similarSize =
                smallestSize > 0 &&
                largestSize / smallestSize < 1.5;

            // 6. 收集右上搜索范围内的轮廓点
            cv::Point2f expectedRightTop = top + (right - left);
            float searchRadius = 0.4F * horizontalGap;
            std::vector<cv::Point> rightTopPoints;

            for (const auto& contour : contours) {
                cv::Rect region = cv::boundingRect(contour);

                cv::Point2f regionCenter(
                    region.x + region.width / 2.0F,
                    region.y + region.height / 2.0F
                );

                float dx = regionCenter.x - expectedRightTop.x;
                float dy = regionCenter.y - expectedRightTop.y;
                float distance = std::sqrt(dx * dx + dy * dy);

                if (distance < searchRadius) {
                    for (const auto& point : contour) {
                        rightTopPoints.push_back(point);
                    }
                }
            }

            // 非空已经表示右上附近存在轮廓点，
            // 因此无需再单独保存 hasRightTop
            if (belowTop && bottomAligned && leftAligned &&
                similarSize && !rightTopPoints.empty()) {
                detected = true;

                // 7. 提取四个外角
                cv::Point leftTop = candidateContours[topIndex][0];

                for (const auto& point : candidateContours[topIndex]) {
                    if (point.x + point.y < leftTop.x + leftTop.y) {
                        leftTop = point;
                    }
                }

                cv::Point leftBottom =
                    candidateContours[leftIndex][0];

                for (const auto& point : candidateContours[leftIndex]) {
                    if (point.y - point.x >
                        leftBottom.y - leftBottom.x) {
                        leftBottom = point;
                    }
                }

                cv::Point rightBottom =
                    candidateContours[rightIndex][0];

                for (const auto& point : candidateContours[rightIndex]) {
                    if (point.x + point.y >
                        rightBottom.x + rightBottom.y) {
                        rightBottom = point;
                    }
                }

                cv::Point rightTop = rightTopPoints[0];

                for (const auto& point : rightTopPoints) {
                    if (point.x - point.y > rightTop.x - rightTop.y) {
                        rightTop = point;
                    }
                }

                // 8. 按固定顺序绘制四边形和角点
                std::vector<cv::Point> corners = {
                    leftTop, rightTop, rightBottom, leftBottom
                };

                for (int i = 0; i < 4; ++i) {
                    cv::line(
                        result,
                        corners[i],
                        corners[(i + 1) % 4],
                        cv::Scalar(0, 255, 0),
                        2
                    );
                }

                const char* labels[] = {"LT", "RT", "RB", "LB"};

                for (int i = 0; i < 4; ++i) {
                    cv::circle(
                        result, corners[i], 5,
                        cv::Scalar(0, 0, 255), cv::FILLED
                    );

                    cv::putText(
                        result,
                        labels[i],
                        corners[i] + cv::Point(8, -8),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(0, 0, 255),
                        1
                    );
                }
            }
        }

        // 9. 显示检测状态与最终画面
        if (detected) {
            cv::putText(
                result, "Detected", cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 255, 0), 2
            );
        } else {
            cv::putText(
                result, "Not detected", cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 0, 255), 2
            );
        }
        writer.write(result);
        cv::imshow("Result", result);

        if (cv::waitKey(15) == 'q') {
            break;
        }
    }
    writer.release();
    cv::destroyAllWindows();
    if (frameCount == 0) {
        std::cerr << "视频没有可读取的帧\n";
        return 1;
    }

    std::cout << "已处理帧数：" << frameCount << "\n";
    return 0;
}