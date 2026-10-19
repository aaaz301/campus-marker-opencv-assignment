#include <iostream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    if (argc != 3) {
        cout << "用法: " << argv[0] << " <输入视频.mp4> <输出视频.mp4>" << endl;
        return -1;
    }

    string input_path = argv[1];
    string output_path = argv[2];

    VideoCapture cap(input_path);
    if (!cap.isOpened()) {
        cout << "错误：无法打开视频文件！" << endl;
        return -1;
    }

    int width = cap.get(CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(CAP_PROP_FPS);
    if (fps <= 0) fps = 30.0;

    VideoWriter writer(output_path, VideoWriter::fourcc('m', 'p', '4', 'v'), fps, Size(width, height));

    Mat frame;
    int lost_frame_count = 0;
    vector<Point2f> last_corners;

    cout << "开始处理视频..." << endl;

    while (cap.read(frame)) {
        if (frame.empty()) break;

        // ---------- 第一步：找白色发光区域 ----------
        Mat gray, mask;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        
        // 【注意】这里是二值化，亮度阈值设为 150
        // 如果你觉得识别不到，可以把 150 改成 130 试试；如果误识别，改成 170
        threshold(gray, mask, 150, 255, THRESH_BINARY);

        // 形态学连接操作：把分散的 L 形连成一个整体
        Mat kernel = getStructuringElement(MORPH_RECT, Size(20, 20));
        morphologyEx(mask, mask, MORPH_CLOSE, kernel);

        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        bool found_target = false;
        vector<Point2f> current_corners;

        // 把所有轮廓按面积从大到小排序
        sort(contours.begin(), contours.end(), [](const vector<Point>& a, const vector<Point>& b) {
            return contourArea(a) > contourArea(b);
        });

        // 只取面积最大的那一个轮廓
        if (!contours.empty()) {
            // 面积过滤：大于 2000 像素即可
            if (contourArea(contours[0]) > 2000) {
                RotatedRect rect = minAreaRect(contours[0]);
                float w = rect.size.width;
                float h = rect.size.height;
                float long_side = max(w, h);
                float short_side = min(w, h);
                
                if (short_side > 1.0f) {
                    float aspect_ratio = long_side / short_side;
                    // 长宽比限制：放宽到 0.5 ~ 2.0（允许方形）
                    if (aspect_ratio >= 0.5f && aspect_ratio <= 2.0f) {
                        Point2f rect_points[4];
                        rect.points(rect_points);
                        found_target = true;
                        for (int j = 0; j < 4; j++) {
                            current_corners.push_back(rect_points[j]);
                        }
                    }
                }
            }
        }

        // ---------- 第二步：时序稳定性 ----------
        if (found_target) {
            lost_frame_count = 0;
            last_corners = current_corners;
        } else {
            lost_frame_count++;
        }

        if (lost_frame_count > 10) {
            last_corners.clear();
        }

        // ---------- 第三步：绘制结果 ----------
        if (!last_corners.empty()) {
            Point2f center(0, 0);
            for (int j = 0; j < 4; j++) {
                center.x += last_corners[j].x;
                center.y += last_corners[j].y;
            }
            center.x /= 4.0;
            center.y /= 4.0;

            vector<pair<float, Point2f>> angles;
            for (int j = 0; j < 4; j++) {
                float angle = atan2(last_corners[j].y - center.y, last_corners[j].x - center.x);
                angles.push_back(make_pair(angle, last_corners[j]));
            }
            sort(angles.begin(), angles.end(),
                 [](const pair<float, Point2f>& a, const pair<float, Point2f>& b) {
                     return a.first < b.first;
                 });

            int lt_index = 0;
            double min_dist = 1e9;
            for (int j = 0; j < 4; j++) {
                double d = angles[j].second.x * angles[j].second.x + angles[j].second.y * angles[j].second.y;
                if (d < min_dist) {
                    min_dist = d;
                    lt_index = j;
                }
            }

            vector<Point2f> sorted_points(4);
            sorted_points[0] = angles[lt_index].second;
            sorted_points[1] = angles[(lt_index + 1) % 4].second;
            sorted_points[2] = angles[(lt_index + 2) % 4].second;
            sorted_points[3] = angles[(lt_index + 3) % 4].second;

            for (int j = 0; j < 4; j++) {
                line(frame, sorted_points[j], sorted_points[(j + 1) % 4], Scalar(0, 255, 0), 2);
            }

            string labels[4] = {"LT", "RT", "RB", "LB"};
            for (int j = 0; j < 4; j++) {
                circle(frame, sorted_points[j], 5, Scalar(0, 0, 255), -1);
                putText(frame, labels[j], sorted_points[j], FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 255), 2);
            }
        } else {
            putText(frame, "Target Lost", Point(20, 50), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2);
        }

        // 【关键】显示中间的黑白图，方便你看电脑到底看到了什么
        imshow("Mask", mask); 
        imshow("Detection", frame);
        writer.write(frame);

        if (waitKey(30) == 27) break;
    }

    cap.release();
    writer.release();
    destroyAllWindows();
    cout << "处理完成！输出视频保存在：" << output_path << endl;

    return 0;
}