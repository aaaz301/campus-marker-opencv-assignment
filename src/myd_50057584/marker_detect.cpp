// Campus marker recognition – traditional OpenCV approach.
//
// The marker is a white square (~190×190 px) with a dark geometric pattern.
// Pipeline: HSV threshold → open → contour cluster → validate → Canny verify
// → convexHull+approxPolyDP corners (fallback: minAreaRect) → order →
// cornerSubPix → square constraint → temporal smoothing → occlusion tolerance.
//
// Usage: ./marker_detect [--input <video>] [--output <video>] [--debug <dir>]
//   --input  输入视频路径（默认 data/raw/marker_video.avi）
//   --output 输出标注视频路径（可选）
//   --debug  调试掩膜视频输出目录（可选）

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/geometry/2d.hpp>

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <climits>
#include <map>
#include <functional>
#include <filesystem>
#include <optional>
#include <numeric>

namespace fs = std::filesystem;

namespace {

struct Params {
    int    s_max = 60,  v_min = 140;        // HSV white threshold
    int    open_size = 3;                    // morphological open kernel
    double min_area = 200.0;                 // min contour area
    double min_height = 120, max_height = 280;
    double min_width = 15,   max_width = 400;
    double min_aspect_full = 0.4, min_aspect_edge = 0.08, max_aspect = 3.0;
    int    cluster_dist = 80;                // Union-Find merge distance (px)
    double edge_density_min = 0.005, edge_density_max = 0.65;  // Canny verify
    int    close_size = 65;                  // fallback morphological close
    double approx_eps = 0.02;                // approxPolyDP epsilon ratio
    cv::Size subpix_win = {5, 5};
    cv::TermCriteria subpix_tc = {cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01};
    double smooth_alpha_min = 0.3;           // smoothing when stationary
    double motion_threshold = 5.0;           // px/frame: above → no smoothing
    int    lost_frames_limit = 5;
};

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

void drawResult(cv::Mat& frame, const std::vector<cv::Point2f>& corners, bool detected) {
    if (!detected || corners.size() != 4) {
        cv::putText(frame, "NOT DETECTED", {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 0, 255}, 2);
        return;
    }
    std::vector<cv::Point> poly;
    for (auto& p : corners) poly.emplace_back(cv::Point(p));
    cv::polylines(frame, poly, true, {0, 255, 0}, 2);
    const char* labels[] = {"LT", "RT", "RB", "LB"};
    const cv::Scalar colors[] = {{0,0,255}, {0,165,255}, {255,0,0}, {255,255,0}};
    for (int i = 0; i < 4; ++i) {
        cv::circle(frame, corners[i], 6, colors[i], -1);
        cv::putText(frame, labels[i], {cvRound(corners[i].x + 8), cvRound(corners[i].y - 8)},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, colors[i], 2);
    }
    cv::putText(frame, "DETECTED", {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
}

// ---------------------------------------------------------------------------
// Step 1-2: HSV white threshold + morphological open
// ---------------------------------------------------------------------------

cv::Mat thresholdWhite(const cv::Mat& bgr, const Params& p) {
    cv::Mat hsv, mask;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, p.v_min), cv::Scalar(180, p.s_max, 255), mask);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {p.open_size, p.open_size});
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    return mask;
}

// ---------------------------------------------------------------------------
// Step 3-4: Contour extraction + Union-Find clustering
// ---------------------------------------------------------------------------

struct ClusterResult {
    cv::Rect bbox;
    std::vector<int> member_indices;
};

std::optional<ClusterResult> findBestCluster(
        const std::vector<std::vector<cv::Point>>& contours,
        double min_area, int cluster_dist) {
    std::vector<cv::Rect> boxes;
    std::vector<int> idx_map;
    for (int i = 0; i < (int)contours.size(); ++i) {
        if (cv::contourArea(contours[i]) < min_area) continue;
        boxes.push_back(cv::boundingRect(contours[i]));
        idx_map.push_back(i);
    }
    if (boxes.empty()) return std::nullopt;

    int n = (int)boxes.size();
    std::vector<int> parent(n);
    std::iota(parent.begin(), parent.end(), 0);
    std::function<int(int)> find = [&](int x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j) {
            cv::Rect exp(boxes[i].x - cluster_dist, boxes[i].y - cluster_dist,
                         boxes[i].width + 2*cluster_dist, boxes[i].height + 2*cluster_dist);
            if ((exp & boxes[j]).area() > 0) {
                int a = find(i), b = find(j);
                if (a != b) parent[a] = b;
            }
        }

    std::map<int, std::vector<int>> groups;
    for (int i = 0; i < n; ++i) groups[find(i)].push_back(i);

    double bestArea = 0;
    std::optional<ClusterResult> best;
    for (auto& [root, members] : groups) {
        int x0 = INT_MAX, y0 = INT_MAX, x1 = INT_MIN, y1 = INT_MIN;
        for (int idx : members) {
            x0 = std::min(x0, boxes[idx].x);
            y0 = std::min(y0, boxes[idx].y);
            x1 = std::max(x1, boxes[idx].x + boxes[idx].width);
            y1 = std::max(y1, boxes[idx].y + boxes[idx].height);
        }
        cv::Rect merged(x0, y0, x1 - x0, y1 - y0);
        double area = (double)merged.width * merged.height;
        if (area > bestArea) {
            bestArea = area;
            std::vector<int> ci;
            for (int idx : members) ci.push_back(idx_map[idx]);
            best = ClusterResult{merged, ci};
        }
    }
    return best;
}

// ---------------------------------------------------------------------------
// Step 5: Size / aspect-ratio validation
// ---------------------------------------------------------------------------

bool validateCluster(const cv::Rect& c, const Params& p) {
    int w = c.width, h = c.height;
    if (h < p.min_height || h > p.max_height) return false;
    if (w < p.min_width  || w > p.max_width)  return false;
    double aspect = (h > 0) ? (double)w / h : 0;
    if (w < 150) return aspect >= p.min_aspect_edge;
    return aspect >= p.min_aspect_full && aspect <= p.max_aspect;
}

// ---------------------------------------------------------------------------
// Step 6: Content verification – Canny edge density in ROI
// ---------------------------------------------------------------------------

bool verifyContent(const cv::Mat& bgr, const cv::Rect& roi, const Params& p) {
    cv::Mat gray, edges;
    cv::cvtColor(bgr(roi), gray, cv::COLOR_BGR2GRAY);
    cv::Canny(gray, edges, 50, 150);
    double density = (double)cv::countNonZero(edges) / (double)roi.area();
    return density >= p.edge_density_min && density <= p.edge_density_max;
}

// ---------------------------------------------------------------------------
// Step 7a: Corner extraction via convexHull + approxPolyDP (primary)
// ---------------------------------------------------------------------------

std::optional<std::vector<cv::Point2f>> extractCornersHull(
        const std::vector<std::vector<cv::Point>>& contours,
        const std::vector<int>& members, double eps_ratio) {
    std::vector<cv::Point> pts;
    for (int idx : members)
        pts.insert(pts.end(), contours[idx].begin(), contours[idx].end());
    if (pts.size() < 10) return std::nullopt;

    std::vector<cv::Point> hull;
    cv::convexHull(pts, hull);
    double perim = cv::arcLength(hull, true);
    for (double f : {eps_ratio, 0.01, 0.015, 0.03, 0.04, 0.05}) {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(hull, approx, f * perim, true);
        if ((int)approx.size() == 4)
            return std::vector<cv::Point2f>(approx.begin(), approx.end());
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// Step 7b: Fallback – morphological close + minAreaRect
// ---------------------------------------------------------------------------

std::optional<std::vector<cv::Point2f>> extractCornersRect(
        const cv::Mat& mask, const cv::Rect& cluster,
        const std::vector<std::vector<cv::Point>>& contours,
        const Params& p, int W, int H) {
    int margin = 10;
    int rx = std::max(0, cluster.x - margin);
    int ry = std::max(0, cluster.y - margin);
    int rw = std::min(W - rx, cluster.width + 2 * margin);
    int rh = std::min(H - ry, cluster.height + 2 * margin);
    cv::Rect roi(rx, ry, rw, rh);

    cv::Mat closed;
    cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, {p.close_size, p.close_size});
    cv::morphologyEx(mask(roi), closed, cv::MORPH_CLOSE, k);
    cv::Mat k2 = cv::getStructuringElement(cv::MORPH_RECT, {5, 5});
    cv::morphologyEx(closed, closed, cv::MORPH_OPEN, k2);

    std::vector<std::vector<cv::Point>> cc;
    cv::findContours(closed, cc, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (cc.empty()) return std::nullopt;

    int best = 0;
    double bestA = 0;
    for (int i = 0; i < (int)cc.size(); ++i) {
        double a = cv::contourArea(cc[i]);
        if (a > bestA) { bestA = a; best = i; }
    }
    // Collect original contour points inside the solid region
    std::vector<cv::Point> solid;
    for (auto& pt : cc[best]) solid.emplace_back(pt.x + roi.x, pt.y + roi.y);
    std::vector<cv::Point> allPts;
    for (auto& c : contours) {
        if (cv::contourArea(c) < p.min_area) continue;
        for (auto& pt : c)
            if (cv::pointPolygonTest(solid, pt, false) >= 0)
                allPts.push_back(pt);
    }
    auto& rp = (allPts.size() >= 20) ? allPts : solid;

    cv::Point2f v[4];
    cv::minAreaRect(rp).points(v);
    return std::vector<cv::Point2f>(v, v + 4);
}

// ---------------------------------------------------------------------------
// Step 8: Order corners as LT, RT, RB, LB (robust polar-angle method)
// ---------------------------------------------------------------------------

void orderCorners(std::vector<cv::Point2f>& pts) {
    if (pts.size() != 4) return;
    cv::Point2f ctr(0, 0);
    for (auto& p : pts) { ctr.x += p.x; ctr.y += p.y; }
    ctr.x /= 4; ctr.y /= 4;
    // Sort by polar angle → cyclic order
    std::sort(pts.begin(), pts.end(), [&](const cv::Point2f& a, const cv::Point2f& b) {
        return std::atan2(a.y - ctr.y, a.x - ctr.x) < std::atan2(b.y - ctr.y, b.x - ctr.x);
    });
    // Find top edge (smallest midpoint Y)
    int top = 0;
    double minMidY = 1e9;
    for (int i = 0; i < 4; ++i) {
        double midY = (pts[i].y + pts[(i + 1) % 4].y) / 2.0;
        if (midY < minMidY) { minMidY = midY; top = i; }
    }
    std::vector<cv::Point2f> ord(4);
    for (int i = 0; i < 4; ++i) ord[i] = pts[(top + i) % 4];
    // Ensure LT.x < RT.x; swap pairs to preserve left/right correspondence
    if (ord[0].x > ord[1].x) { std::swap(ord[0], ord[1]); std::swap(ord[2], ord[3]); }
    if (ord[3].x > ord[2].x) std::swap(ord[2], ord[3]);
    pts = ord;
}

// ---------------------------------------------------------------------------
// Step 9-10: Sub-pixel refinement + square constraint
// ---------------------------------------------------------------------------

void refineCorners(const cv::Mat& bgr, std::vector<cv::Point2f>& corners,
                   const Params& p, int W, int H) {
    if (corners.size() != 4) return;
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    bool inside = true;
    int w2 = p.subpix_win.width, h2 = p.subpix_win.height;
    for (auto& c : corners)
        if (c.x < w2 || c.x >= W - w2 || c.y < h2 || c.y >= H - h2) { inside = false; break; }
    if (inside)
        cv::cornerSubPix(gray, corners, p.subpix_win, {-1, -1}, p.subpix_tc);
    // Square constraint for near-axis-aligned markers
    double topAngle = std::atan2(corners[1].y - corners[0].y, corners[1].x - corners[0].x);
    if (std::abs(topAngle) < 10.0 * CV_PI / 180.0) {
        double aTY = (corners[0].y + corners[1].y) / 2, aBY = (corners[2].y + corners[3].y) / 2;
        double aLX = (corners[0].x + corners[3].x) / 2, aRX = (corners[1].x + corners[2].x) / 2;
        corners = {{(float)aLX,(float)aTY}, {(float)aRX,(float)aTY},
                   {(float)aRX,(float)aBY}, {(float)aLX,(float)aBY}};
    }
}

// ---------------------------------------------------------------------------
// Top-level detection
// ---------------------------------------------------------------------------

bool detectMarker(const cv::Mat& bgr, const Params& p, int W, int H,
                  std::vector<cv::Point2f>& corners, cv::Mat* dbgMask = nullptr) {
    cv::Mat mask = thresholdWhite(bgr, p);
    if (dbgMask) *dbgMask = mask.clone();

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    auto cluster = findBestCluster(contours, p.min_area, p.cluster_dist);
    if (!cluster || !validateCluster(cluster->bbox, p)) return false;
    if (!verifyContent(bgr, cluster->bbox, p)) return false;

    // Primary: convexHull + approxPolyDP
    auto hc = extractCornersHull(contours, cluster->member_indices, p.approx_eps);
    // Fallback: morphological close + minAreaRect
    if (!hc) hc = extractCornersRect(mask, cluster->bbox, contours, p, W, H);
    if (!hc) return false;

    corners = *hc;
    orderCorners(corners);
    refineCorners(bgr, corners, p, W, H);
    return true;
}

} // namespace

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    std::string input_path = "data/raw/marker_video.avi", output_path, debug_dir;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--input" && i + 1 < argc) input_path = argv[++i];
        else if (a == "--output" && i + 1 < argc) output_path = argv[++i];
        else if (a == "--debug" && i + 1 < argc) debug_dir = argv[++i];
        else {
            std::cerr << "Usage: " << argv[0]
                      << " [--input <video>] [--output <video>] [--debug <dir>]\n"
                      << "  --input  输入视频路径（默认 data/raw/marker_video.avi）\n"
                      << "  --output 输出标注视频路径（可选）\n"
                      << "  --debug  调试掩膜视频输出目录（可选）\n";
            return 1;
        }
    }

    cv::VideoCapture cap(input_path);
    if (!cap.isOpened()) { std::cerr << "Error: cannot open " << input_path << "\n"; return 1; }
    double fps = cap.get(cv::CAP_PROP_FPS);
    int W = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int H = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    int total = (int)cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::cout << "Input: " << input_path << "  " << W << "x" << H << "  fps=" << fps << "  frames=" << total << "\n";

    cv::VideoWriter writer, dbgWriter;
    if (!output_path.empty()) {
        writer.open(output_path, cv::CAP_FFMPEG, cv::VideoWriter::fourcc('M','J','P','G'),
                    (fps > 0 ? fps : 25.0), {W, H});
        if (!writer.isOpened()) std::cerr << "Warning: cannot open output video.\n";
    }
    if (!debug_dir.empty()) fs::create_directories(debug_dir);

    Params p;
    cv::Mat frame;
    int frame_idx = 0, detect_count = 0, lost_count = 0;
    std::vector<cv::Point2f> prev(4);
    bool have_prev = false;
    std::vector<std::vector<cv::Point2f>> history;
    const int HIST = 3;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        ++frame_idx;

        std::vector<cv::Point2f> corners;
        cv::Mat dbgMask;
        bool detected = detectMarker(frame, p, W, H, corners,
                                     debug_dir.empty() ? nullptr : &dbgMask);

        if (detected) {
            ++detect_count;
            lost_count = 0;
            // 3-frame median filter on Y (removes single-frame spikes)
            history.push_back(corners);
            if ((int)history.size() > HIST) history.erase(history.begin());
            if ((int)history.size() >= 2) {
                for (int i = 0; i < 4; ++i) {
                    std::vector<double> ys;
                    for (auto& h : history) ys.push_back(h[i].y);
                    std::sort(ys.begin(), ys.end());
                    int n = (int)ys.size();
                    corners[i].y = (float)((n % 2 == 0) ? (ys[n/2-1]+ys[n/2])/2.0 : ys[n/2]);
                }
            }
            // Adaptive exponential smoothing
            if (have_prev) {
                double maxDisp = 0;
                for (int i = 0; i < 4; ++i)
                    maxDisp = std::max(maxDisp, cv::norm(corners[i] - prev[i]));
                double alpha = (maxDisp >= p.motion_threshold) ? 1.0
                    : p.smooth_alpha_min + (1.0 - p.smooth_alpha_min) * (maxDisp / p.motion_threshold);
                for (int i = 0; i < 4; ++i)
                    corners[i] = prev[i] * (1.0 - alpha) + corners[i] * alpha;
            }
            prev = corners;
            have_prev = true;
        } else {
            if (++lost_count <= p.lost_frames_limit && have_prev) {
                corners = prev;
                detected = true;
            } else {
                detected = false;
                have_prev = false;
                history.clear();
            }
        }

        cv::Mat out = frame.clone();
        drawResult(out, corners, detected);
        cv::putText(out, "frame " + std::to_string(frame_idx) + "/" + std::to_string(total)
                    + (detected ? "  DETECTED" : "  NOT DETECTED"),
                    {20, H - 20}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 1);
        if (writer.isOpened()) writer.write(out);

        if (!debug_dir.empty() && !dbgMask.empty()) {
            if (!dbgWriter.isOpened())
                dbgWriter.open((fs::path(debug_dir) / "debug_mask.avi").string(),
                               cv::VideoWriter::fourcc('M','J','P','G'),
                               (fps > 0 ? fps : 25.0), dbgMask.size(), false);
            if (dbgWriter.isOpened()) dbgWriter.write(dbgMask);
        }

        if (detected && corners.size() == 4)
            printf("frame %4d  DETECTED  LT(%.1f,%.1f) RT(%.1f,%.1f) RB(%.1f,%.1f) LB(%.1f,%.1f)\n",
                   frame_idx, corners[0].x, corners[0].y, corners[1].x, corners[1].y,
                   corners[2].x, corners[2].y, corners[3].x, corners[3].y);
        else
            printf("frame %4d  NOT DETECTED\n", frame_idx);
    }
    cap.release();
    if (writer.isOpened()) writer.release();
    if (dbgWriter.isOpened()) dbgWriter.release();
    std::cout << "Done. Processed " << frame_idx << " frames. Detected: " << detect_count << "/" << frame_idx << "\n";
    return 0;
}
