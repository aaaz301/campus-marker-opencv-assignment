// Camera calibration + calibration-board pose estimation.
//
// Stage 1 — Calibration:
//   Reads data/raw/calibration_video.avi, detects a 7×7 symmetric
//   circle grid (spacing 0.03 m) in sampled frames, and runs
//   cv::calibrateCamera to obtain intrinsics + distortion coefficients.
//   Results saved to calib_results.yaml.
//
// Stage 2 — Pose estimation:
//   In every frame of the same calibration video, detects the circle
//   grid and uses cv::solvePnP with the calibrated intrinsics to
//   estimate the board's 6-DOF pose.  The board has an asymmetric
//   feature (a black triangle at one corner of the border) that is
//   used to consistently identify the 4 corners (LT, RT, RB, LB)
//   across all frames, ensuring a stable coordinate origin.
//
// Board coordinate system:
//   Origin at the triangle corner (physical top-left).
//   X axis: along the row (toward RT), Y axis: along the column (toward LB),
//   Z axis: toward the camera (right-handed).
//   Units: metres.  Circle spacing: 0.03 m.
//
// Camera coordinate system (OpenCV convention):
//   X right, Y down, Z forward (into the scene).
//   tvec gives the board origin's position in camera coordinates.
//
// Usage:
//   ./calibrate                                    # calibrate + pose, default paths
//   ./calibrate --input <video>                    # specify input video
//   ./calibrate --calib <yaml>                     # run calibration, output to yaml
//   ./calibrate --pose <video>                     # run pose estimation, output to video
//   If both --calib and --pose are given, calibrate first then estimate pose.
//   If neither is given, do both with default output paths.

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect.hpp>     // findCirclesGrid, CALIB_CB_*
#include <opencv2/calib.hpp>         // calibrateCamera
#include <opencv2/geometry/3d.hpp>   // solvePnP, projectPoints, Rodrigues

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <filesystem>

namespace fs = std::filesystem;
namespace {

// ── Parameters ────────────────────────────────────────────

struct CalibParams {
    cv::Size patternSize = {7, 7};     // 7×7 symmetric circle grid
    double squareSize    = 0.03;       // 0.03 m spacing
    int frameStride      = 30;         // sample every N-th frame for calibration
    int minFrames        = 15;
    int maxFrames        = 80;
};

// ── Generate 3D object points for the circle grid ────────

std::vector<cv::Point3f> circleGridObjectPoints(const CalibParams& cp) {
    std::vector<cv::Point3f> pts;
    for (int r = 0; r < cp.patternSize.height; ++r)
        for (int c = 0; c < cp.patternSize.width; ++c)
            pts.emplace_back((float)(c * cp.squareSize),
                             (float)(r * cp.squareSize), 0.0f);
    return pts;
}

// ── Stage 1: Calibration ─────────────────────────────────

bool runCalibration(const std::string& videoPath, const CalibParams& cp,
                    cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
                    const std::string& outputPath) {
    std::cout << "=== 标定 ===\n"
              << "视频: " << videoPath << "\n"
              << "标定板: " << cp.patternSize.width << "x" << cp.patternSize.height
              << " 对称圆点阵列, 间距=" << cp.squareSize << " m\n"
              << "采样间隔: 每 " << cp.frameStride << " 帧\n";

    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        std::cerr << "Cannot open " << videoPath << "\n";
        return false;
    }
    int W = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int H = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    int total = (int)cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::cout << "分辨率: " << W << "x" << H << "  帧数=" << total << "\n";

    auto objTemplate = circleGridObjectPoints(cp);
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;

    cv::Mat frame, gray;
    int frameIdx = 0, goodCount = 0;
    while (cap.read(frame) && goodCount < cp.maxFrames) {
        ++frameIdx;
        if (frameIdx % cp.frameStride != 0) continue;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> centers;
        if (!cv::findCirclesGrid(gray, cp.patternSize, centers,
                                 cv::CALIB_CB_SYMMETRIC_GRID |
                                 cv::CALIB_CB_CLUSTERING))
            continue;
        objectPoints.push_back(objTemplate);
        imagePoints.push_back(centers);
        if (++goodCount % 10 == 0)
            std::cout << "  已采集 " << goodCount << " 帧...\n";
    }
    cap.release();
    std::cout << "共采集 " << goodCount << " 帧\n";
    if (goodCount < cp.minFrames) {
        std::cerr << "错误: 仅 " << goodCount << " 帧 (需要 >= "
                  << cp.minFrames << ")\n";
        return false;
    }

    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(objectPoints, imagePoints, {W, H},
                                     cameraMatrix, distCoeffs, rvecs, tvecs);

    double totalErr = 0;
    for (size_t i = 0; i < objectPoints.size(); ++i) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i],
                          cameraMatrix, distCoeffs, projected);
        totalErr += cv::norm(imagePoints[i], projected, cv::NORM_L2) / imagePoints[i].size();
    }
    std::cout << "\n标定完成.\n"
              << "  RMS 重投影误差: " << rms << " px\n"
              << "  平均每帧误差: " << totalErr / objectPoints.size() << " px\n"
              << "  相机内参矩阵:\n" << cameraMatrix << "\n"
              << "  畸变系数: " << distCoeffs.t() << "\n";

    cv::FileStorage fs(outputPath, cv::FileStorage::WRITE);
    if (!fs.isOpened()) { std::cerr << "Cannot write " << outputPath << "\n"; return false; }
    fs << "camera_matrix" << cameraMatrix
       << "distortion_coefficients" << distCoeffs
       << "image_width" << W
       << "image_height" << H
       << "rms_reprojection_error" << rms
       << "num_frames_used" << goodCount
       << "pattern_type" << "symmetric_circles"
       << "pattern_size" << cp.patternSize
       << "square_size" << cp.squareSize;
    fs.release();
    std::cout << "  已保存到 " << outputPath << "\n";
    return true;
}

// ── Triangle corner detection (improved) ─────────────────
// The board has a black triangle at one corner of the border.
// We detect it by measuring the dark-pixel ratio in the outward
// region of each corner, using adaptive local cell sizes and
// multi-scale sampling for robustness under perspective distortion.
//
// Returns the index (0-3) of the triangle corner among the 4
// grid corners [0, cols-1, cols*rows-1, cols*(rows-1)], or -1
// if detection is unreliable.

// Compute local cell size at each corner from its two adjacent edges
void localCellSizes(const std::vector<cv::Point2f>& centers,
                    int cols, int rows, float cellSizes[4]) {
    int ci[4] = {0, cols - 1, cols * rows - 1, cols * (rows - 1)};
    // Adjacent indices for each corner (same row, same col)
    int adjR[4] = {1, cols - 2, cols * rows - 2, cols * (rows - 1) + 1};
    int adjC[4] = {cols, cols * 2 - 1, cols * (rows - 1) - 1, cols * (rows - 2)};

    for (int k = 0; k < 4; ++k) {
        float dRow = static_cast<float>(cv::norm(centers[adjR[k]] - centers[ci[k]]));
        float dCol = static_cast<float>(cv::norm(centers[adjC[k]] - centers[ci[k]]));
        cellSizes[k] = (dRow + dCol) * 0.5f;
    }
}

// Measure dark-pixel ratio in the outward region of a corner.
// Uses multi-scale sampling: scans at several offsets from the
// corner and returns the maximum dark ratio found.
float cornerDarkRatio(const cv::Mat& gray,
                      cv::Point2f corner, cv::Point2f outwardDir,
                      float cellSize) {
    float bestRatio = 0;
    // Sample at multiple offsets: 0.3, 0.5, 0.7 cellSize outward
    for (float offset : {0.3f, 0.5f, 0.7f}) {
        cv::Point2f sc = corner + outwardDir * (cellSize * offset);
        int halfR = std::max(3, (int)(cellSize * 0.35f));
        cv::Rect roi((int)sc.x - halfR, (int)sc.y - halfR, 2 * halfR, 2 * halfR);
        roi &= cv::Rect(0, 0, gray.cols, gray.rows);
        if (roi.width <= 0 || roi.height <= 0) continue;
        float ratio = 100.0f * cv::countNonZero(gray(roi) < 100) / (roi.width * roi.height);
        bestRatio = std::max(bestRatio, ratio);
    }
    return bestRatio;
}

// Compute dark ratios for all 4 corners
void computeDarkRatios(const cv::Mat& gray,
                       const std::vector<cv::Point2f>& centers,
                       int cols, int rows, float ratios[4]) {
    int ci[4] = {0, cols - 1, cols * rows - 1, cols * (rows - 1)};
    cv::Point2f corners[4];
    for (int k = 0; k < 4; ++k) corners[k] = centers[ci[k]];
    cv::Point2f center = (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;
    float lcSizes[4];
    localCellSizes(centers, cols, rows, lcSizes);
    for (int k = 0; k < 4; ++k) {
        cv::Point2f dir = corners[k] - center;
        float len = static_cast<float>(cv::norm(dir));
        if (len < 1e-3f) { ratios[k] = 0; continue; }
        dir /= len;
        ratios[k] = cornerDarkRatio(gray, corners[k], dir, lcSizes[k]);
    }
}

// Find best and second-best corner indices from dark ratios
void findTopTwo(const float ratios[4], int& bestIdx, float& bestVal, float& secondVal) {
    bestIdx = 0; bestVal = ratios[0]; secondVal = 0;
    for (int k = 1; k < 4; ++k) {
        if (ratios[k] > bestVal) { secondVal = bestVal; bestIdx = k; bestVal = ratios[k]; }
        else if (ratios[k] > secondVal) secondVal = ratios[k];
    }
}

// Strict triangle detection with perspective-aware threshold.
// Under heavy perspective, the near-side border corners also accumulate
// dark pixels, so the ratio gap between triangle corner and second-best
// shrinks. We relax the threshold when perspective distortion is high.
int detectTriangleCorner(const cv::Mat& gray,
                         const std::vector<cv::Point2f>& centers,
                         int cols, int rows,
                         float darkRatios[4] = nullptr) {
    float ratios[4];
    computeDarkRatios(gray, centers, cols, rows, ratios);
    if (darkRatios) for (int k = 0; k < 4; ++k) darkRatios[k] = ratios[k];

    int bestIdx; float bestVal, secondVal;
    findTopTwo(ratios, bestIdx, bestVal, secondVal);

    // Measure perspective distortion: compare opposite edge lengths
    int ci[4] = {0, cols - 1, cols * rows - 1, cols * (rows - 1)};
    float edge01 = static_cast<float>(cv::norm(centers[ci[1]] - centers[ci[0]]));
    float edge32 = static_cast<float>(cv::norm(centers[ci[2]] - centers[ci[3]]));
    float edge03 = static_cast<float>(cv::norm(centers[ci[3]] - centers[ci[0]]));
    float edge12 = static_cast<float>(cv::norm(centers[ci[2]] - centers[ci[1]]));
    float perspRow = std::abs(edge01 - edge32) / std::max(edge01, edge32);
    float perspCol = std::abs(edge03 - edge12) / std::max(edge03, edge12);
    float maxPersp = std::max(perspRow, perspCol);

    // Adaptive threshold: 1.2x normally, relax to 1.1x under heavy perspective
    float threshold = (maxPersp > 0.10f) ? 1.10f : 1.20f;

    if (bestVal >= 12.0f && bestVal >= secondVal * threshold)
        return bestIdx;
    return -1;
}

// Relaxed detection with temporal continuity.
// Two modes:
//   a) prevTri is valid: accept prevTri if it's within 10% of the best
//   b) prevTri invalid (lost too long): accept overall best if marginally clear
int detectTriangleRelaxed(const cv::Mat& gray,
                          const std::vector<cv::Point2f>& centers,
                          int cols, int rows, int prevTri,
                          bool prevValid) {
    float ratios[4];
    computeDarkRatios(gray, centers, cols, rows, ratios);

    int bestIdx; float bestVal, secondVal;
    findTopTwo(ratios, bestIdx, bestVal, secondVal);

    if (prevValid && prevTri >= 0) {
        // Mode a: accept prevTri if it's the best or within 10% of best
        float prevVal = ratios[prevTri];
        if (prevVal >= bestVal * 0.90f && prevVal >= 12.0f)
            return prevTri;
        // prevTri is no longer the best — the board may have rotated.
        // Fall through to mode b to accept the new best.
    }

    // Mode b: accept overall best with a relaxed threshold (1.05x, abs >= 15%)
    if (bestVal >= 15.0f && bestVal >= secondVal * 1.05f)
        return bestIdx;

    return -1;
}

// ── Order corners using the triangle feature ─────────────

struct OrderedCorners {
    cv::Point2f lt, rt, rb, lb;
};

OrderedCorners orderCorners(const std::vector<cv::Point2f>& centers,
                            int cols, int rows, int triCorner) {
    int ci[4] = {0, cols - 1, cols * rows - 1, cols * (rows - 1)};
    cv::Point2f corners[4];
    for (int k = 0; k < 4; ++k) corners[k] = centers[ci[k]];

    int adjRow[4] = {1, 0, 3, 2};
    int adjCol[4] = {3, 2, 1, 0};
    int oppMap[4]  = {2, 3, 0, 1};

    int a1 = adjRow[triCorner], a2 = adjCol[triCorner], opp = oppMap[triCorner];
    cv::Point2f d1 = corners[a1] - corners[triCorner];
    cv::Point2f d2 = corners[a2] - corners[triCorner];
    float cross = d1.x * d2.y - d1.y * d2.x;

    int rtIdx = (cross > 0) ? a1 : a2;
    int lbIdx = (cross > 0) ? a2 : a1;

    return { corners[triCorner], corners[rtIdx], corners[opp], corners[lbIdx] };
}

// Reorder the 49 detected centers to match the physical LT/RT/RB/LB ordering.
// findCirclesGrid returns centers in row-major order starting from one corner;
// we need to permute them so index 0 = physical LT, etc.
std::vector<cv::Point2f> reorderCenters(const std::vector<cv::Point2f>& centers,
                                        int cols, int rows, int triCorner) {
    // The detected grid has 4 possible orientations. We need to find the
    // permutation that maps detected order → physical order (LT first).
    //
    // Detected grid corners: 0=detTL, 1=detTR, 2=detBR, 3=detBL
    // triCorner tells us which detected corner is physical LT.
    //
    // If triCorner=0: detected order is already correct (detTL=physLT)
    // If triCorner=1: detected grid is rotated 90° CW → need to rotate
    // If triCorner=2: detected grid is rotated 180° → need to flip
    // If triCorner=3: detected grid is rotated 90° CCW → need to rotate

    std::vector<cv::Point2f> reordered(cols * rows);

    // Build mapping: for each physical (r,c), which detected (dr,dc)?
    // triCorner determines the rotation:
    //   0: phys(r,c) = det(r, c)
    //   1: phys(r,c) = det(c, rows-1-r)  — 90° CW
    //   2: phys(r,c) = det(rows-1-r, cols-1-c)  — 180°
    //   3: phys(r,c) = det(cols-1-c, r)  — 90° CCW

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int dr, dc;
            switch (triCorner) {
                case 0: dr = r;            dc = c;            break;
                case 1: dr = c;            dc = rows - 1 - r; break;
                case 2: dr = rows - 1 - r; dc = cols - 1 - c; break;
                case 3: dr = cols - 1 - c; dc = r;            break;
                default: dr = r;           dc = c;            break;
            }
            reordered[r * cols + c] = centers[dr * cols + dc];
        }
    }
    return reordered;
}

// ── Detect physical border corner ────────────────────────

cv::Point2f findBorderCorner(const cv::Mat& gray,
                             cv::Point2f gridCorner, cv::Point2f dir,
                             float cellSize) {
    float maxDist = cellSize * 2.0f;
    float step = std::max(1.0f, cellSize * 0.05f);
    cv::Point2f best = gridCorner;
    bool inWhite = false;

    for (float d = 0; d <= maxDist; d += step) {
        cv::Point2f p = gridCorner + dir * d;
        int px = (int)p.x, py = (int)p.y;
        if (px < 0 || px >= gray.cols || py < 0 || py >= gray.rows) break;
        uchar val = gray.at<uchar>(py, px);
        if (val > 120) { inWhite = true; best = p; }
        else if (inWhite && val < 80) { best = gridCorner + dir * (d - step * 0.5f); break; }
    }
    return best;
}

// ── Stage 2: Pose estimation of the calibration board ────

bool runPoseEstimation(const std::string& videoPath, const CalibParams& cp,
                       const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                       const std::string& outputPath) {
    std::cout << "\n=== 位姿估计 ===\n"
              << "视频: " << videoPath << "\n"
              << "相机内参矩阵:\n" << cameraMatrix << "\n";

    int cols = cp.patternSize.width, rows = cp.patternSize.height;
    float boardW = (cols - 1) * (float)cp.squareSize;
    float boardH = (rows - 1) * (float)cp.squareSize;

    // Full 49-point object points for PnP
    auto objPoints = circleGridObjectPoints(cp);
    // 4 corner object points for visualization
    std::vector<cv::Point3f> boardCorners3D = {
        objPoints[0], objPoints[cols - 1], objPoints[cols * rows - 1], objPoints[cols * (rows - 1)]
    };

    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) { std::cerr << "Cannot open " << videoPath << "\n"; return false; }
    int W = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int H = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    int total = (int)cap.get(cv::CAP_PROP_FRAME_COUNT);
    double fps = cap.get(cv::CAP_PROP_FPS);
    std::cout << "分辨率: " << W << "x" << H << "  fps=" << fps
              << "  帧数=" << total << "\n"
              << "标定板物理尺寸: " << boardW * 100 << " x "
              << boardH * 100 << " cm\n";

    cv::VideoWriter writer;
    if (!outputPath.empty()) {
        writer.open(outputPath, cv::CAP_FFMPEG,
                    cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                    (fps > 0 ? fps : 25.0), {W, H});
    }

    cv::Scalar cornerColors[4] = {{0,0,255}, {0,255,255}, {0,255,0}, {255,0,0}};

    cv::Mat frame, gray;
    int frameIdx = 0, poseCount = 0, triCount = 0;
    int triStrict = 0, triRelaxed = 0;
    std::vector<double> distances, txs, tys, tzs;

    float axisLen = (float)(cp.squareSize * 3);
    std::vector<cv::Point3f> axis3D = {{0,0,0}, {axisLen,0,0}, {0,axisLen,0}, {0,0,-axisLen}};

    // Temporal state
    int prevTriCorner = -1;
    int lostFrames = 0;

    while (cap.read(frame)) {
        ++frameIdx;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> centers;
        bool found = cv::findCirclesGrid(gray, cp.patternSize, centers,
                                         cv::CALIB_CB_SYMMETRIC_GRID |
                                         cv::CALIB_CB_CLUSTERING);

        cv::Vec3d rvec, tvec;
        bool havePose = false;
        int triCorner = -1;
        OrderedCorners oc{};
        std::string triMethod;

        if (found) {
            // 1) Try strict triangle detection
            triCorner = detectTriangleCorner(gray, centers, cols, rows);

            if (triCorner >= 0) {
                triMethod = "strict";
                ++triStrict;
            } else if (lostFrames < 60) {
                // 2) Try relaxed detection with temporal continuity
                triCorner = detectTriangleRelaxed(gray, centers, cols, rows,
                                                  prevTriCorner, lostFrames < 15);
                if (triCorner >= 0) {
                    triMethod = "relaxed";
                    ++triRelaxed;
                }
            }

            if (triCorner >= 0) {
                ++triCount;
                prevTriCorner = triCorner;
                lostFrames = 0;

                oc = orderCorners(centers, cols, rows, triCorner);

                // Use all 49 reordered centers for PnP (more robust than 4 corners)
                auto reordered = reorderCenters(centers, cols, rows, triCorner);
                if (cv::solvePnP(objPoints, reordered, cameraMatrix,
                                 distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
                    havePose = true;
                    ++poseCount;
                    double dist = std::sqrt(tvec[0]*tvec[0] + tvec[1]*tvec[1] + tvec[2]*tvec[2]);
                    distances.push_back(dist);
                    txs.push_back(tvec[0]);
                    tys.push_back(tvec[1]);
                    tzs.push_back(tvec[2]);
                }
            }
        } else {
            ++lostFrames;
        }

        // ── Draw ──
        cv::Mat out = frame.clone();

        if (found) {
            for (auto& c : centers)
                cv::circle(out, c, 2, {128, 128, 128}, -1);

            cv::Point2f drawCorners[4];
            const char* drawLabels[4];
            bool haveOrder = (triCorner >= 0);

            if (haveOrder) {
                drawCorners[0] = oc.lt; drawCorners[1] = oc.rt;
                drawCorners[2] = oc.rb; drawCorners[3] = oc.lb;
                drawLabels[0] = "LT"; drawLabels[1] = "RT";
                drawLabels[2] = "RB"; drawLabels[3] = "LB";
            } else {
                int ci[4] = {0, cols-1, cols*rows-1, cols*(rows-1)};
                for (int k = 0; k < 4; k++) {
                    drawCorners[k] = centers[ci[k]];
                    drawLabels[k] = "?";
                }
            }

            float lcSizes[4];
            localCellSizes(centers, cols, rows, lcSizes);
            float avgCell = std::accumulate(lcSizes, lcSizes+4, 0.0f) / 4;
            cv::Point2f center = (drawCorners[0] + drawCorners[1] +
                                  drawCorners[2] + drawCorners[3]) * 0.25f;

            if (avgCell > 30 && haveOrder) {
                cv::Point2f borderCorners[4];
                for (int k = 0; k < 4; k++) {
                    cv::Point2f dir = drawCorners[k] - center;
                    dir /= cv::norm(dir);
                    borderCorners[k] = findBorderCorner(gray, drawCorners[k], dir, lcSizes[k]);
                }
                std::vector<cv::Point> pts;
                for (int k = 0; k < 4; k++)
                    pts.emplace_back((int)borderCorners[k].x, (int)borderCorners[k].y);
                cv::polylines(out, pts, true, {0, 200, 0}, 2);
                for (int k = 0; k < 4; k++) {
                    cv::circle(out, borderCorners[k], 6, cornerColors[k], 2);
                    cv::putText(out, drawLabels[k],
                                cv::Point((int)borderCorners[k].x + 8, (int)borderCorners[k].y - 8),
                                cv::FONT_HERSHEY_SIMPLEX, 0.7, cornerColors[k], 2);
                }
            } else {
                std::vector<cv::Point> pts;
                for (int k = 0; k < 4; k++)
                    pts.emplace_back((int)drawCorners[k].x, (int)drawCorners[k].y);
                if (haveOrder)
                    cv::polylines(out, pts, true, {0, 200, 0}, 2);
                for (int k = 0; k < 4; k++) {
                    cv::circle(out, drawCorners[k], 5, cornerColors[k], 2);
                    cv::putText(out, drawLabels[k],
                                cv::Point((int)drawCorners[k].x + 8, (int)drawCorners[k].y - 8),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cornerColors[k], 2);
                }
            }
        }

        if (havePose) {
            std::vector<cv::Point2f> axis2D;
            cv::projectPoints(axis3D, rvec, tvec, cameraMatrix, distCoeffs, axis2D);
            cv::line(out, axis2D[0], axis2D[1], {0, 0, 255}, 3);   // X red
            cv::line(out, axis2D[0], axis2D[2], {0, 255, 0}, 3);   // Y green
            cv::line(out, axis2D[0], axis2D[3], {255, 0, 0}, 3);   // Z blue

            double dist = std::sqrt(tvec[0]*tvec[0] + tvec[1]*tvec[1] + tvec[2]*tvec[2]);
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            double pitch = std::asin(-R.at<double>(2, 0)) * 180.0 / CV_PI;
            double yaw   = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2)) * 180.0 / CV_PI;
            double roll  = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0)) * 180.0 / CV_PI;

            std::string info = cv::format("t=(%.3f,%.3f,%.3f)m  d=%.3fm  RPY=(%.1f,%.1f,%.1f)deg  [%s]",
                                          tvec[0], tvec[1], tvec[2], dist, roll, pitch, yaw,
                                          triMethod.c_str());
            cv::putText(out, info, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1);
            cv::putText(out, "POSE ESTIMATED", {20, 70}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
        } else if (found) {
            const char* msg = (triCorner >= 0) ? "DETECTED (PnP failed)" : "DETECTED (no triangle)";
            cv::putText(out, msg, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 165, 255}, 2);
        } else {
            cv::putText(out, "NOT DETECTED", {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 0, 255}, 2);
        }

        cv::putText(out, "frame " + std::to_string(frameIdx) + "/" + std::to_string(total),
                    {20, H - 20}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 1);

        if (writer.isOpened()) writer.write(out);
    }
    cap.release();
    if (writer.isOpened()) writer.release();

    // ── Statistics ──
    auto stats = [](const std::vector<double>& v) {
        if (v.empty()) return std::string("(no data)");
        double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
        double minV = *std::min_element(v.begin(), v.end());
        double maxV = *std::max_element(v.begin(), v.end());
        double var = 0;
        for (double d : v) var += (d - mean) * (d - mean);
        double std_ = std::sqrt(var / v.size());
        return cv::format("mean=%.4f  min=%.4f  max=%.4f  std=%.4f", mean, minV, maxV, std_);
    };

    std::cout << "\n位姿估计完成.\n"
              << "  位姿估计帧数: " << poseCount << "/" << frameIdx
              << " (" << 100.0 * poseCount / frameIdx << "%)\n"
              << "  三角形识别帧数: " << triCount << "/" << frameIdx
              << " (" << 100.0 * triCount / frameIdx << "%)\n"
              << "    严格识别: " << triStrict << "  宽松识别(时域): " << triRelaxed << "\n"
              << "  距离 (m): " << stats(distances) << "\n"
              << "  TX (m): " << stats(txs) << "\n"
              << "  TY (m): " << stats(tys) << "\n"
              << "  TZ (m): " << stats(tzs) << "\n";
    return true;
}

} // namespace

int main(int argc, char** argv) {
    std::string inputPath    = "data/raw/calibration_video.avi";
    std::string calibOutput  = "output/calib_results.yaml";
    std::string poseOutput   = "output/pose_result.avi";
    bool doCalib = false, doPose = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--input" && i + 1 < argc) {
            inputPath = argv[++i];
        } else if (a == "--calib" && i + 1 < argc) {
            calibOutput = argv[++i];
            doCalib = true;
        } else if (a == "--pose" && i + 1 < argc) {
            poseOutput = argv[++i];
            doPose = true;
        } else {
            std::cerr << "Usage: " << argv[0]
                      << " [--input <video>] [--calib <yaml>] [--pose <video>]\n"
                      << "  --input  输入视频路径（默认 data/raw/calibration_video.avi）\n"
                      << "  --calib  执行标定，输出标定结果到指定 yaml 路径\n"
                      << "  --pose   执行位姿估计，输出标注视频到指定路径\n"
                      << "  不带 --calib 和 --pose 时，同时执行标定和位姿估计\n";
            return 1;
        }
    }
    if (!doCalib && !doPose) { doCalib = true; doPose = true; }

    CalibParams cp;
    cv::Mat cameraMatrix, distCoeffs;

    if (doCalib) {
        if (!runCalibration(inputPath, cp, cameraMatrix, distCoeffs, calibOutput))
            return 1;
    }
    if (doPose) {
        if (!doCalib) {
            cv::FileStorage fs(calibOutput, cv::FileStorage::READ);
            if (!fs.isOpened()) {
                std::cerr << "Cannot read " << calibOutput
                          << ". Run calibration first.\n";
                return 1;
            }
            fs["camera_matrix"] >> cameraMatrix;
            fs["distortion_coefficients"] >> distCoeffs;
            fs.release();
        }
        if (!runPoseEstimation(inputPath, cp, cameraMatrix, distCoeffs, poseOutput))
            return 1;
    }
    return 0;
}
