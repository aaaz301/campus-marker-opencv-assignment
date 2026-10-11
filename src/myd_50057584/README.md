# 校内赛标志物识别 – myd_50057584

基于传统 OpenCV C++ 的 RoboMaster 校内赛标志物（"MARK灯"）识别方案，包含必做任务（标志物检测）和挑战任务（相机标定 + 位姿估计）。

## 依赖环境

| 组件 | 版本（已测试） |
|------|--------------|
| 操作系统 | Arch Linux |
| 编译器 | GCC 16.2.1 |
| C++ 标准 | C++17 |
| CMake | 4.4.3 |
| OpenCV | 5.0.0 |

Arch Linux 安装依赖：

```bash
sudo pacman -S cmake gcc opencv
```

## 编译

从仓库根目录执行：

```bash
cmake -S src/myd_50057584 -B build/myd_50057584 -DCMAKE_BUILD_TYPE=Release
cmake --build build/myd_50057584 --parallel
```

编译启用 `-Wall -Wextra`，零警告。

## 必做任务：标志物检测

```bash
# 使用默认输入视频，输出标注视频
./build/myd_50057584/marker_detect --output output/result.avi

# 指定输入视频
./build/myd_50057584/marker_detect --input data/raw/marker_video.avi --output output/result.avi

# 仅打印结果，不输出视频
./build/myd_50057584/marker_detect

# 输出调试掩膜视频
./build/myd_50057584/marker_detect --output output/result.avi --debug output/debug
```

### 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--input <video>` | 输入视频路径 | `data/raw/marker_video.avi` |
| `--output <video>` | 输出标注视频路径（可选） | 无 |
| `--debug <dir>` | 调试掩膜视频输出目录（可选） | 无 |

### 输出

- **stdout**：逐帧检测状态和角点坐标（`LT, RT, RB, LB`，图像像素坐标）
- **输出视频**：绿色标注框 + 彩色角点 + `DETECTED` / `NOT DETECTED` 标签

### 角点顺序

角点按 **LT → RT → RB → LB**（左上 → 右上 → 右下 → 左下）排序，使用极角法保证旋转稳定性。

## 挑战任务：相机标定 + 位姿估计

挑战任务使用 `data/raw/calibration_video.avi`，标定目标为 **7×7 对称圆点阵列**，间距 **0.03 m**。

```bash
# 同时执行标定和位姿估计（默认输入和输出路径）
./build/myd_50057584/calibrate

# 仅标定，指定输出 yaml 路径
./build/myd_50057584/calibrate --calib output/calib.yaml

# 仅位姿估计，指定输出视频路径（需先标定）
./build/myd_50057584/calibrate --pose output/pose.avi

# 同时执行，指定所有路径
./build/myd_50057584/calibrate --input data/raw/calibration_video.avi --calib output/calib.yaml --pose output/pose.avi
```

### 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--input <video>` | 输入视频路径 | `data/raw/calibration_video.avi` |
| `--calib <yaml>` | 执行标定，输出标定结果到指定路径 | `output/calib_results.yaml` |
| `--pose <video>` | 执行位姿估计，输出标注视频到指定路径 | `output/pose_result.avi` |

不带 `--calib` 和 `--pose` 时，同时执行标定和位姿估计。

### 标定输出

yaml 文件包含：相机内参矩阵（3×3）、畸变系数（k1, k2, p1, p2, k3）、图像分辨率、RMS 重投影误差、使用帧数、标定板类型和尺寸。

### 位姿估计输出

每帧检测到圆点阵列且三角形角点识别成功时输出：旋转向量 rvec、平移向量 tvec（米）、3D 坐标轴（X红/Y绿/Z蓝）、RPY 角度（度）。视频中标注 4 个顶点（LT/RT/RB/LB）和黑色边框轮廓。

### 三角形角点识别

标定板的黑色边框有一个角向内多了一个黑色斜三角形，使图形不对称。利用该特征：

1. 检测 49 个圆心后，使用**自适应局部格间距**（每个角点从其相邻边计算）和**多尺度暗像素采样**（0.3/0.5/0.7 倍格间距）统计每个角点向外侧的暗像素比例。
2. 有三角形的角点暗像素比例显著更高。**严格检测**选最高且明确领先的角点，阈值自适应：透视畸变 > 10% 时从 1.2× 放宽到 1.1×。
3. 严格检测失败时，使用**两级宽松回退**：模式 a（丢失 < 15 帧）检查上一帧角点是否仍为最优（10% 容差）；模式 b（丢失 < 60 帧）接受当前帧最优角点（1.05× 阈值）。
4. 三角形角点定义为物理 LT（左上角），用叉积法确定其余 3 个角点的顺序（RT/RB/LB）。
5. 将 49 个圆心重排为物理顺序，使用**全部 49 点**进行 PnP 位姿估计（比仅用 4 角点更鲁棒）。
6. 在输出视频中用不同颜色标注 4 个顶点：LT（红）、RT（黄）、RB（绿）、LB（蓝）。

识别率：严格 97.6% + 时域回退 1.0% = **98.6%**（所有圆点检测成功的帧均能识别）。

### 坐标系定义

| 坐标系 | 定义 |
|--------|------|
| 标定板坐标系 | 原点在三角形角点（物理左上角圆心）；X 向右（朝向 RT），Y 向下（朝向 LB），Z 朝向相机（右手系） |
| 相机坐标系 | OpenCV 约定：X 右，Y 下，Z 前 |
| 单位 | 米 |

## 文档

- `CODE.md` — 算法详解与代码结构
- `REPORT.md` — 方法报告、参数、结果与限制
