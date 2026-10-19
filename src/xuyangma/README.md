# 校内赛标志物识别

## 项目概述

本项目使用 C++、OpenCV 和传统计算机视觉方法实现校内赛标志物识别。

程序支持：

- 连续视频帧中的标志物检测；
- 判断当前帧中是否存在标志物；
- 标志物外边界定位；
- 按 `LT`、`RT`、`RB`、`LB` 顺序输出四个角点；
- 跨帧一致性检查与角点平滑；
- 当不存在可靠目标时正确输出 `NOT DETECTED`；
- 使用对称圆点阵列完成相机标定；
- 使用 PnP 完成标志物位姿估计。

整个流程不使用深度学习模型，也不依赖在线识别服务。

## 运行环境

本项目在以下 Linux 环境中完成测试：

- 操作系统：Ubuntu 24.04.4 LTS
- 编译器：GCC 13.3.0
- C++ 标准：C++17
- CMake：3.28.3
- OpenCV：4.6.0

## 依赖安装

在 Ubuntu 中安装所需依赖：

```bash
sudo apt update
sudo apt install build-essential cmake libopencv-dev
```

可通过以下命令检查 OpenCV 版本：

```bash
pkg-config --modversion opencv4
```

## 编译

在仓库根目录执行：

```bash
cmake -S src/xuyangma -B build/xuyangma -DCMAKE_BUILD_TYPE=Release
cmake --build build/xuyangma --parallel
```

生成的可执行文件位于：

```text
build/xuyangma/marker_detector
```

## 标志物检测

使用仓库提供的视频运行检测：

```bash
./build/xuyangma/marker_detector data/raw/marker_video.avi
```

如需保存处理后的视频：

```bash
./build/xuyangma/marker_detector data/raw/marker_video.avi output.avi
```

检测到标志物时，程序会显示：

- 标志物外边界；
- 四个角点；
- 角点标签 `LT`、`RT`、`RB`、`LB`；
- `DETECTED` 状态。

没有检测到可靠标志物时，程序显示：

```text
NOT DETECTED
```

当标志物离开画面后，程序不会无限沿用上一帧的检测结果。

按 `q` 或 `Esc` 可停止播放。

## 摄像头输入

检测器同时支持摄像头输入：

```bash
./build/xuyangma/marker_detector camera
```

或：

```bash
./build/xuyangma/marker_detector 0
```

## 相机标定

提供的标定板为 `7 x 7` 对称圆点阵列。

相邻圆心之间的实际距离为：

```text
0.03 m
```

执行标定：

```bash
./build/xuyangma/marker_detector \
--calibrate \
data/raw/calibration_video.avi \
7 7 0.03 \
src/xuyangma/calibration.yaml
```

标定程序会输出：

- 相机内参矩阵；
- 畸变系数；
- 标定 RMS 误差；
- 平均重投影误差；
- 实际采用的标定帧编号。

当前标定结果为：

```text
图像分辨率：1440 x 1080
有效帧数：30
RMS 误差：0.06222580695747921
平均重投影误差：0.06089088451195195 px
```

标定结果保存在：

```text
src/xuyangma/calibration.yaml
```

## 位姿估计

位姿估计使用检测得到的标志物四个角点和相机标定参数。

在当前实验中，将标志物外边长按以下尺寸处理：

```text
0.08 m
```

运行位姿估计：

```bash
./build/xuyangma/marker_detector \
--pose \
data/raw/marker_video.avi \
src/xuyangma/calibration.yaml \
0.08
```

位姿模式会显示：

- X 方向平移量；
- Y 方向平移量；
- Z 方向平移量；
- 相机到标志物的距离；
- XYZ 三个坐标轴；
- 重投影误差。

平移量和距离的单位均为米。

## 项目文件

提交目录包含：

```text
src/xuyangma/
├── main.cpp
├── detector.cpp
├── detector.hpp
├── calibration.cpp
├── calibration.hpp
├── CMakeLists.txt
├── README.md
├── REPORT.md
├── calibration.yaml
├── result_demo.avi
├── pose_demo.avi
└── pose_example.png
```

`result_demo.avi`：连续帧标志物识别结果视频。

`pose_demo.avi`：连续帧位姿估计结果视频。

`pose_example.png`：位姿估计结果示例截图。

`calibration.yaml`：相机标定参数。

算法流程、参数选择、失败情况、相机标定和位姿估计的详细说明见 `REPORT.md`。