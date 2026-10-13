# 环境配置：

Ubuntu 24.04

GCC 13.3.0

CMake >= 3.16

OpenCV 4.6.0

C++17 及以上

# 编译：

`IncludePath` 加入 `/usr/include/opencv4/**`

终端运行如下指令

```
cmake -S src/chenzinan_50059063 -B build/chenzinan_50059063 -DCMAKE_BUILD_TYPE=Release

cmake --build build/chenzinan_50059063 --parallel
```

# 运行：

终端运行如下指令

```
./build/chenzinan_50059063/PJ ./data/raw/marker_video.avi ./data/raw/calibration_video.avi ./
```

输出 marker_video_annotated.mp4 标志物检测视频，和 calibration_video_demo.avi 位姿估计视频。