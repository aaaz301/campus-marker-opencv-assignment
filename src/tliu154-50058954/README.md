







----------------------------------作业声明------------------------------------------

项目的工程规范框架，main里一些filesystem，.yaml和cmakelist是AI写的
src和include是我自己写的，

由于我的GUI一直出问题，调参很麻烦，调了好久效果依然不好，但是逻辑是没有问题的。

很抱歉投入时间有限，作业仓库的很多文件管理不完善，有点混乱。

------------------------------------------------------------------------------------









# 蓝色装甲板四角点检测示例

这个示例用 C++17 和 OpenCV 逐帧处理 `videos/` 中的两段视频。它演示从 BGR 原图、滤波、增强、二值化、形态学、轮廓、旋转矩形、灯条筛选、灯条配对到装甲板四角点的完整流程。源码在每一步前都有面向新生的中文注释。

## 构建与运行

需要 CMake 3.16 及以上版本、C++17 编译器和支持 MP4 读写的 OpenCV 4 开发包。仓库中的 MP4 使用 Git LFS；克隆后若看到的是文本指针，请先运行 `git lfs pull`。

```bash
cmake -S . -B build
cmake --build build -j
./build/armor_demo videos/example1.mp4 results
./build/armor_demo videos/example2.mp4 results
```

## 当前项目用法（`armor_detector`）

**WSL 下调参推荐用 YAML 热重载，而不是 GUI trackbar。**

WSL 的 GUI 环境经常不稳定（窗口透明、不刷新、trackbar 回调 warning），因此 `USE_TRACKBAR` 默认 `OFF`。需要 GUI 时再手动打开：

```bash
cmake -S . -B build -DUSE_TRACKBAR=ON
cmake --build build -j
```

默认无 GUI 构建：

```bash
cmake -S . -B build
cmake --build build -j

# 图片
./build/armor_detector config/default_params.yaml data/test_image.jpg red

# 视频
./build/armor_detector config/default_params.yaml videos/example1.mp4 red output.mp4

# 摄像头
./build/armor_detector config/default_params.yaml 0 red output.mp4
```

### 输出位置

运行时会打印绝对路径，例如：

```
[INFO] Debug images will be saved to: "/home/liutao/develop/rm27_opencv_tutorial/data/output"
```

- **中间调试图**：`data/output/`
  - `01_binary.jpg`：二值化掩码
  - `02_light_bars.jpg`：检测到的灯条
  - `03_armors.jpg`：配对出的装甲板
- **结果视频**：如果命令行指定了第 5 个参数（如 `output.mp4`），会保存带装甲板标注的视频。
- **实时窗口**：只有 `-DUSE_TRACKBAR=ON` 且 WSL GUI 正常时才会显示。

### 运行时按键

- `q` / `ESC`：退出
- `s`：保存当前参数到 `config/default_params.yaml`
- `r`：立即重新加载 YAML 参数

### YAML 热重载调参

1. 终端 1 运行程序：
   ```bash
   ./build/armor_detector config/default_params.yaml data/test_image.jpg red
   ```
   图片输入处理完会自动退出；视频/摄像头会循环运行。
2. 终端 2 用 VSCode / nano 修改 `config/default_params.yaml`，保存。
3. 程序会自动检测到文件修改并重新加载参数。
4. 看 `data/output/01_binary.jpg`、`02_light_bars.jpg`、`03_armors.jpg` 验证效果。

每段视频会输出以下内容，全部与输入视频保持相同帧率，由 OpenCV 的 `VideoWriter` 直接写为 `mp4v` 编码的 MP4。程序不再进行外部转码；新生成的视频可能无法在 VS Code 内置播放器中直接打开，可使用支持 MPEG-4 Part 2 的播放器查看。

- `results/<视频名>_annotated.mp4`：原尺寸的最终四角点标注视频。
- `results/<视频名>_stages/`：八段原尺寸中间视频，文件名前的 `01`–`08` 对应去噪、增强、二值化、形态学、轮廓、旋转矩形、灯条筛选和灯条配对。原始视频本身就是 BGR 阶段。
- `results/<视频名>_summary.mp4`：逐帧 2×5 拼图视频，按顺序同时展示 BGR 原图、八个中间状态和最终四角点结果；尺寸为 1600×360。
- `results/<视频名>_steps/`：同一拼图每 30 帧及最后一帧保存的 PNG，便于暂停对照。

仓库另附 H.264 标注视频 `example_results/`，方便直接查看效果。拼图中的英文标题与源码中的中文步骤依次对应；OpenCV 自带的 `putText` 无法直接显示中文。输入视频不会被修改。

## 流程与参数

1. **去噪**：3×3 中值滤波，减少孤立噪点。
2. **增强**：在 HSV 空间用 CLAHE 增强亮度 V，参数为 `clipLimit=2.0`、`tileGridSize=8×8`。
3. **二值化**：参考 [auto-aim-new](https://github.com/PnX-HKUSTGZ/auto-aim-new) 的颜色预处理思路，直接分离 BGR 中的蓝色和红色通道，取两者的绝对差得到单通道颜色差异图，再用固定阈值 `40` 二值化。
4. **形态学**：用 `dilate` 后接 `erode` 实现 3×5 竖向闭运算、连接小断点；再用 `erode` 后接 `dilate` 实现 3×3 开运算、去除小亮点。
5. **轮廓与矩形**：寻找外轮廓，并用 `minAreaRect` 取得倾斜灯条的中心、长宽和方向。
6. **灯条筛选**：轮廓面积至少 `15 px²`、长轴至少 `18 px`、长宽比至少 `1.9`、相对竖直方向的倾角不超过 `30°`。视频中部分灯条会在二值图里变亮、变粗，旧的 `2.5` 下限会漏掉它们。
7. **灯条配对**：长度比不超过 `1.7`、倾角差不超过 `25°`、中心高度差不超过平均长度的 `0.5` 倍、水平间距为平均长度的 `1.0–2.5` 倍；仅在整帧无配对时将上限放宽到 `2.6` 倍。两根灯条之间若已有同高度、相近长度的灯条，就不允许跨越配对。两灯条之间的内部区域还需有至少 `5%` 的灰度大于 `80` 的像素，利用视频里浅色数字排除空支架。候选对综合几何与内部亮度择优，每根灯条最多只能属于一块装甲板。
8. **四角点**：取左右灯条长轴的上下端点，按左上、右上、右下、左下排序，在输出视频中画绿色四边形和红点。

这是便于理解流程的二维几何示例。遮挡、过曝或仅露出一根灯条时可能无法配对；四角点是灯条端点给出的近似位置，不包含数字识别、跨帧跟踪或三维位姿估计。

程序会报告“至少检出一组的帧”占比，用于衡量这两段素材的逐帧覆盖率。这个数字不等于所有可见装甲板的召回率或四角点定位准确率；后两者需要人工标注真值才能计算。
