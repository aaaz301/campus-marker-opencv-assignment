# CODE.md — 算法详解与代码结构

本文档详细介绍校内赛标志物识别程序的完整算法流程、每个步骤的设计理由，以及源代码的模块结构。包含必做任务（标志物检测）和挑战任务（相机标定 + 位姿估计）两部分。

---

## 目录

1. [必做任务：目标特征分析](#1-必做任务目标特征分析)
2. [必做任务：整体流程概览](#2-必做任务整体流程概览)
3. [必做任务：算法分步详解](#3-必做任务算法分步详解)
4. [必做任务：代码结构](#4-必做任务代码结构)
5. [必做任务：参数表](#5-必做任务参数表)
6. [挑战任务：相机标定](#6-挑战任务相机标定)
7. [挑战任务：位姿估计](#7-挑战任务位姿估计)
8. [构建与运行](#8-构建与运行)

---

## 1. 必做任务：目标特征分析

标志物（"MARK灯"）是一个 **白色正方形面板**（约 190×190 px），表面印有深色几何图案。在暗色环境中白色面板非常突出。

**识别难点：**

1. 深色图案把白色区域分割成多个不连通碎块
2. 边缘出入帧时只有部分可见
3. 需要亚像素级角点精度
4. 连续帧结果不能闪烁抖动
5. 需区分真实标志物与其他白色区域

---

## 2. 必做任务：整体流程概览

```
输入帧 (BGR)
    │
    ▼
 1. HSV白色阈值分割         thresholdWhite()
    S≤60, V≥140 + 3×3开运算
    │
    ▼
 2. 轮廓提取 + Union-Find聚类  findBestCluster()
    面积≥200, 80px邻近合并
    │
    ▼
 3. 尺寸/宽高比验收           validateCluster()
    │
    ▼
 4. Canny边缘密度验证         verifyContent()
    │
    ▼
 5. 角点提取（双模式）
    5a. 凸包+approxPolyDP     extractCornersHull()  ← 主方法
    5b. 闭运算+minAreaRect    extractCornersRect()  ← 后备
    │
    ▼
 6. 角点排序                  orderCorners()
    极角法 → LT/RT/RB/LB
    │
    ▼
 7. 亚像素细化 + 方形约束     refineCorners()
    │
    ▼
 8. 时域平滑 + 遮挡容忍       main() 内
    3帧中值Y + 自适应低通 + ≤5帧沿用
    │
    ▼
    输出标注帧
```

---

## 3. 必做任务：算法分步详解

### 3.1 HSV 白色阈值分割 — `thresholdWhite()`

`S ≤ 60` 排除 LED 橙红色发光（高饱和度）；`V ≥ 140` 排除暗色背景（亮度约 50）。3×3 开运算去除孤立噪点，保留标志物白色边框。

### 3.2 轮廓提取与 Union-Find 聚类 — `findBestCluster()`

标志物深色图案把白色区域分割成碎块。面积 ≥ 200 px² 的轮廓包围框在 80 px 内用并查集（Union-Find）合并，取合并后面积最大的聚类。80 px 足以连接同一标志物的各部分，又不会把不同物体连在一起。

### 3.3 尺寸/宽高比验收 — `validateCluster()`

高度 ∈ [120, 280] px，宽度 ∈ [15, 400] px。完整可见时（宽度 ≥ 150 px）宽高比 ∈ [0.4, 3.0]，部分可见时仅要求 ≥ 0.08，容忍标志物从画面边缘移入/移出。

### 3.4 Canny 边缘密度验证 — `verifyContent()`

标志物内部有深色图案 → 边缘密度适中（0.005 ~ 0.65）。空白白色区域密度极低，过曝区域密度异常高。作为尺寸/形状检查之后的第二道防线。

### 3.5 角点提取 — 双模式策略

**主方法 `extractCornersHull()`**：收集聚类内所有轮廓点 → 凸包 → `approxPolyDP`（ε = 0.02 × 周长，自适应尝试多个值）→ 4 个角点。不依赖固定尺寸形态学核，直接从原始白色像素边界提取。

**后备方法 `extractCornersRect()`**：65×65 闭运算填充内部图案 → 最大轮廓 → 收集落在实心区域内的原始像素点 → `minAreaRect` → 4 个顶点。

### 3.6 角点排序 — `orderCorners()`

计算质心，按相对质心的极角排序得到循环序，再通过上边（中点 Y 最小）确定 LT/RT/RB/LB。

### 3.7 亚像素细化与方形约束 — `refineCorners()`

`cornerSubPix`（5×5 窗口，30 次迭代）达到 0.01 px 精度。轴对齐时（上边角度 < 10°）强制上下边水平、左右边垂直，消除独立角点漂移。

### 3.8 时域平滑与遮挡容忍 — `main()` 内

3 帧 Y 中值滤波去除单帧尖刺 + 自适应指数低通（静止 α = 0.3 强平滑，运动 ≥ 5 px/帧时 α = 1.0 无平滑避免滞后）。遮挡 ≤ 5 帧沿用上一帧，超过 5 帧输出 "NOT DETECTED" 并重置。

---

## 4. 必做任务：代码结构

### 文件组织

```
src/myd_50057584/
├── marker_detect.cpp   # 必做：标志物检测（444行）
├── calibrate.cpp       # 挑战：相机标定 + 位姿估计（667行）
├── CMakeLists.txt      # 构建配置
├── README.md           # 使用说明
├── REPORT.md           # 方法报告
└── CODE.md             # 本文档
```

### 函数职责表（marker_detect.cpp）

| 函数 | 行数 | 职责 |
|------|------|------|
| `thresholdWhite()` | 8 | HSV 阈值 + 形态学开运算 |
| `findBestCluster()` | 53 | 轮廓提取 + Union-Find 聚类 |
| `validateCluster()` | 9 | 尺寸/宽高比验收 |
| `verifyContent()` | 8 | Canny 边缘密度验证 |
| `extractCornersHull()` | 22 | 凸包 + approxPolyDP 角点提取 |
| `extractCornersRect()` | 43 | 闭运算 + minAreaRect 角点提取 |
| `orderCorners()` | 23 | 极角法角点排序 |
| `refineCorners()` | 20 | cornerSubPix + 方形约束 |
| `detectMarker()` | 25 | 顶层编排函数 |
| `drawResult()` | 17 | 绘制标注结果 |
| `main()` | 95 | 视频I/O + 时域平滑 + 遮挡容忍 |

### 函数职责表（calibrate.cpp）

| 函数 | 职责 |
|------|------|
| `circleGridObjectPoints()` | 生成 7×7 圆心的 3D 坐标 |
| `runCalibration()` | 采样帧、检测圆心、标定相机、保存结果 |
| `runPoseEstimation()` | 逐帧检测圆心、solvePnP 估计位姿、绘制 3D 轴 |
| `main()` | 命令行解析、调度标定和位姿估计 |

---

## 5. 必做任务：参数表

| 参数 | 值 | 所在函数 | 说明 |
|------|-----|---------|------|
| `s_max` | 60 | `thresholdWhite` | HSV 饱和度上限 |
| `v_min` | 140 | `thresholdWhite` | HSV 亮度下限 |
| `open_size` | 3 | `thresholdWhite` | 形态学开运算核大小 |
| `min_area` | 200 | `findBestCluster` | 最小轮廓面积（px²） |
| `cluster_dist` | 80 | `findBestCluster` | 聚类距离（px） |
| `min_height` | 120 | `validateCluster` | 最小高度（px） |
| `max_height` | 280 | `validateCluster` | 最大高度（px） |
| `min_width` | 15 | `validateCluster` | 最小宽度（px） |
| `max_width` | 400 | `validateCluster` | 最大宽度（px） |
| `min_aspect_full` | 0.4 | `validateCluster` | 完整可见时最小宽高比 |
| `min_aspect_edge` | 0.08 | `validateCluster` | 部分可见时最小宽高比 |
| `max_aspect` | 3.0 | `validateCluster` | 最大宽高比 |
| `edge_density_min` | 0.005 | `verifyContent` | 最小边缘密度 |
| `edge_density_max` | 0.65 | `verifyContent` | 最大边缘密度 |
| `close_size` | 65 | `extractCornersRect` | 闭运算核大小（px） |
| `approx_eps` | 0.02 | `extractCornersHull` | approxPolyDP 精度比 |
| `subpix_win` | 5×5 | `refineCorners` | 亚像素窗口 |
| `smooth_alpha_min` | 0.3 | `main` | 静止时平滑强度 |
| `motion_threshold` | 5.0 | `main` | 运动阈值（px/帧） |
| `lost_frames_limit` | 5 | `main` | 遮挡容忍帧数 |

---

## 6. 挑战任务：相机标定

### 6.1 标定板

| 属性 | 值 |
|------|-----|
| 图案类型 | 对称圆点阵列（symmetric circles） |
| 网格大小 | 7 × 7 圆点 |
| 圆点间距 | 0.03 m（3 cm） |
| 物理尺寸 | 0.18 m × 0.18 m |

视频源：`data/raw/calibration_video.avi`（6966 帧，1440×1080）

### 6.2 算法流程

```
calibration_video.avi
    │
    ▼
每30帧采样 → findCirclesGrid(7×7, SYMMETRIC_GRID | CLUSTERING)
    │           检测49个圆心
    ▼
收集≥15帧（最多80帧）的圆心2D坐标 + 已知3D坐标
    │
    ▼
cv::calibrateCamera → 内参矩阵 K + 畸变系数 D
    │
    ▼
保存 calib_results.yaml
```

### 6.3 3D 物体点坐标

```cpp
// 原点在左上角圆心，X向右，Y向下，Z=0
for (r = 0..6)
    for (c = 0..6)
        Point3f(c * 0.03, r * 0.03, 0.0);
```

### 6.4 标定结果

| 指标 | 值 |
|------|-----|
| RMS 重投影误差 | 0.076 px |
| 平均每视图误差 | 0.011 px |
| 使用帧数 | 80 帧 |
| fx | 2392.84 px |
| fy | 2393.10 px |
| cx | 706.00 px |
| cy | 563.86 px |
| 畸变系数 | [-0.059, 0.054, -0.0005, -0.0007, 2.117] |

**内参来源：** 由 `cv::calibrateCamera` 从圆点阵列检测结果计算，未使用任何外部标定数据或截图推断。

---

## 7. 挑战任务：位姿估计

### 7.1 坐标系定义

| 坐标系 | 定义 |
|--------|------|
| 标定板坐标系 | 原点在三角形角点（物理左上角圆心）；X 向右（朝向 RT），Y 向下（朝向 LB），Z 朝向相机（右手系）。单位：米。 |
| 相机坐标系 | OpenCV 约定：X 右，Y 下，Z 前（朝向场景）。 |
| tvec | 标定板原点在相机坐标系中的位置（米）。 |
| rvec | 标定板坐标系到相机坐标系的旋转向量（Rodrigues 形式）。 |

### 7.2 三角形角点识别 — `detectTriangleCorner()` + `detectTriangleRelaxed()`

**问题：** `findCirclesGrid` 对 7×7 对称阵列返回 49 个圆心，但起始角点在不同帧中可能映射到不同的物理角点（4 种旋转方向），导致坐标系原点不一致。当标定板倾斜时，透视畸变使板面变为不规则四边形，角点识别更困难。

**标定板的不对称特征：** 黑色边框的一个角向内多了一个黑色斜三角形，使图形不对称。

**改进策略：**

**a) 自适应局部格间距 — `localCellSizes()`**

不再使用全局平均格间距，而是从每个角点的两条相邻边各自计算局部格间距。透视畸变下近侧边长、远侧边短，使用局部间距使采样窗口大小自适应。

**b) 多尺度暗像素采样 — `cornerDarkRatio()`**

对每个角点，在向外侧 0.3、0.5、0.7 倍局部格间距处分别取窗口统计暗像素比例（< 100），取最大值。多尺度采样能适应不同倾斜角度下三角形在图像中的位置变化。

**c) 严格检测 + 时域宽松回退（两级）**

```
findCirclesGrid → 49个圆心
    │
    ▼
严格检测 (detectTriangleCorner):
  对每个角点用局部格间距 + 多尺度采样
  计算透视畸变 = max(|对边长度差比|)
  阈值自适应: 透视>10% 时 1.1×, 否则 1.2×
  选暗像素比例最高且 ≥ 阈值×次高 且 ≥ 12% 的角点
    │
    ├─ 成功 → 三角形角点 = 物理 LT
    └─ 失败 → 宽松检测 (detectTriangleRelaxed, 丢失帧数 < 60):
         模式 a (丢失 < 15): 检查 prevTri 是否在当前帧最优的 10% 容差内
           ├─ 是 → 沿用 prevTri
           └─ 否 → 进入模式 b
         模式 b: 接受当前帧暗像素比例最高角点（≥ 1.05×次高，且 ≥ 15%）
           ├─ 成功 → 三角形角点 = 物理 LT
           └─ 失败 → 返回 -1，跳过该帧位姿估计
```

严格检测覆盖 97.6% 的帧，宽松检测额外恢复 1.0%，合计 98.6%。

### 7.3 角点排序 — `orderCorners()`

三角形角点确定为物理 LT 后，需区分两个相邻角点谁是 RT（X 方向）、谁是 LB（Y 方向）：

```
三角形角点 = LT
    │
    ▼
两个相邻角点 d1, d2（分别沿检测行列方向）
    │
    ▼
计算叉积 cross = d1.x × d2.y - d1.y × d2.x
    │
    ├─ cross > 0 → d1 为 RT（X轴），d2 为 LB（Y轴）
    └─ cross < 0 → d2 为 RT（X轴），d1 为 LB（Y轴）
    │
    ▼
对角角点 = RB
    │
    ▼
输出: LT, RT, RB, LB （物理顺序）
```

叉积法保证右手坐标系（Z 朝向相机），与板面旋转无关。

### 7.4 物理边框角点检测 — `findBorderCorner()`

圆心位于板面内部，物理边框角点在外侧。沿对角线方向从网格角点向外扫描，检测亮度从白（板面 > 120）到暗（边框 < 80）的跳变点，定位黑色边框的物理角点。

### 7.5 全部 49 点重排与 PnP — `reorderCenters()`

确定三角形角点后，将 49 个检测圆心重排为物理顺序（物理 LT 对应索引 0）。`findCirclesGrid` 对对称阵列可能从 4 种旋转方向之一返回点集，`reorderCenters()` 根据 `triCorner` 对 49 个点做对应的旋转变换：

```
triCorner=0: phys(r,c) = det(r, c)            — 无旋转
triCorner=1: phys(r,c) = det(c, rows-1-r)     — 顺时针90°
triCorner=2: phys(r,c) = det(rows-1-r, cols-1-c) — 180°
triCorner=3: phys(r,c) = det(cols-1-c, r)     — 逆时针90°
```

重排后的 49 个 2D 点与已知 3D 物体点一一对应，使用全部 49 点进行 `cv::solvePnP`。相比仅用 4 个角点，49 点约束大幅提高了位姿精度和鲁棒性，尤其在透视畸变较大时。

### 7.6 位姿估计算法流程

```
每帧 → findCirclesGrid 检测49个圆心
    │
    ▼
detectTriangleCorner() → 严格三角形识别
    │     失败 → detectTriangleRelaxed() → 时域回退
    │     仍失败 → 跳过位姿估计
    ▼
orderCorners() → 确定物理顺序 LT, RT, RB, LB
    │
    ▼
reorderCenters() → 49个圆心重排为物理顺序
    │
    ▼
cv::solvePnP(objPoints49, reorderedCenters49, K, D)
    │     → rvec, tvec（使用全部49点，精度更高）
    ▼
绘制:
    - 4个顶点（LT红/RT黄/RB绿/LB蓝）+ 标签
    - 黑色边框轮廓（自适应局部格间距）
    - 3D坐标轴（X红, Y绿, Z蓝, 长度9cm）
    - 平移向量、距离、RPY角度、识别模式[strict/relaxed]
```

### 7.6 位姿结果

| 指标 | 值 |
|------|-----|
| 位姿估计帧数 | 6866 / 6966（98.6%） |
| 严格识别 | 6800 帧（97.6%） |
| 宽松识别（时域回退） | 66 帧（1.0%） |
| 距离：均值 | 1.591 m |
| 距离：范围 | 0.748 – 2.247 m |
| 距离：标准差 | 0.607 m |
| TX：均值 / 标准差 | -0.121 / 0.183 m |
| TY：均值 / 标准差 | -0.151 / 0.098 m |
| TZ：均值 / 标准差 | 1.568 / 0.600 m |

标定板从约 0.77 m 移动到约 2.25 m。TZ 占距离的主要部分（板面正对相机），符合物理预期。

### 7.7 误差与稳定性

- **标定 RMS 0.076 px** — 亚像素精度，说明圆心检测和拟合质量高。
- **98.6% 检测率** — 严格识别 97.6% + 时域回退 1.0%。所有 `findCirclesGrid` 成功的帧均能识别三角形角点。
- **坐标系原点一致性** — 通过三角形特征，所有帧的原点固定在物理左上角，消除了对称阵列的起点歧义。
- **49 点 PnP** — 使用全部 49 个圆心进行位姿估计，比仅用 4 个角点精度更高、更鲁棒。
- **每帧独立估计** — 无时域滤波（仅角点识别使用时域回退），每帧的位姿独立计算。

---

## 8. 构建与运行

### 8.1 编译

```bash
cmake -S src/myd_50057584 -B build/myd_50057584 -DCMAKE_BUILD_TYPE=Release
cmake --build build/myd_50057584 --parallel
```

### 8.2 必做任务运行

```bash
# 使用默认输入视频，输出标注视频
./build/myd_50057584/marker_detect --output output/result.avi

# 指定输入和输出
./build/myd_50057584/marker_detect --input data/raw/marker_video.avi --output output/result.avi
```

### 8.3 挑战任务运行

```bash
# 同时标定和位姿估计（默认路径）
./build/myd_50057584/calibrate

# 仅标定
./build/myd_50057584/calibrate --calib output/calib.yaml

# 仅位姿估计（需先标定）
./build/myd_50057584/calibrate --pose output/pose.avi

# 指定所有路径
./build/myd_50057584/calibrate --input data/raw/calibration_video.avi --calib output/calib.yaml --pose output/pose.avi
```
