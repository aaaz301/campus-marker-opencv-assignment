# 白色灯板四角点检测示例

这个示例用 C++17 和 OpenCV 逐帧处理 `videos/` 中的视频。

## 构建与运行

需要 CMake 3.16 及以上版本、C++17 编译器和支持 MP4 读写的 OpenCV 4 开发包。

每段视频会输出以下内容，全部与输入视频保持相同帧率，由 OpenCV 的 `VideoWriter` 直接写为 `mp4v` 编码的 MP4。程序不再进行外部转码；新生成的视频可能无法在 VS Code 内置播放器中直接打开，可使用支持 MPEG-4 Part 2 的播放器查看。

- `results/demo.mp4`：原尺寸的最终四角点标注视频。

## 完整命令
```bash
cmake -S . -B build
cmake --build build -j
./build/white_demo videos/marker_video.avi results
```