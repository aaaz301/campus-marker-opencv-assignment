import cv2
import numpy as np

def get_rect_info(points):
    pts = np.array(points, dtype=np.float32)
    rect = cv2.minAreaRect(pts)
    print(f"  size={rect[1]}, angle={rect[2]:.2f}")
    return rect

print("=== 竖直细长条 (height >> width) ===")
pts = [(100, 50), (105, 50), (105, 200), (100, 200)]
r = get_rect_info(pts)

print("=== 水平细长条 (width >> height) ===")
pts = [(50, 100), (200, 100), (200, 105), (50, 105)]
r = get_rect_info(pts)

print("=== 倾斜细长条 45度 ===")
pts = [(100, 100), (150, 150), (145, 155), (95, 105)]
r = get_rect_info(pts)

print("=== 实际轮廓测试 ===")
# 构造一个细长轮廓
contour = np.array([
    [100, 50], [105, 50], [105, 200], [100, 200]
], dtype=np.int32)
rect = cv2.minAreaRect(contour)
print(f"contour size={rect[1]}, angle={rect[2]:.2f}")
