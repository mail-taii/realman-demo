import sys
import cv2
import numpy as np
from detect_all import segment_image

# 相机内参
color_intr = {"ppx": 292.794675, "ppy": 275.145308, "fx": 660.609370, "fy": 656.300601}

# 旋转矩阵和位移向量
rotation_matrix = np.array([[9.42887983e-01, -3.15123418e-01, 1.07979084e-01],
                            [3.29406364e-01, 9.30261836e-01, -1.61568447e-01],
                            [-4.95348198e-02, 1.87909944e-01, 9.80936366e-01]])
translation_vector = np.array([9.30302738e+01, -7.15735229e+01, 4.91918941e+02])

global color_img
color_img = None  # RGB图像

def get_object_pixel(mask):
    # 获取掩码区域的像素坐标
    ys, xs = np.where(mask == 255)
    if len(xs) == 0 or len(ys) == 0:
        return None
    # 取掩码中心点
    cx = int(np.mean(xs))
    cy = int(np.mean(ys))
    return cx, cy

def pixel_to_camera_coords(cx, cy, depth_img, intr):
    # 获取像素点的深度值
    depth = depth_img[cy, cx] if depth_img is not None else 500  # 假设深度为500mm
    # 像素坐标转相机坐标
    x = (cx - intr["ppx"]) * depth / intr["fx"]
    y = (cy - intr["ppy"]) * depth / intr["fy"]
    z = depth
    return np.array([x, y, z])

def callback(color_img, depth_img=None):
    cv2.imshow("RGB Image", color_img)
    k = cv2.waitKey(30) & 0xFF

    if color_img is None:
        print("等待图像数据更新...")
        return

    mask = segment_image(color_img)

    if mask is not None:
        mask = mask.astype(np.uint8)
        print("掩码尺寸:", mask.shape)

        masked_img = cv2.bitwise_and(color_img, color_img, mask=mask)
        masked_img[mask == 255] = [255, 255, 255]
        cv2.imshow("Masked Image", masked_img)

        # 获取物体像素坐标
        obj_pixel = get_object_pixel(mask)
        if obj_pixel is not None:
            cx, cy = obj_pixel
            # 获取相机系坐标
            obj_cam = pixel_to_camera_coords(cx, cy, depth_img, color_intr)
            print(f"物体像素坐标: ({cx}, {cy})")
            print(f"物体相机系坐标: {obj_cam}")
            # 在图像上标注
            cv2.circle(masked_img, (cx, cy), 5, (0,0,255), -1)
            cv2.putText(masked_img, f"Cam: {obj_cam.astype(int)}", (cx, cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
            cv2.imshow("Masked Image", masked_img)
            cv2.waitKey(0)
        else:
            print("未检测到物体像素点")
    else:
        print("掩码为空，无法进行操作")

