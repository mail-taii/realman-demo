#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
增强版YOLO检测和分割程序
检测图像：显示所有识别到的物体
分割图像：只分割置信度最高的物体
"""

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.models.sam import Predictor as SAMPredictor

# 加载yolo模型
model = YOLO("/home/rm/yolo/best1.pt")
# 加载SAM模型
model_weight = '/home/rm/yolo/sam_b.pt'
predictor = SAMPredictor(overrides=dict(conf=0.25, model=model_weight, save=False))

def detect_all_objects(image, conf_threshold=0.1):
    """
    检测图像中的所有物体，返回所有有效检测框和置信度最高的检测框
    """
    results = model.predict(image, device='cuda' if torch.cuda.is_available() else 'cpu')
    
    # 获取检测结果
    boxes = results[0].boxes
    labels = results[0].names
    
    # 获取所有有效的检测框
    all_valid_boxes = []
    all_detected_classes = []
    
    if boxes is not None:
        for idx, box in enumerate(boxes):
            if box.conf > conf_threshold:
                class_id = int(box.cls[0].item())
                class_name = labels[class_id]
                confidence = box.conf.item()
                
                all_valid_boxes.append({
                    'box': box,
                    'class_name': class_name,
                    'confidence': confidence,
                    'class_id': class_id
                })
                all_detected_classes.append(class_name)
    
    print(f"检测到所有类别: {all_detected_classes}")
    print(f"总共检测到 {len(all_valid_boxes)} 个物体")
    
    # 找到置信度最高的检测框
    max_conf_detection = None
    if all_valid_boxes:
        max_conf_detection = max(all_valid_boxes, key=lambda x: x['confidence'])
        print(f"置信度最高的物体: {max_conf_detection['class_name']} (置信度: {max_conf_detection['confidence']:.2f})")
    
    return all_valid_boxes, max_conf_detection

def draw_all_detections(image, all_detections):
    """
    在图像上绘制所有检测到的物体
    """
    annotated_image = image.copy()
    
    # 定义颜色列表，为不同类别分配不同颜色
    colors = [
        (0, 255, 0),    # 绿色
        (255, 0, 0),    # 蓝色
        (0, 0, 255),    # 红色
        (255, 255, 0),  # 青色
        (255, 0, 255),  # 品红色
        (0, 255, 255),  # 黄色
        (128, 0, 128),  # 紫色
        (255, 165, 0),  # 橙色
    ]
    
    for i, detection in enumerate(all_detections):
        box = detection['box']
        class_name = detection['class_name']
        confidence = detection['confidence']
        
        # 获取边界框坐标
        x1, y1, x2, y2 = box.xyxy[0]
        
        # 选择颜色
        color = colors[i % len(colors)]
        
        # 绘制边界框
        cv2.rectangle(annotated_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        
        # 绘制标签
        label = f"{class_name}: {confidence:.2f}"
        
        # 计算文本大小以绘制背景
        (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        
        # 绘制文本背景
        cv2.rectangle(annotated_image, 
                     (int(x1), int(y1) - text_height - baseline - 5),
                     (int(x1) + text_width, int(y1)),
                     color, -1)
        
        # 绘制文本
        cv2.putText(annotated_image, label,
                   (int(x1), int(y1) - baseline - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        print(f"  {i+1}. {class_name}: {confidence:.2f}")
    
    return annotated_image

def process_sam_results(results):
    """
    处理SAM分割结果，返回中心点和掩码
    """
    center = (0, 0)
    mask = None

    if results[0].masks is not None:
        for index, contour in enumerate(results[0].masks.xy):
            contour = contour.astype(np.int32)
            rect = cv2.minAreaRect(contour)
            center, wh, angle = rect
            center = list(map(int, center))

        masks = results[0].masks.data.clone()
        if isinstance(masks[0], torch.Tensor):
            masks = np.array(masks.cpu())
        
        mask = np.zeros_like(masks[0], dtype=np.uint8)
        for i in range(len(masks)):
            mask = cv2.morphologyEx(masks[i].astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            mask[mask == 1] = 255

    return center, mask

def segment_image(color_img, output_path='segmentation_result.png', 
                          draw_path='detection_result.png'):
    """
    增强版图像分割函数
    检测图像：显示所有物体
    分割图像：只分割置信度最高的物体
    """
    
    # 检测所有物体
    all_detections, max_conf_detection = detect_all_objects(color_img)
    
    if not all_detections:
        print("未检测到任何有效物体")
        return color_img
    
    # 绘制所有检测结果
    detection_image = draw_all_detections(color_img, all_detections)
    cv2.imwrite(draw_path, detection_image)
    print(f"✅ 检测结果已保存: {draw_path}")
    
    # 只对置信度最高的物体进行分割
    if max_conf_detection:
        try:
            # 获取置信度最高物体的中心点
            box = max_conf_detection['box']
            x1, y1, x2, y2 = box.xyxy[0]
            x_center = (x1 + x2) / 2
            y_center = (y1 + y2) / 2
            center = (x_center, y_center)
            
            print(f"🔄 对置信度最高的物体进行分割: {max_conf_detection['class_name']}")
            
            # 使用SAM进行分割
            results = predictor(color_img, points=center, labels=[1])
            center, mask = process_sam_results(results)
            
            if mask is not None:
                cv2.imwrite(output_path, mask)
                print(f"✅ 分割结果已保存: {output_path}")
            else:
                print("❌ 未能生成分割掩码")
                
        except Exception as e:
            print(f"❌ 分割过程出错: {e}")
            import traceback
            traceback.print_exc()
    
    return mask

def main():
    """
    主函数：处理单张图片
    """
    source = "/home/rm/yolo/origin.png"
    
    if not os.path.exists(source):
        print(f"❌ 图片文件不存在: {source}")
        return
    
    print(f"🔄 处理图片: {source}")
    image = cv2.imread(source)
    
    if image is None:
        print(f"❌ 无法读取图片: {source}")
        return
    
    # 调用增强版分割函数
    result_image = segment_image(image)
    print("🎉 处理完成！")

if __name__ == "__main__":
    import os
    main()
