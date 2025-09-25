#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
from datetime import datetime
import threading
import time
from detect_all import segment_image

class RealTimeCameraStream:
    def __init__(self, topic_name):
        rospy.init_node('realtime_camera_stream', anonymous=True)
        self.bridge = CvBridge()
        self.topic_name = topic_name
        self.current_image = None
        self.image_lock = threading.Lock()
        self.running = True
        
        print(f"�� 启动实时摄像头流，话题: {topic_name}")
        print("按 'q' 键退出，按 's' 键保存当前帧，按 'p' 键暂停/继续")
        
        self.image_sub = rospy.Subscriber(topic_name, Image, self.image_callback)
        
    def image_callback(self, msg):
        try:
            # 根据话题类型选择转换方式
            if "depth" in self.topic_name:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                # 将深度图转换为可视化格式
                cv_image = cv2.applyColorMap(cv2.convertScaleAbs(cv_image, alpha=0.03), cv2.COLORMAP_JET)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 线程安全地更新当前图像
            with self.image_lock:
                self.current_image = cv_image.copy()
                
        except Exception as e:
            print(f"❌ 图像转换出错: {e}")
    
    def get_current_image(self):
        """获取当前图像的副本"""
        with self.image_lock:
            if self.current_image is not None:
                return self.current_image.copy()
            return None
    
    def save_current_frame(self):
        """保存当前帧"""
        current_img = self.get_current_image()
        if current_img is not None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"/home/daniel/rmc_aida_l_ros1-develop/src/yolo/saved_frame_{timestamp}.png"
            cv2.imwrite(filename, current_img)
            print(f"✅ 图像已保存: {filename}")
        else:
            print("❌ 没有可保存的图像")
    
    def start_record(self):
        """获取实时图像并进行YOLO检测，显示带边界框的结果"""
        while self.running and not rospy.is_shutdown():
            current_img = self.get_current_image()
            if current_img is not None:
                try:
                    # 进行YOLO检测，返回带边界框的图像
                    result_img = segment_image(current_img)
                    
                    # 显示检测结果（带边界框）
                    cv2.imshow('YOLO Detection Stream', result_img)
                    
                    # 处理键盘输入
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        print("🛑 用户退出")
                        break
                    elif key == ord('s'):
                        self.save_current_frame()
                except Exception as e:
                    print(f"❌ YOLO检测出错: {e}")
                    # 显示原始图像
                    cv2.imshow('YOLO Detection Stream', current_img)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
            else:
                # 显示等待图像
                waiting_img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(waiting_img, "Waiting for camera image...", 
                           (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.imshow('YOLO Detection Stream', waiting_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
        
        self.running = False
        cv2.destroyAllWindows()

    def display_stream(self):
        """显示实时视频流（无YOLO检测）"""
        paused = False
        
        while self.running and not rospy.is_shutdown():
            current_img = self.get_current_image()
            
            if current_img is not None:
                # 在图像上添加信息
                display_img = current_img.copy()
                
                # 添加话题信息
                cv2.putText(display_img, f"Topic: {self.topic_name}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # 添加时间戳
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                cv2.putText(display_img, f"Time: {timestamp}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # 添加控制提示
                if paused:
                    cv2.putText(display_img, "PAUSED - Press 'p' to continue", 
                               (10, display_img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    cv2.putText(display_img, "Press 'q':quit, 's':save, 'p':pause", 
                               (10, display_img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # 显示图像
                cv2.imshow('Camera Stream', display_img)
            else:
                # 显示等待图像
                waiting_img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(waiting_img, "Waiting for camera image...", 
                           (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.imshow('Camera Stream', waiting_img)
            
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("🛑 用户退出")
                break
            elif key == ord('s'):
                self.save_current_frame()
            elif key == ord('p'):
                paused = not paused
                print(f"�� {'暂停' if paused else '继续'}播放")
            
            # 如果暂停，等待一段时间
            if paused:
                time.sleep(0.1)
        
        self.running = False
        cv2.destroyAllWindows()


def main():
    if len(sys.argv) > 1:
        topic_name = sys.argv[1]
    else:
        # 默认使用右臂相机
        topic_name = "/camera_d435_1/color/image_raw"
    
    try:
        # 创建实时摄像头流对象
        camera_stream = RealTimeCameraStream(topic_name)
        
        # 等待一下确保ROS连接建立
        rospy.sleep(1.0)
        
        # 选择运行模式
        print("选择运行模式:")
        print("1. 显示原始视频流（无YOLO检测）")
        print("2. 连续YOLO检测模式（显示边界框）")
        
        choice = input("请输入选择 (1 或 2): ").strip()
        
        if choice == "1":
            print("🎬 开始显示原始视频流...")
            camera_stream.display_stream()
        else:
            print("�� 开始连续YOLO检测（带边界框）...")
            camera_stream.start_record()
        
    except rospy.ROSInterruptException:
        print("�� ROS中断")
    except KeyboardInterrupt:
        print("�� 用户中断")
    except Exception as e:
        print(f"❌ 程序出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        print("👋 程序结束")

if __name__ == '__main__':
    import numpy as np
    main()