#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
from datetime import datetime
from detect_all import segment_image

class SingleImageCapture:
    def __init__(self, topic_name):
        rospy.init_node('single_image_capture', anonymous=True)
        self.bridge = CvBridge()
        self.topic_name = topic_name
        self.image_received = False
        self.image = None
        
        print(f"等待来自话题 {topic_name} 的图像...")
        self.image_sub = rospy.Subscriber(topic_name, Image, self.image_callback)
        
    def image_callback(self, msg):
        if not self.image_received:
            try:
                # 根据话题类型选择转换方式
                if "depth" in self.topic_name:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                    image_type = "depth"
                else:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    image_type = "color"
                
                # 生成文件名
                filepath = f"origin.png"
                
                # 保存图片
                cv2.imwrite(filepath, cv_image)
                
                self.image = cv_image
                self.image_received = True
                rospy.signal_shutdown("图片已成功捕获")
                
            except Exception as e:
                print(f"❌ 保存图片时出错: {e}")

def main():
    if len(sys.argv) > 1:
        topic_name = sys.argv[1]
    else:
        # 默认使用camera_1的彩色图像
        # topic_name = "/camera_1/color/image_raw"
        topic_name = "/camera_d435_1/color/image_raw"
    
    try:
        capture = SingleImageCapture(topic_name)
        
        # 等待图像，最多等待10秒
        timeout = rospy.Duration(10.0)
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown() and not capture.image_received:
            if rospy.Time.now() - start_time > timeout:
                print(f"❌ 超时：未能从话题 {topic_name} 接收到图像")
                break
            rospy.sleep(0.1)

        segment_image(capture.image)
            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
