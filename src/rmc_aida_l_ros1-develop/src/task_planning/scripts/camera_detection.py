#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
测试机器人摄象头实时分割

使用方法：
    roslaunch task_planning camera_detection.launch
    或
    rosrun task_planning camera_detection.py
"""

import rospy
import sys
import os
import cv2


# 添加task_planning包路径到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
task_planning_dir = os.path.dirname(current_dir)
if task_planning_dir not in sys.path:
    sys.path.insert(0, task_planning_dir)

# 从apis包导入客户端类
from apis import CameraClient

camera_client = None

def initialize_clients():
    global camera_client
    startup_delay = rospy.get_param('~startup_delay', 5.0)
    rospy.sleep(startup_delay)
    try:
        camera_client = CameraClient()
    except Exception as e:
        rospy.logerr(f"CameraClient初始化失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)  # 初始化失败直接退出，避免后续NoneType错误

def segment_image_realtime(camera_ids, interval=0.1):
    """
    实时对指定摄像头的当前帧执行分割检测
    camera_ids: list, 如 ['0'], ['1','2']
    interval: 检测间隔秒数
    """
    while not rospy.is_shutdown():
        with camera_client.lock:
            for cid in camera_ids:
                color_img = camera_client.image_data[str(cid)]['color']
                if color_img is not None:
                    result_img = camera_client.segment_image(color_img)
                    cv2.imshow(f"Camera {cid} Segmentation", result_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # rospy.sleep(interval)
    cv2.destroyAllWindows()


def main():
    try:
        rospy.init_node("camera_detection", anonymous=True)
        rospy.loginfo("🚀 启动摄像头检测节点")
        initialize_clients()
        rospy.loginfo("Finish initializing")
        # segment_image_realtime(['1'])
        camera_client.save_images(['0','1','2'])
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")


if __name__ == '__main__':
    main()
