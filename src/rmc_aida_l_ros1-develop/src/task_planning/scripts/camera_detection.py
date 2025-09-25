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
import threading
import queue
import time
import subprocess


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

def segment_worker(camera_ids, result_queue, stop_event):
    while not stop_event.is_set():
        with camera_client.lock:
            for cid in camera_ids:
                color_img = camera_client.image_data[str(cid)]['color']
                if color_img is not None:
                    result_img = camera_client.segment_image(color_img)
                    result_queue.put((cid, result_img))
        time.sleep(0.5)  # 分割间隔

def segment_image_realtime_async(camera_ids):
    result_queue = queue.Queue()
    stop_event = threading.Event()
    worker = threading.Thread(target=segment_worker, args=(camera_ids, result_queue, stop_event))
    worker.start()

    last_result = {}
    try:
        while not rospy.is_shutdown():
            with camera_client.lock:
                for cid in camera_ids:
                    color_img = camera_client.image_data[str(cid)]['color']
                    if color_img is not None:
                        cv2.imshow(f"Camera {cid} Color", color_img)
            # 显示分割结果（如果有新结果）
            while not result_queue.empty():
                cid, result_img = result_queue.get()
                last_result[cid] = result_img
            for cid, result_img in last_result.items():
                cv2.imshow(f"Camera {cid} Segmentation", result_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.03)  # 主线程显示间隔
    finally:
        stop_event.set()
        worker.join()
        cv2.destroyAllWindows()


def is_camera_topic_active():
    topics = rospy.get_published_topics()
    for t in topics:
        if '/camera_d435_0/color/image_raw' in t[0]:
            return True
    return False

def start_camera_node_if_needed():
    if not is_camera_topic_active():
        proc = subprocess.Popen([
            "roslaunch",
            "embodied_camera",
            "d435_camera_pub.launch"
        ])
        time.sleep(5)  # 等待节点发布
        return proc
    return None

def main():
    try:
        rospy.init_node("camera_detection", anonymous=True)
        start_camera_node_if_needed()
        rospy.loginfo("🚀 启动摄像头检测节点")
        initialize_clients()
        rospy.loginfo("Finish initializing")
        segment_image_realtime_async(['1'])
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")


if __name__ == '__main__':
    main()
