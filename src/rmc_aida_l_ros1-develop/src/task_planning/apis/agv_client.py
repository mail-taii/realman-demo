#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
AGV控制API客户端 - 为task_planning提供AGV控制接口
该脚本提供AGV导航控制的API接口，可被task_planning脚本调用

主要功能：
1. AGV导航任务发送 - 发送NavigationTask消息到agv_control节点
2. 任务状态监听 - 实时监听AGV任务执行状态反馈
3. 连接管理 - 自动管理与agv_control节点的连接状态

API使用方法：
    # 导入模块
    from agv_test_client import AGVClient
    
    # 初始化客户端
    agv_client = AGVClient()
    
    # 发送导航任务
    agv_client.send_navigation_task("A1", task_id=1)
    
    # 或直接运行测试
    rosrun task_planning agv_test_client.py

消息格式：
    NavigationTask:
        - task_id: 任务唯一标识符
        - target_mark: 目标点位标记 (如: A1, B2, C3)
        - task_exect: 任务执行模式 (1=正常执行)
        - task_type: 任务类型 (1=导航任务)
        - direction: 到达目标点后的朝向角度 (单位: 度)
"""

import rospy
import sys
from std_msgs.msg import String
from woosh_msgs.msg import RobotStatus
from woosh_msgs.srv import ExecTask, ExecTaskRequest

class AGVClient:

    
    def __init__(self):
        # 任务状态订阅器
        self.status_subscriber = rospy.Subscriber(
            "/robot_status", RobotStatus, self.status_callback, queue_size=10
        )
        
        self.navigation_service = rospy.ServiceProxy("/exec_task", ExecTask)

        self.arrive_state = False


        # 等待发布器连接
        rospy.sleep(1.0)
        
        
    
    def status_callback(self, msg: RobotStatus):

        if msg.task_state == 7:
            self.arrive_state = True
            rospy.loginfo(f"✅ 任务 {msg.task_id} 已完成")

    
    def send_navigation_task(self, target_mark, task_id=1, task_exect=1, task_type=1, direction=0):
        """
        发送AGV导航任务
        
        向agv_control节点发送导航任务，控制AGV移动到指定点位。
        该方法是AGV控制的核心API接口。
        
        参数:
            target_mark (str): 目标点位标记，如 "A1", "B2", "C3" 等
                              必须是地图中已定义的有效点位标识
            task_id (int): 任务唯一标识符，用于跟踪任务状态
                          默认值: 1
            task_exect (int): 任务执行模式
                             1 = 正常执行模式 (默认)
                             其他值根据底盘系统定义
            task_type (int): 任务类型标识
                            1 = 导航任务 (默认)
                            其他值根据系统需求定义
            direction (int): 到达目标点后的朝向角度
                           单位: 度 (0-359)
                           0 = 北方向 (默认)
        
        返回:
            None: 该方法为异步调用，通过status_callback接收执行结果
        
        异常:
            如果发布器未连接，任务可能无法发送
        
        使用示例:
            # 导航到A1点位
            client.send_navigation_task("A1")
            
            # 导航到B2点位，并设置朝向为90度
            client.send_navigation_task("B2", task_id=2, direction=90)
        """
        # 创建导航任务消息
        req = ExecTaskRequest()
        req.task_id = task_id
        req.task_exect = task_exect
        req.task_type = task_type
        req.direction = direction
        req.mark_no = target_mark
        req.task_type_no = 0
        res = self.navigation_service(req)
        
        self.arrive_state = False

        if res.success:

            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                if self.arrive_state:
                    break
                rate.sleep()
            rospy.loginfo(f"Navigation execution was successful!")
            return True
        else:
            rospy.logerr(f"Navigation execution command error!")
            return False    

        


if __name__ == "__main__":
    # 实例化客户端，发送导航任务至TEST点位
    agv_client = AGVClient()
    agv_client.send_navigation_task("TEST", task_id=1)
    rospy.spin()

