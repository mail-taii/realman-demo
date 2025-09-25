#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
任务执行主脚本 - 整合AGV、机械臂和夹爪控制
用于演示完整的机器人任务执行流程

功能：
1. 整合agv_client、arm_client和gripper_client
2. 提供完整的任务执行演示
3. 可通过execute_tasks.launch启动

使用方法：
    roslaunch task_planning execute_tasks.launch
    或
    rosrun task_planning execute_tasks.py
"""

import rospy
import sys
import os
import time


# 添加task_planning包路径到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
task_planning_dir = os.path.dirname(current_dir)
if task_planning_dir not in sys.path:
    sys.path.insert(0, task_planning_dir)

# 从apis包导入客户端类
from apis import AGVClient, ArmClient, GripperClient, LiftClient

agv_client = None
arm_client = None
gripper_client = None
lift_client = None

def initialize_clients():
    global agv_client, arm_client, gripper_client,lift_client
    startup_delay = rospy.get_param('~startup_delay', 5.0)
    rospy.sleep(startup_delay)
    try:
        agv_client = AGVClient()
    except:
        rospy.logerr("AGVClient初始化失败")
        pass
    try:
        arm_client = ArmClient()
    except:
        rospy.logerr("ArmClient初始化失败")
        pass
    try:
        gripper_client = GripperClient()
    except:
        rospy.logerr("GripperClient初始化失败")
        pass
    try:
        lift_client = LiftClient()
    except:
        rospy.logerr("LiftClient初始化失败")
        pass

def execute_task():
    global agv_client, arm_client, gripper_client, lift_client

    # # initial position
    # if agv_client.send_navigation_task("STORE", task_id=1): #这样是为了让agv移动完毕 不然手臂会出问题
    #     pass 

    home = [-9,31,-66,-53,-11,-41,18]
    arm_client.move_joint("left",home)
    arm_client.move_joint("right",[-home[i] for i in range(len(home))])
    lift_client.lift_height(470)
    gripper_client.set_gripper_position("left",0)
    gripper_client.set_gripper_position("right",0)

    # 桌子
    if agv_client.send_navigation_task("TABLE",task_id=1):
        pass

    arm_client.move_pose("right",[171,408,137] + arm_client.get_quaternion([-3.082,0.379,-1.389]))
    gripper_client.set_gripper_position("right", 12000)
    arm_client.move_joint("right",[-home[i] for i in range(len(home))])

    # 架子
    if agv_client.send_navigation_task("SHELF",task_id=1):
        pass
    
    arm_client.move_pose("right",[216,296,434] + arm_client.get_quaternion([-3.086,0.8,-1.333]))
    gripper_client.set_gripper_position("right", 0)
    arm_client.move_joint("left",home)
    arm_client.move_joint("right",[-home[i] for i in range(len(home))])
    agv_client.send_navigation_task("STORE", task_id=1)

    # lift_client.lift_height(470)
    # lift_client.lift_height(200)


def main():
    try:
        rospy.init_node("execute_tasks", anonymous=True)
        rospy.loginfo("🚀 启动任务执行节点")
        initialize_clients()
        rospy.loginfo("Finish initializing")
        execute_task()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")


if __name__ == '__main__':
    main()
