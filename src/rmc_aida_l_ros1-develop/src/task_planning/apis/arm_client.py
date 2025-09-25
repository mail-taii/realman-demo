#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
ARM控制Action客户端 - 为task_planning提供ARM控制接口

"""

import rospy
import math
from geometry_msgs.msg import Pose, Point, Quaternion
from embodied_arm_msgs.msg import MoveJ, MoveJ_P, Plan_State
from tf.transformations import quaternion_from_euler

class ArmClient:
    """ARM控制Action客户端类"""
    
    def __init__(self):
        """初始化Action客户端"""
        # rospy.init_node("arm_client", anonymous=True)
        
        # 创建发布器
        self.pub_MoveJ_l = rospy.Publisher('/l_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
        self.pub_MoveJ_P_l = rospy.Publisher('/l_arm/rm_driver/MoveJ_P_Cmd', MoveJ_P, queue_size=10)
        self.pub_MoveJ_r = rospy.Publisher('/r_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
        self.pub_MoveJ_P_r = rospy.Publisher('/r_arm/rm_driver/MoveJ_P_Cmd', MoveJ_P, queue_size=10)


        # 左臂/右臂执行状态订阅
        self.planState_sub_l = rospy.Subscriber("/l_arm/rm_driver/Plan_State", Plan_State, self.plan_state_callback)

        self.planState_sub_r = rospy.Subscriber("/r_arm/rm_driver/Plan_State", Plan_State, self.plan_state_callback)

        #双臂movej、movej_p轨迹规划情况和升降机升降执行情况
        self.run_state = False
        rospy.sleep(1)

    
    def degrees_to_radians(self, degrees):
        """度转弧度"""
        return degrees * (math.pi / 180)
    
    def plan_state_callback(self, msg):
        if msg.state:
            self.run_state = True
            rospy.loginfo("*******Plan State OK")
        else:
            self.run_state = False
            rospy.loginfo("*******Plan State Fail")
    
    def is_arrive(self):
        """机械臂movej和movej_p动作执行成功检测"""
        while 1:
            rospy.sleep(0.1)
            if self.run_state:
                self.run_state = False
                break

    def move_joint(self, arm_side, joint_angles_deg, speed=0.2, timeout=15.0):
        """关节运动"""
        movej_cmd = MoveJ()
        movej_cmd.speed = speed
        movej_cmd.joint = [self.degrees_to_radians(angle) for angle in joint_angles_deg]
        if arm_side == "left":
            self.pub_MoveJ_l.publish(movej_cmd)
        elif arm_side == "right":
            self.pub_MoveJ_r.publish(movej_cmd)
        else:
            rospy.logerr(f"无效的手臂选择: {arm_side}")
            return False
        self.is_arrive()
        return True
    
    def move_pose(self, arm_side, target_pose, speed=0.1, timeout=15.0):
        """空间运动 (使用 MoveJ_P 消息)"""
        msg = MoveJ_P()
        msg.Pose.position.x = target_pose[0]
        msg.Pose.position.y = target_pose[1]
        msg.Pose.position.z = target_pose[2]
        msg.Pose.orientation.x = target_pose[3]
        msg.Pose.orientation.y = target_pose[4]
        msg.Pose.orientation.z = target_pose[5]
        msg.Pose.orientation.w = target_pose[6]
        msg.speed = 0.1
        if arm_side == "left":
            self.pub_MoveJ_P_l.publish(msg)
        elif arm_side == "right":
            self.pub_MoveJ_P_r.publish(msg)
        else:
            rospy.logerr(f"无效的手臂选择: {arm_side}")
            return False
        self.is_arrive()
        return True

    def home_position(self, arm_side):
        """回零点"""
        if arm_side == "left":
            return self.move_joint(arm_side, [-12.177, -106.699, 4.056, -3.49, 8.441, -31.897, 58.77])
        elif arm_side == "right":
            return self.move_joint(arm_side, [1.551, 87.529, 6.347, 71.049, -2.261, -14.856, -0.056])
        else:
            rospy.logerr(f"无效的手臂选择: {arm_side}")
            return False
            
    def get_quaternion(self,angles):
        return list(quaternion_from_euler(angles[0],angles[1],angles[2]))

if __name__ == "__main__":
    # 简单测试
    arm_client = ArmClient()
    
    arm_client.home_position("left")
    arm_client.home_position("right")

    # 测试关节运动
    # success = arm_client.move_joint("left", [-12.177, -106.699, 4.056, -3.49, 8.441, -31.897, 0])
    # success = arm_client.move_pose("left", [-355.865,94.830,-399.345,3.091,0.093,2.791,0])

    # if success:
    #     rospy.loginfo("关节运动测试完成")
    

    rospy.spin()