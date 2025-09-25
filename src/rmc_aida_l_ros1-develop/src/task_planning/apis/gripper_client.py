#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
夹爪位置控制Action客户端 - 双服务器版本
用于task_planning包中调用gripper_control_handler的左右夹爪Action服务

功能：
1. 提供统一的夹爪位置控制API接口
2. 连接到gripper_control_handler的左右Action服务器
3. 支持真正的并发执行
4. 每个Action自动等待硬件完成（无需wait参数）
"""

import rospy
import actionlib
from gripper_control_handler.msg import GripperSetPositionLeftAction, GripperSetPositionLeftGoal, GripperSetPositionLeftFeedback
from gripper_control_handler.msg import GripperSetPositionRightAction, GripperSetPositionRightGoal, GripperSetPositionRightFeedback


class GripperClient:
    """夹爪位置控制Action客户端类 - 支持并发的双服务器架构"""
    
    def __init__(self):
        """初始化夹爪客户端"""
        # rospy.init_node("gripper_client", anonymous=True)
        
        # 创建两个独立的Action客户端
        self.left_gripper_client = actionlib.SimpleActionClient(
            'gripper_set_position_left', GripperSetPositionLeftAction
        )
        self.right_gripper_client = actionlib.SimpleActionClient(
            'gripper_set_position_right', GripperSetPositionRightAction
        )
        
        self.left_gripper_client.wait_for_server()
        self.right_gripper_client.wait_for_server()
        rospy.loginfo("🎉 所有夹爪Action服务器已连接")
    
    # ===== 核心API接口 =====
    
    def open_gripper(self, gripper_side, speed=500, force=50, timeout=0):
        """
        打开夹爪（设置到位置0）
        
        Args:
            gripper_side: "left" 或 "right"
            speed: 速度 (1-1000)
            force: 力度 (1-1000) 
            timeout: 超时时间 (0=使用默认值)
        
        Returns:
            bool: 成功返回True，失败返回False
        """
        return self.set_gripper_position(gripper_side, 0, speed, force, timeout)
    
    def close_gripper(self, gripper_side, speed=500, force=50, timeout=0):
        """
        关闭夹爪（设置到位置12000）
        
        Args:
            gripper_side: "left" 或 "right"
            speed: 速度 (1-1000)
            force: 力度 (1-1000)
            timeout: 超时时间 (0=使用默认值)
        
        Returns:
            bool: 成功返回True，失败返回False
        """
        return self.set_gripper_position(gripper_side, 12000, speed, force, timeout)
    
    def set_gripper_position(self, gripper_side, position, speed=500, force=50, timeout=0):
        """
        设置夹爪位置（自动等待硬件完成）
        
        Args:
            gripper_side: "left" 或 "right"
            position: 目标位置 (0-12000)
            speed: 速度 (1-1000)
            force: 力度 (1-1000)
            timeout: 超时时间 (0=使用默认值)
        
        Returns:
            bool: 成功返回True，失败返回False
        """
        try:
            # 选择正确的Action客户端和Goal类型
            if gripper_side == "left":
                client = self.left_gripper_client
                goal = GripperSetPositionLeftGoal()
            elif gripper_side == "right":
                client = self.right_gripper_client
                goal = GripperSetPositionRightGoal()
            else:
                raise Exception(f"不支持的夹爪: {gripper_side}")
            
            # 设置Goal参数
            goal.position = position
            goal.speed = speed
            goal.force = force
            goal.timeout = timeout
            
            rospy.loginfo(f"🤖 发送{gripper_side}夹爪位置控制命令: 位置{position}")
            
            # 发送并等待完成（Action服务器会等待硬件完成）
            client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(timeout + 10.0 if timeout > 0 else 45.0))
            result = client.get_result()
            
            if result and result.success:
                rospy.loginfo(f"✅ {gripper_side}夹爪位置设置成功: {result.message}")
                rospy.loginfo(f"   执行时间: {result.execution_time:.2f}秒")
                return True
            else:
                error_msg = result.message if result else "无响应"
                rospy.logerr(f"❌ {gripper_side}夹爪位置设置失败: {error_msg}")
                return False
                
        except Exception as e:
            rospy.logerr(f"{gripper_side}夹爪位置控制异常: {e}")
            return False
    
    def set_gripper_position_async(self, gripper_side, position, speed=500, force=50, timeout=0):
        """
        异步设置夹爪位置（立即返回，不等待完成）
        
        Args:
            gripper_side: "left" 或 "right"
            position: 目标位置 (0-12000)
            speed: 速度 (1-1000)
            force: 力度 (1-1000)
            timeout: 超时时间 (0=使用默认值)
        
        Returns:
            bool: 命令发送成功返回True，失败返回False
        """
        try:
            # 选择正确的Action客户端和Goal类型
            if gripper_side == "left":
                client = self.left_gripper_client
                goal = GripperSetPositionLeftGoal()
            elif gripper_side == "right":
                client = self.right_gripper_client
                goal = GripperSetPositionRightGoal()
            else:
                raise Exception(f"不支持的夹爪: {gripper_side}")
            
            # 设置Goal参数
            goal.position = position
            goal.speed = speed
            goal.force = force
            goal.timeout = timeout
            
            rospy.loginfo(f"🤖 异步发送{gripper_side}夹爪位置控制命令: 位置{position}")
            
            # 异步发送（立即返回）
            client.send_goal(goal, feedback_cb=lambda fb: self.feedback_callback(gripper_side, fb))
            rospy.loginfo(f"📤 已发送{gripper_side}夹爪位置设置命令（异步）")
            return True
                
        except Exception as e:
            rospy.logerr(f"{gripper_side}夹爪异步控制异常: {e}")
            return False
    
    def feedback_callback(self, gripper_side, feedback):
        """反馈回调函数"""
        rospy.loginfo(f"📊 {gripper_side}夹爪进度: {feedback.progress*100:.1f}% | "
                     f"状态: {feedback.status} | "
                     f"当前位置: {feedback.current_position} | "
                     f"用时: {feedback.elapsed_time:.1f}s")
        if feedback.debug_info:
            rospy.loginfo(f"   调试信息: {feedback.debug_info}")
    
    def cancel_all_goals(self):
        """取消所有正在执行的目标"""
        try:
            self.left_gripper_client.cancel_all_goals()
            self.right_gripper_client.cancel_all_goals()
            rospy.loginfo("🛑 已取消所有夹爪动作")
        except Exception as e:
            rospy.logerr(f"取消目标异常: {e}")


def main():
    """主函数"""
    try:
        # 创建夹爪客户端
        gripper_client = GripperClient()
        
        gripper_client.set_gripper_position("left", 6000)
        gripper_client.set_gripper_position("right", 6000)

        
        gripper_client.set_gripper_position("left", 0)
        gripper_client.set_gripper_position("right", 12000)
    except Exception as e:
        rospy.logerr(f"夹爪客户端异常: {e}")


if __name__ == '__main__':
    main()