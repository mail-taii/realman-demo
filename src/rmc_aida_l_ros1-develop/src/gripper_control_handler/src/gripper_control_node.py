#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
夹爪位置控制Action服务端 - 双服务器版本
提供两个独立的Action服务器，分别控制左右夹爪，支持真正的并发执行

功能：
1. GripperSetPositionLeft Action服务器 - 专门处理左夹爪
2. GripperSetPositionRight Action服务器 - 专门处理右夹爪
3. 实时进度反馈和状态监控
4. 任务可取消性和错误处理
5. 支持左右夹爪真正并发控制

基于原有的夹爪通信类进行Action封装
"""

import sys
import os
import socket
import json
import time
import yaml
import threading
import rospy
import actionlib
from gripper_control_handler.msg import GripperSetPositionLeftAction, GripperSetPositionLeftGoal, GripperSetPositionLeftResult, GripperSetPositionLeftFeedback
from gripper_control_handler.msg import GripperSetPositionRightAction, GripperSetPositionRightGoal, GripperSetPositionRightResult, GripperSetPositionRightFeedback


class GripperCommunication:
    """夹爪通信类 - 处理与夹爪的JSON通信"""
    
    def __init__(self, ip, port=502, timeout=10.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        

    def send_command(self, command, wait_response=False):
        """发送JSON命令到夹爪"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            sock.connect((self.ip, self.port))
            sock.send((json.dumps(command) + "\r\n").encode('utf-8'))
            
            if wait_response:
                response = sock.recv(1024).decode('utf-8').strip()
                sock.close()
                return json.loads(response) if response else None
            
            sock.close()
            return True
            
        except Exception as e:
            rospy.logerr(f"通信错误 {self.ip}: {e}")
            return False
    
    def set_tool_voltage(self, voltage_type=3):
        return self.send_command({"command": "set_tool_voltage", "voltage_type": voltage_type}, True)
    
    def set_modbus_mode(self, port=1, baudrate=115200, timeout=2):
        return self.send_command({"command": "set_modbus_mode", "port": port, "baudrate": baudrate, "timeout": timeout}, True)
    
    def write_single_register(self, port, address, data, device=1):
        return self.send_command({"command": "write_single_register", "port": port, "address": address, "data": data, "device": device}, True)
    
    def read_holding_registers(self, port, address, device=1):
        return self.send_command({"command": "read_holding_registers", "port": port, "address": address, "device": device}, True)


class GripperControlNode:
    """夹爪位置控制Action服务端类 - 双服务器架构"""
    
    def __init__(self):
        """初始化Action服务端"""
        # 初始化节点
        rospy.init_node("gripper_control_node", anonymous=True)
        
        # 从launch文件读取参数
        self.left_gripper_ip = rospy.get_param('gripper_control/left_gripper_ip', '169.254.128.18')
        self.right_gripper_ip = rospy.get_param('gripper_control/right_gripper_ip', '169.254.128.19')
        self.port = rospy.get_param('gripper_control/gripper_port', 8080)
        self.communication_timeout = rospy.get_param('gripper_control/communication_timeout', 10.0)
        self.default_timeout = rospy.get_param('~default_timeout', 30.0)  # 节点私有参数
        
        rospy.loginfo(f"夹爪配置参数:")
        rospy.loginfo(f"  左夹爪IP: {self.left_gripper_ip}")
        rospy.loginfo(f"  右夹爪IP: {self.right_gripper_ip}")
        rospy.loginfo(f"  通信端口: {self.port}")
        rospy.loginfo(f"  通信超时: {self.communication_timeout}s")
        rospy.loginfo(f"  默认任务超时: {self.default_timeout}s")
        
        # 创建通信对象，使用从参数服务器读取的配置
        self.left_gripper_comm = GripperCommunication(self.left_gripper_ip, self.port, self.communication_timeout)
        self.right_gripper_comm = GripperCommunication(self.right_gripper_ip, self.port, self.communication_timeout)
        
        # 夹爪状态管理
        self.left_gripper_state = {
            'is_busy': False,
            'current_position': 0,
            'is_connected': False,
            'has_error': False,
            'error_message': '',
            'lock': threading.Lock()
        }
        self.right_gripper_state = {
            'is_busy': False,
            'current_position': 0,
            'is_connected': False,
            'has_error': False,
            'error_message': '',
            'lock': threading.Lock()
        }
        
        # 创建两个独立的Action服务器
        self.left_gripper_server = actionlib.SimpleActionServer(
            'gripper_set_position_left',
            GripperSetPositionLeftAction,
            self.execute_left_gripper_callback,
            False
        )
        
        self.right_gripper_server = actionlib.SimpleActionServer(
            'gripper_set_position_right', 
            GripperSetPositionRightAction,
            self.execute_right_gripper_callback,
            False
        )
        
        # 初始化夹爪
        self.initialize()
        
        # 启动两个Action服务器
        self.left_gripper_server.start()
        self.right_gripper_server.start()
        
        rospy.loginfo("夹爪位置控制Action服务器已启动")
        rospy.loginfo("- /gripper_set_position_left (GripperSetPositionLeftAction)")
        rospy.loginfo("- /gripper_set_position_right (GripperSetPositionRightAction)")
    
    def initialize(self):
        """初始化夹爪系统"""
        try:
            # 初始化左夹爪
            left_voltage_result = self.left_gripper_comm.set_tool_voltage(3)
            if not left_voltage_result or not left_voltage_result.get("state"):
                self.left_gripper_state['has_error'] = True
                self.left_gripper_state['error_message'] = "左夹爪电压设置失败"
                rospy.logwarn("左夹爪初始化失败")
            else:
                self.left_gripper_state['is_connected'] = True
            
            # 初始化右夹爪
            right_voltage_result = self.right_gripper_comm.set_tool_voltage(3)
            if not right_voltage_result or not right_voltage_result.get("state"):
                self.right_gripper_state['has_error'] = True
                self.right_gripper_state['error_message'] = "右夹爪电压设置失败"
                rospy.logwarn("右夹爪初始化失败")
            else:
                self.right_gripper_state['is_connected'] = True
            
            time.sleep(0.5)
            
            # 设置Modbus模式
            if self.left_gripper_state['is_connected']:
                left_modbus_result = self.left_gripper_comm.set_modbus_mode(1, 115200, 2)
                if not left_modbus_result or not left_modbus_result.get("set_state"):
                    self.left_gripper_state['has_error'] = True
                    self.left_gripper_state['error_message'] = "左夹爪Modbus设置失败"
            
            if self.right_gripper_state['is_connected']:
                right_modbus_result = self.right_gripper_comm.set_modbus_mode(1, 115200, 2)
                if not right_modbus_result or not right_modbus_result.get("set_state"):
                    self.right_gripper_state['has_error'] = True
                    self.right_gripper_state['error_message'] = "右夹爪Modbus设置失败"
            
            # 启用夹爪
            if self.left_gripper_state['is_connected'] and not self.left_gripper_state['has_error']:
                left_enable_result = self.left_gripper_comm.write_single_register(1, 256, 1, 1)
                if not left_enable_result or not left_enable_result.get("write_state"):
                    self.left_gripper_state['has_error'] = True
                    self.left_gripper_state['error_message'] = "左夹爪启用失败"
            
            if self.right_gripper_state['is_connected'] and not self.right_gripper_state['has_error']:
                right_enable_result = self.right_gripper_comm.write_single_register(1, 256, 1, 1)
                if not right_enable_result or not right_enable_result.get("write_state"):
                    self.right_gripper_state['has_error'] = True
                    self.right_gripper_state['error_message'] = "右夹爪启用失败"
            
            # 检查初始化结果
            left_ok = self.left_gripper_state['is_connected'] and not self.left_gripper_state['has_error']
            right_ok = self.right_gripper_state['is_connected'] and not self.right_gripper_state['has_error']
            
            if left_ok:
                rospy.loginfo("左夹爪初始化成功")
            if right_ok:
                rospy.loginfo("右夹爪初始化成功")
            
            if not left_ok and not right_ok:
                rospy.logerr("所有夹爪初始化失败")
                return False
            
            rospy.loginfo("夹爪系统初始化完成")
            return True
            
        except Exception as e:
            rospy.logerr(f"夹爪初始化异常: {e}")
            return False
    
    def execute_left_gripper_callback(self, goal):
        """专门处理左夹爪的回调"""
        return self._execute_gripper_callback("left", goal, self.left_gripper_server, 
                                             GripperSetPositionLeftFeedback(), 
                                             GripperSetPositionLeftResult())
    
    def execute_right_gripper_callback(self, goal):
        """专门处理右夹爪的回调"""
        return self._execute_gripper_callback("right", goal, self.right_gripper_server,
                                             GripperSetPositionRightFeedback(),
                                             GripperSetPositionRightResult())
    
    def _execute_gripper_callback(self, gripper_side, goal, server, feedback, result):
        """通用的夹爪执行逻辑"""
        start_time = time.time()
        
        try:
            rospy.loginfo(f"收到{gripper_side}夹爪位置控制请求: 位置{goal.position}")
            
            # 发送初始化反馈
            feedback.status = "initializing"
            feedback.progress = 0.0
            feedback.elapsed_time = 0.0
            feedback.current_position = 0
            feedback.debug_info = f"开始设置{gripper_side}夹爪位置到{goal.position}"
            server.publish_feedback(feedback)
            
            # 检查超时，如果目标中没有设置超时，使用默认值
            if goal.timeout > 0:
                timeout_time = start_time + goal.timeout
            else:
                # 使用默认超时时间
                timeout_time = start_time + self.default_timeout
                rospy.loginfo(f"使用默认超时时间: {self.default_timeout}s")
            
            # 执行夹爪控制
            success = self._execute_single_gripper_action(gripper_side, goal, feedback, timeout_time, server)
            
            if not success:
                raise Exception(f"{gripper_side}夹爪位置设置失败")
            
            # 成功完成
            execution_time = time.time() - start_time
            result.success = True
            result.message = f"{gripper_side}夹爪位置设置成功"
            result.execution_time = execution_time
            result.final_position = goal.position
            
            rospy.loginfo(f"{gripper_side}夹爪位置控制任务成功: {result.message}")
            server.set_succeeded(result)
            
        except Exception as e:
            execution_time = time.time() - start_time
            error_msg = f"{gripper_side}夹爪位置控制失败: {str(e)}"
            
            result.success = False
            result.message = error_msg
            result.execution_time = execution_time
            result.final_position = 0
            
            rospy.logerr(error_msg)
            server.set_aborted(result)
    
    def _execute_single_gripper_action(self, gripper_name, goal, feedback, timeout_time, server):
        """执行单个夹爪的位置设置操作"""
        try:
            comm = self.left_gripper_comm if gripper_name == "left" else self.right_gripper_comm
            state = self.left_gripper_state if gripper_name == "left" else self.right_gripper_state
            
            with state['lock']:
                if state['is_busy']:
                    raise Exception(f"{gripper_name}夹爪正在忙碌")
                if not state['is_connected']:
                    raise Exception(f"{gripper_name}夹爪未连接")
                if state['has_error']:
                    raise Exception(f"{gripper_name}夹爪有错误: {state['error_message']}")
                
                state['is_busy'] = True
            
            try:
                # 更新反馈
                feedback.status = "executing"
                feedback.progress = 0.3
                feedback.current_position = goal.position
                feedback.debug_info = f"正在设置{gripper_name}夹爪位置到{goal.position}"
                server.publish_feedback(feedback)
                
                # 执行位置设置并等待硬件完成
                success = self._set_position_and_wait(comm, goal.position, feedback, server, timeout_time)
                
                if success:
                    # 夹爪已实际完成动作
                    feedback.status = "completed"
                    feedback.progress = 1.0
                    feedback.current_position = goal.position
                    feedback.debug_info = f"{gripper_name}夹爪已到达位置{goal.position}"
                    server.publish_feedback(feedback)
                    
                    # 更新状态中的当前位置
                    with state['lock']:
                        state['current_position'] = goal.position
                    
                    return True
                else:
                    raise Exception(f"{gripper_name}夹爪位置设置失败")
                    
            finally:
                # 重要：只有在硬件真正完成后才释放锁
                with state['lock']:
                    state['is_busy'] = False
                    
        except Exception as e:
            rospy.logerr(f"{gripper_name}夹爪操作异常: {e}")
            return False
    
    def _set_position_and_wait(self, comm, position, feedback, server, timeout_time):
        """设置夹爪位置并等待硬件完成"""
        try:
            # 1. 写入寄存器（启动硬件动作）
            if not comm.write_single_register(1, 258, 0, 1).get("write_state"):
                return False
            if not comm.write_single_register(1, 259, position, 1).get("write_state"):
                return False
            if not comm.write_single_register(1, 264, 1, 1).get("write_state"):
                return False
            
            rospy.loginfo(f"已发送位置命令，目标位置: {position}")
            
            # 2. 等待硬件实际完成动作
            return self._wait_for_hardware_completion(comm, position, feedback, server, timeout_time)
            
        except Exception as e:
            rospy.logerr(f"设置位置异常: {e}")
            return False
    
    def _wait_for_hardware_completion(self, comm, target_position, feedback, server, timeout_time):
        """等待夹爪硬件实际完成动作"""
        start_time = time.time()
        rate = rospy.Rate(10)  # 10Hz轮询
        
        while not rospy.is_shutdown() and time.time() < timeout_time:
            try:
                # 读取夹爪状态寄存器
                result = comm.read_holding_registers(1, 1540, 1)
                if result and result.get("data"):
                    data = result["data"]
                    # 状态值为1表示已到达目标位置
                    if (isinstance(data, list) and len(data) > 0 and data[0] == 1) or (isinstance(data, int) and data == 1):
                        rospy.loginfo(f"夹爪硬件已到达目标位置: {target_position}")
                        return True
                
                # 检查Action是否被取消
                if server.is_preempt_requested():
                    rospy.logwarn("夹爪动作被客户端取消")
                    server.set_preempted()
                    return False
                
                # 更新进度反馈
                elapsed = time.time() - start_time
                total_timeout = timeout_time - start_time
                progress = 0.3 + 0.6 * min(elapsed / total_timeout, 1.0)  # 30%-90%
                
                feedback.progress = progress
                feedback.elapsed_time = elapsed
                feedback.debug_info = f"等待夹爪完成动作... ({elapsed:.1f}s)"
                server.publish_feedback(feedback)
                
                rate.sleep()  # ROS管理的sleep，可以被中断
                
            except Exception as e:
                rospy.logwarn(f"读取夹爪状态异常: {e}")
                rate.sleep()
        
        rospy.logwarn(f"夹爪硬件动作超时，目标位置: {target_position}")
        return False


def main():
    """主函数"""
    try:
        # 创建夹爪控制节点
        gripper_node = GripperControlNode()
        
        rospy.loginfo("夹爪位置控制Action服务器已启动，等待请求...")
        rospy.loginfo("可用的Action服务:")
        rospy.loginfo("  - /gripper_set_position_left (GripperSetPositionLeftAction)")
        rospy.loginfo("  - /gripper_set_position_right (GripperSetPositionRightAction)")
        
        # 运行节点
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logerr(f"节点运行异常: {e}")
    finally:
        rospy.loginfo("夹爪控制节点关闭")


if __name__ == '__main__':
    main()