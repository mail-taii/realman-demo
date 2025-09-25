#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
具身双臂机器人夹爪控制演示脚本
支持知行CTAG2F120夹爪的基本控制功能
"""

import sys
import os
import socket
import json
import time
import yaml
import rospy
from embodied_arm_msgs.msg import Gripper_Pick, Gripper_Set

class GripperCommunication:
    """夹爪通信类 - 处理与夹爪的JSON通信"""
    
    def __init__(self, ip, port=8080, timeout=10.0):
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

class GripperControl:
    """夹爪控制类 - 高层控制逻辑"""
    
    def __init__(self, gripper_id=0, ip="169.254.128.18", port=8080):
        self.gripper_id = gripper_id
        self.ip = ip
        self.port = port
        self.is_initialized = False
        
        # 创建通信对象
        self.comm = GripperCommunication(ip, port)
        
        # ROS节点初始化（只在第一个夹爪时初始化）
        if gripper_id == 0:
            rospy.init_node('gripper_demo', anonymous=True)
            self.gripper_pick_sub = rospy.Subscriber('/gripper_pick', Gripper_Pick, self.gripper_pick_callback)
            self.gripper_set_sub = rospy.Subscriber('/gripper_set', Gripper_Set, self.gripper_set_callback)
        
        rospy.loginfo(f"夹爪{gripper_id} ({ip}) 已初始化")
    
    def initialize(self):
        """初始化夹爪系统"""
        if not self.comm.set_tool_voltage(3).get("state"):
            return False
        
        time.sleep(0.5)
        
        if not self.comm.set_modbus_mode(1, 115200, 2).get("set_state"):
            return False
        
        if not self.comm.write_single_register(1, 256, 1, 1).get("write_state"):
            return False
        
        self.is_initialized = True
        rospy.loginfo(f"夹爪{self.gripper_id} 初始化完成")
        return True
    
    def set_position(self, position, torque=50):
        """设置夹爪位置"""
        if not self.is_initialized and not self.initialize():
            return False
        
        if not self.comm.write_single_register(1, 258, 0, 1).get("write_state"):
            return False
        if not self.comm.write_single_register(1, 259, position, 1).get("write_state"):
            return False
        if not self.comm.write_single_register(1, 264, 1, 1).get("write_state"):
            return False
        
        return True
    
    def open(self, torque=50):
        """打开夹爪"""
        return self.set_position(0, torque)
    
    def close(self, torque=50):
        """闭合夹爪"""
        return self.set_position(12000, torque)
    
    def wait_for_position(self, timeout=3.0):
        """等待夹爪到达目标位置"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            result = self.comm.read_holding_registers(1, 1540, 1)
            if result and result.get("data"):
                data = result["data"]
                if (isinstance(data, list) and len(data) > 0 and data[0] == 1) or (isinstance(data, int) and data == 1):
                    return True
            time.sleep(0.05)
        return False
    
    def gripper_pick_callback(self, msg):
        """夹爪抓取回调函数"""
        if msg.gripper_action == "open":
            self.open(msg.torque if hasattr(msg, 'torque') else 50)
        elif msg.gripper_action == "close":
            self.close(msg.torque if hasattr(msg, 'torque') else 50)
    
    def gripper_set_callback(self, msg):
        """夹爪设置回调函数"""
        if hasattr(msg, 'position'):
            self.set_position(msg.position, msg.torque if hasattr(msg, 'torque') else 50)

def load_gripper_config():
    """加载夹爪配置文件"""
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'gripper_params.yaml')
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        return config
    except Exception as e:
        rospy.logerr(f"无法加载配置文件: {e}")
        return {
            'grippers': [
                {'id': 0, 'ip': '169.254.128.18', 'port': 8080, 'name': '左夹爪'},
                {'id': 1, 'ip': '169.254.128.19', 'port': 8080, 'name': '右夹爪'}
            ],
            'gripper': {
                'open_position': 0,
                'close_position': 12000,
                'default_torque': 50,
                'position_timeout': 3.0
            }
        }

def demo_sequence():
    """演示序列 - 顺序打开和关闭两个夹爪"""
    config = load_gripper_config()
    
    # 创建两个独立的夹爪控制器
    grippers = []
    for gripper_config in config['grippers']:
        gripper = GripperControl(
            gripper_config['id'], 
            gripper_config['ip'], 
            gripper_config['port']
        )
        grippers.append(gripper)
    
    try:
        # 初始化所有夹爪
        for i, gripper in enumerate(grippers):
            if not gripper.initialize():
                rospy.logerr(f"夹爪{i} 初始化失败")
                return
            rospy.loginfo(f"夹爪{i} 初始化成功")
        
        rospy.loginfo("开始演示序列")
        
        # 同时打开所有夹爪
        for gripper in grippers:
            gripper.open(config['gripper']['default_torque'])
        for gripper in grippers:
            gripper.wait_for_position(config['gripper']['position_timeout'])
        rospy.sleep(1)
        
        # 同时闭合所有夹爪
        for gripper in grippers:
            gripper.close(config['gripper']['default_torque'])
        for gripper in grippers:
            gripper.wait_for_position(config['gripper']['position_timeout'])
        rospy.sleep(1)
        
        # 再次同时打开
        for gripper in grippers:
            gripper.open(config['gripper']['default_torque'])
        for gripper in grippers:
            gripper.wait_for_position(config['gripper']['position_timeout'])
        
        rospy.loginfo("演示序列完成")
        
    except Exception as e:
        rospy.logerr(f"演示过程中发生错误: {e}")

def main():
    """主函数"""
    print("启动夹爪演示...")
    demo_sequence()

if __name__ == "__main__":
    main()
