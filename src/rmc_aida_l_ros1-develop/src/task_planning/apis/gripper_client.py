#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
夹爪控制客户端 - 直接发布到arm_control主题版本
用于task_planning包中直接控制机械臂的夹爪功能

功能：
1. 提供统一的夹爪控制API接口
2. 直接发布到arm_control的Modbus主题
3. 支持左右夹爪的独立控制
4. 监听执行结果并处理反馈
"""

import rospy
import threading
import time
from std_msgs.msg import Byte, Bool
from embodied_arm_msgs.msg import Set_Modbus_Mode, Write_Register, Read_Register, Register_Data


class GripperClient:
    """夹爪控制客户端类 - 直接发布到arm_control主题"""
    
    def __init__(self):
        """初始化夹爪客户端"""
        # 初始化ROS节点（如果尚未初始化）
        if not rospy.get_node_uri():
            rospy.init_node("gripper_client", anonymous=True)
        
        # 创建发布者 - 按机械臂区分主题 (/l_arm/rm... 和 /r_arm/rm...)
        self.left_voltage_pub = rospy.Publisher('/l_arm/rm_driver/SetToolVoltage', Byte, queue_size=10)
        self.right_voltage_pub = rospy.Publisher('/r_arm/rm_driver/SetToolVoltage', Byte, queue_size=10)
        
        self.left_modbus_mode_pub = rospy.Publisher('/l_arm/rm_driver/Set_Modbus_Mode', Set_Modbus_Mode, queue_size=10)
        self.right_modbus_mode_pub = rospy.Publisher('/r_arm/rm_driver/Set_Modbus_Mode', Set_Modbus_Mode, queue_size=10)
        
        self.left_write_register_pub = rospy.Publisher('/l_arm/rm_driver/Write_Single_Register', Write_Register, queue_size=10)
        self.right_write_register_pub = rospy.Publisher('/r_arm/rm_driver/Write_Single_Register', Write_Register, queue_size=10)
        
        self.left_read_register_pub = rospy.Publisher('/l_arm/rm_driver/Read_Holding_Registers', Read_Register, queue_size=10)
        self.right_read_register_pub = rospy.Publisher('/r_arm/rm_driver/Read_Holding_Registers', Read_Register, queue_size=10)
        
        # 创建结果订阅者
        self.left_voltage_result_sub = rospy.Subscriber('/l_arm/rm_driver/SetToolVoltage_Result', Bool, self.voltage_result_callback)
        self.right_voltage_result_sub = rospy.Subscriber('/r_arm/rm_driver/SetToolVoltage_Result', Bool, self.voltage_result_callback)
        
        self.left_modbus_result_sub = rospy.Subscriber('/l_arm/rm_driver/Set_Modbus_Mode_Result', Bool, self.modbus_result_callback)
        self.right_modbus_result_sub = rospy.Subscriber('/r_arm/rm_driver/Set_Modbus_Mode_Result', Bool, self.modbus_result_callback)
        
        self.left_write_result_sub = rospy.Subscriber('/l_arm/rm_driver/Write_Single_Register_Result', Bool, self.write_result_callback)
        self.right_write_result_sub = rospy.Subscriber('/r_arm/rm_driver/Write_Single_Register_Result', Bool, self.write_result_callback)
        
        self.left_read_result_sub = rospy.Subscriber('/l_arm/rm_driver/Read_Holding_Registers_Result', Register_Data, self.read_result_callback)
        self.right_read_result_sub = rospy.Subscriber('/r_arm/rm_driver/Read_Holding_Registers_Result', Register_Data, self.read_result_callback)
        
        # 结果存储
        self.results = {}
        self.result_lock = threading.Lock()
        
        # 等待发布者连接
        rospy.sleep(0.5)  # 给发布者时间建立连接
        rospy.loginfo("🎉 夹爪客户端已初始化，连接到arm_control主题")

        self.results['read'] = None
        self.results['write'] = None
        self.results['modbus'] = None
        self.results['voltage'] = None

        self.set_tool_voltage("left", 3)
        self.set_tool_voltage("right", 3)
        self.set_modbus_mode("left", 1, 115200, 2)
        self.set_modbus_mode("right", 1, 115200, 2)
        # 正确顺序：address, data, port, device
        self.write_register("left", 256, 50, 1, 1)
        self.write_register("right", 256, 50, 1, 1)


        rospy.sleep(1)
        rospy.loginfo("初始化完成")
    # ===== 结果回调函数 =====
    
    def voltage_result_callback(self, msg):
        """工具电压设置结果回调"""
        with self.result_lock:
            self.results['voltage'] = msg.data
            rospy.loginfo(f"🔌 工具电压设置结果: {'成功' if msg.data else '失败'}")
    
    def modbus_result_callback(self, msg):
        """Modbus模式设置结果回调"""
        with self.result_lock:
            self.results['modbus'] = msg.data
            rospy.loginfo(f"📡 Modbus模式设置结果: {'成功' if msg.data else '失败'}")
    
    def write_result_callback(self, msg):
        """寄存器写入结果回调"""
        with self.result_lock:
            self.results['write'] = msg.data
            rospy.loginfo(f"✍️ 寄存器写入结果: {'成功' if msg.data else '失败'}")
    
    def read_result_callback(self, msg):
        """寄存器读取结果回调"""
        with self.result_lock:
            self.results['read'] = msg
            # rospy.loginfo(f"📖 寄存器读取结果: {'成功' if msg.state else '失败'}, 数据: {msg.data}")
    
    # ===== 核心API接口 =====
    
    def set_tool_voltage(self, gripper_side, voltage=3):
        
        if gripper_side == "left":
            self.left_voltage_pub.publish(voltage)
        elif gripper_side == "right":
            self.right_voltage_pub.publish(voltage)
        else:
            raise Exception(f"不支持的夹爪: {gripper_side}")
        
        return True
    

    def set_modbus_mode(self, gripper_side, port=1, baudrate=115200, timeout=2):
        msg = Set_Modbus_Mode()
        msg.port = port
        msg.baudrate = baudrate
        msg.timeout = timeout
        if gripper_side == "left":
            self.left_modbus_mode_pub.publish(msg)
        elif gripper_side == "right":
            self.right_modbus_mode_pub.publish(msg)
        else:
            raise Exception(f"不支持的夹爪: {gripper_side}")
        return True
    
    def write_register(self, gripper_side, address, data, port=1, device=1, timeout=10.0):
        msg = Write_Register()
        msg.port = port
        msg.address = address
        msg.num = 1
        msg.data = [int(data)]
        msg.device = device
        # rospy.loginfo(f"✍️ 发布写寄存器: side={gripper_side}, port={msg.port}, addr={msg.address}, num={msg.num}, data={msg.data}, dev={msg.device}")
        # 清空上一次写结果，防止读到陈旧状态
        with self.result_lock:
            self.results['write'] = None
        if gripper_side == "left":
            self.left_write_register_pub.publish(msg)
        elif gripper_side == "right":
            self.right_write_register_pub.publish(msg)
        else:
            raise Exception(f"不支持的夹爪: {gripper_side}")
        # 等待回执并返回真实布尔值
        start = time.time()
        while time.time() - start < timeout:
            with self.result_lock:
                w = self.results.get('write')
            if w is not None:
                return bool(w)
            time.sleep(0.02)
        rospy.logwarn("写寄存器超时")
        return False

    def read_register(self, gripper_side, address, port=1, device=1, timeout=3.0):
        msg = Read_Register()
        msg.port = port
        msg.address = address
        msg.num = 1
        msg.device = device
        # rospy.loginfo(f"📖 发布读寄存器: side={gripper_side}, port={msg.port}, addr={msg.address}, num={msg.num}, dev={msg.device}")
        # 清空上一次读结果
        with self.result_lock:
            self.results['read'] = None
        if gripper_side == "left":
            self.left_read_register_pub.publish(msg)
        elif gripper_side == "right":
            self.right_read_register_pub.publish(msg)
        else:
            raise Exception(f"不支持的夹爪: {gripper_side}")
        # 等待新鲜回执即可（是否成功由调用方解析 Register_Data）
        start = time.time()
        while time.time() - start < timeout:
            with self.result_lock:
                r = self.results.get('read')
            if r is not None:
                return True
            time.sleep(0.02)
        rospy.logwarn("读寄存器超时")
        return False


    def set_gripper_position(self, gripper_side, position, speed=500, force=50, timeout=15.0):
        rospy.loginfo(f"设置{gripper_side}夹爪位置: {position}")
        if not self.write_register(gripper_side, 258, 0, 1):
            return False
        if not self.write_register(gripper_side, 259, position, 1):
            return False
        if not self.write_register(gripper_side, 0x0104, speed, 1):
            return False
        if not self.write_register(gripper_side, 0x0105, force, 1):
            return False
        if not self.write_register(gripper_side, 0x0106, 0x07D0, 1):
            return False
        if not self.write_register(gripper_side, 0x0107, 0x07D0, 1):
            return False
        if not self.write_register(gripper_side, 264, 1, 1):
            return False
        return self.wait_for_completion(gripper_side, timeout)

    def wait_for_completion(self, gripper_side, timeout):
        start_time = time.time()
        # 清空历史读结果
        with self.result_lock:
            self.results['read'] = None
        # 轮询读取完成标志寄存器（手册：1538 位置到达=1）
        
        while time.time() - start_time < timeout:
            # 每轮以较短超时读取一次，避免长阻塞
            self.read_register(gripper_side, 0x0602, 1, timeout=0.3)
            with self.result_lock:
                r = self.results.get('read')
            if r and r.state and hasattr(r, 'data') and len(r.data) > 0:
                if int(r.data[0]) == 1:
                    rospy.loginfo(f"{gripper_side}夹爪位置设置完成")
                    return True
            time.sleep(0.1)
        rospy.logwarn(f"夹爪硬件动作超时")
        return False


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