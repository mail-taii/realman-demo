# 具身双臂机器人开发规范与架构指南

## 📋 项目概述

本项目是一个基于ROS1的具身双臂机器人系统，集成了AGV移动底盘、双臂机械臂、夹爪控制、相机视觉和升降平台等模块。系统采用分布式架构，通过`task_planning`包作为核心协调器，实现多模块协同控制。

## 🏗️ 系统架构

### 核心组件架构图

```
┌─────────────────────────────────────────────────────────────┐
│                    Task Planning Layer                      │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              execute_tasks.launch                   │   │
│  │  ┌─────────────────────────────────────────────────┐ │   │
│  │  │            execute_tasks.py                     │ │   │
│  │  │  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌──────┐ │ │   │
│  │  │  │AGVClient│ │ArmClient│ │Gripper  │ │Lift  │ │ │   │
│  │  │  │         │ │         │ │Client   │ │Client│ │ │   │
│  │  │  └─────────┘ └─────────┘ └─────────┘ └──────┘ │ │   │
│  │  └─────────────────────────────────────────────────┘ │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   Hardware Control Layer                    │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌────────┐ │
│  │AGV Control │ │Arm Control  │ │Gripper      │ │Lift    │ │
│  │Handler     │ │Handler      │ │Control      │ │Control │ │
│  └─────────────┘ └─────────────┘ └─────────────┘ └────────┘ │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    Hardware Layer                          │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌────────┐ │
│  │AGV Chassis  │ │Dual Arms    │ │Grippers     │ │Lift    │ │
│  │(RM底盘)     │ │(RM65/RM75)  │ │(知行)  │ │Platform│ │
│  └─────────────┘ └─────────────┘ └─────────────┘ └────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 包结构说明

```
src/rmc_aida_l_ros1-develop/src/
├── task_planning/                    # 🎯 核心任务规划包
│   ├── apis/                        # API客户端接口
│   │   ├── agv_client.py           # AGV控制客户端
│   │   ├── arm_client.py            # 机械臂控制客户端
│   │   ├── gripper_client.py       # 夹爪控制客户端
│   │   └── lift_client.py          # 升降平台控制客户端
│   ├── launch/
│   │   └── execute_tasks.launch    # 主启动文件
│   ├── scripts/
│   │   └── execute_tasks.py        # 主执行脚本
│   └── API_DOCUMENTATION.md         # 📚 详细API文档
├── embodied_arm/                    # 🤖 机械臂相关包
│   ├── embodied_arm_driver/        # 机械臂驱动
│   ├── embodied_arm_control/       # 机械臂控制
│   └── embodied_arm_msgs/          # 机械臂消息定义
├── embodied_demo/                   # 🎮 演示包
│   ├── embodied_arm_demo/          # 机械臂演示
│   ├── embodied_gripper_demo/      # 夹爪演示
│   └── embodied_camera_demo/       # 相机演示
├── embodied_moveit/                 # 🎯 运动规划包
├── embodied_gazebo/                 # 🌐 仿真包
├── embodied_camera/                 # 📷 相机包
├── embodied_servo/                  # 🎛️ 舵机控制包
└── robot_description/               # 🤖 机器人描述包
```

## 📚 API文档

### 详细API文档

`task_planning`包提供了完整的API文档，详细介绍了所有客户端的使用方法：

```bash
# 查看详细API文档
cat src/rmc_aida_l_ros1-develop/src/task_planning/API_DOCUMENTATION.md
```

**API文档包含**：
- 🤖 **AGV控制API**: 导航任务发送和状态监听
- 🦾 **机械臂控制API**: 关节空间和笛卡尔空间运动
- 🦾 **夹爪控制API**: 双夹爪开合和位置控制
- 📏 **升降平台API**: 平台高度控制
- 📷 **相机控制API**: 图像获取和处理
- 🎛️ **舵机控制API**: 头部舵机转动控制

**快速API使用示例**：
```python
from apis import AGVClient, ArmClient, GripperClient

# 初始化客户端
agv_client = AGVClient()
arm_client = ArmClient()
gripper_client = GripperClient()

# 执行任务
agv_client.send_navigation_task("A1")
arm_client.movej_left([0, -90, 0, -90, 0, 0, 0])
gripper_client.gripper_pick(1, 2)  # 左夹爪闭合
```

## 🚀 快速开始

### 环境要求

- **ROS版本**: ROS Noetic
- **Python版本**: Python 3.6+
- **操作系统**: Ubuntu 20.04 LTS
- **硬件**: 具身双臂机器人系统

### 系统启动

#### 方法1: 一键启动（推荐）

```bash
# 进入工作空间
cd /Users/chloeya/realman_demo

# 编译工作空间
catkin_make

# 启动完整系统
./start_robot_system.sh
```

## 📦 新包开发规范

### 1. 包结构标准

创建新包时，请遵循以下目录结构：

```
your_new_package/
├── CMakeLists.txt                   # CMake构建文件
├── package.xml                      # 包元数据
├── README.md                        # 包说明文档
├── launch/                          # 启动文件目录
│   └── your_package.launch         # 主启动文件
├── scripts/                         # Python脚本目录
│   └── your_script.py              # 主要脚本
├── src/                            # C++源代码目录（可选）
├── config/                         # 配置文件目录
│   └── your_config.yaml            # 配置文件
├── msg/                            # 自定义消息（可选）
├── srv/                            # 自定义服务（可选）
└── action/                         # 自定义动作（可选）
```

### 2. 包命名规范

- **功能包**: 使用`embodied_`前缀，如`embodied_your_feature`
- **演示包**: 使用`embodied_*_demo`格式，如`embodied_sensor_demo`
- **控制包**: 使用`*_control_handler`格式，如`sensor_control_handler`

### 3. 集成到任务规划系统

#### 3.1 创建API客户端

如果需要，在`task_planning/apis/`目录下创建客户端文件：

```python
#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
YourFeature控制API客户端 - 为task_planning提供YourFeature控制接口
"""

import rospy
from your_package_msgs.msg import YourMessage

class YourFeatureClient:
    """YourFeature控制客户端类"""
    
    def __init__(self):
        """初始化客户端"""
        # 创建发布器
        self.publisher = rospy.Publisher('/your_topic', YourMessage, queue_size=10)
        
        # 创建订阅器
        self.subscriber = rospy.Subscriber('/your_status', YourMessage, self.callback)
        
        rospy.loginfo("YourFeatureClient initialized")
    
    def your_control_method(self, parameter):
        """控制方法"""
        # 实现控制逻辑
        pass
    
    def callback(self, msg):
        """状态回调"""
        # 处理状态反馈
        pass
```

#### 3.2 修改execute_tasks.py

在`task_planning/scripts/execute_tasks.py`中集成新客户端：

```python
# 导入新客户端
from apis import YourFeatureClient

# 在initialize_clients()函数中添加
def initialize_clients():
    global your_feature_client
    try:
        your_feature_client = YourFeatureClient()
        rospy.loginfo("YourFeatureClient initialized successfully")
    except Exception as e:
        rospy.logerr(f"YourFeatureClient initialization failed: {e}")
```

#### 3.3 更新execute_tasks.launch

在`task_planning/launch/execute_tasks.launch`中添加新包：

```xml
<!-- 启动YourFeature控制节点 -->
<include file="$(find your_control_handler)/launch/your_control.launch" 
         if="$(arg start_control_nodes)" />
```

#### 3.4 更新CMakeLists.txt

在`task_planning/CMakeLists.txt`中添加新API：

```cmake
## 安装APIs包
catkin_install_python(PROGRAMS
  apis/__init__.py
  apis/agv_client.py
  apis/arm_client.py
  apis/gripper_client.py
  apis/your_feature_client.py  # 添加新客户端
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/apis
)
```

### 4. 创建独立启动文件

如果需要独立运行新包，在`task_planning/launch/`目录下创建：

```xml
<?xml version="1.0"?>
<launch>
    <!-- YourFeature独立启动文件 -->
    
    <!-- 参数配置 -->
    <arg name="debug" default="false" doc="是否启用调试模式" />
    <arg name="output" default="screen" doc="输出模式: screen/log" />
    
    <!-- 启动YourFeature控制节点 -->
    <include file="$(find your_control_handler)/launch/your_control.launch" />
    
    <!-- 启动YourFeature演示节点 -->
    <node name="your_feature_demo" 
          pkg="task_planning" 
          type="your_feature_demo.py" 
          output="$(arg output)">
        <param name="debug_mode" value="$(arg debug)" />
    </node>
    
</launch>
```

## 🔧 开发最佳实践

### 1. 代码规范

- **Python代码**: 遵循PEP 8规范
- **C++代码**: 遵循Google C++ Style Guide
- **注释**: 使用中文注释，详细说明功能
- **文档**: 每个包必须包含README.md

### 2. 错误处理

```python
def robust_method(self):
    """带错误处理的稳健方法"""
    try:
        # 主要逻辑
        result = self.perform_operation()
        rospy.loginfo("操作成功完成")
        return result
    except rospy.ROSException as e:
        rospy.logerr(f"ROS通信错误: {e}")
        return None
    except Exception as e:
        rospy.logerr(f"未知错误: {e}")
        return None
```

### 3. 参数配置

使用ROS参数服务器进行配置：

```python
class YourClient:
    def __init__(self):
        # 从参数服务器获取配置
        self.timeout = rospy.get_param('~timeout', 5.0)
        self.retry_count = rospy.get_param('~retry_count', 3)
```

### 4. 日志规范

```python
# 使用不同级别的日志
rospy.loginfo("信息日志 - 正常操作")
rospy.logwarn("警告日志 - 需要注意的情况")
rospy.logerr("错误日志 - 错误情况")
rospy.logdebug("调试日志 - 调试信息")
```

## 🧪 测试与调试

### 1. 单元测试

为每个客户端创建测试脚本：

```python
#!/usr/bin/env python3
# test_your_feature_client.py

import rospy
import unittest
from apis import YourFeatureClient

class TestYourFeatureClient(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_your_feature_client')
        self.client = YourFeatureClient()
    
    def test_initialization(self):
        """测试客户端初始化"""
        self.assertIsNotNone(self.client)
    
    def test_control_method(self):
        """测试控制方法"""
        result = self.client.your_control_method("test_param")
        self.assertIsNotNone(result)

if __name__ == '__main__':
    unittest.main()
```

### 2. 集成测试

使用launch文件进行集成测试：

```bash
# 测试完整系统
roslaunch task_planning execute_tasks.launch debug:=true

# 测试单个组件
roslaunch your_package your_test.launch
```

## 📚 常用命令

### 系统管理

```bash
# 查看运行中的节点
rosnode list

# 查看话题列表
rostopic list

# 查看服务列表
rosservice list

# 查看参数
rosparam list
```

### 调试命令

```bash
# 查看话题内容
rostopic echo /your_topic

# 查看节点信息
rosnode info /your_node

# 查看服务信息
rosservice info /your_service
```

## 🔄 版本控制

### Git工作流

1. **功能开发**: 在feature分支开发新功能
2. **代码审查**: 通过Pull Request进行代码审查
3. **测试验证**: 在测试环境验证功能
4. **合并发布**: 合并到main分支并打标签

### 提交规范

```
feat: 添加新功能
fix: 修复bug
docs: 更新文档
style: 代码格式调整
refactor: 代码重构
test: 添加测试
chore: 构建过程或辅助工具的变动
```

## 🆘 故障排除

### 常见问题

1. **节点启动失败**
   - 检查依赖包是否安装
   - 检查launch文件路径
   - 查看日志输出

2. **通信问题**
   - 检查话题名称是否正确
   - 检查消息类型是否匹配
   - 使用`rostopic info`检查话题信息

3. **权限问题**
   - 确保脚本有执行权限: `chmod +x your_script.py`
   - 检查ROS环境变量设置

### 调试技巧

```bash
# 启用详细日志
export ROSCONSOLE_CONFIG_FILE=/path/to/rosconsole.conf

# 查看特定节点日志
rosnode info /your_node

# 监控系统资源
htop
```

## 📞 技术支持

- **文档**: 查看各包的README.md文件
- **问题反馈**: 通过GitHub Issues提交问题
- **开发讨论**: 参与项目讨论区

---

## 📝 更新日志

- **v1.0.0** (2024-01-01): 初始版本，包含基础架构和开发规范
- **v1.1.0** (2024-01-15): 添加新包开发指南和最佳实践
- **v1.2.0** (2024-02-01): 完善测试规范和故障排除指南

---

**注意**: 本开发规范会持续更新，请定期查看最新版本。