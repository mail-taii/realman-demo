# 任务执行脚本使用说明

## 概述

`execute_tasks.py` 是一个整合了AGV、机械臂和夹爪控制的主执行脚本，用于演示完整的机器人任务执行流程。

## 功能特性

### 🤖 整合的客户端
- **AGV客户端** (`agv_client.py`) - AGV导航控制
- **机械臂客户端** (`arm_client.py`) - 双臂关节和空间运动控制  
- **夹爪客户端** (`gripper_client.py`) - 双夹爪位置控制

### 🎯 演示任务
1. **AGV导航演示** - 发送导航任务到指定点位
2. **机械臂控制演示** - 双臂回零点和关节运动
3. **夹爪控制演示** - 夹爪开合和位置设置
4. **协调任务演示** - 多系统配合的完整抓取放置流程

## 使用方法

### 方法1: 通过Launch文件启动（推荐）

```bash
# 启动完整系统（包括所有控制节点）
roslaunch task_planning execute_tasks.launch

# 启用调试模式
roslaunch task_planning execute_tasks.launch debug:=true

# 指定输出到日志文件
roslaunch task_planning execute_tasks.launch output:=log

# 只启动任务执行节点（不启动控制节点）
roslaunch task_planning execute_tasks.launch start_control_nodes:=false
```

### 方法2: 直接运行Python脚本

```bash
# 确保在ROS环境中
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash  # 替换为你的工作空间

# 运行脚本
rosrun task_planning execute_tasks.py
```

## 前置条件

### 自动启动模式（推荐）
使用 `roslaunch task_planning execute_tasks.launch` 会自动启动所有必需的控制节点：
- AGV控制节点
- 机械臂控制节点  
- 夹爪控制节点

### 手动启动模式
如果使用 `start_control_nodes:=false` 或直接运行Python脚本，需要手动启动以下服务：

1. **AGV控制服务**
   ```bash
   roslaunch agv_control_handler agv_control.launch
   ```

2. **机械臂控制服务**
   ```bash
   roslaunch arm_control_handler arm_control.launch
   ```

3. **夹爪控制服务**
   ```bash
   roslaunch gripper_control_handler gripper_control.launch
   ```

### ROS话题和服务
脚本会连接到以下ROS接口：
- `/navigation_task` - AGV导航任务发布
- `/agv_task_status` - AGV状态反馈订阅
- `movej_control` - 机械臂关节运动Action
- `movepose_control` - 机械臂空间运动Action
- `gripper_set_position_left` - 左夹爪控制Action
- `gripper_set_position_right` - 右夹爪控制Action

## 脚本结构

### TaskExecutor类
主要的任务执行器类，包含以下方法：

- `__init__()` - 初始化所有客户端
- `demo_agv_navigation()` - AGV导航演示
- `demo_arm_control()` - 机械臂控制演示
- `demo_gripper_control()` - 夹爪控制演示
- `demo_coordinated_task()` - 协调任务演示
- `execute_all_demos()` - 执行所有演示任务

### 执行流程
1. 初始化所有客户端连接
2. 按序执行各个子系统演示
3. 最后执行协调任务演示
4. 保持节点运行状态

## 输出日志

脚本会输出详细的执行日志，包括：
- ✅ 成功操作标识
- ❌ 错误操作标识
- ⚠️ 警告信息标识
- 📡 通信状态信息
- 🤖 机器人动作信息

## 故障排除

### 常见问题

1. **客户端连接失败**
   - 检查相应的控制服务是否启动
   - 确认ROS_MASTER_URI设置正确
   - 验证网络连接状态

2. **动作执行失败**
   - 检查硬件连接状态
   - 确认机器人处于安全位置
   - 查看详细错误日志

3. **超时错误**
   - 增加动作超时时间
   - 检查硬件响应速度
   - 确认系统负载情况

### 调试模式
启用调试模式可获得更详细的日志信息：
```bash
roslaunch task_planning execute_tasks.launch debug:=true
```

## 自定义任务

可以在 `TaskExecutor` 类中添加自定义的任务方法：

```python
def custom_task(self):
    """自定义任务"""
    rospy.loginfo("🎯 执行自定义任务...")
    
    # 调用各个客户端的方法
    self.agv_client.send_navigation_task("CUSTOM_POINT")
    self.arm_client.move_joint("left", [0, 0, 0, 0, 0, 0, 0])
    self.gripper_client.set_gripper_position("left", 5000)
    
    rospy.loginfo("✅ 自定义任务完成")
```

## 文件结构

```
task_planning/
├── scripts/
│   ├── agv_client.py          # AGV控制客户端
│   ├── arm_client.py          # 机械臂控制客户端
│   ├── gripper_client.py      # 夹爪控制客户端
│   └── execute_tasks.py       # 主任务执行脚本
├── launch/
│   └── execute_tasks.launch   # Launch启动文件
└── README_execute_tasks.md    # 使用说明文档
```

## 联系信息

如有问题或建议，请联系开发团队。
