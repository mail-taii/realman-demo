# Task Planning 客户端接口文档

## 概述

`task_planning`包提供了一系列高级客户端接口，用于控制具身双臂机器人的各个组件。这些客户端封装了底层的ROS通信细节，为任务规划提供简洁易用的API接口。

## 客户端列表

### 1. AGV客户端 (`agv_client.py`)
**功能**：控制移动底盘的导航任务
**通信方式**：发布/订阅模式
**主要话题**：
- 发布：`/navigation_task` (NavigationTask)
- 订阅：`/agv_task_status` (String)

### 2. ARM客户端 (`arm_client.py`)  
**功能**：控制双臂机械臂的关节和空间运动
**通信方式**：Action模式
**主要Action**：
- `movej_control` (MoveJAction)
- `movepose_control` (MovePoseAction)

### 3. 夹爪客户端 (`gripper_client.py`) [计划中]
**功能**：控制双臂末端夹爪的开合动作
**通信方式**：发布/订阅或TCP Socket
**主要话题**：
- 发布：`/gripper_pick` (Gripper_Pick)
- 发布：`/gripper_set` (Gripper_Set)

### 4. 相机客户端 (`camera_client.py`) [计划中]
**功能**：获取和处理多相机图像数据
**通信方式**：订阅模式
**主要话题**：
- 订阅：`/camera_d435_*/color/image_raw` (Image)
- 订阅：`/camera_d435_*/depth/image_raw` (Image)

### 5. 舵机客户端 (`servo_client.py`) [计划中]
**功能**：控制头部舵机的转动角度
**通信方式**：发布/订阅模式
**主要话题**：
- 发布：`/servo_control/move` (ServoMove)
- 订阅：`/servo_state` (ServoAngle)

---

## AGV客户端详细说明

### 类：`AGVClient`

#### 初始化
```python
from agv_client import AGVClient
agv_client = AGVClient()
```

#### 主要方法

##### `send_navigation_task(target_mark, task_id=1, task_exect=1, task_type=1, direction=0)`
**功能**：发送AGV导航任务

**参数**：
- `target_mark` (str)：目标点位标记，如 "A1", "B2", "C3"
- `task_id` (int)：任务唯一标识符，默认值：1
- `task_exect` (int)：任务执行模式，默认值：1（正常执行）
- `task_type` (int)：任务类型标识，默认值：1（导航任务）
- `direction` (int)：到达目标后的朝向角度（度），默认值：0

**返回值**：None（异步执行，通过状态回调获取结果）

**使用示例**：
```python
# 基本导航
agv_client.send_navigation_task("A1")

# 带参数导航
agv_client.send_navigation_task("B2", task_id=2, direction=90)
```

##### `status_callback(msg)`
**功能**：自动处理AGV任务状态反馈

**状态类型**：
- `STARTED`：任务开始执行
- `COMPLETED`：任务执行完成
- `FAILED`：任务执行失败

---

## ARM客户端详细说明

### 类：`ArmClient`

#### 初始化
```python
from arm_client import ArmClient
arm_client = ArmClient()
```

#### 主要方法

##### `move_joint(arm_side, joint_angles_deg, speed=0.2, timeout=15.0)`
**功能**：控制机械臂关节运动

**参数**：
- `arm_side` (str)：机械臂选择，"left" 或 "right"
- `joint_angles_deg` (list)：7个关节角度（度），例：[0, -90, 0, 0, 0, 0, 0]
- `speed` (float)：关节运动速度，默认值：0.2
- `timeout` (float)：超时时间（秒），默认值：15.0

**返回值**：bool（True=成功，False=失败）

**使用示例**：
```python
# 左臂关节运动
success = arm_client.move_joint("left", [-12.177, -106.699, 4.056, -3.49, 8.441, -31.897, 58.77])

# 右臂关节运动，自定义速度
success = arm_client.move_joint("right", [0, 90, 0, 70, 0, -15, 0], speed=0.1)
```

##### `move_pose(arm_side, target_pose, speed=0.1, timeout=15.0)`
**功能**：控制机械臂空间位姿运动

**参数**：
- `arm_side` (str)：机械臂选择，"left" 或 "right"
- `target_pose` (list)：目标位姿 [x, y, z, qx, qy, qz, qw]
  - 位置单位：毫米
  - 姿态：四元数表示
- `speed` (float)：笛卡尔空间速度，默认值：0.1
- `timeout` (float)：超时时间（秒），默认值：15.0

**返回值**：bool（True=成功，False=失败）

**使用示例**：
```python
# 左臂空间运动
target_pose = [-355.865, 94.830, -399.345, 3.091, 0.093, 2.791, 0]
success = arm_client.move_pose("left", target_pose)

# 右臂空间运动，自定义速度
success = arm_client.move_pose("right", target_pose, speed=0.05)
```

##### `home_position(arm_side)`
**功能**：机械臂回零点位置

**参数**：
- `arm_side` (str)：机械臂选择，"left" 或 "right"

**返回值**：bool（True=成功，False=失败）

**使用示例**：
```python
# 双臂回零
arm_client.home_position("left")
arm_client.home_position("right")
```

##### `degrees_to_radians(degrees)`
**功能**：角度转换工具（度转弧度）

**参数**：
- `degrees` (float)：角度值

**返回值**：float（弧度值）

---

## 使用流程示例

### 完整任务执行流程
```python
#!/usr/bin/env python3
from agv_client import AGVClient
from arm_client import ArmClient
import rospy

def main():
    # 初始化客户端
    agv_client = AGVClient()
    arm_client = ArmClient()
    
    # 1. AGV导航到工作位置
    agv_client.send_navigation_task("A1", task_id=1)
    rospy.sleep(5)  # 等待导航完成
    
    # 2. 机械臂回零
    arm_client.home_position("left")
    arm_client.home_position("right")
    
    # 3. 左臂移动到抓取位置
    grasp_pose = [-300, 100, -200, 0, 0, 0, 1]
    arm_client.move_pose("left", grasp_pose)
    
    # 4. 执行抓取动作（需要夹爪客户端）
    # gripper_client.close("left")
    
    # 5. 移动到放置位置
    place_pose = [300, 100, -200, 0, 0, 0, 1]
    arm_client.move_pose("left", place_pose)
    
    # 6. 释放物体（需要夹爪客户端）
    # gripper_client.open("left")
    
    # 7. 返回零点
    arm_client.home_position("left")
    
    rospy.loginfo("任务执行完成")

if __name__ == "__main__":
    main()
```

---

## 错误处理

### 常见错误类型
1. **连接错误**：无法连接到底层控制节点
2. **超时错误**：动作执行超时
3. **参数错误**：无效的参数值
4. **状态错误**：机器人处于错误状态

### 错误处理建议
```python
try:
    success = arm_client.move_joint("left", joint_angles)
    if not success:
        rospy.logerr("关节运动失败")
        # 执行恢复策略
        arm_client.home_position("left")
except Exception as e:
    rospy.logerr(f"执行异常: {e}")
    # 执行紧急停止
```

---

## 开发规范

### 新增客户端步骤
1. 在 `scripts/` 目录下创建客户端文件
2. 实现客户端类，继承统一的基础接口
3. 提供完整的文档字符串
4. 添加使用示例和错误处理
5. 更新本README文档

### 命名规范
- 文件名：`{component}_client.py`
- 类名：`{Component}Client`
- 方法名：使用动词+名词的形式，如 `move_joint`, `send_task`

### 日志规范
- 使用rospy.loginfo()记录正常操作
- 使用rospy.logwarn()记录警告信息  
- 使用rospy.logerr()记录错误信息
- 统一使用emoji图标增强可读性

---

## 依赖项

### ROS包依赖
- `rospy`
- `actionlib`
- `geometry_msgs`
- `std_msgs`
- `arm_control_handler`
- `agv_control_handler`
- `embodied_arm_msgs`
- `embodied_servo`

### Python包依赖
- `math`
- `socket`
- `json`
- `cv2`
- `cv_bridge`

---

## 版本历史

- **v1.0.0** - 初始版本，包含AGV和ARM客户端
- **v1.1.0** - 计划添加夹爪客户端
- **v1.2.0** - 计划添加相机和舵机客户端

---

## 联系方式

如有问题或建议，请联系开发团队或提交Issue。
