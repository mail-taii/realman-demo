# Task Planning API 文档

## 📋 概述

本文档详细介绍了`task_planning`包中所有API客户端的使用方法、参数说明和示例代码。这些API客户端为机器人任务规划提供了统一的控制接口。

## 🤖 AGV控制API (agv_client.py)

### AGVClient类

AGV导航控制客户端，提供移动底盘导航功能。

#### 对外函数列表

| 函数名 | 功能描述 | 参数 | 返回值 |
|--------|----------|------|--------|
| `__init__()` | 初始化AGV客户端 | 无 | None |
| `send_navigation_task()` | 发送导航任务 | target_mark, task_id, task_exect, task_type, direction | None |

#### 初始化
```python
from apis import AGVClient

# 创建AGV客户端
agv_client = AGVClient()
```

#### 主要方法

##### send_navigation_task()
```python
def send_navigation_task(self, target_mark, task_id=1, task_exect=1, task_type=1, direction=0):
    """
    发送AGV导航任务
    
    Args:
        target_mark (str): 目标点位标记，如 "A1", "B2", "C3"
        task_id (int, optional): 任务唯一标识符，默认值: 1
        task_exect (int, optional): 任务执行模式，默认值: 1
        task_type (int, optional): 任务类型标识，默认值: 1
        direction (int, optional): 朝向角度(度)，默认值: 0
    
    Returns:
        None: 异步执行，通过回调函数获取结果
    """
```

**使用示例**:
```python
# 导航到A1点位
agv_client.send_navigation_task("A1")

# 导航到B2点位，设置朝向为90度
agv_client.send_navigation_task("B2", task_id=2, direction=90)

# 检查任务完成状态
if agv_client.arrive_state:
    print("AGV已到达目标点位")
```

#### 状态监听
```python
# 任务状态通过arrive_state属性获取
if agv_client.arrive_state:
    print("任务已完成")
```


---

## 🦾 机械臂控制API (arm_client.py)

### ArmClient类

双臂机械臂控制客户端，支持关节空间和笛卡尔空间运动。

#### 对外函数列表

| 函数名 | 功能描述 | 参数 | 返回值 |
|--------|----------|------|--------|
| `__init__()` | 初始化机械臂客户端 | 无 | None |
| `move_joint()` | 关节空间运动 | arm_side, joint_angles_deg, speed, timeout | bool |
| `move_pose()` | 笛卡尔空间运动 | arm_side, target_pose, speed, timeout | bool |
| `get_quaternion()` | 获取四元数 | angles (欧拉角) | list |

#### 初始化
```python
from apis import ArmClient

# 创建机械臂客户端
arm_client = ArmClient()
```

#### 主要方法

##### move_joint() - 关节空间运动
```python
def move_joint(self, arm_side, joint_angles_deg, speed=0.2, timeout=15.0):
    """
    关节空间运动
    
    Args:
        arm_side (str): 手臂选择 ("left" 或 "right")
        joint_angles_deg (list): 关节角度列表(度) [j1, j2, j3, j4, j5, j6, j7]
        speed (float): 运动速度 (0.1-1.0)
        timeout (float): 超时时间(秒)
    
    Returns:
        bool: 运动是否成功
    """
```

##### move_pose() - 笛卡尔空间运动
```python
def move_pose(self, arm_side, target_pose, speed=0.1, timeout=15.0):
    """
    笛卡尔空间运动
    
    Args:
        arm_side (str): 手臂选择 ("left" 或 "right")
        target_pose (list): 目标位姿 [x, y, z, qx, qy, qz, qw]
        speed (float): 运动速度 (0.1-1.0)
        timeout (float): 超时时间(秒)
    
    Returns:
        bool: 运动是否成功
    """
```

##### get_quaternion() - 获取四元数
```python
def get_quaternion(self, angles):
    """
    将欧拉角转换为四元数
    
    Args:
        angles (list): 欧拉角 [rx, ry, rz] (弧度)
    
    Returns:
        list: 四元数 [qx, qy, qz, qw]
    """
```

**使用示例**:
```python
# 关节运动 - 左臂回到初始位置
home = [-9, 31, -66, -53, -11, -41, 18]
arm_client.move_joint("left", home)

# 关节运动 - 右臂对称位置
arm_client.move_joint("right", [-home[i] for i in range(len(home))])

# 笛卡尔运动 - 右臂移动到指定位置
target_pose = [171, 408, 137] + arm_client.get_quaternion([-3.082, 0.379, -1.389])
arm_client.move_pose("right", target_pose)

# 使用四元数进行位姿控制
quaternion = arm_client.get_quaternion([-3.086, 0.8, -1.333])
pose = [216, 296, 434] + quaternion
arm_client.move_pose("right", pose)
```

---

## 🦾 夹爪控制API (gripper_client.py)

### GripperClient类

双夹爪控制客户端，支持夹爪位置控制和Modbus通信。

#### 对外函数列表

| 函数名 | 功能描述 | 参数 | 返回值 |
|--------|----------|------|--------|
| `__init__()` | 初始化夹爪客户端 | 无 | None |
| `set_gripper_position()` | 设置夹爪位置 | gripper_side, position, speed, force, timeout | bool |

#### 初始化
```python
from apis import GripperClient

# 创建夹爪客户端
gripper_client = GripperClient()
```

#### 主要方法

##### set_gripper_position() - 设置夹爪位置
```python
def set_gripper_position(self, gripper_side, position, speed=500, force=50, timeout=15.0):
    """
    设置夹爪位置
    
    Args:
        gripper_side (str): 夹爪选择 ("left" 或 "right")
        position (int): 目标位置 (0-12000)
        speed (int): 运动速度 (默认: 500)
        force (int): 夹持力(%) (默认: 50)
        timeout (float): 超时时间(秒)
    
    Returns:
        bool: 设置是否成功
    """
```

**使用示例**:
```python


# 设置右夹爪位置为12000（闭合）
gripper_client.set_gripper_position("right", 12000)

# 设置右夹爪位置为0（张开）
gripper_client.set_gripper_position("right", 0)

# 双夹爪同时控制
gripper_client.gripper_pick_dual(1, 2)  # 左张开，右闭合
```


## 📏 升降平台控制API (lift_client.py)

### LiftClient类

升降平台控制客户端，控制平台高度。

#### 对外函数列表

| 函数名 | 功能描述 | 参数 | 返回值 |
|--------|----------|------|--------|
| `__init__()` | 初始化升降平台客户端 | 无 | None |
| `lift_height()` | 设置升降平台高度 | height | None |

---

#### 初始化
```python
from apis import LiftClient

# 创建升降平台客户端
lift_client = LiftClient()
```

#### 主要方法

##### 设置平台高度
```python
def lift_height(self, height):
    """
    设置升降平台高度
    
    Args:
        height (int): 目标高度 (mm)
    """
```

**使用示例**:
```python
# 升降到470mm高度 (最高)
lift_client.lift_height(470)

# 升降到200mm高度（最低）
lift_client.lift_height(200)
```
