# Gripper Control Handler Package - Action版本

## 概述

`gripper_control_handler` 是一个基于**ROS Action**的双夹爪机器人控制处理器，类似于`arm_control_handler`的设计理念。该包作为高级任务规划与底层夹爪硬件控制之间的中间层，提供统一的Action接口来控制左右夹爪。

### 架构角色

- **输入层**: 接收高级夹爪控制命令（Action Goals）
- **处理层**: 解析Action参数，执行夹爪控制逻辑
- **硬件接口层**: 与夹爪硬件通过TCP/JSON协议通信
- **反馈层**: 通过Action Feedback实时反馈执行状态和进度

## 🚀 Action架构的优势

相比传统的话题（Topic）或服务（Service），Action提供了以下独特优势：

- **✅ 实时进度反馈**: 可以实时监控夹爪动作执行进度
- **✅ 任务可取消性**: 支持随时中断正在执行的夹爪动作
- **✅ 异步非阻塞**: 客户端不会被长时间的夹爪动作阻塞
- **✅ 详细状态信息**: 提供丰富的执行状态和调试信息
- **✅ 内置错误处理**: 完善的异常处理和状态管理
- **✅ 并发友好**: 支持多个独立Action同时运行
- **✅ 无需额外状态话题**: Action的Feedback机制已提供所有必要的状态信息

## 功能特性

- **双Action架构**: 分离的GripperControl（一般控制）和GripperPick（力控抓取）
- **双夹爪支持**: 支持左夹爪、右夹爪独立控制
- **多种控制模式**: 开合、位置设置、力控抓取
- **硬件抽象**: 将高级控制意图转换为底层JSON通信协议
- **实时反馈**: Goal/Result/Feedback三层反馈机制
- **任务监控**: 实时进度、状态和调试信息
- **安全停止**: 支持紧急取消和安全停止
- **并发控制**: 支持左右夹爪独立并发控制

## 包结构

```
gripper_control_handler/
├── CMakeLists.txt              # 编译配置文件
├── package.xml                 # 包依赖配置
├── README.md                   # 本文档
├── action/                     # Action定义文件
│   ├── GripperControl.action   # 一般夹爪控制Action
│   └── GripperPick.action      # 力控抓取Action
├── launch/                     # 启动文件
│   └── gripper_control.launch  # 夹爪控制服务器启动
├── scripts/                    # 可执行脚本
│   └── gripper_client.py       # 客户端使用示例
└── src/                        # 源代码
    └── gripper_control_node.py # Action服务器实现
```

## Action定义

### GripperControl.action - 一般夹爪控制

```action
# Goal - 夹爪控制目标
string gripper_side          # 夹爪选择: "left", "right"
string action_type           # 动作类型: "open", "close", "set_position"
uint16 position             # 目标位置 (0-12000, 仅当action_type为"set_position"时使用)
uint16 speed                # 速度 (1-1000, 默认500)
uint16 force                # 力度 (1-1000, 默认50)
float64 timeout             # 任务超时时间 (秒，0表示无超时)

---

# Result - 执行结果
bool success                # 任务是否成功完成
string message             # 详细信息或错误描述
float64 execution_time     # 实际执行时间（秒）
string final_gripper_side  # 最终执行的夹爪
uint16 final_position      # 最终位置

---

# Feedback - 实时反馈
string status              # 当前状态: "initializing", "executing", "completed", "error"
float64 progress          # 进度百分比 (0.0-1.0)
float64 elapsed_time      # 已用时间（秒）
string active_gripper     # 当前活动的夹爪
uint16 current_position   # 当前位置
string debug_info         # 调试信息
```

### GripperPick.action - 力控抓取

```action
# Goal - 抓取目标
string gripper_side          # 夹爪选择: "left", "right"
uint16 pick_speed           # 抓取速度 (1-1000)
uint16 pick_force           # 抓取力度 (1-1000)
bool continuous_pick        # 是否持续力控夹取
float64 timeout             # 任务超时时间 (秒，0表示无超时)

---

# Result - 执行结果
bool success                # 任务是否成功完成
string message             # 详细信息或错误描述
float64 execution_time     # 实际执行时间（秒）
string final_gripper_side  # 最终执行的夹爪
bool object_detected       # 是否检测到物体

---

# Feedback - 实时反馈
string status              # 当前状态: "approaching", "gripping", "holding", "completed", "error"
float64 progress          # 进度百分比 (0.0-1.0)
float64 elapsed_time      # 已用时间（秒）
string active_gripper     # 当前活动的夹爪
uint16 current_force      # 当前力度
string debug_info         # 调试信息
```

## Action接口

### 夹爪控制接口

- **`/gripper_control`** (gripper_control_handler/GripperControlAction):
  - **作用**: 接收一般夹爪控制命令（开合、位置设置）
  - **格式**: 抽象的夹爪控制意图（动作类型、目标位置等）
  - **客户端**: task_planning包或其他高级规划模块

- **`/gripper_pick`** (gripper_control_handler/GripperPickAction):
  - **作用**: 接收力控抓取命令
  - **格式**: 抽象的抓取意图（抓取力度、速度等）
  - **客户端**: task_planning包或其他高级规划模块

## 使用方法

### 1. 编译包

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 启动Action服务器

```bash
# 启动夹爪控制处理器
roslaunch gripper_control_handler gripper_control.launch
```

### 3. 使用Action客户端

#### 方法一：使用客户端脚本

```bash
# 交互式演示
rosrun gripper_control_handler gripper_client.py interactive

# 基本控制演示
rosrun gripper_control_handler gripper_client.py basic

# 力控抓取演示
rosrun gripper_control_handler gripper_client.py pick
```

#### 方法二：使用Python API

```python
#!/usr/bin/env python3
import rospy
import actionlib
from gripper_control_handler.msg import GripperControlAction, GripperControlGoal

# 初始化客户端
rospy.init_node('gripper_test')
client = actionlib.SimpleActionClient('gripper_control', GripperControlAction)
client.wait_for_server()

# 发送控制命令
goal = GripperControlGoal()
goal.gripper_side = "left"
goal.action_type = "open"
goal.speed = 500
goal.force = 50

client.send_goal(goal)
client.wait_for_result()

result = client.get_result()
print(f"结果: {result.success}, 消息: {result.message}")
```

## 工作流程

### 夹爪控制流程

```
上层任务规划 (task_planning)
       ↓ 高级夹爪命令
GripperControl Action Goals {gripper_side, action_type, position, speed, force}
       ↓ 
gripper_control_handler (命令处理器)
       ↓ 解析并转换为硬件指令
TCP/JSON 通信 {command, parameters}
       ↓
夹爪硬件 (左夹爪: 169.254.128.18, 右夹爪: 169.254.128.19)
       ↓ 执行动作，返回状态
JSON 响应 {state, data}
       ↓ 状态监控
gripper_control_handler (状态处理器)
       ↓ 当执行完成时
Action Result {success: true, message, execution_time}
       ↓
上层任务规划 (接收完成反馈)
```

## Action状态监控

### 实时反馈监控

```bash
# 监控GripperControl Action反馈
rostopic echo /gripper_control/feedback

# 监控GripperPick Action反馈
rostopic echo /gripper_pick/feedback

# 监控Action状态
rostopic echo /gripper_control/status
rostopic echo /gripper_pick/status

# 监控Action结果
rostopic echo /gripper_control/result
rostopic echo /gripper_pick/result
```

### Action状态机

```
PENDING → ACTIVE → SUCCEEDED
    ↓         ↓         ↑
 REJECTED  PREEMPTED  ABORTED
```

## 高级功能

### 1. 任务取消

```python
# 通过Action客户端取消任务
client.gripper_control_client.cancel_goal()    # 取消控制任务
client.gripper_pick_client.cancel_goal()       # 取消抓取任务
```

### 2. 顺序控制

```python
# 一次控制一个夹爪，可以顺序执行不同操作

# 先控制左夹爪
control_goal = GripperControlGoal()
control_goal.gripper_side = "left"
control_goal.action_type = "open"
client.gripper_control_client.send_goal_and_wait(control_goal)

# 再控制右夹爪
pick_goal = GripperPickGoal()
pick_goal.gripper_side = "right"
pick_goal.pick_force = 100
client.gripper_pick_client.send_goal_and_wait(pick_goal)
```

### 3. 实时状态监控

```python
def feedback_callback(feedback):
    print(f"进度: {feedback.progress*100:.1f}%")
    print(f"状态: {feedback.status}")
    print(f"调试信息: {feedback.debug_info}")

# 发送目标并监控反馈
client.send_goal(goal, feedback_cb=feedback_callback)
```

## 硬件配置

### 夹爪网络配置

- **左夹爪IP**: 169.254.128.18
- **右夹爪IP**: 169.254.128.19
- **通信端口**: 8080
- **协议**: TCP/JSON
- **超时时间**: 10秒

### 参数配置

可在launch文件中修改以下参数：

```xml
<param name="gripper_control/left_gripper_ip" value="169.254.128.18" />
<param name="gripper_control/right_gripper_ip" value="169.254.128.19" />
<param name="gripper_control/gripper_port" value="8080" />
<param name="gripper_control/communication_timeout" value="10.0" />
```

## 设计优势

### 为什么不需要额外的状态话题？

与传统的话题发布方式不同，基于Action的设计有以下优势：

1. **Action Feedback已提供所有状态信息**: 包括执行进度、当前状态、错误信息等
2. **避免重复信息**: 不需要额外的状态话题来重复Action已有的信息
3. **更好的生命周期管理**: Action自动管理任务的开始、进行和结束状态
4. **内置错误处理**: Action框架提供完善的错误处理和超时机制
5. **更清晰的架构**: 客户端只需关注Action接口，无需订阅额外的状态话题

这种设计遵循了ROS Action的最佳实践，与`arm_control_handler`保持一致的架构风格。

## 故障排除

### 常见问题

1. **无法连接到夹爪硬件**
   - 检查网络连接和IP配置
   - 确认夹爪电源和通信端口

2. **Action服务器无响应**
   - 检查节点是否正常启动
   - 确认Action名称和消息类型正确

3. **执行超时**
   - 调整timeout参数
   - 检查夹爪硬件状态

### 调试命令

```bash
# 检查Action服务器状态
rostopic list | grep gripper

# 查看Action服务器信息
rostopic info /gripper_control/goal
rostopic info /gripper_pick/goal

# 监控系统日志
rosrun rqt_console rqt_console
```
