# ROS1具身双臂机器人embodied_gripper_demo功能包详解

## 功能包概述

`embodied_gripper_demo` 是一个用于具身双臂机器人夹爪控制演示的功能包。该功能包提供了知行CTAG2F120夹爪的完整控制功能，支持双夹爪同时控制。

## 功能包结构

```
embodied_gripper_demo/
├── CMakeLists.txt              # CMake构建文件
├── package.xml                 # 功能包配置文件
├── scripts/                    # Python脚本目录
│   └── gripper_demo.py         # 夹爪控制演示脚本(单文件包含所有功能)
├── launch/                     # 启动文件目录
│   └── gripper_demo.launch     # 夹爪演示启动文件
├── config/                     # 配置文件目录
│   └── gripper_params.yaml     # 夹爪参数配置文件
└── ros1具身双臂机器人embodied_gripper_demo功能包详解.md  # 本说明文档
```

## 主要功能

- **双夹爪同时控制**: 支持同时控制两个夹爪的开合动作
- **配置文件驱动**: 通过YAML配置文件管理夹爪参数
- **自动初始化**: 自动初始化夹爪系统(24V输出、MODBUS模式等)
- **位置反馈**: 实时监控夹爪位置状态
- **错误处理**: 完善的错误处理和日志输出

## 系统架构

### 核心类结构

1. **GripperCommunication类**
   - 负责底层JSON通信
   - 处理TCP socket连接
   - 提供MODBUS寄存器读写功能

2. **GripperControl类**
   - 高层控制逻辑
   - 夹爪初始化和状态管理
   - 位置控制和等待功能

### 控制流程

```
配置文件加载 → 创建夹爪控制器 → 初始化夹爪 → 执行控制命令 → 等待位置反馈
```

主要更新：
1. **移除了所有命令行选项说明**
2. **简化了运行方式说明**
3. **强调直接运行脚本即可**
4. **更新了版本说明，反映简化版本**
5. **移除了复杂的launch文件参数说明**

现在README反映了您要求的简化版本：只有一个默认选项，直接运行脚本即可！

### 通信协议

- **协议**: TCP Socket + JSON
- **端口**: 8080 (默认)
- **命令格式**: JSON格式的命令字符串
- **响应**: JSON格式的响应数据

## 依赖关系

- `rospy`: Python ROS客户端库
- `embodied_arm_msgs`: 具身双臂机器人消息类型定义
- `pyyaml`: YAML配置文件解析
- `socket`: TCP网络通信

## 配置说明

### gripper_params.yaml 配置文件

```yaml
# 多夹爪配置
grippers:
  - id: 0
    ip: "169.254.128.18"    # 左夹爪IP地址
    port: 8080
    name: "左夹爪"
  - id: 1
    ip: "169.254.128.19"    # 右夹爪IP地址
    port: 8080
    name: "右夹爪"

# 夹爪参数
gripper:
  open_position: 0          # 打开位置
  close_position: 12000     # 闭合位置
  default_torque: 50        # 默认力矩
  position_timeout: 3.0     # 位置等待超时时间(秒)
```

## 使用方法

### 1. 编译功能包

```bash
cd /home/daniel/rmc_aida_l_ros1-develop
catkin_make
source devel/setup.bash
```

### 2. 配置夹爪IP地址

编辑 `config/gripper_params.yaml` 文件，修改夹爪的IP地址：

```yaml
grippers:
  - id: 0
    ip: "你的左夹爪IP地址"
    port: 8080
    name: "左夹爪"
  - id: 1
    ip: "你的右夹爪IP地址"
    port: 8080
    name: "右夹爪"
```

### 3. 运行演示

**简单运行方式（推荐）**：
```bash
# 直接运行脚本
python /home/daniel/rmc_aida_l_ros1-develop/src/embodied_demo/embodied_gripper_demo/scripts/gripper_demo.py

# 或者使用rosrun
rosrun embodied_gripper_demo gripper_demo.py
```

**使用Launch文件**：
```bash
# 运行演示序列
roslaunch embodied_gripper_demo gripper_demo.launch demo_mode:=true
```

### 4. 演示序列说明

演示序列会自动执行以下步骤：

1. **加载配置**: 从YAML文件读取夹爪配置
2. **创建控制器**: 为每个夹爪创建独立的控制器实例
3. **初始化夹爪**: 
   - 设置24V输出
   - 配置MODBUS模式
   - 启用夹爪
4. **执行演示**:
   - 同时打开两个夹爪
   - 等待夹爪到达位置
   - 同时关闭两个夹爪
   - 等待夹爪到达位置
   - 再次同时打开两个夹爪
5. **完成**: 输出完成信息

## 工作原理

### 夹爪初始化流程

1. **设置工具电压**: 发送 `set_tool_voltage` 命令启用24V输出
2. **配置MODBUS**: 设置MODBUS通信参数(波特率115200, 超时2秒)
3. **启用夹爪**: 写入寄存器256设置为1来启用夹爪

### 位置控制流程

1. **写入位置高位**: 寄存器258设置为0
2. **写入位置低位**: 寄存器259设置为目标位置值
3. **触发运动**: 寄存器264设置为1触发夹爪运动
4. **等待反馈**: 读取寄存器1540检查是否到达目标位置

### 错误处理

- **通信超时**: 10秒超时保护
- **初始化失败**: 自动重试和错误报告
- **位置超时**: 3秒位置等待超时
- **配置文件错误**: 使用默认配置作为备选

## 故障排除

### 常见问题

1. **模块导入错误**
   - 确保所有Python依赖已安装
   - 检查ROS环境是否正确source

2. **通信超时**
   - 检查夹爪IP地址是否正确
   - 确认网络连接正常
   - 验证夹爪是否已上电

3. **初始化失败**
   - 检查24V输出是否正常
   - 确认MODBUS模式配置
   - 验证寄存器写入权限

### 调试方法

```bash
# 启用详细日志
export ROS_LOG_LEVEL=DEBUG

# 检查ROS话题
rostopic list

# 查看节点状态
rosnode list
```

## 版本信息

- 版本: 0.3.0
- 维护者: i
- 许可证: TODO

## 更新日志

- v0.3.0: 简化版本
  - 合并所有功能到单个Python文件
  - 移除所有命令行选项，只保留默认演示
  - 简化运行方式：直接运行脚本即可
  - 优化错误处理
  - 更新文档说明

## 注意事项

- 使用前请确保机器人硬件已正确连接
- 请根据实际硬件调整配置文件中的IP地址
- 建议在仿真环境中先进行测试
- 确保夹爪电源和网络连接正常
- **运行非常简单**：直接运行脚本即可，无需任何参数
