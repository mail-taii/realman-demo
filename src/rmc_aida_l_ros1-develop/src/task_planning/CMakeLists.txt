cmake_minimum_required(VERSION 3.0.2)
project(task_planning)

## 查找catkin宏和库
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  sensor_msgs
)

## catkin特定配置
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs
    actionlib
    actionlib_msgs
    sensor_msgs
)

## 指定头文件的其他位置
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 标记Python脚本进行安装
catkin_install_python(PROGRAMS
  scripts/execute_tasks.py
  scripts/camera_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装APIs包
catkin_install_python(PROGRAMS
  apis/__init__.py
  apis/agv_client.py
  apis/arm_client.py
  apis/gripper_client.py
  apis/camera_client.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/apis
)

