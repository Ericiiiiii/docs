# PX4飞控ROS2快速入门指南

## 概述

本指南详细介绍如何配置和使用PX4飞控与ROS2进行通信，包括环境搭建、配置步骤和实际使用方法。

## 系统架构

```
┌─────────────────┐    uXRCE-DDS    ┌─────────────────┐
│   PX4飞控       │ ◄──────────────► │  任务计算机      │
│                 │                 │                 │
│ uORB消息        │                 │ ROS2节点        │
│ ↕               │                 │ ↕               │
│ uxrce_dds_client│                 │ MicroXRCEAgent  │
└─────────────────┘                 └─────────────────┘
```

## 第一步：PX4端配置

### 1.1 检查uXRCE-DDS支持

首先确认PX4固件支持uXRCE-DDS：

```bash
# 在PX4控制台中检查
uxrce_dds_client status

# 如果命令不存在，需要重新编译固件并启用uXRCE-DDS
```

### 1.2 PX4参数配置

设置uXRCE-DDS相关参数：

```bash
# 进入PX4控制台，设置以下参数：

# 启用uXRCE-DDS客户端
param set UXRCE_DDS_CFG 0    # 0=禁用, 1=UART, 2=UDP

# 如果使用UART连接（推荐）：
param set UXRCE_DDS_CFG 1
param set SER_UXR_BAUD 921600      # 设置波特率
param set UXRCE_DDS_PRT 102        # 使用TELEM2端口

# 如果使用UDP连接：
param set UXRCE_DDS_CFG 2
param set UXRCE_DDS_AG_IP 192168100  # 代理IP地址（192.168.1.100）
param set UXRCE_DDS_PRT 8888       # UDP端口

# 保存参数并重启
param save
reboot
```

### 1.3 启动uXRCE-DDS客户端

PX4重启后自动启动，或手动启动：

```bash
# UART方式启动
uxrce_dds_client start -t serial -d /dev/ttyS1 -b 921600

# UDP方式启动  
uxrce_dds_client start -t udp -h 192.168.1.100 -p 8888

# 检查状态
uxrce_dds_client status
```

## 第二步：任务计算机端配置

### 2.1 安装ROS2 Humble

```bash
# Ubuntu 22.04系统
sudo apt update && sudo apt install curl gnupg lsb-release

# 添加ROS2官方源
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS2
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete
sudo apt install ros-dev-tools python3-colcon-common-extensions

# 配置环境变量
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.2 安装Micro-XRCE-DDS Agent

```bash
# 方法1：使用snap安装（推荐）
sudo snap install micro-xrce-dds-agent --devmode

# 方法2：从源码编译
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/
```

### 2.3 安装PX4消息定义

```bash
# 创建ROS2工作空间
mkdir -p ~/px4_ros2_ws/src
cd ~/px4_ros2_ws/src

# 克隆PX4消息包
git clone https://github.com/PX4/px4_msgs.git

# 编译工作空间
cd ~/px4_ros2_ws
colcon build --packages-select px4_msgs
source install/setup.bash

# 添加到环境变量
echo "source ~/px4_ros2_ws/install/setup.bash" >> ~/.bashrc
```

## 第三步：建立连接

### 3.1 硬件连接

**串口连接（推荐）：**
```
PX4 TELEM2 ←→ 任务计算机串口
GND     ←→ GND
TX      ←→ RX  
RX      ←→ TX
```

**USB连接：**
```
PX4 USB ←→ 任务计算机USB端口
```

### 3.2 启动Micro-XRCE-DDS Agent

```bash
# 串口连接（波特率921600）
micro-xrce-dds-agent serial --dev /dev/ttyUSB0 -b 921600

# 如果是树莓派GPIO串口
micro-xrce-dds-agent serial --dev /dev/ttyAMA0 -b 921600

# UDP连接
micro-xrce-dds-agent udp4 -p 8888

# 成功连接后应该看到类似输出：
# [1640335394.826742] info     | Root.cpp           | set_verbose_level | logger setup
# [1640335394.827503] info     | Agent.cpp          | create_client     | create client (sessionid: 0x01, transport: serial, type: XRCE_CLIENT)
```

## 第四步：验证连接

### 4.1 检查ROS2话题

在任务计算机上运行：

```bash
# 查看所有话题
ros2 topic list

# 应该看到PX4发布的话题：
# /fmu/out/vehicle_status
# /fmu/out/vehicle_attitude  
# /fmu/out/vehicle_local_position
# /fmu/out/sensor_combined
# 等等...

# 查看特定话题的消息
ros2 topic echo /fmu/out/vehicle_status

# 查看话题频率
ros2 topic hz /fmu/out/vehicle_attitude
```

### 4.2 发送测试命令

```bash
# 发送解锁命令
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand '{
  "timestamp": 0,
  "param1": 1.0,
  "param2": 0.0,
  "param3": 0.0,
  "param4": 0.0,
  "param5": 0.0,
  "param6": 0.0,
  "param7": 0.0,
  "command": 400,
  "target_system": 1,
  "target_component": 1,
  "source_system": 1,
  "source_component": 1,
  "from_external": true
}'
```

## 第五步：编写ROS2控制节点

### 5.1 简单的状态监控节点

创建文件 `px4_listener.py`：

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus, VehicleAttitude, VehicleLocalPosition

class PX4Listener(Node):
    def __init__(self):
        super().__init__('px4_listener')
        
        # 订阅飞行器状态
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_callback,
            10)
            
        # 订阅姿态信息
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            10)
            
        # 订阅位置信息
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            10)
    
    def status_callback(self, msg):
        # 飞行模式
        nav_state_map = {
            0: "MANUAL",
            1: "ALTCTL", 
            2: "POSCTL",
            3: "AUTO_MISSION",
            4: "AUTO_LOITER",
            5: "AUTO_RTL",
            14: "OFFBOARD"
        }
        mode = nav_state_map.get(msg.nav_state, f"UNKNOWN({msg.nav_state})")
        
        arming_state = "ARMED" if msg.arming_state == 2 else "DISARMED"
        
        self.get_logger().info(f'状态: {arming_state}, 模式: {mode}')
    
    def attitude_callback(self, msg):
        # 四元数转欧拉角（简化版）
        import math
        
        # Roll
        roll = math.atan2(2.0 * (msg.q[0] * msg.q[1] + msg.q[2] * msg.q[3]),
                         1.0 - 2.0 * (msg.q[1] * msg.q[1] + msg.q[2] * msg.q[2]))
        
        # Pitch  
        pitch = math.asin(2.0 * (msg.q[0] * msg.q[2] - msg.q[3] * msg.q[1]))
        
        # Yaw
        yaw = math.atan2(2.0 * (msg.q[0] * msg.q[3] + msg.q[1] * msg.q[2]),
                        1.0 - 2.0 * (msg.q[2] * msg.q[2] + msg.q[3] * msg.q[3]))
        
        self.get_logger().info(f'姿态: Roll={math.degrees(roll):.1f}°, '
                              f'Pitch={math.degrees(pitch):.1f}°, '
                              f'Yaw={math.degrees(yaw):.1f}°')
    
    def position_callback(self, msg):
        self.get_logger().info(f'位置: X={msg.x:.2f}m, Y={msg.y:.2f}m, Z={msg.z:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    
    px4_listener = PX4Listener()
    
    try:
        rclpy.spin(px4_listener)
    except KeyboardInterrupt:
        pass
    
    px4_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

运行监听节点：

```bash
python3 px4_listener.py
```
