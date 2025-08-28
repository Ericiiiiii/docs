# PX4飞控与任务计算机通信交互使用指南

## 概述

本文档详细介绍PX4飞控系统与任务计算机（Companion Computer）之间的通信方式、协议配置和使用方法。PX4主要通过ROS/ROS2生态系统与任务计算机进行通信，同时也支持直接的MAVLink协议通信。

## 主要通信方式

### 1. ROS2通信（推荐方式）

ROS2是PX4与任务计算机通信的主要和推荐方式，通过uXRCE-DDS中间件实现。

#### 1.1 uXRCE-DDS桥接
PX4通过uXRCE-DDS客户端将内部uORB消息桥接到ROS2网络：
- **uORB → ROS2**: 飞控状态数据发布到ROS2话题
- **ROS2 → uORB**: ROS2控制命令转换为uORB消息
- **实时性**: 低延迟通信，适合控制应用
- **类型安全**: 自动生成的消息类型定义

#### 1.2 主要ROS2话题

**状态发布话题（/fmu/out/）:**
```
/fmu/out/vehicle_status              # 飞行器状态
/fmu/out/vehicle_attitude            # 姿态信息
/fmu/out/vehicle_local_position      # 本地位置
/fmu/out/vehicle_global_position     # 全球位置
/fmu/out/sensor_combined             # 传感器数据
/fmu/out/battery_status              # 电池状态
/fmu/out/vehicle_gps_position        # GPS位置
/fmu/out/vehicle_land_detected       # 着陆检测
```

**控制订阅话题（/fmu/in/）:**
```
/fmu/in/trajectory_setpoint          # 轨迹设定点
/fmu/in/vehicle_command              # 飞行器命令
/fmu/in/offboard_control_mode        # 离线控制模式
/fmu/in/vehicle_rates_setpoint       # 角速率设定点
/fmu/in/vehicle_attitude_setpoint    # 姿态设定点
```

#### 1.3 ROS2环境配置

**安装依赖:**
```bash
# Ubuntu 22.04 + ROS2 Humble
sudo apt install ros-humble-desktop
sudo apt install ros-humble-px4-msgs
sudo apt install python3-colcon-common-extensions

# 配置环境
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**启动uXRCE-DDS代理:**
```bash
# 在任务计算机上启动代理
MicroXRCEAgent udp4 -p 8888

# 或使用串口连接
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

**PX4端配置:**
```bash
# 启动uXRCE-DDS客户端
uxrce_dds_client start -t udp -p 8888 -h 192.168.1.100

# 或使用串口
uxrce_dds_client start -t serial -d /dev/ttyS1 -b 921600
```

### 2. ROS1通信（通过MAVROS）

对于使用ROS1的系统，通过MAVROS包实现MAVLink到ROS的桥接。

#### 2.1 MAVROS配置

**安装MAVROS:**
```bash
# ROS Noetic
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

**启动MAVROS:**
```bash
# USB连接
roslaunch mavros px4.launch fcu_url:="/dev/ttyUSB0:57600"

# UDP连接
roslaunch mavros px4.launch fcu_url:="udp://:14540@14557"
```

#### 2.2 主要MAVROS话题

**状态话题:**
```
/mavros/state                    # 飞行器状态
/mavros/local_position/pose      # 本地位置和姿态
/mavros/global_position/global   # 全球位置
/mavros/imu/data                 # IMU数据
/mavros/battery                  # 电池状态
```

**控制话题:**
```
/mavros/setpoint_position/local     # 本地位置设定点
/mavros/setpoint_attitude/attitude  # 姿态设定点
/mavros/setpoint_raw/local          # 原始设定点
```

### 3. 直接MAVLink协议通信（底层方式）

当不使用ROS时，可以直接使用MAVLink协议与PX4通信。

#### 1.1 MAVLink特性
- **协议版本**: 支持MAVLink v1.0和v2.0
- **消息类型**: 200+ 种标准消息类型
- **通信方式**: 双向异步通信
- **数据格式**: 二进制协议，高效传输
- **校验机制**: CRC校验确保数据完整性

#### 1.2 常用MAVLink消息

**状态监控消息:**
- `HEARTBEAT`: 系统心跳和基本状态
- `SYS_STATUS`: 系统状态详细信息
- `ATTITUDE`: 姿态角度信息
- `GLOBAL_POSITION_INT`: GPS全局位置
- `LOCAL_POSITION_NED`: 本地位置（NED坐标系）
- `VFR_HUD`: 飞行显示数据
- `GPS_RAW_INT`: 原始GPS数据
- `SCALED_IMU`: 缩放后的IMU数据
- `BATTERY_STATUS`: 电池状态

**控制命令消息:**
- `SET_POSITION_TARGET_LOCAL_NED`: 设置本地位置目标
- `SET_POSITION_TARGET_GLOBAL_INT`: 设置全球位置目标
- `SET_ATTITUDE_TARGET`: 设置姿态目标
- `MANUAL_CONTROL`: 手动控制输入
- `COMMAND_LONG`: 长命令格式
- `MISSION_ITEM_INT`: 任务航点

#### 1.3 MAVLink配置

**串口配置示例:**
```bash
# 启动MAVLink实例（TELEM1端口，波特率57600）
mavlink start -d /dev/ttyS1 -b 57600 -m onboard -r 80000

# 启动MAVLink实例（TELEM2端口，波特率921600）
mavlink start -d /dev/ttyS2 -b 921600 -m onboard -r 80000
```

**UDP配置示例:**
```bash
# UDP模式（适用于WiFi连接）
mavlink start -d /dev/ttyACM0 -b 57600 -m onboard -r 40000
mavlink start -x -u 14556 -r 40000 -m onboard
```

### 2. uORB消息系统

uORB是PX4内部的轻量级发布/订阅消息系统，也可用于外部应用程序集成。

#### 2.1 uORB特性
- **发布/订阅模式**: 松耦合的消息传递
- **类型安全**: 强类型消息定义
- **实时性能**: 低延迟消息传递
- **内存效率**: 零拷贝消息传递

#### 2.2 主要uORB消息类型

**传感器数据:**
- `sensor_accel`: 加速度计数据
- `sensor_gyro`: 陀螺仪数据
- `sensor_mag`: 磁力计数据
- `sensor_baro`: 气压计数据
- `sensor_gps`: GPS数据

**车辆状态:**
- `vehicle_status`: 车辆状态
- `vehicle_attitude`: 车辆姿态
- `vehicle_local_position`: 本地位置
- `vehicle_global_position`: 全球位置

**控制消息:**
- `offboard_control_mode`: 离线控制模式
- `trajectory_setpoint`: 轨迹设定点
- `vehicle_command`: 车辆命令

#### 2.3 外部应用uORB访问

```cpp
// C++示例：订阅vehicle_attitude消息
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>

int attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
struct vehicle_attitude_s attitude;

// 非阻塞读取
bool updated;
orb_check(attitude_sub, &updated);
if (updated) {
    orb_copy(ORB_ID(vehicle_attitude), attitude_sub, &attitude);
    // 处理姿态数据
}
```

### 3. 串口通信配置

#### 3.1 Fihawk FC-V1串口配置

根据`boards/fihawk/fc-v1/src/board_config.h`，Fihawk FC-V1支持多个串口：

```c
// 遥控输入串口
#define RC_SERIAL_PORT          "/dev/ttyS5"
#define RC_SERIAL_SINGLEWIRE    // 单线模式

// UART映射（参考其他PX4板型）:
// UART1: /dev/ttyS0 - DEBUG (控制台)
// UART2: /dev/ttyS1 - TELEM1 (任务计算机通信)
// UART3: /dev/ttyS2 - TELEM2 (地面站通信)
// UART4: /dev/ttyS3 - GPS1
// UART5: /dev/ttyS4 - GPS2
// UART6: /dev/ttyS5 - RC输入
// UART7: /dev/ttyS6 - 扩展串口
// UART8: /dev/ttyS7 - 扩展串口
```

#### 3.2 串口参数配置

通过参数系统配置串口行为：

```bash
# 设置TELEM1波特率
param set SER_TEL1_BAUD 921600

# 设置TELEM2波特率  
param set SER_TEL2_BAUD 57600

# 设置GPS波特率
param set SER_GPS1_BAUD 115200
```

## ROS2离线控制实现

### 4.1 基于ROS2的离线控制

使用ROS2进行离线控制是最推荐的方式，提供了类型安全和高性能的通信。

#### 4.1.1 ROS2离线控制步骤

**1. 创建ROS2节点:**
```cpp
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        // 发布者
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
            
        // 订阅者
        vehicle_local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", 10,
            std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));
            
        // 定时器
        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    void timer_callback()
    {
        // 发布离线控制模式
        publish_offboard_control_mode();
        // 发布轨迹设定点
        publish_trajectory_setpoint();
    }
};
```

**2. 发布控制消息:**
```cpp
void OffboardControl::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint()
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -5.0}; // NED坐标系，向上5米
    msg.yaw = 0.0; // 偏航角
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}
```

**3. 模式切换:**
```cpp
void OffboardControl::publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

// 切换到离线模式
void arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

void disarm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}
```

#### 4.1.2 Python ROS2示例

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control')

        # 发布者
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # 订阅者
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, 10)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, 10)

        # 定时器
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

    def vehicle_local_position_callback(self, vehicle_local_position):
        # 处理位置数据
        pass

    def vehicle_status_callback(self, vehicle_status):
        self.nav_state = vehicle_status.nav_state

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=-5.0, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw  # [-PI:PI]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        # 持续发布离线控制模式
        self.publish_offboard_control_mode()

        # 发布轨迹设定点
        self.publish_trajectory_setpoint()

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()

    # 执行飞行序列
    import time
    time.sleep(2)  # 等待连接建立
    offboard_control.arm()
    time.sleep(2)
    offboard_control.engage_offboard_mode()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
