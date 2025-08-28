# PX4 UAVCAN 自定义协议数据添加指南

## 概述

本指南详细介绍如何在PX4的UAVCAN系统中添加自己的协议数据，包括创建自定义DSDL消息类型、编译集成和使用方法。

## 方法一：创建自定义DSDL消息类型 (推荐)

### 1. 创建自定义命名空间

在UAVCAN DSDL目录中创建你的自定义命名空间：

```bash
# 创建自定义命名空间目录
mkdir -p src/drivers/uavcan/libuavcan/dsdl/mycompany
mkdir -p src/drivers/uavcan/libuavcan/dsdl/mycompany/equipment
mkdir -p src/drivers/uavcan/libuavcan/dsdl/mycompany/sensors
```

### 2. 定义自定义消息类型

#### 示例1: 自定义传感器数据

**文件**: `src/drivers/uavcan/libuavcan/dsdl/mycompany/sensors/20100.CustomSensorData.uavcan`

```
#
# 自定义传感器数据消息
# 消息ID: 20100 (在20000-21000范围内为厂商自定义)
#

# 时间戳 (微秒)
uint64 timestamp

# 传感器ID
uint8 sensor_id

# 传感器状态
uint8 STATUS_OK = 0
uint8 STATUS_WARNING = 1
uint8 STATUS_ERROR = 2
uint8 status

# 传感器数据
float32 temperature    # 温度 (摄氏度)
float32 pressure      # 压力 (Pa)
float32 humidity      # 湿度 (%)

# 原始数据 (可选)
uint16[<=10] raw_data

# 校验和
uint16 checksum
```

#### 示例2: 自定义控制命令

**文件**: `src/drivers/uavcan/libuavcan/dsdl/mycompany/equipment/20101.CustomCommand.uavcan`

```
#
# 自定义设备控制命令
# 消息ID: 20101
#

# 命令类型
uint8 CMD_RESET = 0
uint8 CMD_CALIBRATE = 1
uint8 CMD_SET_MODE = 2
uint8 CMD_GET_STATUS = 3
uint8 command_type

# 目标设备ID
uint8 device_id

# 命令参数
float32[<=4] parameters

# 命令序列号
uint16 sequence_number
```

#### 示例3: 自定义服务请求/响应

**文件**: `src/drivers/uavcan/libuavcan/dsdl/mycompany/equipment/200.CustomService.uavcan`

```
#
# 自定义服务 - 获取设备配置
# 服务ID: 200 (在200-255范围内为厂商自定义)
#

# 请求
uint8 device_id
uint8 config_type

---

# 响应
uint8 STATUS_SUCCESS = 0
uint8 STATUS_DEVICE_NOT_FOUND = 1
uint8 STATUS_INVALID_CONFIG = 2
uint8 status

# 配置数据
uint8[<=64] config_data
```

### 3. 修改构建配置

#### 更新CMakeLists.txt

**文件**: `src/drivers/uavcan/CMakeLists.txt`

<augment_code_snippet path="src/drivers/uavcan/CMakeLists.txt" mode="EXCERPT">
```cmake
# generated DSDL
set(DSDLC_DIR "${PX4_SOURCE_DIR}/src/drivers/uavcan/dsdl")
set(DSDLC_INPUTS
	"${LIBUAVCAN_DIR}/dsdl/ardupilot"
	"${LIBUAVCAN_DIR}/dsdl/com"
	"${LIBUAVCAN_DIR}/dsdl/cuav"
	"${LIBUAVCAN_DIR}/dsdl/dronecan"
	"${LIBUAVCAN_DIR}/dsdl/uavcan"
	"${LIBUAVCAN_DIR}/dsdl/mycompany"  # 添加自定义命名空间
)
```
</augment_code_snippet>

### 4. 创建C++接口类

#### 自定义传感器数据发布者

**文件**: `src/drivers/uavcan/sensors/custom_sensor.hpp`

```cpp
#pragma once

#include <uavcan/uavcan.hpp>
#include <mycompany/sensors/CustomSensorData.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_custom.h>

class UavcanCustomSensorBridge
{
public:
    static const char *const NAME;

    UavcanCustomSensorBridge(uavcan::INode &node);

    const char *get_name() const { return NAME; }

    int init();

private:
    void custom_sensor_sub_cb(const uavcan::ReceivedDataStructure<mycompany::sensors::CustomSensorData> &msg);

    typedef uavcan::MethodBinder<UavcanCustomSensorBridge *,
        void (UavcanCustomSensorBridge::*)(const uavcan::ReceivedDataStructure<mycompany::sensors::CustomSensorData>&)>
        CustomSensorCbBinder;

    uavcan::INode &_node;
    uavcan::Subscriber<mycompany::sensors::CustomSensorData, CustomSensorCbBinder> _sub_custom_sensor;
    uORB::PublicationMulti<sensor_custom_s> _sensor_custom_pub{ORB_ID(sensor_custom)};
};
```

**文件**: `src/drivers/uavcan/sensors/custom_sensor.cpp`

```cpp
#include "custom_sensor.hpp"
#include <drivers/drv_hrt.h>

const char *const UavcanCustomSensorBridge::NAME = "custom_sensor";

UavcanCustomSensorBridge::UavcanCustomSensorBridge(uavcan::INode &node) :
    _node(node),
    _sub_custom_sensor(node)
{
}

int UavcanCustomSensorBridge::init()
{
    int res = _sub_custom_sensor.start(CustomSensorCbBinder(this, &UavcanCustomSensorBridge::custom_sensor_sub_cb));

    if (res < 0) {
        PX4_ERR("failed to start uavcan sub: %d", res);
        return res;
    }

    return 0;
}

void UavcanCustomSensorBridge::custom_sensor_sub_cb(const uavcan::ReceivedDataStructure<mycompany::sensors::CustomSensorData> &msg)
{
    sensor_custom_s sensor_custom{};

    sensor_custom.timestamp = hrt_absolute_time();
    sensor_custom.sensor_id = msg.sensor_id;
    sensor_custom.status = msg.status;
    sensor_custom.temperature = msg.temperature;
    sensor_custom.pressure = msg.pressure;
    sensor_custom.humidity = msg.humidity;
    sensor_custom.checksum = msg.checksum;

    // 复制原始数据
    for (size_t i = 0; i < msg.raw_data.size() && i < sizeof(sensor_custom.raw_data)/sizeof(sensor_custom.raw_data[0]); i++) {
        sensor_custom.raw_data[i] = msg.raw_data[i];
    }

    _sensor_custom_pub.publish(sensor_custom);
}
```

#### 自定义命令发布者

**文件**: `src/drivers/uavcan/actuators/custom_command.hpp`

```cpp
#pragma once

#include <uavcan/uavcan.hpp>
#include <mycompany/equipment/CustomCommand.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/custom_command.h>

class UavcanCustomCommandController
{
public:
    UavcanCustomCommandController(uavcan::INode &node);

    int init();
    void update();

private:
    uavcan::INode &_node;
    uavcan::Publisher<mycompany::equipment::CustomCommand> _uavcan_pub_custom_cmd;
    uORB::Subscription _custom_cmd_sub{ORB_ID(custom_command)};
    
    uint16_t _sequence_number{0};
};
```

**文件**: `src/drivers/uavcan/actuators/custom_command.cpp`

```cpp
#include "custom_command.hpp"

UavcanCustomCommandController::UavcanCustomCommandController(uavcan::INode &node) :
    _node(node),
    _uavcan_pub_custom_cmd(node)
{
}

int UavcanCustomCommandController::init()
{
    int res = _uavcan_pub_custom_cmd.init();
    if (res < 0) {
        PX4_ERR("failed to init uavcan custom command pub: %d", res);
        return res;
    }

    return 0;
}

void UavcanCustomCommandController::update()
{
    custom_command_s custom_cmd;

    if (_custom_cmd_sub.update(&custom_cmd)) {
        mycompany::equipment::CustomCommand msg;

        msg.command_type = custom_cmd.command_type;
        msg.device_id = custom_cmd.device_id;
        msg.sequence_number = ++_sequence_number;

        // 复制参数
        for (size_t i = 0; i < sizeof(custom_cmd.parameters)/sizeof(custom_cmd.parameters[0]) && 
             i < msg.parameters.capacity(); i++) {
            msg.parameters.push_back(custom_cmd.parameters[i]);
        }

        _uavcan_pub_custom_cmd.broadcast(msg);
    }
}
```

### 5. 集成到主UAVCAN驱动

#### 更新传感器桥接器

**文件**: `src/drivers/uavcan/sensors/sensor_bridge.hpp`

在类定义中添加：
```cpp
#include "custom_sensor.hpp"

class UavcanSensorBridge
{
    // ... 现有代码 ...
    
private:
    // ... 现有传感器 ...
    UavcanCustomSensorBridge _custom_sensor;
};
```

**文件**: `src/drivers/uavcan/sensors/sensor_bridge.cpp`

在构造函数中添加：
```cpp
UavcanSensorBridge::UavcanSensorBridge(uavcan::INode &node) :
    // ... 现有初始化 ...
    _custom_sensor(node)
{
}
```

在init()函数中添加：
```cpp
int UavcanSensorBridge::init()
{
    // ... 现有初始化代码 ...
    
    int res = _custom_sensor.init();
    if (res < 0) {
        PX4_ERR("failed to init custom sensor bridge: %d", res);
        return res;
    }
    
    return 0;
}
```

#### 更新主UAVCAN节点

**文件**: `src/drivers/uavcan/uavcan_main.hpp`

在类定义中添加：
```cpp
#include "actuators/custom_command.hpp"

class UavcanNode
{
    // ... 现有代码 ...
    
private:
    UavcanCustomCommandController _custom_command_controller;
};
```

**文件**: `src/drivers/uavcan/uavcan_main.cpp`

在构造函数中添加：
```cpp
UavcanNode::UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock) :
    // ... 现有初始化 ...
    _custom_command_controller(_node)
{
}
```

在Run()函数中添加：
```cpp
void UavcanNode::Run()
{
    // ... 现有代码 ...
    
    // 更新自定义命令控制器
    _custom_command_controller.update();
}
```

### 6. 创建uORB消息定义

#### 自定义传感器消息

**文件**: `msg/sensor_custom.msg`

```
# 自定义传感器数据

uint64 timestamp        # 时间戳 [us]

uint8 sensor_id         # 传感器ID
uint8 status           # 传感器状态

float32 temperature    # 温度 [°C]
float32 pressure      # 压力 [Pa]  
float32 humidity      # 湿度 [%]

uint16[10] raw_data   # 原始数据
uint16 checksum       # 校验和
```

#### 自定义命令消息

**文件**: `msg/custom_command.msg`

```
# 自定义设备控制命令

uint64 timestamp        # 时间戳 [us]

uint8 command_type     # 命令类型
uint8 device_id        # 目标设备ID

float32[4] parameters  # 命令参数
```

### 7. 编译和测试

#### 编译系统

```bash
# 清理之前的构建
make clean

# 重新编译 (会自动运行DSDL编译器)
make px4_fmu-v5_default

# 或者只编译UAVCAN模块
make px4_fmu-v5_default uavcan
```

#### 验证DSDL编译

```bash
# 检查生成的头文件
ls build/px4_fmu-v5_default/src/drivers/uavcan/include/dsdlc_generated/mycompany/

# 应该看到类似以下文件:
# sensors/
#   CustomSensorData.hpp
# equipment/
#   CustomCommand.hpp
#   CustomService.hpp
```

#### 测试自定义消息

```bash
# 启动PX4
make px4_fmu-v5_default jmavsim

# 在PX4控制台中
uavcan status

# 监听自定义消息
listener sensor_custom
listener custom_command

# 发送测试命令
custom_command test
```

### 8. Python测试脚本

**文件**: `test_custom_uavcan.py`

```python
#!/usr/bin/env python3

import dronecan
import time

# 连接到UAVCAN网络
node = dronecan.make_node('/dev/ttyUSB0', node_id=100, bitrate=1000000)

def send_custom_sensor_data():
    """发送自定义传感器数据"""
    msg = dronecan.mycompany.sensors.CustomSensorData()
    
    msg.timestamp = int(time.time() * 1000000)  # 微秒时间戳
    msg.sensor_id = 1
    msg.status = 0  # STATUS_OK
    msg.temperature = 25.5
    msg.pressure = 101325.0
    msg.humidity = 60.0
    msg.raw_data = [100, 200, 300, 400, 500]
    msg.checksum = 0x1234
    
    node.broadcast(msg)
    print(f"发送传感器数据: 温度={msg.temperature}°C")

def custom_command_callback(msg):
    """接收自定义命令回调"""
    print(f"收到自定义命令: 类型={msg.command_type}, 设备ID={msg.device_id}")
    print(f"参数: {list(msg.parameters)}")

# 注册回调
node.add_handler(dronecan.mycompany.equipment.CustomCommand, custom_command_callback)

# 定期发送数据
node.periodic(1.0, send_custom_sensor_data)  # 1Hz

print("开始UAVCAN自定义协议测试...")
try:
    node.spin()
except KeyboardInterrupt:
    print("测试结束")
```

## 方法二：使用隧道协议 (简单快速)

如果你不想创建新的DSDL类型，可以使用UAVCAN的隧道协议：

### 使用Tunnel.Broadcast消息

```cpp
#include <uavcan/tunnel/Broadcast.hpp>

void send_custom_data_via_tunnel()
{
    uavcan::tunnel::Broadcast msg;
    
    // 设置协议ID (自定义)
    msg.protocol.protocol = 100;  // 你的自定义协议ID
    msg.channel_id = 1;
    
    // 打包你的数据
    struct CustomData {
        float temperature;
        float pressure;
        uint16_t status;
    } data = {25.5f, 101325.0f, 0x01};
    
    // 复制到消息缓冲区
    msg.buffer.resize(sizeof(data));
    memcpy(msg.buffer.data(), &data, sizeof(data));
    
    // 发送
    _tunnel_pub.broadcast(msg);
}
```

## 总结

1. **推荐方法**: 创建自定义DSDL消息类型，提供类型安全和标准化
2. **快速方法**: 使用隧道协议，适合原型开发和简单数据传输
3. **关键步骤**: DSDL定义 → 编译集成 → C++接口 → uORB集成 → 测试验证

选择适合你需求的方法，自定义DSDL提供更好的结构化和类型安全，而隧道协议更加灵活快速。
