# PX4 UAVCAN 自定义协议集成实例

## 概述

本文档提供了一个完整的实例，展示如何在PX4中添加名为"FiHawk"的自定义UAVCAN协议数据。

## 已创建的文件

### 1. DSDL消息定义

```
src/drivers/uavcan/libuavcan/dsdl/fihawk/
├── README.md
├── sensors/
│   └── 20100.FlightData.uavcan
└── equipment/
    ├── 20101.CustomCommand.uavcan
    └── 200.DeviceConfig.uavcan
```

### 2. C++实现

```
src/drivers/uavcan/sensors/
├── fihawk_flight_data.hpp
└── fihawk_flight_data.cpp
```

### 3. 测试脚本

```
test_fihawk_uavcan.py
```

## 集成步骤

### 步骤1: 修改构建配置

编辑 `src/drivers/uavcan/CMakeLists.txt`:

```cmake
# 在DSDLC_INPUTS中添加fihawk命名空间
set(DSDLC_INPUTS
    "${LIBUAVCAN_DIR}/dsdl/ardupilot"
    "${LIBUAVCAN_DIR}/dsdl/com"
    "${LIBUAVCAN_DIR}/dsdl/cuav"
    "${LIBUAVCAN_DIR}/dsdl/dronecan"
    "${LIBUAVCAN_DIR}/dsdl/uavcan"
    "${LIBUAVCAN_DIR}/dsdl/fihawk"  # 添加这一行
)
```

### 步骤2: 添加源文件到构建

在同一个CMakeLists.txt文件中，找到SRCS部分并添加：

```cmake
SRCS
    # ... 现有文件 ...
    
    # FiHawk自定义传感器
    sensors/fihawk_flight_data.cpp
```

### 步骤3: 集成到传感器桥接器

编辑 `src/drivers/uavcan/sensors/sensor_bridge.hpp`:

```cpp
#include "fihawk_flight_data.hpp"

class UavcanSensorBridge
{
    // ... 现有代码 ...
    
private:
    // ... 现有传感器 ...
    UavcanFiHawkFlightDataSubscriber _fihawk_flight_data;
};
```

编辑 `src/drivers/uavcan/sensors/sensor_bridge.cpp`:

```cpp
UavcanSensorBridge::UavcanSensorBridge(uavcan::INode &node) :
    // ... 现有初始化 ...
    _fihawk_flight_data(node)
{
}

int UavcanSensorBridge::init()
{
    // ... 现有初始化代码 ...
    
    int res = _fihawk_flight_data.init();
    if (res < 0) {
        PX4_ERR("failed to init fihawk flight data bridge: %d", res);
        return res;
    }
    
    return 0;
}
```

### 步骤4: 集成到主UAVCAN节点

编辑 `src/drivers/uavcan/uavcan_main.hpp`:

```cpp
#include "sensors/fihawk_flight_data.hpp"

class UavcanNode
{
    // ... 现有代码 ...
    
private:
    UavcanFiHawkFlightDataPublisher _fihawk_flight_data_pub;
};
```

编辑 `src/drivers/uavcan/uavcan_main.cpp`:

```cpp
UavcanNode::UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock) :
    // ... 现有初始化 ...
    _fihawk_flight_data_pub(_node)
{
}

int UavcanNode::init(uavcan::NodeID node_id, UAVCAN_DRIVER::BusEvent &bus_events)
{
    // ... 现有初始化代码 ...
    
    int res = _fihawk_flight_data_pub.init();
    if (res < 0) {
        PX4_ERR("failed to init fihawk flight data publisher: %d", res);
        return res;
    }
    
    return 0;
}

void UavcanNode::Run()
{
    // ... 现有代码 ...
    
    // 更新FiHawk飞行数据发布器
    _fihawk_flight_data_pub.update();
}
```

### 步骤5: 编译系统

```bash
# 清理构建
make clean

# 重新编译 (会自动运行DSDL编译器)
make px4_fmu-v5_default

# 检查生成的头文件
ls build/px4_fmu-v5_default/src/drivers/uavcan/include/dsdlc_generated/fihawk/
```

应该看到生成的文件：
```
sensors/
  FlightData.hpp
equipment/
  CustomCommand.hpp
  DeviceConfig.hpp
```

### 步骤6: 验证集成

#### 启动PX4并检查UAVCAN

```bash
# 启动仿真
make px4_fmu-v5_default jmavsim

# 在PX4控制台中
uavcan status
uavcan info
```

#### 使用Python脚本测试

```bash
# 安装依赖
pip3 install dronecan

# 运行测试脚本
python3 test_fihawk_uavcan.py --port /dev/ttyUSB0 --test-mode both
```

### 步骤7: 监控自定义消息

在PX4控制台中：

```bash
# 监控UAVCAN流量
uavcan monitor

# 查看日志中的FiHawk消息
dmesg | grep -i fihawk
```

## 消息格式说明

### FlightData (ID: 20100)

包含综合飞行状态信息：
- 飞行模式和系统状态
- 自定义高度和速度数据
- 风速风向信息
- 电池状态
- 温度数据
- 原始传感器数据

### CustomCommand (ID: 20101)

用于设备控制：
- 命令类型 (重置、校准、设置模式等)
- 目标设备ID
- 命令参数数组
- 执行选项和超时

### DeviceConfig (Service ID: 200)

设备配置服务：
- 读取/写入/重置配置
- 设备信息查询
- 配置数据传输

## 扩展指南

### 添加新消息类型

1. 在 `src/drivers/uavcan/libuavcan/dsdl/fihawk/` 中创建新的 `.uavcan` 文件
2. 选择合适的消息ID (避免冲突)
3. 创建对应的C++接口类
4. 集成到主UAVCAN驱动中
5. 重新编译和测试

### 消息ID分配建议

- **20100-20199**: 传感器和状态数据
- **20200-20299**: 控制命令
- **20300-20399**: 配置和管理
- **200-219**: 配置服务
- **220-239**: 数据传输服务

### 最佳实践

1. **命名约定**: 使用清晰的命名，包含厂商前缀
2. **版本控制**: 在消息中包含版本信息
3. **向后兼容**: 新版本应保持向后兼容
4. **文档更新**: 及时更新README和文档
5. **测试覆盖**: 为每个新消息创建测试用例

## 故障排除

### 常见问题

1. **编译错误**: 检查DSDL语法和CMakeLists.txt配置
2. **消息不发送**: 验证发布器初始化和update()调用
3. **消息不接收**: 检查订阅器回调注册
4. **ID冲突**: 确保消息ID在系统中唯一

### 调试技巧

```bash
# 查看DSDL编译输出
make px4_fmu-v5_default VERBOSE=1

# 监控CAN总线 (如果支持)
candump can0

# 查看UAVCAN统计
uavcan status -v
```

## 总结

通过这个实例，你已经学会了：

1. 如何定义自定义DSDL消息
2. 如何创建C++接口类
3. 如何集成到PX4的UAVCAN系统
4. 如何编译和测试自定义协议

这为你开发自己的UAVCAN协议数据提供了完整的模板和指导。
