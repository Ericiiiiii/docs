# PX4 UAVCAN 正确实现指南

## 概述

### 1. 继承UavcanSensorBridgeBase

所有UAVCAN传感器都应该继承`UavcanSensorBridgeBase`类：

```cpp
class UavcanFiHawkFlightDataBridge : public UavcanSensorBridgeBase
{
public:
    static const char *const NAME;

    UavcanFiHawkFlightDataBridge(uavcan::INode &node);

    const char *get_name() const override { return NAME; }
    int init() override;

private:
    int init_driver(uavcan_bridge::Channel *channel) override;
    void flight_data_sub_cb(const uavcan::ReceivedDataStructure<fihawk::sensors::FlightData> &msg);

    // 订阅器
    uavcan::Subscriber<fihawk::sensors::FlightData, FlightDataCbBinder> _sub_flight_data;
};
```

### 2. 实现关键方法

#### 构造函数
```cpp
UavcanFiHawkFlightDataBridge::UavcanFiHawkFlightDataBridge(uavcan::INode &node) :
    UavcanSensorBridgeBase("uavcan_fihawk_flight_data", ORB_ID(sensor_mag)), // 需要创建自定义uORB话题
    _sub_flight_data(node)
{
}
```

#### init()方法
```cpp
int UavcanFiHawkFlightDataBridge::init()
{
    int res = _sub_flight_data.start(FlightDataCbBinder(this, &UavcanFiHawkFlightDataBridge::flight_data_sub_cb));

    if (res < 0) {
        PX4_ERR("failed to start fihawk flight data sub: %d", res);
        return res;
    }

    return 0;
}
```

#### 消息回调
```cpp
void UavcanFiHawkFlightDataBridge::flight_data_sub_cb(const uavcan::ReceivedDataStructure<fihawk::sensors::FlightData> &msg)
{
    // 创建uORB消息
    fihawk_flight_data_s orb_msg{};
    orb_msg.timestamp = hrt_absolute_time();
    orb_msg.flight_mode = msg.flight_mode;
    orb_msg.system_status = msg.system_status;
    orb_msg.custom_altitude = msg.custom_altitude;
    // ... 填充其他字段

    // 发布到uORB (自动处理多通道)
    publish(msg.getSrcNodeID().get(), &orb_msg);
}
```

### 3. 注册到传感器工厂

在`sensor_bridge.cpp`的`make_all()`函数中添加：

```cpp
void IUavcanSensorBridge::make_all(uavcan::INode &node, List<IUavcanSensorBridge *> &list)
{
    // ... 现有传感器 ...

    // FiHawk自定义传感器
#if defined(CONFIG_UAVCAN_SENSOR_FIHAWK)
    int32_t uavcan_sub_fihawk = 1;
    param_get(param_find("UAVCAN_SUB_FIHAWK"), &uavcan_sub_fihawk);

    if (uavcan_sub_fihawk != 0) {
        list.add(new UavcanFiHawkFlightDataBridge(node));
    }
#endif
}
```

### 4. 创建uORB消息定义

**文件**: `msg/fihawk_flight_data.msg`

```
# FiHawk自定义飞行数据

uint64 timestamp        # 时间戳 [us]

uint8 flight_mode      # 飞行模式
uint8 system_status    # 系统状态

float32 custom_altitude    # 自定义高度 [m]
float32 custom_velocity    # 自定义速度 [m/s]
float32 wind_speed        # 风速 [m/s]
float32 wind_direction    # 风向 [度]

float32 battery_voltage   # 电池电压 [V]
float32 battery_current   # 电池电流 [A]
uint8 battery_percentage  # 电池百分比 [%]

float32 motor_temperature # 电机温度 [°C]
float32 esc_temperature   # 电调温度 [°C]

bool custom_sensor_valid  # 自定义传感器有效
bool wind_data_valid     # 风速数据有效
bool temperature_valid   # 温度数据有效

uint16 error_count       # 错误计数
uint16[8] raw_sensor_data # 原始传感器数据
```

### 5. 添加配置参数

**文件**: `src/drivers/uavcan/uavcan_params.c`

```c
/**
 * Enable FiHawk flight data subscription
 *
 * @boolean
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_SUB_FIHAWK, 0);
```

### 6. 更新构建配置

#### CMakeLists.txt
```cmake
# 在UAVCAN驱动的SRCS中添加
SRCS
    # ... 现有文件 ...
    sensors/fihawk_flight_data.cpp
```

#### Kconfig (如果需要)
```kconfig
config UAVCAN_SENSOR_FIHAWK
    bool "Enable FiHawk flight data sensor"
    default n
    help
      Enable FiHawk custom flight data sensor bridge
```

## 与现有传感器的对比

### 磁力计实现 (参考)

<augment_code_snippet path="src/drivers/uavcan/sensors/mag.hpp" mode="EXCERPT">
```cpp
class UavcanMagnetometerBridge : public UavcanSensorBridgeBase
{
public:
    static const char *const NAME;

    UavcanMagnetometerBridge(uavcan::INode &node);

    const char *get_name() const override { return NAME; }
    int init() override;

private:
    int init_driver(uavcan_bridge::Channel *channel) override;
    void mag_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::ahrs::MagneticFieldStrength> &msg);
```
</augment_code_snippet>

### 关键差异

1. **继承关系**: 必须继承`UavcanSensorBridgeBase`
2. **自动注册**: 通过`make_all()`函数自动创建和注册
3. **多通道支持**: 基类自动处理多个相同类型传感器
4. **uORB集成**: 使用`publish()`方法自动发布到uORB

## 完整的集成步骤

### 步骤1: 创建DSDL定义
```bash
# 已完成
src/drivers/uavcan/libuavcan/dsdl/fihawk/sensors/20100.FlightData.uavcan
```

### 步骤2: 创建uORB消息
```bash
# 创建消息定义
msg/fihawk_flight_data.msg
```

### 步骤3: 实现传感器桥接器
```bash
# 已修正
src/drivers/uavcan/sensors/fihawk_flight_data.hpp
src/drivers/uavcan/sensors/fihawk_flight_data.cpp
```

### 步骤4: 注册传感器
编辑`src/drivers/uavcan/sensors/sensor_bridge.cpp`:

```cpp
#include "fihawk_flight_data.hpp"

void IUavcanSensorBridge::make_all(uavcan::INode &node, List<IUavcanSensorBridge *> &list)
{
    // ... 现有代码 ...

    // FiHawk flight data
    int32_t uavcan_sub_fihawk = 1;
    param_get(param_find("UAVCAN_SUB_FIHAWK"), &uavcan_sub_fihawk);

    if (uavcan_sub_fihawk != 0) {
        list.add(new UavcanFiHawkFlightDataBridge(node));
    }
}
```

### 步骤5: 更新构建配置
编辑`src/drivers/uavcan/CMakeLists.txt`:

```cmake
# 添加DSDL命名空间
set(DSDLC_INPUTS
    # ... 现有命名空间 ...
    "${LIBUAVCAN_DIR}/dsdl/fihawk"
)

# 添加源文件
SRCS
    # ... 现有文件 ...
    sensors/fihawk_flight_data.cpp
```

### 步骤6: 编译和测试
```bash
# 编译
make px4_fmu-v5_default

# 测试
param set UAVCAN_SUB_FIHAWK 1
param save
reboot

# 检查传感器
uavcan status
listener fihawk_flight_data
```

## 优势

### 1. 标准化
- 遵循PX4 UAVCAN的标准模式
- 与现有传感器实现一致
- 易于维护和扩展

### 2. 自动化
- 自动多通道支持
- 自动设备管理
- 自动uORB发布

### 3. 集成性
- 无需修改核心代码
- 通过参数控制启用/禁用
- 与现有系统无缝集成

## 总结

正确的实现方式是：

1. ✅ **继承UavcanSensorBridgeBase** (不是修改传感器桥接器)
2. ✅ **在make_all()中注册** (自动发现和创建)
3. ✅ **使用publish()方法** (自动uORB发布)
4. ✅ **创建对应的uORB消息** (标准化数据接口)

这样实现的好处是完全符合PX4的架构设计，易于维护，并且可以自动处理多传感器实例等复杂情况。

