# PX4 MAVLink 自定义消息完整实现指南

## 概述

本文档详细介绍了如何在PX4中添加自定义MAVLink消息的完整流程，包括消息定义、代码实现、构建配置、测试验证等所有步骤。

## 目录

1. [MAVLink自定义消息定义](#1-mavlink自定义消息定义)
2. [PX4构建系统配置](#2-px4构建系统配置)
3. [消息流实现](#3-消息流实现)
4. [系统集成](#4-系统集成)
5. [启动配置](#5-启动配置)
6. [测试验证](#6-测试验证)
7. [故障排除](#7-故障排除)
8. [最佳实践](#8-最佳实践)

## 1. MAVLink自定义消息定义

### 1.1 创建自定义Dialect文件

在 `src/modules/mavlink/mavlink/message_definitions/v1.0/` 目录下创建自定义dialect文件：

**文件：`custom_example.xml`**

```xml
<?xml version="1.0"?>
<mavlink>
  <include>common.xml</include>
  <version>1</version>
  <dialect>0</dialect>

  <!-- 自定义枚举定义 -->
  <enums>
    <enum name="CUSTOM_SENSOR_TYPE">
      <description>自定义传感器类型</description>
      <entry value="0" name="CUSTOM_SENSOR_TYPE_TEMPERATURE">
        <description>温度传感器</description>
      </entry>
      <entry value="1" name="CUSTOM_SENSOR_TYPE_HUMIDITY">
        <description>湿度传感器</description>
      </entry>
      <entry value="2" name="CUSTOM_SENSOR_TYPE_PRESSURE">
        <description>压力传感器</description>
      </entry>
      <entry value="3" name="CUSTOM_SENSOR_TYPE_LIGHT">
        <description>光照传感器</description>
      </entry>
    </enum>

    <enum name="CUSTOM_DEVICE_STATUS">
      <description>自定义设备状态</description>
      <entry value="0" name="CUSTOM_DEVICE_STATUS_OFFLINE">
        <description>设备离线</description>
      </entry>
      <entry value="1" name="CUSTOM_DEVICE_STATUS_ONLINE">
        <description>设备在线</description>
      </entry>
      <entry value="2" name="CUSTOM_DEVICE_STATUS_ERROR">
        <description>设备错误</description>
      </entry>
      <entry value="3" name="CUSTOM_DEVICE_STATUS_CALIBRATING">
        <description>设备校准中</description>
      </entry>
    </enum>
  </enums>

  <!-- 自定义消息定义 -->
  <messages>
    <!-- 自定义传感器数据消息 -->
    <message id="50000" name="CUSTOM_SENSOR_DATA">
      <description>自定义传感器数据消息，用于传输多种传感器的数据</description>
      <field type="uint64_t" name="timestamp" units="us">时间戳 (微秒)</field>
      <field type="uint8_t" name="sensor_id">传感器ID</field>
      <field type="uint8_t" name="sensor_type" enum="CUSTOM_SENSOR_TYPE">传感器类型</field>
      <field type="float" name="value1" units="varies">传感器数值1</field>
      <field type="float" name="value2" units="varies" invalid="NaN">传感器数值2</field>
      <field type="float" name="value3" units="varies" invalid="NaN">传感器数值3</field>
      <field type="uint8_t" name="status" enum="CUSTOM_DEVICE_STATUS">传感器状态</field>
      <field type="uint8_t" name="quality" units="%">数据质量 (0-100%)</field>
    </message>

    <!-- 自定义状态报告消息 -->
    <message id="50002" name="CUSTOM_STATUS_REPORT">
      <description>自定义状态报告消息，用于报告设备或系统的详细状态</description>
      <field type="uint32_t" name="time_boot_ms" units="ms">系统启动时间 (毫秒)</field>
      <field type="uint8_t" name="device_id">设备ID</field>
      <field type="uint8_t" name="status" enum="CUSTOM_DEVICE_STATUS">设备状态</field>
      <field type="uint16_t" name="error_code">错误代码 (0表示无错误)</field>
      <field type="float" name="temperature" units="degC" invalid="NaN">设备温度</field>
      <field type="float" name="voltage" units="V" invalid="NaN">供电电压</field>
      <field type="float" name="current" units="A" invalid="NaN">电流消耗</field>
      <field type="uint32_t" name="uptime" units="s">设备运行时间 (秒)</field>
      <field type="char[32]" name="version">固件版本字符串</field>
    </message>
  </messages>
</mavlink>
```

### 1.2 消息ID分配原则

- **50000-65535**: 自定义消息ID范围
- **避免冲突**: 不要使用已被标准消息占用的ID
- **文档记录**: 为每个自定义消息ID建立使用记录

## 2. PX4构建系统配置

### 2.1 修改CMakeLists.txt

**文件：`src/modules/mavlink/CMakeLists.txt`**

添加自定义dialect的生成规则：

```cmake
# 生成自定义dialect
set(MAVLINK_DIALECT_CUSTOM "custom_example")
add_custom_command(
    OUTPUT ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_CUSTOM}/${MAVLINK_DIALECT_CUSTOM}.h
    COMMAND
        ${PYTHON_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
            --lang C --wire-protocol 2.0
            --output ${MAVLINK_LIBRARY_DIR}
            ${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT_CUSTOM}.xml
    DEPENDS
        git_mavlink_v2
        ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
        ${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT_CUSTOM}.xml
    COMMENT "Generating Mavlink ${MAVLINK_DIALECT_CUSTOM}"
)
add_custom_target(mavlink_c_generate_custom DEPENDS ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_CUSTOM}/${MAVLINK_DIALECT_CUSTOM}.h)

# 添加到主要生成目标的依赖中
# 在主dialect生成命令的DEPENDS中添加：
# mavlink_c_generate_custom
# ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_CUSTOM}/${MAVLINK_DIALECT_CUSTOM}.h

# 添加到target_sources和target_include_directories中
target_sources(mavlink_c
    INTERFACE
        # ... 现有文件 ...
        ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_CUSTOM}/${MAVLINK_DIALECT_CUSTOM}.h
)
target_include_directories(mavlink_c
    INTERFACE
        # ... 现有目录 ...
        ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_CUSTOM}
)
```

### 2.2 修改MAVLink桥接头文件

**文件：`src/modules/mavlink/mavlink_bridge_header.h`**

在文件末尾添加：

```cpp
// 包含自定义dialect
#include <custom_example/mavlink.h>
```

## 3. 消息流实现

### 3.1 创建自定义传感器数据流

**文件：`src/modules/mavlink/streams/CUSTOM_SENSOR_DATA.hpp`**

```cpp
#ifndef CUSTOM_SENSOR_DATA_HPP
#define CUSTOM_SENSOR_DATA_HPP

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_local_position.h>

class MavlinkStreamCustomSensorData : public MavlinkStream {
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) {
        return new MavlinkStreamCustomSensorData(mavlink);
    }

    static constexpr const char *get_name_static() { return "CUSTOM_SENSOR_DATA"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA; }

    const char *get_name() const override { return get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }

    unsigned get_size() override {
        return MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    explicit MavlinkStreamCustomSensorData(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _sensor_combined_sub{ORB_ID(sensor_combined)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

    bool send() override {
        sensor_combined_s sensor_combined;
        vehicle_status_s vehicle_status;
        battery_status_s battery_status;
        vehicle_local_position_s local_pos;

        // 获取最新数据
        bool sensor_updated = _sensor_combined_sub.update(&sensor_combined);
        bool status_updated = _vehicle_status_sub.copy(&vehicle_status);
        bool battery_updated = _battery_status_sub.copy(&battery_status);
        bool pos_updated = _vehicle_local_position_sub.copy(&local_pos);

        if (sensor_updated || status_updated || battery_updated || pos_updated) {
            mavlink_custom_sensor_data_t msg{};

            // 填充消息字段
            msg.timestamp = hrt_absolute_time();
            msg.sensor_id = 1;
            msg.sensor_type = CUSTOM_SENSOR_TYPE_TEMPERATURE;

            // 使用不同的数据源
            if (sensor_updated) {
                msg.value1 = sensor_combined.accelerometer_m_s2[0];
                msg.value2 = sensor_combined.accelerometer_m_s2[1];
                msg.value3 = sensor_combined.accelerometer_m_s2[2];
            } else if (battery_updated) {
                msg.value1 = battery_status.voltage_v;
                msg.value2 = battery_status.current_a;
                msg.value3 = battery_status.remaining;
                msg.sensor_type = CUSTOM_SENSOR_TYPE_PRESSURE;
            }

            // 设置状态
            if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
                msg.status = CUSTOM_DEVICE_STATUS_ONLINE;
                msg.quality = 95;
            } else {
                msg.status = CUSTOM_DEVICE_STATUS_OFFLINE;
                msg.quality = 50;
            }

            mavlink_msg_custom_sensor_data_send_struct(_mavlink->get_channel(), &msg);
            return true;
        }

        return false;
    }
};

#endif // CUSTOM_SENSOR_DATA_HPP
```

### 3.2 创建自定义状态报告流

**文件：`src/modules/mavlink/streams/CUSTOM_STATUS_REPORT.hpp`**

```cpp
#ifndef CUSTOM_STATUS_REPORT_HPP
#define CUSTOM_STATUS_REPORT_HPP

#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/vehicle_status.h>

class MavlinkStreamCustomStatusReport : public MavlinkStream {
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) {
        return new MavlinkStreamCustomStatusReport(mavlink);
    }

    static constexpr const char *get_name_static() { return "CUSTOM_STATUS_REPORT"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CUSTOM_STATUS_REPORT; }

    const char *get_name() const override { return get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }

    unsigned get_size() override {
        return MAVLINK_MSG_ID_CUSTOM_STATUS_REPORT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    explicit MavlinkStreamCustomStatusReport(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
    uORB::Subscription _cpuload_sub{ORB_ID(cpuload)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

    bool send() override {
        battery_status_s battery;
        cpuload_s cpuload;
        vehicle_status_s vehicle_status;

        bool battery_updated = _battery_status_sub.copy(&battery);
        bool cpu_updated = _cpuload_sub.copy(&cpuload);
        bool status_updated = _vehicle_status_sub.copy(&vehicle_status);

        if (battery_updated || cpu_updated || status_updated) {
            mavlink_custom_status_report_t msg{};

            msg.time_boot_ms = hrt_absolute_time() / 1000;
            msg.device_id = 1;

            // 设备状态
            if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
                msg.status = CUSTOM_DEVICE_STATUS_ONLINE;
                msg.error_code = 0;
            } else {
                msg.status = CUSTOM_DEVICE_STATUS_OFFLINE;
                msg.error_code = 100;
            }

            // 温度信息（使用CPU负载模拟）
            if (cpu_updated) {
                msg.temperature = 25.0f + cpuload.load * 50.0f;
            } else {
                msg.temperature = NAN;
            }

            // 电源信息
            if (battery_updated) {
                msg.voltage = battery.voltage_v;
                msg.current = battery.current_a;
            } else {
                msg.voltage = NAN;
                msg.current = NAN;
            }

            msg.uptime = hrt_absolute_time() / 1000000;
            strncpy(msg.version, "PX4-Custom-1.0.0", sizeof(msg.version) - 1);
            msg.version[sizeof(msg.version) - 1] = '\0';

            mavlink_msg_custom_status_report_send_struct(_mavlink->get_channel(), &msg);
            return true;
        }

        return false;
    }
};

#endif // CUSTOM_STATUS_REPORT_HPP
```

## 4. 系统集成

### 4.1 注册消息流

**文件：`src/modules/mavlink/mavlink_messages.cpp`**

在文件顶部添加包含：

```cpp
// 自定义消息流
#ifdef MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA
#include "streams/CUSTOM_SENSOR_DATA.hpp"
#endif
#ifdef MAVLINK_MSG_ID_CUSTOM_STATUS_REPORT
#include "streams/CUSTOM_STATUS_REPORT.hpp"
#endif
```

在 `streams_list` 数组中添加：

```cpp
#if defined(CUSTOM_SENSOR_DATA_HPP)
    create_stream_list_item<MavlinkStreamCustomSensorData>(),
#endif
#if defined(CUSTOM_STATUS_REPORT_HPP)
    create_stream_list_item<MavlinkStreamCustomStatusReport>(),
#endif
```

### 4.2 消息接收处理（可选）

**文件：`src/modules/mavlink/mavlink_receiver.cpp`**

在 `handle_message` 函数中添加：

```cpp
case MAVLINK_MSG_ID_CUSTOM_CONTROL_COMMAND:
    handle_message_custom_control_command(msg);
    break;
```

添加处理函数：

```cpp
void MavlinkReceiver::handle_message_custom_control_command(mavlink_message_t *msg) {
    mavlink_custom_control_command_t custom_cmd;
    mavlink_msg_custom_control_command_decode(msg, &custom_cmd);

    // 检查目标系统和组件ID
    if (custom_cmd.target_system != mavlink_system.sysid ||
        (custom_cmd.target_component != mavlink_system.compid &&
         custom_cmd.target_component != MAV_COMP_ID_ALL)) {
        return;
    }

    // 处理自定义命令
    switch (custom_cmd.command_id) {
        case 1001: // LED控制
            handle_led_control(custom_cmd.param1, custom_cmd.param2, custom_cmd.param3);
            break;
        case 1002: // 传感器配置
            handle_sensor_config(custom_cmd.param1);
            break;
        default:
            PX4_WARN("Unknown custom command ID: %d", custom_cmd.command_id);
            break;
    }
}
```

## 5. 启动配置

### 5.1 修改启动脚本

**文件：`ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`**

添加自定义消息流配置：

```bash
# 自定义消息流
mavlink stream -r 5 -s CUSTOM_SENSOR_DATA -u $udp_gcs_port_local
mavlink stream -r 2 -s CUSTOM_STATUS_REPORT -u $udp_gcs_port_local
```

### 5.2 参数配置

可以通过参数系统配置MAVLink行为：

```bash
# 设置MAVLink参数
param set MAV_0_MODE 0      # Normal模式
param set MAV_0_RATE 100000 # 数据速率
param set MAV_SYS_ID 1      # 系统ID
```

## 6. 测试验证

### 6.1 Python测试脚本

**文件：`test_custom_mavlink.py`**

```python
#!/usr/bin/env python3
"""测试自定义MAVLink消息的脚本"""

import time
from pymavlink import mavutil

def test_custom_messages(connection_string):
    print(f"连接到 {connection_string}...")

    try:
        master = mavutil.mavlink_connection(connection_string)
        master.wait_heartbeat()
        print(f"连接成功! 系统ID: {master.target_system}")
    except Exception as e:
        print(f"连接失败: {e}")
        return False

    print("开始监听自定义消息...")

    custom_sensor_count = 0
    custom_status_count = 0

    try:
        while True:
            msg = master.recv_match(blocking=False)

            if msg is not None:
                msg_type = msg.get_type()

                if msg_type == 'CUSTOM_SENSOR_DATA':
                    custom_sensor_count += 1
                    print(f"[{custom_sensor_count}] 自定义传感器数据:")
                    print(f"  时间戳: {msg.timestamp}")
                    print(f"  传感器ID: {msg.sensor_id}")
                    print(f"  传感器类型: {msg.sensor_type}")
                    print(f"  数值: {msg.value1:.3f}, {msg.value2:.3f}, {msg.value3:.3f}")
                    print(f"  状态: {msg.status}, 质量: {msg.quality}%")

                elif msg_type == 'CUSTOM_STATUS_REPORT':
                    custom_status_count += 1
                    print(f"[{custom_status_count}] 自定义状态报告:")
                    print(f"  设备ID: {msg.device_id}")
                    print(f"  状态: {msg.status}")
                    print(f"  温度: {msg.temperature:.1f}°C")
                    print(f"  电压: {msg.voltage:.2f}V")
                    print(f"  版本: {msg.version}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n测试被用户中断")
    finally:
        master.close()

if __name__ == '__main__':
    test_custom_messages('udp:localhost:14550')
```

### 6.2 编译和运行

```bash
# 生成MAVLink头文件
cd src/modules/mavlink/mavlink
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 \
    --output=../../../build/px4_sitl_default/mavlink/ \
    message_definitions/v1.0/custom_example.xml

# 编译PX4
make px4_sitl_default

# 启动仿真
make px4_sitl jmavsim

# 运行测试脚本
python3 test_custom_mavlink.py
```

### 6.3 验证步骤

1. **检查编译输出**：确认自定义消息头文件生成
2. **检查启动日志**：确认MAVLink实例启动正常
3. **验证消息流**：使用 `mavlink status streams` 检查配置
4. **测试连接**：确认地面站可以连接并接收消息
5. **数据验证**：确认自定义消息数据正确

## 7. 故障排除

### 7.1 常见问题

**问题1：消息流未找到**
```
WARN [mavlink] stream CUSTOM_SENSOR_DATA not found
```

**解决方案：**
- 检查消息流是否正确注册到 `mavlink_messages.cpp`
- 确认头文件包含路径正确
- 验证消息ID定义正确

**问题2：编译错误**
```
error: 'MAVLINK_MSG_ID_CUSTOM_SENSOR_DATA' was not declared
```

**解决方案：**
- 确认自定义dialect头文件生成
- 检查 `mavlink_bridge_header.h` 包含正确
- 验证CMakeLists.txt配置

**问题3：消息接收失败**

**解决方案：**
- 检查消息ID范围（50000-65535）
- 确认消息结构定义正确
- 验证地面站支持自定义消息

### 7.2 调试技巧

```bash
# 检查MAVLink状态
mavlink status

# 查看消息流配置
mavlink status streams

# 监控特定消息
listener custom_sensor_data

# 检查参数设置
param show MAV_*
```

## 8. 最佳实践

### 8.1 消息设计原则

1. **消息ID管理**：使用50000-65535范围，建立ID分配表
2. **向后兼容**：新增字段使用 `<extensions/>` 标签
3. **数据类型选择**：根据精度需求选择合适的数据类型
4. **字段验证**：使用 `invalid` 属性标记无效值

### 8.2 性能优化

1. **消息频率**：根据实际需求设置合理的发送频率
2. **数据压缩**：对于大数据量消息考虑压缩
3. **条件发送**：只在数据更新时发送消息
4. **带宽管理**：监控总体MAVLink带宽使用

### 8.3 维护建议

1. **版本控制**：为自定义dialect建立版本管理
2. **文档更新**：及时更新消息定义文档
3. **测试覆盖**：为每个自定义消息编写测试用例
4. **兼容性测试**：确保与不同地面站的兼容性

## 9. 参考资源

- [MAVLink官方文档](https://mavlink.io/en/)
- [PX4开发指南](https://docs.px4.io/main/en/development/)
- [pymavlink文档](https://github.com/ArduPilot/pymavlink)
- [MAVLink消息定义规范](https://mavlink.io/en/guide/define_xml_element.html)

## 10. 附录

### 10.1 完整文件列表

本实现涉及的所有文件：

```
src/modules/mavlink/mavlink/message_definitions/v1.0/custom_example.xml
src/modules/mavlink/CMakeLists.txt
src/modules/mavlink/mavlink_bridge_header.h
src/modules/mavlink/streams/CUSTOM_SENSOR_DATA.hpp
src/modules/mavlink/streams/CUSTOM_STATUS_REPORT.hpp
src/modules/mavlink/mavlink_messages.cpp
src/modules/mavlink/mavlink_receiver.cpp (可选)
ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
test_custom_mavlink.py
```

### 10.2 消息ID分配表

| 消息ID | 消息名称 | 用途 | 频率 |
|--------|----------|------|------|
| 50000 | CUSTOM_SENSOR_DATA | 传感器数据 | 5Hz |
| 50001 | CUSTOM_CONTROL_COMMAND | 控制命令 | 按需 |
| 50002 | CUSTOM_STATUS_REPORT | 状态报告 | 2Hz |
| 50003 | CUSTOM_CONFIG_SET | 配置设置 | 按需 |
| 50004 | CUSTOM_CONFIG_ACK | 配置确认 | 按需 |

---

**文档版本**: 1.0
**最后更新**: 2024-06-30
**作者**: PX4开发团队
