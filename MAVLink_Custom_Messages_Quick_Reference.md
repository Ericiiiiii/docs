# PX4 MAVLink 自定义消息快速参考

## 快速开始

### 1. 创建消息定义 (5分钟)

**文件**: `src/modules/mavlink/mavlink/message_definitions/v1.0/custom_example.xml`

```xml
<?xml version="1.0"?>
<mavlink>
  <include>common.xml</include>
  <version>1</version>
  <dialect>0</dialect>
  
  <messages>
    <message id="50000" name="MY_CUSTOM_MESSAGE">
      <description>我的自定义消息</description>
      <field type="uint64_t" name="timestamp" units="us">时间戳</field>
      <field type="float" name="data1">数据1</field>
      <field type="float" name="data2">数据2</field>
      <field type="uint8_t" name="status">状态</field>
    </message>
  </messages>
</mavlink>
```

### 2. 修改构建配置 (2分钟)

**文件**: `src/modules/mavlink/CMakeLists.txt`

在适当位置添加：

```cmake
# 生成自定义dialect
set(MAVLINK_DIALECT_CUSTOM "custom_example")
add_custom_command(
    OUTPUT ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_CUSTOM}/${MAVLINK_DIALECT_CUSTOM}.h
    COMMAND ${PYTHON_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
        --lang C --wire-protocol 2.0 --output ${MAVLINK_LIBRARY_DIR}
        ${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT_CUSTOM}.xml
    DEPENDS git_mavlink_v2 ${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT_CUSTOM}.xml
)
add_custom_target(mavlink_c_generate_custom DEPENDS ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_CUSTOM}/${MAVLINK_DIALECT_CUSTOM}.h)

# 添加到主要依赖和包含中
target_sources(mavlink_c INTERFACE ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_CUSTOM}/${MAVLINK_DIALECT_CUSTOM}.h)
target_include_directories(mavlink_c INTERFACE ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_CUSTOM})
```

**文件**: `src/modules/mavlink/mavlink_bridge_header.h`

在文件末尾添加：

```cpp
#include <custom_example/mavlink.h>
```

### 3. 创建消息流 (3分钟)

**文件**: `src/modules/mavlink/streams/MY_CUSTOM_MESSAGE.hpp`

```cpp
#ifndef MY_CUSTOM_MESSAGE_HPP
#define MY_CUSTOM_MESSAGE_HPP

#include <uORB/topics/sensor_combined.h>

class MavlinkStreamMyCustomMessage : public MavlinkStream {
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) { 
        return new MavlinkStreamMyCustomMessage(mavlink); 
    }
    static constexpr const char *get_name_static() { return "MY_CUSTOM_MESSAGE"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MY_CUSTOM_MESSAGE; }
    
    const char *get_name() const override { return get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }
    unsigned get_size() override {
        return MAVLINK_MSG_ID_MY_CUSTOM_MESSAGE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    explicit MavlinkStreamMyCustomMessage(Mavlink *mavlink) : MavlinkStream(mavlink) {}
    uORB::Subscription _sensor_sub{ORB_ID(sensor_combined)};

    bool send() override {
        sensor_combined_s sensor;
        if (_sensor_sub.update(&sensor)) {
            mavlink_my_custom_message_t msg{};
            msg.timestamp = sensor.timestamp;
            msg.data1 = sensor.accelerometer_m_s2[0];
            msg.data2 = sensor.accelerometer_m_s2[1];
            msg.status = 1;
            
            mavlink_msg_my_custom_message_send_struct(_mavlink->get_channel(), &msg);
            return true;
        }
        return false;
    }
};

#endif
```

### 4. 注册消息流 (1分钟)

**文件**: `src/modules/mavlink/mavlink_messages.cpp`

添加包含：
```cpp
#ifdef MAVLINK_MSG_ID_MY_CUSTOM_MESSAGE
#include "streams/MY_CUSTOM_MESSAGE.hpp"
#endif
```

在streams_list数组中添加：
```cpp
#if defined(MY_CUSTOM_MESSAGE_HPP)
    create_stream_list_item<MavlinkStreamMyCustomMessage>(),
#endif
```

### 5. 配置启动脚本 (1分钟)

**文件**: `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`

添加：
```bash
mavlink stream -r 10 -s MY_CUSTOM_MESSAGE -u $udp_gcs_port_local
```

### 6. 编译和测试 (2分钟)

```bash
# 生成头文件
cd src/modules/mavlink/mavlink
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 \
    --output=../../../build/px4_sitl_default/mavlink/ \
    message_definitions/v1.0/custom_example.xml

# 编译
make px4_sitl_default

# 启动仿真
make px4_sitl jmavsim
```

## 常用命令

### MAVLink调试命令

```bash
# 查看MAVLink状态
mavlink status

# 查看消息流
mavlink status streams

# 配置消息流
mavlink stream -u 18570 -s MY_CUSTOM_MESSAGE -r 10

# 监听消息
listener my_custom_message
```

### Python测试脚本

```python
from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:localhost:14550')
master.wait_heartbeat()

while True:
    msg = master.recv_match(type='MY_CUSTOM_MESSAGE', blocking=True)
    if msg:
        print(f"收到自定义消息: {msg.data1}, {msg.data2}")
```

## 常见问题速查

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| `stream not found` | 消息流未注册 | 检查mavlink_messages.cpp |
| `was not declared` | 头文件未包含 | 检查mavlink_bridge_header.h |
| 编译失败 | 头文件未生成 | 运行mavgen生成头文件 |
| 消息接收失败 | ID冲突或格式错误 | 检查消息ID和结构定义 |

## 消息ID分配

- **标准消息**: 0-49999
- **自定义消息**: 50000-65535
- **推荐起始**: 50000

## 数据类型参考

| MAVLink类型 | C类型 | 大小 | 范围 |
|-------------|-------|------|------|
| uint8_t | uint8_t | 1字节 | 0-255 |
| uint16_t | uint16_t | 2字节 | 0-65535 |
| uint32_t | uint32_t | 4字节 | 0-4294967295 |
| uint64_t | uint64_t | 8字节 | 0-18446744073709551615 |
| int8_t | int8_t | 1字节 | -128到127 |
| int16_t | int16_t | 2字节 | -32768到32767 |
| int32_t | int32_t | 4字节 | -2147483648到2147483647 |
| float | float | 4字节 | IEEE 754单精度 |
| double | double | 8字节 | IEEE 754双精度 |
| char[n] | char[] | n字节 | 字符串 |

## 性能建议

- **低频消息**: 0.1-1 Hz (状态、配置)
- **中频消息**: 1-10 Hz (传感器数据)
- **高频消息**: 10-50 Hz (控制数据)
- **超高频**: >50 Hz (仅关键数据)

## 文件清单

完整实现需要修改的文件：

```
✓ src/modules/mavlink/mavlink/message_definitions/v1.0/custom_example.xml
✓ src/modules/mavlink/CMakeLists.txt
✓ src/modules/mavlink/mavlink_bridge_header.h
✓ src/modules/mavlink/streams/MY_CUSTOM_MESSAGE.hpp
✓ src/modules/mavlink/mavlink_messages.cpp
✓ ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
```

## 下一步

1. 阅读完整指南: `doc/PX4_MAVLink_Custom_Messages_Guide.md`
2. 查看MAVLink官方文档: https://mavlink.io/en/
3. 参考PX4开发文档: https://docs.px4.io/main/en/development/

---
**总用时**: ~15分钟  
**难度**: 中等  
**版本**: PX4 v1.14+
