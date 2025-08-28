# PX4串口框架概览

## 简介

PX4串口框架是一个跨平台的串口通信抽象层，为飞控系统提供统一的串口访问接口。

## 核心架构

```
应用层 (GPS, MAVLink, RC等驱动)
    ↓
PX4串口抽象层 (Serial.hpp)
    ↓
平台实现层 (NuttX/POSIX/QURT)
    ↓
操作系统/硬件层
```

## 主要组件

### 1. Serial类
- **位置**: `platforms/common/include/px4_platform_common/Serial.hpp`
- **功能**: 提供统一的串口操作接口
- **主要方法**:
  - `open()` / `close()`: 打开/关闭串口
  - `read()` / `write()`: 数据读写
  - `setBaudrate()`: 设置波特率
  - `setFlowcontrol()`: 设置流控制

### 2. 平台实现
- **NuttX**: `platforms/nuttx/src/px4/common/SerialImpl.cpp`
- **POSIX**: `platforms/posix/src/px4/common/SerialImpl.cpp`
- **QURT**: `platforms/qurt/src/px4/SerialImpl.cpp`

### 3. 配置系统
- **板级配置**: `boards/*/default.px4board`
- **硬件映射**: `boards/*/nuttx-config/include/board.h`
- **参数生成**: `Tools/serial/generate_config.py`

## 串口配置流程

### 1. 板级定义
```bash
# 在 default.px4board 中定义
CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS0"
CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS1"
CONFIG_BOARD_SERIAL_TEL2="/dev/ttyS3"
```

### 2. 硬件映射
```c
// 在 board.h 中定义引脚
#define GPIO_USART1_RX   GPIO_USART1_RX_3      /* PB7 */
#define GPIO_USART1_TX   GPIO_USART1_TX_3      /* PB6 */
```

### 3. 参数配置
```bash
# 在 rc.board_defaults 中设置默认参数
param set-default SER_GPS1_BAUD 115200
param set-default MAV_0_CONFIG 101
```

## 常用串口分配

| 端口 | 功能 | 典型波特率 | 流控 |
|------|------|-----------|------|
| ttyS0 | GPS1 | 115200 | 无 |
| ttyS1 | TELEM1 | 57600 | RTS/CTS |
| ttyS2 | GPS2 | 115200 | 无 |
| ttyS3 | TELEM2 | 921600 | RTS/CTS |
| ttyS4 | DEBUG | 57600 | 无 |
| ttyS5 | RC | 变量 | 无 |

## 驱动程序集成

### 模块配置 (module.yaml)
```yaml
module_name: GPS
serial_config:
    - command: gps start -d ${SERIAL_DEV} -b p:${BAUD_PARAM}
      port_config_param:
        name: GPS_${i}_CONFIG
        group: GPS
      num_instances: 2
```

### 驱动实现示例
```cpp
class GPSDriver : public px4::ScheduledWorkItem {
private:
    device::Serial _uart;
    
public:
    GPSDriver(const char *port, uint32_t baudrate) :
        ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
        _uart(port, baudrate) {}
    
    int init() {
        if (!_uart.open()) {
            return -1;
        }
        ScheduleNow();
        return 0;
    }
    
    void Run() override {
        uint8_t buffer[256];
        ssize_t bytes = _uart.read(buffer, sizeof(buffer));
        if (bytes > 0) {
            process_data(buffer, bytes);
        }
        ScheduleDelayed(10_ms);
    }
};
```

## 参数系统

### 自动生成的参数
- **波特率参数**: `SER_GPS1_BAUD`, `SER_TEL1_BAUD` 等
- **功能配置**: `MAV_0_CONFIG`, `GPS_1_CONFIG` 等

### 参数值含义
- `0`: 禁用
- `101-104`: TELEM 1-4
- `201-203`: GPS 1-3
- `300`: RC端口

## 使用示例

### 简单串口通信
```cpp
#include <px4_platform_common/Serial.hpp>

void example() {
    device::Serial uart("/dev/ttyS1", 115200);
    
    if (uart.open()) {
        const char *msg = "Hello\n";
        uart.write(msg, strlen(msg));
        
        uint8_t buffer[64];
        ssize_t bytes = uart.read(buffer, sizeof(buffer));
        
        uart.close();
    }
}
```

### MAVLink通信
```cpp
class MAVLinkSerial {
private:
    device::Serial _uart;
    
public:
    MAVLinkSerial(const char *port) : _uart(port, 57600) {
        _uart.setFlowcontrol(device::SerialConfig::FlowControl::Enabled);
    }
    
    bool init() {
        return _uart.open();
    }
    
    void send_heartbeat() {
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_QUADROTOR, 
                                  MAV_AUTOPILOT_PX4, 0, 0, MAV_STATE_ACTIVE);
        
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
        _uart.write(buffer, len);
    }
};
```

## 调试技巧

### 检查串口状态
```bash
# 查看可用串口
ls -la /dev/ttyS*

# 检查参数配置
param show SER_*
param show MAV_*_CONFIG

# 监控串口活动
listener serial_status
```

### 常见问题
1. **串口无法打开**: 检查设备节点和权限
2. **数据乱码**: 检查波特率和数据格式配置
3. **性能问题**: 优化工作队列和缓冲区大小

## 最佳实践

1. **选择合适的工作队列**: 根据串口用途选择对应的工作队列
2. **错误处理**: 实现串口连接恢复机制
3. **缓冲区优化**: 使用适当大小的读写缓冲区
4. **资源管理**: 避免重复打开同一串口
5. **配置验证**: 在使用前验证串口配置的有效性

## 扩展开发

### 添加新串口
1. 在 `default.px4board` 中添加串口定义
2. 在 `board.h` 中配置引脚映射
3. 在 `rc.board_defaults` 中设置默认参数
4. 更新 `generate_config.py` 中的串口列表

### 开发新驱动
1. 创建 `module.yaml` 配置文件
2. 实现串口驱动类
3. 集成到工作队列系统
4. 添加参数和启动脚本

---

更多详细信息请参考 [PX4串口框架完整文档](PX4_Serial_Framework_Complete.md)。
