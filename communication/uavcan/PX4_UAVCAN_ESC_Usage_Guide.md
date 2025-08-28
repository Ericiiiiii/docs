# PX4 UAVCAN ESC 使用指南

## 概述

本文档详细介绍如何在PX4中配置和使用UAVCAN协议与外部电调(ESC)进行通信。UAVCAN (现在也称为DroneCAN) 是一个基于CAN总线的通信协议，专门为无人机和机器人系统设计。

## 目录

1. [UAVCAN基本概念](#uavcan基本概念)
2. [硬件要求](#硬件要求)
3. [配置步骤](#配置步骤)
4. [参数设置](#参数设置)
5. [启动和验证](#启动和验证)
6. [故障排除](#故障排除)
7. [高级配置](#高级配置)
8. [示例代码](#示例代码)

## UAVCAN基本概念

### 什么是UAVCAN

UAVCAN是一个开源的轻量级协议，基于CAN总线，具有以下特点：

- **实时性**: 支持实时数据传输
- **可靠性**: 内置错误检测和恢复机制
- **可扩展性**: 支持多达127个节点
- **标准化**: 定义了标准的消息格式

### 支持的功能

- 传感器数据传输 (IMU, GPS, 气压计等)
- 执行器控制 (ESC, 舵机)
- 固件更新
- 参数配置
- 节点状态监控

## 硬件要求

### CAN总线连接

```
飞控板 ---- CAN_H ---- ESC1 ---- ESC2 ---- ESC3 ---- ESC4
     |                                                    |
     +---- CAN_L ---- ESC1 ---- ESC2 ---- ESC3 ---- ESC4 +
     |                                                    |
   120Ω                                                 120Ω
```

### 硬件清单

- 支持CAN总线的飞控板
- 支持UAVCAN协议的ESC
- CAN总线连接线
- 120Ω终端电阻 (总线两端各一个)

### 接线要求

1. **CAN_H和CAN_L信号线**: 使用双绞线，减少干扰
2. **终端电阻**: 在总线两端各放置一个120Ω电阻
3. **地线连接**: 所有设备必须共享地线
4. **电源**: 确保ESC有独立的电源供应

## 配置步骤

### 1. 启用UAVCAN

```bash
# 连接到PX4控制台
# 设置UAVCAN模式
param set UAVCAN_ENABLE 3

# 设置节点ID (1-125之间的唯一值)
param set UAVCAN_NODE_ID 1

# 设置CAN总线波特率 (通常为1Mbps)
param set UAVCAN_BITRATE 1000000

# 保存参数并重启
param save
reboot
```

### 2. 配置ESC功能映射

```bash
# 设置ESC 1-4为电机输出
param set UAVCAN_EC_FUNC1 101  # Motor 1
param set UAVCAN_EC_FUNC2 102  # Motor 2
param set UAVCAN_EC_FUNC3 103  # Motor 3
param set UAVCAN_EC_FUNC4 104  # Motor 4

# 设置ESC输出范围 (0-8191)
param set UAVCAN_EC_MIN1 1
param set UAVCAN_EC_MAX1 8191
param set UAVCAN_EC_MIN2 1
param set UAVCAN_EC_MAX2 8191
param set UAVCAN_EC_MIN3 1
param set UAVCAN_EC_MAX3 8191
param set UAVCAN_EC_MIN4 1
param set UAVCAN_EC_MAX4 8191

# 设置失效保护值
param set UAVCAN_EC_FAIL1 0
param set UAVCAN_EC_FAIL2 0
param set UAVCAN_EC_FAIL3 0
param set UAVCAN_EC_FAIL4 0
```

### 3. 配置ESC怠速设置 (可选)

```bash
# 启用ESC怠速旋转 (位掩码: 1=ESC1, 2=ESC2, 4=ESC3, 8=ESC4)
# 例如: 15 = 1+2+4+8 表示所有ESC都启用怠速
param set UAVCAN_ESC_IDLT 15
```

## 参数设置

### 核心参数

| 参数名 | 描述 | 取值范围 | 默认值 |
|--------|------|----------|--------|
| `UAVCAN_ENABLE` | UAVCAN模式 | 0-3 | 0 |
| `UAVCAN_NODE_ID` | 节点ID | 1-125 | 1 |
| `UAVCAN_BITRATE` | CAN波特率 | 20000-1000000 | 1000000 |

### UAVCAN_ENABLE 模式说明

- **0**: 禁用UAVCAN
- **1**: 仅传感器，手动配置
- **2**: 传感器，自动配置和固件更新
- **3**: 传感器和执行器，自动配置和固件更新

### ESC相关参数

| 参数前缀 | 描述 | 示例 |
|----------|------|------|
| `UAVCAN_EC_FUNC*` | ESC功能映射 | `UAVCAN_EC_FUNC1 = 101` |
| `UAVCAN_EC_MIN*` | ESC最小输出值 | `UAVCAN_EC_MIN1 = 1` |
| `UAVCAN_EC_MAX*` | ESC最大输出值 | `UAVCAN_EC_MAX1 = 8191` |
| `UAVCAN_EC_FAIL*` | ESC失效保护值 | `UAVCAN_EC_FAIL1 = 0` |

## 启动和验证

### 1. 检查UAVCAN状态

```bash
# 查看UAVCAN驱动状态
uavcan status

# 查看在线节点
uavcan info

# 查看节点详细信息
uavcan info <node_id>
```

### 2. 监控ESC状态

```bash
# 监听ESC状态消息
listener esc_status

# 查看执行器输出
listener actuator_outputs
```

### 3. 手动测试ESC

```bash
# 进入UAVCAN测试模式
uavcan test

# 或者使用执行器测试
actuator_test -m uavcan -v 1000
```

## 故障排除

### 常见问题

#### 1. ESC不响应

**症状**: ESC不转动，无状态反馈

**可能原因**:
- CAN总线连接问题
- ESC节点ID冲突
- 波特率不匹配
- 电源问题

**解决方法**:
```bash
# 检查CAN总线状态
uavcan status

# 扫描在线节点
uavcan info

# 检查参数设置
param show UAVCAN*
```

#### 2. CAN总线错误

**症状**: 大量CAN错误，通信不稳定

**可能原因**:
- 缺少终端电阻
- 信号完整性问题
- 电磁干扰

**解决方法**:
- 检查120Ω终端电阻
- 使用屏蔽双绞线
- 远离高功率设备

#### 3. 参数设置不生效

**症状**: 修改参数后ESC行为未改变

**解决方法**:
```bash
# 确保参数已保存
param save

# 重启系统
reboot

# 验证参数值
param show UAVCAN_ENABLE
```

### 调试命令

```bash
# 显示详细的UAVCAN调试信息
uavcan status -v

# 监控CAN总线流量
candump can0

# 检查系统日志
dmesg | grep -i uavcan
```

## 高级配置

### 1. 多CAN总线配置

如果硬件支持多个CAN接口:

```bash
# 启用第二个CAN接口
param set UAVCAN_ENABLE2 1
param set UAVCAN_BITRATE2 1000000
```

### 2. ESC参数配置

通过UAVCAN可以配置ESC内部参数:

```bash
# 获取ESC参数
uavcan param get <node_id> <param_name>

# 设置ESC参数
uavcan param set <node_id> <param_name> <value>
```

### 3. 固件更新

```bash
# 更新ESC固件
uavcan fw update <node_id> <firmware_file>
```

## 示例代码

### Python控制示例

```python
#!/usr/bin/env python3
import dronecan
import time
import math

# 初始化UAVCAN节点
node = dronecan.make_node('/dev/ttyUSB0', node_id=100, bitrate=1000000)

def publish_esc_command():
    # 生成正弦波控制信号
    setpoint = int(512 * (math.sin(time.time()) + 2))

    # 发送ESC命令 (控制ESC 0,1,2,3)
    commands = [setpoint, setpoint, setpoint, setpoint]
    message = dronecan.uavcan.equipment.esc.RawCommand(cmd=commands)
    node.broadcast(message)

    print(f"发送ESC命令: {commands}")

# 设置20Hz发送频率
node.periodic(0.05, publish_esc_command)

# 监听ESC状态
def esc_status_callback(msg):
    print(f"ESC {msg.esc_index}: RPM={msg.rpm}, 电压={msg.voltage}V")

node.add_handler(dronecan.uavcan.equipment.esc.Status, esc_status_callback)

# 运行节点
try:
    node.spin()
except KeyboardInterrupt:
    print("程序退出")
```

### C++控制示例

```cpp
#include <uavcan/equipment/esc/RawCommand.hpp>

void UavcanEscController::update_outputs(bool stop_motors,
                                       uint16_t outputs[MAX_ACTUATORS],
                                       unsigned num_outputs)
{
    uavcan::equipment::esc::RawCommand msg;

    if (stop_motors) {
        // 停止所有电机
        msg.cmd.resize(num_outputs);
        for (unsigned i = 0; i < num_outputs; i++) {
            msg.cmd[i] = 0;
        }
    } else {
        // 设置电机输出值
        msg.cmd.resize(num_outputs);
        for (unsigned i = 0; i < num_outputs; i++) {
            msg.cmd[i] = outputs[i];
        }
    }

    // 广播命令到CAN总线
    _uavcan_pub_raw_cmd.broadcast(msg);
}
```

## 总结

通过本指南，你应该能够成功配置PX4的UAVCAN功能并与外部ESC进行通信。记住以下关键点:

1. 正确的硬件连接是成功的基础
2. 参数配置需要重启后生效
3. 逐步测试，从简单到复杂
4. 充分的地面测试是安全飞行的前提

如果遇到问题，请参考故障排除章节或查看PX4社区文档获取更多帮助。

## 附录

### A. UAVCAN消息类型

#### ESC相关消息

| 消息类型 | 方向 | 描述 |
|----------|------|------|
| `uavcan.equipment.esc.RawCommand` | 发送 | ESC原始命令 |
| `uavcan.equipment.esc.RPMCommand` | 发送 | ESC转速命令 |
| `uavcan.equipment.esc.Status` | 接收 | ESC状态反馈 |

#### 安全相关消息

| 消息类型 | 方向 | 描述 |
|----------|------|------|
| `uavcan.equipment.safety.ArmingStatus` | 发送 | 解锁状态 |
| `ardupilot.indication.SafetyState` | 发送 | 安全状态 |

### B. 常用命令参考

#### 基本操作命令

```bash
# 启动UAVCAN
uavcan start

# 停止UAVCAN
uavcan stop

# 重启UAVCAN
uavcan restart

# 查看帮助
uavcan help
```

#### 参数操作命令

```bash
# 显示所有UAVCAN参数
param show UAVCAN*

# 重置UAVCAN参数为默认值
param reset UAVCAN*

# 导出参数到文件
param export /fs/microsd/params.txt

# 从文件导入参数
param import /fs/microsd/params.txt
```

#### 诊断命令

```bash
# 显示CAN接口统计
cat /proc/net/can/stats

# 显示CAN错误计数
cat /proc/net/can/can0

# 监控系统资源使用
top

# 查看内存使用
free
```

### C. ESC配置示例

#### 四旋翼配置

```bash
# 基本设置
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 1
param set UAVCAN_BITRATE 1000000

# ESC功能映射 (四旋翼X型)
param set UAVCAN_EC_FUNC1 101  # 前右电机
param set UAVCAN_EC_FUNC2 102  # 后左电机
param set UAVCAN_EC_FUNC3 103  # 前左电机
param set UAVCAN_EC_FUNC4 104  # 后右电机

# 输出范围设置
for i in {1..4}; do
    param set UAVCAN_EC_MIN$i 1
    param set UAVCAN_EC_MAX$i 8191
    param set UAVCAN_EC_FAIL$i 0
done

# 启用怠速 (所有电机)
param set UAVCAN_ESC_IDLT 15

param save
reboot
```

#### 六旋翼配置

```bash
# 基本设置
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 1
param set UAVCAN_BITRATE 1000000

# ESC功能映射 (六旋翼)
param set UAVCAN_EC_FUNC1 101  # 电机1
param set UAVCAN_EC_FUNC2 102  # 电机2
param set UAVCAN_EC_FUNC3 103  # 电机3
param set UAVCAN_EC_FUNC4 104  # 电机4
param set UAVCAN_EC_FUNC5 105  # 电机5
param set UAVCAN_EC_FUNC6 106  # 电机6

# 输出范围设置
for i in {1..6}; do
    param set UAVCAN_EC_MIN$i 1
    param set UAVCAN_EC_MAX$i 8191
    param set UAVCAN_EC_FAIL$i 0
done

# 启用怠速 (所有电机: 1+2+4+8+16+32=63)
param set UAVCAN_ESC_IDLT 63

param save
reboot
```

### D. 性能优化建议

#### 1. CAN总线优化

```bash
# 调整CAN接收队列大小
echo 1000 > /sys/class/net/can0/tx_queue_len

# 设置CAN接口优先级
ip link set can0 txqueuelen 1000
```

#### 2. 系统优化

```bash
# 调整调度器优先级
param set UAVCAN_SCHED_PRIO 250

# 设置更新频率 (Hz)
param set UAVCAN_ESC_RATE 400
```

#### 3. 内存优化

```bash
# 调整UAVCAN内存池大小
param set UAVCAN_POOL_SIZE 8192
```

### E. 安全注意事项

#### 飞行前检查清单

- [ ] 验证所有ESC都在线并响应
- [ ] 检查ESC状态消息正常接收
- [ ] 确认电机转向正确
- [ ] 测试失效保护功能
- [ ] 验证解锁/上锁功能正常
- [ ] 检查CAN总线错误计数为零

#### 紧急情况处理

```bash
# 紧急停止所有电机
actuator_test -m uavcan -v 0

# 禁用UAVCAN (需要重启)
param set UAVCAN_ENABLE 0
param save
reboot
```

### F. 兼容性列表

#### 支持的ESC品牌

- **Zubax**: Orel 20, Myxa
- **CUAV**: CAN ESC
- **Holybro**: Tekko32 CAN
- **T-Motor**: Alpha ESC CAN
- **KDE Direct**: UAS ESC

#### 支持的飞控板

- **Pixhawk系列**: 4, 5, 6
- **Cube系列**: Orange, Black, Purple
- **CUAV**: V5+, V5 nano
- **Holybro**: Durandal, Kakute H7

### G. 技术支持

#### 官方资源

- [PX4官方文档](https://docs.px4.io/)
- [UAVCAN规范](https://uavcan.org/)
- [DroneCAN文档](https://dronecan.github.io/)

#### 社区支持

- [PX4论坛](https://discuss.px4.io/)
- [GitHub Issues](https://github.com/PX4/PX4-Autopilot/issues)
- [Discord社区](https://discord.gg/dronecode)

#### 报告问题

提交问题时请包含以下信息:

1. 硬件配置详情
2. PX4版本信息
3. 完整的参数设置
4. 错误日志和状态输出
5. 复现步骤

---

**文档版本**: v1.0
**最后更新**: 2025-07-01
**适用PX4版本**: v1.14+
