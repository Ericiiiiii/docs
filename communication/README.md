# 通信协议文档

本目录包含PX4各种通信协议的详细实现文档，涵盖MAVLink、UAVCAN、ROS2等主要通信方式。

## 文档列表

### MAVLink协议
- **[PX4 MAVLink自定义消息指南](PX4_MAVLink_Custom_Messages_Guide.md)** - MAVLink自定义消息的完整实现指南
- **[PX4 MAVLink文件结构指南](PX4_MAVLink_File_Structure_Guide.md)** - MAVLink相关文件的组织结构和作用说明
- **[PX4 MAVLink启动流程](PX4_MAVLink_Startup_Process.md)** - MAVLink模块的启动和初始化流程分析
- **[MAVLink自定义消息快速参考](MAVLink_Custom_Messages_Quick_Reference.md)** - MAVLink自定义消息的快速参考手册
- **[MAVLink实现示例](MAVLink_Implementation_Example.md)** - MAVLink协议的具体实现示例

### UAVCAN协议
- **[UAVCAN文档集合](uavcan/)** - 完整的UAVCAN协议文档集合
  - 快速参考和文档索引
  - 正确实现指南和最佳实践
  - ESC开发和使用指南
  - 自定义协议和集成示例
  - 故障排除和调试方法

### ROS2与任务计算机通信
- **[PX4飞控与任务计算机通信交互使用指南](PX4任务计算机通信指南.md)** - PX4与任务计算机完整通信方案，包含ROS2、ROS1、MAVLink等多种通信方式的详细配置和应用示例
- **[PX4 ROS2快速入门指南](PX4_ROS2快速入门指南.md)** - PX4通过ROS2进行通信的完整配置步骤，从环境搭建到实际控制的详细指南

## 通信架构总览

### 协议分层
```
应用层
    ↓
[ROS2/ROS1] ←→ [任务计算机应用]
    ↓
[DDS/RTPS] ←→ [MAVLink] ←→ [UAVCAN]
    ↓              ↓           ↓
以太网/WiFi     串口/UDP    CAN总线
    ↓              ↓           ↓
外部计算机      地面站        CAN设备
```

### 主要通信方式

#### 1. MAVLink协议
- **用途**：地面站通信、遥测数据传输
- **特点**：轻量级、跨平台、标准化消息
- **传输层**：串口、UDP、TCP
- **频率**：1-50Hz（根据消息类型）

#### 2. UAVCAN协议  
- **用途**：智能传感器和执行器通信
- **特点**：实时性强、自动配置、冗余设计
- **传输层**：CAN总线
- **频率**：高达1kHz

#### 3. ROS2接口
- **用途**：机器人操作系统集成
- **特点**：分布式、类型安全、QoS控制
- **传输层**：DDS/RTPS
- **频率**：可配置（1Hz-1kHz）

#### 4. uORB内部通信
- **用途**：模块间数据交换
- **特点**：发布/订阅、零拷贝、类型安全
- **传输层**：内存共享
- **频率**：高达8kHz

## 协议选择指南

### MAVLink适用场景
- ✅ 地面站监控和控制
- ✅ 遥测数据记录
- ✅ 任务规划和上传
- ✅ 参数配置管理
- ❌ 高频控制命令
- ❌ 大数据量传输

### UAVCAN适用场景  
- ✅ 智能ESC控制
- ✅ 分布式传感器网络
- ✅ 冗余系统设计
- ✅ 实时执行器控制
- ❌ 大带宽数据传输
- ❌ 非实时监控数据

### ROS2适用场景
- ✅ 复杂任务计算机集成
- ✅ 多机器人协作
- ✅ 高级算法部署
- ✅ 仿真环境集成
- ❌ 简单遥控场景
- ❌ 低延迟要求（<1ms）

## 通信性能对比

| 协议 | 延迟 | 带宽 | 复杂度 | 实时性 | 适用距离 |
|------|------|------|--------|---------|----------|
| uORB | <0.1ms | 高 | 低 | 极高 | 设备内 |
| UAVCAN | 1-5ms | 中 | 中 | 高 | <100m |
| ROS2 | 5-20ms | 高 | 高 | 中 | <1km |
| MAVLink | 10-100ms | 低 | 低 | 低 | 全球 |

## 开发工作流

### 1. MAVLink开发流程
```bash
# 1. 定义消息
vim msg/custom_message.msg

# 2. 生成MAVLink代码
make generate_mavlink

# 3. 实现发送端
# 在对应模块中添加消息发布

# 4. 实现接收端  
# 在mavlink模块中添加消息处理

# 5. 测试验证
# 使用QGC或mavlink_shell测试
```

### 2. UAVCAN开发流程
```bash
# 1. 定义DSDL消息
vim src/drivers/uavcan/libuavcan/dsdl/custom/Message.uavcan

# 2. 实现传感器桥接器
# 继承UavcanSensorBridgeBase

# 3. 注册到传感器工厂
# 在sensor_bridge.cpp中注册

# 4. 编译测试
make px4_fmu-v5_default
```

### 3. ROS2开发流程
```bash
# 1. 配置micro-ROS
# 设置相关参数

# 2. 创建ROS2节点
# 在外部计算机上开发

# 3. 消息映射配置
# 配置PX4消息到ROS2消息的映射

# 4. 网络配置
# 配置DDS通信参数
```

## 调试工具

### MAVLink调试
```bash
# 监听MAVLink消息
mavlink stream -d /dev/ttyS1 -s ATTITUDE -r 10

# MAVLink消息统计
mavlink status

# MAVLink shell
mavlink shell
```

### UAVCAN调试  
```bash
# UAVCAN状态查看
uavcan status

# 节点信息查看
uavcan info

# ESC测试
actuator_test -m uavcan -v 1000
```

### ROS2调试
```bash
# 查看micro-ROS状态
micrortps_client status

# 监听ROS2话题
ros2 topic echo /fmu/out/vehicle_attitude

# ROS2节点信息
ros2 node list
```

## 配置参数

### MAVLink参数
- **MAV_*_MODE**: 数据流配置
- **MAV_*_RATE**: 消息频率设置
- **SER_*_BAUD**: 串口波特率

### UAVCAN参数
- **UAVCAN_ENABLE**: 启用UAVCAN
- **UAVCAN_BITRATE**: CAN总线波特率
- **UAVCAN_NODE_ID**: 节点ID设置

### ROS2参数
- **RTPS_CONFIG**: 配置文件路径
- **RTPS_UDP_CONFIG**: UDP传输配置

## 安全注意事项

### 1. 数据验证
- 严格验证接收数据的有效性
- 实现消息序列号检查
- 添加数据范围限制

### 2. 错误处理
- 实现通信超时检测
- 添加自动重连机制
- 记录通信错误日志

### 3. 权限控制
- 限制危险命令的执行
- 实现操作员身份验证
- 添加安全模式切换

## 相关文档

- [参数系统](../parameters/) - 通信参数的配置管理
- [仿真调试](../simulation/) - 仿真环境中的通信测试
- [硬件特定](../hardware-specific/) - 具体硬件的通信接口配置

---

本目录文档涵盖PX4主要通信协议的实现和应用，适合通信系统开发人员和系统集成工程师参考。