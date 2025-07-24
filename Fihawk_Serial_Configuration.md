# Fihawk FC-V1 串口配置文档

## 概述

本文档详细描述了Fihawk FC-V1飞控的串口配置和使用方法。

## 硬件串口分配

### 串口映射表

| 设备节点 | 硬件UART | GPIO引脚 | 功能 | 默认波特率 | 流控制 | 连接器 |
|---------|---------|----------|------|-----------|--------|--------|
| /dev/ttyS0 | USART1 | PB6(TX)/PB7(RX) | GPS1 | 115200 | 无 | GPS1 |
| /dev/ttyS1 | USART2 | PD5(TX)/PD6(RX)/PD4(RTS)/PD3(CTS) | TELEM1 | 57600 | RTS/CTS | TELEM1 |
| /dev/ttyS2 | UART4 | PD1(TX)/PD0(RX) | GPS2 | 115200 | 无 | GPS2 |
| /dev/ttyS3 | USART6 | PG14(TX)/PG9(RX)/PG8(RTS)/PG15(CTS) | TELEM2 | 921600 | RTS/CTS | TELEM2 |
| /dev/ttyS4 | UART7 | PE8(TX)/PF6(RX) | DEBUG | 57600 | 无 | DEBUG |
| /dev/ttyS5 | UART8 | PE1(TX)/PE0(RX) | RC | 变量 | 无 | RC |

### 特殊功能

- **TELEM1/TELEM2**: 支持硬件流控制，适用于高速数据传输
- **RC串口**: 支持单线模式，兼容SBUS/DSM等协议
- **DEBUG串口**: 用于系统调试和引导加载器通信

## 配置文件

### 1. 板级配置 (default.px4board)

```bash
CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS0"
CONFIG_BOARD_SERIAL_GPS2="/dev/ttyS2"
CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS1"
CONFIG_BOARD_SERIAL_TEL2="/dev/ttyS3"
```

### 2. 硬件配置 (board_config.h)

```c
/* RC串口配置 */
#define RC_SERIAL_PORT          "/dev/ttyS5"
#define RC_SERIAL_SINGLEWIRE

/* 调试串口配置 */
#define INTERFACE_USART_CONFIG  "/dev/ttyS4,57600"

/* 特殊模式支持 */
#define GPIO_PPM_IN_AS_OUT      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN4)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT() px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
```

### 3. 默认参数 (rc.board_defaults)

```bash
# GPS配置
param set-default SER_GPS1_BAUD 115200
param set-default SER_GPS2_BAUD 115200

# 遥测配置
param set-default MAV_0_CONFIG 101          # TELEM1
param set-default MAV_0_RATE 1200
param set-default SER_TEL1_BAUD 57600

param set-default MAV_1_CONFIG 102          # TELEM2
param set-default MAV_1_RATE 80000
param set-default SER_TEL2_BAUD 921600

# RC配置
param set-default RC_INPUT_PROTO 1
```

## 使用场景

### 1. GPS配置

#### 单GPS配置
```bash
# 使用GPS1端口
param set GPS_1_CONFIG 201
param set SER_GPS1_BAUD 115200
```

#### 双GPS配置
```bash
# 主GPS使用GPS1
param set GPS_1_CONFIG 201
param set SER_GPS1_BAUD 115200

# 备用GPS使用GPS2
param set GPS_2_CONFIG 202
param set SER_GPS2_BAUD 115200
```

### 2. 遥测链路配置

#### 标准遥测 (TELEM1)
```bash
# 57600波特率，适用于3DR Radio等
param set MAV_0_CONFIG 101
param set SER_TEL1_BAUD 57600
param set MAV_0_MODE 2          # 标准模式
param set MAV_0_RATE 1200       # 1200 B/s
```

#### 高速遥测 (TELEM2)
```bash
# 921600波特率，适用于机载计算机
param set MAV_1_CONFIG 102
param set SER_TEL2_BAUD 921600
param set MAV_1_MODE 0          # 正常模式
param set MAV_1_RATE 80000      # 80000 B/s
```

### 3. RC接收机配置

#### SBUS接收机
```bash
param set RC_INPUT_PROTO 1      # SBUS协议
# RC串口自动配置为100000波特率，单线模式
```

#### DSM接收机
```bash
param set RC_INPUT_PROTO 3      # DSM协议
# 使用UART8的特殊绑定功能
```

#### PPM接收机
```bash
param set RC_INPUT_PROTO 0      # PPM协议
# 使用TIM3_CH1 (PB4)引脚
```

### 4. 调试配置

#### 系统控制台
```bash
# 通过DEBUG串口访问NSH控制台
# 波特率: 57600
# 连接: PE8(TX) -> USB转串口RX
#       PF6(RX) -> USB转串口TX
```

#### MAVLink调试
```bash
# 在DEBUG端口启用MAVLink
param set MAV_2_CONFIG 104      # 使用TELEM4 (映射到DEBUG)
param set SER_TEL4_BAUD 57600
```

## 高级配置

### 1. 流控制配置

```bash
# 启用TELEM1流控制
param set MAV_0_FLOW_CTRL 1

# 启用TELEM2流控制  
param set MAV_1_FLOW_CTRL 1
```

### 2. 自定义串口应用

#### 添加传感器驱动
```bash
# 例: 在TELEM2上连接激光雷达
param set SENS_EN_SF45_CFG 102  # 使用TELEM2
param set SER_TEL2_BAUD 115200  # 设置合适波特率
```

#### UXRCE-DDS客户端
```bash
# 在TELEM2上启用DDS
param set UXRCE_DDS_CFG 102
param set SER_TEL2_BAUD 921600
```

### 3. 多实例配置

#### 多MAVLink实例
```bash
# 实例0: TELEM1 - 地面站
param set MAV_0_CONFIG 101
param set MAV_0_MODE 2

# 实例1: TELEM2 - 机载计算机
param set MAV_1_CONFIG 102  
param set MAV_1_MODE 0

# 实例2: DEBUG - 调试
param set MAV_2_CONFIG 104
param set MAV_2_MODE 1
```

## 故障排除

### 常见问题

#### 1. GPS无信号
```bash
# 检查GPS配置
param show GPS_*
param show SER_GPS*

# 监控GPS状态
listener vehicle_gps_position
```

#### 2. 遥测连接失败
```bash
# 检查MAVLink配置
param show MAV_*
param show SER_TEL*

# 测试串口连接
mavlink status
```

#### 3. RC信号异常
```bash
# 检查RC配置
param show RC_*

# 监控RC输入
listener input_rc
```

### 调试命令

```bash
# 查看所有串口状态
listener serial_status

# 测试串口通信
echo "test" > /dev/ttyS1

# 监控串口数据
cat /dev/ttyS1

# 检查硬件连接
gpio read <pin_number>
```

## 性能优化

### 1. 波特率选择

| 应用 | 推荐波特率 | 说明 |
|------|-----------|------|
| GPS | 115200 | 标准GPS波特率 |
| 遥测 | 57600 | 兼容性好 |
| 高速数传 | 921600 | 最大性能 |
| 调试 | 57600 | 稳定可靠 |

### 2. 流控制使用

- **启用场景**: 高速数据传输 (>115200)
- **硬件要求**: 连接RTS/CTS引脚
- **软件配置**: 设置 `MAV_x_FLOW_CTRL = 1`

### 3. 工作队列优化

```cpp
// 串口工作队列映射
ttyS0 -> wq_configurations::ttyS0  // GPS专用
ttyS1 -> wq_configurations::ttyS1  // 遥测专用
ttyS3 -> wq_configurations::ttyS3  // 高速数传专用
```

## 扩展应用

### 1. 添加新传感器

1. 选择可用串口 (如GPS2未使用时)
2. 配置传感器驱动参数
3. 设置合适的波特率和协议

### 2. 机载计算机集成

1. 使用TELEM2高速端口
2. 启用流控制
3. 配置MAVLink或DDS通信

### 3. 自定义协议

1. 开发串口驱动程序
2. 添加module.yaml配置
3. 集成到参数系统

---

**注意**: 修改串口配置后需要重启飞控才能生效。建议在地面测试环境中验证配置的正确性。
