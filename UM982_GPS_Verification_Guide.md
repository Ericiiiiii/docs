# UM982 RTK GPS验证指南

## 概述
本文档详细介绍如何在Fihawk飞控上验证UM982 RTK GPS的功能。UM982是一款高精度RTK GPS模块，支持多星座定位和厘米级精度。

## 1. 硬件连接验证

### 1.1 串口连接确认
```bash
# 检查串口设备
ls -la /dev/ttyS*

# 检查串口1是否可用
cat /proc/tty/driver/serial
```

### 1.2 波特率测试
UM982支持多种波特率，常用配置：
- 115200 (默认)
- 230400 (推荐RTK使用)
- 460800 (高频数据)

## 2. PX4参数配置

### 2.1 GPS基本参数
```bash
# 设置GPS1协议为NMEA (UM982使用NMEA协议)
param set GPS_1_PROTOCOL 6

# 设置GPS1串口配置
param set GPS_1_CONFIG 101  # 对应UART1

# 设置波特率参数
param set SER_GPS1_BAUD 115200

# 启用卫星信息
param set GPS_SAT_INFO 1

# 设置GNSS系统 (GPS+GLONASS+BeiDou+Galileo)
param set GPS_1_GNSS 31  # 二进制: 11111 (GPS+SBAS+Galileo+BeiDou+GLONASS)
```

### 2.2 RTK相关参数
```bash
# 启用GPS通信数据记录
param set GPS_DUMP_COMM 1

# 设置动态模型为航空模式
param set GPS_UBX_DYNMODEL 7
```

## 3. GPS驱动启动

### 3.1 手动启动GPS驱动
```bash
# 停止现有GPS驱动
gps stop

# 启动GPS驱动，指定NMEA协议
gps start -d /dev/ttyS0 -p nmea -b 115200

# 或者让系统自动检测协议
gps start -d /dev/ttyS0 -b 115200
```

### 3.2 验证驱动状态
```bash
# 查看GPS状态
gps status

# 监听GPS数据
listener sensor_gps

# 监听卫星信息
listener satellite_info
```

## 4. UM982特定验证

### 4.1 NMEA消息验证
UM982输出标准NMEA消息，主要包括：
- **GPGGA**: 位置信息
- **GPRMC**: 推荐最小数据
- **GPGST**: 精度信息
- **UNIHEADINGA**: 航向信息 (双天线模式)
- **UNIAGRICA**: 速度信息

### 4.2 RTK状态检查
```bash
# 检查RTK状态
listener sensor_gps | grep -E "fix_type|rtk"

# RTK状态含义:
# fix_type = 3: 3D Fix
# fix_type = 4: RTK Float
# fix_type = 5: RTK Fixed
```

## 5. 通信调试

### 5.1 串口通信测试
```bash
# 直接读取串口数据
cat /dev/ttyS0

# 或使用minicom
minicom -D /dev/ttyS0 -b 115200
```

### 5.2 GPS数据转储
```bash
# 启用GPS通信转储
param set GPS_DUMP_COMM 1

# 重启GPS驱动
gps stop
gps start -d /dev/ttyS0 -p nmea -b 115200

# 查看转储数据
listener gps_dump
```

## 6. 性能验证

### 6.1 定位精度测试
```bash
# 监听GPS位置数据
listener sensor_gps | grep -E "lat|lon|alt|eph|epv"

# 检查精度指标:
# eph: 水平精度 (米)
# epv: 垂直精度 (米)
# RTK Fixed模式下应小于0.1米
```

### 6.2 更新频率测试
```bash
# 检查GPS更新频率
gps status | grep "rate"

# 正常应显示:
# rate position: 5.00 Hz (或10.00 Hz)
# rate velocity: 5.00 Hz
# rate publication: 5.00 Hz
```

## 7. 故障排除

### 7.1 常见问题
1. **无GPS数据**: 检查串口连接和波特率
2. **协议不匹配**: 尝试不同协议 (NMEA/UBX)
3. **RTK不工作**: 检查RTCM数据注入

### 7.2 调试命令
```bash
# 重置GPS设备
gps reset warm

# 检查系统日志
dmesg | grep -i gps

# 检查uORB主题
uorb top | grep gps
```

## 8. UM982配置命令

### 8.1 基本配置
通过串口发送AT命令配置UM982：
```
# 设置输出频率为10Hz
GPGGA COM1 0.1
GPRMC COM1 0.1

# 启用RTK模式
MODE ROVER

# 设置高程角
ECUTOFF 10
```

### 8.2 RTK配置
```
# 配置RTCM输入
RTCM1005 COM2 1.0
RTCM1077 COM2 1.0
RTCM1087 COM2 1.0
RTCM1097 COM2 1.0
RTCM1127 COM2 1.0

# 保存配置
SAVECONFIG
```

## 9. 验证清单

- [ ] 硬件连接正确
- [ ] 串口设备可访问
- [ ] GPS驱动启动成功
- [ ] 接收到NMEA数据
- [ ] GPS状态显示"OK"
- [ ] 获得3D定位 (fix_type >= 3)
- [ ] RTK状态正常 (如果有基站)
- [ ] 精度满足要求 (eph < 5m)
- [ ] 更新频率正常 (>= 1Hz)

## 10. 性能指标

### 10.1 定位性能
- **单点定位精度**: 1-3米
- **RTK Float精度**: 0.3-1米  
- **RTK Fixed精度**: 0.01-0.1米
- **首次定位时间**: <30秒
- **RTK收敛时间**: 1-5分钟

### 10.2 系统性能
- **数据更新率**: 1-10Hz
- **串口波特率**: 115200-460800
- **CPU占用**: <5%
- **内存占用**: <1MB

通过以上步骤，可以全面验证UM982 RTK GPS在Fihawk飞控上的功能和性能。
