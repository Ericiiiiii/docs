# RTK982 GPS配置指南

## 1. 基本配置步骤

### 1.1 在NSH中配置RTK982

```bash
# 首先停止GPS驱动
gps stop

# 直接向串口发送配置指令
# 设置NMEA消息输出频率为5Hz (0.2秒间隔)
echo -e "GPGGA COM1 0.2\r\n" > /dev/ttyS0
echo -e "GPRMC COM1 0.2\r\n" > /dev/ttyS0
echo -e "GPGST COM1 0.2\r\n" > /dev/ttyS0
echo -e "GPGSV COM1 1.0\r\n" > /dev/ttyS0

# 设置RTK移动站模式
echo -e "MODE ROVER\r\n" > /dev/ttyS0

# 设置高程角截止为10度
echo -e "ECUTOFF 10\r\n" > /dev/ttyS0

# 启用多星座
echo -e "CONFIG SIGNALGROUP 1\r\n" > /dev/ttyS0

# 如果支持双天线，启用航向输出
echo -e "UNIHEADINGA COM1 0.5\r\n" > /dev/ttyS0

# 保存配置
echo -e "SAVECONFIG\r\n" > /dev/ttyS0

# 等待配置生效
sleep 2

# 启动GPS驱动
gps start -d /dev/ttyS0 -p nmea -b 115200
```

### 1.2 验证配置

```bash
# 查看GPS状态
gps status

# 监听GPS数据
listener sensor_gps

# 查看原始串口数据
cat /dev/ttyS0
```

## 2. RTK982常用指令

### 2.1 输出频率控制
```
GPGGA COM1 0.1    # 10Hz输出
GPGGA COM1 0.2    # 5Hz输出
GPGGA COM1 1.0    # 1Hz输出
GPGGA COM1 0      # 停止输出
```

### 2.2 工作模式
```
MODE ROVER        # RTK移动站模式
MODE BASE         # RTK基站模式
MODE SINGLE       # 单点定位模式
```

### 2.3 系统配置
```
ECUTOFF 10        # 设置高程角截止为10度
PDOP 6.0          # 设置PDOP阈值
CONFIG SIGNALGROUP 1  # 启用所有星座
```

### 2.4 双天线配置
```
UNIHEADINGA COM1 0.2     # 航向信息输出
SETBASELINELENGTH 1.5    # 设置基线长度1.5米
```

### 2.5 保存和重置
```
SAVECONFIG        # 保存配置到NVRAM
FRESET            # 恢复出厂设置
```

## 3. 故障排除

### 3.1 检查串口通信
```bash
# 直接读取串口数据
cat /dev/ttyS0

# 检查是否有NMEA数据输出
cat /dev/ttyS0 | grep "\\$GP"
```

### 3.2 重新配置
```bash
# 如果配置失败，尝试恢复出厂设置
echo -e "FRESET\r\n" > /dev/ttyS0
sleep 5

# 然后重新配置
```

### 3.3 不同波特率尝试
```bash
# 尝试不同波特率
gps start -d /dev/ttyS0 -p nmea -b 230400
gps start -d /dev/ttyS0 -p nmea -b 460800
```

## 4. PX4参数配置

```bash
# 设置GPS协议为NMEA
param set GPS_1_PROTOCOL 6

# 设置串口配置
param set GPS_1_CONFIG 101

# 设置波特率
param set SER_GPS1_BAUD 115200

# 启用卫星信息
param set GPS_SAT_INFO 1

# 启用GPS通信转储
param set GPS_DUMP_COMM 1

# 保存参数
param save
```

## 5. GPS调试模式

### 5.1 启用调试模式
对于开发和调试，可以启用GPS NMEA调试模式：

```bash
# 启用GPS NMEA调试模式
param set GPS_NMEA_DEBUG 1
param save

# 重启GPS驱动
gps stop
gps start -d /dev/ttyS0 -p nmea -b 9600
```

### 5.2 调试模式的作用
启用调试模式后，GPS驱动会：
- **即使没有定位也能正常初始化** - 不再要求有效的GPS定位才能启动
- **发布无定位的GPS数据** - `listener sensor_gps`能显示数据，便于调试
- **使用宽松的数据验证** - 提高调试时的容错性

### 5.3 自动化测试脚本
提供了一个自动化测试脚本来验证GPS调试功能：

```bash
# 运行GPS调试模式测试脚本
./Tools/test_gps_debug.sh
```

该脚本会自动：
1. 检查当前参数状态
2. 启用调试模式并测试GPS功能
3. 监听GPS数据输出
4. 恢复正常模式

### 5.4 生产环境注意事项
**重要**：调试模式仅用于开发和调试，生产环境必须禁用：
```bash
# 禁用调试模式（生产环境）
param set GPS_NMEA_DEBUG 0
param save
```

## 6. 预期输出

配置成功后，应该看到类似以下的NMEA数据：

```
$GPGGA,123456.00,3112.34567,N,12123.45678,E,4,12,1.2,123.4,M,45.6,M,1.0,0000*5A
$GPRMC,123456.00,A,3112.34567,N,12123.45678,E,0.12,123.45,010124,,,A*5B
$GPGST,123456.00,12.3,4.5,6.7,123.45,1.2,2.3,3.4*5C
```

如果配置了双天线，还会看到：
```
#UNIHEADINGA,COM1,0,12.3,FINE,2024,1,1,12,34,56;1.500,123.45,0.12*ABCDEF01
```

## 7. 注意事项

1. **指令格式**：RTK982指令必须以`\r\n`结尾
2. **等待时间**：发送指令后需要等待设备响应
3. **保存配置**：使用`SAVECONFIG`保存配置，否则重启后丢失
4. **波特率**：确保PX4和RTK982使用相同的波特率
5. **协议选择**：RTK982主要使用NMEA协议，但也支持一些Unicore特有消息
