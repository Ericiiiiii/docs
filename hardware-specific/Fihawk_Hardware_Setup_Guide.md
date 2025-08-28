# Fihawk FC-V1 硬件配置指南

本指南详细介绍Fihawk FC-V1飞控板的硬件配置、接线连接和调试方法。

## 硬件概述

### 核心规格
- **主控芯片**: STM32H743XI (480MHz ARM Cortex-M7)
- **Flash存储**: 2MB内部Flash
- **RAM**: 1MB SRAM
- **传感器**: ICM-20689 (IMU), IST8310 (磁力计), MS5611 (气压计)
- **接口**: 8路PWM输出, 6路UART, SPI, I2C, CAN, ADC

### 板载外设
- **IMU**: ICM-20689 六轴陀螺仪/加速度计
- **磁力计**: IST8310 三轴磁力计
- **气压计**: MS5611 高精度气压高度计
- **存储**: microSD卡槽（用于日志存储）
- **LED**: 状态指示LED和RGB LED
- **蜂鸣器**: 报警提示

## 接口定义

### 电源接口
```
VCC_5V_IN  : 5V外部电源输入（推荐使用）
VCC_USB    : USB 5V供电（调试用）
GND        : 电源地
VBAT       : 电池电压监测输入（3.3V-5.5V）
```

### UART串口接口

#### UART1 (GPS主接口)
```
Pin 1: VCC_5V   (5V供电)
Pin 2: GPS_RX   (连接GPS模块TX)
Pin 3: GPS_TX   (连接GPS模块RX)
Pin 4: I2C_SCL  (I2C时钟，可选)
Pin 5: I2C_SDA  (I2C数据，可选)
Pin 6: GND      (地)
```

#### UART2 (调试串口)
```
Pin 1: VCC_5V   (5V供电)
Pin 2: DEBUG_RX (调试串口接收)
Pin 3: DEBUG_TX (调试串口发送)
Pin 4: GND      (地)
```

#### UART3 (数传/遥控器)
```
Pin 1: VCC_5V   (5V供电)
Pin 2: TELEM_RX (数传接收)
Pin 3: TELEM_TX (数传发送)
Pin 4: GND      (地)
```

#### UART4 (扩展接口)
```
Pin 1: VCC_5V   (5V供电)
Pin 2: UART4_RX (扩展串口接收)
Pin 3: UART4_TX (扩展串口发送)
Pin 4: GND      (地)
```

### PWM输出接口

#### 主输出 (MAIN OUT)
```
CH1-8: PWM输出通道 (50Hz-400Hz可配置)
GND:   信号地
VCC:   舵机供电 (可选，5V)
```

#### 辅助输出 (AUX OUT)
```
AUX1-6: 辅助PWM输出通道
GND:    信号地
VCC:    舵机供电 (可选，5V)
```

### SPI接口
- **SPI1**: ICM-20689 (IMU)
- **SPI6**: ICM-20689 (备用IMU，如果配置)
- **SPI4**: 外部扩展SPI接口

### I2C接口
- **I2C1**: IST8310 (磁力计)
- **I2C2**: MS5611 (气压计)
- **I2C3**: 外部扩展I2C接口

### CAN接口
```
CAN1_H: CAN高电平信号
CAN1_L: CAN低电平信号
GND:    地
VCC:    5V供电
```

## 硬件连接示例

### 四旋翼标准连接

#### 电机连接（X型布局）
```
电机1 (前右): MAIN OUT CH1
电机2 (后左): MAIN OUT CH2  
电机3 (前左): MAIN OUT CH3
电机4 (后右): MAIN OUT CH4
```

#### 遥控器连接
```
接收机类型: SBUS/PPM/DSM
连接位置: UART3 (TELEM接口)
波特率:   100000 (SBUS) / PPM信号
```

#### GPS模块连接
```
GPS模块:  UM982/其他UBLOX GPS
连接位置: UART1 (GPS接口)
波特率:   38400/115200
协议:     UBX
```

#### 数传模块连接
```
数传模块: 433MHz/915MHz数传
连接位置: UART3 (TELEM接口，如未使用遥控器)
波特率:   57600/115200
协议:     MAVLink v2
```

### 固定翼连接

#### 舵机连接
```
副翼 (左):  MAIN OUT CH1
副翼 (右):  MAIN OUT CH2
升降舵:     MAIN OUT CH3
方向舵:     MAIN OUT CH4
油门:       MAIN OUT CH8
```

#### 襟翼连接（可选）
```
襟翼 (左):  AUX OUT CH1
襟翼 (右):  AUX OUT CH2
```

## 传感器配置

### IMU配置
```bash
# 检查IMU状态
commander status
sensor_combined status

# 查看IMU参数
param show IMU*
param show SENS_BOARD*

# 校准加速度计
commander calibrate accelerometer

# 校准陀螺仪
commander calibrate gyroscope
```

### 磁力计配置
```bash
# 校准磁力计
commander calibrate magnetometer

# 查看磁力计参数
param show CAL_MAG*
param show SENS_MAG*

# 设置磁力计方向（如需要）
param set SENS_MAG_ROT 0  # 无旋转
```

### 气压计配置
```bash
# 校准气压计
commander calibrate level

# 查看气压计参数
param show SENS_BARO*
param show EKF2_BARO*
```

### GPS配置
```bash
# 查看GPS状态
gps status

# GPS参数配置
param set GPS_1_CONFIG 101  # UART1, 38400波特率
param set GPS_1_PROTOCOL 1  # UBX协议

# 查看GPS数据
listener vehicle_gps_position
```

## 调试接口配置

### JLink调试器连接
```
JLink VCC  -> Fihawk 3.3V
JLink SWDIO -> Fihawk SWDIO
JLink SWCLK -> Fihawk SWCLK  
JLink GND   -> Fihawk GND
```

### 串口调试连接
```
USB转TTL VCC -> Fihawk 5V (可选)
USB转TTL RX  -> Fihawk DEBUG_TX (UART2)
USB转TTL TX  -> Fihawk DEBUG_RX (UART2)
USB转TTL GND -> Fihawk GND
```

## 固件烧录

### 使用JLink烧录
```bash
# 自动编译并烧录
./boards/fihawk/fc-v1/flash_firmware.sh

# 仅烧录（不重新编译）
./boards/fihawk/fc-v1/flash_firmware.sh --no-build

# 详细输出
./boards/fihawk/fc-v1/flash_firmware.sh --verbose
```

### 使用Bootloader烧录
```bash
# 通过QGroundControl固件升级
# 1. 连接USB
# 2. 打开QGC固件设置
# 3. 选择自定义固件文件
# 4. 选择build/fihawk_fc-v1_default/fihawk_fc-v1_default.px4
```

## 参数配置

### 机型配置
```bash
# 四旋翼X型
param set SYS_AUTOSTART 4001

# 四旋翼+型  
param set SYS_AUTOSTART 4010

# 固定翼
param set SYS_AUTOSTART 4200

# 重启应用配置
reboot
```

### PWM输出配置
```bash
# 设置PWM频率 (50Hz标准舵机, 400Hz电调)
param set PWM_MAIN_RATE 400  # 主输出频率
param set PWM_AUX_RATE 400   # 辅助输出频率

# 设置PWM范围
param set PWM_MAIN_MIN 1000  # 最小脉宽
param set PWM_MAIN_MAX 2000  # 最大脉宽
```

### 遥控器配置
```bash
# SBUS遥控器
param set RC_MAP_MODE_SW 5   # 飞行模式切换通道
param set RC_MAP_RETURN_SW 7 # 返航开关通道
param set RC_MAP_KILL_SW 8   # 紧急停止开关

# 校准遥控器
commander calibrate radio
```

### 电池监测配置
```bash
# 设置电池芯数和电压
param set BAT_N_CELLS 4      # 4S锂电池
param set BAT_V_CHARGED 4.2  # 满电电压
param set BAT_V_EMPTY 3.4    # 空电电压

# 设置低电压报警
param set BAT_LOW_THR 0.25   # 25%电量报警
param set BAT_CRIT_THR 0.1   # 10%电量紧急
```

## 状态监控

### 系统状态检查
```bash
# 整体系统状态
commander status

# 传感器状态
sensor_combined status

# 执行器状态  
actuator_test status

# GPS状态
gps status

# 电池状态
battery_status
```

### 实时数据监控
```bash
# 实时传感器数据
listener sensor_combined

# 实时姿态数据
listener vehicle_attitude

# 实时位置数据
listener vehicle_local_position

# 实时GPS数据
listener vehicle_gps_position
```

## 性能调优

### 控制频率设置
```bash
# IMU采样频率（通常1000Hz）
param show SENS_IMU_RATE

# EKF2更新频率（通常250Hz）
param show EKF2_PREDICT_US

# 控制器更新频率（通常500Hz）
param show MC_*_RATE*
```

### 传感器滤波
```bash
# 陀螺仪滤波参数
param show IMU_GYRO_CUTOFF

# 加速度计滤波参数  
param show IMU_ACCEL_CUTOFF

# D项滤波参数
param show MC_*_D_LPF
```

## 故障排除

### 常见硬件问题

#### 传感器故障
- 检查传感器供电和连接
- 重新校准传感器
- 查看传感器数据是否正常

#### PWM输出故障
- 检查PWM参数设置
- 使用`pwm test`命令测试输出
- 检查执行器连接

#### 通信故障
- 检查串口接线和参数
- 确认波特率设置正确
- 使用示波器检查信号质量

### 调试工具
```bash
# PWM输出测试
pwm arm
pwm test -c 1 -p 1500  # 通道1输出1500μs

# 传感器原始数据
sensor_combined log

# 系统资源监控
top
free
```

## 相关文档

- [传感器配置详解](Fihawk_Sensor_Issues_Analysis.md)
- [串口配置指南](Fihawk_Serial_Configuration.md)
- [控制频率分析](fihawk-fc-v1-sensor-control-frequencies.md)
- [快速入门指南](../development/Quick_Start_Guide.md)

---

正确的硬件配置是飞控系统稳定运行的基础。如遇到问题，请参考[故障排除文档](../troubleshooting/)或联系技术支持。