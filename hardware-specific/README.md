# 硬件特定配置文档

本目录包含特定硬件平台的配置、调试和优化文档，主要针对Fihawk FC-V1飞控板和相关硬件。

## 文档列表

### Fihawk FC-V1飞控板
- **[Fihawk传感器问题分析](Fihawk_Sensor_Issues_Analysis.md)** - Fihawk FC-V1传感器相关问题的诊断和解决方案
- **[Fihawk串口配置指南](Fihawk_Serial_Configuration.md)** - Fihawk FC-V1飞控的串口配置详解和实际应用
- **[Fihawk FC-V1传感器采集周期与控制频率分析](fihawk-fc-v1-sensor-control-frequencies.md)** - 详细分析传感器采集周期、数据融合频率、控制频率和电机输出控制频率

### 系统级工具
- **[PX4引导加载器刷写指南](PX4_Bootloader_Flashing_Guide.md)** - PX4引导加载器的刷写方法和调试技巧

## Fihawk FC-V1硬件架构

### 核心规格
```
MCU: STM32H743XI
- ARM Cortex-M7 @ 480MHz
- 2MB Flash, 1MB RAM
- 双精度FPU支持

传感器配置:
- IMU: ICM-20689 (SPI6)
- 磁力计: QMC5883L (I2C)  
- 气压计: MS5611 (SPI1)
- GPS: UART4 (TELEM2)

接口配置:
- USB: USB-C全速
- 串口: 8路UART
- SPI: 6路SPI总线
- I2C: 4路I2C总线
- CAN: 2路CAN总线
- PWM: 16路PWM输出
```

### 引脚映射
```cpp
// 主要GPIO定义
#define GPIO_SPI6_CS_ICM20689    /* PH5 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN5)
#define GPIO_SPI1_CS_MS5611      /* PC2 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN2)

// 串口配置
#define PX4_SERIAL_TEL1_UART     4  // UART4 - GPS/TELEM2
#define PX4_SERIAL_TEL2_UART     1  // UART1 - TELEM1
#define PX4_SERIAL_GPS1_UART     4  // UART4 - GPS主接口
```

## 传感器系统分析

### 采集频率配置
```bash
# IMU高频采集 (ICM-20689)
参数: IMU_GYRO_RATEMAX = 2000    # 陀螺仪最大采样率 2kHz
参数: IMU_ACCEL_RATEMAX = 1000   # 加速度计最大采样率 1kHz
实际: 陀螺仪 = 1kHz, 加速度计 = 1kHz

# 磁力计低频采集 (QMC5883L)  
参数: SENS_MAG_RATE = 50         # 磁力计采样率 50Hz
实际: 磁力计 = 50Hz

# 气压计中频采集 (MS5611)
参数: SENS_BARO_RATE = 50        # 气压计采样率 50Hz  
实际: 气压计 = 50Hz
```

### 数据处理流水线
```
传感器原始数据采集
        ↓
[1kHz] IMU数据预处理
        ↓  
[500Hz] EKF2状态估计
        ↓
[250Hz] 姿态控制器
        ↓
[250Hz] 角速度控制器
        ↓
[400Hz] PWM电机输出
```

## 串口系统配置

### 串口资源分配
| 串口 | 用途 | 波特率 | 管脚 | 备注 |
|------|------|--------|------|------|
| UART1 | TELEM1 | 57600 | PB6/PB7 | 主遥测链路 |
| UART4 | GPS/TELEM2 | 38400 | PA0/PA1 | GPS主接口 |
| UART5 | TELEM3 | 57600 | PC12/PD2 | 备用遥测 |
| UART6 | EXT | 57600 | PC6/PC7 | 外部设备 |
| UART7 | DEBUG | 57600 | PE8/PE7 | 调试控制台 |
| UART8 | GPS2 | 38400 | PE1/PE0 | GPS备用 |

### 串口配置实例
```bash
# GPS配置 (UART4)
param set GPS_1_CONFIG 102      # TELEM2端口
param set GPS_1_PROTOCOL 1      # UBX协议
param set SER_TEL2_BAUD 38400   # 38.4k波特率

# 遥测配置 (UART1)  
param set MAV_0_CONFIG 101      # TELEM1端口
param set MAV_0_MODE 2          # 全功能模式
param set SER_TEL1_BAUD 57600   # 57.6k波特率

# 调试控制台 (UART7)
param set MAV_2_CONFIG 107      # DEBUG端口
param set SER_URT6_BAUD 57600   # 57.6k波特率
```

## 性能优化配置

### 1. 高频任务调度
```cpp
// 关键任务优先级设置
#define SCHED_PRIORITY_FAST_LOOP    180  // 高频控制回路
#define SCHED_PRIORITY_ATTITUDE     170  // 姿态控制器
#define SCHED_PRIORITY_POSITION     160  // 位置控制器
#define SCHED_PRIORITY_NAVIGATION   150  // 导航器
```

### 2. 内存优化配置
```bash
# 内存池大小优化
param set SYS_STCK_EN 0          # 禁用栈检查
param set UAVCAN_POOL_SIZE 4096  # UAVCAN内存池
param set COM_CPU_MAX 95         # CPU使用限制
```

### 3. 存储性能优化
```bash
# SD卡日志优化
param set SDLOG_DIRS_MAX 10      # 最大日志目录数
param set SDLOG_PROFILE 19       # 高性能日志模式
param set SDLOG_MODE 1           # 启动时开始记录
```

## 故障诊断流程

### 1. 硬件自检
```bash
# 传感器状态检查
sensors status                   # 传感器总状态
mpu6000 info                    # IMU状态
ms5611 info                     # 气压计状态  
qmc5883l info                   # 磁力计状态

# 串口连接检查
serial info                     # 串口配置状态
gps status                      # GPS连接状态
```

### 2. 电源系统检查
```bash
# 电源监控
battery status                  # 电池状态
power_monitor status            # 电源监控器

# 电压检查
listener battery_status         # 电池电压监控
param show BAT*                 # 电池参数配置
```

### 3. 通信链路测试
```bash
# MAVLink通信测试
mavlink status                  # MAVLink状态
listener vehicle_command        # 命令接收

# uORB消息流测试
listener vehicle_attitude       # 姿态数据
listener actuator_outputs       # 电机输出
```

## 定制固件构建

### 1. 板级配置
```cmake
# fihawk_fc-v1配置文件
set(config_module_list
    # 核心系统
    modules/commander
    modules/navigator  
    modules/sensors
    
    # 控制器
    modules/mc_pos_control
    modules/mc_att_control  
    modules/mc_rate_control
    
    # 驱动
    drivers/imu/mpu6000
    drivers/barometer/ms5611
    drivers/magnetometer/qmc5883l
    drivers/gps
)
```

### 2. 编译目标
```bash
# 标准固件编译
make fihawk_fc-v1_default

# 引导程序编译  
make fihawk_fc-v1_bootloader

# 全量编译（固件+引导程序）
make fihawk_fc-v1_default fihawk_fc-v1_bootloader
```

### 3. 固件烧录
```bash
# 使用专用脚本烧录
./boards/fihawk/fc-v1/flash_firmware.sh

# 强制重新编译烧录
./boards/fihawk/fc-v1/flash_firmware.sh --force

# 手动烧录方式
# 1. 连接JLink调试器
# 2. 进入bootloader模式
# 3. 执行烧录脚本
```

## 调试与测试

### 1. JLink调试配置
```bash
# 启动JLink GDB服务器
JLinkGDBServer -if SWD -device STM32H743XI -speed 4000

# GDB连接调试
arm-none-eabi-gdb build/fihawk_fc-v1_default/fihawk_fc-v1_default.elf
(gdb) target remote localhost:2331
(gdb) monitor reset
(gdb) load
```

### 2. 性能基准测试
```bash
# CPU性能测试
perf                           # 性能计数器
top                            # CPU使用率

# 传感器性能测试  
sensors status                 # 传感器更新频率
listener -r sensor_accel       # 加速度计数据率
listener -r sensor_gyro        # 陀螺仪数据率
```

### 3. 飞行测试验证
```bash
# 预飞检查清单
commander check               # 系统健康检查
sensors calibrate            # 传感器校准
esc_calibration              # 电调校准

# 测试飞行模式
commander arm                # 解锁测试
commander takeoff            # 起飞测试
commander land               # 着陆测试
```

## 维护与升级

### 1. 固件版本管理
```bash
# 查看当前版本
ver all                      # 完整版本信息
param show SYS_FMU_TASK      # 系统任务配置

# 参数备份恢复
param export /fs/microsd/params_backup_$(date +%Y%m%d).txt
param import /fs/microsd/params_backup_20241201.txt
```

### 2. 硬件维护检查
- **定期检查**：连接器、传感器安装
- **环境测试**：温度、湿度、振动  
- **老化测试**：长期运行稳定性
- **EMC测试**：电磁兼容性验证

## 相关文档

- [驱动架构](../drivers/) - 底层驱动的详细实现
- [通信协议](../communication/) - 串口和通信接口配置
- [参数系统](../parameters/) - 硬件相关参数配置
- [仿真调试](../simulation/) - 硬件在环测试方法

---

本目录文档提供Fihawk FC-V1等特定硬件平台的完整技术支持，适合硬件工程师和系统集成人员参考。