# GPS导航定位系统文档

本目录包含PX4 GPS导航定位系统的详细技术文档，从GPS配置流程到RTK高精度定位的完整实现。

## 文档列表

### GPS核心系统
- **[PX4 GPS_CONFIG参数判断流程详解](PX4_GPS_CONFIG_Parameter_Flow.md)** - GPS配置参数的完整判断流程，从参数设置到串口绑定的全过程分析
- **[PX4 GPS运行状态流程](PX4_GPS_runningstateflow.md)** - GPS模块的运行机制和状态流程分析
- **[PX4 GPS状态变化处理](PX4_GPSstatechange.md)** - GPS状态管理和事件处理机制详解

### RTK高精度定位
- **[RTK982配置指南](RTK982_Configuration_Guide.md)** - RTK982 GPS模块的配置和调试指南
- **[UM982 GPS验证指南](UM982_GPS_Verification_Guide.md)** - UM982 RTK GPS的功能验证和测试方法

## GPS系统架构

### 整体架构图
```
GNSS卫星信号
        ↓
GPS接收机 (RTK982/UM982)
        ↓
串口通信 (UART)
        ↓
GPS驱动模块 (gps)
        ↓
uORB消息发布
        ↓
EKF2数据融合
        ↓
位置估计输出
```

### 核心组件

#### 1. GPS驱动层
```cpp
// GPS驱动基类
class GPSBaseStation : public ModuleBase<GPSBaseStation>
{
    // 协议解析器
    GPSHelper *_helper{nullptr};
    
    // 数据缓存
    sensor_gps_s _report_gps{};
    
    // 状态管理
    gps_dump_state_e _dump_state{GPS_DUMP_STATE_DISABLED};
};
```

#### 2. 协议解析层
- **UBX协议**：u-blox GPS标准协议
- **NMEA协议**：通用GPS协议
- **MTK协议**：联发科GPS协议
- **RTCM协议**：RTK差分数据协议

#### 3. 数据融合层
- **位置数据**：经纬度、高度、速度
- **精度信息**：HDOP、VDOP、位置精度
- **时间同步**：GPS时间与系统时间同步
- **状态标志**：定位状态、卫星数量、信号强度

## GPS配置流程

### 1. 参数配置阶段
```bash
# GPS基本配置
param set GPS_1_CONFIG 101    # TELEM 2端口
param set GPS_1_PROTOCOL 1    # UBX协议
param set GPS_1_GNSS 0        # 自动GNSS系统选择

# RTK配置（如果使用）
param set GPS_YAW_OFFSET 0.0  # GPS航向偏移
param set GPS_1_RTK 1         # 启用RTK
```

### 2. 硬件连接验证
```bash
# 检查串口配置
param show SER_TEL2_BAUD     # 波特率设置
param show SER_TEL2_*        # 串口参数

# GPS模块状态检查
gps status                   # GPS驱动状态
listener sensor_gps          # GPS数据监听
```

### 3. 定位质量评估
```bash
# 查看定位精度
listener vehicle_gps_position

# 检查卫星信息
listener satellite_info

# RTK状态监控（如适用）
listener rtk_status
```

## RTK高精度定位

### RTK工作原理
```
基站 (Base Station)
    ↓ RTCM差分数据
移动站 (Rover)
    ↓ 载波相位解算
高精度位置 (厘米级)
```

### RTK配置步骤

#### 1. 基站设置
```bash
# 基站GPS配置
param set GPS_1_CONFIG 102    # 基站串口
param set GPS_1_RTK 2         # 基站模式

# 基站位置设定（已知坐标）
param set GPS_1_RTK_LAT 31.123456
param set GPS_1_RTK_LON 121.123456  
param set GPS_1_RTK_ALT 10.0
```

#### 2. 移动站配置
```bash
# 移动站GPS配置  
param set GPS_1_RTK 1         # 移动站模式
param set GPS_2_CONFIG 103    # 差分数据接收串口

# RTK数据链配置
param set GPS_RTK_LINK 1      # 启用数据链
```

#### 3. 精度验证
```bash
# RTK固定解状态
listener rtk_status
# fix_type: 6 (RTK固定解)
# precision: < 0.1m

# 位置精度监控
listener vehicle_gps_position  
# eph: < 0.1 (水平精度)
# epv: < 0.2 (垂直精度)
```

## GPS状态机

### 状态定义
```cpp
enum gps_fix_type_t {
    GPS_FIX_TYPE_NONE = 0,      // 无定位
    GPS_FIX_TYPE_2D = 2,        // 2D定位
    GPS_FIX_TYPE_3D = 3,        // 3D定位
    GPS_FIX_TYPE_DGPS = 4,      // 差分定位
    GPS_FIX_TYPE_RTK_FLOAT = 5, // RTK浮点解
    GPS_FIX_TYPE_RTK_FIXED = 6  // RTK固定解
};
```

### 状态转换
```
[无定位] → [2D定位] → [3D定位]
                           ↓
                      [DGPS定位]
                           ↓
                    [RTK浮点解]
                           ↓
                    [RTK固定解]
```

## 常见GPS模块配置

### 1. u-blox系列
```bash
# M8N配置
param set GPS_1_PROTOCOL 1    # UBX协议
param set SER_TEL2_BAUD 38400 # 波特率

# F9P RTK配置
param set GPS_1_PROTOCOL 1    # UBX协议
param set GPS_1_RTK 1         # RTK模式
param set SER_TEL2_BAUD 115200
```

### 2. 司南导航UM982
```bash
# UM982配置
param set GPS_1_PROTOCOL 1    # UBX协议
param set GPS_1_GNSS 15       # 全星座
param set SER_TEL2_BAUD 230400
```

### 3. 华测RTK982
```bash
# RTK982配置  
param set GPS_1_PROTOCOL 5    # NMEA协议
param set GPS_1_RTK 1         # RTK模式
param set SER_TEL2_BAUD 115200
```

## 性能优化与调试

### 性能指标
- **定位精度**：RTK < 10cm, 单点定位 < 3m
- **更新频率**：5-10Hz典型值
- **冷启动时间**：< 30秒
- **热启动时间**：< 5秒

### 调试工具
```bash
# GPS数据记录
param set SDLOG_GPS_MSG 1     # 记录GPS消息
param set SDLOG_MODE 1        # 启动时开始记录

# 实时监控
gps status -v                 # 详细状态信息
listener sensor_gps           # 原始GPS数据
listener vehicle_gps_position # 处理后位置信息
```

### 故障诊断

#### 常见问题
1. **无GPS信号**
   - 检查天线连接
   - 确认天线位置（避免遮挡）
   - 验证串口配置

2. **定位精度差**
   - 检查卫星数量（>6颗）
   - 验证HDOP值（<2.0）
   - 排除多路径干扰

3. **RTK无法固定**
   - 确认基站数据接收
   - 检查基站坐标准确性
   - 验证移动站与基站距离（<20km）

#### 诊断命令
```bash
# GPS健康检查
gps reset                     # 重启GPS模块
param show GPS_*              # 查看所有GPS参数
dmesg | grep gps              # 查看GPS相关日志
```

## 高级功能

### 1. 双GPS冗余
```bash
# 主GPS配置
param set GPS_1_CONFIG 101
param set GPS_1_PROTOCOL 1

# 备份GPS配置
param set GPS_2_CONFIG 102  
param set GPS_2_PROTOCOL 1

# 自动切换启用
param set GPS_BLEND_MASK 1
```

### 2. GPS航向辅助
```bash
# 双天线GPS航向
param set GPS_YAW_OFFSET 90.0  # 天线基线角度
param set GPS_1_YAW 1          # 启用GPS航向
```

### 3. GPS拒止环境
```bash
# 视觉定位辅助
param set EKF2_AID_MASK 1      # 启用视觉辅助
param set EKF2_GPS_MASK 0      # 禁用GPS（可选）

# 光流定位
param set EKF2_OF_CTRL 1       # 启用光流
```

## 相关文档

- [数据融合系统](../navigation-sensor/) - GPS数据在EKF2中的融合处理
- [硬件特定配置](../hardware-specific/) - 特定硬件平台的GPS接口配置  
- [参数系统](../parameters/) - GPS相关参数的详细说明

---

本目录文档涵盖GPS导航定位系统的完整技术实现，适合导航系统工程师和高级用户参考。