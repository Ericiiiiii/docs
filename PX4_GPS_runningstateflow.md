# PX4 GPS 运行逻辑详解

## 概述

PX4中的GPS系统是一个复杂的多层架构，支持多种GPS协议和双GPS配置。本文档详细介绍GPS从启动到数据发布的完整运行逻辑。

## 1. GPS模块架构

### 1.1 核心组件
- **GPS主驱动** (`src/drivers/gps/gps.cpp`): 主要的GPS驱动程序
- **GPS Helper类** (`src/drivers/gps/devices/src/gps_helper.h`): GPS协议解析器的基类
- **协议驱动**: UBX, MTK, ASHTECH, EMLID_REACH, FEMTOMES, NMEA等具体协议实现

### 1.2 支持的GPS协议
```cpp
enum class gps_driver_mode_t {
    None = 0,
    UBX,        // u-blox (主要支持)
    MTK,        // MediaTek
    ASHTECH,    // Ashtech
    EMLIDREACH, // Emlid Reach
    FEMTOMES,   // Femtomes
    NMEA,       // 标准NMEA协议
};
```

## 2. GPS启动流程

### 2.1 模块启动配置 (`module.yaml`)
```yaml
serial_config:
    # 次要GPS必须先启动
    - command: set DUAL_GPS_ARGS "-e ${SERIAL_DEV} -g p:${BAUD_PARAM}"
      port_config_param:
        name: GPS_2_CONFIG
        group: GPS
      label: Secondary GPS

    # 主GPS启动
    - command: gps start -d ${SERIAL_DEV} -b p:${BAUD_PARAM} ${DUAL_GPS_ARGS}
      port_config_param:
        name: GPS_1_CONFIG
        group: GPS
        default: GPS1
      label: Main GPS
```

### 2.2 GPS实例管理
PX4支持双GPS配置：
- **Instance::Main**: 主GPS (实例0)
- **Instance::Secondary**: 次要GPS (实例1)

## 3. GPS主运行循环 (`GPS::run()`)

### 3.1 参数初始化
```cpp
void GPS::run() {
    // 1. 读取GPS相关参数
    param_t handle = param_find("GPS_YAW_OFFSET");
    float heading_offset = 0.f;

    // 2. 读取UBX动态模型参数
    int32_t gps_ubx_dynmodel = 7; // 默认: 空中<2g加速度

    // 3. 读取UBX模式参数 (Normal/MovingBase/Rover等)
    GPSDriverUBX::UBXMode ubx_mode{GPSDriverUBX::UBXMode::Normal};

    // 4. 读取GNSS系统配置
    int32_t gnssSystemsParam = static_cast<int32_t>(GPSHelper::GNSSSystemsMask::RECEIVER_DEFAULTS);
}
```

### 3.2 主循环逻辑
```cpp
while (!should_exit()) {
    // 1. 清理旧的helper实例
    if (_helper != nullptr) {
        delete (_helper);
        _helper = nullptr;
    }

    // 2. 配置串口/SPI接口
    if ((_interface == GPSHelper::Interface::UART) && (!_uart.isOpen())) {
        // 配置UART端口和波特率
    }

    // 3. 根据模式创建对应的GPS helper
    switch (_mode) {
        case gps_driver_mode_t::UBX:
            _helper = new GPSDriverUBX(...);
            break;
        // 其他协议...
    }

    // 4. 配置GPS设备
    if (_helper && _helper->configure(_baudrate, gpsConfig) == 0) {
        // 5. 数据接收和处理循环
        while ((helper_ret = _helper->receive(receive_timeout)) > 0 && !should_exit()) {
            if (helper_ret & 1) {
                publish();  // 发布GPS位置数据
            }
            if (_p_report_sat_info && (helper_ret & 2)) {
                publishSatelliteInfo();  // 发布卫星信息
            }

            // 6. 更新速率统计
            // 7. 处理RTCM注入数据
            // 8. 处理设备重置
        }
    }
}
```

## 4. GPS数据流

### 4.1 数据接收流程
1. **串口/SPI读取**: 从GPS设备读取原始数据
2. **协议解析**: 各协议helper解析特定格式的消息
3. **数据验证**: 检查数据完整性和有效性
4. **格式转换**: 转换为PX4标准的sensor_gps_s格式

### 4.2 uORB主题发布
GPS驱动发布以下uORB主题：

#### sensor_gps (主要GPS数据)
```cpp
struct sensor_gps_s {
    uint64_t timestamp;           // 系统时间戳
    uint64_t timestamp_sample;    // 采样时间戳
    uint32_t device_id;           // 设备ID

    // 位置信息
    double latitude_deg;          // 纬度 (度)
    double longitude_deg;         // 经度 (度)
    double altitude_msl_m;        // 海拔高度 (米)
    double altitude_ellipsoid_m;  // 椭球高度 (米)

    // 精度信息
    float eph;                    // 水平位置精度
    float epv;                    // 垂直位置精度
    float hdop, vdop;            // 精度因子

    // 速度信息
    float vel_m_s;               // 地面速度
    float vel_n_m_s, vel_e_m_s, vel_d_m_s; // NED速度
    float cog_rad;               // 航向角

    // 状态信息
    uint8_t fix_type;            // 定位类型 (2D/3D/RTK等)
    uint8_t satellites_used;     // 使用的卫星数量
    uint8_t jamming_state;       // 干扰状态
    uint8_t spoofing_state;      // 欺骗状态

    // RTK相关
    float rtcm_injection_rate;   // RTCM注入速率
    uint8_t selected_rtcm_instance; // RTCM实例
};
```

#### satellite_info (卫星信息)
```cpp
struct satellite_info_s {
    uint64_t timestamp;
    uint8_t count;               // 卫星数量
    // 每颗卫星的详细信息...
};
```

#### sensor_gnss_relative (相对定位)
用于RTK相对定位数据。

## 5. GPS协议支持

### 5.1 UBX协议 (主要支持)
- **支持设备**: u-blox 6/7/8/9系列
- **特殊功能**: RTK, 双天线测向, 移动基站
- **配置**: 支持完整的UBX配置命令

### 5.2 其他协议
- **MTK**: MediaTek GPS芯片
- **NMEA**: 标准NMEA 0183协议
- **ASHTECH**: Ashtech GPS接收机
- **FEMTOMES**: Femtomes高精度GPS

## 6. 高级功能

### 6.1 RTK支持
- **RTCM消息注入**: 通过gps_inject_data主题接收RTCM校正数据
- **基站模式**: 支持作为RTK基站运行
- **移动基站**: 支持移动基站配置

### 6.2 双GPS配置
- **主次GPS**: 自动选择最佳GPS作为主要导航源
- **冗余**: 提供GPS故障时的备份
- **测向**: 双天线GPS可提供航向信息

### 6.3 故障检测
- **信号干扰检测**: 监测GPS信号干扰
- **欺骗检测**: 检测GPS欺骗攻击
- **健康状态监控**: 持续监控GPS设备状态

## 7. 参数配置

### 7.1 主要参数
- `GPS_1_CONFIG` / `GPS_2_CONFIG`: GPS串口配置
- `GPS_YAW_OFFSET`: 双天线GPS航向偏移
- `GPS_UBX_DYNMODEL`: UBX动态模型 (0-9)
- `GPS_UBX_MODE`: UBX工作模式
- `GPS_1_GNSS` / `GPS_2_GNSS`: GNSS系统选择

### 7.2 GNSS系统配置
```cpp
enum class GNSSSystemsMask : int32_t {
    RECEIVER_DEFAULTS = 0,
    ENABLE_GPS =        1 << 0,  // GPS
    ENABLE_SBAS =       1 << 1,  // SBAS
    ENABLE_GALILEO =    1 << 2,  // Galileo
    ENABLE_BEIDOU =     1 << 3,  // BeiDou
    ENABLE_GLONASS =    1 << 4   // GLONASS
};
```

## 8. 调试和诊断

### 8.1 GPS状态检查
```bash
gps status          # 查看GPS状态
listener sensor_gps # 监听GPS数据
```

### 8.2 通信调试
- GPS驱动支持通信数据转储功能
- 可以记录与GPS设备的完整通信过程
- 支持RTCM消息监控

## 9. 性能特性

### 9.1 更新频率
- **位置更新**: 通常1-10Hz
- **速度更新**: 与位置更新同步
- **卫星信息**: 较低频率更新

### 9.2 延迟特性
- **串口延迟**: 取决于波特率和消息大小
- **处理延迟**: 协议解析和数据转换时间
- **发布延迟**: uORB消息发布时间

## 10. 关键实现细节

### 10.1 数据接收机制
GPS驱动使用轮询机制从设备读取数据：

<augment_code_snippet path="src/drivers/gps/gps.cpp" mode="EXCERPT">
````cpp
int GPS::pollOrRead(uint8_t *buf, size_t buf_length, int timeout) {
    // 根据接口类型选择读取方式
    if (_interface == GPSHelper::Interface::UART) {
        return _uart.readAtLeast(buf, buf_length, 1, timeout);
    }
#ifdef __PX4_LINUX
    else if (_interface == GPSHelper::Interface::SPI) {
        // SPI读取实现
    }
#endif
}
````
</augment_code_snippet>

### 10.2 协议自动检测
GPS驱动支持自动检测GPS协议类型：

<augment_code_snippet path="src/drivers/gps/gps.cpp" mode="EXCERPT">
````cpp
switch (_mode) {
case gps_driver_mode_t::None:
    _mode = gps_driver_mode_t::UBX;  // 默认尝试UBX

case gps_driver_mode_t::UBX:
    _helper = new GPSDriverUBX(_interface, &GPS::callback, this,
                               &_report_gps_pos, _p_report_sat_info,
                               gps_ubx_dynmodel, heading_offset,
                               f9p_uart2_baudrate, ubx_mode);
    break;
}
````
</augment_code_snippet>

### 10.3 回调机制
GPS helper通过回调函数与主驱动通信：

<augment_code_snippet path="src/drivers/gps/gps.cpp" mode="EXCERPT">
````cpp
int GPS::callback(GPSCallbackType type, void *data1, int data2, void *user) {
    GPS *gps = (GPS *)user;
    switch (type) {
    case GPSCallbackType::readDeviceData:
        return gps->pollOrRead((uint8_t *)data1, data2, *((int *)data1));
    case GPSCallbackType::writeDeviceData:
        return gps->_uart.write((uint8_t *)data1, data2);
    case GPSCallbackType::setBaudrate:
        return gps->setBaudrate(data2);
    case GPSCallbackType::gotRTCMMessage:
        gps->publishRTCMCorrections((uint8_t *)data1, data2);
        break;
    }
    return 0;
}
````
</augment_code_snippet>

### 10.4 RTCM数据注入
支持实时差分校正数据注入：

<augment_code_snippet path="src/drivers/gps/gps.cpp" mode="EXCERPT">
````cpp
void GPS::handleInjectDataTopic() {
    // 检查是否有新的RTCM数据需要注入
    for (auto &inject_data_sub : _orb_inject_data_sub) {
        gps_inject_data_s inject_data;
        if (inject_data_sub.update(&inject_data)) {
            injectData(inject_data.data, inject_data.len);
        }
    }
}
````
</augment_code_snippet>

## 11. 错误处理和恢复

### 11.1 设备重置机制
```cpp
void GPS::schedule_reset(GPSRestartType restart_type) {
    _scheduled_reset.store((int)restart_type);
}

void GPS::reset_if_scheduled() {
    int restart_type = _scheduled_reset.load();
    if (restart_type != (int)GPSRestartType::None) {
        if (_helper) {
            _helper->reset((GPSRestartType)restart_type);
        }
        _scheduled_reset.store((int)GPSRestartType::None);
    }
}
```

### 11.2 健康状态监控
- **超时检测**: 如果长时间没有收到有效数据，标记为不健康
- **数据验证**: 检查GPS数据的合理性
- **信号质量**: 监控信号强度和精度

## 12. 性能优化

### 12.1 缓冲区管理
- **读取缓冲区**: 150字节默认大小，可配置
- **消息队列**: uORB消息队列深度优化
- **内存管理**: 动态分配卫星信息结构

### 12.2 CPU使用优化
- **轮询超时**: 根据GPS更新频率调整超时时间
- **批量处理**: 一次处理多个GPS消息
- **优先级**: GPS任务使用适当的优先级

## 13. 总结

PX4的GPS系统是一个高度模块化、可扩展的导航子系统，具有以下特点：

1. **多协议支持**: 支持主流GPS协议，以UBX为主
2. **高精度定位**: 支持RTK差分定位，精度可达厘米级
3. **冗余设计**: 双GPS配置提供故障保护
4. **实时性**: 低延迟的数据处理和发布
5. **可配置性**: 丰富的参数配置选项
6. **诊断能力**: 完善的状态监控和调试功能

这个GPS系统为PX4提供了可靠、精确的位置和导航信息，是自主飞行的关键组件。
