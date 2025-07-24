# Fihawk FC-V1 传感器采集周期与控制频率分析

本文档详细分析了Fihawk FC-V1飞控硬件上各传感器的采集周期、数据融合频率、控制频率和电机输出控制频率。

## 目录

- [传感器采集周期](#传感器采集周期)
- [数据融合频率](#数据融合频率)
- [控制系统频率](#控制系统频率)
- [电机输出控制频率](#电机输出控制频率)
- [完整控制链路频率总结](#完整控制链路频率总结)
- [控制分配器分析](#控制分配器分析)
- [系统特点](#系统特点)

## 传感器采集周期

### IMU传感器

Fihawk FC-V1配备了多个IMU传感器，分布在不同的SPI总线上：

#### ICM20689 (SPI1, SPI6)
```cpp
// Sensor Configuration
static constexpr float FIFO_SAMPLE_DT{1e6f / 8000.f};     // 125μs
static constexpr float GYRO_RATE{1e6f / FIFO_SAMPLE_DT};  // 8000 Hz gyro
static constexpr float ACCEL_RATE{GYRO_RATE / SAMPLES_PER_TRANSFER}; // 4000 Hz accel
```

#### ICM20649 (如果使用)
```cpp
// Sensor Configuration  
static constexpr float FIFO_SAMPLE_DT{1e6f / 9000.f};     // 111μs
static constexpr float GYRO_RATE{1e6f / FIFO_SAMPLE_DT}; // 9000 Hz gyro
static constexpr float ACCEL_RATE{GYRO_RATE / SAMPLES_PER_TRANSFER}; // 4500 Hz accel
```

#### ICM42688P (SPI4)
```cpp
// Sensor Configuration
static constexpr float FIFO_SAMPLE_DT{1e6f / 8000.f};     // 125μs
static constexpr float GYRO_RATE{1e6f / FIFO_SAMPLE_DT};  // 8000 Hz
static constexpr float ACCEL_RATE{1e6f / FIFO_SAMPLE_DT}; // 8000 Hz
```

### 气压计

#### MS5611 (SPI4, SPI6)
```cpp
/*
 * Maximum internal conversion time for OSR 1024 is 2.28 ms. We set an update
 * rate of 100Hz which is be very safe not to read the ADC before the
 * conversion finished
 */
#define MS5611_CONVERSION_INTERVAL	10000	/* microseconds = 100Hz */
```

### 磁力计

#### RM3100 (SPI1)
- 典型采集频率：~75Hz (约13.3ms周期)

## 数据融合频率

### EKF2估计器
```cpp
int32_t filter_update_interval_us{10000}; ///< filter update interval = 10ms = 100Hz
```

### IMU数据积分频率
```cpp
/**
* IMU integration rate.
* Recommended to set this to a multiple of the estimator update period (currently 10 ms for ekf2).
*/
PARAM_DEFINE_INT32(IMU_INTEG_RATE, 200); // 默认200Hz = 5ms周期
```

### 传感器模块调度
```cpp
// backup schedule as a watchdog timeout
ScheduleDelayed(10_ms); // 传感器模块10ms调度周期 = 100Hz
```

## 控制系统频率

### 工作队列优先级配置
```cpp
static constexpr wq_config_t rate_ctrl{"wq:rate_ctrl", 3150, 0}; // PX4 inner loop highest priority

// PX4 att/pos controllers, highest priority after sensors.
static constexpr wq_config_t nav_and_controllers{"wq:nav_and_controllers", 2240, -13};
```

### 控制器频率特性

#### 角速度控制器（最高优先级）
- 运行在 `rate_ctrl` 工作队列
- **事件驱动**：由陀螺仪数据更新触发
- 实际频率：**~8000Hz**（跟随IMU陀螺仪采样率）
- dt约束：0.125ms - 20ms

#### 姿态控制器
- 运行在 `nav_and_controllers` 工作队列  
- **事件驱动**：由姿态估计更新触发
- 实际频率：**~100Hz**（跟随EKF2更新率）
- dt约束：0.2ms - 20ms

#### 位置控制器
- 运行在 `nav_and_controllers` 工作队列
- **事件驱动**：由本地位置更新触发
- 实际频率：**~100Hz**（跟随EKF2更新率）
- dt约束：2ms - 40ms

## 电机输出控制频率

### 控制分配器
```cpp
ControlAllocator::ControlAllocator() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl), // 最高优先级
```

### PWM输出频率
```yaml
pwm_timer_param:
    description:
        short: Output Protocol Configuration for ${label}
    type: enum
    default: 400  # 默认400Hz PWM频率
    values:
        -5: DShot150
        -4: DShot300  
        -3: DShot600
        -2: DShot1200
        -1: OneShot
        50: PWM 50 Hz
        100: PWM 100 Hz
        200: PWM 200 Hz
        400: PWM 400 Hz    # 默认值
```

## 完整控制链路频率总结

| 控制环节 | 频率 | 触发方式 | 工作队列 | 备注 |
|---------|------|----------|----------|------|
| **IMU原始数据** | 8000-9000Hz | 硬件中断 | SPI工作队列 | 陀螺仪/加速度计 |
| **IMU数据积分** | 200Hz (5ms) | 定时调度 | INS工作队列 | 可配置参数 |
| **EKF2状态估计** | 100Hz (10ms) | 定时调度 | INS工作队列 | 姿态/位置融合 |
| **位置控制器** | ~100Hz | 事件驱动 | nav_and_controllers | 位置→姿态设定值 |
| **姿态控制器** | ~100Hz | 事件驱动 | nav_and_controllers | 姿态→角速度设定值 |
| **角速度控制器** | ~8000Hz | 事件驱动 | rate_ctrl (最高优先级) | 角速度→力矩输出 |
| **控制分配器** | ~8000Hz | 事件驱动 | rate_ctrl (最高优先级) | 力矩→电机指令 |
| **PWM输出** | 400Hz | 定时调度 | rate_ctrl | 电机PWM信号 |

## 控制分配器分析

基于 `control_allocator status` 输出的实际配置分析：

### 效率矩阵 (Effectiveness Matrix)
```
  | 0      | 1      | 2      | 3      | 4      | 5
 0|-6.50000  6.50000  0.32500  0        0       -6.50000
 1| 6.50000 -6.50000  0.32500  0        0       -6.50000
 2| 6.50000  6.50000 -0.32500  0        0       -6.50000
 3|-6.50000 -6.50000 -0.32500  0        0       -6.50000
```

### 电机布局 (X型四旋翼)
```
    0(CW)     1(CCW)
        \   /
         \ /
          X
         / \
        /   \
   3(CCW)   2(CW)
```

### 各轴控制分析

**Roll轴 (列0)**：
- 电机0: -6.5 (左倾时减小推力)
- 电机1: +6.5 (左倾时增大推力)  
- 电机2: +6.5 (左倾时增大推力)
- 电机3: -6.5 (左倾时减小推力)

**Pitch轴 (列1)**：
- 电机0: +6.5 (前倾时增大推力)
- 电机1: -6.5 (前倾时减小推力)
- 电机2: +6.5 (前倾时增大推力)  
- 电机3: -6.5 (前倾时减小推力)

**Yaw轴 (列2)**：
- 电机0: +0.325 (顺时针旋转，产生逆时针反扭矩)
- 电机1: +0.325 (逆时针旋转，产生顺时针反扭矩)
- 电机2: -0.325 (顺时针旋转，产生逆时针反扭矩)
- 电机3: -0.325 (逆时针旋转，产生顺时针反扭矩)

### 性能统计
- **总循环数**: 887,038次
- **总运行时间**: 16,316,331μs (约16.3秒)
- **平均执行时间**: 18.39μs
- **最小执行时间**: 15μs  
- **最大执行时间**: 129μs
- **RMS**: 2.321μs

## 系统特点

### 1. 高频内环控制
- 角速度控制和控制分配运行在最高优先级
- 频率达8kHz，确保极高的响应速度

### 2. 中频外环控制
- 姿态和位置控制运行在100Hz
- 满足稳定性和精度要求

### 3. 事件驱动架构
- 控制器由数据更新触发，而非固定时间间隔
- 提高系统响应性和效率

### 4. 优先级分层设计
- 确保内环控制有最高实时性
- 避免低优先级任务影响关键控制环路

### 5. 硬件限制适配
- 电机输出受硬件PWM频率限制
- 通常使用400Hz PWM频率

### 6. 标准X型四旋翼配置
- 对角电机旋转方向相同
- Roll/Pitch轴效率系数为6.5，表示较大的力臂
- Yaw轴效率系数仅0.325，符合多旋翼特点

## 结论

Fihawk FC-V1的控制系统设计体现了现代飞控系统的先进理念：
- **高频采样**：IMU传感器8-9kHz采样确保数据精度
- **分层控制**：从8kHz内环到100Hz外环的合理频率分配
- **事件驱动**：提高系统响应性和资源利用率
- **优先级管理**：确保关键控制环路的实时性

这种设计确保了飞控系统具有极高的响应速度和控制精度，特别是在快速机动和抗干扰方面表现优异。

## 参考文件

- `src/modules/control_allocator/ControlAllocator.cpp`
- `src/modules/mc_rate_control/MulticopterRateControl.cpp`
- `src/modules/mc_att_control/mc_att_control_main.cpp`
- `src/modules/mc_pos_control/MulticopterPositionControl.cpp`
- `src/drivers/imu/invensense/icm20689/ICM20689.hpp`
- `src/drivers/imu/invensense/icm42688p/ICM42688P.hpp`
- `src/drivers/barometer/ms5611/MS5611.hpp`
- `platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp`
