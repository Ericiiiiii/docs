# PX4四旋翼控制系统架构

## 概述

PX4飞控系统采用分层控制架构，通过模块化设计实现四旋翼无人机的精确控制。整个控制系统从高层任务规划到底层电机控制，形成了完整的控制链路。

## 系统架构总体框图

```
[地面站/遥控器] --> [Commander] --> [Navigator] --> [Position Control] --> [Attitude Control] --> [Rate Control] --> [Control Allocator] --> [电机驱动]
                        ↓              ↓                ↓                    ↓                   ↓                      ↓
                   [飞行模式管理]   [路径规划]         [位置环]              [姿态环]             [角速度环]            [电机混控]
```

## 主要控制模块

### 1. Commander（指挥官模块）
**位置**: `src/modules/commander/`

**主要功能**:
- 飞行模式状态机管理
- 解锁/上锁逻辑控制  
- 故障检测与处理
- 安全监控与故障保护
- 用户输入解析（遥控器、地面站）

**关键文件**:
- `Commander.cpp` - 主控制逻辑
- `ModeManagement.cpp` - 飞行模式管理
- `failsafe/` - 故障保护框架
- `HealthAndArmingChecks/` - 健康状态与解锁检查

### 2. Navigator（导航模块）
**位置**: `src/modules/navigator/`

**主要功能**:
- 自主飞行任务执行
- 航点导航与路径规划
- 地理围栏检查
- 返航(RTL)逻辑
- 降落与精确着陆

**关键文件**:
- `navigator_main.cpp` - 导航主控制器
- `mission.cpp` - 任务执行逻辑
- `rtl.cpp` - 返航控制
- `geofence.cpp` - 地理围栏
- `takeoff.cpp/land.cpp` - 起飞/降落控制

### 3. Position Control（位置控制）
**位置**: `src/modules/mc_pos_control/`

**主要功能**:
- 外环位置控制器（Position → Velocity → Acceleration）
- 速度控制与限制
- 轨迹跟踪控制
- 起飞/降落状态管理

**控制架构**:
```
位置设定值 --> [位置控制器] --> 速度设定值 --> [速度控制器] --> 加速度设定值 --> 推力输出
     ↑                                ↑                              ↑
   当前位置                         当前速度                        当前加速度
```

**关键文件**:
- `MulticopterPositionControl.cpp` - 主位置控制器
- `PositionControl/PositionControl.cpp` - 位置控制算法
- `Takeoff/Takeoff.cpp` - 起飞逻辑

### 4. Attitude Control（姿态控制）
**位置**: `src/modules/mc_att_control/`

**主要功能**:
- 中环姿态控制器（Acceleration → Attitude → Angular Rate）
- 四元数姿态控制
- 推力矢量计算
- 角速度设定值生成

**控制架构**:
```
推力矢量 --> [姿态控制器] --> 角速度设定值
    ↑                           ↑
当前姿态                     目标姿态
```

**关键文件**:
- `mc_att_control_main.cpp` - 主姿态控制器
- `AttitudeControl/AttitudeControl.cpp` - 姿态控制算法

### 5. Rate Control（角速度控制）
**位置**: `src/modules/mc_rate_control/`

**主要功能**:
- 内环角速度控制器（Angular Rate → Torque）
- PID控制器实现
- 力矩输出计算
- 手动模式支持

**控制架构**:
```
角速度设定值 --> [PID控制器] --> 力矩输出
      ↑                          ↑
   当前角速度                   前馈控制
```

**关键文件**:
- `MulticopterRateControl.cpp` - 主角速度控制器
- PID参数配置文件

### 6. Control Allocator（控制分配）
**位置**: `src/modules/control_allocator/`

**主要功能**:
- 力矩/推力到电机信号的分配
- 电机故障处理
- 执行器效率建模
- 控制输出限制

**控制架构**:
```
[推力矢量] + [力矩矢量] --> [控制分配矩阵] --> [电机PWM信号]
```

**关键文件**:
- `ControlAllocator.cpp` - 主控制分配器
- `ActuatorEffectiveness/` - 执行器效率建模
- `ControlAllocation/` - 分配算法实现

## 控制回路分析

### 三环控制结构

PX4采用经典的三环串级控制结构：

1. **外环 - 位置控制环**
   - 输入：位置设定值、当前位置
   - 输出：速度设定值
   - 控制器：P控制器
   - 频率：~250Hz

2. **中环 - 姿态控制环**  
   - 输入：姿态设定值、当前姿态
   - 输出：角速度设定值
   - 控制器：四元数姿态控制器
   - 频率：~500Hz

3. **内环 - 角速度控制环**
   - 输入：角速度设定值、当前角速度
   - 输出：力矩控制量
   - 控制器：PID控制器
   - 频率：~1000Hz

### 控制信号流向

```
遥控器输入/任务指令
        ↓
    Commander (模式管理)
        ↓
    Navigator (路径规划)
        ↓
Position Control (位置→速度→加速度)
        ↓
Attitude Control (推力→姿态→角速度)
        ↓
Rate Control (角速度→力矩)
        ↓
Control Allocator (力矩→电机信号)
        ↓
    电机驱动输出
```

## uORB通信架构

### 主要消息类型

- `vehicle_command` - 飞行指令
- `vehicle_status` - 飞行状态
- `trajectory_setpoint` - 轨迹设定点
- `vehicle_attitude_setpoint` - 姿态设定点
- `vehicle_rates_setpoint` - 角速度设定点
- `vehicle_torque_setpoint` - 力矩设定点
- `actuator_outputs` - 执行器输出

### 消息传递流程

```
外部指令 --> Commander --> Navigator --> Position Control
                                              ↓
Actuator Outputs <-- Control Allocator <-- Rate Control <-- Attitude Control
```

## 参数系统

### 主要参数组

- **MPC_*** - 多旋翼位置控制参数
- **MC_*** - 多旋翼姿态/角速度控制参数  
- **NAV_*** - 导航控制参数
- **RTL_*** - 返航控制参数
- **COM_*** - Commander模块参数

### 关键调参点

1. **位置控制**:
   - `MPC_XY_P` - 水平位置比例增益
   - `MPC_Z_P` - 垂直位置比例增益
   - `MPC_XY_VEL_P/I/D` - 水平速度PID参数

2. **姿态控制**:
   - `MC_ROLL_P/PITCH_P/YAW_P` - 姿态比例增益
   - `MC_ROLLRATE_MAX/PITCHRATE_MAX` - 最大角速度限制

3. **角速度控制**:
   - `MC_ROLLRATE_P/I/D` - 滚转角速度PID
   - `MC_PITCHRATE_P/I/D` - 俯仰角速度PID
   - `MC_YAWRATE_P/I/D` - 偏航角速度PID

## 故障保护机制

### 故障检测
- 传感器故障检测
- 通信链路监控
- 电池电压监控
- GPS信号质量检查

### 故障处理
- 自动返航(RTL)
- 紧急降落
- 悬停模式
- 安全解锁

## 性能特点

- **实时性**: 支持1kHz控制频率
- **模块化**: 各控制环节独立，便于调试
- **可扩展**: 支持多种飞行器类型
- **鲁棒性**: 完整的故障保护机制
- **开放性**: 开源架构，便于二次开发

## 总结

PX4的四旋翼控制系统通过分层架构实现了从高层任务到底层执行的完整控制链路。三环串级控制结构保证了系统的稳定性和响应性能，模块化设计便于系统的维护和扩展。完善的故障保护机制确保了飞行安全，是一套成熟可靠的无人机控制系统。