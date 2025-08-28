# PX4飞行模式管理模块详解

## 概述

PX4飞行模式管理模块(Commander)是整个飞控系统的核心决策模块，负责管理飞机的飞行状态、模式切换、解锁逻辑和安全监控。该模块实现了复杂的状态机逻辑，确保飞机在各种条件下的安全飞行。

## 模块结构

### 文件位置
- **主要源码**: `src/modules/commander/`
- **核心文件**: `Commander.cpp`, `ModeManagement.cpp`
- **状态机定义**: `px4_custom_mode.h`

### 关键组件

```
Commander模块
├── 飞行模式管理 (ModeManagement)
├── 解锁状态管理 (Arming)  
├── 故障保护系统 (Failsafe)
├── 健康检查 (HealthAndArmingChecks)
├── 安全监控 (Safety)
└── 用户意图解析 (UserModeIntention)
```

## 飞行模式体系

### 主要飞行模式分类

PX4定义了以下主要飞行模式类型:

#### 1. 手动模式 (MANUAL)
```cpp
PX4_CUSTOM_MAIN_MODE_MANUAL
```
- **描述**: 直接遥控器控制，无稳定辅助
- **控制方式**: 遥控器输入直接映射到执行器输出
- **适用场景**: 高级飞手、特技飞行

#### 2. 稳定模式 (STABILIZED)
```cpp
PX4_CUSTOM_MAIN_MODE_STABILIZED
```
- **描述**: 姿态稳定，但无位置保持
- **控制方式**: 遥控器控制角速度，系统自动稳定姿态
- **适用场景**: 新手练习、室内飞行

#### 3. 特技模式 (ACRO)
```cpp
PX4_CUSTOM_MAIN_MODE_ACRO
```
- **描述**: 角速度控制模式
- **控制方式**: 遥控器直接控制角速度设定值
- **适用场景**: 竞速飞行、特技表演

#### 4. 高度控制模式 (ALTCTL)
```cpp
PX4_CUSTOM_MAIN_MODE_ALTCTL
```
- **描述**: 保持高度，水平方向手动控制
- **控制方式**: 垂直方向位置控制，水平方向姿态控制
- **适用场景**: 半自主飞行、航拍

#### 5. 位置控制模式 (POSCTL)
```cpp
PX4_CUSTOM_MAIN_MODE_POSCTL
```
- **描述**: 三维位置保持与控制
- **控制方式**: 遥控器控制速度设定值
- **子模式**:
  - `PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL` - 标准位置控制
  - `PX4_CUSTOM_SUB_MODE_POSCTL_ORBIT` - 绕点环绕模式
  - `PX4_CUSTOM_SUB_MODE_POSCTL_SLOW` - 慢速精确模式

#### 6. 自主模式 (AUTO)
```cpp
PX4_CUSTOM_MAIN_MODE_AUTO
```
- **描述**: 全自主飞行模式
- **子模式详解**:

```cpp
// 待机模式
PX4_CUSTOM_SUB_MODE_AUTO_READY      // 解锁准备状态

// 飞行阶段
PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF    // 自动起飞
PX4_CUSTOM_SUB_MODE_AUTO_MISSION    // 任务执行  
PX4_CUSTOM_SUB_MODE_AUTO_LOITER     // 悬停等待
PX4_CUSTOM_SUB_MODE_AUTO_RTL        // 返航模式
PX4_CUSTOM_SUB_MODE_AUTO_LAND       // 自动降落

// 高级功能
PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET  // 目标跟踪
PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND       // 精确降落
PX4_CUSTOM_SUB_MODE_AUTO_VTOL_TAKEOFF   // VTOL起飞

// 外部模式
PX4_CUSTOM_SUB_MODE_EXTERNAL1-8     // 自定义外部模式
```

#### 7. 离板控制模式 (OFFBOARD)
```cpp
PX4_CUSTOM_MAIN_MODE_OFFBOARD
```
- **描述**: 外部计算机控制模式
- **控制方式**: 通过MAVLink接收外部设定值
- **适用场景**: 机器视觉、AI控制、研究开发

#### 8. 终止模式 (TERMINATION)
```cpp
PX4_CUSTOM_MAIN_MODE_TERMINATION
```
- **描述**: 紧急终止飞行
- **控制方式**: 立即切断所有电机
- **触发条件**: 严重故障、地理围栏违规

## 模式管理架构

### 模式执行器 (Mode Executors)

```cpp
class ModeExecutors {
private:
    struct ModeExecutor {
        bool valid;
        uint8_t owned_nav_state;
        // 执行器状态信息
    };
    
    ModeExecutor _mode_executors[MAX_NUM];
    
public:
    int addExecutor(const ModeExecutor &executor);
    void removeExecutor(int id);
    bool hasFreeExecutors() const;
};
```

**功能**:
- 管理不同模式的执行器实例
- 处理模式间的切换逻辑
- 维护模式执行状态

### 外部模式管理

```cpp
class Modes {
private:
    struct Mode {
        bool valid;
        char name[16];
        uint8_t nav_state;
    };
    
    Mode _modes[MAX_NUM];
    
public:
    uint8_t addExternalMode(const Mode &mode);
    bool hasFreeExternalModes() const;
};
```

**功能**:
- 支持自定义外部飞行模式
- 动态模式注册与管理
- 模式持久化存储

## 状态机管理

### 飞行状态定义

PX4使用导航状态(nav_state)来定义飞机的飞行状态:

```cpp
// 主要导航状态
vehicle_status_s::NAVIGATION_STATE_MANUAL        // 手动模式
vehicle_status_s::NAVIGATION_STATE_ALTCTL        // 高度控制
vehicle_status_s::NAVIGATION_STATE_POSCTL        // 位置控制  
vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION  // 自动任务
vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER   // 自动悬停
vehicle_status_s::NAVIGATION_STATE_AUTO_RTL      // 返航
vehicle_status_s::NAVIGATION_STATE_ACRO          // 特技模式
vehicle_status_s::NAVIGATION_STATE_OFFBOARD      // 离板控制
vehicle_status_s::NAVIGATION_STATE_STAB          // 稳定模式
vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF  // 自动起飞
vehicle_status_s::NAVIGATION_STATE_AUTO_LAND     // 自动降落
vehicle_status_s::NAVIGATION_STATE_TERMINATION   // 终止模式
```

### 模式切换逻辑

```cpp
// 模式切换判断流程
bool modeCanBeSet(uint8_t new_nav_state) {
    // 1. 检查预条件
    if (!checkPreConditions(new_nav_state)) return false;
    
    // 2. 检查传感器要求  
    if (!checkSensorRequirements(new_nav_state)) return false;
    
    // 3. 检查解锁状态
    if (!checkArmingState(new_nav_state)) return false;
    
    // 4. 检查故障保护状态
    if (!checkFailsafeState(new_nav_state)) return false;
    
    return true;
}
```

## 解锁管理 (Arming)

### 解锁状态
```cpp
// 解锁状态定义
enum arming_state_t {
    vehicle_status_s::ARMING_STATE_INIT,
    vehicle_status_s::ARMING_STATE_STANDBY,     // 待机
    vehicle_status_s::ARMING_STATE_ARMED,       // 已解锁
    vehicle_status_s::ARMING_STATE_STANDBY_ERROR, // 错误状态
    vehicle_status_s::ARMING_STATE_SHUTDOWN,    // 关闭
    vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE, // 空中恢复
};
```

### 解锁条件检查

```cpp
// 关键解锁检查项
class HealthAndArmingChecks {
public:
    // 传感器检查
    bool accelerometerCheck();
    bool gyroCheck(); 
    bool magnetometerCheck();
    bool baroCheck();
    
    // 系统检查  
    bool batteryCheck();
    bool estimatorCheck();
    bool manualControlCheck();
    bool missionCheck();
    
    // 安全检查
    bool geofenceCheck();
    bool homePositionCheck();
    bool armPermissionCheck();
};
```

## 故障保护系统

### 故障保护框架

```cpp
class FailsafeFramework {
public:
    enum class Action {
        None,
        Warn,           // 警告
        FallbackPosCtl, // 回退到位置控制
        FallbackAltCtl, // 回退到高度控制  
        FallbackStab,   // 回退到稳定模式
        Hold,           // 保持位置
        ReturnToLaunch, // 返航
        Land,           // 降落
        DescendToLand,  // 下降降落
        Disarm,         // 解除解锁
        Terminate       // 终止飞行
    };
    
    struct State {
        bool valid;
        Action action;
        Cause cause;
    };
};
```

### 故障类型与处理

```cpp
// 主要故障类型
enum class FailsafeType {
    BATTERY_LOW,           // 电池电量低
    BATTERY_CRITICAL,      // 电池电量严重不足
    GPS_LOSS,             // GPS信号丢失
    RC_LOSS,              // 遥控信号丢失  
    DATALINK_LOSS,        // 数据链路丢失
    GEOFENCE_BREACH,      // 地理围栏违规
    MISSION_FAILURE,      // 任务执行失败
    VTOL_TRANSITION_FAIL, // VTOL转换失败
    WIND_TOO_HIGH,        // 风速过大
    FLIGHT_TIME_LIMIT,    // 飞行时间限制
};

// 故障处理策略
struct FailsafeAction {
    FailsafeType type;
    Action immediate_action;  // 立即动作
    Action delayed_action;    // 延迟动作  
    uint32_t delay_ms;       // 延迟时间
};
```

## 用户输入处理

### 遥控器输入

```cpp
// 遥控器通道映射
enum RC_CHANNELS {
    RC_CHANNEL_ROLL = 0,        // 滚转
    RC_CHANNEL_PITCH,           // 俯仰  
    RC_CHANNEL_THROTTLE,        // 油门
    RC_CHANNEL_YAW,             // 偏航
    RC_CHANNEL_MODE,            // 模式开关
    RC_CHANNEL_RETURN,          // 返航开关
    RC_CHANNEL_POSCTL,          // 位置控制开关
    RC_CHANNEL_LOITER,          // 悬停开关
    RC_CHANNEL_OFFBOARD,        // 离板模式开关
    RC_CHANNEL_KILLSWITCH,      // 急停开关
};
```

### MAVLink指令处理

```cpp
// 主要MAVLink指令
void handleMAVLinkCommand(const vehicle_command_s &cmd) {
    switch (cmd.command) {
        case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
            // 起飞指令处理
            break;
            
        case vehicle_command_s::VEHICLE_CMD_NAV_LAND:
            // 降落指令处理
            break;
            
        case vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
            // 返航指令处理
            break;
            
        case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM:
            // 解锁/解除解锁指令
            break;
            
        case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE:
            // 模式设置指令
            break;
    }
}
```

## 关键参数配置

### 飞行模式参数
```cpp
// 模式切换参数
COM_FLTMODE1-6     // 遥控器飞行模式映射
COM_RCOVR_TH       // 遥控器超控阈值

// 故障保护参数  
COM_RC_LOSS_T      // 遥控丢失时间阈值
COM_DL_LOSS_T      // 数据链路丢失时间阈值
COM_LOW_BAT_ACT    // 低电量故障保护动作
COM_FLIGHT_UUID    // 飞行唯一标识符

// 解锁参数
COM_ARM_WO_GPS     // 无GPS解锁许可
COM_ARM_EKF_AB     // EKF解锁检查
COM_ARM_IMU_ACC    // 加速度计解锁检查
COM_ARM_IMU_GYR    // 陀螺仪解锁检查
```

### 安全参数
```cpp
// 地理围栏参数
GF_ACTION          // 地理围栏违规动作
GF_MAX_HOR_DIST    // 最大水平距离
GF_MAX_VER_DIST    // 最大垂直距离

// 返航参数  
RTL_RETURN_ALT     // 返航高度
RTL_DESCEND_ALT    // 下降开始高度
RTL_LAND_DELAY     // 降落延迟时间
```

## 工作流程

### 模式切换流程

```
用户输入 → 模式请求 → 预条件检查 → 健康检查 → 故障保护检查 → 模式切换 → 状态发布
```

### 解锁流程

```
解锁请求 → 系统健康检查 → 传感器校准检查 → 安全条件检查 → 解锁许可 → 电机解锁
```

### 故障处理流程

```
故障检测 → 故障分类 → 选择处理策略 → 执行故障保护动作 → 状态通知 → 日志记录
```

## 性能特点

1. **实时响应**: 快速模式切换，典型响应时间<100ms
2. **安全优先**: 多重安全检查，确保飞行安全
3. **容错能力**: 完善的故障保护机制
4. **扩展性**: 支持自定义外部模式
5. **兼容性**: 支持多种遥控器和地面站协议

## 开发接口

### 添加自定义模式
```cpp
// 注册自定义模式
Modes::Mode custom_mode;
strcpy(custom_mode.name, "CUSTOM_MODE");
custom_mode.nav_state = vehicle_status_s::NAVIGATION_STATE_EXTERNAL1;
modes.addExternalMode(custom_mode);
```

### 故障保护扩展
```cpp
// 添加自定义故障保护
failsafe.registerFailsafeAction(
    FailsafeType::CUSTOM_FAILURE,
    FailsafeFramework::Action::Land,
    5000 // 5秒延迟
);
```

## 总结

PX4飞行模式管理模块是一个复杂而完善的状态管理系统，它不仅负责飞行模式的切换和管理，还承担着系统安全监控、故障保护等关键功能。通过分层的架构设计和完善的安全机制，确保了飞机在各种飞行条件下的安全可靠运行。