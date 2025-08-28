# PX4路径规划与导航模块详解

## 概述

PX4导航模块(Navigator)是自主飞行的核心模块，负责任务规划、路径生成、航点导航、地理围栏管理等高级导航功能。该模块将高层次的飞行任务转换为具体的位置设定点，为位置控制器提供目标轨迹。

## 模块结构

### 文件位置
- **主要源码**: `src/modules/navigator/`
- **核心文件**: `navigator_main.cpp`, `navigator.h`
- **导航模式**: `mission.cpp`, `rtl.cpp`, `takeoff.cpp`, `land.cpp`, `loiter.cpp`

### 核心组件架构

```
Navigator模块
├── Mission (任务执行)
├── RTL (返航模式)  
├── Takeoff (自动起飞)
├── Land (自动降落)
├── Loiter (悬停等待)
├── Precision Land (精确降落)
├── Geofence (地理围栏)
├── GeofenceBreachAvoidance (围栏规避)
└── VTOL Takeoff (垂直起降)
```

## 导航架构总览

### 数据流程图

```
MAVLink任务 → Mission Manager → Navigation Modes → Position Setpoints → Position Controller
     ↓             ↓                   ↓                    ↓
  任务存储      模式切换            轨迹生成             控制执行
```

### 关键数据结构

```cpp
// 位置设定点三元组
struct position_setpoint_triplet_s {
    position_setpoint_s previous;    // 前一个设定点
    position_setpoint_s current;     // 当前设定点  
    position_setpoint_s next;        // 下一个设定点
};

// 单个位置设定点
struct position_setpoint_s {
    float lat, lon;           // 纬度，经度
    float alt;                // 高度
    float x, y, z;           // 本地坐标系位置
    float vx, vy, vz;        // 速度设定值
    float yaw;               // 偏航角设定值
    float yawspeed;          // 偏航角速度
    uint8_t type;            // 设定点类型
    bool valid;              // 有效性标志
};
```

## 主要导航模式

### 1. Mission (任务执行)

**文件位置**: `mission.cpp`, `mission.h`

#### 功能特性
- MAVLink任务协议支持
- 复杂航点序列执行
- 任务状态持久化
- 断点续飞功能

#### 核心类结构
```cpp
class Mission : public MissionBase {
private:
    static constexpr int32_t DEFAULT_MISSION_CACHE_SIZE = 10;
    
    // 任务状态
    bool _need_mission_save{false};
    mission_s _mission_sub_cache;
    
    // 关键方法
    void on_activation() override;
    void on_inactive() override;
    bool isLanding();
    void check_mission_valid(bool force_check = false);
    
public:
    Mission(Navigator *navigator);
    
    // 任务执行控制
    void update() override;
    bool set_current_mission_index(uint16_t index);
    void save_mission_state();
};
```

#### 任务项类型支持
```cpp
// MAVLink任务指令类型
MAV_CMD_NAV_WAYPOINT           // 航点飞行
MAV_CMD_NAV_LOITER_UNLIM       // 无限悬停
MAV_CMD_NAV_LOITER_TIME        // 定时悬停
MAV_CMD_NAV_LAND               // 降落
MAV_CMD_NAV_TAKEOFF            // 起飞
MAV_CMD_NAV_DELAY              // 延迟等待
MAV_CMD_DO_JUMP                // 条件跳转
MAV_CMD_DO_CHANGE_SPEED        // 改变速度
MAV_CMD_DO_SET_HOME            // 设置起始点
MAV_CMD_DO_SET_SERVO           // 控制舵机
MAV_CMD_DO_LAND_START          // 降落序列开始
MAV_CMD_IMAGE_START_CAPTURE    // 开始拍照
MAV_CMD_VIDEO_START_CAPTURE    // 开始录像
```

#### 任务执行流程
```cpp
void Mission::update() {
    // 1. 检查任务有效性
    if (!check_mission_valid()) return;
    
    // 2. 获取当前任务项
    mission_item_s current_item;
    if (!get_mission_item(current_index, current_item)) return;
    
    // 3. 处理不同类型任务项
    switch (current_item.nav_cmd) {
        case MAV_CMD_NAV_WAYPOINT:
            handle_waypoint(current_item);
            break;
        case MAV_CMD_NAV_LOITER_TIME:
            handle_loiter(current_item);
            break;
        case MAV_CMD_NAV_LAND:
            handle_landing(current_item);
            break;
    }
    
    // 4. 检查任务项完成条件
    if (is_mission_item_reached()) {
        advance_to_next_mission_item();
    }
    
    // 5. 发布位置设定点
    publish_position_setpoint();
}
```

### 2. RTL (Return to Launch)

**文件位置**: `rtl.cpp`, `rtl.h`

#### RTL策略类型
```cpp
enum class RTLType {
    RTL_DIRECT,              // 直接返航
    RTL_MISSION_FAST,        // 快速任务返航
    RTL_MISSION_FAST_REVERSE,// 反向快速返航
    RTL_DIRECT_MISSION_LAND  // 直接任务降落返航
};
```

#### RTL状态机
```cpp
enum class RTLState {
    CLIMBING,      // 爬升阶段
    RETURN,        // 返航阶段  
    DESCEND,       // 下降阶段
    LOITER,        // 悬停阶段
    LAND,          // 降落阶段
    LANDED         // 已降落
};
```

#### RTL执行逻辑
```cpp
void RTL::update() {
    switch (_rtl_state) {
        case RTLState::CLIMBING:
            // 爬升到返航高度
            if (current_alt >= rtl_return_alt) {
                _rtl_state = RTLState::RETURN;
            }
            break;
            
        case RTLState::RETURN:
            // 水平返回起始点
            if (distance_to_home < acceptance_radius) {
                _rtl_state = RTLState::DESCEND;
            }
            break;
            
        case RTLState::DESCEND:
            // 下降到降落高度
            if (current_alt <= rtl_descend_alt) {
                _rtl_state = RTLState::LAND;
            }
            break;
            
        case RTLState::LAND:
            // 执行自动降落
            if (landed_state.landed) {
                _rtl_state = RTLState::LANDED;
            }
            break;
    }
}
```

### 3. Takeoff (自动起飞)

**文件位置**: `takeoff.cpp`, `takeoff.h`

#### 起飞状态机
```cpp
enum class TakeoffState {
    TAKEOFF_RAMP,           // 起飞爬升
    TAKEOFF_HOVER,          // 悬停稳定
    TAKEOFF_FINISHED        // 起飞完成
};
```

#### 起飞逻辑实现
```cpp
void Takeoff::update() {
    switch (_takeoff_state) {
        case TakeoffState::TAKEOFF_RAMP:
            // 垂直爬升到目标高度
            setpoint.z = target_takeoff_altitude;
            setpoint.vz = takeoff_speed;
            
            if (current_alt >= target_alt - acceptance_radius) {
                _takeoff_state = TakeoffState::TAKEOFF_HOVER;
            }
            break;
            
        case TakeoffState::TAKEOFF_HOVER:
            // 悬停等待稳定
            setpoint.z = target_takeoff_altitude;
            setpoint.vz = 0.0f;
            
            if (hover_time_elapsed > hover_time_threshold) {
                _takeoff_state = TakeoffState::TAKEOFF_FINISHED;
            }
            break;
    }
}
```

### 4. Land (自动降落)

**文件位置**: `land.cpp`, `land.h`

#### 降落状态机
```cpp
enum class LandState {
    LAND_DESCEND,          // 下降阶段
    LAND_FINAL,            // 最终降落
    LAND_FINISHED          // 降落完成
};
```

#### 降落控制逻辑
```cpp
void Land::update() {
    switch (_land_state) {
        case LandState::LAND_DESCEND:
            // 受控下降
            setpoint.vz = -descend_speed;
            
            if (current_alt <= final_approach_alt) {
                _land_state = LandState::LAND_FINAL;
            }
            break;
            
        case LandState::LAND_FINAL:
            // 最终降落阶段
            setpoint.vz = -final_descend_speed;
            
            if (landed_state.landed) {
                _land_state = LandState::LAND_FINISHED;
                // 自动解除解锁
                send_disarm_command();
            }
            break;
    }
}
```

### 5. Precision Land (精确降落)

**文件位置**: `precland.cpp`, `precland.h`

#### 精确降落传感器支持
```cpp
enum class PrecisionLandMode {
    NONE,                  // 无精确降落
    OPPORTUNISTIC,         // 机会性精确降落
    REQUIRED              // 强制精确降落
};
```

#### 视觉标识跟踪
```cpp
void PrecisionLand::update() {
    // 1. 获取视觉目标信息
    if (irlock_report_valid()) {
        target_pos = get_target_position_from_irlock();
    } else if (landing_target_valid()) {
        target_pos = get_target_position_from_mavlink();
    }
    
    // 2. 计算修正量
    if (target_pos.valid) {
        Vector2f correction = calculate_landing_correction(target_pos);
        
        // 3. 应用位置修正
        apply_position_correction(correction);
    }
    
    // 4. 执行降落序列
    execute_land_sequence();
}
```

## 地理围栏系统

### 文件位置
- **核心文件**: `geofence.cpp`, `geofence.h`
- **规避算法**: `GeofenceBreachAvoidance/`

### 围栏类型支持

```cpp
enum class GeofenceType {
    INCLUSION_CIRCLE,      // 包含圆形区域
    EXCLUSION_CIRCLE,      // 排除圆形区域  
    INCLUSION_POLYGON,     // 包含多边形区域
    EXCLUSION_POLYGON,     // 排除多边形区域
    ALTITUDE_MAX,          // 最大高度限制
    ALTITUDE_MIN           // 最小高度限制
};
```

### 围栏检查逻辑

```cpp
class Geofence {
private:
    struct GeofenceData {
        float vertices[GEOFENCE_MAX_VERTICES * 2];  // 顶点坐标
        uint16_t vertex_count;
        GeofenceType type;
        bool valid;
    };
    
public:
    // 主要接口
    bool checkGeofence(const vehicle_global_position_s &global_position);
    GeofenceResult getGeofenceResult() const;
    void loadGeofenceFromFile();
    
private:
    // 检查算法
    bool isInsidePolygon(const Vector2d &point, const GeofenceData &fence);
    bool isInsideCircle(const Vector2d &point, const Vector2d &center, float radius);
    float getDistanceToClosestBoundary(const Vector2d &point);
};
```

### 围栏违规处理

```cpp
enum class GeofenceAction {
    NONE,                  // 无动作
    WARNING,               // 仅警告
    HOLD,                  // 保持位置
    RETURN,                // 返航
    TERMINATE,             // 终止飞行
    LAND                   // 立即降落
};

void GeofenceBreachAvoidance::update() {
    GeofenceResult result = _geofence.getGeofenceResult();
    
    if (result.geofence_violated) {
        switch (geofence_action) {
            case GeofenceAction::HOLD:
                generateHoldSetpoint();
                break;
                
            case GeofenceAction::RETURN:
                activateRTLMode();
                break;
                
            case GeofenceAction::LAND:
                activateLandMode();
                break;
        }
    }
}
```

## 任务可行性检查

### 文件位置
- **检查器**: `MissionFeasibility/FeasibilityChecker.cpp`
- **测试**: `MissionFeasibility/FeasibilityCheckerTest.cpp`

### 检查项目

```cpp
class FeasibilityChecker {
public:
    struct FeasibilityResult {
        bool feasible;
        uint16_t warning_vector;    // 警告位向量
        uint16_t error_vector;      // 错误位向量
    };
    
    // 主要检查方法
    FeasibilityResult checkMissionFeasibility(const mission_s &mission);
    
private:
    // 具体检查项
    bool checkDistanceBetweenWaypoints(const mission_item_s &item1, const mission_item_s &item2);
    bool checkAltitudeConstraints(const mission_item_s &item);
    bool checkTurnRadius(const mission_item_s &prev, const mission_item_s &curr, const mission_item_s &next);
    bool checkBatteryConsumption(const mission_s &mission);
    bool checkGeofenceCompliance(const mission_s &mission);
    bool checkFixedWingConstraints(const mission_item_s &item);
    bool checkMulticopterConstraints(const mission_item_s &item);
};
```

### 检查算法示例

```cpp
bool FeasibilityChecker::checkTurnRadius(
    const mission_item_s &prev, 
    const mission_item_s &curr, 
    const mission_item_s &next) {
    
    // 计算转弯角度
    Vector2f vec1(curr.lat - prev.lat, curr.lon - prev.lon);
    Vector2f vec2(next.lat - curr.lat, next.lon - curr.lon);
    
    float turn_angle = acosf(vec1.dot(vec2) / (vec1.length() * vec2.length()));
    
    // 计算最小转弯半径
    float min_turn_radius = pow(cruise_speed, 2) / (CONSTANTS_ONE_G * tanf(max_bank_angle));
    
    // 检查是否满足转弯约束
    float required_radius = cruise_speed / tanf(turn_angle / 2);
    
    return required_radius >= min_turn_radius;
}
```

## 轨迹生成与优化

### 平滑轨迹生成

```cpp
class TrajectoryGenerator {
private:
    struct TrajectoryPoint {
        Vector3f position;
        Vector3f velocity;
        Vector3f acceleration;
        float yaw;
        float timestamp;
    };
    
public:
    // 生成平滑轨迹
    std::vector<TrajectoryPoint> generateSmoothTrajectory(
        const std::vector<mission_item_s> &waypoints,
        float max_velocity,
        float max_acceleration
    );
    
private:
    // B样条曲线生成
    TrajectoryPoint evaluateBSpline(float t, const std::vector<Vector3f> &control_points);
    
    // 约束优化
    void optimizeTrajectoryConstraints(std::vector<TrajectoryPoint> &trajectory);
};
```

### 动态路径调整

```cpp
void Navigator::adjustPathForObstacles() {
    // 1. 检测障碍物
    ObstacleData obstacles = getObstacleData();
    
    // 2. 计算规避路径
    if (obstacles.detected) {
        Vector3f avoidance_waypoint = calculateAvoidanceWaypoint(
            current_position, 
            target_position, 
            obstacles
        );
        
        // 3. 更新路径
        updateCurrentSetpoint(avoidance_waypoint);
    }
}
```

## 关键参数配置

### 任务执行参数
```cpp
// 任务相关参数
MIS_TAKEOFF_ALT     // 起飞高度
MIS_TAKEOFF_REQ     // 起飞要求
MIS_YAW_TMT         // 偏航超时时间
MIS_YAW_ERR         // 偏航误差阈值
MIS_DIST_1WP        // 第一个航点距离
MIS_DIST_WPS        // 航点间距离

// 航点接受参数
NAV_ACC_RAD         // 航点接受半径
NAV_FW_ALTL_RAD     // 固定翼高度接受半径
NAV_MC_ALT_RAD      // 多旋翼高度接受半径
```

### RTL参数
```cpp
// 返航参数
RTL_RETURN_ALT      // 返航高度
RTL_DESCEND_ALT     // 下降开始高度
RTL_LAND_DELAY      // 降落延迟时间
RTL_MIN_DIST        // 最小返航距离
RTL_TYPE            // 返航类型选择
RTL_FMT_ALT         // 格式化返航高度

// 返航策略参数
RTL_CONE_ANG        // 返航圆锥角度
RTL_PLD_MD          // 精确降落模式
RTL_HDG_MD          // 返航朝向模式
```

### 地理围栏参数
```cpp
// 围栏配置
GF_ACTION           // 围栏违规动作
GF_ALTMODE          // 高度模式(相对/绝对)
GF_COUNT            // 围栏顶点数量
GF_SOURCE           // 围栏数据源
GF_MAX_HOR_DIST     // 最大水平距离
GF_MAX_VER_DIST     // 最大垂直距离

// 围栏预测
GF_PREDICT          // 启用围栏预测
GF_PREDICT_TIME     // 预测时间窗口
```

## 性能优化与监控

### 计算性能监控

```cpp
class NavigatorPerformance {
private:
    perf_counter_t _loop_perf;
    perf_counter_t _mission_perf;
    perf_counter_t _geofence_perf;
    
public:
    void startPerformanceTimer(perf_counter_t counter) {
        perf_begin(counter);
    }
    
    void endPerformanceTimer(perf_counter_t counter) {
        perf_end(counter);
    }
    
    void printPerformanceStats() {
        PX4_INFO("Navigator performance:");
        PX4_INFO("  Loop: %lu us", perf_mean(_loop_perf));
        PX4_INFO("  Mission: %lu us", perf_mean(_mission_perf));
        PX4_INFO("  Geofence: %lu us", perf_mean(_geofence_perf));
    }
};
```

### 内存管理

```cpp
// 任务缓存管理
class MissionCache {
private:
    static constexpr size_t CACHE_SIZE = 10;
    mission_item_s _cache[CACHE_SIZE];
    uint16_t _cache_start_index;
    uint16_t _cache_size;
    
public:
    bool getMissionItem(uint16_t index, mission_item_s &item);
    void updateCache(uint16_t start_index);
    void clearCache();
};
```

## 故障处理与恢复

### 导航故障类型

```cpp
enum class NavigationFailure {
    GPS_LOSS,              // GPS信号丢失
    MISSION_INVALID,       // 任务无效
    GEOFENCE_VIOLATION,    // 地理围栏违规
    OBSTACLE_DETECTED,     // 障碍物检测
    COMMUNICATION_LOSS,    // 通信丢失
    SENSOR_FAILURE,        // 传感器故障
    WAYPOINT_UNREACHABLE   // 航点不可达
};
```

### 故障恢复策略

```cpp
void Navigator::handleNavigationFailure(NavigationFailure failure) {
    switch (failure) {
        case NavigationFailure::GPS_LOSS:
            // 切换到惯性导航模式
            activatePositionHoldMode();
            break;
            
        case NavigationFailure::MISSION_INVALID:
            // 终止任务，切换到悬停
            abortMission();
            activateLoiterMode();
            break;
            
        case NavigationFailure::GEOFENCE_VIOLATION:
            // 执行围栏规避
            activateGeofenceRecovery();
            break;
            
        case NavigationFailure::OBSTACLE_DETECTED:
            // 执行障碍物规避
            activateObstacleAvoidance();
            break;
    }
}
```

## 开发接口与扩展

### 自定义导航模式

```cpp
class CustomNavigationMode : public NavigatorModeBase {
public:
    CustomNavigationMode(Navigator *navigator) : NavigatorModeBase(navigator) {}
    
    void on_activation() override {
        // 模式激活时的初始化
    }
    
    void on_inactive() override {
        // 模式失活时的清理
    }
    
    void update() override {
        // 主要更新逻辑
        generateCustomSetpoints();
        publishPositionSetpoint(_pos_sp_triplet);
    }
    
private:
    void generateCustomSetpoints();
};
```

### 插件式扩展接口

```cpp
class NavigatorPlugin {
public:
    virtual ~NavigatorPlugin() = default;
    
    virtual void initialize() = 0;
    virtual void update() = 0;
    virtual void handleCommand(const vehicle_command_s &cmd) = 0;
    virtual bool isActive() const = 0;
};

// 插件管理器
class PluginManager {
    std::vector<std::unique_ptr<NavigatorPlugin>> _plugins;
    
public:
    void registerPlugin(std::unique_ptr<NavigatorPlugin> plugin);
    void updateAllPlugins();
};
```

## 总结

PX4导航模块是一个功能强大、架构清晰的路径规划与导航系统。它支持复杂的任务执行、多种导航模式、地理围栏约束和故障恢复机制。模块化的设计使其易于扩展和定制，为各种自主飞行应用提供了坚实的基础。通过精确的轨迹生成和实时的路径调整，确保了飞机能够安全、高效地完成各种飞行任务。