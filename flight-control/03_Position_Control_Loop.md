# PX4位置环控制模块详解

## 概述

PX4位置环控制模块(MulticopterPositionControl)是多旋翼控制系统的外环控制器，负责将导航模块提供的位置设定点转换为姿态控制器所需的推力矢量和偏航设定值。该模块实现了位置→速度→加速度的串级控制结构。

## 模块结构

### 文件位置
- **主要源码**: `src/modules/mc_pos_control/`
- **核心文件**: `MulticopterPositionControl.cpp`, `MulticopterPositionControl.hpp`
- **控制算法**: `PositionControl/PositionControl.cpp`
- **数学库**: `PositionControl/ControlMath.cpp`

### 核心组件架构

```
Position Control模块
├── MulticopterPositionControl (主控制器)
├── PositionControl (位置控制算法)
├── ControlMath (控制数学库)
├── Takeoff (起飞控制)
├── GotoControl (定点飞行)
└── 各种参数配置文件
```

## 控制系统架构

### 三层串级控制结构

```
位置设定值 → [位置控制器] → 速度设定值 → [速度控制器] → 加速度设定值 → 推力输出
     ↑               P控制器              ↑              PID控制器           ↑
   当前位置                             当前速度                           当前加速度
```

### 数据流程图

```
trajectory_setpoint → Position Controller → vehicle_attitude_setpoint → Attitude Controller
        ↑                    ↓                         ↓
vehicle_local_position    速度设定值计算              推力矢量计算
                             ↓                         ↓
                        Speed Controller            偏航角设定值
                             ↓
                        加速度设定值计算
```

## 核心算法实现

### 主控制器类结构

```cpp
class MulticopterPositionControl : public ModuleBase<MulticopterPositionControl>, 
                                  public control::SuperBlock,
                                  public ModuleParams,
                                  public px4::ScheduledWorkItem {
private:
    // uORB订阅
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
    uORB::Subscription _vehicle_constraints_sub{ORB_ID(vehicle_constraints)};
    uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
    
    // uORB发布
    uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub;
    uORB::Publication<takeoff_status_s> _takeoff_status_pub;
    
    // 控制算法核心
    PositionControl _control;
    Takeoff _takeoff;
    
    // 状态变量
    Vector3f _position;          // 当前位置
    Vector3f _velocity;          // 当前速度
    Vector3f _acceleration;      // 当前加速度
    float _yaw;                  // 当前偏航角
    
    // 控制输出
    Vector3f _thrust_setpoint;   // 推力设定值
    float _yaw_setpoint;         // 偏航设定值
    
public:
    MulticopterPositionControl(bool vtol = false);
    
    // 主要接口
    bool init() override;
    void Run() override;
    
private:
    // 核心控制方法
    void update_trajectory_setpoint();
    void control_position();
    void control_velocity(); 
    void control_acceleration();
    void generate_attitude_setpoint();
    void publish_attitude_setpoint();
};
```

### 位置控制算法

```cpp
class PositionControl {
private:
    // 控制增益
    Vector3f _gain_pos_p;        // 位置比例增益
    Vector3f _gain_vel_p;        // 速度比例增益
    Vector3f _gain_vel_i;        // 速度积分增益
    Vector3f _gain_vel_d;        // 速度微分增益
    
    // 控制限制
    float _lim_vel_horizontal;   // 水平速度限制
    float _lim_vel_up;          // 上升速度限制  
    float _lim_vel_down;        // 下降速度限制
    float _lim_thr_min;         // 最小推力限制
    float _lim_thr_max;         // 最大推力限制
    
    // 状态变量
    Vector3f _pos;              // 当前位置
    Vector3f _vel;              // 当前速度
    Vector3f _vel_dot;          // 当前加速度
    Vector3f _acc_sp;           // 加速度设定值
    Vector3f _vel_int;          // 速度积分项
    
    // 设定值
    Vector3f _pos_sp;           // 位置设定值
    Vector3f _vel_sp;           // 速度设定值
    
public:
    // 主要控制接口
    void updateState(const PositionControlStates &states);
    void updateSetpoint(const trajectory_setpoint_s &setpoint);
    Vector3f updatePosition();
    Vector3f updateVelocity(float dt);
    Vector3f updateAcceleration(float dt);
    
    // 配置接口
    void setPositionGains(const Vector3f &P);
    void setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D);
    void setVelocityLimits(float vel_horizontal, float vel_up, float vel_down);
    void setThrustLimits(float min, float max);
};
```

## 控制算法详解

### 1. 位置控制器 (Position Controller)

#### 数学模型
```cpp
Vector3f PositionControl::updatePosition() {
    // 位置误差计算
    Vector3f pos_error = _pos_sp - _pos;
    
    // P控制器计算速度设定值
    Vector3f vel_sp_position = pos_error.emult(_gain_pos_p);
    
    // 速度限制
    vel_sp_position = constrainVelocity(vel_sp_position);
    
    return vel_sp_position;
}

Vector3f PositionControl::constrainVelocity(const Vector3f &vel_sp) {
    Vector3f vel_sp_constrained = vel_sp;
    
    // 水平速度限制
    const Vector2f vel_sp_xy(vel_sp(0), vel_sp(1));
    const float vel_sp_xy_norm = vel_sp_xy.norm();
    
    if (vel_sp_xy_norm > _lim_vel_horizontal) {
        vel_sp_constrained(0) = vel_sp(0) / vel_sp_xy_norm * _lim_vel_horizontal;
        vel_sp_constrained(1) = vel_sp(1) / vel_sp_xy_norm * _lim_vel_horizontal;
    }
    
    // 垂直速度限制
    if (vel_sp(2) > _lim_vel_up) {
        vel_sp_constrained(2) = _lim_vel_up;
    } else if (vel_sp(2) < -_lim_vel_down) {
        vel_sp_constrained(2) = -_lim_vel_down;
    }
    
    return vel_sp_constrained;
}
```

### 2. 速度控制器 (Velocity Controller)

#### PID控制实现
```cpp
Vector3f PositionControl::updateVelocity(float dt) {
    // 速度误差计算
    Vector3f vel_error = _vel_sp - _vel;
    
    // 比例项
    Vector3f acc_sp_proportional = vel_error.emult(_gain_vel_p);
    
    // 积分项 (带抗积分饱和)
    Vector3f acc_sp_integral = _vel_int.emult(_gain_vel_i);
    
    // 微分项 (基于加速度反馈)
    Vector3f acc_sp_derivative = -_vel_dot.emult(_gain_vel_d);
    
    // PID输出组合
    Vector3f acc_sp = acc_sp_proportional + acc_sp_integral + acc_sp_derivative;
    
    // 重力补偿
    acc_sp(2) += CONSTANTS_ONE_G;
    
    // 更新积分项
    updateVelocityIntegral(vel_error, dt);
    
    return acc_sp;
}

void PositionControl::updateVelocityIntegral(const Vector3f &vel_error, float dt) {
    // 积分项更新
    Vector3f vel_int_new = _vel_int + vel_error * dt;
    
    // 积分限制 (防止积分饱和)
    for (int i = 0; i < 3; i++) {
        if (_gain_vel_i(i) > FLT_EPSILON) {
            float integral_limit = _lim_thr_max / _gain_vel_i(i);
            vel_int_new(i) = math::constrain(vel_int_new(i), -integral_limit, integral_limit);
        }
    }
    
    _vel_int = vel_int_new;
}
```

### 3. 推力分配与姿态计算

#### 推力矢量计算
```cpp
void MulticopterPositionControl::generate_attitude_setpoint() {
    // 1. 获取加速度设定值
    Vector3f acc_sp = _control.getAccelerationSetpoint();
    
    // 2. 计算推力大小
    float thrust_setpoint = acc_sp.length();
    
    // 3. 推力限制
    thrust_setpoint = math::constrain(thrust_setpoint, _param_mpc_thr_min.get(), _param_mpc_thr_max.get());
    
    // 4. 计算期望推力方向 (机体Z轴方向)
    Vector3f body_z_sp;
    if (thrust_setpoint > FLT_EPSILON) {
        body_z_sp = acc_sp.normalized();
    } else {
        body_z_sp = Vector3f(0.f, 0.f, 1.f);  // 默认向上
    }
    
    // 5. 计算机体X轴期望方向 (根据偏航角)
    Vector3f body_x_sp = Vector3f(cosf(_yaw_setpoint), sinf(_yaw_setpoint), 0.f);
    
    // 6. 通过叉积计算机体Y轴
    Vector3f body_y_sp = body_z_sp % body_x_sp;
    body_y_sp.normalize();
    
    // 7. 重新计算正交的机体X轴
    body_x_sp = body_y_sp % body_z_sp;
    
    // 8. 构造旋转矩阵
    matrix::Dcmf R_sp;
    R_sp.col(0) = body_x_sp;
    R_sp.col(1) = body_y_sp;
    R_sp.col(2) = body_z_sp;
    
    // 9. 转换为四元数
    Quatf q_sp(R_sp);
    
    // 10. 发布姿态设定值
    vehicle_attitude_setpoint_s att_sp{};
    q_sp.copyTo(att_sp.q_d);
    att_sp.thrust_body[2] = -thrust_setpoint;  // 负Z方向推力
    att_sp.yaw_sp_move_rate = _yaw_sp_move_rate;
    
    _vehicle_attitude_setpoint_pub.publish(att_sp);
}
```

## 特殊飞行模式处理

### 1. 起飞控制 (Takeoff)

```cpp
class Takeoff {
private:
    enum class TakeoffState {
        TAKEOFF_RAMP,      // 爬升阶段
        TAKEOFF_HOVER,     // 悬停阶段
        TAKEOFF_FINISHED   // 完成
    };
    
    TakeoffState _takeoff_state{TakeoffState::TAKEOFF_RAMP};
    hrt_abstime _takeoff_start_time;
    float _takeoff_reference_altitude;
    
public:
    void generateSetpoints(Vector3f &position_setpoint, Vector3f &velocity_setpoint);
    bool isTakeoffComplete() const;
    void updateTakeoffState(const Vector3f &position, bool landed);
    
private:
    void generateRampSetpoints(Vector3f &pos_sp, Vector3f &vel_sp);
    void generateHoverSetpoints(Vector3f &pos_sp, Vector3f &vel_sp);
};

void Takeoff::generateSetpoints(Vector3f &position_setpoint, Vector3f &velocity_setpoint) {
    switch (_takeoff_state) {
        case TakeoffState::TAKEOFF_RAMP:
            generateRampSetpoints(position_setpoint, velocity_setpoint);
            break;
            
        case TakeoffState::TAKEOFF_HOVER:
            generateHoverSetpoints(position_setpoint, velocity_setpoint);
            break;
            
        case TakeoffState::TAKEOFF_FINISHED:
            // 起飞完成，使用外部设定值
            break;
    }
}
```

### 2. 降落控制

```cpp
void MulticopterPositionControl::control_landing() {
    // 1. 检查降落条件
    if (!_vehicle_land_detected_sub.get().landed && _control_mode.flag_control_auto_enabled) {
        
        // 2. 降落速度控制
        Vector3f vel_sp_land;
        vel_sp_land(0) = 0.0f;  // 无水平速度
        vel_sp_land(1) = 0.0f;
        vel_sp_land(2) = _param_mpc_land_speed.get();  // 下降速度
        
        // 3. 软着陆逻辑
        if (_position(2) - _reference_position(2) < _param_mpc_land_alt2.get()) {
            // 接近地面时减速
            float land_speed_factor = math::constrain(
                (_position(2) - _reference_position(2)) / _param_mpc_land_alt2.get(), 
                0.1f, 1.0f
            );
            vel_sp_land(2) *= land_speed_factor;
        }
        
        // 4. 应用降落速度设定
        _control.setVelocitySetpoint(vel_sp_land);
    }
}
```

### 3. 悬停控制

```cpp
void MulticopterPositionControl::control_hover() {
    // 1. 保持当前位置
    if (!_control.inputValid()) {
        // 如果没有外部输入，保持当前位置
        _pos_sp = _position;
        _vel_sp = Vector3f(0.f, 0.f, 0.f);
    }
    
    // 2. 位置锁定
    _control.setPositionSetpoint(_pos_sp);
    _control.setVelocitySetpoint(_vel_sp);
    
    // 3. 高度保持优化
    if (_control_mode.flag_control_altitude_enabled) {
        // 使用气压高度融合进行精确高度控制
        float baro_altitude = getBaometricAltitude();
        if (fabsf(baro_altitude - _position(2)) < 0.5f) {
            _pos_sp(2) = baro_altitude;
        }
    }
}
```

## 约束处理与安全机制

### 1. 速度约束

```cpp
void PositionControl::applyVelocityConstraints() {
    // 水平速度约束
    Vector2f vel_sp_xy(_vel_sp(0), _vel_sp(1));
    float vel_sp_xy_norm = vel_sp_xy.norm();
    
    if (vel_sp_xy_norm > _lim_vel_horizontal) {
        _vel_sp(0) = _vel_sp(0) / vel_sp_xy_norm * _lim_vel_horizontal;
        _vel_sp(1) = _vel_sp(1) / vel_sp_xy_norm * _lim_vel_horizontal;
    }
    
    // 垂直速度约束
    if (_vel_sp(2) > _lim_vel_up) {
        _vel_sp(2) = _lim_vel_up;
    } else if (_vel_sp(2) < -_lim_vel_down) {
        _vel_sp(2) = -_lim_vel_down;
    }
}
```

### 2. 推力约束

```cpp
Vector3f PositionControl::applyThrustConstraints(const Vector3f &acc_sp) {
    Vector3f acc_sp_constrained = acc_sp;
    
    // 计算推力大小
    float thrust_sp = acc_sp.length();
    
    // 推力限制
    if (thrust_sp < _lim_thr_min) {
        // 最小推力约束
        if (thrust_sp > FLT_EPSILON) {
            acc_sp_constrained = acc_sp.normalized() * _lim_thr_min;
        } else {
            acc_sp_constrained = Vector3f(0.f, 0.f, _lim_thr_min);
        }
        
    } else if (thrust_sp > _lim_thr_max) {
        // 最大推力约束
        acc_sp_constrained = acc_sp.normalized() * _lim_thr_max;
    }
    
    return acc_sp_constrained;
}
```

### 3. 倾斜角限制

```cpp
void MulticopterPositionControl::applyTiltConstraints() {
    // 计算期望倾斜角
    Vector3f acc_sp = _control.getAccelerationSetpoint();
    Vector2f acc_sp_xy(acc_sp(0), acc_sp(1));
    float tilt_sp = atan2f(acc_sp_xy.norm(), acc_sp(2));
    
    // 倾斜角限制
    float tilt_max = math::radians(_param_mpc_tilt_max.get());
    
    if (tilt_sp > tilt_max) {
        // 重新分配加速度以满足倾斜角约束
        float acc_xy_max = acc_sp(2) * tanf(tilt_max);
        float acc_xy_norm = acc_sp_xy.norm();
        
        if (acc_xy_norm > FLT_EPSILON) {
            float reduction_factor = acc_xy_max / acc_xy_norm;
            acc_sp(0) *= reduction_factor;
            acc_sp(1) *= reduction_factor;
        }
        
        _control.setAccelerationSetpoint(acc_sp);
    }
}
```

## 自适应控制与参数调节

### 1. 悬停推力估计

```cpp
class HoverThrustEstimator {
private:
    float _hover_thrust{0.5f};
    float _accel_innov_lpf;
    AlphaFilter<float> _filter;
    
public:
    void update(float dt, float thrust_setpoint, const Vector3f &acceleration) {
        // 计算加速度创新
        float accel_innov = acceleration(2) + CONSTANTS_ONE_G - thrust_setpoint;
        
        // 低通滤波
        _accel_innov_lpf = _filter.apply(accel_innov, dt);
        
        // 悬停推力估计更新
        if (fabsf(_accel_innov_lpf) < 2.0f) {  // 稳定条件下更新
            _hover_thrust += _accel_innov_lpf * 0.01f * dt;
            _hover_thrust = math::constrain(_hover_thrust, 0.1f, 0.9f);
        }
    }
    
    float getHoverThrust() const { return _hover_thrust; }
};
```

### 2. 响应性参数自动调节

```cpp
void MulticopterPositionControl::updateResponsivenessParameters() {
    float responsiveness = _param_sys_vehicle_resp.get();
    
    if (responsiveness >= 0.f) {
        // 使其在低端不那么敏感
        responsiveness = responsiveness * responsiveness;
        
        // 根据响应性调节参数
        float acc_hor = math::lerp(1.f, 15.f, responsiveness);
        float acc_hor_max = math::lerp(2.f, 15.f, responsiveness);
        float man_y_max = math::lerp(80.f, 450.f, responsiveness);
        
        _param_mpc_acc_hor.set(acc_hor);
        _param_mpc_acc_hor_max.set(acc_hor_max);
        _param_mpc_man_y_max.set(man_y_max);
        
        // 高响应性时设置更快的偏航响应
        if (responsiveness > 0.6f) {
            _param_mpc_man_y_tau.set(0.f);
        }
    }
}
```

## 关键参数配置

### 位置控制参数
```cpp
// 位置环增益
MPC_XY_P         // 水平位置比例增益 [0.5-2.0]
MPC_Z_P          // 垂直位置比例增益 [0.5-2.0]

// 速度环增益  
MPC_XY_VEL_P     // 水平速度比例增益 [0.05-0.3]
MPC_XY_VEL_I     // 水平速度积分增益 [0.01-0.1] 
MPC_XY_VEL_D     // 水平速度微分增益 [0.001-0.02]
MPC_Z_VEL_P      // 垂直速度比例增益 [0.1-0.4]
MPC_Z_VEL_I      // 垂直速度积分增益 [0.01-0.1]
MPC_Z_VEL_D      // 垂直速度微分增益 [0.001-0.02]
```

### 速度限制参数
```cpp
// 水平速度限制
MPC_XY_VEL_MAX   // 最大水平速度 [3-20 m/s]
MPC_XY_CRUISE    // 巡航水平速度 [2-15 m/s]

// 垂直速度限制
MPC_Z_VEL_MAX_UP // 最大上升速度 [1-8 m/s]
MPC_Z_VEL_MAX_DN // 最大下降速度 [1-4 m/s]

// 加速度限制
MPC_ACC_HOR      // 水平加速度限制 [1-15 m/s²]
MPC_ACC_HOR_MAX  // 最大水平加速度 [2-15 m/s²]
MPC_ACC_UP_MAX   // 最大上升加速度 [2-15 m/s²]
MPC_ACC_DOWN_MAX // 最大下降加速度 [2-10 m/s²]
```

### 推力参数
```cpp
// 推力限制
MPC_THR_MIN      // 最小推力 [0.05-0.2]
MPC_THR_MAX      // 最大推力 [0.8-1.0]
MPC_THR_HOVER    // 悬停推力 [0.2-0.8]

// 倾斜角限制
MPC_TILT_MAX     // 最大倾斜角 [10-90度]
MPC_MAN_TILT_MAX // 手动模式最大倾斜角 [10-90度]
```

### 特殊模式参数
```cpp
// 起飞参数
MPC_TKO_SPEED    // 起飞速度 [0.5-5 m/s]
MPC_TKO_RAMP_T   // 起飞爬升时间 [1-10 s]

// 降落参数
MPC_LAND_SPEED   // 降落速度 [0.5-2 m/s]
MPC_LAND_ALT1    // 降落减速高度1 [2-10 m]
MPC_LAND_ALT2    // 降落减速高度2 [1-5 m]
```

## 性能监控与调试

### 控制性能指标
```cpp
struct PositionControlPerformance {
    float position_error_norm;      // 位置误差范数
    float velocity_error_norm;      // 速度误差范数
    float thrust_setpoint_norm;     // 推力设定值范数
    float tilt_angle;              // 当前倾斜角
    float computation_time_us;      // 计算时间(微秒)
};

void MulticopterPositionControl::updatePerformanceMetrics() {
    _perf_metrics.position_error_norm = (_pos_sp - _position).norm();
    _perf_metrics.velocity_error_norm = (_vel_sp - _velocity).norm();
    _perf_metrics.thrust_setpoint_norm = _thrust_setpoint.norm();
    _perf_metrics.tilt_angle = atan2f(Vector2f(_thrust_setpoint(0), _thrust_setpoint(1)).norm(), 
                                     fabsf(_thrust_setpoint(2)));
}
```

### 调试输出
```cpp
void MulticopterPositionControl::printDebugInfo() {
    PX4_INFO("Position Control Debug:");
    PX4_INFO("  Pos Error: %.2f m", _perf_metrics.position_error_norm);
    PX4_INFO("  Vel Error: %.2f m/s", _perf_metrics.velocity_error_norm);
    PX4_INFO("  Thrust: %.2f", _perf_metrics.thrust_setpoint_norm);
    PX4_INFO("  Tilt: %.1f deg", math::degrees(_perf_metrics.tilt_angle));
    PX4_INFO("  Computation: %lu us", _perf_metrics.computation_time_us);
}
```

## 故障处理与安全

### 传感器故障处理
```cpp
void MulticopterPositionControl::handleSensorFailure() {
    // GPS故障处理
    if (!_vehicle_local_position_sub.get().xy_valid) {
        // 切换到仅高度控制模式
        _control_mode_flags.flag_control_position_enabled = false;
        _control_mode_flags.flag_control_velocity_enabled = false;
        _control_mode_flags.flag_control_altitude_enabled = true;
    }
    
    // 气压计故障处理
    if (!_vehicle_local_position_sub.get().z_valid) {
        // 切换到仅水平位置控制
        _control_mode_flags.flag_control_altitude_enabled = false;
        _control_mode_flags.flag_control_climb_rate_enabled = false;
    }
}
```

### 控制饱和处理
```cpp
void PositionControl::handleControlSaturation() {
    // 检测推力饱和
    if (_thrust_setpoint.norm() >= _lim_thr_max * 0.95f) {
        // 减少积分项以防止积分饱和
        _vel_int *= 0.95f;
        
        // 优先保证垂直控制
        if (fabsf(_acc_sp(2)) > _lim_thr_max * 0.8f) {
            float reduction_factor = _lim_thr_max * 0.8f / fabsf(_acc_sp(2));
            _acc_sp(0) *= reduction_factor;
            _acc_sp(1) *= reduction_factor;
        }
    }
}
```

## 总结

PX4位置环控制模块是多旋翼控制系统的关键外环控制器，通过精确的串级PID控制实现了从位置到加速度的平滑转换。模块具有完善的约束处理、自适应调节和故障保护机制，能够在各种飞行条件下提供稳定可靠的位置控制性能。合理的参数配置和调节对于获得最佳控制效果至关重要。