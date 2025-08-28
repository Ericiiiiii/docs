# PX4姿态环控制模块详解

## 概述

PX4姿态环控制模块(MulticopterAttitudeControl)是多旋翼控制系统的中环控制器，负责将位置控制器输出的推力矢量转换为角速度设定值。该模块实现了基于四元数的姿态控制算法，具有良好的全姿态稳定性和快速响应特性。

## 模块结构

### 文件位置
- **主要源码**: `src/modules/mc_att_control/`
- **核心文件**: `mc_att_control_main.cpp`, `mc_att_control.hpp`
- **控制算法**: `AttitudeControl/AttitudeControl.cpp`
- **数学库**: `AttitudeControl/AttitudeControlMath.hpp`

### 核心组件架构

```
Attitude Control模块
├── MulticopterAttitudeControl (主控制器)
├── AttitudeControl (姿态控制算法)
├── AttitudeControlMath (姿态数学库)
└── 参数配置文件
```

## 控制系统架构

### 姿态控制结构图

```
推力矢量设定值 → [姿态解算] → 期望姿态四元数 → [姿态控制器] → 角速度设定值 → Rate Controller
       ↑               ↓                    ↑                 ↓
    位置控制器输出     偏航角设定值          当前姿态四元数      角速度前馈
```

### 数据流程图

```
vehicle_attitude_setpoint → Attitude Controller → vehicle_rates_setpoint → Rate Controller
            ↑                      ↓                       ↓
        推力矢量计算             四元数姿态控制           角速度计算
            ↑                      ↓                       ↓
    vehicle_attitude           姿态误差计算             前馈补偿
```

## 核心算法实现

### 主控制器类结构

```cpp
class MulticopterAttitudeControl : public ModuleBase<MulticopterAttitudeControl>, 
                                  public ModuleParams,
                                  public px4::WorkItem {
private:
    // uORB订阅
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
    uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
    uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    
    // uORB发布
    uORB::Publication<vehicle_rates_setpoint_s> _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
    uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub;
    
    // 核心姿态控制器
    AttitudeControl _attitude_control;
    
    // 状态变量
    Quatf _attitude_q;              // 当前姿态四元数
    Vector3f _angular_velocity;     // 当前角速度
    
    // 设定值
    Quatf _attitude_setpoint_q;     // 姿态设定四元数
    Vector3f _rates_setpoint;       // 角速度设定值
    float _thrust_setpoint;         // 推力设定值
    float _yaw_setpoint;           // 偏航角设定值
    
    // 手动控制相关
    float _man_tilt_max;           // 手动模式最大倾斜角
    SlewRate<float> _manual_throttle_minimum;
    SlewRate<float> _manual_throttle_maximum;
    
public:
    MulticopterAttitudeControl(bool vtol = false);
    
    // 主要接口
    bool init() override;
    void Run() override;
    
private:
    // 核心控制方法
    void control_attitude();
    void control_attitude_auto();
    void control_attitude_manual();
    void generate_attitude_setpoint();
    void publish_rates_setpoint();
    void publish_attitude_setpoint();
    
    // 参数更新
    void parameters_updated();
};
```

### 姿态控制算法核心

```cpp
class AttitudeControl {
private:
    // 控制增益
    Vector3f _proportional_gain;    // 比例增益 [roll, pitch, yaw]
    Vector3f _rate_limit;          // 角速度限制
    float _yaw_w;                  // 偏航权重
    
    // 设定值
    Quatf _attitude_setpoint_q;     // 姿态设定四元数
    float _yawspeed_setpoint;       // 偏航角速度设定值
    
public:
    // 主要控制接口
    void setAttitudeSetpoint(const Quatf &q_sp, float yawspeed_sp = NAN);
    void setProportionalGain(const Vector3f &proportional_gain, float yaw_weight = 1.f);
    void setRateLimit(const Vector3f &rate_limit);
    
    // 控制更新
    Vector3f update(const Quatf &q) const;
    
private:
    // 内部计算方法
    Vector3f calculateRateSetpoint(const Quatf &q_error) const;
    void addYawFeedforward(Vector3f &rates_sp, const Quatf &q) const;
};
```

## 姿态控制算法详解

### 1. 四元数姿态控制

#### 数学原理
姿态控制基于四元数表示，避免了欧拉角的奇点问题：

```cpp
Vector3f AttitudeControl::update(const Quatf &q) const {
    // 1. 获取期望姿态
    Quatf qd = _attitude_setpoint_q;
    
    // 2. 计算简化的期望姿态(忽略偏航，优先俯仰滚转)
    const Vector3f e_z = q.dcm_z();        // 当前机体Z轴
    const Vector3f e_z_d = qd.dcm_z();     // 期望机体Z轴
    Quatf qd_red(e_z, e_z_d);              // 简化姿态四元数
    
    // 3. 处理特殊情况(推力方向完全相反)
    if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
        qd_red = qd;  // 使用完整姿态控制
    } else {
        qd_red *= q;  // 转换到世界坐标系
    }
    
    // 4. 混合完整和简化期望姿态
    Quatf q_mix = qd_red.inversed() * qd;
    q_mix.canonicalize();
    
    // 限制四元数分量范围
    q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
    q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
    
    // 应用偏航权重
    qd = qd_red * Quatf(cosf(_yaw_w * acosf(q_mix(0))), 0, 0, sinf(_yaw_w * asinf(q_mix(3))));
    
    // 5. 计算姿态误差四元数
    const Quatf qe = q.inversed() * qd;
    
    // 6. 提取姿态误差轴角表示
    const Vector3f eq = 2.f * qe.canonical().imag();
    
    // 7. 计算角速度设定值
    Vector3f rate_setpoint = eq.emult(_proportional_gain);
    
    // 8. 添加偏航角速度前馈
    addYawFeedforward(rate_setpoint, q);
    
    // 9. 应用角速度限制
    for (int i = 0; i < 3; i++) {
        rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
    }
    
    return rate_setpoint;
}
```

#### 偏航前馈控制

```cpp
void AttitudeControl::addYawFeedforward(Vector3f &rates_sp, const Quatf &q) const {
    if (std::isfinite(_yawspeed_setpoint)) {
        // 将世界坐标系偏航角速度转换为机体坐标系
        // 世界Z轴在机体坐标系中的表示
        const Vector3f world_z_body = q.inversed().dcm_z();
        
        // 添加偏航角速度前馈
        rates_sp += world_z_body * _yawspeed_setpoint;
    }
}
```

### 2. 推力矢量到姿态的转换

#### 推力矢量解算

```cpp
void MulticopterAttitudeControl::generate_attitude_setpoint() {
    vehicle_attitude_setpoint_s att_sp{};
    
    if (_vehicle_attitude_setpoint_sub.copy(&att_sp)) {
        // 1. 提取推力设定值
        Vector3f thrust_body = Vector3f(att_sp.thrust_body);
        _thrust_setpoint = thrust_body.norm();
        
        // 2. 计算期望机体Z轴方向
        Vector3f body_z_sp;
        if (_thrust_setpoint > FLT_EPSILON) {
            body_z_sp = -thrust_body.normalized();  // 负Z方向
        } else {
            body_z_sp = Vector3f(0.f, 0.f, 1.f);   // 默认向上
        }
        
        // 3. 处理偏航角设定值
        float yaw_setpoint = att_sp.yaw_body;
        if (!PX4_ISFINITE(yaw_setpoint)) {
            yaw_setpoint = _yaw_setpoint;  // 保持当前偏航角
        }
        
        // 4. 计算期望机体X轴方向
        Vector3f body_x_sp = Vector3f(cosf(yaw_setpoint), sinf(yaw_setpoint), 0.f);
        
        // 5. 通过叉积计算机体Y轴
        Vector3f body_y_sp = body_z_sp % body_x_sp;
        body_y_sp.normalize();
        
        // 6. 重新计算正交的机体X轴
        body_x_sp = body_y_sp % body_z_sp;
        
        // 7. 构造旋转矩阵
        matrix::Dcmf R_sp;
        R_sp.col(0) = body_x_sp;
        R_sp.col(1) = body_y_sp; 
        R_sp.col(2) = body_z_sp;
        
        // 8. 转换为四元数
        _attitude_setpoint_q = Quatf(R_sp);
        
        // 9. 设置偏航角速度前馈
        _attitude_control.setAttitudeSetpoint(_attitude_setpoint_q, att_sp.yaw_sp_move_rate);
    }
}
```

### 3. 手动模式姿态控制

#### 手动输入处理

```cpp
void MulticopterAttitudeControl::control_attitude_manual() {
    manual_control_setpoint_s manual_control_setpoint{};
    
    if (_manual_control_setpoint_sub.copy(&manual_control_setpoint)) {
        // 1. 提取手动控制输入
        float roll_input = manual_control_setpoint.y;    // 滚转输入
        float pitch_input = -manual_control_setpoint.x;  // 俯仰输入
        float yaw_input = manual_control_setpoint.r;     // 偏航输入
        float throttle_input = manual_control_setpoint.z; // 油门输入
        
        // 2. 应用死区
        roll_input = applyDeadzone(roll_input, _param_mpc_hold_dz.get());
        pitch_input = applyDeadzone(pitch_input, _param_mpc_hold_dz.get());
        yaw_input = applyDeadzone(yaw_input, _param_mpc_hold_dz.get());
        
        // 3. 计算期望倾斜角
        Vector3f euler_sp;
        euler_sp(0) = roll_input * _man_tilt_max;   // 滚转角
        euler_sp(1) = pitch_input * _man_tilt_max;  // 俯仰角
        euler_sp(2) = _yaw_setpoint;                // 偏航角(保持当前值)
        
        // 4. 处理偏航输入
        if (fabsf(yaw_input) > FLT_EPSILON) {
            float yaw_rate_sp = yaw_input * math::radians(_param_mpc_man_y_max.get());
            
            // 偏航角速度积分
            float dt = math::constrain((hrt_absolute_time() - _timestamp_last_run) * 1e-6f, 
                                     0.0002f, 0.02f);
            euler_sp(2) += yaw_rate_sp * dt;
            euler_sp(2) = matrix::wrap_pi(euler_sp(2));
            _yaw_setpoint = euler_sp(2);
        }
        
        // 5. 转换为四元数
        _attitude_setpoint_q = Quatf(Eulerf(euler_sp));
        
        // 6. 处理油门输入
        float thrust_setpoint = processThrottleInput(throttle_input);
        
        // 7. 发布姿态设定值
        publishManualAttitudeSetpoint(thrust_setpoint);
    }
}
```

#### 油门处理

```cpp
float MulticopterAttitudeControl::processThrottleInput(float throttle_input) {
    // 1. 油门输入范围调整 [0, 1]
    throttle_input = math::constrain(throttle_input, 0.f, 1.f);
    
    // 2. 应用油门曲线
    float throttle_curve = _param_mpc_thr_curve.get();
    if (throttle_curve > FLT_EPSILON) {
        // 指数曲线: y = x^curve
        throttle_input = powf(throttle_input, throttle_curve);
    }
    
    // 3. 应用最小/最大油门限制
    float throttle_min = _manual_throttle_minimum.update(_param_mpc_manthr_min.get(), 0.01f);
    float throttle_max = _manual_throttle_maximum.update(1.0f, 0.01f);
    
    // 4. 映射到推力范围
    float thrust_setpoint = throttle_min + throttle_input * (throttle_max - throttle_min);
    
    return thrust_setpoint;
}
```

### 4. 自动模式姿态控制

```cpp
void MulticopterAttitudeControl::control_attitude_auto() {
    vehicle_attitude_setpoint_s att_sp{};
    
    if (_vehicle_attitude_setpoint_sub.update(&att_sp)) {
        // 1. 检查设定值有效性
        if (PX4_ISFINITE(att_sp.q_d[0]) && PX4_ISFINITE(att_sp.q_d[1]) && 
            PX4_ISFINITE(att_sp.q_d[2]) && PX4_ISFINITE(att_sp.q_d[3])) {
            
            // 2. 提取姿态设定四元数
            _attitude_setpoint_q = Quatf(att_sp.q_d);
            
            // 3. 提取推力设定值
            Vector3f thrust_body(att_sp.thrust_body);
            _thrust_setpoint = thrust_body.norm();
            
            // 4. 设置偏航角速度前馈
            float yawspeed_feedforward = NAN;
            if (PX4_ISFINITE(att_sp.yaw_sp_move_rate)) {
                yawspeed_feedforward = att_sp.yaw_sp_move_rate;
            }
            
            // 5. 更新姿态控制器设定值
            _attitude_control.setAttitudeSetpoint(_attitude_setpoint_q, yawspeed_feedforward);
        }
    }
}
```

## 控制模式适配

### 不同控制模式的处理

```cpp
void MulticopterAttitudeControl::Run() {
    // 1. 更新传感器数据
    updateSensorData();
    
    // 2. 更新控制模式
    vehicle_control_mode_s control_mode{};
    _vehicle_control_mode_sub.copy(&control_mode);
    
    // 3. 根据控制模式选择控制策略
    if (control_mode.flag_control_manual_enabled) {
        // 手动控制模式
        if (control_mode.flag_control_attitude_enabled) {
            control_attitude_manual();
        } else {
            // 纯手动模式(无姿态稳定)
            bypass_attitude_control();
        }
        
    } else if (control_mode.flag_control_auto_enabled) {
        // 自动控制模式
        control_attitude_auto();
        
    } else if (control_mode.flag_control_offboard_enabled) {
        // 离板控制模式
        control_attitude_offboard();
    }
    
    // 4. 执行姿态控制计算
    if (control_mode.flag_control_attitude_enabled) {
        _rates_setpoint = _attitude_control.update(_attitude_q);
        
        // 5. 发布角速度设定值
        publish_rates_setpoint();
    }
}
```

### 控制权限管理

```cpp
bool MulticopterAttitudeControl::checkControlAuthority(const vehicle_control_mode_s &control_mode) {
    // 检查各种控制权限
    bool attitude_control_enabled = control_mode.flag_control_attitude_enabled;
    bool rates_control_enabled = control_mode.flag_control_rates_enabled;
    bool manual_input_enabled = control_mode.flag_control_manual_enabled;
    
    // 根据不同模式组合确定控制策略
    if (attitude_control_enabled && rates_control_enabled) {
        // 完整姿态控制
        return true;
    } else if (rates_control_enabled) {
        // 仅角速度控制
        return false;  // 跳过姿态控制，直接使用角速度设定值
    } else {
        // 无控制权限
        return false;
    }
}
```

## 姿态估计与融合

### 姿态数据获取

```cpp
void MulticopterAttitudeControl::updateSensorData() {
    vehicle_attitude_s attitude{};
    
    if (_vehicle_attitude_sub.update(&attitude)) {
        // 1. 更新姿态四元数
        _attitude_q = Quatf(attitude.q);
        
        // 2. 更新角速度(机体坐标系)
        _angular_velocity = Vector3f(attitude.rollspeed, attitude.pitchspeed, attitude.yawspeed);
        
        // 3. 检查数据有效性
        if (!_attitude_q.isAllFinite()) {
            PX4_ERR("Attitude quaternion contains NaN or Inf");
            _attitude_q = Quatf(1.f, 0.f, 0.f, 0.f);  // 重置为单位四元数
        }
        
        // 4. 姿态四元数标准化
        _attitude_q.normalize();
    }
}
```

### 姿态预测与补偿

```cpp
Quatf MulticopterAttitudeControl::predictAttitude(float dt) const {
    // 使用当前角速度预测下一时刻姿态
    Vector3f angular_velocity_pred = _angular_velocity;
    
    // 角速度积分预测姿态变化
    float rotation_magnitude = angular_velocity_pred.norm() * dt;
    
    if (rotation_magnitude > FLT_EPSILON) {
        Vector3f rotation_axis = angular_velocity_pred.normalized();
        Quatf delta_q = Quatf(AxisAnglef(rotation_axis, rotation_magnitude));
        return _attitude_q * delta_q;
    }
    
    return _attitude_q;
}
```

## 控制约束与限制

### 角速度限制

```cpp
Vector3f MulticopterAttitudeControl::applyRateLimits(const Vector3f &rates_sp) const {
    Vector3f rates_sp_limited = rates_sp;
    
    // 应用各轴角速度限制
    rates_sp_limited(0) = math::constrain(rates_sp(0), 
                                        -math::radians(_param_mc_rollrate_max.get()),
                                         math::radians(_param_mc_rollrate_max.get()));
    
    rates_sp_limited(1) = math::constrain(rates_sp(1),
                                        -math::radians(_param_mc_pitchrate_max.get()),
                                         math::radians(_param_mc_pitchrate_max.get()));
    
    rates_sp_limited(2) = math::constrain(rates_sp(2),
                                        -math::radians(_param_mc_yawrate_max.get()),
                                         math::radians(_param_mc_yawrate_max.get()));
    
    return rates_sp_limited;
}
```

### 倾斜角限制

```cpp
void MulticopterAttitudeControl::applyTiltLimits() {
    // 计算当前倾斜角
    Vector3f body_z = _attitude_q.dcm_z();
    float tilt_current = acosf(body_z(2));
    
    // 检查是否超过限制
    float tilt_max = math::radians(_param_mpc_tilt_max.get());
    
    if (tilt_current > tilt_max) {
        // 限制倾斜角，重新计算姿态设定值
        Vector3f body_z_sp = _attitude_setpoint_q.dcm_z();
        Vector2f tilt_sp_xy(body_z_sp(0), body_z_sp(1));
        float tilt_sp_xy_norm = tilt_sp_xy.norm();
        
        if (tilt_sp_xy_norm > sinf(tilt_max)) {
            // 重新归一化XY分量
            tilt_sp_xy *= sinf(tilt_max) / tilt_sp_xy_norm;
            body_z_sp(0) = tilt_sp_xy(0);
            body_z_sp(1) = tilt_sp_xy(1);
            body_z_sp(2) = cosf(tilt_max);
            
            // 重新计算受限的姿态设定值
            reconstructAttitudeSetpoint(body_z_sp);
        }
    }
}
```

## 关键参数配置

### 姿态控制增益
```cpp
// 姿态比例增益
MC_ROLL_P       // 滚转姿态增益 [2.0-12.0]
MC_PITCH_P      // 俯仰姿态增益 [2.0-12.0] 
MC_YAW_P        // 偏航姿态增益 [1.0-5.0]
MC_YAW_WEIGHT   // 偏航权重 [0.0-1.0]

// 角速度限制
MC_ROLLRATE_MAX  // 最大滚转角速度 [10-1800 deg/s]
MC_PITCHRATE_MAX // 最大俯仰角速度 [10-1800 deg/s]
MC_YAWRATE_MAX   // 最大偏航角速度 [10-1800 deg/s]
```

### 手动模式参数
```cpp
// 手动模式倾斜角
MPC_MAN_TILT_MAX // 手动模式最大倾斜角 [5-90 deg]
MPC_TILT_MAX     // 自动模式最大倾斜角 [5-90 deg]

// 偏航控制
MPC_MAN_Y_MAX    // 手动偏航最大角速度 [10-400 deg/s]
MPC_MAN_Y_TAU    // 偏航响应时间常数 [0.0-1.0 s]

// 油门参数
MPC_THR_CURVE    // 油门曲线指数 [0.0-3.0]
MPC_MANTHR_MIN   // 最小手动油门 [0.0-0.3]
```

### 响应性参数
```cpp
// 系统响应性
SYS_VEHICLE_RESP // 整体车辆响应性 [0.0-1.0]

// 自适应参数调节
MPC_ACC_HOR      // 水平加速度 (自动调节)
MPC_ACC_HOR_MAX  // 最大水平加速度 (自动调节)
MPC_MAN_Y_MAX    // 手动偏航速度 (自动调节)
```

## 性能监控与调试

### 姿态控制性能指标

```cpp
struct AttitudeControlPerformance {
    float attitude_error_norm;      // 姿态误差范数
    float rates_setpoint_norm;      // 角速度设定值范数
    float tilt_angle;              // 当前倾斜角
    float yaw_error;               // 偏航角误差
    float control_latency_us;       // 控制延迟(微秒)
};

void MulticopterAttitudeControl::updatePerformanceMetrics() {
    // 计算姿态误差
    Quatf q_error = _attitude_q.inversed() * _attitude_setpoint_q;
    Vector3f euler_error = Eulerf(q_error);
    _perf_metrics.attitude_error_norm = euler_error.norm();
    
    // 计算倾斜角
    Vector3f body_z = _attitude_q.dcm_z();
    _perf_metrics.tilt_angle = acosf(math::constrain(body_z(2), -1.f, 1.f));
    
    // 计算偏航误差
    _perf_metrics.yaw_error = matrix::wrap_pi(euler_error(2));
    
    // 角速度设定值
    _perf_metrics.rates_setpoint_norm = _rates_setpoint.norm();
}
```

### 调试输出

```cpp
void MulticopterAttitudeControl::printDebugInfo() {
    PX4_INFO("Attitude Control Debug:");
    PX4_INFO("  Att Error: %.2f rad", _perf_metrics.attitude_error_norm);
    PX4_INFO("  Rates SP: %.2f rad/s", _perf_metrics.rates_setpoint_norm);
    PX4_INFO("  Tilt: %.1f deg", math::degrees(_perf_metrics.tilt_angle));
    PX4_INFO("  Yaw Error: %.1f deg", math::degrees(_perf_metrics.yaw_error));
    PX4_INFO("  Latency: %lu us", _perf_metrics.control_latency_us);
}
```

## 故障处理与安全

### 姿态估计故障

```cpp
void MulticopterAttitudeControl::handleAttitudeEstimationFailure() {
    vehicle_status_s vehicle_status{};
    _vehicle_status_sub.copy(&vehicle_status);
    
    // 检查姿态估计器状态
    if (vehicle_status.pre_flight_checks_pass == false) {
        // 姿态估计器故障，切换到安全模式
        PX4_ERR("Attitude estimation failure detected");
        
        // 1. 停止姿态控制
        _rates_setpoint.zero();
        
        // 2. 发布紧急状态
        vehicle_attitude_setpoint_s emergency_setpoint{};
        emergency_setpoint.timestamp = hrt_absolute_time();
        emergency_setpoint.thrust_body[2] = 0.0f;  // 停止推力
        _vehicle_attitude_setpoint_pub.publish(emergency_setpoint);
        
        // 3. 触发故障保护
        triggerFailsafe(FailsafeReason::ATTITUDE_ESTIMATION_FAILURE);
    }
}
```

### 控制饱和处理

```cpp
void MulticopterAttitudeControl::handleControlSaturation() {
    // 检测角速度设定值是否饱和
    Vector3f rate_limits = Vector3f(
        math::radians(_param_mc_rollrate_max.get()),
        math::radians(_param_mc_pitchrate_max.get()), 
        math::radians(_param_mc_yawrate_max.get())
    );
    
    for (int i = 0; i < 3; i++) {
        if (fabsf(_rates_setpoint(i)) >= rate_limits(i) * 0.95f) {
            // 角速度接近饱和，降低姿态增益
            Vector3f gain_reduction = _attitude_control.getProportionalGain();
            gain_reduction(i) *= 0.9f;
            _attitude_control.setProportionalGain(gain_reduction, _param_mc_yaw_weight.get());
            
            PX4_WARN("Rate saturation detected on axis %d, reducing gain", i);
        }
    }
}
```

## 总结

PX4姿态环控制模块是多旋翼控制系统的关键中环控制器，通过先进的四元数姿态控制算法实现了高精度、快响应的姿态控制。模块支持多种控制模式，具有完善的约束处理和故障保护机制，能够在各种飞行条件下提供稳定可靠的姿态控制性能。正确的参数调节和控制增益设置对于获得最佳控制效果至关重要。