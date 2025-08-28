# PX4角速度环控制模块详解

## 概述

PX4角速度环控制模块(MulticopterRateControl)是多旋翼控制系统的内环控制器，负责将姿态控制器输出的角速度设定值转换为力矩控制量。作为控制系统的最内环，该模块直接影响飞机的快速响应性能和稳定性，是整个控制链路的关键环节。

## 模块结构

### 文件位置
- **主要源码**: `src/modules/mc_rate_control/`
- **核心文件**: `MulticopterRateControl.cpp`, `MulticopterRateControl.hpp`
- **特技模式**: `mc_acro_params.c`
- **参数配置**: `mc_rate_control_params.c`

### 核心组件架构

```
Rate Control模块
├── MulticopterRateControl (主控制器)
├── RateControl (PID控制算法)
├── 特技模式处理 (Acro Mode)
├── 手动模式处理 (Manual Mode)
└── 参数配置文件
```

## 控制系统架构

### 角速度控制结构图

```
角速度设定值 → [PID控制器] → 力矩输出 → Control Allocator → 电机PWM信号
      ↑           ↓              ↓             ↓
   姿态控制器    误差计算        推力分配      电机驱动
      ↑           ↓              ↓
   当前角速度    前馈补偿       执行器输出
```

### 数据流程图

```
vehicle_rates_setpoint → Rate Controller → vehicle_torque_setpoint → Control Allocator
            ↑                ↓                      ↓
        姿态控制器输出      PID控制计算            力矩输出
            ↑                ↓                      ↓
    vehicle_angular_velocity  前馈/积分/微分       推力设定值
```

## 核心算法实现

### 主控制器类结构

```cpp
class MulticopterRateControl : public ModuleBase<MulticopterRateControl>, 
                              public ModuleParams,
                              public px4::WorkItem {
private:
    // uORB订阅
    uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
    uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
    uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    
    // uORB发布
    uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub;
    uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub;
    uORB::Publication<controller_status_s> _controller_status_pub;
    
    // 核心PID控制器
    PIDController _rate_control;
    
    // 状态变量
    Vector3f _angular_velocity;     // 当前角速度 [rad/s]
    Vector3f _rates_setpoint;       // 角速度设定值 [rad/s]
    Vector3f _torque_output;        // 力矩输出
    float _thrust_setpoint;         // 推力设定值
    
    // 特技模式相关
    Vector3f _acro_rate_max;        // 特技模式最大角速度
    Vector3f _acro_expo;            // 特技模式指数曲线
    
    // 性能监控
    perf_counter_t _loop_perf;
    hrt_abstime _timestamp_last_run{0};
    
public:
    MulticopterRateControl(bool vtol = false);
    
    // 主要接口
    bool init() override;
    void Run() override;
    
private:
    // 核心控制方法
    void control_rates_manual();
    void control_rates_acro();
    void control_rates_auto();
    void generate_torque_setpoint();
    void publish_torque_setpoint();
    void publish_thrust_setpoint();
    
    // 参数更新
    void parameters_updated();
    
    // 工具方法
    Vector3f applyExpoFunction(const Vector3f &input, const Vector3f &expo);
    void updateControllerStatus();
};
```

### PID控制器核心

```cpp
class RateController {
private:
    // PID增益
    Vector3f _gain_p;               // 比例增益
    Vector3f _gain_i;               // 积分增益  
    Vector3f _gain_d;               // 微分增益
    Vector3f _gain_ff;              // 前馈增益
    
    // 积分项管理
    Vector3f _integral;             // 积分累积值
    Vector3f _integral_limit;       // 积分限制
    
    // 微分项管理
    Vector3f _rate_prev;            // 上次角速度值
    Vector3f _derivative_lpf;       // 微分项低通滤波
    
    // 控制约束
    Vector3f _output_limit;         // 输出限制
    
public:
    // 主要控制接口
    Vector3f updatePID(const Vector3f &rate_sp, const Vector3f &rate, float dt);
    void setPidGains(const Vector3f &p, const Vector3f &i, const Vector3f &d);
    void setFeedForwardGain(const Vector3f &ff);
    void setIntegratorLimit(const Vector3f &limit);
    void resetIntegral();
    
private:
    // 内部计算方法
    Vector3f calculateProportional(const Vector3f &error);
    Vector3f calculateIntegral(const Vector3f &error, float dt);
    Vector3f calculateDerivative(const Vector3f &rate, float dt);
    Vector3f calculateFeedForward(const Vector3f &rate_sp);
    void applyIntegralSaturation(const Vector3f &output);
};
```

## 角速度控制算法详解

### 1. PID控制核心算法

#### 完整PID计算

```cpp
Vector3f RateController::updatePID(const Vector3f &rate_sp, const Vector3f &rate, float dt) {
    // 1. 计算角速度误差
    Vector3f rate_error = rate_sp - rate;
    
    // 2. 比例项计算
    Vector3f proportional = calculateProportional(rate_error);
    
    // 3. 积分项计算
    Vector3f integral = calculateIntegral(rate_error, dt);
    
    // 4. 微分项计算
    Vector3f derivative = calculateDerivative(rate, dt);
    
    // 5. 前馈项计算
    Vector3f feedforward = calculateFeedForward(rate_sp);
    
    // 6. PID输出组合
    Vector3f output = proportional + integral + derivative + feedforward;
    
    // 7. 输出限制
    for (int i = 0; i < 3; i++) {
        output(i) = math::constrain(output(i), -_output_limit(i), _output_limit(i));
    }
    
    // 8. 积分抗饱和处理
    applyIntegralSaturation(output);
    
    return output;
}
```

#### 比例项计算

```cpp
Vector3f RateController::calculateProportional(const Vector3f &error) {
    // 简单比例控制
    return error.emult(_gain_p);
}
```

#### 积分项计算与抗积分饱和

```cpp
Vector3f RateController::calculateIntegral(const Vector3f &error, float dt) {
    // 1. 积分累积
    _integral += error * dt;
    
    // 2. 积分限制(防止积分饱和)
    for (int i = 0; i < 3; i++) {
        _integral(i) = math::constrain(_integral(i), -_integral_limit(i), _integral_limit(i));
    }
    
    // 3. 积分项输出
    return _integral.emult(_gain_i);
}

void RateController::applyIntegralSaturation(const Vector3f &output) {
    // 检测输出饱和，调整积分项
    for (int i = 0; i < 3; i++) {
        if ((output(i) > _output_limit(i) && _integral(i) > 0.0f) ||
            (output(i) < -_output_limit(i) && _integral(i) < 0.0f)) {
            // 输出饱和时停止积分增长
            if (fabsf(_gain_i(i)) > FLT_EPSILON) {
                float integral_reduction = fabsf(output(i)) - _output_limit(i);
                _integral(i) -= copysignf(integral_reduction / _gain_i(i), _integral(i));
            }
        }
    }
}
```

#### 微分项计算与滤波

```cpp
Vector3f RateController::calculateDerivative(const Vector3f &rate, float dt) {
    // 1. 计算角速度变化率(负微分，基于测量值)
    Vector3f rate_derivative = -(rate - _rate_prev) / dt;
    
    // 2. 低通滤波抑制噪声
    float cutoff_freq = 30.0f;  // 30Hz截止频率
    float alpha = dt / (dt + 1.0f / (2.0f * M_PI * cutoff_freq));
    
    for (int i = 0; i < 3; i++) {
        _derivative_lpf(i) = (1.0f - alpha) * _derivative_lpf(i) + alpha * rate_derivative(i);
    }
    
    // 3. 更新上次角速度值
    _rate_prev = rate;
    
    // 4. 微分项输出
    return _derivative_lpf.emult(_gain_d);
}
```

#### 前馈控制

```cpp
Vector3f RateController::calculateFeedForward(const Vector3f &rate_sp) {
    // 前馈补偿提高响应速度
    return rate_sp.emult(_gain_ff);
}
```

### 2. 控制模式适配

#### 自动模式角速度控制

```cpp
void MulticopterRateControl::control_rates_auto() {
    vehicle_rates_setpoint_s rates_setpoint{};
    
    if (_vehicle_rates_setpoint_sub.update(&rates_setpoint)) {
        // 1. 提取角速度设定值
        _rates_setpoint = Vector3f(rates_setpoint.roll, rates_setpoint.pitch, rates_setpoint.yaw);
        
        // 2. 提取推力设定值
        _thrust_setpoint = rates_setpoint.thrust_body[2];
        
        // 3. 执行PID控制
        float dt = math::constrain((hrt_absolute_time() - _timestamp_last_run) * 1e-6f, 
                                 0.0002f, 0.02f);  // 限制在0.2ms-20ms
        
        _torque_output = _rate_control.updatePID(_rates_setpoint, _angular_velocity, dt);
        
        // 4. 发布控制输出
        publish_torque_setpoint();
        publish_thrust_setpoint();
    }
}
```

#### 手动模式角速度控制

```cpp
void MulticopterRateControl::control_rates_manual() {
    manual_control_setpoint_s manual_setpoint{};
    
    if (_manual_control_setpoint_sub.update(&manual_setpoint)) {
        // 1. 获取遥控器输入
        Vector3f manual_input(manual_setpoint.y,    // 滚转
                             -manual_setpoint.x,   // 俯仰  
                             manual_setpoint.r);   // 偏航
        
        // 2. 应用死区
        for (int i = 0; i < 3; i++) {
            manual_input(i) = applyDeadzone(manual_input(i), 0.05f);
        }
        
        // 3. 映射到角速度设定值
        Vector3f rate_limits(math::radians(_param_mc_rollrate_max.get()),
                            math::radians(_param_mc_pitchrate_max.get()),
                            math::radians(_param_mc_yawrate_max.get()));
        
        _rates_setpoint = manual_input.emult(rate_limits);
        
        // 4. 处理油门输入
        _thrust_setpoint = processThrottleInput(manual_setpoint.z);
        
        // 5. 执行PID控制
        float dt = getLoopDeltaTime();
        _torque_output = _rate_control.updatePID(_rates_setpoint, _angular_velocity, dt);
        
        // 6. 发布控制输出
        publish_torque_setpoint();
        publish_thrust_setpoint();
    }
}
```

#### 特技模式(ACRO)控制

```cpp
void MulticopterRateControl::control_rates_acro() {
    manual_control_setpoint_s manual_setpoint{};
    
    if (_manual_control_setpoint_sub.update(&manual_setpoint)) {
        // 1. 获取遥控器输入
        Vector3f manual_input(manual_setpoint.y,
                             -manual_setpoint.x,
                             manual_setpoint.r);
        
        // 2. 应用指数曲线增强操控感
        Vector3f expo_input = applyExpoFunction(manual_input, _acro_expo);
        
        // 3. 映射到特技模式角速度范围
        _rates_setpoint = expo_input.emult(_acro_rate_max);
        
        // 4. 特技模式特殊处理
        // 更高的响应性，减少滤波
        _rate_control.setFilterCutoff(50.0f);  // 提高截止频率
        
        // 5. 执行控制
        float dt = getLoopDeltaTime();
        _torque_output = _rate_control.updatePID(_rates_setpoint, _angular_velocity, dt);
        
        // 6. 处理油门
        _thrust_setpoint = processAcroThrottle(manual_setpoint.z);
        
        // 7. 发布输出
        publish_torque_setpoint();
        publish_thrust_setpoint();
    }
}
```

### 3. 指数曲线处理

```cpp
Vector3f MulticopterRateControl::applyExpoFunction(const Vector3f &input, const Vector3f &expo) {
    Vector3f output;
    
    for (int i = 0; i < 3; i++) {
        float x = input(i);
        float exp_factor = expo(i);
        
        if (fabsf(x) > FLT_EPSILON) {
            // 指数曲线: f(x) = x * (exp_factor * x^2 + (1 - exp_factor))
            float x_abs = fabsf(x);
            float expo_term = exp_factor * x_abs * x_abs + (1.0f - exp_factor);
            output(i) = copysignf(x_abs * expo_term, x);
        } else {
            output(i) = x;
        }
    }
    
    return output;
}
```

## 力矩输出与推力管理

### 1. 力矩设定值发布

```cpp
void MulticopterRateControl::publish_torque_setpoint() {
    vehicle_torque_setpoint_s torque_setpoint{};
    
    // 1. 设置时间戳
    torque_setpoint.timestamp = hrt_absolute_time();
    torque_setpoint.timestamp_sample = _timestamp_last_run;
    
    // 2. 填充力矩输出值
    torque_setpoint.xyz[0] = _torque_output(0);  // 滚转力矩
    torque_setpoint.xyz[1] = _torque_output(1);  // 俯仰力矩
    torque_setpoint.xyz[2] = _torque_output(2);  // 偏航力矩
    
    // 3. 发布力矩设定值
    _vehicle_torque_setpoint_pub.publish(torque_setpoint);
}
```

### 2. 推力设定值发布

```cpp
void MulticopterRateControl::publish_thrust_setpoint() {
    vehicle_thrust_setpoint_s thrust_setpoint{};
    
    // 1. 设置时间戳
    thrust_setpoint.timestamp = hrt_absolute_time();
    thrust_setpoint.timestamp_sample = _timestamp_last_run;
    
    // 2. 填充推力值(机体坐标系)
    thrust_setpoint.xyz[0] = 0.0f;              // X方向推力(通常为0)
    thrust_setpoint.xyz[1] = 0.0f;              // Y方向推力(通常为0)  
    thrust_setpoint.xyz[2] = _thrust_setpoint;   // Z方向推力(主推力)
    
    // 3. 发布推力设定值
    _vehicle_thrust_setpoint_pub.publish(thrust_setpoint);
}
```

## 参数自适应调节

### 1. 根据飞行条件调节参数

```cpp
void MulticopterRateControl::adaptiveParameterTuning() {
    vehicle_status_s vehicle_status{};
    _vehicle_status_sub.copy(&vehicle_status);
    
    // 1. 根据飞行阶段调节参数
    if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF) {
        // 起飞阶段：使用保守增益
        adjustGainsForTakeoff();
        
    } else if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
        // 降落阶段：增强稳定性
        adjustGainsForLanding();
        
    } else if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ACRO) {
        // 特技模式：最大性能增益
        adjustGainsForAcro();
    }
    
    // 2. 根据环境条件调节
    adjustGainsForEnvironment();
}

void MulticopterRateControl::adjustGainsForTakeoff() {
    // 起飞时减少D增益，避免地面效应干扰
    Vector3f d_gain = _rate_control.getDerivativeGain();
    d_gain *= 0.5f;
    _rate_control.setDerivativeGain(d_gain);
    
    // 增加积分增益，提高稳定性
    Vector3f i_gain = _rate_control.getIntegralGain();
    i_gain *= 1.2f;
    _rate_control.setIntegralGain(i_gain);
}
```

### 2. 电池电压补偿

```cpp
void MulticopterRateControl::compensateForBatteryVoltage() {
    battery_status_s battery_status{};
    
    if (_battery_status_sub.update(&battery_status)) {
        float voltage_ratio = battery_status.voltage_filtered_v / _param_bat_v_charged.get();
        voltage_ratio = math::constrain(voltage_ratio, 0.7f, 1.2f);
        
        // 根据电池电压调节增益
        Vector3f gain_compensation = Vector3f(1.0f / voltage_ratio);
        
        Vector3f p_gain = _rate_control.getProportionalGain();
        Vector3f i_gain = _rate_control.getIntegralGain();
        
        _rate_control.setProportionalGain(p_gain.emult(gain_compensation));
        _rate_control.setIntegralGain(i_gain.emult(gain_compensation));
    }
}
```

## 性能监控与诊断

### 1. 控制器状态监控

```cpp
void MulticopterRateControl::updateControllerStatus() {
    controller_status_s status{};
    
    // 1. 基本信息
    status.timestamp = hrt_absolute_time();
    
    // 2. 角速度跟踪性能
    Vector3f rate_error = _rates_setpoint - _angular_velocity;
    status.rate_error_norm = rate_error.norm();
    
    // 3. 控制器输出
    status.output_norm = _torque_output.norm();
    
    // 4. 积分项状态
    Vector3f integral = _rate_control.getIntegral();
    status.integral_norm = integral.norm();
    
    // 5. 输出饱和检测
    Vector3f output_limit = _rate_control.getOutputLimit();
    status.saturated = false;
    for (int i = 0; i < 3; i++) {
        if (fabsf(_torque_output(i)) >= output_limit(i) * 0.95f) {
            status.saturated = true;
            break;
        }
    }
    
    // 6. 发布状态
    _controller_status_pub.publish(status);
}
```

### 2. 性能指标计算

```cpp
struct RateControlPerformance {
    float tracking_error_rms;       // 跟踪误差RMS
    float settling_time_ms;         // 建立时间
    float overshoot_percent;        // 超调量百分比
    float bandwidth_hz;             // 控制带宽
    float phase_margin_deg;         // 相位裕度
    float gain_margin_db;           // 增益裕度
};

void MulticopterRateControl::calculatePerformanceMetrics() {
    // 1. 计算跟踪误差RMS
    Vector3f error = _rates_setpoint - _angular_velocity;
    _perf_metrics.tracking_error_rms = sqrtf(error.dot(error) / 3.0f);
    
    // 2. 检测系统响应特性
    analyzeStepResponse();
    
    // 3. 频域分析
    performFrequencyAnalysis();
}
```

### 3. 故障检测

```cpp
void MulticopterRateControl::detectControlFailures() {
    // 1. 检测传感器故障
    if (!PX4_ISFINITE(_angular_velocity.norm())) {
        PX4_ERR("Invalid angular velocity measurement");
        triggerFailsafe(FailsafeReason::GYRO_FAILURE);
        return;
    }
    
    // 2. 检测控制发散
    if (_torque_output.norm() > 5.0f) {  // 力矩过大
        PX4_WARN("Large control output detected: %.2f", (double)_torque_output.norm());
        
        // 重置积分项
        _rate_control.resetIntegral();
        
        // 降低增益
        adjustGainsForSafety();
    }
    
    // 3. 检测振荡
    detectOscillation();
}

void MulticopterRateControl::detectOscillation() {
    // 简单振荡检测：检查输出符号变化频率
    static Vector3f output_prev;
    static int sign_change_count[3] = {0, 0, 0};
    static hrt_abstime last_check_time = 0;
    
    hrt_abstime now = hrt_absolute_time();
    
    if (now - last_check_time > 1000000) {  // 每秒检查一次
        for (int i = 0; i < 3; i++) {
            if ((_torque_output(i) > 0 && output_prev(i) < 0) ||
                (_torque_output(i) < 0 && output_prev(i) > 0)) {
                sign_change_count[i]++;
            }
            
            // 如果1秒内符号变化超过20次，判断为振荡
            if (sign_change_count[i] > 20) {
                PX4_WARN("Oscillation detected on axis %d", i);
                
                // 降低该轴的D增益
                Vector3f d_gain = _rate_control.getDerivativeGain();
                d_gain(i) *= 0.8f;
                _rate_control.setDerivativeGain(d_gain);
            }
            
            sign_change_count[i] = 0;
        }
        
        last_check_time = now;
    }
    
    output_prev = _torque_output;
}
```

## 关键参数配置

### PID控制参数
```cpp
// 滚转轴PID参数
MC_ROLLRATE_P       // 滚转角速度比例增益 [0.05-0.5]
MC_ROLLRATE_I       // 滚转角速度积分增益 [0.01-0.8]
MC_ROLLRATE_D       // 滚转角速度微分增益 [0.0-0.05]
MC_ROLLRATE_FF      // 滚转角速度前馈增益 [0.0-0.5]
MC_RR_INT_LIM       // 滚转积分限制 [0.1-1.0]

// 俯仰轴PID参数
MC_PITCHRATE_P      // 俯仰角速度比例增益 [0.05-0.5]
MC_PITCHRATE_I      // 俯仰角速度积分增益 [0.01-0.8]
MC_PITCHRATE_D      // 俯仰角速度微分增益 [0.0-0.05]
MC_PITCHRATE_FF     // 俯仰角速度前馈增益 [0.0-0.5]
MC_PR_INT_LIM       // 俯仰积分限制 [0.1-1.0]

// 偏航轴PID参数
MC_YAWRATE_P        // 偏航角速度比例增益 [0.05-0.6]
MC_YAWRATE_I        // 偏航角速度积分增益 [0.01-0.6]
MC_YAWRATE_D        // 偏航角速度微分增益 [0.0-0.02]
MC_YAWRATE_FF       // 偏航角速度前馈增益 [0.0-0.5]
MC_YR_INT_LIM       // 偏航积分限制 [0.1-1.0]
```

### 控制器整体增益
```cpp
// 控制器增益系数
MC_ROLLRATE_K       // 滚转轴控制器增益 [0.5-3.0]
MC_PITCHRATE_K      // 俯仰轴控制器增益 [0.5-3.0]  
MC_YAWRATE_K        // 偏航轴控制器增益 [0.5-3.0]
```

### 特技模式参数
```cpp
// 特技模式角速度限制
MC_ACRO_R_MAX       // 特技模式最大滚转角速度 [10-1800 deg/s]
MC_ACRO_P_MAX       // 特技模式最大俯仰角速度 [10-1800 deg/s]
MC_ACRO_Y_MAX       // 特技模式最大偏航角速度 [10-1800 deg/s]

// 特技模式指数曲线
MC_ACRO_EXPO        // 滚转俯仰指数 [0.0-1.0]
MC_ACRO_EXPO_Y      // 偏航指数 [0.0-1.0]
MC_ACRO_SUPEXPO     // 超级指数模式 [0.0-1.0]
```

### 滤波参数
```cpp
// 微分项滤波
MC_DTERM_CUTOFF     // D项低通滤波截止频率 [10-200 Hz]

// 陀螺仪滤波
IMU_GYRO_CUTOFF     // 陀螺仪低通滤波截止频率 [30-300 Hz]
```

## 调参指南

### 1. 基础调参流程

```cpp
// 调参步骤
1. 首先调节P增益，使系统能够跟踪设定值
2. 然后调节D增益，减少超调和振荡
3. 最后调节I增益，消除稳态误差
4. 调节前馈增益，提高响应速度

// P增益调节
// 从小值开始，逐渐增大直到系统出现振荡，然后退回到振荡点的70%

// D增益调节  
// 增加D增益可以减少超调，但过大会导致高频噪声放大

// I增益调节
// I增益可以消除稳态误差，但过大会导致积分饱和和系统不稳定
```

### 2. 参数优化建议

```cpp
// 滚转俯仰轴通常使用相同参数
MC_ROLLRATE_P = MC_PITCHRATE_P

// 偏航轴增益通常较小
MC_YAWRATE_P = MC_ROLLRATE_P * 0.8

// 积分增益通常为比例增益的20%-50%
MC_ROLLRATE_I = MC_ROLLRATE_P * 0.3

// 微分增益通常很小
MC_ROLLRATE_D = MC_ROLLRATE_P * 0.01
```

## 总结

PX4角速度环控制模块是多旋翼控制系统的关键内环控制器，通过精确的PID控制算法实现了高精度的角速度跟踪。模块支持多种飞行模式，具有完善的参数自适应、性能监控和故障检测功能。正确的PID参数调节对于获得最佳控制性能至关重要，需要根据具体飞机特性和飞行要求进行精心调节。