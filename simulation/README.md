# 仿真与调试工具文档

本目录包含PX4仿真环境配置、数据回放分析和调试工具的详细使用文档。

## 文档列表

### SITL仿真系统
- **[PX4 SITL QGC连接指南](PX4_SITL_QGC_Connection_Guide.md)** - SITL仿真与QGroundControl的连接配置和故障排除

### 数据回放系统
- **[PX4 Replay演示指南](PX4_Replay_Demo_Guide.md)** - PX4数据回放功能的使用演示和实际案例
- **[PX4 Replay原理解析](PX4_Replay_Principles.md)** - PX4数据回放功能的技术原理和架构设计

### 日志调试工具
- **[PX4日志等级配置](PX4_Log_Level_Configuration.md)** - PX4日志系统的配置和等级管理

## 仿真系统架构

### SITL架构图
```
物理模型仿真
    ↓
[Gazebo/jMAVSim] ←→ [PX4 SITL]
    ↓                    ↓
传感器仿真            MAVLink通信
    ↓                    ↓
环境交互              QGroundControl
```

### 仿真组件

#### 1. 仿真器选择
- **Gazebo**：高保真度3D仿真环境
  - 物理引擎：ODE/Bullet/Simbody
  - 传感器模拟：相机、激光雷达、IMU等
  - 环境建模：复杂3D场景支持

- **jMAVSim**：轻量级Java仿真器
  - 快速启动：适合算法验证
  - 基础物理：简单动力学模型
  - 多机仿真：支持集群仿真

- **FlightGear**：专业飞行仿真器
  - 真实飞行动力学
  - 详细地形数据
  - 天气系统仿真

#### 2. 仿真模型
```cpp
// 仿真器接口
class Simulator
{
    // 传感器数据注入
    void update_sensors();
    
    // 执行器输出处理  
    void handle_actuator_outputs();
    
    // 物理模型更新
    void update_dynamics();
};
```

## SITL仿真配置

### 1. 环境搭建
```bash
# 安装依赖
sudo apt install gazebo11 gazebo11-dev

# 编译SITL目标
make px4_sitl_default gazebo

# 启动仿真
make px4_sitl gazebo_iris
```

### 2. 仿真参数配置
```bash
# 仿真物理参数
param set SIM_GZ_EN 1         # 启用Gazebo
param set SIM_GZ_EC_FUNC1 101 # 电机映射
param set SIM_GZ_EC_MIN1 0    # 最小PWM
param set SIM_GZ_EC_MAX1 1000 # 最大PWM

# 传感器噪声设置
param set SIM_GPS_NOISE 1     # GPS噪声
param set SIM_BARO_NOISE 0.1  # 气压计噪声
param set SIM_GYRO_NOISE 0.01 # 陀螺仪噪声
```

### 3. 多机仿真
```bash
# 启动多个实例
DONT_RUN=1 make px4_sitl gazebo_iris
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
gazebo Tools/sitl_gazebo/worlds/iris_multi.world

# 启动多个PX4实例
./build/px4_sitl_default/bin/px4 -i 1 -d
./build/px4_sitl_default/bin/px4 -i 2 -d
```

## 数据回放系统

### Replay架构
```
飞行日志 (.ulg)
        ↓
[Replay模块]
        ↓
uORB消息重放
        ↓
算法模块处理
        ↓
结果对比分析
```

### 回放流程

#### 1. 日志录制
```bash
# 启用日志记录
param set SDLOG_MODE 1        # 从启动开始记录
param set SDLOG_PROFILE 1     # 高频率记录

# 自定义日志内容
param set SDLOG_DIRS_MAX 7    # 最大目录数
param set SDLOG_GPS_MSG 1     # 记录GPS消息
```

#### 2. 回放执行
```bash
# 基本回放
make px4_sitl_default replay < logfile.ulg

# 指定模块回放
make px4_sitl_default replay_ekf2 < logfile.ulg

# 参数化回放
replay trystart ekf2 -r 100 < logfile.ulg
```

#### 3. 结果分析
```bash
# 生成对比报告
python Tools/replay/compare_logs.py original.ulg replay.ulg

# 可视化分析
python Tools/replay/plot_replay.py --log replay.ulg
```

## 调试工具集

### 1. 实时监控
```bash
# 系统状态监控
top                           # CPU和内存使用
listener vehicle_status       # 飞行器状态
listener commander_state      # 指挥器状态

# 传感器数据监控
listener sensor_accel         # 加速度计
listener sensor_gyro          # 陀螺仪  
listener vehicle_gps_position # GPS位置
```

### 2. 日志分析
```bash
# 日志查看
dmesg                         # 系统日志
param show SDLOG_*            # 日志配置参数

# 日志等级设置
param set SYS_LOGGER 1        # 启用系统日志记录器
param set SENS_LOG_LEVEL 2    # 传感器日志等级
```

### 3. 性能分析
```bash
# 任务性能监控
work_queue status             # 工作队列状态
perf                          # 性能计数器

# 内存使用分析
free                          # 内存使用情况
ps                            # 进程状态
```

## 仿真环境定制

### 1. 自定义机型
```xml
<!-- iris_custom.sdf -->
<model name='iris_custom'>
    <include>
        <uri>model://iris</uri>
    </include>
    
    <!-- 自定义传感器 -->
    <plugin name='camera' filename='libgazebo_camera_plugin.so'>
        <camera_name>front_camera</camera_name>
    </plugin>
</model>
```

### 2. 环境场景配置
```xml
<!-- custom_world.world -->
<world name='custom_world'>
    <include>
        <uri>model://ground_plane</uri>
    </include>
    
    <!-- 添加障碍物 -->
    <include>
        <uri>model://oak_tree</uri>
        <pose>10 10 0 0 0 0</pose>
    </include>
</world>
```

### 3. 传感器模拟配置
```xml
<!-- IMU传感器配置 -->
<plugin name='imu' filename='libgazebo_imu_plugin.so'>
    <noise>
        <accel>
            <mean>0.0</mean>
            <stddev>0.1</stddev>
        </accel>
        <gyro>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
        </gyro>
    </noise>
</plugin>
```

## 测试验证流程

### 1. 算法验证
```bash
# 控制器测试
commander takeoff             # 起飞测试
commander land               # 着陆测试
commander mode offboard     # 外部控制模式

# 导航系统测试
navigator takeoff            # 导航起飞
navigator rtl                # 返航测试
```

### 2. 集成测试
```bash
# 任务仿真
commander mode auto:mission   # 自动任务模式
mission load mission.plan     # 加载任务文件

# 故障注入测试
failure inject gps            # GPS故障注入
failure inject motor 1        # 电机故障注入
```

### 3. 性能测试  
```bash
# 延迟测试
listener -s vehicle_attitude  # 姿态更新频率
listener -s actuator_outputs  # 输出更新频率

# 精度测试
param set SIM_GPS_NOISE 0     # 关闭GPS噪声
navigator precision_land      # 精确着陆测试
```

## 仿真最佳实践

### 1. 仿真精度设置
- **物理时间步**：1ms-5ms典型值
- **传感器更新频率**：与实际硬件一致
- **噪声模型**：基于实际传感器规格

### 2. 性能优化
- **多核利用**：启用多线程物理引擎
- **LOD优化**：根据需要调整模型细节
- **网络延迟**：模拟真实通信延迟

### 3. 测试覆盖
- **正常飞行场景**：起飞、巡航、着陆
- **边界条件**：风扰、传感器故障
- **极限情况**：低电量、通信中断

## 相关文档

- [飞行控制系统](../flight-control/) - 控制算法的仿真验证
- [通信协议](../communication/) - MAVLink仿真接口
- [参数系统](../parameters/) - 仿真参数配置

---

本目录文档提供完整的仿真环境搭建和调试工具使用指南，适合算法开发人员和测试工程师参考。