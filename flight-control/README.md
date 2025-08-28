# 飞行控制系统文档

本目录包含PX4四旋翼飞行控制系统的核心架构文档，详细说明从高层任务管理到底层电机控制的完整控制链路。

## 文档列表

### 系统架构总览
- **[PX4四旋翼控制系统架构总览](PX4_Quadcopter_Control_Architecture.md)** - 整体系统框架和三环串级控制结构

### 三环串级控制详解

#### 外环：飞行模式与任务管理
- **[01_飞行模式管理详解](01_Flight_Mode_Management.md)** - Commander模块架构、飞行模式状态机、解锁逻辑与安全检查
- **[02_路径规划与导航详解](02_Path_Planning_Navigation.md)** - Navigator模块架构、任务执行与航点导航、返航逻辑

#### 中环：位置与姿态控制
- **[03_位置环控制详解](03_Position_Control_Loop.md)** - 位置→速度→加速度串级控制、推力矢量计算
- **[04_姿态环控制详解](04_Attitude_Control_Loop.md)** - 四元数姿态控制算法、推力矢量到姿态转换

#### 内环：角速度与电机控制
- **[05_角速度环控制详解](05_Rate_Control_Loop.md)** - PID控制算法实现、控制模式适配、性能监控
- **[06_电机混控与控制分配详解](06_Motor_Mixing_Control_Allocation.md)** - 控制分配算法、执行器效率建模、故障处理

## 阅读建议

### 初学者路径
1. 先阅读**系统架构总览**了解整体框架
2. 按照编号顺序学习三环控制（01→06）
3. 结合实际参数配置进行调试

### 开发者路径
1. 快速浏览架构总览
2. 根据需要修改的模块直接查阅对应文档
3. 重点关注模块间的uORB消息接口

### 调试路径
1. 从控制流程确定故障环节
2. 查阅对应模块的故障处理章节
3. 根据性能监控指标调整参数

## 控制流程图

```
用户输入/任务指令
        ↓
[01] 飞行模式管理 (Commander)
        ↓
[02] 路径规划导航 (Navigator)  
        ↓
[03] 位置环控制 (Position Control)
        ↓
[04] 姿态环控制 (Attitude Control)
        ↓
[05] 角速度环控制 (Rate Control)
        ↓
[06] 电机混控分配 (Control Allocator)
        ↓
    电机PWM输出
```

## 源码对应关系

| 文档模块 | 主要源码位置 | 核心文件 |
|---------|-------------|---------|  
| 飞行模式管理 | `src/modules/commander/` | `Commander.cpp` |
| 路径规划导航 | `src/modules/navigator/` | `navigator_main.cpp` |
| 位置环控制 | `src/modules/mc_pos_control/` | `MulticopterPositionControl.cpp` |
| 姿态环控制 | `src/modules/mc_att_control/` | `mc_att_control_main.cpp` |
| 角速度环控制 | `src/modules/mc_rate_control/` | `MulticopterRateControl.cpp` |
| 电机混控分配 | `src/modules/control_allocator/` | `ControlAllocator.cpp` |

## 相关文档

- [数据融合与导航系统](../navigation-sensor/) - 传感器数据处理和状态估计
- [参数系统](../parameters/) - 控制参数的配置和管理
- [仿真调试](../simulation/) - 仿真环境中的控制系统测试

---

本目录文档基于PX4 v1.14版本，涵盖了完整的飞行控制算法实现和工程实践。