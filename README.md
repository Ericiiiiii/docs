# FihawkFlyCtrl 文档索引

基于PX4的Fihawk飞控系统技术文档集合，涵盖飞行控制、硬件驱动、通信协议、开发环境等完整技术栈。

## 🚁 核心飞行系统

### 飞行控制系统
- **[PX4四旋翼控制架构](flight-control/PX4_Quadcopter_Control_Architecture.md)** - 系统架构总览
- **[飞行模式管理](flight-control/01_Flight_Mode_Management.md)** - 状态机与安全系统
- **[路径规划导航](flight-control/02_Path_Planning_Navigation.md)** - 任务执行与航点导航
- **[位置环控制](flight-control/03_Position_Control_Loop.md)** - 位置控制算法
- **[姿态环控制](flight-control/04_Attitude_Control_Loop.md)** - 姿态控制算法
- **[角速度环控制](flight-control/05_Rate_Control_Loop.md)** - 角速度控制算法
- **[电机混控分配](flight-control/06_Motor_Mixing_Control_Allocation.md)** - 控制分配算法

### 数据融合与导航
- **[数据融合系统](navigation-sensor/PX4四旋翼数据融合系统详解.md)** - EKF2算法详解
- **[惯性导航系统](navigation-sensor/PX4四旋翼惯性导航系统详解.md)** - IMU导航算法
- **[传感器标定系统](navigation-sensor/PX4四旋翼传感器标定系统详解.md)** - 标定算法与流程
- **[EKF后处理系统](navigation-sensor/系统后处理和EKF后处理实现机制详解.md)** - 后处理技术分析

### GPS定位系统
- **[GPS配置流程](gps-navigation/PX4_GPS_CONFIG_Parameter_Flow.md)** - GPS参数配置详解
- **[GPS运行机制](gps-navigation/PX4_GPS_runningstateflow.md)** - GPS模块运行流程
- **[RTK配置指南](gps-navigation/RTK982_Configuration_Guide.md)** - RTK GPS配置
- **[UM982验证指南](gps-navigation/UM982_GPS_Verification_Guide.md)** - UM982 GPS测试

## 🔧 底层系统

### 硬件驱动
- **[串口驱动框架](drivers/PX4_Serial_Framework_Complete.md)** - 串口系统完整文档
- **[SPI驱动架构](drivers/PX4_SPI_Driver_Architecture.md)** - SPI驱动系统
- **[I2C驱动框架](drivers/PX4_I2C_Driver_Framework.md)** - I2C驱动系统
- **[ADC驱动框架](drivers/PX4_ADC_Driver_Framework.md)** - ADC驱动系统

### 通信协议
- **[MAVLink协议](communication/PX4_MAVLink_Custom_Messages_Guide.md)** - MAVLink自定义消息
- **[UAVCAN协议](communication/uavcan/)** - UAVCAN协议文档集合
- **[ROS2通信](communication/PX4_ROS2快速入门指南.md)** - ROS2集成指南
- **[任务计算机通信](communication/PX4任务计算机通信指南.md)** - 完整通信方案

### 参数系统
- **[参数系统学习指南](parameters/PX4_Parameter_Study_README.md)** - 参数系统入门
- **[参数系统文档](parameters/PX4_Parameter_System_Documentation.md)** - 详细技术文档
- **[YAML参数系统](parameters/PX4_YAML_Parameter_System_Guide.md)** - YAML参数配置

## 🛠️ 开发环境

### 快速开始
- **[快速入门指南](development/Quick_Start_Guide.md)** - 新手开发环境搭建
- **[开发环境配置](development/)** - 完整开发环境文档

### 构建系统
- **[构建系统概览](build-system/)** - CMake构建配置
- **[CMake配置指南](build-system/CMake_Configuration_Guide.md)** - 详细构建配置

### 测试与调试
- **[测试框架](testing/)** - 单元测试与集成测试
- **[仿真环境](simulation/PX4_SITL_QGC_Connection_Guide.md)** - SITL仿真配置
- **[故障排除](troubleshooting/)** - 常见问题解决方案

## 🔩 硬件平台

### Fihawk FC-V1
- **[硬件配置指南](hardware-specific/Fihawk_Hardware_Setup_Guide.md)** - 硬件连接与配置
- **[串口配置](hardware-specific/Fihawk_Serial_Configuration.md)** - 串口接口配置
- **[传感器分析](hardware-specific/Fihawk_Sensor_Issues_Analysis.md)** - 传感器问题诊断
- **[控制频率分析](hardware-specific/fihawk-fc-v1-sensor-control-frequencies.md)** - 系统频率分析
- **[Bootloader刷写](hardware-specific/PX4_Bootloader_Flashing_Guide.md)** - 固件烧录指南

## 📖 使用指南

### 新手入门
1. [快速入门指南](development/Quick_Start_Guide.md) - 环境搭建
2. [PX4控制架构](flight-control/PX4_Quadcopter_Control_Architecture.md) - 系统概览
3. [Fihawk硬件配置](hardware-specific/Fihawk_Hardware_Setup_Guide.md) - 硬件连接

### 开发进阶
1. [数据融合系统](navigation-sensor/PX4四旋翼数据融合系统详解.md) - 核心算法
2. [驱动开发](drivers/) - 底层驱动
3. [通信协议](communication/) - 通信接口
4. [参数系统](parameters/) - 配置管理

### 问题解决
1. [故障排除](troubleshooting/) - 常见问题
2. [硬件诊断](hardware-specific/) - 硬件问题
3. [测试调试](testing/) - 调试方法

## 🔗 相关资源

- [PX4官方文档](https://dev.px4.io/) - PX4开发指南
- [PX4用户手册](https://docs.px4.io/) - 用户使用手册
- [MAVLink协议](https://mavlink.io/) - MAVLink官方文档

---

> 💡 **提示**: 建议收藏本页面作为技术文档导航入口，所有文档都会在这里保持最新索引。

> ⚠️ **注意**: 本文档基于PX4 v1.14版本，不同版本可能存在差异。