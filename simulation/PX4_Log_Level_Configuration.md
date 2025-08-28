# PX4日志等级配置指南

## 概述
PX4系统中的日志输出可能会产生大量重复的警告信息，特别是在飞行前检查(Preflight Check)阶段。本文档介绍如何调整日志等级以减少不必要的输出。

## 问题分析
你遇到的重复警告信息来自于：
1. **health_and_arming_checks模块**: 飞行前安全检查
2. **加速度计不一致**: `Accel 0 inconsistent - check cal`
3. **磁场干扰**: `Strong magnetic interference`

这些警告来自以下源码位置：
- `src/modules/commander/HealthAndArmingChecks/checks/imuConsistencyCheck.cpp`
- `src/modules/commander/HealthAndArmingChecks/checks/estimatorCheck.cpp`

## 解决方案

### 方案1: 参数配置 (推荐)

#### 1.1 调整IMU一致性检查参数
```bash
# 放宽加速度计一致性检查阈值 (默认0.7 m/s²)
param set COM_ARM_IMU_ACC 1.5

# 放宽陀螺仪一致性检查阈值 (默认0.25 rad/s)  
param set COM_ARM_IMU_GYR 0.5

# 保存参数
param save
```

#### 1.2 调整磁场干扰检查参数
```bash
# 设置磁场强度检查为仅警告模式
# 0=禁用, 1=拒绝解锁, 2=仅警告
param set COM_ARM_MAG_STR 0

# 或者放宽磁场角度一致性检查 (默认60度)
param set COM_ARM_MAG_ANG 90

# 保存参数
param save
```

#### 1.3 临时禁用特定检查 (调试用)
```bash
# 禁用IMU检查 (仅用于调试，不推荐飞行)
param set SYS_HAS_NUM_GYRO 0
param set SYS_HAS_NUM_ACCEL 0

# 禁用磁力计检查
param set SYS_HAS_MAG 0

# 保存参数
param save
```

### 方案2: 运行时日志等级控制

#### 2.1 使用dmesg控制内核日志
```bash
# 设置内核日志等级 (只显示ERROR及以上)
dmesg -n 3

# 或者完全禁用内核消息到控制台
echo 0 > /proc/sys/kernel/printk
```

#### 2.2 重定向串口输出
```bash
# 将标准输出重定向到null (临时)
exec 1>/dev/null

# 恢复标准输出
exec 1>/dev/console
```

### 方案3: 编译时配置 (高级)

#### 3.1 修改日志宏定义
在编译时可以通过修改以下文件来改变日志行为：

**文件**: `platforms/common/include/px4_platform_common/log.h`

```cpp
// 在RELEASE_BUILD模式下，WARN消息被编译时移除
#elif defined(RELEASE_BUILD)
#define PX4_WARN(FMT, ...) 	__px4_log_omit(_PX4_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
```

#### 3.2 编译发布版本
```bash
# 编译RELEASE版本 (自动减少日志输出)
make cuav_x7pro_default RELEASE=1
```

### 方案4: 运行时过滤 (实用)

#### 4.1 使用管道过滤
```bash
# 过滤掉特定警告信息
your_command 2>&1 | grep -v "Preflight Fail"

# 或者只显示ERROR级别以上的消息
your_command 2>&1 | grep -E "(ERROR|PANIC)"
```

#### 4.2 创建日志过滤脚本
```bash
#!/bin/bash
# 文件: filter_logs.sh

# 过滤掉重复的预检查警告
exec 2> >(grep -v -E "(Accel.*inconsistent|Strong magnetic interference)" >&2)

# 运行你的命令
"$@"
```

使用方法：
```bash
chmod +x filter_logs.sh
./filter_logs.sh your_px4_command
```

## 推荐配置

### 开发环境配置
```bash
# 适合开发调试的配置
param set COM_ARM_IMU_ACC 1.0    # 稍微放宽
param set COM_ARM_IMU_GYR 0.35   # 稍微放宽  
param set COM_ARM_MAG_STR 2      # 仅警告模式
param set COM_ARM_MAG_ANG 75     # 稍微放宽
param save
```

### 生产环境配置
```bash
# 适合实际飞行的配置 (保持安全)
param set COM_ARM_IMU_ACC 0.8    # 略微放宽但保持安全
param set COM_ARM_IMU_GYR 0.3    # 略微放宽但保持安全
param set COM_ARM_MAG_STR 1      # 保持拒绝解锁
param set COM_ARM_MAG_ANG 65     # 略微放宽但保持安全
param save
```

## 验证配置

### 检查当前参数
```bash
# 查看IMU相关参数
param show COM_ARM_IMU*

# 查看磁力计相关参数  
param show COM_ARM_MAG*

# 查看系统传感器配置
param show SYS_HAS*
```

### 测试效果
```bash
# 重启commander模块以应用新参数
commander stop
commander start

# 观察日志输出变化
dmesg -w
```

## 注意事项

1. **安全第一**: 不要完全禁用安全检查，只是调整阈值
2. **校准重要**: 如果频繁出现传感器不一致警告，应该重新校准传感器
3. **环境因素**: 磁场干扰可能来自周围环境，尝试更换测试位置
4. **硬件检查**: 持续的传感器问题可能表明硬件故障

## 故障排除

### 如果参数修改无效
```bash
# 强制重新加载参数
param reset_all
reboot

# 或者清除参数缓存
param reset
param save
reboot
```

### 如果问题持续存在
1. 检查传感器硬件连接
2. 重新校准所有传感器
3. 检查飞控板周围是否有磁场干扰源
4. 考虑更换传感器模块

通过以上配置，可以有效减少重复的警告信息，同时保持系统的安全性。
