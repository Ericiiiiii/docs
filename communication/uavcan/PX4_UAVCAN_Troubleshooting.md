# PX4 UAVCAN 故障排除指南

## 问题诊断流程

### 1. 基础检查

```bash
# 检查UAVCAN是否启用
param show UAVCAN_ENABLE

# 检查UAVCAN状态
uavcan status

# 查看系统日志
dmesg | grep -i uavcan
```

### 2. 硬件检查

```bash
# PX4内置检查
uavcan status -v

# 检查参数设置
param show UAVCAN*

# 查看日志信息
dmesg | grep -i can
```

## 常见问题及解决方案

### 问题1: ESC不响应

**症状**: 
- ESC不转动
- `uavcan info` 显示无在线节点
- 无ESC状态反馈

**诊断步骤**:

```bash
# 1. 检查UAVCAN配置
param show UAVCAN*

# 2. 检查CAN接口
uavcan status -v

# 3. 检查在线节点
uavcan info
```

**可能原因及解决方案**:

#### 原因1: UAVCAN未正确启用
```bash
# 解决方案
param set UAVCAN_ENABLE 3
param save
reboot
```

#### 原因2: CAN总线连接问题
- 检查CAN_H和CAN_L连线
- 确认120Ω终端电阻
- 检查接触是否良好

#### 原因3: 波特率不匹配
```bash
# 尝试不同波特率
param set UAVCAN_BITRATE 500000  # 或250000
param save
reboot
```

#### 原因4: 节点ID冲突
```bash
# 更改节点ID
param set UAVCAN_NODE_ID 2
param save
reboot
```

### 问题2: CAN总线错误

**症状**:
- 大量CAN错误
- 通信不稳定
- 间歇性连接

**诊断步骤**:

```bash
# 检查UAVCAN统计信息
uavcan status

# 查看节点信息
uavcan info

# 检查系统负载
top
```

**解决方案**:

#### 信号完整性问题
1. 使用屏蔽双绞线
2. 减少线缆长度
3. 远离高功率设备
4. 确保良好接地

#### 终端电阻问题
```bash
# 检查终端电阻
# 应该在总线两端各有一个120Ω电阻
# 总阻抗应该约为60Ω
```

#### 电源问题
1. 确保电源稳定
2. 检查电压波动
3. 使用足够容量的电源

### 问题3: 参数设置不生效

**症状**:
- 修改参数后行为未改变
- ESC功能映射错误

**解决方案**:

```bash
# 1. 确保参数已保存
param save

# 2. 重启系统
reboot

# 3. 验证参数值
param show UAVCAN_EC_FUNC1

# 4. 如果参数丢失，重新设置
param set UAVCAN_EC_FUNC1 101
param save
```

### 问题4: ESC状态异常

**症状**:
- ESC温度过高
- 电流异常
- 转速不稳定

**诊断步骤**:

```bash
# 监听ESC状态
listener esc_status

# 检查执行器输出
listener actuator_outputs
```

**解决方案**:

#### 温度过高
1. 检查散热
2. 降低负载
3. 检查ESC参数设置

#### 电流异常
1. 检查电机负载
2. 验证螺旋桨规格
3. 检查电源容量

#### 转速不稳定
1. 检查PID参数
2. 验证传感器校准
3. 检查机械振动

## 高级诊断

### 使用示波器检查信号

```
CAN_H: 2.5V ± 1V (差分信号)
CAN_L: 2.5V ∓ 1V (差分信号)
差分电压: 2V (显性) / 0V (隐性)
```

### 网络分析

```bash
# PX4 UAVCAN调试
# 查看详细状态
uavcan status -v

# 监控节点活动
uavcan info

# 测试模式
uavcan test

# 监听消息
listener esc_status
```

### 日志分析

```bash
# 查看详细日志  
dmesg

# 查看启动日志
cat /fs/microsd/bootlog.txt | grep -i uavcan

# 实时监控(在QGC MAVLink控制台中)
# 或使用PX4控制台的dmesg命令
```

## 性能优化

### 减少延迟

```bash
# 调整调度优先级
param set UAVCAN_SCHED_PRIO 250

# 增加更新频率
param set UAVCAN_ESC_RATE 400
```

### 内存优化

```bash
# 调整内存池大小
param set UAVCAN_POOL_SIZE 8192

# 监控内存使用
free -h
```

### CPU优化

```bash
# 检查系统状态
top

# 查看任务状态
ps
```

## 紧急恢复

### 完全重置UAVCAN

```bash
# 禁用UAVCAN
param set UAVCAN_ENABLE 0
param save
reboot

# 重新配置
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 1
param set UAVCAN_BITRATE 1000000
param save
reboot
```

### 恢复默认参数

```bash
# 重置所有UAVCAN参数
param reset UAVCAN*
param save
reboot
```

### 紧急停止

```bash
# 立即停止所有电机
actuator_test -m uavcan -v 0

# 或者禁用执行器
param set UAVCAN_ENABLE 2  # 仅传感器模式
```

## 预防措施

### 定期检查

1. 每次飞行前检查CAN错误计数
2. 监控ESC温度和电流
3. 验证所有节点在线
4. 测试失效保护功能

### 备份配置

```bash
# 导出参数配置
param export /fs/microsd/uavcan_backup.params

# 恢复配置
param import /fs/microsd/uavcan_backup.params
```

### 文档记录

1. 记录硬件配置
2. 保存参数设置
3. 记录问题和解决方案
4. 维护变更日志

## 联系支持

如果问题仍然存在，请收集以下信息并联系技术支持:

1. **硬件信息**:
   - 飞控板型号和版本
   - ESC型号和固件版本
   - CAN总线拓扑图

2. **软件信息**:
   - PX4版本
   - 完整参数列表
   - 启动日志

3. **问题描述**:
   - 详细症状描述
   - 复现步骤
   - 错误日志

4. **测试结果**:
   - 诊断命令输出
   - 示波器截图 (如有)
   - 网络分析结果

---

**提示**: 大多数UAVCAN问题都与硬件连接相关。在深入软件调试之前，请务必仔细检查所有物理连接。
