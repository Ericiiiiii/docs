# PX4 UAVCAN 快速参考

## 快速配置 (5分钟设置)

### 1. 基本启用

```bash
# 启用UAVCAN ESC模式
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 1
param set UAVCAN_BITRATE 1000000
```

### 2. 四旋翼ESC配置

```bash
# ESC功能映射
param set UAVCAN_EC_FUNC1 101  # 电机1
param set UAVCAN_EC_FUNC2 102  # 电机2
param set UAVCAN_EC_FUNC3 103  # 电机3
param set UAVCAN_EC_FUNC4 104  # 电机4

# 输出范围
param set UAVCAN_EC_MIN1 1; param set UAVCAN_EC_MAX1 8191
param set UAVCAN_EC_MIN2 1; param set UAVCAN_EC_MAX2 8191
param set UAVCAN_EC_MIN3 1; param set UAVCAN_EC_MAX3 8191
param set UAVCAN_EC_MIN4 1; param set UAVCAN_EC_MAX4 8191

# 保存并重启
param save
reboot
```

## 常用命令

### 状态检查

```bash
uavcan status          # 查看UAVCAN状态
uavcan info            # 查看在线节点
listener esc_status    # 监听ESC状态
```

### 测试命令

```bash
actuator_test -m uavcan -v 1000    # 测试ESC (低速)
uavcan test                        # UAVCAN测试模式
```

### 故障排除

```bash
param show UAVCAN*     # 显示所有UAVCAN参数
dmesg | grep -i can    # 查看CAN相关日志
candump can0           # 监控CAN总线 (如果支持)
```

## 参数速查表

| 参数 | 值 | 说明 |
|------|----|----- |
| `UAVCAN_ENABLE` | 3 | 启用传感器+ESC |
| `UAVCAN_NODE_ID` | 1-125 | 节点ID |
| `UAVCAN_BITRATE` | 1000000 | 1Mbps波特率 |
| `UAVCAN_EC_FUNC*` | 101-108 | 电机1-8 |
| `UAVCAN_ESC_IDLT` | 15 | 四电机怠速 |

## 硬件连接

```
飞控 ---- CAN_H ---- ESC1 ---- ESC2 ---- ESC3 ---- ESC4
    |                                              |
    +---- CAN_L ---- ESC1 ---- ESC2 ---- ESC3 ---- ESC4 +
    |                                              |
  120Ω                                           120Ω
```

## 常见问题

### ESC不响应
1. 检查CAN连线
2. 确认终端电阻 (120Ω)
3. 验证参数设置
4. 重启系统

### CAN错误
1. 检查信号完整性
2. 确认电源稳定
3. 减少电磁干扰

## 电机功能码

| 功能码 | 描述 |
|--------|------|
| 101 | 电机1 |
| 102 | 电机2 |
| 103 | 电机3 |
| 104 | 电机4 |
| 105 | 电机5 |
| 106 | 电机6 |
| 107 | 电机7 |
| 108 | 电机8 |

## 安全检查清单

- [ ] 所有ESC在线
- [ ] 电机转向正确
- [ ] 失效保护测试
- [ ] CAN错误计数为0
- [ ] 地面测试通过

---

详细文档请参考: `PX4_UAVCAN_ESC_Usage_Guide.md`
