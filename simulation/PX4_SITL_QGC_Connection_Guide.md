# PX4 SITL + jMAVSim 与 QGroundControl 连接配置指南

## 环境说明

- **仿真环境**: WSL Ubuntu (运行PX4 SITL + jMAVSim)
- **地面站**:
  - Windows QGroundControl (QGC)
  - 手机/平板 QGroundControl (通过路由器连接)
- **网络**:
  - WSL与Windows主机通过虚拟网络连接
  - 通过路由器实现局域网设备连接

## 网络配置检查

### 1. 查看WSL网络配置

```bash
# 查看WSL IP地址
ip addr show

# 查看路由信息
route -n
```

**示例输出**:
- WSL IP: `172.24.196.42`
- Windows主机IP: `172.24.192.1`
- 网络段: `172.24.192.0/20`

### 3. 查看局域网配置 (用于手机连接)

```bash
# 查看Windows主机的局域网IP
# 在Windows命令提示符中执行
ipconfig

# 或在WSL中查看路由器网关
ip route show default
```

**示例局域网配置**:
- 路由器IP: `192.168.1.1`
- Windows主机局域网IP: `192.168.1.100`
- 手机IP: `192.168.1.50` (自动分配)

### 2. 测试网络连通性

```bash
# 从WSL ping Windows主机
ping 172.24.192.1

# 测试局域网连通性 (如果要连接手机)
ping 192.168.1.100  # Windows主机局域网IP
```

## PX4 SITL 启动与配置

### 1. 启动PX4 SITL + jMAVSim

```bash
cd /path/to/PX4-Autopilot
make px4_sitl jmavsim
```

### 2. 配置MAVLink网络连接

等待PX4启动完成后，在PX4控制台 (`pxh>`) 中执行以下命令：

#### 方案A: 连接Windows QGC

```bash
# 停止所有MAVLink实例
mavlink stop-all

# 启动支持网络广播的MAVLink实例 (连接Windows)
mavlink start -x -u 14550 -r 4000000 -f -p -t 172.24.192.1
mavlink start -x -u 14550 -r 4000000 -f -p -t 192.168.2.158
```

#### 方案B: 连接手机QGC (通过路由器)

```bash
# 停止所有MAVLink实例
mavlink stop-all

# 启动广播模式 (让局域网内所有设备都能发现)
mavlink start -x -u 14550 -r 4000000 -f -p
```

#### 方案C: 同时支持多个QGC连接

```bash
# 停止所有MAVLink实例
mavlink stop-all

# 启动多个MAVLink实例
# 实例1: 连接Windows QGC
mavlink start -x -u 14550 -r 4000000 -f -p -t 172.24.192.1

# 实例2: 广播给局域网设备 (包括手机)
mavlink start -x -u 14551 -r 2000000 -f -p
```

**参数说明**:
- `-x`: 启用FTP
- `-u 14550`: 本地UDP端口
- `-r 4000000`: 数据传输速率 (4MB/s)
- `-f`: 启用消息转发
- `-p`: 启用广播
- `-t 172.24.192.1`: 目标IP (Windows主机)

### 3. 验证MAVLink状态

```bash
# 检查MAVLink状态
mavlink status
```

**期望输出**:
```
instance #0:
  GCS heartbeat valid
  transport protocol: UDP (14550, remote port: 14550)
  Broadcast enabled: YES
  partner IP: 172.24.192.1
  using network interface eth0, IP: 172.24.196.42
```

## QGroundControl 连接配置

### Windows QGC 连接

#### 方法一: 手动配置连接

1. **打开QGroundControl**

2. **进入连接设置**
   - 点击左上角QGC图标
   - 选择 "Application Settings"
   - 点击 "Comm Links"

3. **添加新连接**
   - 点击 "Add" 按钮
   - 配置连接参数：
     - **Type**: UDP
     - **Listening Port**: 14550
     - **Target Hosts**: `172.24.196.42:14550` (替换为你的WSL IP)

4. **连接**
   - 点击 "OK" 保存配置
   - 选择新创建的连接
   - 点击 "Connect"

#### 方法二: 自动发现 (推荐)

由于已启用MAVLink广播，QGC通常会自动发现PX4连接：

1. **直接启动QGroundControl**
2. **等待自动连接** (通常需要5-10秒)
3. **确认连接状态** - 查看QGC界面是否显示飞行器信息

### 手机/平板 QGC 连接

#### 前提条件

1. **确保网络连通**
   - 手机和电脑连接到同一个WiFi网络
   - 电脑能够访问互联网 (WSL通过Windows主机联网)

2. **配置Windows网络共享** (如果需要)
   - 打开Windows "网络和共享中心"
   - 启用网络发现和文件共享

#### 连接步骤

1. **在手机上安装QGroundControl**
   - Android: Google Play Store
   - iOS: App Store

2. **配置手机QGC连接**
   - 打开QGC应用
   - 进入设置 → 通信链接
   - 添加新的UDP连接：
     - **类型**: UDP
     - **监听端口**: 14550 或 14551
     - **目标主机**: 留空 (使用广播模式)

3. **自动发现连接**
   - 如果使用了广播模式，QGC会自动发现PX4
   - 等待连接建立 (可能需要10-30秒)

#### 高级配置: 指定目标IP

如果自动发现失败，可以手动指定Windows主机的局域网IP：

1. **查找Windows局域网IP**
   ```cmd
   # 在Windows命令提示符中
   ipconfig | findstr "IPv4"
   ```

2. **在手机QGC中配置**
   - **目标主机**: `192.168.1.100:14550` (替换为实际IP)

#### 网络配置示例

```
路由器 (192.168.1.1)
├── Windows电脑 (192.168.1.100)
│   └── WSL Ubuntu (172.24.196.42)
│       └── PX4 SITL (MAVLink端口14550)
└── 手机 (192.168.1.50)
    └── QGroundControl App
```

## 验证连接成功

连接成功后，你应该看到：

- ✅ **QGC界面显示飞行器状态**
- ✅ **jMAVSim 3D仿真窗口**
- ✅ **能够查看传感器数据**
- ✅ **可以进行解锁、起飞等操作**

## 故障排除

### 1. 连接失败

**检查网络连通性**:
```bash
# 在WSL中测试Windows主机连接
ping 172.24.192.1

# 测试局域网连接 (手机连接时)
ping 192.168.1.100  # Windows局域网IP
```

**检查端口占用**:
```bash
# 检查14550端口是否被占用
netstat -an | grep 14550
```

### 2. Windows防火墙问题

**Windows QGC连接**:
- 打开Windows防火墙设置
- 允许QGroundControl通过防火墙
- 允许端口14550的UDP通信

**手机QGC连接**:
- 确保Windows防火墙允许局域网通信
- 在防火墙中添加端口14550的入站规则
- 或临时关闭防火墙进行测试

### 3. 手机连接特殊问题

**WiFi网络隔离**:
- 某些路由器启用了AP隔离，阻止设备间通信
- 在路由器设置中关闭"AP隔离"或"客户端隔离"
- 使用路由器的"访客网络"可能无法连接

**手机热点模式**:
如果使用手机热点，可以让电脑连接手机热点：
```bash
# 查看新的网络配置
ip addr show
# 相应调整MAVLink目标IP
```

**网络发现问题**:
```bash
# 在WSL中启用更广泛的广播
mavlink start -x -u 14550 -r 4000000 -f -p -t 255.255.255.255
```

### 3. MAVLink配置问题

**重新配置MAVLink**:
```bash
# 停止所有实例
mavlink stop-all

# 重新启动
mavlink start -x -u 14550 -r 4000000 -f -p -t <Windows_IP>
```

### 4. 端口冲突

如果14550端口被占用，可以使用其他端口：

```bash
# 使用14551端口
mavlink start -x -u 14551 -r 4000000 -f -p -t 172.24.192.1
```

相应地在QGC中也要修改为14551端口。

## 常用命令参考

### PX4控制台命令

```bash
# 查看所有MAVLink实例状态
mavlink status

# 停止特定端口的MAVLink实例
mavlink stop -u 14550

# 查看参数
param show MAV_0_BROADCAST

# 设置广播参数
param set MAV_0_BROADCAST 1
```

### 网络诊断命令

```bash
# 查看网络接口
ip addr show

# 查看路由表
route -n

# 测试连通性
ping <target_ip>

# 查看端口监听状态
netstat -tulpn | grep 14550
```

## 一键启动脚本

### 基础启动脚本

创建便捷的启动脚本 `start_px4_sitl.sh`:

```bash
#!/bin/bash

# 获取Windows主机IP (通常是默认网关)
WINDOWS_IP=$(route -n | grep '^0.0.0.0' | awk '{print $2}')

echo "检测到Windows主机IP: $WINDOWS_IP"

# 启动PX4 SITL
echo "启动PX4 SITL + jMAVSim..."
make px4_sitl jmavsim &

# 等待PX4启动
sleep 10

# 配置MAVLink (需要手动在PX4控制台执行)
echo "请在PX4控制台中执行以下命令:"
echo "mavlink stop-all"
echo "mavlink start -x -u 14550 -r 4000000 -f -p -t $WINDOWS_IP"
```

### 支持手机连接的启动脚本

创建 `start_px4_sitl_mobile.sh`:

```bash
#!/bin/bash

# 获取网络信息
WINDOWS_IP=$(route -n | grep '^0.0.0.0' | awk '{print $2}')
WSL_IP=$(ip route get 8.8.8.8 | awk '{print $7; exit}')

echo "网络配置信息:"
echo "WSL IP: $WSL_IP"
echo "Windows主机IP: $WINDOWS_IP"

# 启动PX4 SITL
echo "启动PX4 SITL + jMAVSim..."
make px4_sitl jmavsim &

# 等待PX4启动
sleep 15

echo "请在PX4控制台中执行以下命令之一:"
echo ""
echo "=== 仅连接Windows QGC ==="
echo "mavlink stop-all"
echo "mavlink start -x -u 14550 -r 4000000 -f -p -t $WINDOWS_IP"
echo ""
echo "=== 支持手机QGC连接 ==="
echo "mavlink stop-all"
echo "mavlink start -x -u 14550 -r 4000000 -f -p"
echo ""
echo "=== 同时支持Windows和手机QGC ==="
echo "mavlink stop-all"
echo "mavlink start -x -u 14550 -r 4000000 -f -p -t $WINDOWS_IP"
echo "mavlink start -x -u 14551 -r 2000000 -f -p"
echo ""
echo "手机QGC连接信息:"
echo "- 确保手机连接到同一WiFi网络"
echo "- 在QGC中监听端口14550或14551"
echo "- 等待自动发现连接"
```

## 注意事项

### 通用注意事项

1. **IP地址会变化**: WSL的IP地址可能在重启后发生变化，需要相应调整配置
2. **防火墙设置**: 确保Windows防火墙允许UDP 14550端口通信
3. **端口冲突**: 如果14550端口被占用，选择其他可用端口
4. **网络延迟**: WSL与Windows之间的网络可能有轻微延迟，属于正常现象

### 手机连接特殊注意事项

1. **WiFi网络要求**: 手机和电脑必须连接到同一个WiFi网络
2. **路由器设置**: 某些路由器的AP隔离功能会阻止设备间通信
3. **移动数据**: 手机使用移动数据时无法连接到局域网内的PX4
4. **电池优化**: Android系统可能会限制QGC的后台网络活动
5. **网络权限**: 确保QGC应用有网络访问权限

### 性能优化建议

1. **多连接时降低数据率**: 同时连接多个QGC时，适当降低MAVLink数据传输速率
2. **优先级设置**: Windows QGC使用更高的数据率，手机QGC使用较低数据率
3. **网络稳定性**: 有线连接比WiFi连接更稳定，建议电脑使用有线网络

## 参考资料

- [PX4 SITL 仿真文档](https://docs.px4.io/main/en/simulation/)
- [QGroundControl 用户指南](https://docs.qgroundcontrol.com/master/en/)
- [MAVLink 协议文档](https://mavlink.io/en/)

---

**最后更新**: 2025-06-23
**适用版本**: PX4 v1.14+, QGroundControl v4.0+
