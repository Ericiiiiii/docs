# 故障排除文档

本目录包含PX4 FihawkFlyCtrl项目开发和使用过程中常见问题的解决方案。

## 故障分类

### 构建相关问题
- 编译依赖缺失
- 工具链配置问题
- 构建错误和警告
- 链接错误

### 硬件相关问题
- 传感器故障
- 通信接口问题
- 电源和信号问题
- 固件烧录问题

### 软件相关问题
- 参数配置错误
- 模块启动失败
- 性能和实时性问题
- 通信协议问题

### 仿真相关问题
- SITL启动失败
- 仿真器连接问题
- 地面站连接问题
- 仿真环境配置问题

## 常见构建问题

### 1. 编译依赖缺失
**问题描述**: 编译时提示缺少某些依赖包或工具

**解决方案**:
```bash
# 重新运行依赖安装脚本
sudo Tools/setup/ubuntu.sh

# 手动安装特定依赖
sudo apt-get install <missing-package>

# 更新包列表
sudo apt-get update
```

### 2. ARM工具链问题
**问题描述**: 找不到ARM编译器或版本不兼容

**解决方案**:
```bash
# 检查工具链是否安装
arm-none-eabi-gcc --version

# 重新安装ARM工具链
sudo apt-get remove gcc-arm-none-eabi
sudo apt-get install gcc-arm-none-eabi

# 手动下载官方工具链（如果包版本有问题）
wget -O gcc-arm-none-eabi.tar.bz2 \
  "https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux.tar.bz2"
```

### 3. CMake配置错误
**问题描述**: CMake配置阶段失败

**解决方案**:
```bash
# 清理CMake缓存
rm -rf build/
make clean

# 重新配置
make fihawk_fc-v1_default

# 检查CMake版本
cmake --version  # 要求3.16+
```

### 4. 内存不足问题
**问题描述**: 编译过程中内存不足，编译器被杀死

**解决方案**:
```bash
# 减少并行编译线程
make fihawk_fc-v1_default -j2

# 增加交换空间
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

## 常见硬件问题

### 1. 传感器数据异常
**问题描述**: IMU、GPS等传感器数据不正常

**诊断方法**:
```bash
# 查看传感器状态
commander status
sensor_combined status

# 实时监控传感器数据
listener sensor_combined
listener vehicle_gps_position
```

**解决方案**:
- 检查传感器连接和供电
- 重新校准传感器
- 检查传感器驱动配置
- 验证硬件是否损坏

### 2. 串口通信问题
**问题描述**: 串口设备无法正常工作

**诊断方法**:
```bash
# 检查串口设备
ls /dev/tty*

# 测试串口通信
minicom -D /dev/ttyUSB0 -b 57600

# 检查串口配置
param show SER*
```

**解决方案**:
- 检查串口线缆连接
- 确认波特率和通信参数
- 检查设备权限问题
- 验证硬件信号质量

### 3. 固件烧录失败
**问题描述**: 无法成功烧录固件到硬件

**诊断方法**:
```bash
# 检查调试器连接
JLinkExe
# 在JLink命令行中输入: connect, ?

# 检查目标芯片
./boards/fihawk/fc-v1/flash_firmware.sh --verbose
```

**解决方案**:
- 检查JLink调试器连接
- 确认目标板供电正常
- 检查SWD接口连接
- 尝试不同的烧录速度

## 常见软件问题

### 1. 参数加载失败
**问题描述**: 系统启动时参数加载失败

**诊断方法**:
```bash
# 检查参数状态
param status

# 重置参数到默认值
param reset_all

# 检查参数文件
ls /fs/microsd/params*
```

**解决方案**:
- 检查SD卡或存储设备
- 重新格式化参数存储
- 恢复默认参数配置

### 2. 模块启动失败
**问题描述**: 某些关键模块无法启动

**诊断方法**:
```bash
# 查看系统状态
top
ps

# 检查模块状态
<module_name> status

# 查看启动脚本
cat /etc/init.d/rc.txt
```

**解决方案**:
- 检查模块依赖关系
- 增加模块启动延迟
- 检查资源分配冲突
- 分析模块日志输出

### 3. 实时性能问题
**问题描述**: 系统响应延迟，控制性能下降

**诊断方法**:
```bash
# 检查系统负载
top
work_queue status

# 监控任务调度
perf

# 分析任务执行时间
logger help
```

**解决方案**:
- 优化任务优先级
- 减少不必要的计算
- 检查中断处理时间
- 优化内存使用

## 仿真相关问题

### 1. SITL启动失败
**问题描述**: SITL仿真无法正常启动

**解决方案**:
```bash
# 清理构建文件
make clean
make px4_sitl_default

# 检查端口占用
netstat -tlnp | grep :14540

# 手动启动并查看详细输出
./build/px4_sitl_default/bin/px4 -d ROMFS/px4fmu_common -s etc/init.d-posix/rcS
```

### 2. QGroundControl连接问题
**问题描述**: QGC无法连接到SITL或硬件

**解决方案**:
- 检查网络连接和防火墙设置
- 确认MAVLink端口配置
- 重启QGC和PX4系统
- 检查MAVLink消息流

### 3. Gazebo仿真器问题
**问题描述**: Gazebo仿真器无法启动或模型加载失败

**解决方案**:
```bash
# 设置Gazebo环境变量
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/Tools/sitl_gazebo/models

# 重新安装Gazebo
sudo apt-get remove gazebo*
sudo apt-get install gazebo9

# 检查图形驱动
glxinfo | grep OpenGL
```

## 调试技巧

### 使用系统控制台
```bash
# 通过MAVLink shell连接
./Tools/mavlink_shell.py

# 通过串口连接
minicom -D /dev/ttyACM0

# 常用调试命令
dmesg          # 查看内核消息
ps             # 查看进程状态
free           # 查看内存使用
df             # 查看磁盘空间
```

### 日志分析
```bash
# 查看实时日志
logger help
sdlog2 help

# 导出日志文件
sdlog2 stop
sdlog2 start

# 离线分析日志
python Tools/flight_log_analysis.py log_file.ulg
```

### 性能分析
```bash
# 系统性能监控
perf start
# ... 运行一段时间后 ...
perf stop

# 查看任务执行统计
work_queue status
```

## 获取帮助

### 内置帮助系统
```bash
# 查看命令帮助
<command> help

# 查看可用命令
help
```

### 日志输出配置
```bash
# 增加日志详细程度
param set SDLOG_PROFILE 1

# 启用特定模块日志
param set CBRK_SUPPLY_CHK 894281
```

### 远程调试
```bash
# 启用MAVLink调试输出
param set MAV_0_MODE 0  # Normal
param set MAV_0_RATE 80000

# 通过网络调试
mavproxy.py --master=tcp:192.168.1.100:5760
```

## 预防性维护

### 定期检查项目
- 检查硬件连接和磨损情况
- 更新固件到最新稳定版本
- 定期校准传感器
- 备份重要参数和配置

### 开发最佳实践
- 使用版本控制管理代码变更
- 编写和维护测试用例
- 定期进行代码审查
- 保持文档同步更新

## 相关文档

- [开发环境](../development/) - 开发环境配置问题
- [构建系统](../build-system/) - 构建相关问题
- [硬件特定](../hardware-specific/) - Fihawk硬件问题
- [测试框架](../testing/) - 测试相关问题

---

遇到问题时，建议先查看相关日志输出，然后对照本文档寻找解决方案。如果问题持续存在，请在项目Issue中报告详细信息。