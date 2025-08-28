# Fihawk FlyCtrl 快速入门指南

本指南帮助新开发者快速上手Fihawk FlyCtrl项目的开发环境搭建和基本使用。

## 前提条件

### 系统要求
- Ubuntu 20.04 或更高版本（推荐 Ubuntu 22.04）
- 至少8GB RAM（推荐16GB）
- 至少50GB可用磁盘空间
- 稳定的网络连接

### 硬件要求（可选）
- Fihawk FC-V1 飞控板
- JLink调试器
- USB转TTL串口适配器
- 四旋翼测试机架

## 快速开始

### 1. 克隆代码仓库
```bash
git clone <repository-url> FihawkFlyCtrl
cd FihawkFlyCtrl
git submodule update --init --recursive
```

### 2. 安装开发环境
```bash
# 运行自动安装脚本（推荐）
sudo ./Tools/setup/ubuntu.sh

# 安装Python依赖
pip3 install -r Tools/setup/requirements.txt
```

### 3. 验证环境安装
```bash
# 检查ARM工具链
arm-none-eabi-gcc --version

# 检查CMake版本
cmake --version  # 需要 >= 3.16

# 检查Python环境
python3 -c "import numpy, empy, toml"
```

### 4. 第一次编译
```bash
# 编译Fihawk FC-V1固件
make fihawk_fc-v1_default

# 编译SITL仿真（可选）
make px4_sitl_default
```

### 5. 运行测试
```bash
# 运行单元测试
make tests

# 检查代码格式
make check_format
```

## 硬件开发工作流

### 连接Fihawk FC-V1
1. **准备硬件**
   - 连接JLink调试器到SWD接口
   - 连接USB电源或外部电源
   - 连接串口到UART2（可选，用于控制台）

2. **烧录固件**
   ```bash
   # 自动编译并烧录
   ./boards/fihawk/fc-v1/flash_firmware.sh
   
   # 强制重新编译并烧录
   ./boards/fihawk/fc-v1/flash_firmware.sh --force
   ```

3. **验证运行**
   ```bash
   # 通过串口连接控制台
   minicom -D /dev/ttyUSB0 -b 57600
   
   # 或使用MAVLink shell
   ./Tools/mavlink_shell.py
   ```

### 开发调试
1. **查看系统状态**
   ```bash
   # 在PX4控制台中执行
   commander status    # 系统状态
   sensor_combined status  # 传感器状态
   param show         # 查看参数
   ```

2. **实时监控数据**
   ```bash
   # 监控传感器数据
   listener sensor_combined
   listener vehicle_gps_position
   listener vehicle_attitude
   ```

3. **日志分析**
   ```bash
   # 启动日志记录
   sdlog2 start
   
   # 停止并下载日志
   sdlog2 stop
   # 通过MAVLink或SD卡获取.ulg日志文件
   ```

## 仿真开发工作流

### SITL仿真
```bash
# 启动SITL仿真
make px4_sitl_default gazebo

# 或者启动JMAVSim仿真
make px4_sitl_default jmavsim
```

### 连接地面站
1. **安装QGroundControl**
   ```bash
   sudo snap install qgroundcontrol --classic
   ```

2. **连接SITL**
   - 启动QGroundControl
   - 自动检测连接到localhost:14550

3. **基本飞行测试**
   - 切换到Manual模式
   - 解锁飞行器
   - 切换到Altitude模式进行飞行测试

## 常用开发任务

### 修改飞控参数
```bash
# 查看所有参数
param show

# 设置特定参数
param set MC_ROLL_P 0.8

# 保存参数到文件
param export > my_params.txt

# 从文件加载参数
param load my_params.txt
```

### 添加新传感器驱动
1. 在`src/drivers/`下创建驱动目录
2. 实现驱动类，继承基础设备类
3. 在板级配置中添加驱动启动
4. 编译测试

### 修改飞行控制算法
1. 找到相关控制模块（如`mc_pos_control`）
2. 修改控制算法代码
3. 调整相关参数
4. 在SITL中测试
5. 在真实硬件上验证

## 开发最佳实践

### 版本控制
```bash
# 创建功能分支
git checkout -b feature/my-new-feature

# 提交变更
git add .
git commit -m "feat: add new sensor driver"

# 推送到远程仓库
git push origin feature/my-new-feature
```

### 代码规范
```bash
# 自动格式化代码
make format

# 检查代码风格
make check_format

# 运行静态分析
make px4_sitl_default clang-tidy
```

### 测试驱动开发
- 始终编写单元测试
- 在SITL中验证功能
- 在硬件上进行集成测试
- 记录性能数据

## 常见问题解决

### 编译问题
```bash
# 清理并重新编译
make clean
make fihawk_fc-v1_default

# 检查依赖
sudo ./Tools/setup/ubuntu.sh --fix-missing
```

### 烧录问题
```bash
# 检查JLink连接
JLinkExe
# 在JLink命令行输入: connect

# 检查目标板供电
# 确保SWD接口连接正确
```

### 仿真问题
```bash
# 检查端口占用
netstat -tlnp | grep 14540

# 重置仿真环境
pkill -f px4
pkill -f gazebo
make clean
```

## 进一步学习

### 推荐阅读顺序
1. [构建系统文档](../build-system/) - 了解构建配置
2. [PX4四旋翼控制架构](../flight-control/PX4_Quadcopter_Control_Architecture.md) - 理解系统架构
3. [驱动开发文档](../drivers/) - 学习驱动框架
4. [参数系统文档](../parameters/) - 掌握参数配置

### 社区资源
- [PX4官方文档](https://dev.px4.io/)
- [PX4讨论社区](https://discuss.px4.io/)
- [GitHub Issues](https://github.com/PX4/PX4-Autopilot/issues)

---

通过本快速入门指南，你应该能够成功搭建开发环境并开始Fihawk FlyCtrl项目的开发工作。遇到问题时，请参考[故障排除文档](../troubleshooting/)或向社区求助。