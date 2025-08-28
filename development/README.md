# 开发环境文档

本目录包含PX4 FihawkFlyCtrl项目的开发环境搭建和开发流程相关文档。

## 开发环境搭建

### 系统要求
- **Ubuntu 20.04+** 或 **macOS 10.15+**
- **8GB RAM** 以上（推荐16GB）
- **50GB** 可用磁盘空间
- **稳定的网络连接**（用于下载依赖）

### 快速搭建
```bash
# Ubuntu系统
sudo Tools/setup/ubuntu.sh

# macOS系统
Tools/setup/macos.sh

# 安装Python依赖
pip install -r Tools/setup/requirements.txt
```

### 手动安装依赖

#### 必需工具
```bash
# 基础开发工具
sudo apt-get install git cmake build-essential

# ARM交叉编译工具链
sudo apt-get install gcc-arm-none-eabi

# Python开发环境
sudo apt-get install python3-pip python3-dev

# 调试工具
sudo apt-get install gdb-multiarch openocd
```

#### 可选工具
```bash
# 代码编辑器
sudo snap install --classic code

# 仿真环境
sudo apt-get install gazebo9

# 地面站
sudo apt-get install qgroundcontrol
```

## 开发工具配置

### Git配置
```bash
# 配置用户信息
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# 配置编辑器
git config --global core.editor "code --wait"

# 配置自动换行
git config --global core.autocrlf input
```

### VSCode配置
推荐安装以下扩展：
- C/C++ Extension Pack
- CMake Tools
- Python
- GitLens
- Markdown All in One

### 调试器配置
```json
// .vscode/launch.json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "PX4 SITL Debug",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/build/px4_sitl_default/bin/px4",
            "args": ["-d", "${workspaceFolder}/ROMFS/px4fmu_common", "-s", "etc/init.d-posix/rcS"],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}/build/px4_sitl_default/tmp",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb"
        }
    ]
}
```

## 开发流程

### 代码开发流程
1. **创建分支**: `git checkout -b feature/your-feature-name`
2. **开发功能**: 编写代码和测试
3. **运行测试**: `make tests`
4. **代码格式**: `make format`
5. **提交代码**: 遵循提交信息规范
6. **创建PR**: 通过代码审查后合并

### 提交信息规范
```
<type>(<scope>): <subject>

<body>

<footer>
```

类型说明：
- `feat`: 新功能
- `fix`: 问题修复
- `docs`: 文档更新
- `style`: 代码格式
- `refactor`: 重构
- `test`: 测试相关
- `chore`: 构建过程或辅助工具的变动

### 代码审查规范
- **功能正确性**: 代码实现是否符合需求
- **代码质量**: 遵循编码规范，逻辑清晰
- **性能考虑**: 避免性能瓶颈和资源浪费
- **测试覆盖**: 包含充分的单元测试
- **文档完整**: 更新相关文档

## 开发工具使用

### 编译和构建
```bash
# 增量编译
make fihawk_fc-v1_default

# 清理重建
make clean && make fihawk_fc-v1_default

# 并行编译
make fihawk_fc-v1_default -j$(nproc)

# 详细输出
make fihawk_fc-v1_default VERBOSE=1
```

### 调试技巧
```bash
# GDB调试SITL
make px4_sitl_default
gdb build/px4_sitl_default/bin/px4

# 硬件调试（JLink）
JLinkGDBServerCLExe -device STM32H743XI -if SWD -speed 4000
arm-none-eabi-gdb build/fihawk_fc-v1_default/fihawk_fc-v1_default.elf
```

### 日志分析
```bash
# 实时日志查看
./Tools/mavlink_shell.py

# 离线日志分析
python Tools/flight_log_analysis.py <log_file>

# 参数导出
param export > params.txt
```

## Fihawk特定开发

### 硬件调试环境
- **JLink调试器**: 用于固件烧录和调试
- **串口调试**: 系统控制台和MAVLink通信
- **示波器**: 信号时序分析

### 固件开发流程
1. **修改代码**: 更新源码
2. **编译固件**: `make fihawk_fc-v1_default`
3. **烧录测试**: `./boards/fihawk/fc-v1/flash_firmware.sh`
4. **功能验证**: 通过QGC或MAVLink测试
5. **日志分析**: 检查系统运行状态

### 板级配置修改
- **引脚配置**: `boards/fihawk/fc-v1/src/board_config.h`
- **传感器配置**: `boards/fihawk/fc-v1/default.px4board`
- **NuttX配置**: `boards/fihawk/fc-v1/nuttx-config/`

## 性能优化

### 编译优化
```bash
# 发布版本（优化性能）
make fihawk_fc-v1_default CMAKE_BUILD_TYPE=Release

# 调试版本（便于调试）
make fihawk_fc-v1_default CMAKE_BUILD_TYPE=Debug

# 大小优化
make fihawk_fc-v1_default CMAKE_BUILD_TYPE=MinSizeRel
```

### 代码优化技巧
- **避免动态内存分配**: 使用静态数组和对象池
- **减少拷贝操作**: 使用引用和指针
- **优化循环**: 减少不必要的计算
- **使用编译器优化**: 合理使用内联函数

### 实时性优化
- **任务优先级**: 合理设置任务优先级
- **中断延迟**: 减少中断处理时间
- **内存对齐**: 优化数据结构对齐
- **缓存优化**: 提高数据访问局部性

## 故障排除

### 编译问题
- **依赖缺失**: 重新运行setup脚本
- **权限问题**: 检查文件和目录权限
- **路径问题**: 确保工具链在PATH中
- **版本冲突**: 检查工具版本兼容性

### 调试问题
- **连接问题**: 检查调试器和目标板连接
- **固件版本**: 确保调试信息与固件匹配
- **权限问题**: 可能需要sudo权限访问调试接口
- **时序问题**: 调整调试器速度和时序

## 相关文档

- [构建系统](../build-system/) - 构建配置和选项
- [测试框架](../testing/) - 开发过程中的测试
- [故障排除](../troubleshooting/) - 常见开发问题解决
- [硬件特定](../hardware-specific/) - Fihawk硬件开发详情

---

良好的开发环境是高效开发的基础，建议花时间正确配置开发环境和工具链。