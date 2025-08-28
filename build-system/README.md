# 构建系统文档

本目录包含PX4 FihawkFlyCtrl项目的构建系统相关文档，涵盖CMake构建配置、交叉编译、依赖管理等内容。

## 文档概述

本项目基于CMake构建系统，支持多平台交叉编译，包括：
- Fihawk FC-V1硬件平台构建
- SITL仿真环境构建
- 单元测试和集成测试构建
- 工具链配置和依赖管理

## 核心构建命令

### 基本构建命令
```bash
# 构建Fihawk FC-V1默认配置
make fihawk_fc-v1_default

# 构建Fihawk FC-V1引导程序
make fihawk_fc-v1_bootloader

# 构建SITL仿真
make px4_sitl_default

# 清理构建
make clean

# 查看所有可用配置
make list_config_targets
```

### 高级构建选项
```bash
# 并行构建（加速编译）
make fihawk_fc-v1_default -j8

# 详细构建输出
make fihawk_fc-v1_default VERBOSE=1

# 调试版本构建
make fihawk_fc-v1_debug

# 发布版本构建
make fihawk_fc-v1_default CMAKE_BUILD_TYPE=Release
```

## 构建系统架构

### 目录结构
```
├── CMakeLists.txt          # 顶层CMake配置
├── Makefile               # Make包装器
├── cmake/                 # CMake工具和宏
├── boards/                # 板级配置文件
│   └── fihawk/
│       └── fc-v1/         # Fihawk FC-V1特定配置
└── platforms/             # 平台特定代码
    ├── nuttx/             # NuttX RTOS平台
    └── posix/             # POSIX平台（仿真）
```

### 关键配置文件
- `boards/fihawk/fc-v1/default.px4board` - Fihawk板级配置
- `boards/fihawk/fc-v1/nuttx-config/` - NuttX内核配置
- `platforms/nuttx/cmake/` - NuttX构建工具链

## 构建流程

1. **配置阶段**: CMake解析配置文件，生成构建文件
2. **编译阶段**: 编译源代码，生成目标文件
3. **链接阶段**: 链接生成最终固件
4. **后处理**: 生成固件映像和调试信息

## 故障排除

### 常见构建问题
- **依赖缺失**: 运行`Tools/setup/ubuntu.sh`安装依赖
- **工具链问题**: 检查ARM工具链安装和PATH配置
- **内存不足**: 减少并行构建线程数
- **权限问题**: 检查文件和目录权限

### 调试方法
- 使用`VERBOSE=1`查看详细构建日志
- 检查`build/`目录下的构建文件
- 查看CMake缓存文件`CMakeCache.txt`

## 相关文档

- [开发环境搭建](../development/) - 开发环境配置
- [测试框架](../testing/) - 单元测试和集成测试
- [故障排除](../troubleshooting/) - 常见问题解决方案

---

有关构建系统的详细信息，请参考项目根目录的`CLAUDE.md`文件。