# CMake构建系统配置指南

本文档详细介绍PX4 FihawkFlyCtrl项目的CMake构建系统配置、自定义选项和高级用法。

## CMake构建系统架构

### 目录结构
```
├── CMakeLists.txt                 # 顶层CMake配置
├── cmake/                         # CMake工具和宏
│   ├── common/                    # 通用CMake工具
│   ├── configs/                   # 构建配置
│   └── platforms/                 # 平台特定配置
├── boards/                        # 板级配置文件
│   └── fihawk/fc-v1/             # Fihawk FC-V1配置
│       ├── default.px4board       # 默认板级配置
│       ├── bootloader.px4board    # 引导程序配置  
│       └── src/                   # 板级特定源码
└── platforms/                     # 平台抽象层
    ├── nuttx/                     # NuttX RTOS平台
    └── posix/                     # POSIX仿真平台
```

### 构建配置文件

#### 顶层CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.16)

# 项目定义
project(px4 CXX C ASM)

# 设置构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING 
        "Build type" FORCE)
endif()

# 包含平台配置
include(cmake/px4_config.cmake)
include(cmake/px4_base.cmake)
```

#### 板级配置文件 (.px4board)
```python
# boards/fihawk/fc-v1/default.px4board
CONFIG_BOARD_TOOLCHAIN="arm-none-eabi"
CONFIG_BOARD_ARCHITECTURE="cortex-m7"
CONFIG_BOARD_CPU="stm32h7"
CONFIG_BOARD_IO_TIMER="timer_config"
CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS0"
CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS1"
CONFIG_MODULES_SENSORS="y"
CONFIG_MODULES_COMMANDER="y"
CONFIG_MODULES_NAVIGATOR="y"
# ... 其他模块配置
```

## 构建目标配置

### 可用构建目标
```bash
# 查看所有可用目标
make list_config_targets

# 常用目标
make fihawk_fc-v1_default      # Fihawk主固件
make fihawk_fc-v1_bootloader   # Fihawk引导程序
make px4_sitl_default          # SITL仿真
make tests                     # 单元测试
```

### 自定义构建目标
```cmake
# 在 boards/fihawk/fc-v1/custom.px4board 中
CONFIG_BOARD_TOOLCHAIN="arm-none-eabi"
CONFIG_BOARD_ARCHITECTURE="cortex-m7"

# 自定义模块配置
CONFIG_MODULES_CUSTOM_DRIVER="y"
CONFIG_MODULES_EXPERIMENTAL="n"

# 自定义参数
CONFIG_PARAM_CUSTOM_DEFAULT="y"
```

## 构建选项配置

### 编译器选项
```cmake
# 在 cmake/configs/nuttx_<target>_default.cmake 中

# 优化级别
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG")

# 警告选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")

# 架构特定选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")
```

### 链接器选项
```cmake
# 链接器脚本
set(CMAKE_EXE_LINKER_FLAGS 
    "-T${BOARD_DIR}/scripts/ld.script")

# 内存布局
set(CMAKE_EXE_LINKER_FLAGS 
    "${CMAKE_EXE_LINKER_FLAGS} --specs=nosys.specs")

# 符号剥离（Release版本）
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_EXE_LINKER_FLAGS 
        "${CMAKE_EXE_LINKER_FLAGS} -s")
endif()
```

### 预处理器定义
```cmake
# 全局定义
add_definitions(-DCONFIG_ARCH_BOARD_PX4_FIHAWK_FC_V1)
add_definitions(-D__PX4_NUTTX)

# 条件编译
if(CONFIG_BOARD_SENSORS)
    add_definitions(-DBOARD_HAS_SENSORS)
endif()

# 调试模式定义
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG_BUILD)
endif()
```

## 模块配置系统

### 模块包含配置
```python
# 在 .px4board 文件中
CONFIG_MODULES_ATTITUDE_ESTIMATOR_Q="y"
CONFIG_MODULES_LOCAL_POSITION_ESTIMATOR="n"
CONFIG_MODULES_EKF2="y"
CONFIG_DRIVERS_BATT_SMBUS="y"
CONFIG_DRIVERS_GPS="y"
```

### 模块CMakeLists.txt
```cmake
# src/modules/commander/CMakeLists.txt
px4_add_module(
    MODULE modules__commander
    MAIN commander
    STACK_MAIN 3500
    PRIORITY_DEFAULT "SCHED_PRIORITY_MAX-40"
    SRCS
        commander.cpp
        commander_helper.cpp
        state_machine_helper.cpp
        # ... 其他源文件
    DEPENDS
        circuit_breaker
        commander_state
        # ... 其他依赖
    EXTERNAL_DEPENDENCIES
        Eigen3
)
```

### 驱动模块配置
```cmake
# src/drivers/gps/CMakeLists.txt
px4_add_module(
    MODULE drivers__gps
    MAIN gps
    SRCS
        gps.cpp
        devices/src/ubx.cpp
        devices/src/rtcm.cpp
        # ... 其他源文件
    DEPENDS
        drivers_gps_devices
)

# 板级特定GPS配置
if(BOARD STREQUAL "fihawk_fc-v1")
    target_compile_definitions(drivers__gps PRIVATE
        -DGPS_DEFAULT_UART_PORT="/dev/ttyS0"
        -DGPS_DEFAULT_BAUDRATE=38400
    )
endif()
```

## 平台抽象层

### NuttX平台配置
```cmake
# platforms/nuttx/CMakeLists.txt
add_definitions(-D__PX4_NUTTX)

# NuttX特定源文件
set(PX4_PLATFORM_SRCS
    src/px4_init.cpp
    src/px4_sem.cpp
    src/px4_log.cpp
    src/px4_time.cpp
    # ... 其他平台文件
)

# 系统库链接
target_link_libraries(px4
    PRIVATE
        nuttx_kernel
        nuttx_drivers  
        nuttx_arch
)
```

### POSIX平台配置
```cmake
# platforms/posix/CMakeLists.txt  
add_definitions(-D__PX4_POSIX)

# POSIX特定源文件
set(PX4_PLATFORM_SRCS
    src/px4_daemon.cpp
    src/px4_posix_impl.cpp
    src/px4_time.cpp
    # ... 其他平台文件
)

# 系统库链接
target_link_libraries(px4
    PRIVATE
        pthread
        rt
        dl
)
```

## 交叉编译配置

### ARM工具链配置
```cmake
# cmake/toolchains/arm-none-eabi.cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

# 工具链路径
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

# 工具链特性
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

# 查找模式
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
```

### STM32H7特定配置
```cmake
# cmake/configs/nuttx_fihawk-fc-v1_default.cmake
set(config_module_list
    # 系统模块
    modules/commander
    modules/navigator
    modules/ekf2
    
    # 控制模块
    modules/mc_pos_control
    modules/mc_att_control
    modules/mc_rate_control
    
    # 驱动模块
    drivers/gps
    drivers/batt_smbus
    drivers/pwm_out_sim
    
    # 板级特定
    drivers/boards/fihawk-fc-v1
)

# STM32H7特定参数
set(config_define_symbols
    CONFIG_ARCH_BOARD_PX4_FIHAWK_FC_V1
    CONFIG_ARCH_CHIP_STM32H743XI
    CONFIG_STM32H7_FLASH_CONFIG_I
)
```

## 依赖管理

### 外部依赖
```cmake
# cmake/px4_find_dependencies.cmake
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

# 可选依赖
find_package(Boost COMPONENTS system filesystem)
if(Boost_FOUND)
    add_definitions(-DHAVE_BOOST)
endif()
```

### 内部依赖
```cmake  
# 模块间依赖
px4_add_module(
    MODULE modules__mc_pos_control
    DEPENDS
        matrix           # 数学库
        geo             # 地理坐标
        flight_tasks    # 飞行任务
        land_detector   # 着陆检测
)
```

## 构建脚本

### Make包装器
```makefile
# Makefile
# 默认目标
all: fihawk_fc-v1_default

# 目标定义  
fihawk_fc-v1_default:
    @cmake -Bbuild/fihawk_fc-v1_default -H. \
        -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake \
        -DCONFIG=fihawk_fc-v1_default
    @cmake --build build/fihawk_fc-v1_default

# 清理目标
clean:
    @rm -rf build/

# 测试目标
tests:
    @cmake -Bbuild/tests -H. -DCMAKE_BUILD_TYPE=Debug
    @cmake --build build/tests --target tests
```

### 自定义构建脚本
```bash
#!/bin/bash
# tools/build.sh

set -e

BOARD=${1:-fihawk_fc-v1}
CONFIG=${2:-default}
BUILD_TYPE=${3:-Release}

BUILD_DIR="build/${BOARD}_${CONFIG}"

echo "Building ${BOARD}_${CONFIG} (${BUILD_TYPE})"

# 配置构建
cmake -B${BUILD_DIR} -H. \
    -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-none-eabi.cmake \
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
    -DCONFIG=${BOARD}_${CONFIG}

# 执行构建
cmake --build ${BUILD_DIR} -j$(nproc)

echo "Build completed: ${BUILD_DIR}/${BOARD}_${CONFIG}.elf"
```

## 调试和分析

### 构建详细信息
```bash
# 详细构建输出
make fihawk_fc-v1_default VERBOSE=1

# CMake调试信息
cmake -Bbuild/debug -H. \
    -DCMAKE_VERBOSE_MAKEFILE=ON \
    --debug-output
```

### 依赖分析
```bash
# 查看链接库
arm-none-eabi-objdump -p build/fihawk_fc-v1_default/fihawk_fc-v1_default.elf

# 查看符号表  
arm-none-eabi-nm build/fihawk_fc-v1_default/fihawk_fc-v1_default.elf

# 查看段信息
arm-none-eabi-size build/fihawk_fc-v1_default/fihawk_fc-v1_default.elf
```

## 性能优化

### 编译优化
```cmake
# 大小优化
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -DNDEBUG")

# 链接时优化(LTO)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

# 去除未使用代码
set(CMAKE_EXE_LINKER_FLAGS 
    "${CMAKE_EXE_LINKER_FLAGS} --gc-sections")
```

### 并行构建
```bash
# 使用所有CPU核心
make fihawk_fc-v1_default -j$(nproc)

# 限制并行数（避免内存不足）
make fihawk_fc-v1_default -j4
```

## 故障排除

### 常见构建错误
```bash
# CMake缓存问题
rm -rf build/
rm CMakeCache.txt

# 工具链问题
which arm-none-eabi-gcc
arm-none-eabi-gcc --version

# 依赖问题
sudo apt-get install build-essential cmake
```

### 调试技巧
```bash
# 查看构建命令
make fihawk_fc-v1_default VERBOSE=1 2>&1 | tee build.log

# 检查CMake变量
cmake -LH build/fihawk_fc-v1_default

# 生成依赖图
cmake --build build/fihawk_fc-v1_default --target help
```

## 相关文档

- [构建系统概览](README.md) - 构建系统基本概念
- [开发环境搭建](../development/) - 开发工具安装配置  
- [故障排除](../troubleshooting/) - 构建问题解决方案

---

深入理解CMake构建系统有助于进行高级定制和性能优化。如需更多帮助，请参考CMake官方文档或PX4开发者指南。