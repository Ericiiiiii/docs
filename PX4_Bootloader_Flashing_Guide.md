# PX4 Bootloader烧录指南

## 概述

本文档详细介绍了如何使用Debug Probe（调试探针）更新PX4飞控的bootloader。Bootloader是飞控启动时首先运行的程序，负责初始化硬件并加载主固件。

## 支持的硬件

- **调试探针**: J-Link、Black Magic Probe、OpenOCD兼容调试器
- **飞控板**: 支持SWD/JTAG接口的PX4飞控板
- **常见型号**: FMUv5、FMUv6X、FMUv6XRT、CUAV X7、Holybro等

## 前置要求

### 硬件要求
1. Debug Probe（推荐J-Link或Black Magic Probe）
2. 目标飞控板
3. Debug连接线（通常为10pin或6pin SWD线）
4. 稳定的电源供应

### 软件要求
1. ARM工具链：`arm-none-eabi-gcc`、`arm-none-eabi-gdb`
2. J-Link软件包（如使用J-Link）
3. OpenOCD（如使用OpenOCD兼容调试器）
4. PX4源码

## 方法一：使用J-Link调试器

### 1. 硬件连接

1. **连接Debug Probe到飞控**
   - 找到飞控板上的FMU-DEBUG接口（通常为6pin或10pin接口）
   - 使用SWD线连接J-Link和飞控板
   - 确保连接正确：VCC、GND、SWDIO、SWCLK

2. **供电**
   - 通过USB或外部电源为飞控供电
   - 确保电压稳定（通常3.3V或5V）

### 2. 准备Bootloader文件

#### 方法A：使用预编译的Bootloader
```bash
# 检查是否存在预编译的bootloader
ls boards/[vendor]/[board]/extras/

# 例如：CUAV X7
ls boards/cuav/x7pro/extras/cuav_x7pro_bootloader.bin
```

#### 方法B：编译Bootloader
```bash
# 编译特定板子的bootloader
make [board]_bootloader

# 例如：
make px4_fmu-v6x_bootloader
make cuav_x7pro_bootloader
make holybro_durandal-v1_bootloader
```

### 3. 转换为ELF格式

GDB需要ELF格式的文件，将bin文件转换：

```bash
# 创建构建目录
mkdir -p build/[board]_bootloader

# 转换bin到elf（注意地址0x08000000是STM32的Flash起始地址）
arm-none-eabi-objcopy -I binary -O elf32-little \
    --change-section-address .data=0x08000000 \
    boards/[vendor]/[board]/extras/[board]_bootloader.bin \
    build/[board]_bootloader/[board]_bootloader.elf

# 例如：CUAV X7
arm-none-eabi-objcopy -I binary -O elf32-little \
    --change-section-address .data=0x08000000 \
    boards/cuav/x7pro/extras/cuav_x7pro_bootloader.bin \
    build/cuav_x7pro_bootloader/cuav_x7pro_bootloader.elf
```

### 4. 启动J-Link GDB Server

```bash
# 启动J-Link GDB Server
JLinkGDBServerCLExe -device [MCU_TYPE] -if SWD -speed 4000 -port 2331

# 常见MCU类型：
# STM32F7: STM32F765II, STM32F777NI
# STM32H7: STM32H743XI, STM32H753II
# STM32F4: STM32F427VI

# 例如：CUAV X7 (STM32H743)
JLinkGDBServerCLExe -device STM32H743XI -if SWD -speed 4000 -port 2331
```

### 5. 使用GDB烧录

```bash
# 启动GDB
arm-none-eabi-gdb build/[board]_bootloader/[board]_bootloader.elf

# 在GDB中执行以下命令：
(gdb) target remote localhost:2331
(gdb) monitor swdp_scan
(gdb) attach 1
(gdb) load
(gdb) monitor reset
(gdb) monitor go
(gdb) quit
```

### 6. 使用PX4提供的脚本（推荐）

PX4提供了自动化脚本：

```bash
# 使用upload脚本
cd build/[board]_bootloader
../../platforms/nuttx/Debug/upload_jlink_gdb.sh [board]_bootloader.elf

# 例如：
cd build/cuav_x7pro_bootloader
../../platforms/nuttx/Debug/upload_jlink_gdb.sh cuav_x7pro_bootloader.elf


# 例如：
cd build/fihawk_fc-v1_default
../../platforms/nuttx/Debug/upload_jlink_gdb.sh fihawk_fc-v1_default.elf

cd build/fihawk_fc-v1_default
../../platforms/nuttx/Debug/upload_jlink_gdb.sh fihawk_fc-v1_default.elf
```

## 方法二：使用OpenOCD

### 1. 配置OpenOCD

创建或使用现有的OpenOCD配置文件：

```bash
# 使用PX4提供的配置
openocd -f .vscode/openocd-jlink-[board].cfg

# 或手动配置
openocd -f interface/jlink.cfg -f target/stm32h7x.cfg
```

### 2. 使用OpenOCD烧录

```bash
# 启动OpenOCD（在一个终端）
openocd -f interface/jlink.cfg -f target/stm32h7x.cfg

# 在另一个终端启动GDB
arm-none-eabi-gdb build/[board]_bootloader/[board]_bootloader.elf
(gdb) target remote localhost:3333
(gdb) load
(gdb) monitor reset
(gdb) quit
```

## 方法三：使用Black Magic Probe

### 1. 连接Black Magic Probe

```bash
# 找到Black Magic Probe设备
ls /dev/ttyACM*

# 通常是/dev/ttyACM0（GDB接口）和/dev/ttyACM1（串口接口）
```

### 2. 使用GDB连接

```bash
arm-none-eabi-gdb build/[board]_bootloader/[board]_bootloader.elf
(gdb) target extended-remote /dev/ttyACM0
(gdb) monitor swdp_scan
(gdb) attach 1
(gdb) load
(gdb) kill
(gdb) quit
```

## 常见飞控板配置

### FMUv6X (px4_fmu-v6x)
- **MCU**: STM32H753II
- **Flash地址**: 0x08000000
- **编译命令**: `make px4_fmu-v6x_bootloader`

### CUAV X7 Pro (cuav_x7pro)
- **MCU**: STM32H743XI
- **Flash地址**: 0x08000000
- **编译命令**: `make cuav_x7pro_bootloader`

### Holybro Durandal (holybro_durandal-v1)
- **MCU**: STM32H743II
- **Flash地址**: 0x08000000
- **编译命令**: `make holybro_durandal-v1_bootloader`

## 故障排除

### 1. 连接问题

**问题**: `Connection timed out` 或 `No target found`

**解决方案**:
- 检查硬件连接
- 确认飞控供电正常
- 检查Debug Probe是否被系统识别：`lsusb`
- 尝试降低SWD速度：`-speed 1000`

### 2. 权限问题

**问题**: `Permission denied` 访问串口设备

**解决方案**:
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER
# 重新登录或重启

# 或临时修改权限
sudo chmod 666 /dev/ttyACM0
```

### 3. 设备识别问题

**问题**: 找不到`/dev/serial/by-id/`目录

**解决方案**:
- 使用`lsusb`查看USB设备
- 使用`dmesg | grep tty`查看串口设备
- 直接使用`/dev/ttyACM0`等设备文件

### 4. Flash写入失败

**问题**: `Flash write failed`

**解决方案**:
- 检查Flash是否被写保护
- 尝试先擦除Flash：`monitor flash erase_sector 0 0 7`
- 确认MCU型号正确

## 验证更新

### 1. 硬件验证
1. 断开Debug Probe
2. 重新给飞控上电
3. 观察LED指示灯状态

### 2. 软件验证
```bash
# 检查USB设备（bootloader模式）
lsusb | grep -i px4

# 检查串口设备
ls /dev/ttyACM*

# 使用QGroundControl连接测试
```

### 3. 串口输出检查
```bash
# 监听bootloader串口输出
screen /dev/ttyACM0 57600
# 或
minicom -D /dev/ttyACM0 -b 57600
```

## 安全注意事项

1. **备份原始bootloader**（如果可能）
2. **确保电源稳定**，避免烧录过程中断电
3. **使用正确的MCU型号**，错误的配置可能损坏硬件
4. **验证bootloader文件**的完整性
5. **保持Debug连接稳定**，避免烧录过程中断开

## 参考资料

- [PX4官方文档 - Bootloader更新](https://docs.px4.io/main/en/advanced_config/bootloader_update.html)
- [J-Link用户手册](https://www.segger.com/downloads/jlink/)
- [OpenOCD用户手册](http://openocd.org/documentation/)
- [Black Magic Probe文档](https://black-magic.org/)

---

**注意**: 本文档基于PX4 v1.14+版本编写，不同版本可能存在差异。烧录bootloader存在一定风险，请谨慎操作。
