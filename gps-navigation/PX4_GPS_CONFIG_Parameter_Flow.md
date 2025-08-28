# PX4 GPS_1_CONFIG参数判断流程详解

## 概述

本文档详细解释了PX4中`GPS_1_CONFIG`参数是如何被系统判断和使用的，以及它如何决定GPS驱动绑定到哪个串口设备。

## 1. 参数编码系统

### 1.1 编码规则
PX4使用3位数字编码来标识串口功能和端口：
```
XYZ格式：
- X: 功能类型 (1=TELEM, 2=GPS, 3=RADIO, 4=其他)
- YZ: 端口编号 (01=第1个端口, 02=第2个端口)
```

### 1.2 常用编码映射
```
101 → TEL1  → TELEM第1个端口 → /dev/ttyS1
102 → TEL2  → TELEM第2个端口 → /dev/ttyS3
201 → GPS1  → GPS第1个端口  → /dev/ttyS0
202 → GPS2  → GPS第2个端口  → /dev/ttyS2
```

## 2. 参数定义源头

### 2.1 模块配置文件 (`src/drivers/gps/module.yaml`)
```yaml
module_name: GPS
serial_config:
    - command: gps start -d ${SERIAL_DEV} -b p:${BAUD_PARAM} ${DUAL_GPS_ARGS}
      port_config_param:
        name: GPS_1_CONFIG        # 参数名
        group: GPS
        default: GPS1             # 默认值：GPS1标识符
      label: Main GPS
```

**作用**：定义GPS驱动需要一个名为`GPS_1_CONFIG`的串口配置参数。

### 2.2 参数编码定义 (`Tools/serial/generate_config.py`)
```python
serial_ports = {
    "TEL1": {
        "label": "TELEM 1", 
        "index": 101,        # GPS_1_CONFIG = 101的来源
        "default_baudrate": 57600,
    },
    "GPS1": {
        "label": "GPS 1",
        "index": 201,        # GPS_1_CONFIG = 201的来源
        "default_baudrate": 0,
    },
}
```

**作用**：定义每个端口标识符对应的数字编码。

### 2.3 板级配置文件 (`boards/fihawk/fc-v1/default.px4board`)
```
CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS0"   # GPS1 → /dev/ttyS0
CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS1"   # TEL1 → /dev/ttyS1
CONFIG_BOARD_SERIAL_GPS2="/dev/ttyS2"   # GPS2 → /dev/ttyS2
CONFIG_BOARD_SERIAL_TEL2="/dev/ttyS3"   # TEL2 → /dev/ttyS3
```

**作用**：定义每个端口标识符对应的实际设备节点。

## 3. 启动脚本生成流程

### 3.1 脚本生成过程
```bash
# 编译时执行
python Tools/serial/generate_config.py \
    --serial-ports GPS1:/dev/ttyS0 TEL1:/dev/ttyS1 \
    --config-files src/drivers/gps/module.yaml
```

### 3.2 生成的启动脚本模板 (`Tools/serial/rc.serial_port.jinja`)
```bash
set SERIAL_DEV none

# 当GPS_1_CONFIG = 201时
if param compare "$PRT" 201; then
    if [ "x$PRT_GPS1_" = "x" ]; then
        set SERIAL_DEV /dev/ttyS0      # 来自CONFIG_BOARD_SERIAL_GPS1
        set BAUD_PARAM SER_GPS1_BAUD
        set PRT_GPS1_ 1
    fi
fi

# 当GPS_1_CONFIG = 101时  
if param compare "$PRT" 101; then
    if [ "x$PRT_TEL1_" = "x" ]; then
        set SERIAL_DEV /dev/ttyS1      # 来自CONFIG_BOARD_SERIAL_TEL1
        set BAUD_PARAM SER_TEL1_BAUD
        set PRT_TEL1_ 1
    fi
fi
```

### 3.3 实际生成的启动脚本 (`ROMFS/px4fmu_common/init.d/rc.serial_port`)
编译后会生成包含所有端口判断逻辑的启动脚本。

## 4. 系统启动时的参数判断

### 4.1 启动脚本执行 (`ROMFS/px4fmu_common/init.d/rc.serial`)
```bash
# 从module.yaml生成的GPS启动命令
set PRT p:GPS_1_CONFIG                    # 读取GPS_1_CONFIG参数值
. ${R}etc/init.d/rc.serial_port          # 执行端口判断脚本

# 根据判断结果启动GPS驱动
gps start -d ${SERIAL_DEV} -b p:${BAUD_PARAM}
```

### 4.2 参数判断过程
```bash
# 假设用户设置了 GPS_1_CONFIG = 201
set PRT 201                              # PRT变量 = 201

# rc.serial_port脚本执行判断
if param compare "$PRT" 201; then       # 201 == 201，条件成立
    set SERIAL_DEV /dev/ttyS0           # 设置设备为GPS1端口
    set BAUD_PARAM SER_GPS1_BAUD        # 设置波特率参数
fi

# 最终执行
gps start -d /dev/ttyS0 -b p:SER_GPS1_BAUD
```

## 5. GPS驱动内部的参数处理

### 5.1 协议选择 (`src/drivers/gps/gps.cpp`)
```cpp
// GPS驱动读取GPS_1_PROTOCOL参数来确定协议
char protocol_param_name[17];
snprintf(protocol_param_name, sizeof(protocol_param_name), "GPS_%i_PROTOCOL", (int)_instance + 1);
int32_t protocol = 0;
param_get(param_find(protocol_param_name), &protocol);

switch (protocol) {
case 1: _mode = gps_driver_mode_t::UBX; break;
case 6: _mode = gps_driver_mode_t::NMEA; break;  // RTK982使用NMEA
}
```

### 5.2 串口初始化
```cpp
// GPS驱动使用启动脚本传入的设备路径
GPS::GPS(const char *path, ...) {
    strncpy(_port, path, sizeof(_port) - 1);  // path = "/dev/ttyS0"
    _port[sizeof(_port) - 1] = '\0';
}
```

## 6. 完整的参数流转链条

```
1. 用户设置参数:
   param set GPS_1_CONFIG 201

2. 系统启动时读取:
   set PRT p:GPS_1_CONFIG  → PRT = 201

3. 启动脚本判断:
   if param compare "$PRT" 201  → 条件成立

4. 查找板级配置:
   201 → GPS1 → CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS0"

5. 设置环境变量:
   set SERIAL_DEV /dev/ttyS0
   set BAUD_PARAM SER_GPS1_BAUD

6. 启动GPS驱动:
   gps start -d /dev/ttyS0 -b p:SER_GPS1_BAUD

7. GPS驱动绑定:
   GPS驱动打开 /dev/ttyS0 设备
```

## 7. 关键文件总结

| 文件 | 作用 | GPS_1_CONFIG相关内容 |
|------|------|---------------------|
| `src/drivers/gps/module.yaml` | 定义GPS模块需要串口配置 | `name: GPS_1_CONFIG` |
| `Tools/serial/generate_config.py` | 定义参数编码映射 | `"GPS1": {"index": 201}` |
| `boards/fihawk/fc-v1/default.px4board` | 定义硬件端口映射 | `CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS0"` |
| `Tools/serial/rc.serial_port.jinja` | 启动脚本模板 | `if param compare "$PRT" 201` |
| `ROMFS/px4fmu_common/init.d/rc.serial` | 实际启动脚本 | 执行GPS驱动启动命令 |
| `src/drivers/gps/gps.cpp` | GPS驱动实现 | 处理串口设备和协议参数 |

## 8. 调试方法

### 8.1 检查参数设置
```bash
param show GPS_1_CONFIG    # 查看当前设置值
param show GPS_1_PROTOCOL  # 查看协议设置
```

### 8.2 检查启动脚本生成
```bash
# 查看生成的启动脚本
cat ROMFS/px4fmu_common/init.d/rc.serial_port
```

### 8.3 检查GPS驱动状态
```bash
gps status                 # 查看GPS驱动状态
ps | grep gps             # 查看GPS进程
```

## 9. 常见问题

### 9.1 GPS绑定到错误端口
**原因**：`GPS_1_CONFIG`参数设置错误
**解决**：
```bash
param set GPS_1_CONFIG 201  # 绑定到GPS1端口(/dev/ttyS0)
param set GPS_1_CONFIG 101  # 绑定到TEL1端口(/dev/ttyS1)
```

### 9.2 GPS驱动无法启动
**原因**：板级配置中缺少对应的设备定义
**解决**：检查`default.px4board`中是否定义了对应的`CONFIG_BOARD_SERIAL_*`

### 9.3 参数不生效
**原因**：需要重启系统或重新启动GPS驱动
**解决**：
```bash
param save
reboot
```

这个参数系统的设计保证了**灵活性**和**一致性**：用户可以通过简单的参数配置将GPS连接到任何可用的串口，而不需要修改代码或重新编译固件。
