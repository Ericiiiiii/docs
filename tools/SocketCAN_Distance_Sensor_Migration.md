# SocketCAN Distance Sensor Migration Guide

## 概述

本文档描述了将Fihawk飞控中的距离传感器CAN接口从传统CAN设备接口迁移到SocketCAN接口的修改。这些修改使得H7处理器能够使用SocketCAN进行CAN通信。

## 修改的驱动

以下距离传感器驱动已经更新以支持SocketCAN：

1. **BenewakeRadar** - 统一的Benewake雷达驱动（TR24DA100C, TR24DCA100）
2. **TR24DCA100** - 障碍物避让雷达传感器
3. **TR24DA100C** - 高度计雷达传感器

## 主要修改内容

### 1. 头文件包含

为每个驱动添加了条件编译的头文件包含：

```cpp
#ifdef CONFIG_NET_CAN
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <net/if.h>
#include <netinet/in.h>
#include <netpacket/can.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
extern "C" {
#include <nuttx/can.h>
}
#else
extern "C" {
#include <nuttx/can/can.h>
}
#endif
```

### 2. 构造函数修改

修改了CAN设备名称的设置逻辑：

```cpp
#ifdef CONFIG_NET_CAN
    // Store CAN interface name for SocketCAN (e.g., "can0", "can1")
    snprintf(_can_device, sizeof(_can_device), "%s", can_interface);
#else
    // Store CAN device name for traditional CAN (e.g., "/dev/can0", "/dev/can1")
    snprintf(_can_device, sizeof(_can_device), "/dev/%s", can_interface);
#endif
```

### 3. open_can_device() 方法

实现了双重CAN接口支持：

**SocketCAN实现：**
- 创建PF_CAN套接字
- 获取网络接口索引
- 绑定到CAN接口
- 设置非阻塞模式

**传统CAN实现：**
- 使用px4_open()打开设备文件
- 设置非阻塞标志

### 4. close_can_device() 方法

根据编译配置选择合适的关闭方法：

```cpp
#ifdef CONFIG_NET_CAN
    close(_can_fd);
#else
    px4_close(_can_fd);
#endif
```

### 5. read_can_message() 方法

实现了两种不同的消息读取方式：

**SocketCAN：**
- 读取`struct can_frame`
- 转换为`struct can_msg_s`格式进行解析

**传统CAN：**
- 直接读取`struct can_msg_s`

## 编译配置

修改依赖于`CONFIG_NET_CAN`宏定义：

- 当定义了`CONFIG_NET_CAN`时，使用SocketCAN接口
- 未定义时，使用传统CAN设备接口

## 兼容性

这些修改保持了向后兼容性：

1. **API兼容性** - 所有公共接口保持不变
2. **参数兼容性** - 启动参数格式保持一致
3. **功能兼容性** - 数据解析和处理逻辑完全相同

## 使用方法

驱动的使用方法保持不变：

```bash
# 启动BenewakeRadar驱动
benewake_radar start -i can0 -c 5 -R 25

# 启动TR24DCA100驱动  
tr24dca100 start -i can0 -R 0

# 启动TR24DA100C驱动
tr24da100c start -i can0 -R 25
```

## 测试建议

1. **功能测试** - 验证传感器数据读取正常
2. **性能测试** - 确认SocketCAN性能满足要求
3. **稳定性测试** - 长时间运行测试
4. **兼容性测试** - 在不同硬件平台上测试

## 注意事项

1. 确保系统已正确配置SocketCAN支持
2. 验证CAN接口名称（can0, can1等）正确
3. 检查CAN总线波特率设置
4. 监控系统资源使用情况

## 相关文件

- `src/drivers/distance_sensor/benewake_radar/BenewakeRadar.cpp`
- `src/drivers/distance_sensor/tr24dca100/TR24DCA100.cpp`
- `src/drivers/distance_sensor/tr24da100c/TR24DA100C.cpp`
- `boards/fihawk/fc-v1/src/can.c`
- `boards/fihawk/fc-v1/src/can_test.c`
