# PX4 SPI驱动架构详解

本文档详细介绍PX4的SPI驱动抽象层次，从board目录的IO配置到底层NuttX驱动的完整数据流转过程。

## 1. 整体架构概览

PX4的SPI驱动采用分层架构设计，从上到下分为以下几层：

```
┌─────────────────────────────────────────────────────────────┐
│                    应用层 (Drivers)                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │ ICM20689    │  │ BMI088      │  │ MS5611      │  ...    │
│  │ Driver      │  │ Driver      │  │ Driver      │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
└─────────────────────────────────────────────────────────────┘
                           │
┌─────────────────────────────────────────────────────────────┐
│                PX4 SPI 抽象层                                │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │        device::SPI 基类                                 │ │
│  │  - transfer()                                          │ │
│  │  - init()                                              │ │
│  │  - probe()                                             │ │
│  └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                           │
┌─────────────────────────────────────────────────────────────┐
│              Board 配置层                                    │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │  boards/xxx/src/spi.cpp                                │ │
│  │  - px4_spi_buses[] 配置                                │ │
│  │  - GPIO CS/DRDY 引脚映射                               │ │
│  │  - 设备类型定义                                        │ │
│  └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                           │
┌─────────────────────────────────────────────────────────────┐
│            平台抽象层                                        │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │  platforms/nuttx/src/px4/stm/stm32_common/spi/spi.cpp │ │
│  │  - stm32_spiinitialize()                              │ │
│  │  - GPIO 配置                                           │ │
│  │  - CS 控制逻辑                                         │ │
│  └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                           │
┌─────────────────────────────────────────────────────────────┐
│                NuttX SPI 驱动                                │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │  - SPI_SELECT()                                        │ │
│  │  - SPI_EXCHANGE()                                      │ │
│  │  - 硬件寄存器操作                                       │ │
│  └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## 2. Board配置层 - 硬件描述

### 2.1 配置文件位置
每个板子的SPI配置位于：`boards/{vendor}/{board}/src/spi.cpp`

### 2.2 配置示例 (CUAV X7 Pro)

```cpp
constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
    initSPIBus(SPI::Bus::SPI1, {
        initSPIDevice(DRV_IMU_DEVTYPE_ADIS16470,
                     SPI::CS{GPIO::PortF, GPIO::Pin10},
                     SPI::DRDY{GPIO::PortE, GPIO::Pin7}),
        initSPIDevice(DRV_IMU_DEVTYPE_ICM20689,
                     SPI::CS{GPIO::PortG, GPIO::Pin6},
                     SPI::DRDY{GPIO::PortJ, GPIO::Pin0}),
    }, {GPIO::PortE, GPIO::Pin3}), // 电源控制引脚

    initSPIBus(SPI::Bus::SPI4, {
        initSPIDevice(DRV_ACC_DEVTYPE_BMI088,
                     SPI::CS{GPIO::PortF, GPIO::Pin3},
                     SPI::DRDY{GPIO::PortB, GPIO::Pin15}),
        initSPIDevice(DRV_GYR_DEVTYPE_BMI088,
                     SPI::CS{GPIO::PortF, GPIO::Pin4},
                     SPI::DRDY{GPIO::PortB, GPIO::Pin14}),
    }),
};
```

### 2.3 配置参数说明

- **物理SPI总线**: `SPI::Bus::SPI1`, `SPI::Bus::SPI4`
- **设备类型**: `DRV_IMU_DEVTYPE_ADIS16470` (用于驱动匹配)
- **CS引脚**: `SPI::CS{GPIO::PortF, GPIO::Pin10}` (片选信号)
- **DRDY引脚**: `SPI::DRDY{GPIO::PortE, GPIO::Pin7}` (数据就绪中断)
- **电源控制引脚**: `{GPIO::PortE, GPIO::Pin3}` (传感器电源开关)

## 3. 核心数据结构

### 3.1 设备描述结构

```cpp
struct px4_spi_bus_device_t {
    uint32_t cs_gpio;           ///< 片选GPIO (0表示未使用)
    spi_drdy_gpio_t drdy_gpio;  ///< 数据就绪GPIO (0表示未设置)
    uint32_t devid;             ///< SPIDEV_ID(type,index)
    uint16_t devtype_driver;    ///< 驱动设备类型，如DRV_IMU_DEVTYPE_ICM20689
};
```

### 3.2 总线描述结构

```cpp
struct px4_spi_bus_t {
    px4_spi_bus_device_t devices[SPI_BUS_MAX_DEVICES];  ///< 总线上的设备列表
    uint32_t power_enable_gpio{0};  ///< 电源控制GPIO
    int8_t bus{-1};                 ///< 物理总线号 (1, 2, 3...)
    bool is_external;               ///< 是否为外部总线
    bool requires_locking;          ///< 是否需要总线锁
};
```

### 3.3 硬件版本支持

```cpp
struct px4_spi_bus_all_hw_t {
    px4_spi_bus_t buses[SPI_BUS_MAX_BUS_ITEMS];
    #if defined(BOARD_HAS_HW_SPLIT_VERSIONING)
    hw_fmun_id_t board_hw_fmun_id {USHRT_MAX};
    #else
    int board_hw_version_revision {-1};  ///< 硬件版本号
    #endif
};
```

## 4. 平台初始化层

### 4.1 初始化流程

```cpp
__EXPORT void stm32_spiinitialize()
{
    // 1. 根据硬件版本设置总线配置
    px4_set_spi_buses_from_hw_version();

    // 2. 配置电源控制GPIO
    board_control_spi_sensors_power_configgpio();

    // 3. 打开传感器电源
    board_control_spi_sensors_power(true, 0xffff);

    // 4. 建立总线映射
    for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i) {
        switch (px4_spi_buses[i].bus) {
        case 1: _spi_bus1 = &px4_spi_buses[i]; break;
        case 2: _spi_bus2 = &px4_spi_buses[i]; break;
        // ...
        }
    }

    // 5. 配置CS引脚
    if (board_has_bus(BOARD_SPI_BUS, 1)) {
        spi_bus_configgpio_cs(_spi_bus1);
    }
}
```

### 4.2 CS引脚配置

```cpp
static void spi_bus_configgpio_cs(const px4_spi_bus_t *bus)
{
    for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
        if (bus->devices[i].cs_gpio != 0) {
            px4_arch_configgpio(bus->devices[i].cs_gpio);  // 配置为GPIO输出
        }
    }
}
```

## 5. CS信号控制机制

### 5.1 CS控制逻辑

```cpp
static inline void stm32_spixselect(const px4_spi_bus_t *bus,
                                   struct spi_dev_s *dev,
                                   uint32_t devid,
                                   bool selected)
{
    for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
        if (bus->devices[i].cs_gpio == 0) {
            break;
        }

        if (devid == bus->devices[i].devid) {
            // SPI CS是低电平有效，所以写入!selected来选择设备
            stm32_gpiowrite(bus->devices[i].cs_gpio, !selected);
        }
    }
}
```

### 5.2 CS信号时序

```
时间轴: ----[开始]----[数据传输]----[结束]----

CS信号:     ↓                        ↑
         拉低(选中)              拉高(取消选中)

CLK信号:      ︿︿︿︿︿︿︿︿︿︿︿︿

MOSI/MISO:   ←------ 数据传输 ------→
```

**CS拉低的时机**：
1. **SPI事务开始前** - 选中目标设备
2. **整个数据传输期间保持低电平**
3. **数据传输完成后拉高** - 取消选中设备

## 6. 设备驱动层接口

### 6.1 SPI设备基类

```cpp
class SPI : public CDev
{
public:
    SPI(uint8_t device_type, const char *name, int bus,
        uint32_t device, enum spi_mode_e mode, uint32_t frequency);

    virtual ~SPI();

    virtual int init();
    virtual int probe() = 0;

protected:
    int transfer(uint8_t *send, uint8_t *recv, unsigned len);
    int transferhword(uint16_t *send, uint16_t *recv, unsigned len);

private:
    struct spi_dev_s *_dev{nullptr};
    uint32_t _device;
    enum spi_mode_e _mode;
    uint32_t _frequency;
};
```

### 6.2 设备初始化流程

```cpp
int SPI::init()
{
    /* 连接到SPI总线 */
    if (_dev == nullptr) {
        int bus = get_device_bus();

        if (!board_has_bus(BOARD_SPI_BUS, bus)) {
            return -ENOENT;
        }

        _dev = px4_spibus_initialize(bus);  // 获取NuttX SPI设备句柄
    }

    if (_dev == nullptr) {
        DEVICE_DEBUG("failed to init SPI");
        return -ENOENT;
    }

    /* 取消选择设备，确保CS引脚从高到低的跳变 */
    SPI_SELECT(_dev, _device, false);

    /* 调用probe函数检查设备是否存在 */
    int ret = probe();

    if (ret != OK) {
        DEVICE_DEBUG("probe failed");
        return ret;
    }

    /* 执行基类初始化，创建设备节点等 */
    ret = CDev::init();

    return ret;
}
```

### 6.3 数据传输接口

```cpp
int SPI::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
    if (_dev == nullptr) {
        return -ENODEV;
    }

    /* 锁定SPI总线 */
    if (_locking_mode != LOCK_NONE) {
        SPI_LOCK(_dev, true);
    }

    /* 配置SPI参数 */
    SPI_SETFREQUENCY(_dev, _frequency);
    SPI_SETMODE(_dev, _mode);
    SPI_SETBITS(_dev, 8);

    /* 选择设备 - CS拉低 */
    SPI_SELECT(_dev, _device, true);

    /* 执行数据交换 */
    SPI_EXCHANGE(_dev, send, recv, len);

    /* 取消选择设备 - CS拉高 */
    SPI_SELECT(_dev, _device, false);

    /* 释放SPI总线 */
    if (_locking_mode != LOCK_NONE) {
        SPI_LOCK(_dev, false);
    }

    return OK;
}
```

## 7. 完整的数据流转过程

### 7.1 系统启动阶段

```
1. 系统启动
   ↓
2. board_app_initialize()
   ↓
3. stm32_spiinitialize()
   ↓
4. px4_set_spi_buses_from_hw_version()
   ↓
5. 读取 px4_spi_buses[] 配置
   ↓
6. 配置GPIO引脚 (CS, DRDY, Power)
   ↓
7. 建立总线映射 (_spi_bus1, _spi_bus2, ...)
   ↓
8. 初始化NuttX SPI控制器
```

### 7.2 设备注册阶段

```
1. 传感器驱动模块加载
   ↓
2. 调用 SPI构造函数
   ↓
3. 指定 device_type, bus, device_id
   ↓
4. 调用 init()
   ↓
5. px4_spibus_initialize(bus)
   ↓
6. 获得NuttX spi_dev_s句柄
   ↓
7. 调用 probe() 检测设备存在
   ↓
8. 注册设备节点
```

### 7.3 数据传输阶段

```
1. 驱动调用 transfer(send, recv, len)
   ↓
2. SPI_LOCK() - 锁定总线
   ↓
3. SPI_SETFREQUENCY/MODE/BITS() - 配置参数
   ↓
4. SPI_SELECT(_dev, _device, true) - CS拉低
   ↓
5. 查找对应的px4_spi_bus_t和设备配置
   ↓
6. stm32_spixselect() → stm32_gpiowrite(cs_gpio, 0)
   ↓
7. SPI_EXCHANGE() → NuttX硬件驱动
   ↓
8. 硬件寄存器操作，时钟和数据传输
   ↓
9. SPI_SELECT(_dev, _device, false) - CS拉高
   ↓
10. SPI_LOCK(false) - 释放总线
```

## 8. 关键的抽象层次

### 8.1 硬件抽象层
- **文件位置**: `platforms/nuttx/src/px4/{arch}/{chip}/include/px4_arch/spi_hw_description.h`
- **功能**: 平台相关的GPIO配置和硬件特性
- **不同平台实现**: STM32, iMXRT, S32K, Kinetis等

```cpp
// STM32平台的实现示例
static inline constexpr px4_spi_bus_device_t initSPIDevice(uint32_t devid, SPI::CS cs_gpio, SPI::DRDY drdy_gpio = {})
{
    px4_spi_bus_device_t ret{};
    ret.cs_gpio = getGPIOPort(cs_gpio.port) | getGPIOPin(cs_gpio.pin) |
                  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET);

    if (drdy_gpio.port != GPIO::PortInvalid) {
        ret.drdy_gpio = getGPIOPort(drdy_gpio.port) | getGPIOPin(drdy_gpio.pin) |
                        (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI);
    }

    ret.devid = PX4_SPIDEV_ID(PX4_SPI_DEVICE_ID, devid);
    ret.devtype_driver = PX4_SPI_DEV_ID(devid);
    return ret;
}
```

### 8.2 配置抽象层
- **文件位置**: `boards/{vendor}/{board}/src/spi.cpp`
- **功能**: 板级硬件配置，编译时确定
- **特点**: 运行时不可更改，支持多硬件版本

### 8.3 驱动抽象层
- **文件位置**: `src/lib/drivers/device/{platform}/SPI.{hpp,cpp}`
- **功能**: 统一的SPI设备接口
- **跨平台支持**: NuttX, POSIX, QURT

### 8.4 总线管理层
- **全局配置**: `px4_spi_buses[]` 全局总线配置数组
- **设备枚举**: `SpiDeviceIterator` 设备枚举和查找
- **总线锁管理**: 防止多个驱动同时访问同一总线

## 9. 设备枚举和查找机制

### 9.1 SpiDeviceIterator类

```cpp
class SpiDeviceIterator
{
public:
    SpiDeviceIterator(FilterType filter, uint16_t devid_driver_index,
                     int16_t chipselect = -1, int bus = -1);

    bool next();  // 枚举下一个设备

    const px4_spi_bus_t &bus() const;
    spi_drdy_gpio_t DRDYGPIO() const;
    uint32_t devid() const;
    int externalBusIndex() const;
    bool external() const;

private:
    const FilterType _filter;
    const uint16_t _devid_driver_index;
    const int16_t _chipselect;
    const int _bus;
    int _index{0};
    int _external_bus_counter{1};
    int _bus_device_index{-1};
};
```

### 9.2 设备查找示例

```cpp
// 查找所有ICM20689设备
for (SpiDeviceIterator it(SpiDeviceIterator::FilterType::InternalBus,
                         DRV_IMU_DEVTYPE_ICM20689); it.next();) {

    const px4_spi_bus_t &bus = it.bus();
    uint32_t devid = it.devid();
    spi_drdy_gpio_t drdy_gpio = it.DRDYGPIO();

    // 创建设备实例
    ICM20689 *dev = new ICM20689(bus.bus, devid, it.external());

    if (dev && dev->init() == OK) {
        // 设备初始化成功
    }
}
```

## 10. 电源管理

### 10.1 传感器电源控制

```cpp
// 电源控制GPIO配置
void board_control_spi_sensors_power_configgpio()
{
    for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
        if (px4_spi_buses[bus].power_enable_gpio != 0) {
            px4_arch_configgpio(px4_spi_buses[bus].power_enable_gpio);
        }
    }
}

// 电源开关控制
void board_control_spi_sensors_power(bool enable_power, int bus_mask)
{
    for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
        if (px4_spi_buses[bus].power_enable_gpio == 0) {
            continue;
        }

        const bool bus_requested = bus_mask & (1 << (px4_spi_buses[bus].bus - 1));

        if (bus_requested) {
            px4_arch_gpiowrite(px4_spi_buses[bus].power_enable_gpio, enable_power ? 1 : 0);
        }
    }
}
```

### 10.2 总线复位功能

```cpp
void board_spi_reset(int ms, int bus_mask)
{
    // 1. 关闭传感器电源
    board_control_spi_sensors_power(false, bus_mask);

    // 2. 将CS和DRDY引脚配置为输入下拉
    for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
        for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
            if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
                px4_arch_configgpio(_PIN_OFF(px4_spi_buses[bus].devices[i].cs_gpio));
            }
        }
    }

    // 3. 等待指定时间
    usleep(ms * 1000);

    // 4. 重新打开电源
    board_control_spi_sensors_power(true, bus_mask);

    // 5. 重新配置SPI引脚
    for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
        for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i) {
            if (px4_spi_buses[bus].devices[i].cs_gpio != 0) {
                px4_arch_configgpio(px4_spi_buses[bus].devices[i].cs_gpio);
            }
        }
    }
}
```

## 11. 外部总线支持

### 11.1 外部SPI总线配置

```cpp
// 外部总线配置示例
initSPIBusExternal(SPI::Bus::SPI5, {
    initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin4}),
    initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin10}),
    initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin13}),
}),
```

### 11.2 外部设备检测

```cpp
bool px4_spi_bus_external(const px4_spi_bus_t &bus)
{
    // 检查是否为外部总线
    return bus.is_external;
}

// 外部设备自动检测
for (SpiDeviceIterator it(SpiDeviceIterator::FilterType::ExternalBus,
                         DRV_IMU_DEVTYPE_ICM20689); it.next();) {

    if (it.external()) {
        // 这是一个外部设备
        int external_bus_index = it.externalBusIndex();
        // 创建外部设备实例...
    }
}
```

## 12. 设计优势总结

### 12.1 架构优势

1. **分层清晰**: 硬件配置与驱动逻辑完全分离
   - Board层负责硬件描述
   - Platform层负责硬件抽象
   - Driver层负责设备逻辑

2. **平台无关**: 同一驱动可在不同平台运行
   - NuttX (嵌入式实时系统)
   - POSIX (Linux仿真)
   - QURT (高通DSP)

3. **配置灵活**: 支持复杂的硬件配置需求
   - 多硬件版本支持
   - 外部总线自动检测
   - 运行时设备枚举

4. **类型安全**: 编译时检查配置正确性
   - constexpr配置验证
   - 强类型GPIO定义
   - 设备ID类型检查

5. **资源管理**: 统一的电源和总线锁管理
   - 自动电源控制
   - 总线访问仲裁
   - 设备复位功能

### 12.2 扩展性

1. **新平台移植**: 只需实现平台相关的硬件抽象层
2. **新板子支持**: 只需添加board配置文件
3. **新设备支持**: 继承SPI基类即可
4. **新功能添加**: 在相应层次添加功能

### 12.3 维护性

1. **配置集中**: 所有硬件配置在一个文件中
2. **代码复用**: 平台相关代码最小化
3. **调试友好**: 清晰的层次便于问题定位
4. **文档完整**: 每层都有明确的接口定义

## 13. 常见问题和调试

### 13.1 CS引脚选择约束

**问题**: CUAV X7的SPI CS引脚是否可以随意选择？

**答案**: 不是完全随意的，需要考虑：

1. **GPIO功能限制**: CS引脚必须是可配置为GPIO输出的引脚
2. **电气特性**: 需要满足SPI时序要求
3. **引脚复用冲突**: 不能与其他必需功能冲突
4. **PCB布线考虑**: 需要考虑信号完整性
5. **驱动能力**: 引脚需要有足够的驱动能力

### 13.2 调试方法

1. **配置验证**: 使用`validateSPIConfig()`编译时检查
2. **设备枚举**: 使用`listener spi_device_iterator`查看检测到的设备
3. **GPIO状态**: 使用示波器或逻辑分析仪检查CS时序
4. **总线状态**: 检查`px4_spi_buses`数组内容
5. **电源状态**: 确认传感器电源正常

### 13.3 性能优化

1. **总线锁优化**: 合理设置`requires_locking`
2. **频率设置**: 根据设备特性设置合适的SPI频率
3. **DMA使用**: 在支持的平台上启用DMA传输
4. **中断优化**: 合理使用DRDY中断减少轮询

## 14. 结论

PX4的SPI驱动架构是一个设计精良的分层系统，它成功地将硬件抽象、配置管理、设备驱动和应用逻辑分离开来。这种设计使得PX4能够：

- **支持众多不同的硬件平台**
- **保持驱动代码的简洁和可维护性**
- **提供灵活的配置和扩展能力**
- **确保系统的稳定性和性能**

每一层都有明确的职责，从最底层的寄存器操作到最上层的传感器数据处理，形成了一个完整而优雅的SPI驱动框架。这个架构不仅解决了当前的需求，也为未来的扩展和维护奠定了坚实的基础。
```
