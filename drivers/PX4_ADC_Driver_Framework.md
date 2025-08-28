# PX4 ADC驱动框架详解

## 概述

本文档详细分析PX4中ADC（模数转换器）驱动的完整架构，从硬件抽象层到应用层的实现机制。ADC驱动负责采集模拟信号并转换为数字值，主要用于电池电压监测、电流检测、硬件版本识别等功能。

## 🏗️ 架构层次

PX4的ADC驱动采用分层架构设计，从底层到顶层包括：

```
┌─────────────────────────────────────────┐
│           应用层 (Application)           │
│  - 电池监测模块                          │
│  - 系统电源管理                          │
│  - 硬件版本检测                          │
└─────────────────────────────────────────┘
                    ↕ uORB Topics
┌─────────────────────────────────────────┐
│         PX4 ADC驱动层 (ADC Driver)       │
│  - ADC类 (board_adc/ADC.cpp)           │
│  - 外部ADC驱动 (ads1115)                │
└─────────────────────────────────────────┘
                    ↕ px4_arch_adc_*
┌─────────────────────────────────────────┐
│      硬件抽象层 (Hardware Abstraction)   │
│  - STM32 ADC实现                        │
│  - NXP i.MX RT ADC实现                  │
│  - 其他平台特定实现                      │
└─────────────────────────────────────────┘
                    ↕ 寄存器操作
┌─────────────────────────────────────────┐
│           硬件层 (Hardware)              │
│  - ADC控制器                            │
│  - 模拟多路复用器                        │
│  - 参考电压源                            │
└─────────────────────────────────────────┘
```

## 📁 核心文件结构

### 主要驱动文件
```
src/drivers/adc/
├── board_adc/              # 板载ADC驱动
│   ├── ADC.hpp            # ADC类声明
│   ├── ADC.cpp            # ADC类实现
│   └── CMakeLists.txt     # 构建配置
├── ads1115/               # 外部I2C ADC芯片驱动
│   ├── ADS1115.h          # ADS1115类声明
│   ├── ADS1115.cpp        # ADS1115类实现
│   └── ads1115_main.cpp   # 主程序入口
└── drv_adc.h              # ADC驱动接口定义
```

### 平台特定实现
```
platforms/nuttx/src/px4/
├── stm/stm32h7/adc/adc.cpp      # STM32H7 ADC实现
├── stm/stm32_common/adc/adc.cpp # STM32通用ADC实现
├── nxp/imxrt/adc/adc.cpp        # i.MX RT ADC实现
├── nxp/kinetis/adc/adc.cpp      # Kinetis ADC实现
└── nxp/s32k3xx/adc/adc.cpp      # S32K3XX ADC实现
```

### uORB消息定义
```
msg/
├── AdcReport.msg          # ADC采样数据报告
└── SystemPower.msg        # 系统电源状态
```

## 🔧 硬件抽象层接口

### 核心API函数

PX4定义了统一的ADC硬件抽象层接口，所有平台都必须实现这些函数：

```c
// 初始化ADC硬件
int px4_arch_adc_init(uint32_t base_address);

// 反初始化ADC硬件
void px4_arch_adc_uninit(uint32_t base_address);

// 采样指定通道
uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel);

// 获取参考电压
float px4_arch_adc_reference_v();

// 获取温度传感器通道掩码
uint32_t px4_arch_adc_temp_sensor_mask();

// 获取ADC满量程计数值
uint32_t px4_arch_adc_dn_fullcount();
```

### 平台特定实现示例

#### STM32H7平台实现
```cpp
uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
    irqstate_t flags = px4_enter_critical_section();

    // 清除之前的转换完成标志
    if (rISR(base_address) & ADC_INT_EOC) {
        rISR(base_address) &= ~ADC_INT_EOC;
    }

    // 配置采样通道
    rSQR1(base_address) = (channel & 0x1f);

    // 启动转换
    rCR(base_address) |= ADC_CR_ADSTART;

    // 等待转换完成
    hrt_abstime now = hrt_absolute_time();
    while (!(rISR(base_address) & ADC_INT_EOC)) {
        if ((hrt_absolute_time() - now) > 50) {
            px4_leave_critical_section(flags);
            return UINT32_MAX;
        }
    }

    // 读取转换结果
    uint32_t result = rDR(base_address);
    px4_leave_critical_section(flags);

    return result;
}
```

#### i.MX RT平台实现
```cpp
uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
    irqstate_t flags = px4_enter_critical_section();

    // 清除之前的转换完成标志
    uint16_t result = rR0(base_address);

    // 设置转换通道
    rHC0(base_address) = channel;

    // 等待转换完成
    hrt_abstime now = hrt_absolute_time();
    while (!(rHS(base_address) & ADC_HS_COCO0)) {
        if ((hrt_absolute_time() - now) > 30) {
            px4_leave_critical_section(flags);
            return UINT32_MAX;
        }
    }

    // 读取转换结果
    result = rR0(base_address);
    px4_leave_critical_section(flags);

    return result;
}
```

## 🎯 ADC驱动类实现

### ADC类结构

主ADC驱动类继承自`ModuleBase`和`ScheduledWorkItem`，实现定时采样功能：

```cpp
class ADC : public ModuleBase<ADC>, public px4::ScheduledWorkItem
{
public:
    ADC(uint32_t base_address = SYSTEM_ADC_BASE,
        uint32_t channels = ADC_CHANNELS,
        bool publish_adc_report = true);

    ~ADC() override;

    int init();
    int test();

private:
    void Run() override;                    // 定时执行函数
    uint32_t sample(unsigned channel);     // 采样单个通道
    void update_adc_report(hrt_abstime now);    // 更新ADC报告
    void update_system_power(hrt_abstime now);  // 更新系统电源状态

    // 成员变量
    static const hrt_abstime kINTERVAL{10_ms};  // 100Hz采样率
    const bool _publish_adc_report;
    perf_counter_t _sample_perf;
    unsigned _channel_count{0};
    const uint32_t _base_address;
    px4_adc_msg_t *_samples{nullptr};

    // uORB发布者
    uORB::Publication<adc_report_s> _to_adc_report{ORB_ID(adc_report)};
    uORB::Publication<system_power_s> _to_system_power{ORB_ID(system_power)};
};
```

### 初始化流程

```cpp
int ADC::init()
{
    // 1. 初始化硬件ADC
    int ret_init = px4_arch_adc_init(_base_address);
    if (ret_init < 0) {
        PX4_ERR("arch adc init failed");
        return ret_init;
    }

    // 2. 启动定时采样
    ScheduleOnInterval(kINTERVAL, kINTERVAL);

    return PX4_OK;
}
```

### 采样执行流程

```cpp
void ADC::Run()
{
    if (_first_run) {
        open_gpio_devices();  // 打开GPIO设备
        _first_run = false;
    }

    hrt_abstime now = hrt_absolute_time();

    // 扫描所有配置的通道并采样
    for (unsigned i = 0; i < _channel_count; i++) {
        _samples[i].am_data = sample(_samples[i].am_channel);
    }

    // 发布ADC报告
    if (_publish_adc_report) {
        update_adc_report(now);
    }

    // 更新系统电源状态
    update_system_power(now);
}
```

## 📊 板级配置

### 通道配置示例

不同飞控板的ADC通道配置在`board_config.h`中定义：

#### CUAV X7 Pro配置
```c
#define ADC_BATTERY1_VOLTAGE_CHANNEL        2   /* PA2  */
#define ADC_BATTERY1_CURRENT_CHANNEL        3   /* PA3  */
#define ADC_BATTERY2_VOLTAGE_CHANNEL        4   /* PA4  */
#define ADC_BATTERY2_CURRENT_CHANNEL        0   /* PA0  */
#define ADC1_6V6_IN_CHANNEL                 6   /* PA6  */
#define ADC1_3V3_IN_CHANNEL                 7   /* PA7  */
#define ADC_RSSI_IN_CHANNEL                 15  /* PC5  */
#define ADC_SCALED_V5_CHANNEL               10  /* PC0  */
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL  11  /* PC1  */
#define ADC_HW_VER_SENSE_CHANNEL            12  /* PC2  */
#define ADC_HW_REV_SENSE_CHANNEL            13  /* PC3  */

#define ADC_CHANNELS \
    ((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
     (1 << ADC_BATTERY1_CURRENT_CHANNEL)       | \
     (1 << ADC_BATTERY2_VOLTAGE_CHANNEL)       | \
     (1 << ADC_BATTERY2_CURRENT_CHANNEL)       | \
     (1 << ADC1_6V6_IN_CHANNEL)                | \
     (1 << ADC1_3V3_IN_CHANNEL)                | \
     (1 << ADC_RSSI_IN_CHANNEL)                | \
     (1 << ADC_SCALED_V5_CHANNEL)              | \
     (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
     (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
     (1 << ADC_HW_REV_SENSE_CHANNEL))
```

#### Holybro Kakute H7配置
```c
#define ADC_BATTERY_VOLTAGE_CHANNEL        10  /* PC0 */
#define ADC_BATTERY_CURRENT_CHANNEL        11  /* PC1 */
#define ADC_RSSI_IN_CHANNEL                8   /* PC5 */

#define ADC_CHANNELS \
    ((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
     (1 << ADC_BATTERY_CURRENT_CHANNEL) | \
     (1 << ADC_RSSI_IN_CHANNEL))
```

### GPIO配置

ADC通道对应的GPIO引脚配置：

```c
#define PX4_ADC_GPIO  \
    /* PA2 */  GPIO_ADC123_INP14, \
    /* PA3 */  GPIO_ADC12_INP15,  \
    /* PA4 */  GPIO_ADC12_INP18,  \
    /* PA0 */  GPIO_ADC123_INP16, \
    /* PA6 */  GPIO_ADC12_INP3,   \
    /* PA7 */  GPIO_ADC12_INP7,   \
    /* PC5 */  GPIO_ADC12_INP8,   \
    /* PC0 */  GPIO_ADC123_INP10, \
    /* PC1 */  GPIO_ADC123_INP11, \
    /* PC2 */  GPIO_ADC123_INP12, \
    /* PC3 */  GPIO_ADC123_INP13
```

## 📡 uORB消息接口

### AdcReport消息

ADC采样数据通过`AdcReport`消息发布：

```c
uint64 timestamp        # 时间戳 (微秒)
uint32 device_id        # 设备ID，重启后保持不变
int16[12] channel_id    # ADC通道ID，负值表示不存在
int32[12] raw_data      # ADC通道原始值，支持负值
uint32 resolution       # ADC分辨率
float32 v_ref           # ADC参考电压，用于计算LSB电压
```

### SystemPower消息

系统电源状态通过`SystemPower`消息发布：

```c
uint64 timestamp            # 时间戳 (微秒)
float32 voltage5v_v         # 外设5V电压
float32 voltage_payload_v   # 载荷电压
float32[4] sensors3v3       # 传感器3.3V电压
uint8 sensors3v3_valid      # 传感器3.3V电压有效性 (位域)
uint8 usb_connected         # USB连接状态
uint8 brick_valid           # 电源模块有效性 (位域)
uint8 usb_valid             # USB电源有效性
uint8 servo_valid           # 舵机电源有效性
uint8 periph_5v_oc          # 外设5V过流状态
uint8 hipower_5v_oc         # 大功率外设5V过流状态
uint8 comp_5v_valid         # 伴随计算机5V有效性
uint8 can1_gps1_5v_valid    # CAN1/GPS1 5V有效性
uint8 payload_v_valid       # 载荷电压有效性
```

## 🔌 外部ADC驱动 (ADS1115)

### ADS1115驱动架构

对于没有内置ADC的飞控板，PX4支持外部I2C ADC芯片ADS1115：

```cpp
class ADS1115 : public I2C, public I2CSPIDriver<ADS1115>, public px4::ScheduledWorkItem
{
public:
    ADS1115(const I2CSPIDriverConfig &config);
    ~ADS1115() override;

    int init() override;
    void RunImpl();

private:
    enum Channel {
        A0 = 0,
        A1 = 1,
        A2 = 2,
        A3 = 3
    };

    int setChannel(Channel channel);
    int readChannel();

    // 配置寄存器定义
    static constexpr uint8_t CONFIG_HIGH_OS_NOACT = 0x80;
    static constexpr uint8_t CONFIG_HIGH_MUX_P0NG = 0x40;
    static constexpr uint8_t CONFIG_HIGH_PGA_6144 = 0x00;
    // ... 更多配置常量

    adc_report_s _adc_report{};
    perf_counter_t _cycle_perf;
    uORB::Publication<adc_report_s> _to_adc_report{ORB_ID(adc_report)};
};
```

### ADS1115初始化配置

```cpp
int ADS1115::init()
{
    int ret = I2C::init();
    if (ret != PX4_OK) {
        return ret;
    }

    // 配置ADS1115寄存器
    uint8_t config[2] = {};
    config[0] = CONFIG_HIGH_OS_NOACT | CONFIG_HIGH_MUX_P0NG |
                CONFIG_HIGH_PGA_6144 | CONFIG_HIGH_MODE_SS;
    config[1] = CONFIG_LOW_DR_250SPS | CONFIG_LOW_COMP_MODE_TRADITIONAL |
                CONFIG_LOW_COMP_POL_RESET | CONFIG_LOW_COMP_LAT_NONE |
                CONFIG_LOW_COMP_QU_DISABLE;

    ret = writeReg(ADDRESSPOINTER_REG_CONFIG, config, 2);
    if (ret != PX4_OK) {
        PX4_ERR("writeReg failed (%i)", ret);
        return ret;
    }

    setChannel(ADS1115::A0);  // 准备第一次测量
    ScheduleOnInterval(SAMPLE_INTERVAL / 4, SAMPLE_INTERVAL / 4);

    return PX4_OK;
}
```

## ⚡ 电源管理集成

### 电池电压监测

ADC驱动与电池监测系统紧密集成：

```cpp
void ADC::update_system_power(hrt_abstime now)
{
    system_power_s system_power{};
    system_power.timestamp = now;

    // 计算5V电压
    if (_samples[ADC_SCALED_V5_CHANNEL].am_data != UINT32_MAX) {
        float voltage = _samples[ADC_SCALED_V5_CHANNEL].am_data *
                       (ADC_V5_V_FULL_SCALE / px4_arch_adc_dn_fullcount());
        system_power.voltage5v_v = voltage;
    }

    // 计算传感器3.3V电压
    if (_samples[ADC_SCALED_VDD_3V3_SENSORS_CHANNEL].am_data != UINT32_MAX) {
        float voltage = _samples[ADC_SCALED_VDD_3V3_SENSORS_CHANNEL].am_data *
                       (ADC_3V3_SCALE / px4_arch_adc_dn_fullcount());
        system_power.sensors3v3[0] = voltage;
        system_power.sensors3v3_valid |= 1;
    }

    // 读取GPIO状态
    system_power.comp_5v_valid = read_gpio_value(_5v_comp_valid_fd);
    system_power.can1_gps1_5v_valid = read_gpio_value(_5v_can1_gps1_valid_fd);

    _to_system_power.publish(system_power);
}
```

### 硬件版本检测

通过ADC读取硬件版本和修订版本：

```cpp
// 硬件版本检测
uint32_t hw_ver_raw = sample(ADC_HW_VER_SENSE_CHANNEL);
uint32_t hw_rev_raw = sample(ADC_HW_REV_SENSE_CHANNEL);

if (hw_ver_raw != UINT32_MAX && hw_rev_raw != UINT32_MAX) {
    float hw_ver_v = hw_ver_raw * px4_arch_adc_reference_v() / px4_arch_adc_dn_fullcount();
    float hw_rev_v = hw_rev_raw * px4_arch_adc_reference_v() / px4_arch_adc_dn_fullcount();

    // 根据电压值确定硬件版本
    if (hw_ver_v > 2.8f) {
        board_hw_version = 3;
    } else if (hw_ver_v > 2.2f) {
        board_hw_version = 2;
    } else {
        board_hw_version = 1;
    }
}
```

## 🛠️ 调试和测试

### ADC测试命令

PX4提供了ADC测试命令用于调试：

```bash
# 启动ADC驱动
board_adc start

# 测试ADC功能
board_adc test

# 查看ADC状态
board_adc status

# 停止ADC驱动
board_adc stop
```

### 测试输出示例

```
ADC test
ADC channels: 11
Channel 0: 2048 (1.65V)
Channel 2: 3276 (2.64V)  # 电池电压
Channel 3: 1638 (1.32V)  # 电池电流
Channel 4: 2457 (1.98V)  # 第二电池电压
Channel 6: 2048 (1.65V)  # 6.6V监测
Channel 7: 1365 (1.10V)  # 3.3V监测
Channel 10: 2867 (2.31V) # 5V监测
Channel 11: 1638 (1.32V) # 传感器3.3V
Channel 12: 2048 (1.65V) # 硬件版本
Channel 13: 1365 (1.10V) # 硬件修订
Channel 15: 0 (0.00V)    # RSSI输入
```

### 性能监控

ADC驱动包含性能计数器用于监控采样性能：

```cpp
// 创建性能计数器
_sample_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": sample");

// 在采样时测量性能
uint32_t ADC::sample(unsigned channel)
{
    perf_begin(_sample_perf);
    uint32_t result = px4_arch_adc_sample(_base_address, channel);
    perf_end(_sample_perf);
    return result;
}
```

## 🔧 常见问题和解决方案

### 1. ADC读数不稳定

**问题**: ADC读数波动较大，数值不稳定

**原因分析**:
- 参考电压不稳定
- 采样时间不足
- 电磁干扰
- 硬件滤波不足

**解决方案**:
```cpp
// 增加采样时间
rSMPR1(base_address) = 0b00000111111111111111111111111111;  // 最长采样时间

// 多次采样取平均
uint32_t total = 0;
for (int i = 0; i < 8; i++) {
    total += px4_arch_adc_sample(base_address, channel);
    px4_usleep(100);
}
return total / 8;
```

### 2. 温度传感器读数异常

**问题**: 内部温度传感器读数异常或无法读取

**原因分析**:
- 温度传感器未正确使能
- 通道映射错误
- 参考电压配置错误

**解决方案**:
```cpp
// 确保温度传感器使能
#ifdef ADC_CR2_TSVREFE
rCR2(base_address) |= ADC_CR2_TSVREFE;
#endif

// 正确的通道映射
if (channel == PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL) {
    channel = ADC3_INTERNAL_TEMP_SENSOR_CHANNEL;
    base_address = STM32_ADC3_BASE;
}
```

### 3. 外部ADC通信失败

**问题**: ADS1115等外部ADC芯片通信失败

**原因分析**:
- I2C地址配置错误
- I2C时钟频率过高
- 电源供电不稳定

**解决方案**:
```cpp
// 检查I2C地址
cli.i2c_address = 0x48;  // ADS1115默认地址

// 降低I2C频率
cli.default_i2c_frequency = 100000;  // 100kHz

// 增加重试机制
int retry_count = 3;
while (retry_count-- > 0) {
    ret = writeReg(ADDRESSPOINTER_REG_CONFIG, config, 2);
    if (ret == PX4_OK) break;
    px4_usleep(1000);
}
```

## 📚 参考资料

### 相关文档
- [PX4 SPI驱动架构详解](PX4_SPI_Driver_Architecture.md)
- [PX4 UART驱动架构详解](PX4_Serial_Driver_Framework.md)
- [PX4参数系统文档](PX4_Parameter_System_Documentation.md)

### 源码文件
- `src/drivers/adc/board_adc/ADC.cpp` - 主ADC驱动实现
- `src/drivers/adc/ads1115/ADS1115.cpp` - 外部ADC驱动实现
- `platforms/nuttx/src/px4/stm/stm32h7/adc/adc.cpp` - STM32H7平台实现
- `msg/AdcReport.msg` - ADC报告消息定义
- `msg/SystemPower.msg` - 系统电源消息定义

### 硬件参考
- STM32H7系列参考手册 - ADC章节
- i.MX RT系列参考手册 - LPADC章节
- ADS1115数据手册 - 16位I2C ADC

---

> 💡 **提示**: ADC驱动是PX4电源管理和硬件监测的基础，理解其架构对于飞控系统的可靠性至关重要。建议结合实际硬件进行测试和调试。
