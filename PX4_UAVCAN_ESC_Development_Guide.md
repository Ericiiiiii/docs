# PX4 UAVCAN 电调开发完整指南

## 概述

本指南详细介绍如何从零开始开发一个支持UAVCAN协议的电调(ESC)，并将其集成到PX4生态系统中。

## 目录

1. [硬件设计](#硬件设计)
2. [软件架构](#软件架构)
3. [UAVCAN实现](#uavcan实现)
4. [PX4集成](#px4集成)
5. [测试验证](#测试验证)
6. [高级功能](#高级功能)
7. [生产部署](#生产部署)

## 硬件设计

### 系统架构

```
电调硬件架构:
┌─────────────────┐    ┌──────────────┐    ┌─────────────┐
│   微控制器      │────│  CAN收发器   │────│   CAN总线   │
│  (STM32F4/F7)   │    │  (MCP2515)   │    │             │
└─────────────────┘    └──────────────┘    └─────────────┘
         │
         ├── PWM输出 ──→ 功率级 ──→ 无刷电机
         ├── 电流检测 ←── 电流传感器 (ACS712)
         ├── 电压检测 ←── 分压电路
         ├── 温度检测 ←── 温度传感器 (NTC)
         └── 转速检测 ←── 反电动势检测
```

### 核心组件选择

#### 微控制器
- **推荐**: STM32F405/F407 (168MHz, 内置CAN)
- **备选**: STM32F303 (72MHz, 内置CAN)
- **低成本**: ESP32 (240MHz, 需外接CAN收发器)

**选择标准**:
- 内置CAN控制器 (推荐)
- 足够的PWM通道 (至少3路)
- ADC通道 (电压、电流、温度检测)
- 充足的Flash和RAM

#### CAN收发器
- **内置CAN**: 使用外部收发器如TJA1050
- **外接CAN**: MCP2515 + MCP2551组合
- **工业级**: ISO1050 (隔离型)

#### 功率级设计
```
三相桥式电路:
     VCC
      │
   ┌──┴──┐
   │ Q1  │ Q3  │ Q5
   └──┬──┘
      │     A相  B相  C相
   ┌──┴──┐
   │ Q2  │ Q4  │ Q6
   └──┬──┘
      │
     GND
```

**MOSFET选择**:
- 低导通电阻 (Rds_on < 10mΩ)
- 快速开关速度
- 足够的电流容量
- 合适的电压等级

#### 传感器电路

**电流检测**:
```cpp
// 使用霍尔电流传感器
float readCurrent() {
    uint16_t adc_value = ADC_Read(CURRENT_CHANNEL);
    float voltage = (adc_value * 3.3f) / 4096.0f;
    float current = (voltage - 2.5f) / 0.1f;  // ACS712-20A: 100mV/A
    return current;
}
```

**电压检测**:
```cpp
// 分压电路检测
float readVoltage() {
    uint16_t adc_value = ADC_Read(VOLTAGE_CHANNEL);
    float voltage = (adc_value * 3.3f) / 4096.0f;
    float battery_voltage = voltage * (R1 + R2) / R2;  // 分压比
    return battery_voltage;
}
```

**温度检测**:
```cpp
// NTC热敏电阻
float readTemperature() {
    uint16_t adc_value = ADC_Read(TEMP_CHANNEL);
    float resistance = calculateNTCResistance(adc_value);
    float temperature = calculateTemperature(resistance);
    return temperature;
}
```

### PCB设计要点

#### 布局原则
1. **功率部分与控制部分分离**
2. **CAN信号使用差分布线**
3. **电源去耦充分**
4. **散热设计充分**

#### 关键信号
- CAN_H/CAN_L: 差分阻抗120Ω
- PWM信号: 短而粗的走线
- 电源: 足够宽的铜皮
- 模拟信号: 远离开关噪声

## 软件架构

### 整体架构

```
软件架构层次:
┌─────────────────┐
│   应用层        │ ← UAVCAN协议处理
├─────────────────┤
│   中间件层      │ ← 电机控制算法
├─────────────────┤
│   驱动层        │ ← 硬件抽象层
├─────────────────┤
│   硬件层        │ ← 寄存器操作
└─────────────────┘
```

### 任务调度

```cpp
// 主要任务及其频率
void taskScheduler() {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();

    // 高频任务 (20kHz) - 电机控制
    if (current_time - last_motor_update >= 0.05) {
        updateMotorControl();
        last_motor_update = current_time;
    }

    // 中频任务 (1kHz) - UAVCAN处理
    if (current_time - last_uavcan_update >= 1) {
        processUAVCAN();
        last_uavcan_update = current_time;
    }

    // 低频任务 (10Hz) - 状态发布
    if (current_time - last_status_update >= 100) {
        publishStatus();
        last_status_update = current_time;
    }
}
```

### 状态机设计

```cpp
enum ESCState {
    ESC_STATE_INIT,
    ESC_STATE_IDLE,
    ESC_STATE_RUNNING,
    ESC_STATE_ERROR,
    ESC_STATE_CALIBRATION
};

class ESCStateMachine {
private:
    ESCState current_state;

public:
    void update() {
        switch (current_state) {
            case ESC_STATE_INIT:
                handleInitState();
                break;
            case ESC_STATE_IDLE:
                handleIdleState();
                break;
            case ESC_STATE_RUNNING:
                handleRunningState();
                break;
            case ESC_STATE_ERROR:
                handleErrorState();
                break;
            case ESC_STATE_CALIBRATION:
                handleCalibrationState();
                break;
        }
    }

private:
    void handleInitState() {
        // 初始化硬件
        if (initializeHardware()) {
            current_state = ESC_STATE_IDLE;
        } else {
            current_state = ESC_STATE_ERROR;
        }
    }

    void handleIdleState() {
        // 等待命令
        if (hasValidCommand()) {
            current_state = ESC_STATE_RUNNING;
        }
    }

    void handleRunningState() {
        // 执行电机控制
        if (hasError()) {
            current_state = ESC_STATE_ERROR;
        } else if (!hasValidCommand()) {
            current_state = ESC_STATE_IDLE;
        }
    }

    void handleErrorState() {
        // 错误处理
        stopMotor();
        if (errorCleared()) {
            current_state = ESC_STATE_IDLE;
        }
    }
};
```

## UAVCAN实现

### 基础节点实现

```cpp
#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/esc/RawCommand.hpp>
#include <uavcan/equipment/esc/Status.hpp>
#include <uavcan/protocol/param/GetSet.hpp>

class UavcanESCNode {
private:
    // UAVCAN节点
    uavcan::Node<> node;

    // 消息订阅器
    uavcan::Subscriber<uavcan::equipment::esc::RawCommand> cmd_subscriber;

    // 消息发布器
    uavcan::Publisher<uavcan::equipment::esc::Status> status_publisher;

    // 服务服务器
    uavcan::ServiceServer<uavcan::protocol::param::GetSet> param_server;

    // ESC参数
    struct {
        uint8_t esc_index;
        uint8_t node_id;
        float max_current;
        float max_temperature;
        bool reverse_direction;
    } config;

    // 运行时数据
    struct {
        float voltage;
        float current;
        float temperature;
        int32_t rpm;
        uint32_t error_count;
        uavcan::MonotonicTime last_command_time;
    } runtime;

public:
    UavcanESCNode(uint8_t node_id, uint8_t esc_index)
        : node(getCanDriver(), getSystemClock())
        , cmd_subscriber(node)
        , status_publisher(node)
        , param_server(node)
    {
        config.node_id = node_id;
        config.esc_index = esc_index;
        config.max_current = 30.0f;      // 30A
        config.max_temperature = 80.0f;  // 80°C
        config.reverse_direction = false;
    }

    int initialize() {
        // 设置节点信息
        node.setNodeID(config.node_id);
        node.setName("CustomESC_v1.0");

        // 设置版本信息
        uavcan::protocol::SoftwareVersion sw_version;
        sw_version.major = 1;
        sw_version.minor = 0;
        sw_version.optional_field_flags = 0;
        sw_version.vcs_commit = 0x12345678;
        node.setSoftwareVersion(sw_version);

        uavcan::protocol::HardwareVersion hw_version;
        hw_version.major = 1;
        hw_version.minor = 0;
        // 设置唯一ID (通常使用MCU的唯一ID)
        fillUniqueID(hw_version.unique_id);
        node.setHardwareVersion(hw_version);

        // 初始化发布器
        int res = status_publisher.init();
        if (res < 0) return res;

        // 初始化订阅器
        res = cmd_subscriber.start(
            [this](const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RawCommand>& msg) {
                handleRawCommand(msg);
            });
        if (res < 0) return res;

        // 初始化参数服务器
        res = param_server.start(
            [this](const uavcan::protocol::param::GetSet::Request& req,
                   uavcan::protocol::param::GetSet::Response& rsp) {
                handleParameterRequest(req, rsp);
            });
        if (res < 0) return res;

        // 启动节点
        res = node.start();
        if (res < 0) return res;

        return 0;
    }

    void spin() {
        // 处理UAVCAN事件
        node.spin(uavcan::MonotonicDuration::fromMSec(1));

        // 更新传感器数据
        updateSensorData();

        // 检查安全条件
        checkSafetyConditions();

        // 定期发布状态
        publishStatusIfNeeded();
    }

private:
    void handleRawCommand(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::RawCommand>& msg) {
        // 检查命令数组大小
        if (config.esc_index >= msg.cmd.size()) {
            return;
        }

        // 获取命令值
        int16_t command = msg.cmd[config.esc_index];

        // 记录接收时间
        runtime.last_command_time = node.getMonotonicTime();

        // 处理命令
        processMotorCommand(command);
    }

    void processMotorCommand(int16_t command) {
        // 命令值范围: -8192 到 8191
        // 0 表示停止
        // 正值表示正转，负值表示反转

        if (command == 0) {
            stopMotor();
            return;
        }

        bool reverse = (command < 0);
        if (config.reverse_direction) {
            reverse = !reverse;
        }

        uint16_t throttle = abs(command);

        // 转换为PWM占空比 (0-100%)
        float duty_cycle = (float)throttle / 8191.0f * 100.0f;

        // 设置电机PWM
        setMotorPWM(duty_cycle, reverse);
    }

    void updateSensorData() {
        runtime.voltage = readVoltage();
        runtime.current = readCurrent();
        runtime.temperature = readTemperature();
        runtime.rpm = readRPM();
    }

    void checkSafetyConditions() {
        bool error_detected = false;

        // 检查过流
        if (runtime.current > config.max_current) {
            error_detected = true;
            runtime.error_count++;
        }

        // 检查过温
        if (runtime.temperature > config.max_temperature) {
            error_detected = true;
            runtime.error_count++;
        }

        // 检查命令超时
        auto time_since_command = node.getMonotonicTime() - runtime.last_command_time;
        if (time_since_command > uavcan::MonotonicDuration::fromMSec(500)) {
            error_detected = true;
        }

        // 如果检测到错误，停止电机
        if (error_detected) {
            stopMotor();
        }
    }

    void publishStatusIfNeeded() {
        static uavcan::MonotonicTime last_publish_time;

        // 10Hz发布频率
        if (node.getMonotonicTime() - last_publish_time > uavcan::MonotonicDuration::fromMSec(100)) {
            publishStatus();
            last_publish_time = node.getMonotonicTime();
        }
    }

    void publishStatus() {
        uavcan::equipment::esc::Status status_msg;

        status_msg.error_count = runtime.error_count;
        status_msg.voltage = runtime.voltage;
        status_msg.current = runtime.current;
        status_msg.temperature = runtime.temperature + 273.15f;  // 转换为开尔文
        status_msg.rpm = runtime.rpm;
        status_msg.power_rating_pct = calculatePowerRating();
        status_msg.esc_index = config.esc_index;

        status_publisher.broadcast(status_msg);
    }

    uint8_t calculatePowerRating() {
        float power = runtime.voltage * runtime.current;
        float max_power = 1000.0f;  // 假设最大功率1000W
        uint8_t rating = (uint8_t)((power / max_power) * 100.0f);
        return (rating > 100) ? 100 : rating;
    }

    void handleParameterRequest(const uavcan::protocol::param::GetSet::Request& req,
                               uavcan::protocol::param::GetSet::Response& rsp) {
        // 处理参数读写请求
        if (req.name == "esc_index") {
            if (req.value.is(uavcan::protocol::param::Value::Tag::integer_value)) {
                config.esc_index = req.value.to<uavcan::protocol::param::Value::Tag::integer_value>();
            }
            rsp.value.to<uavcan::protocol::param::Value::Tag::integer_value>() = config.esc_index;
        }
        else if (req.name == "max_current") {
            if (req.value.is(uavcan::protocol::param::Value::Tag::real_value)) {
                config.max_current = req.value.to<uavcan::protocol::param::Value::Tag::real_value>();
            }
            rsp.value.to<uavcan::protocol::param::Value::Tag::real_value>() = config.max_current;
        }
        else if (req.name == "reverse_direction") {
            if (req.value.is(uavcan::protocol::param::Value::Tag::boolean_value)) {
                config.reverse_direction = req.value.to<uavcan::protocol::param::Value::Tag::boolean_value>();
            }
            rsp.value.to<uavcan::protocol::param::Value::Tag::boolean_value>() = config.reverse_direction;
        }

        rsp.name = req.name;
    }

    // 硬件接口函数 (需要根据具体硬件实现)
    void setMotorPWM(float duty_cycle, bool reverse);
    void stopMotor();
    float readVoltage();
    float readCurrent();
    float readTemperature();
    int32_t readRPM();
    void fillUniqueID(uavcan::protocol::HardwareVersion::FieldTypes::unique_id& uid);
};
```

### 硬件抽象层实现

```cpp
// hardware_abstraction.cpp
#include "hardware_abstraction.h"

// PWM控制实现
void UavcanESCNode::setMotorPWM(float duty_cycle, bool reverse) {
    // 限制占空比范围
    if (duty_cycle > 100.0f) duty_cycle = 100.0f;
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;

    // 计算PWM值 (假设使用16位定时器)
    uint16_t pwm_value = (uint16_t)(duty_cycle / 100.0f * 65535);

    if (reverse) {
        // 反转逻辑 (根据具体电机驱动实现)
        setThreePhaseReverse(pwm_value);
    } else {
        // 正转逻辑
        setThreePhaseForward(pwm_value);
    }
}

void UavcanESCNode::stopMotor() {
    // 停止所有PWM输出
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);  // A相
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);  // B相
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);  // C相
}

// 传感器读取实现
float UavcanESCNode::readVoltage() {
    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
    float voltage = (adc_value * 3.3f) / 4096.0f;

    // 分压电路: R1=10k, R2=2k, 最大检测电压=19.8V
    float battery_voltage = voltage * (10000 + 2000) / 2000;

    return battery_voltage;
}

float UavcanESCNode::readCurrent() {
    uint32_t adc_value = HAL_ADC_GetValue(&hadc2);
    float voltage = (adc_value * 3.3f) / 4096.0f;

    // ACS712-30A: 2.5V零点, 66mV/A
    float current = (voltage - 2.5f) / 0.066f;

    return (current < 0) ? 0 : current;  // 只检测正向电流
}

float UavcanESCNode::readTemperature() {
    uint32_t adc_value = HAL_ADC_GetValue(&hadc3);
    float voltage = (adc_value * 3.3f) / 4096.0f;

    // NTC热敏电阻计算 (Steinhart-Hart方程)
    float resistance = (3.3f - voltage) * 10000.0f / voltage;  // 10k上拉
    float temperature = calculateNTCTemperature(resistance);

    return temperature;
}

int32_t UavcanESCNode::readRPM() {
    // 基于反电动势的转速检测
    static uint32_t last_zero_cross_time = 0;
    static uint32_t zero_cross_count = 0;

    // 检测反电动势过零点
    if (detectZeroCrossing()) {
        uint32_t current_time = HAL_GetTick();
        uint32_t period = current_time - last_zero_cross_time;

        if (period > 0) {
            // 计算RPM (每个电周期6个过零点)
            float frequency = 1000.0f / (period * 6);  // Hz
            int32_t rpm = (int32_t)(frequency * 60.0f);

            last_zero_cross_time = current_time;
            return rpm;
        }
    }

    return 0;  // 无法检测到转速
}

void UavcanESCNode::fillUniqueID(uavcan::protocol::HardwareVersion::FieldTypes::unique_id& uid) {
    // 使用STM32的唯一ID
    uint32_t* stm32_uid = (uint32_t*)UID_BASE;

    uid[0] = (stm32_uid[0] >> 0) & 0xFF;
    uid[1] = (stm32_uid[0] >> 8) & 0xFF;
    uid[2] = (stm32_uid[0] >> 16) & 0xFF;
    uid[3] = (stm32_uid[0] >> 24) & 0xFF;

    uid[4] = (stm32_uid[1] >> 0) & 0xFF;
    uid[5] = (stm32_uid[1] >> 8) & 0xFF;
    uid[6] = (stm32_uid[1] >> 16) & 0xFF;
    uid[7] = (stm32_uid[1] >> 24) & 0xFF;

    uid[8] = (stm32_uid[2] >> 0) & 0xFF;
    uid[9] = (stm32_uid[2] >> 8) & 0xFF;
    uid[10] = (stm32_uid[2] >> 16) & 0xFF;
    uid[11] = (stm32_uid[2] >> 24) & 0xFF;

    // 填充剩余字节
    for (int i = 12; i < 16; i++) {
        uid[i] = 0;
    }
}
```

## PX4集成

### 配置PX4参数

```bash
# 启用UAVCAN ESC模式
param set UAVCAN_ENABLE 3

# 设置节点ID
param set UAVCAN_NODE_ID 1

# 设置CAN波特率
param set UAVCAN_BITRATE 1000000

# 配置ESC功能映射
param set UAVCAN_EC_FUNC1 101  # 电机1
param set UAVCAN_EC_FUNC2 102  # 电机2
param set UAVCAN_EC_FUNC3 103  # 电机3
param set UAVCAN_EC_FUNC4 104  # 电机4

# 设置ESC输出范围
param set UAVCAN_EC_MIN1 1
param set UAVCAN_EC_MAX1 8191
param set UAVCAN_EC_MIN2 1
param set UAVCAN_EC_MAX2 8191
param set UAVCAN_EC_MIN3 1
param set UAVCAN_EC_MAX3 8191
param set UAVCAN_EC_MIN4 1
param set UAVCAN_EC_MAX4 8191

# 保存参数并重启
param save
reboot
```

### 验证集成

```bash
# 检查UAVCAN状态
uavcan status

# 查看在线节点
uavcan info

# 应该看到你的ESC节点:
# Node ID | Mode | Health | Name
#   20    | OPER | OK     | CustomESC_v1.0
#   21    | OPER | OK     | CustomESC_v1.0
#   22    | OPER | OK     | CustomESC_v1.0
#   23    | OPER | OK     | CustomESC_v1.0

# 监听ESC状态
listener esc_status

# 测试ESC控制
actuator_test -m uavcan -v 1000
```

## 测试验证

### 单元测试

```cpp
// test_esc.cpp
#include "uavcan_esc_node.h"
#include <gtest/gtest.h>

class ESCTest : public ::testing::Test {
protected:
    void SetUp() override {
        esc = new UavcanESCNode(20, 0);
        ASSERT_EQ(0, esc->initialize());
    }

    void TearDown() override {
        delete esc;
    }

    UavcanESCNode* esc;
};

TEST_F(ESCTest, CommandProcessing) {
    // 测试正向命令
    esc->processMotorCommand(4000);
    // 验证PWM输出...

    // 测试反向命令
    esc->processMotorCommand(-4000);
    // 验证PWM输出...

    // 测试停止命令
    esc->processMotorCommand(0);
    // 验证电机停止...
}

TEST_F(ESCTest, SafetyLimits) {
    // 测试过流保护
    esc->simulateOvercurrent();
    esc->checkSafetyConditions();
    // 验证电机已停止...

    // 测试过温保护
    esc->simulateOvertemperature();
    esc->checkSafetyConditions();
    // 验证电机已停止...
}

TEST_F(ESCTest, ParameterHandling) {
    // 测试参数读写
    uavcan::protocol::param::GetSet::Request req;
    uavcan::protocol::param::GetSet::Response rsp;

    req.name = "max_current";
    req.value.to<uavcan::protocol::param::Value::Tag::real_value>() = 25.0f;

    esc->handleParameterRequest(req, rsp);

    EXPECT_EQ(25.0f, rsp.value.to<uavcan::protocol::param::Value::Tag::real_value>());
}
```

### 集成测试

```bash
#!/bin/bash
# integration_test.sh

echo "开始ESC集成测试..."

# 启动PX4仿真
make px4_fmu-v5_default jmavsim &
PX4_PID=$!

sleep 10

# 连接到PX4控制台
px4-console() {
    echo "$1" | nc localhost 4560
}

# 检查UAVCAN状态
echo "检查UAVCAN状态..."
px4-console "uavcan status"

# 检查ESC节点
echo "检查ESC节点..."
px4-console "uavcan info"

# 测试ESC控制
echo "测试ESC控制..."
px4-console "actuator_test -m uavcan -v 2000"
sleep 2
px4-console "actuator_test -m uavcan -v 0"

# 监听ESC状态
echo "监听ESC状态..."
px4-console "listener esc_status" &
sleep 5

# 清理
kill $PX4_PID
echo "集成测试完成"
```

### 硬件在环测试

```cpp
// hardware_in_loop_test.cpp
class HILTest {
public:
    void runFullTest() {
        // 1. 初始化测试
        initializeTest();

        // 2. 基本功能测试
        testBasicOperation();

        // 3. 性能测试
        testPerformance();

        // 4. 安全测试
        testSafetyFeatures();

        // 5. 耐久性测试
        testDurability();

        // 6. 生成测试报告
        generateReport();
    }

private:
    void testBasicOperation() {
        // 测试启动序列
        sendCommand(0);
        verifyMotorStopped();

        // 测试低速运行
        sendCommand(1000);
        verifyMotorRunning(1000);

        // 测试高速运行
        sendCommand(8000);
        verifyMotorRunning(8000);

        // 测试停止
        sendCommand(0);
        verifyMotorStopped();
    }

    void testPerformance() {
        // 测试响应时间
        auto start_time = std::chrono::high_resolution_clock::now();
        sendCommand(4000);
        waitForResponse();
        auto end_time = std::chrono::high_resolution_clock::now();

        auto response_time = std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - start_time).count();

        EXPECT_LT(response_time, 1000);  // 响应时间应小于1ms
    }

    void testSafetyFeatures() {
        // 测试命令超时
        sendCommand(4000);
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
        verifyMotorStopped();  // 应该因超时而停止

        // 测试过流保护
        simulateOvercurrent();
        verifyMotorStopped();

        // 测试过温保护
        simulateOvertemperature();
        verifyMotorStopped();
    }
};
```

## 高级功能

### 固件更新支持

```cpp
// firmware_update.cpp
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>

class ESCFirmwareUpdater {
private:
    uavcan::ServiceServer<uavcan::protocol::file::BeginFirmwareUpdate> fw_server;

public:
    ESCFirmwareUpdater(uavcan::INode& node) : fw_server(node) {}

    int init() {
        return fw_server.start([this](const auto& req, auto& rsp) {
            handleFirmwareUpdate(req, rsp);
        });
    }

private:
    void handleFirmwareUpdate(
        const uavcan::protocol::file::BeginFirmwareUpdate::Request& req,
        uavcan::protocol::file::BeginFirmwareUpdate::Response& rsp) {

        // 验证固件文件路径
        if (req.image_file_remote_path.empty()) {
            rsp.error = rsp.ERROR_INVALID_REQUEST;
            return;
        }

        // 检查是否已在更新中
        if (isUpdateInProgress()) {
            rsp.error = rsp.ERROR_IN_PROGRESS;
            return;
        }

        // 开始固件更新流程
        rsp.error = rsp.ERROR_OK;
        rsp.optional_error_message = "Firmware update started";

        // 异步执行固件更新
        startFirmwareUpdateProcess(req.image_file_remote_path);
    }

    void startFirmwareUpdateProcess(const std::string& firmware_path) {
        // 1. 进入bootloader模式
        enterBootloaderMode();

        // 2. 擦除应用程序区域
        eraseApplicationFlash();

        // 3. 下载并写入新固件
        downloadAndWriteFirmware(firmware_path);

        // 4. 验证固件完整性
        if (verifyFirmware()) {
            // 5. 重启到新固件
            systemReset();
        } else {
            // 固件验证失败，恢复到安全模式
            enterSafeMode();
        }
    }
};
```

### 参数存储系统

```cpp
// parameter_storage.cpp
class ESCParameterStorage {
private:
    struct ParameterData {
        float max_current;
        float max_temperature;
        uint16_t min_pwm;
        uint16_t max_pwm;
        bool reverse_direction;
        uint32_t crc32;  // 数据完整性校验
    };

    static constexpr uint32_t FLASH_PARAM_ADDR = 0x08010000;  // Flash地址

public:
    bool loadParameters(ParameterData& params) {
        // 从Flash读取参数
        const ParameterData* flash_params =
            reinterpret_cast<const ParameterData*>(FLASH_PARAM_ADDR);

        // 验证CRC
        uint32_t calculated_crc = calculateCRC32(flash_params,
            sizeof(ParameterData) - sizeof(uint32_t));

        if (calculated_crc == flash_params->crc32) {
            params = *flash_params;
            return true;
        }

        // CRC校验失败，使用默认参数
        setDefaultParameters(params);
        return false;
    }

    bool saveParameters(const ParameterData& params) {
        ParameterData params_with_crc = params;

        // 计算CRC
        params_with_crc.crc32 = calculateCRC32(&params_with_crc,
            sizeof(ParameterData) - sizeof(uint32_t));

        // 擦除Flash页
        if (!eraseFlashPage(FLASH_PARAM_ADDR)) {
            return false;
        }

        // 写入参数
        return writeFlash(FLASH_PARAM_ADDR, &params_with_crc, sizeof(ParameterData));
    }

private:
    void setDefaultParameters(ParameterData& params) {
        params.max_current = 30.0f;
        params.max_temperature = 80.0f;
        params.min_pwm = 1000;
        params.max_pwm = 2000;
        params.reverse_direction = false;
    }

    uint32_t calculateCRC32(const void* data, size_t length) {
        // CRC32计算实现
        const uint8_t* bytes = static_cast<const uint8_t*>(data);
        uint32_t crc = 0xFFFFFFFF;

        for (size_t i = 0; i < length; i++) {
            crc ^= bytes[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 1) {
                    crc = (crc >> 1) ^ 0xEDB88320;
                } else {
                    crc >>= 1;
                }
            }
        }

        return ~crc;
    }
};
```

### 诊断和日志系统

```cpp
// diagnostics.cpp
#include <uavcan/protocol/debug/LogMessage.hpp>

class ESCDiagnostics {
private:
    uavcan::Publisher<uavcan::protocol::debug::LogMessage> log_publisher;

    struct DiagnosticData {
        uint32_t total_runtime;      // 总运行时间 (秒)
        uint32_t motor_starts;       // 电机启动次数
        uint32_t overcurrent_events; // 过流事件次数
        uint32_t overtemp_events;    // 过温事件次数
        uint32_t comm_errors;        // 通信错误次数
        float max_current_seen;      // 历史最大电流
        float max_temp_seen;         // 历史最大温度
    } diagnostics;

public:
    ESCDiagnostics(uavcan::INode& node) : log_publisher(node) {
        memset(&diagnostics, 0, sizeof(diagnostics));
    }

    int init() {
        return log_publisher.init();
    }

    void logInfo(const std::string& message) {
        sendLogMessage(uavcan::protocol::debug::LogMessage::LEVEL_INFO, message);
    }

    void logWarning(const std::string& message) {
        sendLogMessage(uavcan::protocol::debug::LogMessage::LEVEL_WARNING, message);
    }

    void logError(const std::string& message) {
        sendLogMessage(uavcan::protocol::debug::LogMessage::LEVEL_ERROR, message);
    }

    void recordOvercurrentEvent(float current) {
        diagnostics.overcurrent_events++;
        if (current > diagnostics.max_current_seen) {
            diagnostics.max_current_seen = current;
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "过流事件: %.2fA", current);
        logWarning(msg);
    }

    void recordOvertempEvent(float temperature) {
        diagnostics.overtemp_events++;
        if (temperature > diagnostics.max_temp_seen) {
            diagnostics.max_temp_seen = temperature;
        }

        char msg[64];
        snprintf(msg, sizeof(msg), "过温事件: %.1f°C", temperature);
        logWarning(msg);
    }

    void publishDiagnosticSummary() {
        char summary[128];
        snprintf(summary, sizeof(summary),
            "运行时间:%us 启动:%u 过流:%u 过温:%u 最大电流:%.2fA 最大温度:%.1f°C",
            diagnostics.total_runtime,
            diagnostics.motor_starts,
            diagnostics.overcurrent_events,
            diagnostics.overtemp_events,
            diagnostics.max_current_seen,
            diagnostics.max_temp_seen);

        logInfo(summary);
    }

private:
    void sendLogMessage(uint8_t level, const std::string& message) {
        uavcan::protocol::debug::LogMessage log_msg;
        log_msg.level = level;
        log_msg.source = "ESC";
        log_msg.text = message.c_str();

        log_publisher.broadcast(log_msg);
    }
};
```

## 生产部署

### 质量控制检查清单

#### 硬件检查
- [ ] PCB布局符合EMC要求
- [ ] 所有连接器牢固可靠
- [ ] 散热设计充分
- [ ] 电源滤波充分
- [ ] CAN总线终端电阻正确

#### 软件检查
- [ ] 所有安全功能正常工作
- [ ] 参数存储和恢复正常
- [ ] UAVCAN通信稳定
- [ ] 固件更新功能正常
- [ ] 诊断日志完整

#### 测试验证
- [ ] 单元测试全部通过
- [ ] 集成测试全部通过
- [ ] 硬件在环测试通过
- [ ] 长时间稳定性测试通过
- [ ] 极限条件测试通过

### 生产配置

```cpp
// production_config.h
#ifndef PRODUCTION_CONFIG_H
#define PRODUCTION_CONFIG_H

// 生产版本配置
#define ESC_VERSION_MAJOR       1
#define ESC_VERSION_MINOR       0
#define ESC_VERSION_PATCH       0

// 硬件配置
#define MAX_CURRENT_RATING      30.0f   // 30A
#define MAX_VOLTAGE_RATING      25.0f   // 25V
#define MAX_TEMPERATURE_RATING  85.0f   // 85°C
#define MAX_RPM_RATING          50000   // 50000 RPM

// 安全配置
#define COMMAND_TIMEOUT_MS      500     // 命令超时
#define OVERCURRENT_THRESHOLD   32.0f   // 过流阈值 (留10%余量)
#define OVERTEMP_THRESHOLD      80.0f   // 过温阈值
#define STARTUP_DELAY_MS        1000    // 启动延迟

// UAVCAN配置
#define DEFAULT_NODE_ID         20      // 默认节点ID
#define DEFAULT_ESC_INDEX       0       // 默认ESC索引
#define STATUS_PUBLISH_RATE_HZ  10      // 状态发布频率

// 调试配置
#ifdef DEBUG
    #define ENABLE_DEBUG_LOGGING    1
    #define ENABLE_PERFORMANCE_MONITORING 1
#else
    #define ENABLE_DEBUG_LOGGING    0
    #define ENABLE_PERFORMANCE_MONITORING 0
#endif

#endif // PRODUCTION_CONFIG_H
```

### 制造测试程序

```cpp
// manufacturing_test.cpp
class ManufacturingTest {
public:
    struct TestResults {
        bool hardware_test_passed;
        bool communication_test_passed;
        bool motor_test_passed;
        bool safety_test_passed;
        bool calibration_completed;

        float measured_resistance;
        float measured_inductance;
        float no_load_current;

        std::string serial_number;
        std::string test_date;
        std::string operator_id;
    };

    TestResults runFullTest() {
        TestResults results = {};

        // 1. 硬件测试
        results.hardware_test_passed = testHardware();

        // 2. 通信测试
        results.communication_test_passed = testCommunication();

        // 3. 电机测试
        results.motor_test_passed = testMotor(results);

        // 4. 安全功能测试
        results.safety_test_passed = testSafetyFeatures();

        // 5. 校准
        results.calibration_completed = performCalibration();

        // 6. 生成序列号和测试记录
        results.serial_number = generateSerialNumber();
        results.test_date = getCurrentDate();
        results.operator_id = getOperatorID();

        // 7. 保存测试结果
        saveTestResults(results);

        return results;
    }

private:
    bool testHardware() {
        // 测试电源电压
        float voltage = readSupplyVoltage();
        if (voltage < 11.0f || voltage > 13.0f) return false;

        // 测试CAN通信
        if (!testCANInterface()) return false;

        // 测试ADC
        if (!testADCChannels()) return false;

        // 测试PWM输出
        if (!testPWMOutputs()) return false;

        return true;
    }

    bool testMotor(TestResults& results) {
        // 测量电机参数
        results.measured_resistance = measureMotorResistance();
        results.measured_inductance = measureMotorInductance();

        // 验证参数范围
        if (results.measured_resistance < 0.1f || results.measured_resistance > 1.0f) {
            return false;
        }

        // 测试空载电流
        results.no_load_current = measureNoLoadCurrent();
        if (results.no_load_current > 2.0f) {
            return false;
        }

        return true;
    }

    bool performCalibration() {
        // 电流传感器零点校准
        calibrateCurrentSensor();

        // 温度传感器校准
        calibrateTemperatureSensor();

        // 电压传感器校准
        calibrateVoltageSensor();

        // 保存校准数据
        saveCalibrationData();

        return true;
    }
};
```

### 部署指南

#### 1. 固件烧录
```bash
# 使用ST-Link烧录固件
st-flash write esc_firmware.bin 0x8000000

# 或使用DFU模式
dfu-util -a 0 -s 0x08000000 -D esc_firmware.bin
```

#### 2. 初始配置
```bash
# 设置节点ID (每个ESC唯一)
uavcan param set node_id 20

# 设置ESC索引
uavcan param set esc_index 0

# 设置安全参数
uavcan param set max_current 30.0
uavcan param set max_temperature 80.0

# 保存配置
uavcan param save
```

#### 3. 验证部署
```bash
# 检查节点状态
uavcan info

# 测试基本功能
uavcan test esc 20

# 验证参数设置
uavcan param list 20
```

## 总结

本指南涵盖了从硬件设计到生产部署的完整电调开发流程：

### 关键要点
1. **硬件设计**: 选择合适的MCU和功率器件，注意EMC和散热
2. **软件架构**: 分层设计，确保实时性和安全性
3. **UAVCAN实现**: 严格遵循协议标准，确保互操作性
4. **安全功能**: 实现完整的保护机制，确保飞行安全
5. **测试验证**: 全面的测试覆盖，确保产品质量
6. **生产部署**: 标准化的制造和配置流程

### 开发建议
- 从简单功能开始，逐步增加复杂性
- 重视安全功能的实现和测试
- 保持与PX4社区的沟通和协作
- 遵循开源硬件和软件的最佳实践

通过遵循本指南，你可以开发出高质量、安全可靠的UAVCAN电调，并成功集成到PX4生态系统中。
```
