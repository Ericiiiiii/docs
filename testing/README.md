# 测试框架文档

本目录包含PX4 FihawkFlyCtrl项目的测试框架相关文档，涵盖单元测试、集成测试、硬件在环测试等内容。

## 测试体系概述

PX4测试框架支持多层次的测试验证：
- **单元测试**: 模块和函数级别的测试
- **集成测试**: 模块间交互的测试
- **SITL测试**: 软件在环仿真测试
- **HITL测试**: 硬件在环测试
- **飞行测试**: 实际硬件飞行验证

## 测试命令

### 基本测试命令
```bash
# 运行所有单元测试
make tests

# 运行特定模块测试
make fihawk_fc-v1_default test_<module_name>

# SITL集成测试
make px4_sitl_default test

# 代码覆盖率测试
make coverage
```

### 代码质量检查
```bash
# 代码格式检查
make check_format

# 自动格式化
make format

# 静态代码分析
make px4_sitl_default clang-tidy

# 代码复杂度分析
make complexity_analysis
```

## 测试框架结构

### 单元测试框架
- **GoogleTest**: C++单元测试框架
- **模拟对象**: 硬件抽象层的模拟实现
- **测试夹具**: 可重用的测试环境设置

### 集成测试框架
- **ROS2测试**: ROS2接口的集成测试
- **MAVLink测试**: 通信协议的集成测试
- **uORB测试**: 消息总线的集成测试

### SITL仿真测试
- **Gazebo仿真**: 3D物理仿真环境
- **JMAVSim仿真**: 轻量级仿真环境
- **AirSim仿真**: 微软AirSim仿真平台

## 测试目录结构

```
├── test/                  # 测试根目录
│   ├── unit/             # 单元测试
│   │   ├── math/         # 数学库测试
│   │   ├── control/      # 控制算法测试
│   │   └── drivers/      # 驱动测试
│   ├── integration/      # 集成测试
│   │   ├── ros2/         # ROS2集成测试
│   │   └── mavlink/      # MAVLink集成测试
│   └── sitl/             # SITL测试
│       ├── standard/     # 标准飞行测试
│       └── mission/      # 任务飞行测试
```

## 测试开发指南

### 编写单元测试
```cpp
#include <gtest/gtest.h>
#include "module_under_test.h"

class ModuleTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 测试环境设置
    }
    
    void TearDown() override {
        // 测试环境清理
    }
};

TEST_F(ModuleTest, TestFunctionality) {
    // 测试用例实现
    EXPECT_EQ(expected_value, actual_value);
}
```

### 测试最佳实践
- **AAA模式**: Arrange(准备) → Act(执行) → Assert(断言)
- **独立性**: 每个测试用例应该独立，不依赖其他测试
- **可读性**: 测试名称应该清晰描述被测试的功能
- **覆盖率**: 追求高代码覆盖率，但重视质量胜过数量

## 持续集成

### CI/CD流程
1. **代码提交**: 触发自动化测试
2. **单元测试**: 验证模块功能正确性
3. **集成测试**: 验证模块间交互
4. **SITL测试**: 验证飞行逻辑
5. **代码质量**: 检查格式和复杂度
6. **构建验证**: 多平台构建测试

### 测试报告
- **覆盖率报告**: 代码覆盖率统计
- **性能报告**: 测试执行时间分析
- **质量报告**: 代码质量度量
- **回归报告**: 功能回归测试结果

## Fihawk特定测试

### 硬件相关测试
- **传感器测试**: IMU、GPS、磁力计等传感器功能测试
- **执行器测试**: 电机、舵机等执行器响应测试
- **通信测试**: 串口、SPI、I2C等通信接口测试

### 板级测试
- **引导程序测试**: Bootloader功能验证
- **固件加载测试**: 固件烧录和启动测试
- **实时性测试**: 系统实时性能验证

## 故障排除

### 常见测试问题
- **测试环境**: 确保测试依赖正确安装
- **权限问题**: 检查测试文件的执行权限
- **资源限制**: 注意内存和CPU使用限制
- **并发冲突**: 避免测试间的资源竞争

### 调试技巧
- 使用`--gtest_filter`运行特定测试
- 使用`--gtest_repeat`重复运行测试
- 使用调试器逐步调试测试代码
- 查看详细的测试输出日志

## 相关文档

- [构建系统](../build-system/) - 测试构建配置
- [开发环境](../development/) - 开发和测试环境配置
- [故障排除](../troubleshooting/) - 测试相关问题解决

---

完善的测试是保证代码质量和系统稳定性的关键，建议在开发过程中持续进行测试驱动开发。