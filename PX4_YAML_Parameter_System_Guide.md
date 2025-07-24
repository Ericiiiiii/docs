# PX4模块.yaml文件与参数系统详解

## 概述

PX4中的`.yaml`文件是一个**参数定义和模块配置系统**，用于声明式地定义模块参数，并在构建时自动生成对应的C代码。这是PX4参数系统的核心组成部分。

## 主要用途

### 1. 参数定义
- 定义模块的可配置参数
- 包含参数的元数据（描述、类型、默认值、范围等）
- 支持多种参数类型（boolean、int32、float、enum、bitmask）

### 2. 模块配置
- 执行器输出配置（PWM输出、舵机控制等）
- 串口配置参数
- 模块级别的配置信息

### 3. 文档生成
- 自动生成参数文档
- 为QGroundControl提供参数元数据
- 生成XML/JSON格式的参数描述文件

## 文件结构

### 基本结构
```yaml
module_name: <模块名称>

parameters:
    - group: <参数组名>
      definitions:
        <参数名>:
            description:
                short: <简短描述>
                long: <详细描述>
            type: <参数类型>
            default: <默认值>
            # 其他属性...
```

### 支持的参数类型
- `boolean` - 布尔值（转换为INT32）
- `int32` - 32位整数
- `float` - 浮点数
- `enum` - 枚举类型
- `bitmask` - 位掩码

### 常用属性
- `default` - 默认值
- `min/max` - 取值范围
- `unit` - 单位
- `decimal` - 小数位数
- `reboot_required` - 是否需要重启生效
- `volatile` - 易失性参数（不保存到EEPROM）
- `category` - 参数分类

## 代码生成过程

### 1. 构建时处理
```bash
# CMake收集所有.yaml文件
file(GLOB_RECURSE yaml_config_files ${PX4_SOURCE_DIR}/src/modules/*.yaml)

# Python脚本处理.yaml文件
python3 Tools/module_config/generate_params.py \
    --params-file generated_params/module_params.c \
    --config-files ${yaml_config_files}
```

### 2. 生成的文件位置
- `build/{board}/generated_params/module_params.c` - 模块参数定义
- `build/{board}/generated_params/serial_params.c` - 串口参数定义
- `build/{board}/parameters.xml` - 参数元数据XML
- `build/{board}/parameters.json` - 参数元数据JSON

### 3. 代码转换示例

**输入 (.yaml):**
```yaml
module_name: ekf2
parameters:
- group: EKF2
  definitions:
    EKF2_PREDICT_US:
      description:
        short: EKF prediction period
        long: EKF prediction period in microseconds...
      type: int32
      default: 10000
      min: 1000
      max: 20000
      unit: us
```

**输出 (生成的C代码):**
```c
/**
 * EKF prediction period
 *
 * EKF prediction period in microseconds...
 *
 * @group EKF2
 * @min 1000
 * @max 20000
 * @unit us
 */
PARAM_DEFINE_INT32(EKF2_PREDICT_US, 10000);
```

## 参数定义宏详解

### 基本语法
```c
PARAM_DEFINE_INT32(参数名, 默认值);
PARAM_DEFINE_FLOAT(参数名, 默认值);
```

### 宏定义家族
```c
// 整数参数
PARAM_DEFINE_INT32(EKF2_PREDICT_US, 10000);
PARAM_DEFINE_INT32(COM_FLTMODE1, -1);

// 浮点参数  
PARAM_DEFINE_FLOAT(EKF2_GYR_NOISE, 0.015f);
PARAM_DEFINE_FLOAT(BAT1_V_EMPTY, 3.6f);
```

## 特殊类型的.yaml文件

### 1. 多实例参数
```yaml
# params_multi.yaml - 支持多个EKF实例
module_name: ekf2
parameters:
- group: EKF2
  definitions:
    EKF2_${i}_PARAM:
      num_instances: 2
      instance_start: 1
```

### 2. 易失性参数
```yaml
# params_volatile.yaml - 不保存到EEPROM的参数
module_name: ekf2
parameters:
- group: EKF2
  definitions:
    EKF2_MAG_DECL:
      volatile: true  # 易失性参数
```

### 3. 执行器输出配置
```yaml
module_name: pwm_out
actuator_output:
  output_groups:
    - param_prefix: PWM_MAIN
      channel_label: PWM Main
      num_channels: 8
      standard_params:
        disarmed: { min: 800, max: 2200, default: 900 }
        min: { min: 800, max: 2200, default: 1000 }
        max: { min: 800, max: 2200, default: 2000 }
```

## 运行时机制

### 1. 构建时
- CMake收集所有.yaml文件
- Python脚本生成C参数定义
- 编译到parameters库中

### 2. 运行时
- 系统启动时加载参数定义
- 参数值从EEPROM读取
- 通过uORB发布参数更新事件

### 3. 用户交互
- QGroundControl参数配置界面
- 命令行`param`命令
- MAVLink参数协议

## 实际应用示例

### EKF2模块参数
```yaml
module_name: ekf2
parameters:
- group: EKF2
  definitions:
    EKF2_EN:
      description:
        short: EKF2 enable
      type: boolean
      default: 1
    
    EKF2_GPS_CHECK:
      description:
        short: Integer bitmask controlling GPS checks
      type: bitmask
      bit:
        0: Min sat count
        1: Max PDOP
        2: Max horizontal position error
      default: 245
```

### 电池状态模块参数
```yaml
module_name: battery_status
parameters:
- group: Battery Calibration
  definitions:
    BAT${i}_V_DIV:
      description:
        short: Battery ${i} voltage divider
      type: float
      num_instances: 2
      instance_start: 1
      default: [-1.0, -1.0]
      reboot_required: true
```

## 参数系统架构

### 参数存储结构
```c
typedef struct param_info_s {
    const char *name;           // 参数名
    param_type_t type;          // 参数类型
    param_value_u default_val;  // 默认值
    // 元数据信息...
} param_info_t;
```

### 运行时参数操作
```c
// 参数读取
int32_t predict_us;
param_get(param_find("EKF2_PREDICT_US"), &predict_us);

// 参数设置
int32_t new_value = 8000;
param_set(param_find("EKF2_PREDICT_US"), &new_value);

// 参数保存
param_save_default();
```

## 调试和故障排除

### 参数查看命令
```bash
# 在PX4控制台中
param show EKF2_PREDICT_US    # 显示参数值
param set EKF2_PREDICT_US 8000  # 设置参数值
param save                    # 保存参数
param reset EKF2_PREDICT_US   # 重置为默认值
```

## 最佳实践

### 1. 参数命名
- 使用模块前缀（如`EKF2_`、`MC_`）
- 参数名要简洁明了
- 遵循PX4命名约定

### 2. 描述编写
- `short`描述要简洁（用于UI显示）
- `long`描述要详细（包含使用说明）
- 包含单位和取值范围说明

### 3. 默认值设置
- 选择安全的默认值
- 考虑不同平台的兼容性
- 文档化默认值的选择原因

### 4. 参数分组
- 按功能逻辑分组
- 组名要直观易懂
- 避免过深的层级结构

## 总结

PX4的.yaml文件系统是一个强大的**声明式参数定义框架**，它：

1. **在构建时**将YAML配置转换为C代码
2. **在运行时**提供完整的参数系统支持
3. **为用户提供**统一的参数配置接口
4. **为开发者提供**高效的参数管理方案

这种设计使得PX4的参数系统既强大又易于维护，是现代嵌入式软件架构的优秀实践。

---

*本文档基于PX4 v1.14版本编写，详细分析了.yaml文件的作用机制和代码生成过程。*
