# 参数系统文档

本目录包含PX4参数系统的完整技术文档，从参数定义到配置管理的详细实现分析。

## 文档列表

- **[PX4参数系统学习指南](PX4_Parameter_Study_README.md)** - PX4参数系统的学习入门指南
- **[PX4参数系统详细文档](PX4_Parameter_System_Documentation.md)** - PX4参数系统的详细技术文档和架构分析
- **[PX4 YAML参数系统指南](PX4_YAML_Parameter_System_Guide.md)** - YAML格式参数配置的使用指南和最佳实践

## 参数系统架构

### 核心组件
```
参数定义 (PARAM_DEFINE_*)
        ↓
参数存储 (Flash/EEPROM)
        ↓
参数管理器 (param module)
        ↓
参数API (param_get/param_set)
        ↓
应用模块使用
```

### 参数分层结构

#### 1. 定义层 (Definition Layer)
```cpp
// 在源码中定义参数
PARAM_DEFINE_FLOAT(MC_ROLL_P, 6.5f);
PARAM_DEFINE_INT32(MAV_SYS_ID, 1);
```

#### 2. 存储层 (Storage Layer)
- **持久化存储**：Flash/EEPROM
- **运行时存储**：RAM缓存
- **备份机制**：多重备份保护

#### 3. 管理层 (Management Layer)
- **参数服务器**：集中式参数管理
- **访问控制**：读写权限管理
- **变更通知**：参数修改事件

#### 4. 接口层 (Interface Layer)
- **C API**：`param_get()`, `param_set()`
- **命令行**：`param` 命令
- **MAVLink**：远程参数访问
- **Web界面**：QGroundControl参数编辑器

## 参数类型系统

### 基础数据类型
| 类型 | 定义宏 | 范围 | 示例 |
|------|-------|------|------|
| INT32 | PARAM_DEFINE_INT32 | -2³¹ ~ 2³¹-1 | 节点ID、计数器 |
| FLOAT | PARAM_DEFINE_FLOAT | IEEE 754 | PID系数、阈值 |

### 参数属性
```cpp
/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 12.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required false
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLL_P, 6.5f);
```

### 参数分组
- **Vehicle**: 飞行器基础配置
- **Commander**: 飞行模式和安全
- **Navigator**: 导航和任务
- **MC_***: 多旋翼控制参数
- **FW_***: 固定翼控制参数
- **SENS_***: 传感器配置
- **CAL_***: 校准参数

## 参数配置工具

### 1. 命令行工具
```bash
# 显示所有参数
param show

# 显示特定参数
param show MC_ROLL_P

# 设置参数值
param set MC_ROLL_P 7.0

# 保存参数到存储
param save

# 重置参数到默认值
param reset MC_ROLL_P

# 导出参数到文件
param export /fs/microsd/params_backup.txt

# 从文件导入参数
param import /fs/microsd/params_backup.txt
```

### 2. MAVLink协议
```cpp
// 参数请求
PARAM_REQUEST_READ
PARAM_REQUEST_LIST

// 参数设置
PARAM_SET

// 参数值返回
PARAM_VALUE
```

### 3. QGroundControl
- **参数编辑器**：图形化参数配置界面
- **分组浏览**：按功能分组显示参数
- **搜索功能**：快速定位参数
- **批量操作**：导入/导出参数文件
- **实时同步**：自动同步参数变更

## YAML配置系统

### 配置文件结构
```yaml
# airframe.yaml
parameters:
  # 多旋翼基础配置
  MC_ROLL_P: 6.5
  MC_PITCH_P: 6.5
  MC_YAW_P: 2.8
  
  # 传感器配置
  SENS_BOARD_ROT: 0
  CAL_GYRO0_ID: 2293768
  
  # 电调配置
  PWM_MAIN_FUNC1: 101  # Motor 1
  PWM_MAIN_FUNC2: 102  # Motor 2
```

### 应用场景
- **机架配置**：预定义机架参数
- **批量部署**：统一配置多台设备
- **版本管理**：参数配置版本控制
- **模板化**：可重用的配置模板

## 参数开发指南

### 1. 参数定义最佳实践
```cpp
/**
 * 参数名称要清晰描述功能
 * 
 * 详细描述参数的作用和影响
 * 提供使用建议和注意事项
 *
 * @min 最小值
 * @max 最大值  
 * @decimal 小数位数
 * @increment 调节步长
 * @unit 单位
 * @reboot_required 是否需要重启
 * @group 参数分组
 */
PARAM_DEFINE_FLOAT(PREFIX_PARAM_NAME, default_value);
```

### 2. 参数命名规范
- **模块前缀**：如MC_、FW_、NAV_等
- **功能描述**：清晰表达参数用途
- **层次结构**：使用下划线分隔层次
- **一致性**：同类参数保持命名风格统一

### 3. 参数使用模式
```cpp
class MyModule : public ModuleBase<MyModule>
{
private:
    // 参数句柄缓存
    param_t _param_gain_p_handle{PARAM_INVALID};
    param_t _param_gain_i_handle{PARAM_INVALID};
    
    // 参数值缓存
    float _param_gain_p{0.0f};
    float _param_gain_i{0.0f};
    
    // 参数更新检查
    void updateParams();
};

void MyModule::updateParams()
{
    // 检查参数是否有更新
    updateParams();
    
    // 获取最新参数值
    param_get(_param_gain_p_handle, &_param_gain_p);
    param_get(_param_gain_i_handle, &_param_gain_i);
}
```

## 参数持久化机制

### 存储策略
- **延迟写入**：避免频繁Flash擦写
- **完整性校验**：CRC校验防止数据损坏
- **多重备份**：主备份+备用备份
- **磨损均衡**：Flash擦写均衡算法

### 存储格式
```
参数存储块:
[Header]
- Magic Number (4字节)
- Version (2字节) 
- Count (2字节)
- CRC (4字节)

[Parameter Data]
- Param1: Type(1) + Length(1) + Name(n) + Value(m)
- Param2: Type(1) + Length(1) + Name(n) + Value(m)
- ...
```

## 调试与诊断

### 参数问题诊断
```bash
# 检查参数系统状态
param status

# 验证参数完整性
param verify

# 查看参数变更历史
dmesg | grep param

# 重置损坏的参数
param reset_all
```

### 常见问题
1. **参数丢失**：Flash损坏或电源异常
2. **参数不生效**：模块未正确读取参数
3. **参数范围错误**：超出定义的min/max范围
4. **重启要求**：某些参数需要重启后生效

## 性能优化

### 1. 参数缓存策略
- 模块启动时批量读取参数
- 使用本地变量缓存频繁访问的参数
- 监听参数变更事件，及时更新缓存

### 2. 参数访问优化
- 避免在中断中进行参数访问
- 使用参数句柄而非参数名访问
- 批量获取相关参数减少API调用

### 3. 存储空间优化
- 合理规划参数数量和大小
- 定期清理未使用的参数定义
- 使用合适的数据类型

## 相关文档

- [飞行控制系统](../flight-control/) - 控制参数的具体应用
- [通信协议](../communication/) - 参数的远程配置方法
- [硬件特定](../hardware-specific/) - 特定硬件的参数配置

---

本目录文档详细介绍PX4参数系统的设计理念和使用方法，适合系统开发人员和高级用户参考。