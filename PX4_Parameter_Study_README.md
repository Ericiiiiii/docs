# PX4参数系统学习笔记

## 文档说明

本仓库包含了对PX4飞控系统中`.yaml`文件和参数系统的详细分析和学习笔记。

## 文档列表

### 📋 [PX4_YAML_Parameter_System_Guide.md](./PX4_YAML_Parameter_System_Guide.md)
**PX4模块.yaml文件与参数系统详解**

这是一份完整的技术文档，详细介绍了：

- ✅ PX4中`.yaml`文件的作用和用途
- ✅ 参数定义的语法和结构
- ✅ 从YAML到C代码的自动生成过程
- ✅ `PARAM_DEFINE_INT32`等宏的详细解析
- ✅ 参数系统的运行时机制
- ✅ 实际应用示例和最佳实践

## 核心要点总结

### 🎯 关键概念
1. **代码生成系统**: `.yaml`文件在构建时被转换为C参数定义代码
2. **声明式配置**: 通过YAML声明参数，自动生成实现代码
3. **元数据驱动**: 参数包含完整的元数据（描述、类型、范围等）

### 🔄 工作流程
```
YAML配置 → Python脚本处理 → 生成C代码 → 编译到固件 → 运行时使用
```

### 📁 生成文件位置
- `build/{board}/generated_params/module_params.c` - 模块参数定义
- `build/{board}/generated_params/serial_params.c` - 串口参数定义
- `build/{board}/parameters.xml` - 参数元数据XML
- `build/{board}/parameters.json` - 参数元数据JSON

### 🛠️ 常用工具
- `Tools/module_config/generate_params.py` - 参数生成脚本
- `param` 命令 - PX4控制台参数操作
- QGroundControl - 图形化参数配置

## 学习路径建议

### 初学者
1. 先阅读"概述"和"主要用途"部分
2. 理解基本的YAML文件结构
3. 查看简单的代码转换示例

### 进阶学习
1. 深入理解代码生成过程
2. 学习参数系统架构
3. 掌握调试和故障排除方法

### 实践应用
1. 尝试修改现有模块的参数定义
2. 创建自定义模块的参数配置
3. 理解参数在QGC中的显示机制

## 相关资源

### 官方文档
- [PX4开发指南](https://dev.px4.io/)
- [PX4参数参考](https://docs.px4.io/main/en/advanced_config/parameter_reference.html)

### 源码位置
- `src/lib/parameters/` - 参数系统核心代码
- `Tools/module_config/` - 参数生成工具
- `src/modules/*/module.yaml` - 各模块参数定义

### 示例文件
- `src/modules/ekf2/module.yaml` - EKF2模块参数
- `src/modules/commander/module.yaml` - 指挥官模块参数
- `src/modules/battery_status/module.yaml` - 电池状态模块参数

## 实用命令速查

### 参数操作命令
```bash
# 查看参数值
param show EKF2_PREDICT_US

# 设置参数值
param set EKF2_PREDICT_US 8000

# 保存参数
param save

# 重置参数
param reset EKF2_PREDICT_US

# 导出/导入参数
param export /fs/microsd/params.txt
param import /fs/microsd/params.txt
```

### 构建相关命令
```bash
# 手动生成参数文件
python3 Tools/module_config/generate_params.py \
    --config-files src/modules/ekf2/module.yaml \
    --params-file /tmp/test_params.c

# 查看生成的参数文件
find build -name "*params*.c" -type f
```

## 常见问题解答

### Q: 为什么有些模块有多个.yaml文件？
A: 不同的.yaml文件有不同用途：
- `module.yaml` - 主要参数定义
- `params_multi.yaml` - 多实例参数
- `params_volatile.yaml` - 易失性参数
- `params_selector.yaml` - 选择器参数

### Q: 参数修改后何时生效？
A: 取决于参数的`reboot_required`属性：
- `reboot_required: true` - 需要重启生效
- `reboot_required: false` - 立即生效

### Q: 如何添加新的参数？
A: 
1. 在对应模块的`module.yaml`中添加参数定义
2. 重新构建固件
3. 在模块代码中使用`DEFINE_PARAMETERS`宏引用参数

## 更新记录

- **2024-06-25**: 创建初始版本，包含完整的参数系统分析
- 基于PX4 v1.14版本的代码分析

## 贡献说明

这是个人学习笔记，如有错误或需要补充的内容，欢迎指正。

---

**Happy Learning! 🚁**
