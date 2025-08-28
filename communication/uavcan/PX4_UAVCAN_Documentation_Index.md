# PX4 UAVCAN 文档索引

本目录包含PX4 UAVCAN功能的完整文档集合，帮助用户快速上手和解决问题。

## 文档列表

### 📚 主要文档

#### 1. [PX4_UAVCAN_ESC_Usage_Guide.md](./PX4_UAVCAN_ESC_Usage_Guide.md)
**完整使用指南** - 详细的UAVCAN ESC配置和使用教程

**内容包括**:
- UAVCAN基本概念和原理
- 硬件连接要求和布线图
- 详细的配置步骤和参数说明
- 启动验证和测试方法
- 高级配置选项
- Python和C++示例代码
- 完整的附录和参考资料

**适用对象**: 初学者到高级用户
**预计阅读时间**: 30-45分钟

#### 2. [PX4_UAVCAN_Quick_Reference.md](./PX4_UAVCAN_Quick_Reference.md)
**快速参考手册** - 5分钟快速配置指南

**内容包括**:
- 最小化配置步骤
- 常用命令速查
- 参数对照表
- 硬件连接简图
- 常见问题快速解决

**适用对象**: 有经验的用户
**预计阅读时间**: 5-10分钟

#### 3. [PX4_UAVCAN_Troubleshooting.md](./PX4_UAVCAN_Troubleshooting.md)
**故障排除指南** - 专业的问题诊断和解决方案

**内容包括**:
- 系统化的问题诊断流程
- 常见问题及解决方案
- 高级诊断工具使用
- 性能优化建议
- 紧急恢复程序
- 预防措施和最佳实践

**适用对象**: 技术支持人员和高级用户
**预计阅读时间**: 20-30分钟

#### 4. [PX4_UAVCAN_Custom_Protocol_Guide.md](./PX4_UAVCAN_Custom_Protocol_Guide.md)
**自定义协议开发指南** - 详细的自定义UAVCAN消息开发教程

**内容包括**:
- DSDL消息定义语法和规范
- 自定义命名空间创建
- C++接口类开发
- 编译集成流程
- Python测试脚本
- 隧道协议使用方法

**适用对象**: 开发人员和高级用户
**预计阅读时间**: 45-60分钟

#### 5. [PX4_UAVCAN_Custom_Integration_Example.md](./PX4_UAVCAN_Custom_Integration_Example.md)
**自定义协议集成实例** - 完整的FiHawk自定义协议实现示例

**内容包括**:
- 完整的实例代码和文件结构
- 逐步集成指导
- 编译和测试验证
- 故障排除和调试技巧
- 扩展开发建议

**适用对象**: 开发人员
**预计阅读时间**: 30-40分钟

#### 6. [PX4_UAVCAN_Correct_Implementation_Guide.md](./PX4_UAVCAN_Correct_Implementation_Guide.md)
**正确实现指南** - 纠正错误，展示标准的UavcanSensorBridgeBase继承模式

**内容包括**:
- 正确的继承UavcanSensorBridgeBase模式
- 与现有传感器实现的对比
- 标准的注册和集成流程
- uORB消息创建和使用
- 完整的代码示例

**适用对象**: 开发人员 (必读)
**预计阅读时间**: 20-30分钟

#### 7. [PX4_UAVCAN_ESC_Development_Guide.md](./PX4_UAVCAN_ESC_Development_Guide.md)
**电调开发完整指南** - 从零开始开发UAVCAN电调的完整教程

**内容包括**:
- 硬件设计和PCB布局指导
- 软件架构和状态机设计
- 完整的UAVCAN节点实现
- 硬件抽象层和驱动开发
- PX4集成和参数配置
- 测试验证和质量控制
- 高级功能 (固件更新、诊断)
- 生产部署和制造测试

**适用对象**: 硬件开发人员和系统集成工程师
**预计阅读时间**: 60-90分钟

## 使用建议

### 🚀 新手入门路径

1. **第一次使用UAVCAN**:
   ```
   快速参考手册 → 完整使用指南 → 实际配置测试
   ```

2. **遇到问题时**:
   ```
   故障排除指南 → 完整使用指南相关章节 → 社区求助
   ```

3. **日常使用**:
   ```
   快速参考手册 (常用命令和参数)
   ```

### 📖 阅读顺序建议

#### 初学者 (第一次接触UAVCAN)
1. 阅读完整使用指南的"UAVCAN基本概念"章节
2. 按照"硬件要求"准备硬件
3. 跟随"配置步骤"进行设置
4. 使用"启动和验证"确认配置正确
5. 收藏快速参考手册备用

#### 有经验用户 (熟悉CAN总线或其他ESC协议)
1. 快速浏览快速参考手册
2. 直接按照快速配置步骤操作
3. 如遇问题查阅故障排除指南
4. 需要详细信息时参考完整使用指南

#### 开发人员 (需要自定义协议)
1. 先完成基本UAVCAN配置 (使用快速参考手册)
2. 阅读自定义协议开发指南了解原理
3. 参考自定义协议集成实例进行实际开发
4. 使用故障排除指南解决开发中的问题

#### 硬件开发人员 (开发UAVCAN设备)
1. 阅读完整使用指南了解UAVCAN基础
2. 重点学习电调开发完整指南
3. 参考正确实现指南了解软件架构
4. 使用故障排除指南解决硬件集成问题

#### 技术支持人员
1. 熟悉所有文档的内容
2. 重点掌握故障排除指南的诊断流程
3. 了解完整使用指南的高级配置选项
4. 掌握自定义协议的调试方法

## 相关源码文件

### 核心实现文件
- `src/drivers/uavcan/actuators/esc.cpp` - ESC控制实现
- `src/drivers/uavcan/actuators/esc.hpp` - ESC控制头文件
- `src/drivers/uavcan/uavcan_main.cpp` - UAVCAN主驱动
- `src/drivers/uavcan/uavcan_params.c` - UAVCAN参数定义

### 配置文件
- `src/drivers/uavcan/module.yaml` - 模块配置
- `ROMFS/px4fmu_common/init.d/rcS` - 启动脚本

### 示例代码
- `src/drivers/uavcan/libuavcan/libuavcan/dsdl_compiler/pyuavcan/examples/esc_control.py`
- `src/drivers/uavcan/libuavcan/libuavcan/dsdl_compiler/pyuavcan/examples/esc_enumeration.py`

## 外部资源

### 官方文档
- [PX4官方文档](https://docs.px4.io/) - PX4完整文档
- [UAVCAN规范](https://uavcan.org/) - UAVCAN协议规范
- [DroneCAN文档](https://dronecan.github.io/) - DroneCAN实现文档

### 社区支持
- [PX4论坛](https://discuss.px4.io/) - 官方论坛
- [GitHub Issues](https://github.com/PX4/PX4-Autopilot/issues) - 问题报告
- [Discord社区](https://discord.gg/dronecode) - 实时交流

### 硬件厂商
- [Zubax Robotics](https://zubax.com/) - UAVCAN设备制造商
- [CUAV](https://www.cuav.net/) - 飞控和ESC制造商
- [Holybro](https://holybro.com/) - 无人机硬件制造商

## 版本信息

| 文档 | 版本 | 最后更新 | 适用PX4版本 |
|------|------|----------|-------------|
| 完整使用指南 | v1.0 | 2024-08-22 | v1.14+ |
| 快速参考手册 | v1.0 | 2024-08-22 | v1.14+ |
| 故障排除指南 | v1.0 | 2024-08-22 | v1.14+ |
| 自定义协议开发指南 | v1.0 | 2024-08-22 | v1.14+ |
| 自定义协议集成实例 | v1.0 | 2024-08-22 | v1.14+ |
| 正确实现指南 | v1.0 | 2024-08-22 | v1.14+ |
| 电调开发完整指南 | v1.0 | 2024-08-22 | v1.14+ |

## 贡献指南

### 文档更新
如果您发现文档中的错误或需要补充内容，请:

1. 在GitHub上提交Issue
2. 或者直接提交Pull Request
3. 或者在PX4论坛上反馈

### 内容建议
我们欢迎以下类型的贡献:

- 错误修正和澄清
- 新的配置示例
- 额外的故障排除案例
- 性能优化建议
- 硬件兼容性信息

### 格式要求
- 使用Markdown格式
- 保持与现有文档风格一致
- 包含适当的代码示例
- 添加必要的警告和注意事项

## 反馈和支持

### 文档反馈
- 📧 邮箱: [文档维护者邮箱]
- 💬 论坛: [PX4论坛UAVCAN版块]
- 🐛 问题报告: [GitHub Issues]

### 技术支持
- 🔧 技术问题: 请先查阅故障排除指南
- 💡 功能建议: 在GitHub上提交Feature Request
- 🤝 社区讨论: 加入Discord或论坛讨论

---

**最后更新**: 2024-08-22
**文档维护者**: PX4开发团队
**许可证**: BSD 3-Clause
