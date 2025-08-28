# PX4 UAVCAN 文档集合

本目录包含PX4中UAVCAN（CAN总线协议）实现和使用的完整文档。

## 文档概述

### 快速入门
- **[快速参考手册](PX4_UAVCAN_Quick_Reference.md)** - 快速命令和基本UAVCAN操作
- **[文档索引](PX4_UAVCAN_Documentation_Index.md)** - 所有UAVCAN文档的完整概览

### 实现指南
- **[正确实现指南](PX4_UAVCAN_Correct_Implementation_Guide.md)** - UAVCAN实现的最佳实践
- **[自定义协议指南](PX4_UAVCAN_Custom_Protocol_Guide.md)** - 创建自定义UAVCAN协议
- **[自定义集成示例](PX4_UAVCAN_Custom_Integration_Example.md)** - 实际集成示例

### ESC（电子调速器）文档
- **[ESC使用指南](PX4_UAVCAN_ESC_Usage_Guide.md)** - 如何在PX4中使用UAVCAN电调
- **[ESC开发指南](PX4_UAVCAN_ESC_Development_Guide.md)** - 详细的电调开发和实现

### 故障排除
- **[故障排除指南](PX4_UAVCAN_Troubleshooting.md)** - 常见问题和解决方案

## UAVCAN概述

UAVCAN是一个轻量级协议，专为航空航天和机器人应用中的可靠CAN总线通信而设计。它提供：

- **确定性通信** - 基于优先级的消息调度
- **自动节点发现** - 自动配置功能
- **强大的错误处理** - 诊断和恢复机制
- **标准化数据类型** - 常见航空航天应用的标准格式

## PX4中的关键功能

- **电调控制** - 通过UAVCAN协议控制电调
- **传感器集成** - GPS、磁力计、气压计等传感器
- **节点监控** - 节点状态和诊断信息
- **固件更新** - 通过UAVCAN进行固件升级
- **自定义消息** - 支持专用应用的自定义消息

## 快速开始

1. 从[快速参考手册](PX4_UAVCAN_Quick_Reference.md)开始进行基本设置
2. 查看[文档索引](PX4_UAVCAN_Documentation_Index.md)获取全面覆盖
3. 遵循[正确实现指南](PX4_UAVCAN_Correct_Implementation_Guide.md)了解最佳实践
4. 遇到问题时使用[故障排除指南](PX4_UAVCAN_Troubleshooting.md)

## 相关PX4文档

更多PX4文档请参见主[docs目录](../README.md)。