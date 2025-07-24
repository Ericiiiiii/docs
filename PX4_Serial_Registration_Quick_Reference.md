# PX4串口注册快速参考

## 注册流程概览

```
系统启动 → NuttX初始化 → 串口注册 → 设备节点创建 → PX4应用访问
```

## 关键函数调用链

```
arm_serialinit()
    ↓
uart_register("/dev/ttySx", &g_uart_devs[i]->dev)
    ↓
register_driver(path, &g_serialops, 0666, dev)
    ↓
设备节点 /dev/ttySx 创建完成
```

## Fihawk FC-V1映射表

| 硬件UART | 配置宏 | 设备节点 | 功能 | 引脚 |
|---------|--------|---------|------|------|
| USART1 | CONFIG_STM32H7_USART1 | /dev/ttyS0 | GPS1 | PB6/PB7 |
| USART2 | CONFIG_STM32H7_USART2 | /dev/ttyS1 | TELEM1 | PD5/PD6 |
| UART4 | CONFIG_STM32H7_UART4 | /dev/ttyS2 | GPS2 | PD0/PD1 |
| USART6 | CONFIG_STM32H7_USART6 | /dev/ttyS3 | TELEM2 | PG14/PG9 |
| UART7 | CONFIG_STM32H7_UART7 | /dev/ttyS4 | DEBUG | PE8/PF6 |
| UART8 | CONFIG_STM32H7_UART8 | /dev/ttyS5 | RC | PE0/PE1 |

## 调试命令

```bash
# 查看设备节点
ls -la /dev/ttyS*

# 查看串口状态
listener serial_status

# 测试串口
echo "test" > /dev/ttyS1
```

## 注意事项

1. **设备节点编号**: 按g_uart_devs数组顺序分配，不是硬件UART编号
2. **控制台特殊处理**: 控制台UART会同时注册为/dev/console和/dev/ttySx
3. **配置依赖**: 只有在NuttX配置中启用的UART才会被注册
4. **权限设置**: 所有串口设备节点权限为0666

## 相关文档

- [PX4串口注册流程详解](PX4_Serial_Registration_Process.md) - 完整的注册流程分析
- [PX4串口框架概览](PX4_Serial_Framework_Overview.md) - 串口框架整体介绍
- [Fihawk串口配置指南](Fihawk_Serial_Configuration.md) - 硬件特定配置
