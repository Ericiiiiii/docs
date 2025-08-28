# 工具与实用程序文档

本目录包含PX4开发和维护中使用的各种工具、实用程序和迁移指南。

## 文档列表

### 系统迁移工具
- **[SocketCAN距离传感器迁移](SocketCAN_Distance_Sensor_Migration.md)** - 距离传感器从传统接口到SocketCAN的迁移指南

## 工具分类概览

虽然本目录当前只包含一个文档，但它代表了PX4开发中重要的工具类文档。以下是相关工具的完整分类：

### 1. 系统迁移工具
- **SocketCAN迁移**：现代化CAN总线接口迁移
- **驱动接口升级**：从旧版API到新版API的迁移
- **参数系统迁移**：参数格式和配置的升级

### 2. 开发辅助工具
- **代码生成器**：MAVLink、uORB消息自动生成
- **配置工具**：机架配置、参数批处理工具
- **测试工具**：单元测试、集成测试框架

### 3. 调试分析工具
- **日志分析器**：飞行日志解析和可视化
- **性能分析器**：系统性能监控和优化
- **协议分析器**：通信协议调试工具

### 4. 部署工具
- **固件构建**：自动化构建和打包工具
- **批量配置**：多设备统一配置管理
- **版本管理**：固件版本控制和发布

## SocketCAN距离传感器迁移

### 迁移背景
传统的距离传感器接口存在以下限制：
- **扩展性差**：难以支持多传感器
- **实时性不足**：缺乏优先级机制
- **标准化低**：各厂商接口不统一
- **调试困难**：缺乏标准调试工具

SocketCAN接口提供：
- **标准化协议**：遵循CAN 2.0标准
- **高实时性**：基于优先级的消息调度
- **良好扩展性**：支持多节点网络
- **丰富工具链**：can-utils等标准工具

### 迁移策略

#### 1. 硬件层迁移
```
传统接口              SocketCAN接口
[传感器] → [串口] → [MCU]  vs  [传感器] → [CAN] → [MCU]
                                    ↓
                              [can-utils调试]
```

#### 2. 软件架构迁移
```cpp
// 传统方式
class DistanceSensor : public device::CDev
{
    int read(struct file *filp, char *buffer, size_t buflen, off_t offset) override;
    int ioctl(struct file *filp, int cmd, unsigned long arg) override;
};

// SocketCAN方式  
class DistanceSensorCAN : public UavcanSensorBridgeBase
{
    void distance_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::range_sensor::Measurement> &msg);
};
```

#### 3. 配置参数迁移
```bash
# 传统配置
param set SENS_EN_DIST 1      # 启用距离传感器
param set DIST_SENS_ID 1      # 传感器ID

# SocketCAN配置
param set UAVCAN_ENABLE 3     # 启用UAVCAN
param set UAVCAN_SUB_DIST 1   # 订阅距离传感器消息
```

## 开发工具链

### 1. 构建工具
```bash
# 传统Make构建
make px4_fmu-v5_default

# CMake配置构建
cmake -B build -G Ninja
ninja -C build
```

### 2. 代码生成工具
```bash
# MAVLink消息生成
python Tools/mavlink_generator.py

# uORB消息生成  
python Tools/uorb_rtps/generate_microRTPS_bridge.py

# 参数代码生成
python Tools/px_generate_params.py
```

### 3. 测试工具框架
```bash
# 单元测试
make tests

# SITL集成测试
make px4_sitl_default test_results

# 硬件在环测试
make px4_fmu-v5_test
```

## 系统维护工具

### 1. 日志管理
```bash
# 日志分析工具
python Tools/flight_log_analysis.py logfile.ulg

# 日志格式转换
python Tools/ulog_tools/convert.py logfile.ulg --format csv

# 日志统计分析
python Tools/ulog_tools/ulog_info.py logfile.ulg
```

### 2. 参数管理工具
```bash
# 参数批量处理
python Tools/param_tools/px_process_params.py

# 参数对比分析
python Tools/param_tools/param_compare.py params1.txt params2.txt

# 参数文档生成
python Tools/param_tools/px_generate_param_docs.py
```

### 3. 固件工具
```bash
# 固件打包工具
python Tools/px_mkfw.py --prototype board.prototype --image firmware.bin

# 固件验证工具  
python Tools/px_verify_fw.py firmware.px4

# 版本信息提取
python Tools/px_get_version.py
```

## 性能分析工具

### 1. 实时性能监控
```bash
# 系统负载监控
top                    # CPU使用率
work_queue status      # 工作队列状态  
perf                   # 性能计数器

# 内存使用分析
free                   # 内存使用情况
ps                     # 进程状态
```

### 2. 通信协议分析
```bash
# MAVLink协议分析
mavlink shell
mavlink stream -d /dev/ttyS0 -s ATTITUDE -r 10

# UAVCAN协议分析
uavcan status
candump can0

# uORB消息监控
listener vehicle_attitude
uorb top
```

### 3. 传感器性能分析
```bash
# 传感器数据率监控
listener -r sensor_accel    # 加速度计数据率
listener -r sensor_gyro     # 陀螺仪数据率

# 传感器质量评估
sensors status              # 传感器状态
sensors calibrate           # 传感器校准
```

## 开发最佳实践

### 1. 工具选择原则
- **标准化优先**：选择行业标准工具
- **自动化导向**：减少手动操作
- **可扩展性**：支持项目规模增长
- **社区支持**：活跃的开发社区

### 2. 工具集成策略
- **CI/CD集成**：持续集成和部署
- **版本控制**：工具配置版本化
- **文档同步**：工具使用文档及时更新
- **培训支持**：团队工具使用培训

### 3. 维护策略
- **定期更新**：跟随上游工具更新
- **备份策略**：关键工具和配置备份
- **监控告警**：工具运行状态监控
- **故障恢复**：工具故障快速恢复

## 工具开发指南

### 1. 自定义工具开发
```python
#!/usr/bin/env python3
"""
PX4工具模板
"""
import argparse
import sys

class PX4Tool:
    def __init__(self):
        self.parser = argparse.ArgumentParser(description='PX4 Custom Tool')
        self.setup_arguments()
    
    def setup_arguments(self):
        self.parser.add_argument('--input', help='Input file')
        self.parser.add_argument('--output', help='Output file')
    
    def run(self):
        args = self.parser.parse_args()
        # 实现工具功能
        pass

if __name__ == '__main__':
    tool = PX4Tool()
    tool.run()
```

### 2. 工具集成规范
- **命令行接口**：统一的参数格式
- **错误处理**：清晰的错误信息和退出码
- **日志输出**：结构化的日志格式
- **配置管理**：支持配置文件和环境变量

## 相关文档

- [驱动架构](../drivers/) - 驱动迁移和开发工具
- [通信协议](../communication/) - 协议分析和调试工具
- [仿真调试](../simulation/) - 仿真环境相关工具
- [参数系统](../parameters/) - 参数管理工具

---

本目录文档涵盖PX4开发中的各种工具和实用程序，适合开发人员和维护人员参考使用。