# PX4数据重放功能演示指南

## 概述

PX4的System-wide Replay功能可以读取SD卡记录的ULog文件，将传感器数据重新输入到算法中重新运行，生成新的日志文件用于对比分析。这对于参数调优、算法验证和故障分析非常有用。

## 前置条件

- PX4-Autopilot源码
- 已编译的PX4 SITL环境
- Python环境（用于日志分析工具）

## 第一步：生成示例日志文件

首先我们需要一个ULog文件用于演示。通过运行仿真来生成：

```bash
# 进入PX4根目录
cd /path/to/PX4-Autopilot

# 编译SITL目标
make px4_sitl_default

# 运行30秒仿真生成日志文件
timeout 30s make px4_sitl_default jmavsim || true
```

查找生成的日志文件：
```bash
find build/px4_sitl_default/rootfs -name "*.ulg" -type f | head -3
```

## 第二步：设置重放环境

设置重放日志文件的环境变量：
```bash
# 设置要重放的日志文件路径（替换为实际路径）
export replay=$(pwd)/build/px4_sitl_default/rootfs/log/2025-06-14/06_05_10.ulg
echo "设置重放文件: $replay"
```

## 第三步：构建重放目标

当设置了`replay`环境变量后，PX4会自动创建专门的重放构建目录：

```bash
# 构建重放目标（会自动创建 build/px4_sitl_default_replay 目录）
make px4_sitl_default
```

## 第四步：配置ORB发布规则

创建ORB发布规则文件，控制哪些模块可以发布特定的消息：

```bash
# 创建ORB发布规则文件
cat > build/px4_sitl_default_replay/rootfs/orb_publisher.rules << EOF
restrict_topics: sensor_combined, vehicle_gps_position, vehicle_land_detected
module: replay
ignore_others: true
EOF
```

这个配置确保只有replay模块可以发布传感器数据，其他模块的发布会被忽略。

## 第五步：执行基本重放

运行重放功能：

```bash
# 设置环境变量并运行重放
export replay=$(pwd)/build/px4_sitl_default/rootfs/log/2025-06-14/06_05_10.ulg
cd build/px4_sitl_default_replay
./bin/px4 ../../../ROMFS/px4fmu_common -s etc/init.d-posix/rcS
```

## 第六步：EKF2专用重放模式

EKF2重放模式是一个更高效的专门用于EKF2算法测试的模式：

```bash
# 设置EKF2重放模式
export replay_mode=ekf2
export replay=$(pwd)/build/px4_sitl_default/rootfs/log/2025-06-27/01_34_45.ulg

# 运行EKF2重放
make px4_sitl none
```

## 第七步：参数覆盖功能

### 安装日志分析工具
```bash
pip install --user pyulog
```

### 提取原始日志参数
```bash
# 查看原始日志中的EKF2参数
ulog_params -i build/px4_sitl_default/rootfs/log/2025-06-27/01_34_45.ulg | grep -e '^EKF2' | head -10
```

### 创建参数覆盖文件

**固定参数覆盖** (`replay_params.txt`)：
```bash
cat > build/px4_sitl_default_replay/rootfs/replay_params.txt << EOF
# 演示参数覆盖 - 调整EKF2噪声参数
EKF2_ACC_NOISE 0.5
EKF2_GYR_NOISE 0.02
EKF2_MAG_NOISE 0.08
EOF
```

**动态参数覆盖** (`replay_params_dynamic.txt`)：
```bash
cat > build/px4_sitl_default_replay/rootfs/replay_params_dynamic.txt << EOF
# 在重放过程中动态改变参数
EKF2_RNG_NOISE 0.15 23.4
EKF2_RNG_NOISE 0.05 56.7
EKF2_RNG_DELAY 4.5 30.0
EOF
```

## 第八步：分析重放结果

### 查看生成的重放日志
```bash
# 查看重放日志文件
ls -lh build/px4_sitl_default_replay/log/*/
```

### 对比原始日志和重放日志
```bash
# 原始日志信息
echo "=== 原始日志文件信息 ==="
ulog_info build/px4_sitl_default/rootfs/log/2025-06-14/06_05_10.ulg | head -15

# 重放日志信息
echo "=== 重放日志文件信息 ==="
ulog_info build/px4_sitl_default_replay/log/*/06_*_replayed.ulg | head -15
```

## 重放成功的标志

重放成功时会看到类似以下输出：
```
INFO  [replay] using replay log file: /path/to/log.ulg
INFO  [replay] Applying params from ULog file...
INFO  [replay] Replay in progress...
INFO  [replay] Replay done (published 54335 msgs, 22.251 s)
INFO  [logger] Opened full log file: ./log/2025-06-14/06_07_54_replayed.ulg
```

## 清理环境

重放完成后清理环境变量：
```bash
unset replay
unset replay_mode
```

## 应用场景

1. **参数调优**: 使用相同传感器数据测试不同参数设置
2. **算法对比**: 比较不同估计器的性能
3. **故障分析**: 重现飞行中的问题进行调试
4. **算法验证**: 验证新算法在真实数据上的表现

## 注意事项

- 重放文件必须是有效的ULog格式
- 确保所有相关的传感器数据都被记录在原始日志中
- 重放过程中的时间戳错误通常表示数据不完整
- 重放日志文件会自动添加`_replayed`后缀

## 故障排除

1. **找不到日志文件**: 检查`replay`环境变量路径是否正确
2. **权限错误**: 确保对日志文件和构建目录有读写权限
3. **时间戳错误**: 原始日志可能有数据丢失，尝试使用完整的日志文件
4. **模块启动失败**: 检查ORB发布规则配置是否正确

## 高级用法

### 批量重放脚本
创建脚本自动化重放多个日志文件：

```bash
#!/bin/bash
# batch_replay.sh - 批量重放脚本

LOG_DIR="path/to/logs"
PARAM_SETS=("default" "tuned" "aggressive")

for log_file in $LOG_DIR/*.ulg; do
    for param_set in "${PARAM_SETS[@]}"; do
        echo "重放 $log_file 使用参数集 $param_set"
        export replay="$log_file"

        # 复制对应的参数文件
        cp "params_${param_set}.txt" build/px4_sitl_default_replay/rootfs/replay_params.txt

        # 执行重放
        cd build/px4_sitl_default_replay
        ./bin/px4 ../../../ROMFS/px4fmu_common -s etc/init.d-posix/rcS > "replay_${param_set}_$(basename $log_file).log" 2>&1
        cd ../..

        # 移动结果日志
        mv build/px4_sitl_default_replay/log/*/*.ulg "results/$(basename $log_file)_${param_set}_replayed.ulg"
    done
done
```

### 重放结果分析
使用Python脚本分析重放结果：

```python
#!/usr/bin/env python3
# analyze_replay.py - 重放结果分析脚本

import numpy as np
from pyulog import ULog

def compare_logs(original_log, replayed_log):
    """对比原始日志和重放日志"""

    # 加载日志文件
    ulog_orig = ULog(original_log)
    ulog_replay = ULog(replayed_log)

    # 提取关键数据
    orig_attitude = ulog_orig.get_dataset('vehicle_attitude')
    replay_attitude = ulog_replay.get_dataset('vehicle_attitude')

    # 计算差异
    attitude_diff = np.abs(orig_attitude.data['q[0]'] - replay_attitude.data['q[0]'])

    print(f"姿态估计最大差异: {np.max(attitude_diff):.6f}")
    print(f"姿态估计平均差异: {np.mean(attitude_diff):.6f}")

if __name__ == "__main__":
    compare_logs("original.ulg", "replayed.ulg")
```

### 实时重放监控
监控重放过程的脚本：

```bash
#!/bin/bash
# monitor_replay.sh - 重放监控脚本

REPLAY_LOG="build/px4_sitl_default_replay/log"

echo "开始监控重放过程..."
while true; do
    if [ -f "$REPLAY_LOG"/*/*.ulg ]; then
        SIZE=$(du -h "$REPLAY_LOG"/*/*.ulg | cut -f1)
        echo "$(date): 重放日志大小: $SIZE"
    fi
    sleep 5
done
```

## 最佳实践

### 1. 日志记录建议
- 使用 `SDLOG_MODE = 1` 从启动开始记录
- 确保记录所有相关传感器数据
- 避免日志记录中断和数据丢失

### 2. 参数调优策略
- 先使用默认参数重放验证基线
- 逐步调整单个参数观察影响
- 记录每次调整的结果用于对比

### 3. 性能优化
- EKF2模式比通用重放模式更快
- 使用SSD存储提高I/O性能
- 关闭不必要的日志记录减少开销

### 4. 数据管理
- 建立清晰的文件命名规范
- 定期备份重要的重放结果
- 使用版本控制管理参数配置

## 扩展应用

### 1. 自动化测试集成
将重放功能集成到CI/CD流程中：

```yaml
# .github/workflows/replay_test.yml
name: Replay Tests
on: [push, pull_request]

jobs:
  replay-test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Setup PX4 Environment
        run: |
          make px4_sitl_default
      - name: Run Replay Tests
        run: |
          ./scripts/run_replay_tests.sh
```

### 2. 参数优化工具
结合优化算法自动寻找最佳参数：

```python
# parameter_optimization.py
from scipy.optimize import minimize
import subprocess

def objective_function(params):
    """目标函数：运行重放并返回性能指标"""
    # 设置参数
    with open('replay_params.txt', 'w') as f:
        f.write(f'EKF2_ACC_NOISE {params[0]}\n')
        f.write(f'EKF2_GYR_NOISE {params[1]}\n')

    # 运行重放
    result = subprocess.run(['./run_replay.sh'], capture_output=True)

    # 计算性能指标（如估计误差）
    return calculate_estimation_error()

# 运行优化
result = minimize(objective_function, x0=[0.35, 0.015], method='Nelder-Mead')
```

---

通过这个完整的演示指南，您可以：
1. **快速上手** PX4数据重放功能
2. **深入理解** 重放机制和配置选项
3. **实际应用** 到参数调优和算法验证
4. **扩展使用** 到自动化测试和优化流程

这正是您需要的"把采集的数据再灌入算法接口，重新再跑一遍算法"的完整解决方案！
