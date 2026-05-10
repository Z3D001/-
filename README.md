# 单线雷达拼接节点 (GL-K20)

这个项目实现了基于GL-K20单线激光雷达的三维点云拼接功能，并包含基于强度值的颜色映射。

## 功能描述

通过将单线雷达的二维扫描数据与机器人里程计信息结合，生成三维彩色点云数据。点云颜色根据激光强度值自动映射，提供更好的可视化效果。

### 核心组件

1. **RadarConnect** - 雷达通信模块
   - TCP连接管理（默认：192.168.2.83:1112）
   - 登录认证、心跳保持连接
   - 自动重连机制
   - 解析雷达二维扫描数据

2. **SingleRadarSplicingNode** - 拼接节点
   - 直接连接雷达获取二维扫描数据
   - 订阅 `/odom` 里程计话题
   - 根据里程计X轴位移计算Z轴高度
   - 发布拼接后的三维点云到 `/SingleRadar/pointcloud`

## 拼接方案

```
1. 通过ROS 2话题控制拼接开始/停止
2. 节点累积指定时间段内的点云数据
3. 根据里程计位移将二维数据转换为三维点云
4. 发布拼接后的三维点云
```

## 使用方法

### 1. 启动节点

```bash
ros2 launch driver.py
```

### 2. 控制拼接

通过ROS 2话题控制（发送浮点数控制指令）：

```bash
# 开始拼接（0.5秒周期，推荐设置）
ros2 topic pub /SingleRadar/Start_or_Stop std_msgs/msg/Float32 "{data: 0.5}" --once

# 开始拼接（1秒周期）
ros2 topic pub /SingleRadar/Start_or_Stop std_msgs/msg/Float32 "{data: 1.0}" --once

# 开始拼接（2秒周期）
ros2 topic pub /SingleRadar/Start_or_Stop std_msgs/msg/Float32 "{data: 2.0}" --once

# 开始拼接（5秒周期）
ros2 topic pub /SingleRadar/Start_or_Stop std_msgs/msg/Float32 "{data: 5.0}" --once

# 停止拼接
ros2 topic pub /SingleRadar/Start_or_Stop std_msgs/msg/Float32 "{data: 0.0}" --once
```

**控制指令说明：**
- `0.0` 或负数 = 停止拼接
- `正数` = 开始拼接（使用指定值作为周期秒数，支持小数）

**特殊功能：**
- ✅ 支持重复开始-停止-开始循环
- ✅ 支持动态更新周期（拼接过程中发送新周期会立即生效）
- ✅ 持续监听控制信号，直到手动关闭节点

**不同周期的预计点数：**
- `0.5秒` ≈ 108,000点（约30圈）
- `1.0秒` ≈ 216,000点（约60圈）
- `2.0秒` ≈ 432,000点（约120圈）

## 🎨 颜色映射说明

点云使用**热力图颜色映射**方案，根据激光强度值自动着色：

| 强度范围 | 颜色效果 | 说明 |
|---------|---------|------|
| 低强度 (0-127) | 🔵 蓝色→绿色 | 较弱的回波信号 |
| 中强度 (127-191) | 🟢 绿色 | 中等回波信号 |
| 高强度 (191-255) | 🔴 红色 | 强回波信号 |

**颜色映射的优势：**
- ✅ 直观显示目标反射强度
- ✅ 便于识别不同材质目标
- ✅ 增强三维可视化效果
- ✅ 保持原始强度数据不变
- `0.5秒` ≈ 108,000点（约30圈）
- `1.0秒` ≈ 216,000点（约60圈）
- `2.0秒` ≈ 432,000点（约120圈）

### 3. 高级使用示例

```bash
# 启动节点（持续运行）
ros2 launch driver.py

# 第一次开始拼接（0.5秒周期）
ros2 topic pub /SingleRadar/Start_or_Stop std_msgs/msg/Float32 "{data: 0.5}" --once

# 运行10秒后，改为1秒周期
ros2 topic pub /SingleRadar/Start_or_Stop std_msgs/msg/Float32 "{data: 1.0}" --once

# 再运行5秒后，暂时停止
ros2 topic pub /SingleRadar/Start_or_Stop std_msgs/msg/Float32 "{data: 0.0}" --once

# 重新开始（0.3秒周期）
ros2 topic pub /SingleRadar/Start_or_Stop std_msgs/msg/Float32 "{data: 0.3}" --once

# 最后停止
ros2 topic pub /SingleRadar/Start_or_Stop std_msgs/msg/Float32 "{data: 0.0}" --once
```

### 4. 订阅点云数据

```bash
ros2 topic echo /SingleRadar/pointcloud
```

## 配置参数

在 `driver.py` 中可以配置以下参数：

- `lidar_ip`: 雷达IP地址（默认：192.168.2.83）
- `lidar_port`: 雷达端口（默认：1112）
- `use_sim_time`: 是否使用仿真时间（默认：False）

## 话题

### 发布的话题

- `/SingleRadar/pointcloud` (sensor_msgs/msg/PointCloud2): 拼接后的三维彩色点云
  - 字段：`x, y, z, intensity, r, g, b`
  - 颜色：基于强度值自动映射的热力图

### 订阅的话题

- `/odom` (nav_msgs/msg/Odometry): 里程计数据
- `/SingleRadar/Start_or_Stop` (std_msgs/msg/Float32): 控制指令
  - data: 0.0或负数=停止, 正数=开始并设置周期(秒)

## 技术特点

- **坐标系转换**: 二维数据(X,Y)与里程计位移(Z)结合
- **单位转换**: 米与毫米之间的转换确保精度
- **线程安全**: 使用mutex保护共享数据
- **ROS 2标准**: 完全符合ROS 2标准接口
- **彩色点云**: 基于激光强度值自动映射颜色（热力图方案）

## 依赖项

- rclcpp
- sensor_msgs
- nav_msgs
- std_msgs

## 构建方法

```bash
colcon build --packages-select sigle_radar
source install/setup.bash
```

## 应用场景

主要用于移动机器人三维建图，通过单线雷达的低成本方案实现三维环境扫描，适用于：
- 室内导航
- 障碍物检测
- 环境建模