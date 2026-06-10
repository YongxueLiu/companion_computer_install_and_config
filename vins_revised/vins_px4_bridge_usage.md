# vins_px4_bridge 使用指南

## 1. 概述

`vins_px4_bridge` 是 VINS-Fusion 与 PX4 飞控之间的 ROS2 桥接节点，负责将 VINS 输出的 `nav_msgs/Odometry` 转换为 PX4 的 `px4_msgs/VehicleOdometry` 消息。

**核心功能**：
- 订阅 `/vins_estimator/odometry`
- 发布 `/fmu/in/vehicle_visual_odometry`
- 自动完成坐标系转换：位置/速度从 VINS World 转到 NED，姿态从 VINS Body 转到 FRD→NED
- 支持可选的 magnetometer yaw 对齐

**重要前提**：VINS-Fusion 的 World 坐标系不是真正的 ENU，而是一个"重力对齐的初始水平坐标系"。World X = 初始化时 body 的 right，World Y = 初始化时 body 的 forward，World Z = up。只有在初始化时 body right 朝东、forward 朝北，它才与 ENU 重合。

---

## 2. 坐标系约定

### 2.1 VINS Body（RealSense D435i）

RealSense D435i 的 IMU 默认发布在 `camera_imu_optical_frame`：

| 轴 | 方向 |
|---|---|
| X | right（右）|
| Y | down（下）|
| Z | forward（前）|

静止时典型加速度：`linear_acceleration.y ≈ -9.8`

### 2.2 VINS World

| 轴 | 方向 | 实测验证 |
|---|---|---|
| X | 初始化时 body 的 right 在水平面投影 | 右移 → x 增大 |
| Y | 初始化时 body 的 forward 在水平面投影 | 前移 → y 增大 |
| Z | up（重力反方向）| 上移 → z 增大 |

### 2.3 PX4 坐标系

| 坐标系 | 轴定义 |
|---|---|
| FRD（机体）| X=Forward, Y=Right, Z=Down |
| NED（导航）| X=North, Y=East, Z=Down |

---

## 3. 安装与构建

### 3.1 依赖

- ROS2 Humble
- `px4_msgs`（PX4 uXRCE-DDS 接口消息包）
- VINS-Fusion 已编译并正常运行

### 3.2 构建

```bash
cd /home/lingzhilab/vins
colcon build --parallel-workers 1 --packages-select vins_px4_bridge
```

> ⚠️ **注意**：此包为 `ament_python` 类型，**不要**加 `--symlink-install`，否则 setuptools 会报错。

### 3.3 源码目录结构

```
src/vins_px4_bridge/
├── package.xml
├── setup.py
├── resource/vins_px4_bridge
└── vins_px4_bridge/
    ├── __init__.py
    └── bridge_node.py      # 主节点
```

---

## 4. 参数详解

| 参数名 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `odometry_topic` | string | `/vins_estimator/odometry` | VINS 输出话题 |
| `body_frame` | string | `OPENCV` | Body 坐标系约定。`OPENCV`=optical frame(X-right,Y-down,Z-forward)；`FLU`=Forward-Left-Up；`FRD`=直接透传 |
| `yaw_alignment_mode` | string | `none` | Yaw 对齐模式：`none`/`px4_mag`/`manual` |
| `manual_yaw_offset_rad` | float | `0.0` | `manual` 模式下的固定 yaw 偏移（弧度） |
| `position_jump_threshold` | float | `0.5` | 位置跳变检测阈值（米） |
| `default_position_variance` | float[3] | `[0.01,0.01,0.01]` | 默认位置方差 |
| `default_orientation_variance` | float[3] | `[0.01,0.01,0.01]` | 默认姿态方差 |
| `default_velocity_variance` | float[3] | `[0.01,0.01,0.01]` | 默认速度方差 |

### 4.1 `body_frame` 参数

这是最重要的参数，直接决定姿态转换是否正确。

**`OPENCV`（默认，RealSense D435i）**：
- VINS body = optical frame（X-right, Y-down, Z-forward）
- 内部使用 `q_frd_to_opencv = [-0.5, 0.5, 0.5, 0.5]`
- 转换链：`FRD → optical → ENU → NED`

**`FLU`（Livox/FAST-LIO）**：
- Body = FLU（X-forward, Y-left, Z-up）
- 内部使用 `q_frd_to_flu = [0, 1, 0, 0]`
- 转换链：`FRD → FLU → ENU → NED`

**`FRD`**：
- Body 已经是 FRD，不做额外 body 转换
- 转换链：`FRD → ENU → NED`

> ⚠️ **常见错误**：对 VINS+RealSense 使用 `FLU` 会导致 yaw 偏差约 90°。

### 4.2 `yaw_alignment_mode` 参数

**`none`**：
- 不修正 yaw，直接把虚拟 NED 发给 PX4
- 适用于：纯视觉定位、室内无磁环境、或者 world 已经通过初始化对准了北

**`px4_mag`**：
- 订阅 `/fmu/out/vehicle_attitude`，取 PX4 magnetometer 的 yaw
- 在第一次收到有效姿态时锁定 `δ_yaw = yaw_px4 - yaw_virtual`
- 后续每帧都应用这个偏移
- 当 VINS 发生位置跳变（reset）时，offset 自动清除并重新锁定

**`manual`**：
- 使用固定的 `manual_yaw_offset_rad` 作为偏移量
- 适用于：已知初始化方向与正北的固定偏差

---

## 5. 启动方式

### 5.1 直接运行

```bash
source /home/lingzhilab/vins/install/setup.bash
ros2 run vins_px4_bridge bridge_node
```

### 5.2 带参数运行

```bash
ros2 run vins_px4_bridge bridge_node \
  --ros-args \
  -p body_frame:=OPENCV \
  -p yaw_alignment_mode:=px4_mag \
  -p position_jump_threshold:=0.5
```

### 5.3 在 launch 文件中启动

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vins_px4_bridge',
            executable='bridge_node',
            name='vins_px4_bridge',
            parameters=[{
                'odometry_topic': '/vins_estimator/odometry',
                'body_frame': 'OPENCV',
                'yaw_alignment_mode': 'none',
                'position_jump_threshold': 0.5,
                'default_position_variance': [0.01, 0.01, 0.01],
                'default_orientation_variance': [0.01, 0.01, 0.01],
                'default_velocity_variance': [0.01, 0.01, 0.01],
            }],
            output='screen',
        ),
    ])
```

---

## 6. 完整使用流程

### 步骤 1：启动 RealSense 相机

```bash
# 先清理可能残留的 realsense 进程
pkill -9 -f realsense2_camera_node

# 启动
ros2 launch vins realsense_d435i_vins.launch.py
```

### 步骤 2：启动 VINS-Fusion

RealSense launch 文件内部已经包含 VINS 节点，如果单独启动：

```bash
ros2 run vins vins_node \
  /home/lingzhilab/vins/src/VINS-Fusion-ROS2/config/realsense_d435i_vio/realsense_stereo_imu_config.yaml
```

### 步骤 3：确认 VINS 正常输出

```bash
# 检查 odometry 话题
ros2 topic hz /vins_estimator/odometry

# 查看一帧数据
ros2 topic echo /vins_estimator/odometry --once
```

### 步骤 4：启动 uXRCE-DDS Agent

```bash
# 在飞控已连接的情况下
MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600
```

或 if using UDP:
```bash
MicroXRCEAgent udp4 -p 8888
```

### 步骤 5：启动 bridge

```bash
source /home/lingzhilab/vins/install/setup.bash
ros2 run vins_px4_bridge bridge_node
```

### 步骤 6：验证 PX4 收到数据

```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry --once
```

检查：
- `pose_frame == 1`（POSE_FRAME_NED）
- `timestamp` 是 Unix 微秒
- `q` 四元数归一化

### 步骤 7：地面站查看

在 QGroundControl 或 MAVSDK 中查看：
- `vehicle_visual_odometry` 是否被 EKF2 采纳
- 检查 `EKF2_EV_CTRL` 参数，确保 visual odometry 融合已启用
- 推荐设置 `EKF2_EV_DELAY = 60`（毫秒）

---

## 7. 验证方法

### 7.1 位置验证

手持相机分别向**右 / 前 / 上**移动，观察 `/vins_estimator/odometry`：
- 右移 → x 增大
- 前移 → y 增大
- 上移 → z 增大

对应 PX4 `/fmu/in/vehicle_visual_odometry`：
- 右移（ENU X 增）→ NED Y（East）增大
- 前移（ENU Y 增）→ NED X（North）增大
- 上移（ENU Z 增）→ NED Z（Down）减小

### 7.2 姿态验证

初始化时让相机：
- **right（X）指向正东**
- **forward（Z）指向正北**

此时 VINS world ≈ ENU，bridge 输出的 `q_frd_to_ned` 应接近 identity，即 yaw ≈ 0°。

```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry --once
```

计算 yaw：`atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))`，结果应接近 0°。

### 7.3 body frame 验证

从 odometry 四元数反推 body 各轴在 world 中的方向：

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

q = [w, x, y, z]  # from odometry
R_mat = R.from_quat([x, y, z, w]).as_matrix()
print("Body X in world:", R_mat[:, 0])  # 应 ≈ world right
print("Body Y in world:", R_mat[:, 1])  # 应 ≈ world down
print("Body Z in world:", R_mat[:, 2])  # 应 ≈ world forward
```

---

## 8. 时间戳处理

**不要在 bridge 中做任何时间偏移修正。**

PX4 uXRCE-DDS client 在反序列化时会自动转换：
```cpp
topic.timestamp = msg.timestamp - session->time_offset;
```

bridge 直接发送原始 Unix 微秒：
```python
vo.timestamp = int(msg.header.stamp.sec * 1_000_000
                   + msg.header.stamp.nanosec // 1000)
```

如果手动减去 offset，会造成双重转换，导致时间戳错误。

---

## 9. 常见问题

### Q1: 启动 bridge 后 PX4 没有收到数据

检查步骤：
1. `ros2 topic list | grep vehicle_visual_odometry` 确认话题存在
2. `ros2 topic hz /fmu/in/vehicle_visual_odometry` 确认有数据流
3. 检查 uXRCE-Agent 是否正常运行
4. 检查 PX4 参数 `EKF2_EV_CTRL` 是否启用了外部视觉

### Q2: 地面站看到的航向偏了 90°

原因：使用了错误的 `body_frame`。VINS+RealSense 必须用 `OPENCV`，不能用 `FLU`。

解决：
```bash
ros2 run vins_px4_bridge bridge_node --ros-args -p body_frame:=OPENCV
```

### Q3: 位置跳变后 PX4 姿态异常

VINS 发生 track lost 或 reset 后，位置会跳变。bridge 会自动：
1. 检测跳变（超过 `position_jump_threshold`）
2. `reset_counter` 加 1
3. 清除 yaw offset，下次收到 PX4 attitude 时重新锁定

这是预期行为。如果频繁跳变，需要改善 VINS 前端跟踪质量（光照、纹理、运动速度）。

### Q4: EKF2 不采纳 visual odometry

检查 PX4 参数：
- `EKF2_EV_CTRL`：启用 bit 0（位置）和 bit 1（速度）
- `EKF2_EV_DELAY`：设置为 60（毫秒）
- `EKF2_EV_GATE`：可适当放宽
- `EKF2_HGT_REF`：设为 3（VISION）

### Q5: 为什么 angular_velocity 是 NaN

VINS-Fusion 的 `nav_msgs/Odometry` 中 `twist.twist.angular` 字段为空，bridge 填入 NaN。PX4 EKF2 在 angular_velocity 为 NaN 时会忽略该字段，只使用位置和姿态。

---

## 10. 参考文档

- `tutorial/bridge_coordinate_transform.md` — 坐标转换数学推导
- `tutorial/VINS_World_Coordinate_System.md` — VINS World 定义
- `tutorial/px4_timesync_and_latency.md` — 时间同步与延迟
- PX4 uXRCE-DDS 时间转换源码：`PX4-Autopilot/src/modules/uxrce_dds_client/utilities/conversions.cpp`
