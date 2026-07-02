# vins_px4_bridge 使用指南

## 1. 概述

`vins_px4_bridge` 是 VINS-Fusion 与 PX4 飞控之间的 ROS2 桥接节点，负责将 VINS 输出的 `nav_msgs/Odometry` 转换为 PX4 的 `px4_msgs/VehicleOdometry` 消息。

**核心功能**：
- 订阅 `/vins_estimator/odometry`
- 发布 `/fmu/in/vehicle_visual_odometry`
- 自动完成坐标系转换：VINS World → NED，VINS Body (OPENCV) → FRD → NED
- 支持 VO 和 VIO 两种运行模式（默认 VIO）
- 支持可选的 PX4 magnetometer yaw 对齐

**重要前提**：VINS-Fusion 的 World 坐标系不是真正的 ENU，而是一个"重力对齐的初始水平坐标系"。World X = 初始化时 body 的 right，World Y = 初始化时 body 的 forward，World Z = up。只有在初始化时 body right 朝东、forward 朝北，它才与 ENU 重合。

---

## 2. 坐标系约定

### 2.1 VINS Body（RealSense D435i）

RealSense D435i 的 IMU 数据发布在 `camera_imu_optical_frame`：

| 轴 | 方向 |
|---|---|
| X | right（右）|
| Y | down（下）|
| Z | forward（前）|

静止时典型加速度：`linear_acceleration.y ≈ -9.8`

> 必须使用 `body_frame:=OPENCV`，不能用 `FLU`。

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
- `px4_msgs`
- VINS-Fusion 已编译并正常运行（注意 OpenCV 4.8.0 环境，见 `vins_d435i_vo_vio_usage.md` 第 0 节）

### 3.2 源码目录

```
/home/lyx/ros2_vins/src/vins_px4_bridge/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/vins_px4_bridge
├── config/bridge_config.yaml
├── launch/bridge.launch.py
└── vins_px4_bridge/
    ├── __init__.py
    └── bridge_node.py
```

### 3.3 构建

```bash
cd /home/lyx/ros2_vins
source setup_vins_env.sh

colcon build --packages-select vins_px4_bridge --parallel-workers 1
```

> 此包为 `ament_python` 类型，构建很快，无需 `--symlink-install`。

---

## 4. 参数详解

| 参数名 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `mode` | string | `vio` | 运行模式：`vio` 或 `vo`。VO 模式下默认方差更大 |
| `odometry_topic` | string | `/vins_estimator/odometry` | VINS 输出话题 |
| `body_frame` | string | `OPENCV` | Body 坐标系：`OPENCV`（VINS+RealSense）、`FLU`、`FRD` |
| `yaw_alignment_mode` | string | `none` | Yaw 对齐：`none`/`px4_mag`/`manual` |
| `manual_yaw_offset_rad` | float | `0.0` | `manual` 模式下的固定 yaw 偏移 |
| `position_jump_threshold` | float | `0.5` | 位置跳变检测阈值（米）|
| `publish_rate` | float | `100.0` | 最大发布频率（Hz）|
| `default_position_variance` | float[3] | `[0.01,0.01,0.01]` | 默认位置方差 |
| `default_orientation_variance` | float[3] | `[0.01,0.01,0.01]` | 默认姿态方差 |
| `default_velocity_variance` | float[3] | `[0.01,0.01,0.01]` | 默认速度方差 |

### 4.1 `mode` 参数

**`vio`（默认）**：
- 使用 VINS-Fusion VIO 输出
- 默认方差较小 `[0.01, 0.01, 0.01]`

**`vo`**：
- 使用 VINS-Fusion VO 输出
- 自动使用较大方差 `[0.1, 0.1, 0.1]` / `[0.05, 0.05, 0.05]`，因为 VO 无绝对尺度、漂移更大

### 4.2 `body_frame` 参数

| body_frame | Body 约定 | 适用场景 |
|---|---|---|
| `OPENCV`（默认）| X-right, Y-down, Z-forward | VINS + RealSense D435i |
| `FLU` | X-forward, Y-left, Z-up | FAST-LIO / Livox |
| `FRD` | X-forward, Y-right, Z-down | 已经是 FRD |

> ⚠️ **常见错误**：对 VINS+RealSense 使用 `FLU` 会导致 yaw 偏差约 90°。

### 4.3 `yaw_alignment_mode` 参数

**`none`**：不修正 yaw，直接把虚拟 NED 发给 PX4。

**`px4_mag`**：订阅 `/fmu/out/vehicle_attitude`，取 PX4 magnetometer yaw，锁定并应用偏移。

**`manual`**：使用固定的 `manual_yaw_offset_rad`。

---

## 5. 启动方式

### 5.1 直接运行（默认 VIO 模式）

```bash
cd /home/lyx/ros2_vins
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run vins_px4_bridge bridge_node
```

### 5.2 VO 模式

```bash
ros2 run vins_px4_bridge bridge_node --ros-args -p mode:=vo
```

### 5.3 带参数运行

```bash
ros2 run vins_px4_bridge bridge_node \
  --ros-args \
  -p mode:=vio \
  -p body_frame:=OPENCV \
  -p yaw_alignment_mode:=px4_mag \
  -p position_jump_threshold:=0.5
```

### 5.4 通过 launch 文件启动

```bash
ros2 launch vins_px4_bridge bridge.launch.py
```

自定义配置：
```bash
ros2 launch vins_px4_bridge bridge.launch.py \
  config:=/home/lyx/ros2_vins/src/vins_px4_bridge/config/bridge_config.yaml
```

---

## 6. 完整使用流程

### 步骤 1：启动 VINS（VIO 或 VO）

先 source 环境脚本（设置 OpenCV 4.8.0 路径）：
```bash
cd /home/lyx/ros2_vins
source setup_vins_env.sh
```

VIO（推荐）：
```bash
pkill -9 -f realsense2_camera_node
sleep 2

ros2 launch vins realsense_d435i_vins.launch.py \
  config_path:=/home/lyx/ros2_vins/src/vins-fusion-jetson-humble/config/realsense_d435i_vio/realsense_stereo_imu_config.yaml
```

VO：
```bash
pkill -9 -f realsense2_camera_node
sleep 2

ros2 launch vins realsense_d435i_vins.launch.py \
  config_path:=/home/lyx/ros2_vins/src/vins-fusion-jetson-humble/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

### 步骤 2：确认 VINS 正常输出

```bash
ros2 topic hz /vins_estimator/odometry
ros2 topic echo /vins_estimator/odometry --once
```

### 步骤 3：启动 uXRCE-DDS Agent

```bash
# USB 连接飞控
MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600
```

或 UDP：
```bash
MicroXRCEAgent udp4 -p 8888
```

### 步骤 4：启动 bridge

```bash
cd /home/lyx/ros2_vins
source setup_vins_env.sh

ros2 run vins_px4_bridge bridge_node
```

### 步骤 5：验证 PX4 收到数据

```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry --once
```

检查：
- `pose_frame == 1`（POSE_FRAME_NED）
- `timestamp` 是 Unix 微秒
- `q` 四元数归一化

### 步骤 6：地面站查看

- 检查 `EKF2_EV_CTRL` 参数，启用 visual odometry 融合
- 推荐设置 `EKF2_EV_DELAY = 60`（毫秒）
- `EKF2_HGT_REF` 可设为 3（VISION）

---

## 7. 验证方法

### 7.1 位置验证

手持相机分别向**右 / 前 / 上**移动，观察 `/vins_estimator/odometry`：
- 右移 → x 增大
- 前移 → y 增大
- 上移 → z 增大

对应 PX4 `/fmu/in/vehicle_visual_odometry`：
- 右移（World X 增）→ NED Y（East）增大
- 前移（World Y 增）→ NED X（North）增大
- 上移（World Z 增）→ NED Z（Down）减小

### 7.2 姿态验证

初始化时让相机：
- **right（X）指向正东**
- **forward（Z）指向正北**

此时 VINS world ≈ ENU，bridge 输出的 `q_frd_to_ned` 应接近 identity，即 yaw ≈ 0°。

```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry --once
```

---

## 8. 常见问题

### Q1: PX4 没有收到数据

1. `ros2 topic list | grep vehicle_visual_odometry` 确认话题存在
2. `ros2 topic hz /fmu/in/vehicle_visual_odometry` 确认有数据流
3. 检查 uXRCE-Agent 是否正常运行
4. 检查 PX4 参数 `EKF2_EV_CTRL` 是否启用了外部视觉

### Q2: 地面站看到的航向偏了 90°

原因：使用了错误的 `body_frame`。VINS+RealSense 必须用 `OPENCV`。

解决：
```bash
ros2 run vins_px4_bridge bridge_node --ros-args -p body_frame:=OPENCV
```

### Q3: 位置跳变后 PX4 姿态异常

VINS 发生 track lost 或 reset 后，位置会跳变。bridge 会自动：
1. 检测跳变（超过 `position_jump_threshold`）
2. `reset_counter` 加 1
3. 清除 yaw offset，下次收到 PX4 attitude 时重新锁定

### Q4: EKF2 不采纳 visual odometry

检查 PX4 参数：
- `EKF2_EV_CTRL`：启用 bit 0（位置）和 bit 1（速度）
- `EKF2_EV_DELAY`：60（毫秒）
- `EKF2_HGT_REF`：3（VISION）

---

## 9. 参考

- `src/vins_px4_bridge/vins_px4_bridge/bridge_node.py`
- `src/vins-fusion-jetson-humble/vins_config_reference/vins_px4_bridge_usage.md`
- `src/vins-fusion-jetson-humble/vins_config_reference/bridge_coordinate_transform.md`
- `src/vins-fusion-jetson-humble/vins_config_reference/bridge_coordinate_transform_fields.md`
- PX4 external position estimation: https://docs.px4.io/main/en/ros/external_position_estimation.html
