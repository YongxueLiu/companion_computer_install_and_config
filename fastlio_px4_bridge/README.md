# fastlio_px4_bridge

将 FAST-LIO 的 SLAM 位姿桥接到 PX4 飞控的外部视觉（External Vision）输入。

## 依赖

- ROS 2 Humble
- `px4_msgs`（已软链接到 `~/ws_livox/src/px4_msgs`）
- `nav_msgs`, `sensor_msgs`

## 编译

```bash
cd ~/ws_livox
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs fastlio_px4_bridge --symlink-install
```

> 注意：`px4_msgs` 包含约 240 个消息定义，首次编译可能需要 10~15 分钟。

## 运行

### 1. 启动 FAST-LIO（已在运行）

确保 `/Odometry` 和 `/livox/imu` 话题已发布。

### 2. 启动桥接节点

```bash
cd ~/ws_livox
source install/setup.bash
ros2 launch fastlio_px4_bridge bridge.launch.py
```

### 3. 验证 PX4 融合状态

```bash
# 查看桥接节点输出
ros2 topic echo /fmu/in/vehicle_visual_odometry

# 查看 EKF2 是否已融合外部视觉
ros2 topic echo /fmu/out/estimator_status_flags | grep cs_ev
```

## 参数说明

参数文件位于 `config/bridge_config.yaml`：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `world_to_ned_q` | `[0, 1, 0, 0]` | FLU world → NED 的四元数（w,x,y,z）。默认绕 X 轴 180° |
| `imu_flu_to_frd` | `true` | 是否将 IMU 角速度从 FLU 转换到 FRD |
| `position_jump_threshold` | `0.5` | 位置跳变检测阈值（米） |
| `default_position_variance` | `[0.01, 0.01, 0.01]` | 默认位置方差 |
| `default_orientation_variance` | `[0.01, 0.01, 0.01]` | 默认姿态方差 |
| `publish_rate` | `100.0` | 输出频率限制（Hz） |

## PX4 参数配置（QGroundControl）

| 参数 | 推荐值 |
|---|---|
| `EKF2_EV_CTRL` | `15` |
| `EKF2_HGT_REF` | `3` (Vision) |
| `EKF2_GPS_CTRL` | `0`（室内）|
| `EKF2_EV_DELAY` | `20` ~ `50` |

## 安全验证清单

1. **地面手持验证**：推动无人机，确认 QGC 本地位置与真实运动方向一致。
2. **六自由度方向**：分别验证 X/Y/Z 平移和 Roll/Pitch/Yaw 旋转方向。
3. **故障保护**：遮挡 LiDAR 2~3 秒，确认飞控进入安全模式（Land / Hold）。
4. **首次飞行**：先使用 `Stabilized` 或 `Altitude` 模式起飞，确认稳定后再切 `Position` 模式。
