# FAST-LIVO2 本机使用说明（Mid-360 + RealSense D435i）

> 适配环境：ROS2 Humble / Ubuntu 22.04  
> 硬件：Livox Mid-360 + RealSense D435i（仅使用左红外 + IMU）  
> 工作空间：`~/ros2_fast_livo2`  
> 有效功能包：`src/FAST-LIVO2-ROS2`（`src/FAST-LIVO2` 已放置 `COLCON_IGNORE`，避免同名冲突）

---

## 1. 环境依赖

本机已经配置好的依赖：

- ROS2 Humble：`/opt/ros/humble/setup.bash`
- Livox ROS Driver2：`~/ros2_fast_lio2/install`（提供 `livox_ros_driver2`）
- RealSense ROS Driver：`~/ros2_vins`（提供 `realsense2_camera`，并强制使用 OpenCV 4.8.0）
- 本地 Sophus 1.22.10：`~/.local`
- OpenCV 4.8.0：`/home/lyx/.local/opencv-4.8.0/lib`
- vikit：`~/ros2_fast_livo2/src/vikit_common`、`vikit_ros`

**每次打开新终端都需要 source 环境脚本：**

```bash
source ~/ros2_fast_livo2/setup_livo2_env.sh
```

该脚本已写好，会自动完成以下 source 顺序：

1. `/opt/ros/humble/setup.bash`
2. `~/ros2_vins/setup_vins_env.sh`（RealSense / VINS 及 OpenCV 4.8.0）
3. `~/ros2_fast_lio2/install/setup.bash`（`livox_ros_driver2`）
4. `~/ros2_fast_livo2/install/setup.bash`（FAST-LIVO2 本工作空间）
5. `export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib`

> 注意：`livo2_env` 这类别名如果直接输入提示 `command not found`，是因为当前 shell 没有加载别名定义。请使用上面的 `source` 脚本方式。

---

## 2. 编译构建

在修改源码或更换配置后需要重新编译：

```bash
cd ~/ros2_fast_livo2

# 首次编译（Release）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 只编译 FAST-LIVO2 相关包
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   --packages-select fast_livo vikit_common vikit_ros
```

编译完成后 source overlay：

```bash
source ~/ros2_fast_livo2/install/setup.bash
```

> 注意：`src/FAST-LIVO2` 目录下已有 `COLCON_IGNORE`，编译时不会重复构建同名 `fast_livo` 包。

---

## 3. 硬件连接

### 3.1 Livox Mid-360

- 通过以太网连接本机，网络配置参考 Livox 官方文档（通常设置为 `192.168.1.x` 段）。
- 确认 Livox 可通过 `Livox Viewer2` 或 `ros2 launch livox_ros_driver2 msg_MID360_launch.py` 正常出流。

### 3.2 RealSense D435i

- 通过 USB3.0 连接。
- 确认设备被识别：

```bash
rs-enumerate-devices
```

- 启动参数中只启用左红外 `infra1` 和 IMU，不启用彩色/深度图像，以节省带宽。

---

## 4. 启动 FAST-LIVO2

### 4.1 一键启动

```bash
livo2_env
ros2 launch fast_livo mapping_mid360_d435i.launch.py use_rviz:=False
```

### 4.2 带可视化 RViz2

```bash
ros2 launch fast_livo mapping_mid360_d435i.launch.py use_rviz:=True
```

### 4.3 Launch 文件说明

- **文件路径**：`~/ros2_fast_livo2/src/FAST-LIVO2-ROS2/launch/mapping_mid360_d435i.launch.py`
- **启动内容**：
  - `livox_ros_driver2_node`：直接以 `xfer_format: 0` 发布 `sensor_msgs/PointCloud2`。
  - `realsense2_camera`：发布 `/camera/camera/infra1/image_rect_raw` 与 IMU。
  - `fastlivo_mapping`：FAST-LIVO2 主节点。
  - 可选 `rviz2`。

### 4.4 加载的参数文件

- `config/mid360_d435i.yaml`：主参数（传感器话题、外参、LIO/VIO 参数）。
- `config/camera_d435i_pinhole.yaml`：D435i 左红外相机内参，供 vikit 使用。

可以通过 launch 参数替换：

```bash
ros2 launch fast_livo mapping_mid360_d435i.launch.py \
  params_file:=/path/to/your/mid360_d435i.yaml \
  camera_params_file:=/path/to/your/camera.yaml
```

---

## 5. 验证数据流

启动后，在另一个终端执行：

```bash
livo2_env
ros2 topic hz /livox/lidar /livox/imu \
  /camera/camera/infra1/image_rect_raw \
  /aft_mapped_to_init /path /rgb_img
```

正常情况下的频率参考：

| 话题 | 类型 | 频率 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | ~10 Hz |
| `/livox/imu` | `sensor_msgs/Imu` | ~200 Hz |
| `/camera/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | 30 Hz |
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | ~20 Hz |
| `/path` | `nav_msgs/Path` | ~20 Hz |
| `/rgb_img` | `sensor_msgs/Image` | ~18 Hz |

查看 TF 树：

```bash
ros2 run tf2_tools view_frames.py
```

---

## 6. 关键参数说明

### 6.1 LiDAR 类型

`mid360_d435i.yaml` 中：

```yaml
preprocess:
  lidar_type: 7        # MID360 对应 PointCloud2 输入
  scan_line: 4
  point_filter_num: 3
  filter_size_surf: 0.2
  blind: 0.5
```

`lidar_type: 7` 表示 MID360，且代码会订阅 `sensor_msgs/PointCloud2`，因此 Livox driver 必须使用 `xfer_format: 0`。

### 6.2 外参（LiDAR → D435i 左相机）

当前使用参考标定值：

```yaml
extrin_calib:
  Rcl: [-0.005228, -0.998807,  0.048561,
         0.018604, -0.048650, -0.998643,
         0.999813, -0.004318,  0.018836]
  Pcl: [0.029013, -0.150000, -0.095000]
```

单位：旋转矩阵无量纲，平移向量单位为 **米**。

若更改安装方式，需要用 kalibr / VINS-Fusion / 自定义标定重新估计该外参。

### 6.3 相机内参

`camera_d435i_pinhole.yaml`：

```yaml
camera:
  model: Pinhole       # vikit 只识别 "Pinhole"，不能写 "PinholeCamera"
  width: 640
  height: 480
  fx: 391.0418701171875
  fy: 391.0418701171875
  cx: 319.01458740234375
  cy: 245.74755859375
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
```

### 6.4 小房间调参

为适应室内小场景，当前做了以下调整：

```yaml
preprocess:
  filter_size_surf: 0.2   # 下采样体素

lio:
  max_iterations: 3
  voxel_size: 0.3

uav:
  gravity_align_en: true
```

---

## 7. 辅助脚本

### 7.1 点云曲率/时间戳验证

验证 `/livox/lidar` 中每点的 `timestamp` 字段是否可用于图像时间对齐：

```bash
python3 ~/ros2_fast_livo2/scripts/pc2_curvature_demo.py
```

正常输出示例：

```text
Frame header: 1783240525.208842 s, points: 19872, scan duration: 99.83 ms
index |     x      |     y      |     z      | intensity | tag | line |    timestamp (ns)    | curvature (ms)
    0 |    -0.3150 |    -1.6020 |     0.9010 |      1.00 |   0 |    0 |  1783240525208842240 |          0.000
...
19871 | ... |          99.835 ms
```

这说明 `PointCloud2` 同样保留了每点时间戳，`sync_packages()` 可据此把 10 Hz 点云切分为 30 Hz 图像同步段。

### 7.2 保存结果

- 日志/PCD 默认目录：`~/ros2_fast_livo2/src/FAST-LIVO2-ROS2/Log/`
- 开启 PCD 保存：将 `mid360_d435i.yaml` 中 `pcd_save.pcd_save_en` 设为 `true`。
- 开启位姿输出：将 `evo.pose_output_en` 设为 `true`。

---

## 8. 常见问题

### 8.1 `[ LIO ]: No point!!!` / `[ VIO ] No point!!!`

- 通常是图像时间与 LiDAR 扫描边界对齐时的瞬时现象。
- 如果静止时偶发、随后恢复，可忽略；若运动中持续出现，检查：
  - Livox 是否丢包（`ros2 topic hz /livox/lidar` 是否稳定 ~10 Hz）
  - `imu_topic`、`img_topic` 是否正确
  - 时间戳是否跳变

### 8.2 `IMU Calibration is not available, default intrinsic and extrinsic will be used.`

- RealSense 的 IMU 出厂未写入标定，属于非致命警告。
- 如需更高精度，可用 Intel `rs-imu-calibration-tool` 标定后重刷设备。

### 8.3 `IMU and LiDAR not synced!`

- Mid-360 的 LiDAR 与 IMU 硬件时间可能随运行漂移。
- 确保 Livox 固件较新，尽量避免长时间运行后时间差过大。

### 8.4 OpenCV 版本冲突警告

- 系统默认 OpenCV 为 4.5d，FAST-LIVO2/VINS 链路需要 4.8.0。
- 已通过 `source ~/ros2_vins/setup_vins_env.sh` 将 `LD_LIBRARY_PATH` 指向 4.8.0。
- 如仍报 ABI 错误，确认 source 顺序（先 `setup_vins_env.sh`，再 overlay）。

### 8.5 `model: PinholeCamera` 报错

- vikit 只接受 `Pinhole`，配置文件中已修正。
- 不要改回 `PinholeCamera`。

### 8.6 Livox driver 发布 `CustomMsg` 导致 FAST-LIVO2 无法订阅

- 本机 launch 文件已使用 `xfer_format: 0`（PointCloud2）。
- 若单独启动 Livox，请确保：

```bash
ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args -p xfer_format:=0
```

---

## 9. 后续建议

1. **静态测试**：先保持传感器静止 1–2 分钟，观察 `/path` 是否漂移。
2. **移动测试**：缓慢移动传感器，看 `/aft_mapped_to_init` 轨迹是否连续、回环是否闭合。
3. **外参精调**：如轨迹漂移明显，重新标定 `Rcl` / `Pcl`。
4. **保存地图**：开启 `dense_map_en` 和 `pcd_save_en`，运行后查看 `Log/pcd/`。
5. **EVO 评估**：开启 `pose_output_en`，用 `evo_ape` / `evo_rpe` 对比真值。

---

## 10. 相关文件速查

| 文件 | 路径 |
|---|---|
| 主 launch | `~/ros2_fast_livo2/src/FAST-LIVO2-ROS2/launch/mapping_mid360_d435i.launch.py` |
| 主参数 | `~/ros2_fast_livo2/src/FAST-LIVO2-ROS2/config/mid360_d435i.yaml` |
| 相机内参 | `~/ros2_fast_livo2/src/FAST-LIVO2-ROS2/config/camera_d435i_pinhole.yaml` |
| RViz 配置 | `~/ros2_fast_livo2/src/FAST-LIVO2-ROS2/rviz_cfg/fast_livo2.rviz` |
| 曲率验证脚本 | `~/ros2_fast_livo2/scripts/pc2_curvature_demo.py` |
| 日志/PCD | `~/ros2_fast_livo2/src/FAST-LIVO2-ROS2/Log/` |

---

## 11. 启动命令速记

```bash
# 1. 环境
source ~/ros2_fast_livo2/setup_livo2_env.sh

# 2. 启动（无 RViz）
ros2 launch fast_livo mapping_mid360_d435i.launch.py use_rviz:=False

# 3. 验证
ros2 topic hz /livox/lidar /livox/imu /camera/camera/infra1/image_rect_raw /aft_mapped_to_init
```

---

*最后更新：2026-07-05*
