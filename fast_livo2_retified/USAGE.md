# FAST-LIVO2 (ROS2 Humble) 使用说明

> 适用平台：Jetson Orin Nano `aarch64` + Ubuntu 22.04 + ROS2 Humble  
> 传感器：Livox MID-360 + RealSense D435i

## 0. 仓库结构

```text
/home/lingzhilab/fast_liov
├── src/
│   ├── FAST-LIVO2/          # ROS1 原版（本说明不使用）
│   └── FAST-LIVO2-ROS2/     # 当前使用的 ROS2 版本
│
/home/lingzhilab/ws_livo      # colcon 工作空间（overlay）
├── src/
│   ├── fast_livo -> /home/lingzhilab/fast_liov/src/FAST-LIVO2-ROS2
│   ├── vikit_common
│   └── vikit_ros
```

## 1. 硬件与网络配置

### 1.1 Livox MID-360 网络

- LiDAR IP：`192.168.1.135`
- 主机 IP：`192.168.1.50/24`

在 Jetson 上设置静态 IP：

```bash
# 假设网卡名为 eth0，请按实际情况替换
sudo ip addr add 192.168.1.50/24 dev eth0
sudo ip link set eth0 up
```

### 1.2 检查数据流

```bash
source /home/lingzhilab/ws_livox/install/setup.bash
ros2 topic list | grep livox
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

应看到 `/livox/lidar`（约 10 Hz）和 `/livox/imu`（约 200 Hz）。

### 1.3 RealSense D435i

通过 USB 3.0 连接。launch 文件中已关闭 color/depth，只启用 `infra1/infra2` 灰度图（30 Hz）和 IMU。

## 2. 工作空间准备

所有终端都必须按此顺序 source：

```bash
source /home/lingzhilab/ws_livox/install/setup.bash
source /home/lingzhilab/vins/install/setup.bash
source /home/lingzhilab/ws_livo/install/setup.bash
```

| 工作空间 | 内容 |
|---|---|
| `ws_livox` (underlay) | `livox_ros_driver2`、FAST-LIO2 |
| `vins` (underlay) | `realsense2_camera`、VINS-Fusion-ROS2 |
| `ws_livo` (overlay) | `fast_livo`、`vikit_common`、`vikit_ros`、本地 Sophus |

本地 Sophus 已安装在 `~/ws_livo/install/sophus`。

## 3. 编译

```bash
cd /home/lingzhilab/ws_livo
colcon build --packages-select fast_livo --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/home/lingzhilab/ws_livo/install/sophus
```

编译约 3 分钟。若修改了 `vikit_common`/`vikit_ros`，需先编译它们：

```bash
colcon build --packages-select vikit_common vikit_ros fast_livo --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/home/lingzhilab/ws_livo/install/sophus
```

## 4. 参数配置

主要配置文件：

- `src/FAST-LIVO2-ROS2/config/mid360_d435i.yaml`
- `src/FAST-LIVO2-ROS2/config/camera_d435i.yaml`

### 4.1 话题名

```yaml
common:
  img_topic: "/camera/camera/infra1/image_rect_raw"
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"
```

### 4.2 关键性能参数（已针对 Jetson 优化）

```yaml
preprocess:
  point_filter_num: 3      # 每 3 个点取 1 个，降低点数
  filter_size_surf: 0.05   # VoxelGrid 降采样 leaf size
  lidar_type: 7            # MID-360

lio:
  max_iterations: 3        # ICP 最大迭代数（原 5）
  voxel_size: 0.3          # 地图体素大小（原 0.5）
```

### 4.3 外参

#### IMU → LiDAR

```yaml
extrin_calib:
  extrinsic_T: [-0.011, -0.02329, 0.04412]
  extrinsic_R: [1., 0., 0.,
                0., 1., 0.,
                0., 0., 1.]
```

#### LiDAR → Camera（OpenCV 坐标系）

```yaml
extrin_calib:
  Rcl: [0.0,  0.0,  1.0,
       -1.0,  0.0,  0.0,
        0.0, -1.0,  0.0]
  Pcl: [0.05, 0.0, -0.05]
```

> 若 VIO 效果差或点云投影错位，需用标定工具（如 Kalibr、lidar_camera_calib）重新标定。

**是否在线优化？**  
当前程序**不对相机-激光雷达外参进行在线优化**。`Rcl`/`Pcl` 在启动时从配置文件读取一次，之后作为固定值参与 VIO 投影计算。EKF 状态维度 `DIM_STATE = 19`，仅包含姿态、位置、速度、IMU bias、重力和曝光时间倒数，不包含外参。因此运行前务必把外参标定准确。

### 4.4 相机内参

```yaml
camera:
  model: Pinhole
  width: 640
  height: 480
  fx: 384.6004638671875
  fy: 384.6004638671875
  cx: 316.4323425292969
  cy: 239.29095458984375
```

## 5. 运行

### 5.1 不带 RViz（推荐用于性能测试）

```bash
source /home/lingzhilab/ws_livox/install/setup.bash
source /home/lingzhilab/vins/install/setup.bash
source /home/lingzhilab/ws_livo/install/setup.bash

timeout --signal=INT 75 ros2 launch fast_livo mapping_mid360_d435i.launch.py use_rviz:=False
```

### 5.2 带 RViz（可视化）

```bash
ros2 launch fast_livo mapping_mid360_d435i.launch.py use_rviz:=True
```

### 5.3 launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `use_rviz` | `False` | 是否启动 RViz2 |
| `use_respawn` | `True` | 节点崩溃后是否自动重启 |

## 6. 验证是否正常运行

### 6.1 查看关键日志

正常运行时应看到：

```text
[fastlivo_mapping-3] FIRST LIDAR FRAME!
[fastlivo_mapping-3] Gravity Alignment Finished
[fastlivo_mapping-3] [ LIO ] Raw feature num: XXXX, downsampled feature num:XXX ...
[fastlivo_mapping-3] [ VIO ] Raw feature num: XXXX
[fastlivo_mapping-3] [timing] stateEstimationAndMapping() took XX.XXX ms
```

### 6.2 检查话题

```bash
ros2 topic hz /aft_mapped_to_init          # 里程计应 10 Hz
ros2 topic hz /livox/lidar                  # LiDAR 应 10 Hz
ros2 topic hz /livox/imu                    # IMU 应 200 Hz
```

### 6.3 异常指标

若出现以下日志，说明实时性仍未达标：

```text
IMU and LiDAR not synced! delta time: X.XXX
imu time stamp Jumps X.XXXX seconds
```

正常情况下这两项应为 0。

## 7. 性能调优建议

| 现象 | 调整方向 |
|---|---|
| LIO 平均耗时 > 80 ms | 增大 `point_filter_num`、增大 `filter_size_surf`、减小 `lio.max_iterations` |
| 地图太稀疏 / 定位漂移 | 减小 `point_filter_num`、减小 `voxel_size`、增大 `max_iterations` |
| VIO 跟踪丢失 | 检查 LiDAR-Camera 外参；降低 `img_point_cov`；增大 `patch_size` |
|  still 出现 IMU/LiDAR sync warning | 已修复：当前版本使用独立 spin 线程，不应再出现 |

## 8. 常见问题

### 8.1 `Integer indices would overflow`

原因为 `filter_size_surf` 过小（如 0.001）。已改为 `0.05`，可消除该警告。

### 8.2 RViz 在 Jetson 上卡顿

Jetson GPU 资源有限，性能测试时建议 `use_rviz:=False`。

### 8.3 保存 PCD

```yaml
pcd_save:
  pcd_save_en: true
  interval: -1   # -1 保存到一个文件；正数表示每隔多少帧保存
```

## 9. 快速检查清单

- [ ] Livox 主机 IP 已设为 `192.168.1.50/24`
- [ ] `/livox/lidar` 和 `/livox/imu` 有数据
- [ ] 三个工作空间已按顺序 source
- [ ] 编译使用 `Release` 模式
- [ ] 外参已根据实际安装位置修改
- [ ] 运行时不带 RViz 进行首次性能验证

## 10. ROS2 话题说明

以下按功能分组介绍运行后会出现在系统中的话题。

### 10.1 传感器原始输入

| 话题 | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | Livox driver | MID-360 原始点云，10 Hz |
| `/livox/imu` | `sensor_msgs/Imu` | Livox driver | MID-360 原始 IMU，~200 Hz |
| `/camera/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | RealSense | 左红外灰度图，VIO 使用，30 Hz |
| `/camera/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | RealSense | 右红外灰度图 |
| `/camera/camera/imu` | `sensor_msgs/Imu` | RealSense | RealSense 融合 IMU（当前未使用， Livox IMU 优先） |
| `/camera/camera/accel/sample` | `sensor_msgs/Imu` | RealSense | 加速度计原始数据 |
| `/camera/camera/gyro/sample` | `sensor_msgs/Imu` | RealSense | 陀螺仪原始数据 |

### 10.2 相机标定与内参

| 话题 | 类型 | 说明 |
|---|---|---|
| `/camera/camera/infra1/camera_info` | `sensor_msgs/CameraInfo` | 左红外相机内参 |
| `/camera/camera/infra2/camera_info` | `sensor_msgs/CameraInfo` | 右红外相机内参 |
| `/camera/camera/extrinsics/depth_to_infra1` | ` Extrinsics` | depth 到 infra1 外参 |
| `/camera/camera/extrinsics/depth_to_infra2` | ` Extrinsics` | depth 到 infra2 外参 |
| `/camera/camera/extrinsics/depth_to_accel` | ` Extrinsics` | depth 到加速度计外参 |
| `/camera/camera/extrinsics/depth_to_gyro` | ` Extrinsics` | depth 到陀螺仪外参 |
| `/camera/camera/infra1/metadata` | ` Metadata` | 左红外元数据 |
| `/camera/camera/infra2/metadata` | ` Metadata` | 右红外元数据 |
| `/camera/camera/accel/imu_info` | ` IMUInfo` | 加速度计信息 |
| `/camera/camera/gyro/imu_info` | ` IMUInfo` | 陀螺仪信息 |

### 10.3 FAST-LIVO2 位姿与轨迹输出

| 话题 | 类型 | 说明 |
|---|---|---|
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | **主要里程计输出**，地图坐标系到初始坐标系 |
| `/path` | `nav_msgs/Path` | 历史轨迹可视化 |
| `/LIVO2/imu_propagate` | `nav_msgs/Odometry` | IMU 高频传播里程计（未经 LiDAR 修正） |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | 给 MAVROS 的视觉位姿（若接入飞控） |

### 10.4 FAST-LIVO2 点云与地图输出

| 话题 | 类型 | 说明 |
|---|---|---|
| `/Laser_map` | `sensor_msgs/PointCloud2` | 全局/局部稠密地图 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 当前帧配准后的点云 |
| `/cloud_effected` | `sensor_msgs/PointCloud2` | 当前帧有效特征点云 |
| `/cloud_visual_sub_map_before` | `sensor_msgs/PointCloud2` | VIO 视觉子地图 |
| `/rgb_img` | `sensor_msgs/Image` | 带有 RGB/深度投影的图像输出 |

### 10.5 可视化与调试

| 话题 | 类型 | 说明 |
|---|---|---|
| `/visualization_marker` | `visualization_msgs/Marker` | 单标记可视化 |
| `/visualization_marker_array` | `visualization_msgs/MarkerArray` | 标记数组可视化 |
| `/voxels` | `visualization_msgs/MarkerArray` | 体素地图可视化 |
| `/planes` | `visualization_msgs/MarkerArray` | 平面特征可视化 |
| `/planner_normal` | `sensor_msgs/PointCloud2` | 平面法向量点云 |
| `/planner_normal_array` | `visualization_msgs/MarkerArray` | 平面法向量标记 |

### 10.6 动态物体（可选功能）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/dyn_obj` | `sensor_msgs/PointCloud2` | 检测到的动态物体点云 |
| `/dyn_obj_dbg_hist` | `sensor_msgs/PointCloud2` | 动态物体调试历史点云 |
| `/dyn_obj_removed` | `sensor_msgs/PointCloud2` | 移除动态物体后的点云 |

### 10.7 TF 与系统话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/tf` | `tf2_msgs/TFMessage` | 动态坐标变换 |
| `/tf_static` | `tf2_msgs/TFMessage` | 静态坐标变换 |
| `/parameter_events` | `rcl_interfaces/ParameterEvent` | 参数变更事件 |
| `/rosout` | `rcl_interfaces/Log` | ROS2 日志聚合 |

### 10.8 RViz 交互话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz 中点击的点 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz 中设置的初始位姿 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz 中设置的目标点 |

## 11. LiDAR-Camera 外参标定方法

当前 FAST-LIVO2 **不在线优化** LiDAR-Camera 外参，因此 `Rcl`/`Pcl` 必须提前标定准确。下面介绍几种常用方法，按推荐程度排序。

### 11.1 方法 A：基于标定板（最推荐，精度最高）

工具：**`lidar_camera_calib`** 或 **`cam_lidar_calibration`**（ROS 包）。

#### 步骤

1. 打印一块大棋盘格或 AprilTag 标定板，尺寸要同时被 LiDAR 和相机看到。
2. 固定 Livox MID-360 和 D435i，确保安装后不再移动。
3. 录制 bag：
   ```bash
   ros2 bag record /livox/lidar /camera/camera/infra1/image_rect_raw
   ```
4. 在不同角度、距离摆放标定板，录制 10~20 组数据。
5. 用标定工具提取角点/平面约束，优化得到 `Rcl` 和 `Pcl`。
6. 将结果填入 `config/mid360_d435i.yaml`：
   ```yaml
   extrin_calib:
     Rcl: [r11, r12, r13,
           r21, r22, r23,
           r31, r32, r33]
     Pcl: [tx, ty, tz]
   ```

> 坐标系：`Rcl`/`Pcl` 表示 **LiDAR 坐标系 → Camera 坐标系** 的旋转和平移。

### 11.2 方法 B：无目标标定（适合不方便摆标定板的场景）

工具：**`lidar_camera_calib`**（Livox-SDK，基于边缘对齐的无目标标定）。

原理：利用图像中的边缘（Canny）和 LiDAR 点云中的深度不连续边缘，通过迭代优化使两类边缘对齐，从而估计 `Rcl`/`Pcl`。

优点：不需要标定板。  
缺点：需要纹理丰富、有清晰边缘的室内/室外场景；精度通常比有目标标定低；需要给出一个不太离谱的初值。

#### 11.2.1 安装

```bash
cd ~/ws_livo/src
git clone https://github.com/Livox-SDK/lidar_camera_calib.git
cd ../
colcon build --packages-select lidar_camera_calib
source install/setup.bash
```

#### 11.2.2 准备数据

确保 MID-360 和 D435i 固定不动，然后运行：

```bash
source /home/lingzhilab/ws_livox/install/setup.bash
source /home/lingzhilab/vins/install/setup.bash
source /home/lingzhilab/ws_livo/install/setup.bash

ros2 launch fast_livo mapping_mid360_d435i.launch.py use_rviz:=False
```

> 这里只是借用它发布 `/livox/lidar` 和 `/camera/camera/infra1/image_rect_raw`，也可以单独启动 Livox driver + RealSense。

#### 11.2.3 配置相机内参

在标定工具包内创建/修改相机配置文件，例如 `cam_d435i_infra1.yaml`：

```yaml
Camera:
  camera_matrix:
    rows: 3
    cols: 3
    data: [384.6005, 0.0,      316.4323,
           0.0,      384.6005, 239.2910,
           0.0,      0.0,      1.0]
  distortion_coefficients:
    rows: 1
    cols: 5
    data: [0.0, 0.0, 0.0, 0.0, 0.0]
  image_width: 640
  image_height: 480
```

> 因为订阅的是 `image_rect_raw`，畸变系数可填 0；若用原始图则需填入真实畸变系数。

#### 11.2.4 给出一个外参初值

创建一个初值文件 `init_extrinsic.yaml`，格式类似：

```yaml
extrinsicMatrix:
  rows: 4
  cols: 4
  data: [
    r11, r12, r13, tx,
    r21, r22, r23, ty,
    r31, r32, r33, tz,
    0.0, 0.0, 0.0, 1.0
  ]
```

`Rcl`/`Pcl` 可用尺子测量大致安装关系得到。方向注意：该矩阵通常表示 **LiDAR → Camera**。

#### 11.2.5 运行标定

```bash
ros2 run lidar_camera_calib lidar_camera_calib \
  --ros-args \
  -p lidar_topic:=/livox/lidar \
  -p camera_topic:=/camera/camera/infra1/image_rect_raw \
  -p camera_info_file:=/path/to/cam_d435i_infra1.yaml \
  -p init_extrinsic_file:=/path/to/init_extrinsic.yaml \
  -p output_path:=/home/lingzhilab/calib_result
```

#### 11.2.6 数据录制与离线标定（推荐）

在线标定对实时性要求较高，更稳的做法是先录 bag：

```bash
ros2 bag record /livox/lidar /camera/camera/infra1/image_rect_raw -o calib_bag
```

然后在场景中缓慢移动标定板或让传感器对准不同墙面/物体边缘，录制 30~60 秒。之后回放：

```bash
ros2 bag play calib_bag
ros2 run lidar_camera_calib lidar_camera_calib \
  --ros-args \
  -p lidar_topic:=/livox/lidar \
  -p camera_topic:=/camera/camera/infra1/image_rect_raw \
  -p camera_info_file:=/path/to/cam_d435i_infra1.yaml \
  -p init_extrinsic_file:=/path/to/init_extrinsic.yaml \
  -p output_path:=/home/lingzhilab/calib_result
```

#### 11.2.7 输出结果转换

标定工具一般会输出一个 4×4 矩阵文件 `extrinsic.txt`。读取后提取左上 3×3 作为 `Rcl`，右上角 3×1 作为 `Pcl`，按行优先填入 `config/mid360_d435i.yaml`：

```yaml
extrin_calib:
  Rcl: [r11, r12, r13,
        r21, r22, r23,
        r31, r32, r33]
  Pcl: [tx, ty, tz]
```

> 不同工具对矩阵方向的约定可能不同。若填入 FAST-LIVO2 后 `/rgb_img` 中点云投影左右/上下镜像，通常说明方向反了，需要转置 `Rcl` 或取反 `Pcl` 再试。

#### 11.2.8 注意事项

1. **初值不能太差**：无目标标定容易陷入局部最优，初值误差建议在 ±10 cm / ±10° 以内。
2. **场景选择**：
   - 选有明显直线边缘的环境（门框、墙角、柜子、书架）；
   - 避免天空、白墙、玻璃、动态物体；
   - LiDAR 和相机视野要有足够重叠。
3. **红外图适用**：D435i 红外图有纹理，适合该标定方法；但红外发射器可能造成过曝，必要时关闭 IR emitter。
4. **参数差异**：不同 fork 的 `lidar_camera_calib` 参数名可能不同（如 `camera_model`、`use_voxel`、`voxel_size`），具体请参照其 README 和 launch/config 示例。

如果该工具在你的 ROS2 环境下编译困难，可直接改用**方法 C（手动 + RViz 微调）**或**方法 A（有目标标定）**。

### 11.3 方法 C：手动测量 + RViz 微调（快速但粗糙）

1. 用尺子/ CAD 粗略测量 LiDAR 到相机的相对位置，得到 `Pcl` 初值。
2. 根据安装姿态给 `Rcl` 一个合理的初值（例如相机朝前、LiDAR 也朝前时近似单位阵）。
3. 运行 FAST-LIVO2，打开 RViz，订阅 `/rgb_img` 或 `/cloud_visual_sub_map_before`。
4. 观察 LiDAR 点云投影到图像上是否错位：
   - 如果点云整体偏移 → 调整 `Pcl`；
   - 如果近处准、远处偏 → 调整 `Rcl`；
   - 如果旋转方向反了 → 检查 `Rcl` 是 LiDAR→Camera 还是 Camera→LiDAR。
5. 反复小幅度修改 yaml 并重启，直到投影对齐。

### 11.4 方法 D：分步标定（Camera-IMU + LiDAR-IMU）

如果你已经有：
- Camera-IMU 外参（用 Kalibr 标定）
- LiDAR-IMU 外参（用 FAST-LIO 的 `extrinsic_T`/`extrinsic_R`）

可以通过 IMU 作为中介链式合成：

```
T_lidar_camera = T_lidar_imu * T_imu_camera
```

注意矩阵方向：
- `T_imu_camera` = (`T_camera_imu`)^-1
- 结果再转成 FAST-LIVO2 要求的 `Rcl`/`Pcl` 格式

### 11.5 验证外参是否准确

运行后观察以下话题：

- `/rgb_img`：看 LiDAR 点云投影到图像上是否贴在真实物体边缘；
- `/cloud_visual_sub_map_before`：看视觉地图点与 LiDAR 地图是否重合；
- VIO 残差日志：如果 `average residual` 持续很大，可能是外参不准。

### 11.6 标定工具安装参考

```bash
cd ~/ws_livo/src

# 方法 A（有目标标定）
git clone https://github.com/Livox-SDK/livox_camera_calib.git

# 方法 B（无目标标定）
git clone https://github.com/Livox-SDK/lidar_camera_calib.git

# 方法 A 的另一种选择（ROS2）
git clone https://github.com/epanholz/cam_lidar_calibration.git

cd ../
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> 不同 fork 的 `lidar_camera_calib` 参数名可能不同，编译前请阅读对应 README，并根据实际情况修改 launch/config。上面方法 B 的命令是基于常见参数形式的示例。
