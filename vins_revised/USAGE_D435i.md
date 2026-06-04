# Intel RealSense D435 / D435i 使用指南（ROS2 Rolling）

本指南说明如何在 **Ubuntu 22.04 + ROS2 Rolling** 环境下，使用 Intel RealSense **D435**（纯视觉）或 **D435i**（视觉+IMU）运行 VINS-Fusion-ROS2。

| 设备 | 是否支持 VO（纯视觉） | 是否支持 VIO（视觉惯性） |
|------|----------------------|-------------------------|
| D435 | ✅ 支持 | ❌ 无 IMU，不支持 |
| D435i | ✅ 支持 | ✅ 支持 |

---

## 前置条件

1. 已完成本仓库的编译：
   ```bash
   cd ~/ros2_ws
   source /opt/ros/rolling/setup.bash
   colcon build --symlink-install --packages-select camera_models vins loop_fusion global_fusion
   ```
2. 已编译 `realsense-ros`（ROS2 驱动）：
   ```bash
   # 确认已安装
   ros2 pkg list | grep realsense2_camera
   ```
3. **必须使用 USB 3.0/3.2 接口**（蓝色接口）。D435i 的 IMU 数据需要 USB 3.0 带宽，插到 USB 2.0 会导致 `Motion Module failure` 硬件错误，IMU 无法输出数据。
4. 相机已插入，且 `realsense-viewer` 能正常显示左右红外图。

---

## 配置文件位置

### 纯 VO 模式（848×480）

```
~/ros2_ws/src/VINS-Fusion-ROS2/config/realsense_d435i/
├── realsense_stereo_imu_config.yaml   # 主配置文件
├── left.yaml                          # 左相机标定
├── right.yaml                         # 右相机标定
└── rs_camera.launch                   # ROS1 遗留（ROS2 不用）
```

### VIO 模式（640×480，推荐）

```
~/ros2_ws/src/VINS-Fusion-ROS2/config/realsense_d435i_vio/
├── realsense_stereo_imu_config.yaml   # 主配置文件（imu:1, 640×480）
├── left.yaml                          # 左相机标定（640×480）
└── right.yaml                         # 右相机标定（640×480）
```

> **为什么用 640×480？** 降低分辨率可显著减少前端光流计算量，提高实时性。D435i 红外相机在 640×480 下的内参通过 `ros2 topic echo /camera/camera/infra1/camera_info` 实测获得，基线与 848×480 相同（~50mm）。

### 关键参数说明

```yaml
imu: 0          # 0=纯视觉VO，1=视觉惯性VIO
num_of_cam: 2   # 双目固定为 2

imu_topic: "/camera/imu"
image0_topic: "/camera/camera/infra1/image_rect_raw"
image1_topic: "/camera/camera/infra2/image_rect_raw"

estimate_extrinsic: 1   # 1=在线优化外参（推荐初次使用）；0=信任标定值
estimate_td: 1          # 1=在线估计相机与 IMU 时间偏移
td: 0.00
```

---

## 一、D435（无 IMU）—— 纯视觉 VO 模式

D435 没有 IMU，只能运行纯双目视觉里程计。该模式**没有绝对尺度**，存在尺度漂移。

### 1. 修改配置

编辑 `realsense_stereo_imu_config.yaml`：
```yaml
imu: 0
```

### 2. 启动 RealSense D435

```bash
source /opt/ros/rolling/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  depth_module.infra_profile:=848x480x30
```

> ⚠️ **USB 3.2 必需**：D435i 必须插在 USB 3.0/3.2（蓝色）接口上。若日志出现 `Device USB type: 2.1`，请更换端口。
>
> `enable_sync:=true` 开启硬件帧同步，确保左右目时间戳严格一致，避免 VINS 频繁 `throw img1`。

### 3. 启动 VINS

```bash
source /opt/ros/rolling/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run vins vins_node \
  ~/ros2_ws/src/VINS-Fusion-ROS2/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

---

## 二、D435i —— 纯视觉 VO 模式

D435i 也可以关闭 IMU，像 D435 一样只跑纯视觉。操作步骤与 D435 完全相同。

### 1. 修改配置

```yaml
imu: 0
```

### 2. 启动 RealSense D435i（VO 模式，无需 IMU）

```bash
source /opt/ros/rolling/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  depth_module.infra_profile:=848x480x30
```

> `enable_sync:=true` 开启硬件帧同步，确保左右目时间戳严格一致。

### 3. 启动 VINS

```bash
source /opt/ros/rolling/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run vins vins_node \
  ~/ros2_ws/src/VINS-Fusion-ROS2/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

---

## 三、D435i —— 视觉惯性 VIO 模式（推荐，640×480）

D435i 的核心优势是带有 IMU，运行 VIO 可获得**绝对尺度**，精度远高于纯 VO。本配置使用 **640×480** 分辨率，降低计算压力。

### 1. 配置文件

已为你准备好 VIO 专用配置（无需手动修改）：

```
config/realsense_d435i_vio/
├── realsense_stereo_imu_config.yaml   # imu:1, 640×480
├── left.yaml                          # fx=384.6, cx=316.4
└── right.yaml                         # fx=384.6, cx=316.4
```

参数来源（从 realsense-ros 实测）：

```bash
# 内参（640×480）
$ ros2 topic echo /camera/camera/infra1/camera_info
  k: [384.6005, 0, 316.4323, 0, 384.6005, 239.2910, 0, 0, 1]

# 基线（与 848×480 相同）
$ ros2 topic echo /camera/camera/infra2/camera_info
  p[0,3] = -19.2110  →  baseline = 19.211 / 384.6005 ≈ 0.04995 m

# IMU-相机外参
$ ros2 topic echo /camera/camera/extrinsics/depth_to_accel
  rotation: I, translation: [-0.00552, 0.00510, 0.01174]
```

#### 为什么 `body_T_cam0` 不是单位矩阵？

D435i 的 **IMU 坐标系** 和 **相机坐标系** 方向不同：

| 坐标轴 | 红外相机 (infra) | IMU (BMI085) |
|--------|-----------------|--------------|
| X | **右** (right) | **前** (forward) |
| Y | **下** (down) | **左** (left) |
| Z | **前** (forward) | **上** (up) |

因此从相机到 IMU 需要一个 **旋转矩阵** 做坐标变换。配置文件中 `body_T_cam0` 和 `body_T_cam1` 的旋转部分已使用 RealSense 默认标定值，平移部分包含了 IMU 到左右目的实际偏移。`estimate_extrinsic: 1` 会在 VIO 初始化时在线精修这些外参。

> ⚠️ **不要**把 `body_T_cam0` 简化为单位矩阵 `I`，否则 IMU 数据会被错误地投影到相机坐标系，导致 VIO 初始化失败或发散。

### 2. 启动 RealSense D435i（640×480 + IMU）

```bash
source /opt/ros/rolling/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  depth_module.infra_profile:=640x480x30 \
  enable_depth:=false \
  enable_color:=false
```

关键参数说明：
- `depth_module.infra_profile:=640x480x30`：红外流分辨率降为 640×480
- `enable_depth:=false enable_color:=false`：关闭深度和 RGB，减少计算和带宽占用
- `unite_imu_method:=2`：将 gyro 和 accel 按线性插值合并为 `/camera/camera/imu`
- `enable_sync:=true`：开启硬件帧同步
- ⚠️ **USB 3.2 必需**：确认日志中有 `Device USB type: 3.2`

### 3. 启动 VINS（VIO 模式）

```bash
source /opt/ros/rolling/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run vins vins_node \
  ~/ros2_ws/src/VINS-Fusion-ROS2/config/realsense_d435i_vio/realsense_stereo_imu_config.yaml
```

启动成功后应看到：
```
USE_IMU: 1
waiting for image and imu...
```

> **话题名注意**：源码编译的 realsense-ros 默认 namespace 为 `camera`，节点名为 `camera`，因此话题前缀为 `/camera/camera/...`。配置文件已设置为：
> - `imu_topic: "/camera/camera/imu"`
> - `image0_topic: "/camera/camera/infra1/image_rect_raw"`
> - `image1_topic: "/camera/camera/infra2/image_rect_raw"`

---

## 四、回环检测 Loop Fusion（抑制长期漂移）

Loop Fusion 是 VINS-Fusion 的可选后处理模块，用于检测相机是否回到了曾经到过的地方（回环），并通过位姿图优化（Pose Graph Optimization）修正累积的漂移。

**工作原理**：
- 订阅 VINS 输出的关键帧（位姿、图像、地图点）
- 使用 **DBoW2 + BRIEF** 描述子构建词袋模型，检索历史关键帧
- 检测到回环后，通过特征匹配和几何验证确认闭环
- 调用 **Ceres Solver** 进行位姿图优化，全局修正轨迹

> **注意**：Loop Fusion 是纯 **CPU** 计算，不占用 GPU。它运行在独立线程中，不会阻塞 VINS 前端的实时位姿估计。

---

### 4.1 启动 Loop Fusion

在 VINS 已经成功运行、各话题正常发布的前提下，打开新终端启动回环检测：

```bash
source /opt/ros/rolling/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run loop_fusion loop_fusion_node \
  ~/ros2_ws/src/VINS-Fusion-ROS2/config/realsense_d435i_vio/realsense_stereo_imu_config.yaml
```

启动成功后，终端会输出词袋加载信息：
```
vocabulary_file: /home/lyx/ros2_ws/install/loop_fusion/share/loop_fusion/support_files/brief_k10L6.bin
loop_fusion_node start
```

---

### 4.2 Loop Fusion 订阅的话题

Loop Fusion 从 VINS 接收以下 6 个话题：

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/vins_estimator/odometry` | `nav_msgs/Odometry` | VINS 实时位姿 |
| `/vins_estimator/keyframe_pose` | `nav_msgs/Odometry` | 关键帧位姿 |
| `/vins_estimator/keyframe_point` | `sensor_msgs/PointCloud` | 关键帧地图点 |
| `/vins_estimator/margin_cloud` | `sensor_msgs/PointCloud` | 边缘化点云 |
| `/vins_estimator/extrinsic` | `nav_msgs/Odometry` | 相机外参 |
| `/camera/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | 左目图像（用于 BRIEF 描述子） |

> 确保 VINS 已经正常运行且以上话题都在 `ros2 topic list` 中可见，否则 Loop Fusion 会等待数据。

---

### 4.3 Loop Fusion 发布的话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/loop_fusion/odometry_rect` | `nav_msgs/Odometry` | 回环修正后的实时位姿 |
| `/loop_fusion/pose_graph_path` | `nav_msgs/Path` | 回环修正后的全局轨迹 |
| `/loop_fusion/match_image` | `sensor_msgs/Image` | 回环匹配可视化图像（DEBUG_IMAGE=1 时） |

> **RViz 显示建议**：添加 `Path` 显示 `/loop_fusion/pose_graph_path`，观察轨迹是否因回环而被拉直。

---

### 4.4 验证回环检测是否工作

**方法 1：观察终端输出**

当相机回到曾经到过的地方时，终端会输出类似：
```
30 detect loop with 5
loop_fusion with previous sequence
calculate global optimization
begin global optimization
finish global optimization
```

- `30 detect loop with 5`：第 30 帧与第 5 帧检测到回环
- `finish global optimization`：位姿图优化完成，全局轨迹已修正

**方法 2：观察 RViz 轨迹**

1. 让相机绕房间走一圈回到起点
2. 观察 `/vins_estimator/path` 是否形成闭合漂移环（未回环时常见）
3. 观察 `/loop_fusion/pose_graph_path` 是否在回环后将尾部轨迹“拉”回起点，形成闭合

**方法 3：对比两个位姿**

```bash
# 原始 VINS 位姿（有漂移）
echo "VINS raw:"
ros2 topic echo /vins_estimator/odometry --once

# 回环修正后位姿
echo "Loop corrected:"
ros2 topic echo /loop_fusion/odometry_rect --once
```

---

### 4.5 性能特征

| 指标 | 数值/特征 |
|------|----------|
| **计算设备** | 纯 CPU（无 GPU/CUDA 参与） |
| **正常情况下（无回环）** | 每个关键帧处理 **< 5 ms**，几乎无延迟 |
| **回环发生时** | `findConnection` + 位姿图优化 **50~200 ms** |
| **是否阻塞 VINS 前端** | **否**，Loop Fusion 运行在独立线程 |
| **回环修正延迟** | `/loop_fusion/odometry_rect` 在回环瞬间可能滞后 **50~200 ms**，随后追上 |
| **长期运行** | 关键帧增多后，单次位姿图优化可能增至 **300~500 ms**（30 分钟以上） |

> 如果后期感到明显卡顿，可修改 `loop_fusion/src/pose_graph_node.cpp` 中的 `SKIP_DIS` 增大关键帧空间采样间隔，或减少 `max_num_iterations`，然后重新编译。

---

### 4.6 常见问题

**Q1：启动 Loop Fusion 后终端没有输出，也没有 `/loop_fusion/*` 话题**
- 检查 VINS 是否已正常运行：
  ```bash
  ros2 topic echo /vins_estimator/keyframe_pose --once
  ```
- 确保配置文件路径正确，Loop Fusion 需要与 VINS 使用**同一个** `realsense_stereo_imu_config.yaml`

**Q2：长时间运行后 Loop Fusion 越来越卡**
- 这是正常的，位姿图优化的计算量随关键帧数量线性增长。
- 解决：在 `loop_fusion/src/pose_graph_node.cpp` 中修改 `SKIP_DIS = 0.5`（默认 0），只在大位移时创建关键帧，然后重新编译。

**Q3：回环检测成功率低**
- 确保场景有足够的纹理（白墙、天花板纹理少，回环检测困难）
- 确保光照稳定，红外图不能太暗
- 回到同一位置时，视角不宜差异过大（>45° 可能匹配失败）

---

## 五、可视化与对比工具

### 5.1 RViz2 可视化

```bash
ros2 launch vins vins_rviz.launch.xml
```

或在 RViz2 中手动添加以下话题：

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/vins_estimator/odometry` | `nav_msgs/Odometry` | 实时位姿 |
| `/vins_estimator/path` | `nav_msgs/Path` | 轨迹 |
| `/vins_estimator/point_cloud` | `sensor_msgs/PointCloud` | 地图点 |
| `/vins_estimator/image_track` | `sensor_msgs/Image` | 特征跟踪图 |
| `/loop_fusion/pose_graph_path` | `nav_msgs/Path` | 回环后全局轨迹（Loop Fusion） |
| `/loop_fusion/odometry_rect` | `nav_msgs/Odometry` | 回环修正位姿（Loop Fusion） |

---

### 5.2 实时对比终端输出

脚本路径：`~/VINS-Fusion-ROS2/scripts/odom_compare.py`

功能：实时订阅 `/vins_estimator/odometry`（原始 VINS）和 `/odometry_rect`（回环修正），每 0.5 秒计算并打印位置差异和角度差异。

```bash
cd ~/VINS-Fusion-ROS2/scripts
python3 odom_compare.py
```

**输出示例：**
```
[45.20s] 🔄 LOOP CORRECTED
  Position diff : 0.8234 m
  Angle diff    : 3.521 deg
  VINS raw      : (5.234, 2.100, 0.150)
  Loop corrected: (4.412, 1.890, 0.145)
```

- **✅ MATCHED**：差异很小（位置 < 5cm，角度 < 2°），回环尚未触发或修正量很小
- **🔄 LOOP CORRECTED**：差异明显，回环已触发，Loop Fusion 正在修正漂移

**保存数据到 CSV（可选）：**
```bash
python3 odom_compare.py --save ~/odom_compare_result.csv
```

---

### 5.3 实时可视化窗口

脚本路径：`~/VINS-Fusion-ROS2/scripts/realtime_odom_plot.py`

功能：弹出实时窗口，动态绘制四幅图，坐标轴根据数据范围自动缩放。

```bash
cd ~/VINS-Fusion-ROS2/scripts
python3 realtime_odom_plot.py
```

| 子图 | 内容 | 坐标轴缩放 |
|------|------|-----------|
| **Top View (X-Y)** | 蓝线=原始 VINS，红线=回环修正 | 等比例，每 10 帧自动重新计算范围 |
| **Height (Z)** | Z 高度随时间 | 根据 Z 的 min/max 自动调整 |
| **Position Drift** | 位置差异，橙色虚线=5cm 阈值 | 根据漂移量自动调整上限 |
| **Angular Drift** | 角度差异，橙色虚线=2° 阈值 | 根据角度差自动调整上限 |

窗口标题实时显示统计：`Points: 234 | Max Pos: 0.823m | Max Ang: 3.52°`

> 如果报错 `ImportError: No module named 'tkinter'`：
> ```bash
> sudo apt install python3-tk
> ```

---

### 5.4 离线可视化（CSV 回放）

先用 `odom_compare.py --save` 采集数据，再用 `plot_odom_compare.py` 画图：

```bash
# 采集（运行 VINS + Loop Fusion 时执行）
python3 odom_compare.py --save ~/odom_compare_result.csv

# 离线画图
python3 plot_odom_compare.py ~/odom_compare_result.csv
```

输出四幅子图（X-Y 轨迹、Z 高度、位置漂移、角度漂移），并自动保存同名 `.png` 文件。

---

## 六、快速指令速查表

| 步骤 | D435 VO | D435i VO | D435i VIO |
|------|---------|----------|-----------|
| **改配置** | `imu: 0` | `imu: 0` | `imu: 1` |
| **启动相机** | 基础参数 | 基础参数 | 加 IMU 参数* |
| **启动 VINS** | 同上 | 同上 | 同上 |
| **Loop Fusion** | 可选 | 可选 | 可选 |
| **RViz** | 可选 | 可选 | 可选 |

\* 基础相机参数：`enable_sync:=true enable_infra1:=true enable_infra2:=true depth_module.infra_profile:=848x480x30`  
\* IMU 参数（VIO 模式额外加）：`enable_gyro:=true enable_accel:=true unite_imu_method:=2`

---

## 七、常见问题

### 1. 提示 `waiting for image and imu...` 后没反应
- 检查 RealSense 是否已发布图像：
  ```bash
  ros2 topic hz /camera/infra1/image_rect_raw
  ros2 topic hz /camera/imu        # VIO 模式才需要
  ```
- 若话题名不同，修改 `realsense_stereo_imu_config.yaml` 中的 `imu_topic`、`image0_topic`、`image1_topic`。

### 2. VIO 初始化失败 / 轨迹发散
- 确保 `unite_imu_method:=2`，否则 gyro 和 accel 分开发布，VINS 收不到 `/camera/imu`
- 确保 `enable_sync:=true`，时间不同步会导致初始化失败
- 初次使用建议保持 `estimate_extrinsic: 1` 和 `estimate_td: 1`
- 手持相机**充分激励**（平移+旋转），静止状态下 VIO 无法初始化

### 3. 图像很暗或没有红外图
- D435/D435i 默认红外图是暗的，需要在 `realsense-viewer` 中确认能出图
- 确保 `enable_infra1:=true enable_infra2:=true`

### 4. 大量 `throw img1` / 左右目时间戳不同步
- **必须**加 `enable_sync:=true` 开启硬件帧同步
- USB 线材质量差也会导致时间戳抖动和数据丢帧，建议使用原装线或认证 USB 3.0 线
- 如果仍频繁 `throw img1`，检查 `ros2 topic hz /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw` 两路帧率是否稳定

---

## 参考

- [VINS-Fusion 原仓库（ROS1）](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [Intel RealSense ROS2 官方文档](https://github.com/IntelRealSense/realsense-ros)
- 本项目配置文件：`config/realsense_d435i/realsense_stereo_imu_config.yaml`
