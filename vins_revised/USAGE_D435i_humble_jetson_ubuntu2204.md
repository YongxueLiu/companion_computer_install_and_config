# Intel RealSense D435 / D435i 使用指南（ROS2 Humble）

本指南说明如何在 **Ubuntu 22.04 + ROS2 Humble** 环境下，使用 Intel RealSense **D435**（纯视觉）或 **D435i**（视觉+IMU）运行 VINS-Fusion-ROS2。

| 设备 | 是否支持 VO（纯视觉） | 是否支持 VIO（视觉惯性） |
|------|----------------------|-------------------------|
| D435 | ✅ 支持 | ❌ 无 IMU，不支持 |
| D435i | ✅ 支持 | ✅ 支持 |

---

## 前置条件

1. 已完成本仓库的编译（Jetson Orin Nano 内存有限，必须单包顺序编译，避免 OOM）：
   ```bash
   cd /home/lingzhilab/vins
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --parallel-workers 1 \
     --packages-select camera_models vins loop_fusion global_fusion
   ```
   > ⚠️ **Jetson 编译约束**：`--parallel-workers 1` 强制单线程编译。Jetson Orin Nano 8GB 内存不足以支撑多包并行编译，去掉此参数会导致 `c++: internal compiler error: Killed (program cc1plus)`。
2. 已编译 `realsense-ros`（ROS2 驱动）：
   ```bash
   # 确认已安装
   ros2 pkg list | grep realsense2_camera
   ```
3. **必须使用 USB 3.0/3.2 接口**（蓝色接口）。D435i 的 IMU 数据需要 USB 3.0 带宽，插到 USB 2.0 会导致 `Motion Module failure` 硬件错误，IMU 无法输出数据。
4. 相机已插入，且 `realsense-viewer` 能正常显示左右红外图。

---

## Jetson Orin Nano 特别说明

本章节汇总了在 **NVIDIA Jetson Orin Nano (JetPack 6.0, L4T R36.4.7)** 上运行 RealSense + VINS 的平台特定注意事项。

### RSUSB 后端（非 UVC）

Jetson 的 tegra 内核缺少标准的 `uvcvideo` 和 `hid_sensor_hub` 驱动模块，因此无法使用 apt 安装的 `librealsense2`（基于 UVC 后端）。本环境的 `librealsense2` **从源码编译**，启用了 `-DFORCE_RSUSB_BACKEND=ON`：

| 特性 | UVC 后端（x86/apt） | RSUSB 后端（Jetson） |
|---|---|---|
| 驱动位置 | Kernel (`uvcvideo.ko`) | 用户态库 (`libusb`) |
| USB 锁定 | 可多进程共享 | **单实例独占** |
| HID/IMU | 通过 `hid_sensor_hub` | 完全由 RSUSB 处理 |
| `Protocol error` | 常见 | **已根治** |

**关键影响**：
1. **必须先 `pkill` 再启动**：RSUSB 独占 USB 接口，残留进程会导致 `RS2_USB_STATUS_BUSY`
2. **库路径冲突**：ROS2 Humble 自带的 `/opt/ros/humble/lib/aarch64-linux-gnu/librealsense2.so.2.57` 是旧版 UVC 后端，必须通过 `LD_LIBRARY_PATH` 优先加载 RSUSB 版本：
   ```bash
   export LD_LIBRARY_PATH=/home/lingzhilab/vins/install_realsense/lib:$LD_LIBRARY_PATH
   ```
   此设置已加入 `~/.bashrc`。

### 内存与编译

Jetson Orin Nano 8GB 内存在编译大型 C++ 包（如 `vins`、`loop_fusion`）时容易 OOM。

- **必须**使用 `--parallel-workers 1`（单包顺序编译）
- 如果单个包仍然 OOM，可临时创建 swap：
  ```bash
  sudo fallocate -l 4G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile
  ```

### RealSense 固件

当前相机固件：**5.17.0.10**。RSUSB 后端对该固件版本兼容性良好，无需升级。

---

## 配置文件位置

### 纯 VO 模式（640×480）

```
/home/lingzhilab/vins/src/VINS-Fusion-ROS2/config/realsense_d435i/
├── realsense_stereo_imu_config.yaml   # 主配置文件
├── left.yaml                          # 左相机标定
├── right.yaml                         # 右相机标定
└── rs_camera.launch                   # ROS1 遗留（ROS2 不用）
```

### VIO 模式（640×480，推荐）

```
/home/lingzhilab/vins/src/VINS-Fusion-ROS2/config/realsense_d435i_vio/
├── realsense_stereo_imu_config.yaml   # 主配置文件（imu:1, 640×480）
├── left.yaml                          # 左相机标定（640×480）
└── right.yaml                         # 右相机标定（640×480）
```

> **为什么用 640×480？** 降低分辨率可显著减少前端光流计算量，提高实时性。D435i 红外相机在 640×480 下的内参通过 `ros2 topic echo /camera/camera/infra1/camera_info` 实测获得，基线与 848×480 相同（~50mm）。

### 关键参数说明

```yaml
imu: 0          # 0=纯视觉VO，1=视觉惯性VIO
num_of_cam: 2   # 双目固定为 2

imu_topic: "/camera/camera/imu"
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

### 2. 一键启动 RealSense + VINS（推荐）

使用 launch 文件同时启动 RealSense 和 VINS，所有 VINS 输出话题自动带 `/vins_estimator/` 前缀：

```bash
source /opt/ros/humble/setup.bash
source /home/lingzhilab/vins/install/setup.bash

# Jetson RSUSB 后端：USB 设备被单实例独占，启动前必须清理残留进程
pkill -9 -f realsense2_camera_node
sleep 2

ros2 launch vins realsense_d435i_vins.launch.py \
  config_path:=/home/lingzhilab/vins/src/VINS-Fusion-ROS2/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

> ⚠️ **USB 3.2 必需**：D435i 必须插在 USB 3.0/3.2（蓝色）接口上。若日志出现 `Device USB type: 2.1`，请更换端口。
>
> `enable_sync:=true` 开启硬件帧同步，确保左右目时间戳严格一致，避免 VINS 频繁 `throw img1`。

### 3. 手动启动（调试用）

如需单独调试 RealSense 或 VINS，可分开启动：

```bash
# 终端 A：RealSense
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  depth_module.infra_profile:="640,480,30"

# 终端 B：VINS（注意：直接 ros2 run 的话题名不带 /vins_estimator/ 前缀）
ros2 run vins vins_node \
  /home/lingzhilab/vins/src/VINS-Fusion-ROS2/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

---

## 二、D435i —— 纯视觉 VO 模式

D435i 也可以关闭 IMU，像 D435 一样只跑纯视觉。操作步骤与 D435 完全相同。

### 1. 修改配置

```yaml
imu: 0
```

### 2. 一键启动 RealSense + VINS（推荐）

```bash
source /opt/ros/humble/setup.bash
source /home/lingzhilab/vins/install/setup.bash

pkill -9 -f realsense2_camera_node
sleep 2

ros2 launch vins realsense_d435i_vins.launch.py \
  config_path:=/home/lingzhilab/vins/src/VINS-Fusion-ROS2/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

> `enable_sync:=true` 开启硬件帧同步，确保左右目时间戳严格一致。

### 3. 手动启动（调试用）

分开启动方式与 D435 完全相同。直接 `ros2 run vins vins_node` 的话题名不带 `/vins_estimator/` 前缀。

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

#### `body_T_cam0` / `body_T_cam1` 从何而来？

D435i 的 `body_T_cam0` 和 `body_T_cam1` **不能照搬 Euroc 默认值**，必须从 RealSense 出厂标定数据推导。

**快速获取（启动 RealSense 后执行）：**

```bash
# IMU → 左目外参
ros2 topic echo /camera/camera/extrinsics/depth_to_accel --once

# 左目 → 右目基线
ros2 topic echo /camera/camera/extrinsics/depth_to_infra2 --once
```

**出厂标定输出示例：**

```yaml
# depth_to_accel
translation: [ -0.00552, 0.00510, 0.01174 ]   # ← cam0 原点在 IMU 系中的坐标
rotation:    [ 1, 0, 0, 0, 1, 0, 0, 0, 1 ]   # ← SDK 内部已对齐

# depth_to_infra2
translation: [ -0.05015, 0, 0 ]                # ← cam0 原点在 cam1 系中的坐标
```

**推导结果（已写入配置文件）：**

```yaml
body_T_cam0.t = [ -0.00552, 0.00510, 0.01174 ]   # 直接取自 depth_to_accel
body_T_cam1.t = body_T_cam0.t - depth_to_infra2.t
               = [ -0.00552, 0.00510, 0.01174 ] - [ -0.05015, 0, 0 ]
               = [  0.04463, 0.00510, 0.01174 ]
```

> 详细推导过程、librealsense extrinsics 约定、常见误区见：
> `tutorial/realsense_extrinsic_calibration_guide.md`

**坐标系对照：**

| 轴 | 红外相机 (infra) | IMU (BMI085) |
|---|---|---|
| X | **右** (right) | **前** (forward) |
| Y | **下** (down) | **左** (left) |
| Z | **前** (forward) | **上** (up) |

> **为什么旋转是单位矩阵 I？** 理论上 IMU 和相机坐标轴方向不同，但 **librealsense SDK 内部已自动将 IMU 数据转换到相机坐标系**，ROS 发布的 `/camera/camera/imu` 已使用 camera frame。`body_T_cam0` 的旋转部分不需要额外变换。如果把 Euroc 的 ~90° 旋转搬过来，IMU 数据会被**二次投影**，VIO 必然发散。

### 2. 启动 RealSense D435i（640×480 + IMU）

#### 2.1 一键启动 RealSense + VINS（推荐）

launch 文件已同时集成 RealSense 和 VINS，所有参数预配置为 Jetson 最优值：

```bash
source /opt/ros/humble/setup.bash
source /home/lingzhilab/vins/install/setup.bash

# Jetson RSUSB 后端：USB 设备被单实例独占，启动前必须清理残留进程
pkill -9 -f realsense2_camera_node
sleep 2

ros2 launch vins realsense_d435i_vins.launch.py \
  config_path:=/home/lingzhilab/vins/src/VINS-Fusion-ROS2/config/realsense_d435i_vio/realsense_stereo_imu_config.yaml
```

启动成功后应看到：
```
USE_IMU: 1
waiting for image and imu...
```

**此方式的优势**：
- RealSense 参数已内置（`enable_sync`, `infra_profile=640,480,30`, `unite_imu_method=2` 等）
- VINS 所有输出话题自动带 `/vins_estimator/` 前缀（与 loop_fusion 默认订阅名一致）
- 无需手动协调两个节点的启动顺序

#### 2.2 带硬复位启动（解决设备残留状态）

如果上次运行后 RealSense 未正常关闭，添加 `initial_reset:=true`：

```bash
pkill -9 -f realsense2_camera_node
sleep 2

ros2 launch vins realsense_d435i_vins.launch.py \
  config_path:=/home/lingzhilab/vins/src/VINS-Fusion-ROS2/config/realsense_d435i_vio/realsense_stereo_imu_config.yaml \
  initial_reset:=true
```

> `initial_reset:=true` 会在启动前向设备发送硬件复位指令。复位后设备需要约 1~2 秒重新枚举，属于正常现象。

#### 2.3 手动启动（调试用）

如需单独调试，可分开启动：

```bash
# 终端 A：RealSense
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  depth_module.infra_profile:="640,480,30" \
  enable_depth:=false \
  enable_color:=false

# 终端 B：VINS（话题名不带 /vins_estimator/ 前缀）
ros2 run vins vins_node \
  /home/lingzhilab/vins/src/VINS-Fusion-ROS2/config/realsense_d435i_vio/realsense_stereo_imu_config.yaml
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
source /opt/ros/humble/setup.bash
source /home/lingzhilab/vins/install/setup.bash

ros2 run loop_fusion loop_fusion_node \
  /home/lingzhilab/vins/src/VINS-Fusion-ROS2/config/realsense_d435i_vio/realsense_stereo_imu_config.yaml
```

启动成功后，终端会输出词袋加载信息：
```
vocabulary_file: /home/lingzhilab/vins/install/loop_fusion/share/loop_fusion/support_files/brief_k10L6.bin
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
| `/odometry_rect` | `nav_msgs/Odometry` | 回环修正后的实时位姿 |
| `/pose_graph_path` | `nav_msgs/Path` | 回环修正后的全局轨迹 |
| `/match_image` | `sensor_msgs/Image` | 回环匹配可视化图像（DEBUG_IMAGE=1 时） |

> **RViz 显示建议**：添加 `Path` 显示 `/pose_graph_path`，观察轨迹是否因回环而被拉直。

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
3. 观察 `/pose_graph_path` 是否在回环后将尾部轨迹“拉”回起点，形成闭合

**方法 3：对比两个位姿**

```bash
# 原始 VINS 位姿（有漂移）
echo "VINS raw:"
ros2 topic echo /vins_estimator/odometry --once

# 回环修正后位姿
echo "Loop corrected:"
ros2 topic echo /odometry_rect --once
```

---

### 4.5 性能特征

| 指标 | 数值/特征 |
|------|----------|
| **计算设备** | 纯 CPU（无 GPU/CUDA 参与） |
| **正常情况下（无回环）** | 每个关键帧处理 **< 5 ms**，几乎无延迟 |
| **回环发生时** | `findConnection` + 位姿图优化 **50~200 ms** |
| **是否阻塞 VINS 前端** | **否**，Loop Fusion 运行在独立线程 |
| **回环修正延迟** | `/odometry_rect` 在回环瞬间可能滞后 **50~200 ms**，随后追上 |
| **长期运行** | 关键帧增多后，单次位姿图优化可能增至 **300~500 ms**（30 分钟以上） |

> 如果后期感到明显卡顿，可修改 `loop_fusion/src/pose_graph_node.cpp` 中的 `SKIP_DIS` 增大关键帧空间采样间隔，或减少 `max_num_iterations`，然后重新编译。

---

### 4.6 常见问题

**Q1：启动 Loop Fusion 后终端没有输出，也没有 `/odometry_rect` 或 `/pose_graph_path` 话题**
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

**Q4：第二次跑的起飞点和第一次不一样，回环后位置会乱吗？**

**不会乱。** 这是 `load_previous_pose_graph: 1` 的正确行为：

- 第二次的 VIO 初始化会建立一个**新的局部坐标系**（原点 = 第二次起飞点 B）
- 当在 C 点触发回环时，系统计算一个**刚性变换**（`shift_r`, `shift_t`）
- 整个第二次的轨迹被**平移+旋转**到第一次的全局坐标系中
- 第二次的 B 点在旧坐标系中有确定位置，C 点被修正到 D 点附近
- **从 C 之后，所有导航输出都基于旧全局坐标系**

简言之：回环对齐是**全局坐标系刚性拼接**，不是"只修正 C 点"。只要你的应用需要全局一致地图（如无人机多次巡逻），这就是想要的效果。

> 如果你需要"每次起飞都是新原点"的局部定位，应设置 `load_previous_pose_graph: 0`。

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
| `/pose_graph_path` | `nav_msgs/Path` | 回环后全局轨迹（Loop Fusion） |
| `/odometry_rect` | `nav_msgs/Odometry` | 回环修正位姿（Loop Fusion） |

---

### 5.2 实时对比终端输出

脚本路径：`/home/lingzhilab/vins/src/VINS-Fusion-ROS2/scripts/odom_compare.py`

功能：实时订阅 `/vins_estimator/odometry`（原始 VINS）和 `/odometry_rect`（回环修正），每 0.5 秒计算并打印位置差异和角度差异。

```bash
cd /home/lingzhilab/vins/src/VINS-Fusion-ROS2/scripts
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

脚本路径：`/home/lingzhilab/vins/src/VINS-Fusion-ROS2/scripts/realtime_odom_plot.py`

功能：弹出实时窗口，动态绘制姿态角和轨迹，坐标轴根据数据范围自动缩放。

**对比模式（默认，需同时运行 Loop Fusion）：**

```bash
cd /home/lingzhilab/vins/src/VINS-Fusion-ROS2/scripts
python3 realtime_odom_plot.py
```

| 子图 | 内容 | 坐标轴缩放 |
|---|---|---|
| X/Y/Z Position vs Time | VINS raw vs Loop corrected | 自动 |
| Position Drift | 位置差异 | 自动 |
| Angular Drift (Quaternion) | 四元数角度差异 | 自动 |
| Roll / Pitch / Yaw | VINS vs Loop 姿态角（PX4 FRD） | 自动 |

**单 VINS 模式（不运行 Loop Fusion 时用）：**

```bash
cd /home/lingzhilab/vins/src/VINS-Fusion-ROS2/scripts
python3 realtime_odom_plot.py --no-loop
```

布局（3×2，全部显示 VINS raw）：

| 位置 | 内容 |
|---|---|
| [0,0] | X Position vs Time |
| [0,1] | Y Position vs Time |
| [1,0] | Z Position vs Time |
| [1,1] | Roll vs Time |
| [2,0] | Pitch vs Time |
| [2,1] | Yaw vs Time |

RPY 采用 **PX4 FRD 坐标系**：
- Roll（绕 X/forward）：右侧下沉为正
- Pitch（绕 Y/right）：低头（nose down）为正
- Yaw（绕 Z/down）：向右转为正（从上方看顺时针）

---

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

### 推荐启动方式（launch 文件，话题统一前缀）

| 模式 | 一键启动命令 |
|------|-------------|
| **D435 VO** | `ros2 launch vins realsense_d435i_vins.launch.py config_path:=.../realsense_d435i/realsense_stereo_imu_config.yaml` |
| **D435i VO** | `ros2 launch vins realsense_d435i_vins.launch.py config_path:=.../realsense_d435i/realsense_stereo_imu_config.yaml` |
| **D435i VIO** | `ros2 launch vins realsense_d435i_vins.launch.py config_path:=.../realsense_d435i_vio/realsense_stereo_imu_config.yaml` |
| **Loop Fusion** | `ros2 run loop_fusion loop_fusion_node .../realsense_d435i_vio/realsense_stereo_imu_config.yaml` |

> 使用 launch 文件启动后，VINS 所有输出话题自动带 `/vins_estimator/` 前缀（与 loop_fusion 默认订阅名一致）。

### 手动启动方式（调试用）

| 步骤 | D435 VO | D435i VO | D435i VIO |
|------|---------|----------|-----------|
| **改配置** | `imu: 0` | `imu: 0` | `imu: 1` |
| **启动相机** | 基础参数 | 基础参数 | 加 IMU 参数* |
| **启动 VINS** | `ros2 run vins vins_node ...` | 同上 | 同上 |
| **Loop Fusion** | 可选 | 可选 | 可选 |
| **RViz** | 可选 | 可选 | 可选 |

\* 基础相机参数：`enable_sync:=true enable_infra1:=true enable_infra2:=true depth_module.infra_profile:="640,480,30"`  
\* IMU 参数（VIO 模式额外加）：`enable_gyro:=true enable_accel:=true unite_imu_method:=2`

---

## 七、坐标系与输出约定（重要）

VINS-Fusion 的 World 坐标系定义取决于运行模式（VO 还是 VIO），**两者完全不同**。理解这一点对解读 `/vins_estimator/odometry` 输出至关重要。

---

### 7.1 RealSense 相机坐标系

D435i 的红外相机（infra1/infra2）使用 **OpenCV 惯例**：

| 轴 | 方向 | 说明 |
|---|---|---|
| **X** | right（右）| 沿图像水平向右 |
| **Y** | down（下）| 沿图像垂直向下 |
| **Z** | forward（前）| 镜头光轴朝前 |

> **重要**：librealsense SDK 内部已将 IMU 数据自动转换到相机坐标系（见官方文档），因此 ROS 发布的 `/camera/camera/imu` 和相机共享同一坐标系。

---

### 7.2 VIO 模式（imu: 1）—— World 坐标系

VIO 初始化时调用 `initialStructure()`，通过 `g2R(g)` 将重力方向对齐到 World Z 轴：

| 轴 | 方向 | 与相机坐标系的关系 |
|---|---|---|
| **X** | right（右）| ≈ Camera X |
| **Y** | forward（前）| ≈ Camera Z |
| **Z** | up（上）| ≈ -Camera Y（重力反方向）|

**验证方法**：手持相机在空间中移动，观察 `/vins_estimator/odometry` 的 position：
- **往右移动** → `x` 增大
- **往前移动** → `y` 增大
- **向上移动** → `z` 增大

**特点**：
- World Z 轴固定向上（重力对齐），不随第一帧相机朝向改变
- X/Y 在水平面内，具体方向取决于初始化时第一帧的姿态
- 适合需要绝对高度（Z-up）的应用场景

---

### 7.3 VO 模式（imu: 0）—— World 坐标系

纯视觉 VO 初始化时调用 `clearState()`，`Rs[0] = I`，World 系直接等于**第一帧 Camera 坐标系**：

| 轴 | 方向 | 与相机坐标系的关系 |
|---|---|---|
| **X** | right（右）| = Camera X |
| **Y** | down（下）| = Camera Y |
| **Z** | forward（前）| = Camera Z |

**验证方法**：
- **往右移动** → `x` 增大
- **往下移动** → `y` 增大（注意：不是向上！）
- **往前移动** → `z` 增大

**特点**：
- World Z 轴 = 第一帧相机光轴方向，不是真正的"上"
- 没有绝对尺度，存在尺度漂移
- 相机倾斜放置时，World 系也随之倾斜

---

### 7.4 VO 与 VIO 坐标系对比

| 特征 | VIO（imu: 1） | VO（imu: 0） |
|---|---|---|
| World Z 轴 | **up（上）** 重力对齐 | **forward（前）** 第一帧相机朝向 |
| World Y 轴 | **forward（前）** | **down（下）** |
| 绝对高度 | ✅ Z 表示真实高度 | ❌ Z 只是深度，不代表高度 |
| 适用场景 | 无人机、机器人导航 | 纯视觉 SLAM、无 IMU 设备 |

> ⚠️ **常见误区**：在 RViz 中查看 VO 轨迹时，如果相机朝下安装，轨迹会在 RViz 的 XY 平面"平铺"，看起来像 2D 地图，但实际上 Z 轴是相机朝前方向，不是高度。

---

### 7.5 坐标系变换源码对照

| 模式 | 初始化函数 | 关键代码 | World 定义 |
|---|---|---|---|
| **VO** | `clearState()` | `Rs[0] = I` | 第一帧 Camera 系 |
| **VIO** | `initialStructure()` | `R0 = g2R(g)` | Z-up，重力对齐 |

其中 `g2R(g)` 的实现：
```cpp
// g ≈ [0, -9.8, 0] 在相机坐标系中（Y 向下）
// FromTwoVectors([0,-1,0], [0,0,1]) 把 Camera Y（向下）转到 World Z（向上）
R0 = Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
```

---

## 八、常见问题

### 1. 提示 `waiting for image and imu...` 后没反应
- 检查 RealSense 是否已发布图像：
  ```bash
  ros2 topic hz /camera/camera/infra1/image_rect_raw
  ros2 topic hz /camera/camera/imu        # VIO 模式才需要
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


### 5. `RS2_USB_STATUS_BUSY` / `failed to set power state`

Jetson Orin Nano 使用 **RSUSB 后端**（`-DFORCE_RSUSB_BACKEND=ON` 编译的 librealsense2），该后端通过用户态 USB 驱动与相机通信。RSUSB 会**独占锁定 USB 接口**，如果之前的 `realsense2_camera_node` 进程未正常退出，再次启动时会报错：

```
RS2_USB_STATUS_BUSY
failed to set power state
```

**解决**：每次启动 RealSense 前执行：
```bash
pkill -9 -f realsense2_camera_node
sleep 2
```

这是 Jetson RSUSB 后端的正常限制，非错误。

---

### 6. 启动时警告 `sequence size exceeds remaining buffer`

RSUSB 后端在启动瞬间可能打印：
```
[ WARN] sequence size exceeds remaining buffer ...
```

此警告**无害**，通常出现在单实例首次启动时，数据流稳定后会自动消失。如果持续出现，检查是否有多个 RealSense 节点在同时运行（见问题 5）。

---

### 7. 警告 `Motion Module force pause` / `enable_imu` 失败

D435i 的 IMU（BMI085）与彩色流共享内部资源。如果在启动参数中同时启用了 `enable_color:=true` 和 IMU，相机会自动暂停 IMU 以优先保证彩色流。解决：
- VINS 只使用红外图，**务必设置 `enable_color:=false`**
- 如果需要彩色图用于其他用途，建议单独启动一个只出彩色流的 RealSense 节点（不同命名空间）

> Jetson 的 tegra 内核**没有 `hid_sensor_hub`**，IMU 数据完全依赖 RSUSB 用户态驱动。`Motion Module force pause` 不是驱动错误，而是 D435i 硬件资源调度行为。

---

### 8. 频繁 `xioctl(UVCIOC_CTRL_QUERY) failed: Protocol error`

此错误通常来自 `global_timestamp_reader`，表示 librealsense SDK 与相机固件之间的 UVC 扩展单元协议不兼容。**在 Jetson RSUSB 后端上，该错误已被根治**（RSUSB 不通过 kernel UVC 驱动通信，而是使用用户态 USB 协议栈）。如果仍然看到类似错误，通常是因为系统中同时存在 apt 安装的 `librealsense2`（UVC 后端）和源码编译的 RSUSB 版本，发生了库冲突。

**检查库路径优先级**：
```bash
ldd /home/lingzhilab/vins/install/realsense2_camera/lib/realsense2_camera/realsense2_camera_node | grep realsense
```

应指向 `/home/lingzhilab/vins/install_realsense/lib/librealsense2.so.2.58`，而不是 `/opt/ros/humble/lib/aarch64-linux-gnu/librealsense2.so.2.57`。确保 `~/.bashrc` 中已设置：
```bash
export LD_LIBRARY_PATH=/home/lingzhilab/vins/install_realsense/lib:$LD_LIBRARY_PATH
```

## 参考

- [VINS-Fusion 原仓库（ROS1）](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [Intel RealSense ROS2 官方文档](https://github.com/IntelRealSense/realsense-ros)
- 本项目配置文件：`config/realsense_d435i/realsense_stereo_imu_config.yaml`
