# Intel RealSense D435i 运行 VINS-Fusion VO/VIO 使用指南

本指南说明如何在 **Jetson Nano / Ubuntu 22.04 / ROS2 Humble** 环境下，使用 Intel RealSense **D435**（纯 VO）或 **D435i**（VO/VIO）运行 VINS-Fusion-ROS2。

| 设备 | VO（纯视觉） | VIO（视觉+IMU） |
|------|-------------|----------------|
| D435 | ✅ | ❌（无 IMU） |
| D435i | ✅ | ✅（推荐） |

---

## 0. 重要：OpenCV 版本（必须先做）

VINS-Fusion 前端光流 `cv::calcOpticalFlowPyrLK` 与系统默认 OpenCV **4.5d** 存在 ABI 不兼容，运行时会在第一帧直接段错误。本仓库已：

1. 从源码编译并安装 **OpenCV 4.8.0** 到用户目录：
   ```bash
   /home/lyx/.local/opencv-4.8.0
   ```
2. 重新编译 `cv_bridge`、`camera_models`、`vins`、`loop_fusion` 链接到该 OpenCV。

**每次打开新终端运行 VINS 前，建议直接 source 环境脚本：**
```bash
source /home/lyx/ros2_vins/setup_vins_env.sh
```

该脚本会依次 source ROS 2、工作区 overlay，并导出 OpenCV 4.8.0 库路径。

> 下文的一键 launch 文件已自动为 `vins_node` 设置该环境变量；手动启动时请勿忘记 `export LD_LIBRARY_PATH`。

---

## 1. 前置条件

1. 已完成本仓库编译（必须指定 OpenCV 4.8.0）：
   ```bash
   cd /home/lyx/ros2_vins
   source /opt/ros/humble/setup.bash

   # 编译依赖 OpenCV 的包，强制使用 4.8.0
   colcon build --symlink-install --parallel-workers 1 \
     --packages-select cv_bridge image_geometry camera_models vins loop_fusion global_fusion \
     --allow-overriding cv_bridge \
     --cmake-args \
       -DOpenCV_DIR=/home/lyx/.local/opencv-4.8.0/lib/cmake/opencv4 \
       -DCMAKE_BUILD_TYPE=Release
   ```
   > Jetson 7.4GB 内存有限，必须 `--parallel-workers 1` 单包顺序编译，否则容易 OOM。

2. 已编译 `realsense-ros`：
   ```bash
   ros2 pkg list | grep realsense2_camera
   ```

3. 必须使用 **USB 3.0/3.2** 接口。D435i 的 IMU 数据需要 USB 3 带宽。

4. 相机已插入，且 `rs-enumerate-devices` 能识别到设备。

5. 一键 launch 已固定红外曝光参数：
   - `depth_module.enable_auto_exposure:=false`
   - `depth_module.exposure:=20000`
   
   这是因为自动曝光下红外图像非常暗（平均灰度约 60/255），VINS 前端几乎提取不到特征点，导致初始化后迅速漂移。固定曝光后亮度提升到约 200/255，初始化稳定。

---

## 2. 配置文件位置

```
/home/lyx/ros2_vins/src/vins-fusion-jetson-humble/config/
├── realsense_d435i/                          # VO 模式
│   ├── realsense_stereo_imu_config.yaml      # imu: 0
│   ├── left.yaml                             # 640×480 实测内参
│   └── right.yaml
└── realsense_d435i_vio/                      # VIO 模式
    ├── realsense_stereo_imu_config.yaml      # imu: 1
    ├── left.yaml                             # 640×480 实测内参
    └── right.yaml
```

> **为什么用 640×480？** 降低分辨率可显著减少前端光流计算量，提高 Jetson 实时性。D435i 红外相机在 640×480 下的内参通过 `ros2 topic echo /camera/camera/infra1/camera_info` 实测获得。

### 2.1 内参（left.yaml / right.yaml）

```yaml
model_type: PINHOLE
image_width: 640
image_height: 480
distortion_parameters:
   k1: 0.0
   k2: 0.0
   p1: 0.0
   p2: 0.0
projection_parameters:
   fx: 391.0418701171875
   fy: 391.0418701171875
   cx: 319.01458740234375
   cy: 245.74755859375
```

畸变参数全部为 0，因为使用 `image_rect_raw`（已 rectified）。

> 内参通过 `ros2 topic echo /camera/camera/infra1/camera_info --once` 实测获得；若你更换了 D435i 设备或分辨率，请重新读取并更新 `left.yaml` / `right.yaml`。

### 2.2 VO 模式外参

VO 模式下 `body` 即左目相机，因此把 body 坐标系和左目相机坐标系重合：

```yaml
imu: 0
estimate_extrinsic: 0

body_T_cam0: I
body_T_cam1: I + [0.05017, 0, 0]   # 双目基线 50.17 mm
```

#### 坐标变换语义

`body_T_cam` 表示 **从 camera 坐标系到 body 坐标系** 的刚体变换：

```text
p_body = body_R_cam * p_cam + body_t_cam
p_cam  = body_R_cam^T * (p_body - body_t_cam)
```

- `body_T_cam0 = I`：body 与左目相机 cam0 重合
- `body_T_cam1.t = [0.05017, 0, 0]`：右目相机 cam1 原点在 body/cam0 坐标系中的坐标，即 cam1 在 cam0 右侧 50.17 mm

#### 关于 `estimate_extrinsic`

源码中 `parameters.cpp` 会强制把 VO 模式（`imu: 0`）下的 `ESTIMATE_EXTRINSIC` 设为 0：

```cpp
if(!USE_IMU)
{
    ESTIMATE_EXTRINSIC = 0;
    ESTIMATE_TD = 0;
    printf("no imu, fix extrinsic param; no time offset calibration\n");
}
```

因此 VO 模式下无论 YAML 里写多少，外参都不会被在线优化，必须给定准确的 `body_T_cam0` / `body_T_cam1`。

基线来源：
```bash
ros2 topic echo /camera/camera/infra2/camera_info --once
# p[0,3] = -19.6193
# baseline = |p[0,3]| / fx = 19.6193 / 391.0419 ≈ 0.05017 m
```

---

### 2.3 VIO 模式外参

D435i 的 IMU 数据经 librealsense SDK 内部已转换到 camera frame，因此旋转部分使用单位阵，平移部分从出厂标定读取：

```bash
# IMU → 左目
ros2 topic echo /camera/camera/extrinsics/depth_to_accel --once
# translation: [-0.00552, 0.00510, 0.01174]

# 左目 → 右目
ros2 topic echo /camera/camera/extrinsics/depth_to_infra2 --once
# translation: [-0.05017, 0, 0]
```

推导：
```yaml
body_T_cam0.t = [-0.00552, 0.00510, 0.01174]
body_T_cam1.t = body_T_cam0.t - depth_to_infra2.t
              = [0.04465, 0.00510, 0.01174]
```

#### 坐标变换语义

与 VO 模式相同，`body_T_cam` 表示 **从 camera 坐标系到 body（IMU）坐标系** 的刚体变换。源码中的证据：

```cpp
// src/factor/projection_factor.cpp
Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;       // 点在 cam-i 坐标系
Eigen::Vector3d pts_imu_i    = qic * pts_camera_i + tic; // cam -> body
Eigen::Vector3d pts_w        = Qi * pts_imu_i + Pi;     // body -> world
```

其中 `qic`/`tic` 就是 `body_R_cam` / `body_t_cam`。世界坐标系下的相机位姿由两者组合得到：

```text
world_T_cam_i = world_T_body * body_T_cam_i
```

#### 在线外参估计

VIO 模式下 `estimate_extrinsic` 的取值含义：

| 值 | 含义 |
|---|------|
| `0` | 完全信任 YAML 中给定的 `body_T_cam`，优化时固定不变 |
| `1` | 以 YAML 中的值为初始猜测，滑窗初始化成功后开始在线优化 |
| `2` | 没有任何先验，在线从运动中标定旋转，成功后自动切换到 `1` 继续 refine |

当前配置使用 `estimate_extrinsic: 1`，VINS 会在滑窗填满且速度足够（`Vs[0].norm() > 0.2`）后解锁外参优化。

#### 估计结果保存位置

优化后的外参会写入 `output_path + "/extrinsic_parameter.csv"`（源码：`src/utility/visualization.cpp:99-119`）。注意该文件虽然后缀是 `.csv`，但实际是 OpenCV YAML 格式，内容示例：

```yaml
%YAML:1.0
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ ... ]
body_T_cam1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ ... ]
```

运行结束后可以把它复制回配置文件，并将 `estimate_extrinsic` 改为 `0` 以获得更稳定的位姿输出。

---

## 3. 一键启动（推荐）

已新增 launch 文件 `vins/launch/realsense_d435i_vins.launch.py`，同时启动 RealSense 和 VINS，输出话题统一带 `/vins_estimator/` 前缀。

### 3.1 VO 模式

```bash
cd /home/lyx/ros2_vins
source setup_vins_env.sh

pkill -9 -f realsense2_camera_node
sleep 2

ros2 launch vins realsense_d435i_vins.launch.py \
  config_path:=/home/lyx/ros2_vins/src/vins-fusion-jetson-humble/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

### 3.2 VIO 模式（推荐）

```bash
cd /home/lyx/ros2_vins
source setup_vins_env.sh

pkill -9 -f realsense2_camera_node
sleep 2

ros2 launch vins realsense_d435i_vins.launch.py \
  config_path:=/home/lyx/ros2_vins/src/vins-fusion-jetson-humble/config/realsense_d435i_vio/realsense_stereo_imu_config.yaml
```

启动成功后应看到：
```
USE_IMU: 0   # VO 模式
# 或
USE_IMU: 1   # VIO 模式
waiting for image and imu...
```

如果上次运行后 RealSense 未正常关闭，加 `initial_reset:=true`：
```bash
ros2 launch vins realsense_d435i_vins.launch.py \
  config_path:=.../realsense_d435i_vio/realsense_stereo_imu_config.yaml \
  initial_reset:=true
```

---

## 4. 手动启动（调试用）

### 4.1 启动 RealSense

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false \
  enable_depth:=false \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2 \
  enable_sync:=true \
  depth_module.infra_profile:="640,480,30"
```

> VO 模式可以省略 `enable_gyro`、`enable_accel`、`unite_imu_method`。

### 4.2 启动 VINS

手动启动时，**必须**先导出 OpenCV 4.8.0 库路径：

```bash
export LD_LIBRARY_PATH=/home/lyx/.local/opencv-4.8.0/lib:$LD_LIBRARY_PATH
```

VO：
```bash
ros2 run vins vins_node \
  /home/lyx/ros2_vins/src/vins-fusion-jetson-humble/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

VIO：
```bash
ros2 run vins vins_node \
  /home/lyx/ros2_vins/src/vins-fusion-jetson-humble/config/realsense_d435i_vio/realsense_stereo_imu_config.yaml
```

> 手动启动时 VINS 输出话题**不带** `/vins_estimator/` 前缀（launch 文件通过 remapping 统一加前缀）。

---

## 5. 验证

### 5.1 检查话题

```bash
ros2 topic list | grep -E "vins|infra|imu"
ros2 topic hz /vins_estimator/odometry
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/imu        # VIO 模式
```

### 5.2 查看输出

```bash
ros2 topic echo /vins_estimator/odometry --once
```

移动相机验证坐标方向（VIO 模式）：
- 往**右**移动 → `position.x` 增大
- 往**前**移动 → `position.y` 增大
- 往**上**移动 → `position.z` 增大

VO 模式坐标系不同：
- 往**右**移动 → `position.x` 增大
- 往**下**移动 → `position.y` 增大
- 往**前**移动 → `position.z` 增大

### 5.3 RViz2 可视化

```bash
ros2 launch vins vins_rviz.launch.py
```

常用话题：
| 话题 | 说明 |
|------|------|
| `/vins_estimator/odometry` | 实时位姿 |
| `/vins_estimator/path` | 轨迹 |
| `/vins_estimator/point_cloud` | 地图点 |
| `/vins_estimator/image_track` | 特征跟踪图 |

---

## 6. 回环检测 Loop Fusion（可选）

在 VINS 已正常运行后，新终端启动。Loop Fusion 同样链接到 OpenCV 4.8.0，需要设置库路径：

```bash
cd /home/lyx/ros2_vins
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=/home/lyx/.local/opencv-4.8.0/lib:$LD_LIBRARY_PATH

ros2 run loop_fusion loop_fusion_node \
  /home/lyx/ros2_vins/src/vins-fusion-jetson-humble/config/realsense_d435i_vio/realsense_stereo_imu_config.yaml
```

Loop Fusion 发布：
- `/odometry_rect`：回环修正后的位姿
- `/pose_graph_path`：全局轨迹

---

## 7. 常见问题

### Q1: `waiting for image and imu...` 没反应

- 检查图像和 IMU 是否正常发布：
  ```bash
  ros2 topic hz /camera/camera/infra1/image_rect_raw
  ros2 topic hz /camera/camera/imu
  ```
- 检查配置文件中的话题名是否匹配：
  ```yaml
  imu_topic: "/camera/camera/imu"
  image0_topic: "/camera/camera/infra1/image_rect_raw"
  image1_topic: "/camera/camera/infra2/image_rect_raw"
  ```

### Q2: VIO 初始化失败 / 轨迹发散

- 确保 `unite_imu_method:=2`
- 确保 `enable_sync:=true`
- 保持 `estimate_extrinsic: 1` 和 `estimate_td: 1`
- 手持相机充分运动（平移+旋转），静止无法初始化

### Q3: `RS2_USB_STATUS_BUSY`

每次启动前清理残留进程：
```bash
pkill -9 -f realsense2_camera_node
sleep 2
```

### Q4: `Motion Module force pause`

- 确保 `enable_color:=false`
- D435i 的 IMU 与彩色流共享内部资源，同时开启会暂停 IMU

### Q5: VINS 启动后立即 `Segmentation fault`（段错误）

绝大多数是 OpenCV 版本混用导致。请检查：

```bash
# 1. vins_node 是否只链接到 OpenCV 4.8.0
source install/setup.bash
export LD_LIBRARY_PATH=/home/lyx/.local/opencv-4.8.0/lib:$LD_LIBRARY_PATH
ldd $(ros2 pkg prefix vins)/lib/vins/vins_node | grep opencv
# 应看到 libopencv_*.so.408，不应有 libopencv_*.so.4.5d

# 2. 若仍有 4.5d，重新编译并指定 OpenCV 4.8.0
colcon build --symlink-install --parallel-workers 1 \
  --packages-select cv_bridge image_geometry camera_models vins loop_fusion global_fusion \
  --allow-overriding cv_bridge \
  --cmake-args \
    -DOpenCV_DIR=/home/lyx/.local/opencv-4.8.0/lib/cmake/opencv4 \
    -DCMAKE_BUILD_TYPE=Release
```

---

## 8. 快速指令速查

| 模式 | 一键启动命令 |
|------|-------------|
| **VO** | `ros2 launch vins realsense_d435i_vins.launch.py config_path:=.../realsense_d435i/realsense_stereo_imu_config.yaml` |
| **VIO** | `ros2 launch vins realsense_d435i_vins.launch.py config_path:=.../realsense_d435i_vio/realsense_stereo_imu_config.yaml` |
| **Loop Fusion** | `ros2 run loop_fusion loop_fusion_node .../realsense_d435i_vio/realsense_stereo_imu_config.yaml` |
| **RViz** | `ros2 launch vins vins_rviz.launch.py` |

---

## 参考

- `src/vins-fusion-jetson-humble/vins_config_reference/USAGE_D435i_humble_jetson_ubuntu2204.md`
- `src/vins-fusion-jetson-humble/vins_config_reference/tutorial/realsense_extrinsic_calibration_guide.md`
- `src/vins-fusion-jetson-humble/vins_config_reference/VINS_World_Coordinate_System.md`
