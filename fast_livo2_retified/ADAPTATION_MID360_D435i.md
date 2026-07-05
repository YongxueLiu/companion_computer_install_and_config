# FAST-LIVO2-ROS2 适配文档：Mid-360 + RealSense D435i

> 本文档记录将 `FAST-LIVO2-ROS2` 从默认的 Mid-360 + 鱼眼相机配置，适配到 **Mid-360 + RealSense D435i（左红外）** 的全过程。

---

## 1. 目标与环境

### 1.1 目标

在本地机器上跑通 `FAST-LIVO2-ROS2`，输入：
- **LiDAR + IMU**: Livox Mid-360（来自 `~/ros2_fast_lio2`，已跑通）
- **Camera**: Intel RealSense D435i 左红外图像（来自 `~/ros2_vins`，已跑通）

输出：
- `/aft_mapped_to_init`：融合里程计
- `/path`：轨迹
- `/rgb_img`：VIO 跟踪可视化图像
- `/cloud_registered`、`/Laser_map` 等：点云

### 1.2 本机环境

| 工作空间 | 内容 | 状态 |
|---|---|---|
| `~/ros2_fast_lio2` | `FAST_LIO_ROS2` + `livox_ros_driver2` + `Livox-SDK2` | 已编译并可用 Mid-360 运行 |
| `~/ros2_vins` | `realsense2_camera` + `vins-fusion-jetson-humble` | 已编译并可用 D435i 运行 |
| `~/ros2_fast_livo2` | `FAST-LIVO2` + `FAST-LIVO2-ROS2` | 适配目标工作空间 |

系统：Ubuntu 22.04 + ROS2 Humble，ARM64 (Jetson)。

---

## 2. 关键问题分析

在动手前发现的几个核心问题：

1. **`FAST-LIVO2-ROS2` 默认硬编码为鱼眼相机模型**
   - `include/vio.h` 使用 `vikit/equidistant_camera.h`
   - D435i `infra1/image_rect_raw` 是已 rectified 的针孔图像，必须支持 `PinholeCamera`

2. **已知 ROS2 迁移 bug**
   - `src/voxel_map.cpp` 把 ROS1 参数 `lio/max_iterations` 错误迁移为 `lio.min_iterations`
   - 导致 YAML 中的 `lio.max_iterations` 被忽略，硬编码为 5

3. **缺少依赖**
   - 工作空间没有 `vikit_common` / `vikit_ros`
   - 系统没有 `Sophus`（`FAST_LIO_ROS2` 不依赖它，但 `FAST-LIVO2-ROS2` 需要）

4. **包名冲突**
   - `src/FAST-LIVO2` 和 `src/FAST-LIVO2-ROS2` 都声明包名为 `fast_livo`
   - `colcon` 报重复包名错误

5. **OpenCV ABI 风险**
   - VINS 使用私有的 OpenCV 4.8.0，系统 `cv_bridge` 链接系统 OpenCV 4.5d
   - 构建时会有混链警告，运行时通过 `LD_LIBRARY_PATH` 强制加载 4.8.0

---

## 3. 修改内容清单

### 3.1 修复 bug

#### 3.1.1 `src/voxel_map.cpp`

将 ROS2 参数名从错误的 `lio.min_iterations` 改回 `lio.max_iterations`：

```cpp
// 修改前
try_declare.template operator()<int>("lio.min_iterations", 5);
node->get_parameter("lio.min_iterations", voxel_config.max_iterations_);

// 修改后
try_declare.template operator()<int>("lio.max_iterations", 5);
node->get_parameter("lio.max_iterations", voxel_config.max_iterations_);
```

#### 3.1.2 `include/common_lib.h`

原 ROS2 移植漏掉了 `<deque>`，导致编译失败。补上：

```cpp
#include <deque>
```

---

### 3.2 让 VIO 支持针孔 + 鱼眼双相机模型

#### 3.2.1 `include/vio.h`

- 增加 `#include <vikit/pinhole_camera.h>`
- 同时保留 `PinholeCamera` 和 `EquidistantCamera` 指针：

```cpp
vk::EquidistantCamera *equidistant_cam = nullptr;
vk::PinholeCamera *pinhole_cam = nullptr;
```

#### 3.2.2 `src/vio.cpp`

在 `initializeVIO()` 开头根据 `vikit` 实际加载的相机模型自动识别：

```cpp
pinhole_cam = dynamic_cast<vk::PinholeCamera*>(cam);
equidistant_cam = dynamic_cast<vk::EquidistantCamera*>(cam);
if (!pinhole_cam && !equidistant_cam)
{
  RCLCPP_ERROR(rclcpp::get_logger("vio"), "Camera model is neither Pinhole nor Equidistant; VIO will fail.");
}
```

COLMAP 相机输出改为根据实际模型写：

```cpp
if (pinhole_cam)
  fout_camera << "1 PINHOLE ";
else if (equidistant_cam)
  fout_camera << "1 EQUIDISTANT ";
else
  fout_camera << "1 UNKNOWN ";
```

`dumpDataForColmap()` 中去畸变改为分支调用：

```cpp
if(equidistant_cam) {
  equidistant_cam->undistortImage(img_rgb, img_rgb_undistort);
} else if (pinhole_cam) {
  pinhole_cam->undistortImage(img_rgb, img_rgb_undistort);
} else {
  RCLCPP_WARN(...);
  img_rgb_undistort = img_rgb.clone();
}
```

---

### 3.3 新增配置文件

#### 3.3.1 `config/camera_d435i_pinhole.yaml`

D435i 左红外相机内参（640×480，已 rectified，畸变为 0）：

```yaml
/**:
  ros__parameters:
    camera:
      model: Pinhole
      width: 640
      height: 480
      scale: 1.0
      fx: 391.0418701171875
      fy: 391.0418701171875
      cx: 319.01458740234375
      cy: 245.74755859375
      k1: 0.0
      k2: 0.0
      p1: 0.0
      p2: 0.0
```

> `vikit_ros/include/vikit/camera_loader.h` 只识别字符串 `"Pinhole"`，原写法 `PinholeCamera` 会导致节点初始化失败并抛出 `Camera model not correctly specified.`。

> 参数来自 `~/ros2_vins/src/vins-fusion-jetson-humble/config/realsense_d435i_vio/left.yaml`

#### 3.3.2 `config/mid360_d435i.yaml`

FAST-LIVO2 节点主参数，关键项：

```yaml
common:
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"
  img_topic: "/camera/camera/infra1/image_rect_raw"
  img_en: 1
  lidar_en: 1
  enable_image_processing: false

preprocess:
  lidar_type: 7        # MID360
  scan_line: 4
  filter_size_surf: 0.2

lio:
  max_iterations: 3
  voxel_size: 0.3

uav:
  gravity_align_en: true

extrin_calib:
  # LiDAR -> Mid-360 IMU，来自 FAST_LIO_ROS2 mid360.yaml
  extrinsic_T: [-0.011, -0.02329, 0.04412]
  extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]
  # LiDAR -> D435i left camera，参考 fast_livo2_retified 的标定值
  Rcl: [-0.005228, -0.998807,  0.048561,
         0.018604, -0.048650, -0.998643,
         0.999813, -0.004318,  0.018836]
  Pcl: [0.029013, -0.150000, -0.095000]
```

完整文件见 `src/FAST-LIVO2-ROS2/config/mid360_d435i.yaml`。

---

### 3.4 新增启动文件

#### 3.4.1 `launch/mapping_mid360_d435i.launch.py`

一键启动：
1. `livox_ros_driver2` 节点（Mid-360 驱动，直接以 `sensor_msgs/PointCloud2` 输出，因为 FAST-LIVO2 的 MID360 分支订阅的是 PointCloud2）
2. `realsense2_camera::rs_launch.py`（D435i，只开左红外 + IMU）
3. `fast_livo::fastlivo_mapping`（本节点）
4. 可选 `rviz2`

RealSense 参数与 VINS 保持一致：

```python
{
    "enable_color": False,
    "enable_depth": False,
    "enable_infra1": True,
    "enable_infra2": False,
    "enable_gyro": True,
    "enable_accel": True,
    "unite_imu_method": 2,
    "enable_sync": True,
    "depth_module.infra_profile": "640,480,30",
    "depth_module.enable_auto_exposure": False,
    "depth_module.exposure": 10000,
}
```

> 注意：
> - 默认 `msg_MID360_launch.py` 使用 `xfer_format=1`（`livox_ros_driver2/CustomMsg`），但 FAST-LIVO2 的 MID360 分支订阅的是 `sensor_msgs/PointCloud2`，因此我们的启动文件直接启动 `livox_ros_driver2_node` 并设置 `xfer_format=0`。
> - Livox 的配置文件仍使用 `share/livox_ros_driver2/config/MID360_config.json`。

---

### 3.5 解决包名冲突

`src/FAST-LIVO2`（ROS1 版本）与 `src/FAST-LIVO2-ROS2` 包名均为 `fast_livo`。在 ROS1 版本目录下放置：

```bash
touch src/FAST-LIVO2/COLCON_IGNORE
```

使 `colcon` 忽略它。

---

## 4. 依赖安装

### 4.1 Sophus 1.22.10

安装到用户目录 `/home/lyx/.local`，避免 sudo：

```bash
cd /tmp
git clone https://github.com/strasdat/Sophus.git -b 1.22.10
cd Sophus
mkdir build && cd build
cmake .. -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF \
         -DSOPHUS_USE_BASIC_LOGGING=ON -DCMAKE_INSTALL_PREFIX=/home/lyx/.local
make -j1
make install
```

### 4.2 vikit_ros2_fisheye

```bash
cd ~/ros2_fast_livo2/src
git clone https://github.com/Rhymer-Lcy/rpg_vikit_ros2_fisheye.git
cp -r rpg_vikit_ros2_fisheye/vikit_common .
cp -r rpg_vikit_ros2_fisheye/vikit_ros .
rm -rf rpg_vikit_ros2_fisheye
```

### 4.3 livox_ros_driver2

不需要重新编译/复制，直接复用 `~/ros2_fast_lio2/install` 作为 underlay。

---

## 5. 构建命令

```bash
cd ~/ros2_fast_livo2
source /opt/ros/humble/setup.bash
source ~/ros2_fast_lio2/install/setup.bash
source ~/ros2_vins/setup_vins_env.sh
export CMAKE_PREFIX_PATH=/home/lyx/.local:${CMAKE_PREFIX_PATH}

colcon build --symlink-install --packages-select vikit_common vikit_ros \
  --parallel-workers 1 \
  --cmake-args -DOpenCV_DIR=/home/lyx/.local/opencv-4.8.0/lib/cmake/opencv4

colcon build --symlink-install --packages-select fast_livo \
  --parallel-workers 1 \
  --cmake-args -DOpenCV_DIR=/home/lyx/.local/opencv-4.8.0/lib/cmake/opencv4
```

构建成功后：

```bash
ros2 pkg executables fast_livo
# 应输出：fast_livo fastlivo_mapping
```

---

## 6. 运行命令

### 6.1 启动所有节点

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_fast_lio2/install/setup.bash
source ~/ros2_vins/setup_vins_env.sh
source ~/ros2_fast_livo2/install/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib

ros2 launch fast_livo mapping_mid360_d435i.launch.py use_rviz:=True
```

### 6.2 检查数据流

```bash
ros2 topic hz /livox/lidar /livox/imu /camera/camera/infra1/image_rect_raw
ros2 topic echo /aft_mapped_to_init
```

---

## 7. 已知问题与注意事项

### 7.1 LiDAR-camera 外参

`Rcl`/`Pcl` 已填入 `fast_livo2_retified/mid360_d435i.yaml` 中的参考标定值（旋转来自 run_1782119262 init，平移来自机械测量）。**该外参仍可能需要根据实际安装微调；若 VIO 跟踪不稳定，建议重新标定。**

建议后续：
1. 先用 `img_en: 0` 跑纯 LIO，确认 Mid-360 部分稳定。
2. 使用 [FAST-Calib](https://github.com/hku-mars/FAST-Calib) 或其他 LiDAR-camera 标定工具获取更精确的 `T_lidar_camera`。
3. 结合已知的 `T_lidar_imu` 换算出 `T_imu_camera`，更新 `mid360_d435i.yaml` 的 `Rcl`/`Pcl`。

### 7.2 时间同步

Mid-360 和 D435i 使用独立时钟，没有硬件同步。如果 `sync_packages()` 频繁丢包，需要调整：

```yaml
time_offset:
  imu_time_offset: 0.0
  img_time_offset: 0.0
  lidar_time_offset: 0.0
```

### 7.3 OpenCV 版本冲突

构建时会有如下警告：

```
/usr/bin/ld: warning: libopencv_imgcodecs.so.4.5d, needed by /opt/ros/humble/lib/libcv_bridge.so, may conflict with libopencv_imgcodecs.so.408
```

运行时通过 `source ~/ros2_vins/setup_vins_env.sh` 确保 `LD_LIBRARY_PATH` 指向 OpenCV 4.8.0。

### 7.4 图像话题

当前使用 `/camera/camera/infra1/image_rect_raw`（灰度、已 rectified）。如果想改用 D435i 彩色相机 `/camera/camera/color/image_raw`，需要：
1. 重新标定或获取 color 相机内参
2. 新建对应的 `camera_d435i_color.yaml`
3. 修改 `mid360_d435i.yaml` 中的 `img_topic`

---

## 8. 文件变更总结

### 修改的文件

| 文件 | 变更 |
|---|---|
| `src/FAST-LIVO2-ROS2/src/voxel_map.cpp` | `lio.min_iterations` → `lio.max_iterations` |
| `src/FAST-LIVO2-ROS2/include/common_lib.h` | 增加 `#include <deque>` |
| `src/FAST-LIVO2-ROS2/include/vio.h` | 增加 `pinhole_camera.h`，保留双相机模型指针 |
| `src/FAST-LIVO2-ROS2/src/vio.cpp` | 运行时根据相机模型选择 Pinhole/Equidistant |
| `src/FAST-LIVO2-ROS2/config/camera_d435i_pinhole.yaml` | 模型名 `PinholeCamera` → `Pinhole`（vikit 只识别 `"Pinhole"`） |
| `src/FAST-LIVO2-ROS2/config/mid360_d435i.yaml` | 填入参考 LiDAR-camera 外参；同步 `filter_size_surf`（按小房间调为 0.2）、`voxel_size`、`lio.max_iterations`、`gravity_align_en` |
| `src/FAST-LIVO2-ROS2/launch/mapping_mid360_d435i.launch.py` | 直接启动 `livox_ros_driver2_node`，设置 `xfer_format=0`（PointCloud2），匹配 FAST-LIVO2 MID360 订阅类型 |
| `src/FAST-LIVO2/COLCON_IGNORE` | 忽略 ROS1 版本，避免包名冲突 |

### 新增的文件

| 文件 | 用途 |
|---|---|
| `src/FAST-LIVO2-ROS2/config/camera_d435i_pinhole.yaml` | D435i 左红外针孔内参 |
| `src/FAST-LIVO2-ROS2/config/mid360_d435i.yaml` | Mid-360 + D435i 主配置 |
| `src/FAST-LIVO2-ROS2/launch/mapping_mid360_d435i.launch.py` | 一键启动 |

### 安装到系统/用户目录的依赖

| 依赖 | 位置 |
|---|---|
| Sophus 1.22.10 | `/home/lyx/.local` |
| vikit_common / vikit_ros | `~/ros2_fast_livo2/src/` |

---

## 9. 后续推荐步骤

1. **标定 LiDAR-camera 外参**（最关键）。
2. **纯 LIO 验证**：先设 `img_en: 0` 跑几分钟，确认 `/aft_mapped_to_init` 稳定。
3. **打开 VIO**：恢复 `img_en: 1`，观察 `/rgb_img` 是否正常跟踪。
4. **时间对齐**：如果日志提示 sync 失败，打印三个传感器的时间戳差，调 `time_offset`。
5. **场地测试**：在纹理丰富、结构化的环境中测试，避免白墙/天空等无纹理区域。

---

*文档生成时间：2026-07-02*
