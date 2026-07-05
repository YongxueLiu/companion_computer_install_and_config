# FAST-LIVO2 ROS2 on Jetson Orin Nano — 部署与调测记录

> **⚠️ 注意：应用户要求，仅 `livox_ros_driver2` 中的时间戳相关改动（`pub_handler.cpp` / `pub_handler.h`）已在 2026-06-17 复原。其余使系统能跑起来的改动（`fast_livo` CMakeLists / LIVMapper MID360 CustomMsg 订阅、新增 config/launch 文件、`MID360_config.json` 网络配置）仍然保留。**

> 本文档记录把 FAST-LIVO2-ROS2 在 Jetson Orin Nano (aarch64, Ubuntu 22.04, ROS2 Humble) 上跑起来，并接入 Livox MID360 + RealSense D435i 的全过程、踩坑与源码改动。
> 
> 状态（复原前）：LIO-only 已可稳定出 `/aft_mapped_to_init` 里程计；VIO 已能进入视觉 EKF 流程，但 **Livox IMU 与 LiDAR 时间戳存在持续漂移**，需要继续修复。

---

## 1. 硬件/软件环境

| 项目 | 配置 |
|------|------|
| 平台 | NVIDIA Jetson Orin Nano |
| 架构 | aarch64 |
| OS | Ubuntu 22.04 |
| ROS | ROS2 Humble |
| 传感器 | Livox MID360（网口） + RealSense D435i（USB3） |

### 1.1 工作空间（source 顺序很重要）

```bash
source /home/lingzhilab/ws_livox/install/setup.bash    # livox_ros_driver2, FAST-LIO2
source /home/lingzhilab/vins/install/setup.bash        # realsense2_camera, VINS-Fusion-ROS2
source ~/ws_livo/install/setup.bash                    # fast_livo, vikit_common, vikit_ros, Sophus
```

### 1.2 相关目录

- 本记录仓库：`/home/lingzhilab/fast_liov/`
- FAST-LIVO2-ROS2 源码：`/home/lingzhilab/fast_liov/src/FAST-LIVO2-ROS2/`（已软链到 `~/ws_livo/src/fast_livo`）
- Livox 驱动源码：`/home/lingzhilab/ws_livox/src/livox_ros_driver2/`
- vikit（ROS2 fisheye 分支）：`~/ws_livo/src/rpg_vikit_ros2_fisheye/`
- 本地 Sophus：`~/ws_livo/install/sophus/`

---

## 2. 已完成的工作

### 2.1 构建 `fast_livo`

1. 创建 `~/ws_livo/src/`，把 `FAST-LIVO2-ROS2` 软链为 `fast_livo`。
2. 克隆 `rpg_vikit_ros2_fisheye` 到 `~/ws_livo/src/`。
3. 从源码编译安装 Sophus 1.22.10 到 `~/ws_livo/install/sophus`（避免系统 `/usr/local` 需要 sudo）。
4. 修复 `fast_livo/CMakeLists.txt`：原仓库硬编码了 `../../install/vikit_*.so`，改为使用导出的 CMake target：
   ```cmake
   target_link_libraries(laser_mapping 
       vikit_common::vikit_common
       vikit_ros::vikit_ros
       ${COMMON_DEPENDENCIES}
   )
   ```
5. 在 Jetson 上 Release 编译成功。

### 2.2 新增配置文件

- `config/camera_d435i.yaml`：Pinhole 640×480，使用 RealSense 已 rectify 的 `infra1/image_rect_raw`。
- `config/mid360_d435i.yaml`：
  - 订阅 `/camera/camera/infra1/image_rect_raw`、`/livox/lidar`、`/livox/imu`
  - `lidar_type: 7`（MID360）
  - `enable_image_processing: false`（用原始灰度图）
  - 粗略 LiDAR↔Camera 外参 `Rcl`/`Pcl`
- `launch/mapping_mid360_d435i.launch.py`：统一启动 Livox MID360 + RealSense D435i + FAST-LIVO2。

### 2.3 Livox 驱动调整

- 修改 `msg_MID360_launch.py`：`xfer_format = 1`，让 `/livox/lidar` 发布 `livox_ros_driver2/msg/CustomMsg`。
- 修复 `MID360_config.json` 中 `host_net_info` 的 IP：原配置写的是 `192.168.1.5`，实际主机 IP 是 `192.168.1.50`，导致驱动 `Init lds lidar fail!`。已改为 `192.168.1.50`。
- 关键补丁：`src/comm/pub_handler.cpp` 中的 `GetEthPacketTimestamp()`。原实现只对 `gPTP/PTP/GPS` 同步模式使用硬件时间戳，否则回退到 `std::chrono::high_resolution_clock::now()`，导致 **IMU 走系统时钟、LiDAR 走设备时钟，二者相差约 42 秒**。

### 2.4 系统服务

- 禁用并停止 `mid360-full.service`，避免它与手动启动的 FAST-LIVO2 冲突（抢占 `/livox/lidar`、`/livox/imu`）。
- 保留 `mid360-network.service`，它负责把 `enP8p1s0` 配置为 `192.168.1.50/24`。

---

## 3. 当前源码改动详情

### 3.1 `fast_livo/CMakeLists.txt`

**改动前：**
```cmake
get_filename_component(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../.. ABSOLUTE)
set(VIKIT_COMMON_LIB ${PROJECT_ROOT}/install/vikit_common/lib/libvikit_common.so)
set(VIKIT_ROS_LIB    ${PROJECT_ROOT}/install/vikit_ros/lib/libvikit_ros.so)
...
target_link_libraries(laser_mapping 
    ${VIKIT_COMMON_LIB}
    ${VIKIT_ROS_LIB}
    ${COMMON_DEPENDENCIES}
)
```

**改动后：**
```cmake
target_link_libraries(laser_mapping 
    vikit_common::vikit_common
    vikit_ros::vikit_ros
    ${COMMON_DEPENDENCIES}
)
```

### 3.2 `~/ws_livox/src/livox_ros_driver2/src/comm/pub_handler.cpp`

#### 3.2.1 第一次补丁（解决 IMU/LiDAR 不同源）

原实现：
```cpp
if (timestamp_type == kTimestampTypeGptpOrPtp ||
    timestamp_type == kTimestampTypeGps) {
  return time.stamp;
}
return std::chrono::high_resolution_clock::now().time_since_epoch().count();
```

改为强制使用硬件时间戳：
```cpp
uint64_t PubHandler::GetEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp, uint8_t size) {
  LdsStamp time;
  memcpy(time.stamp_bytes, time_stamp, size);
  // Always use the LiDAR hardware timestamp so that LiDAR and IMU share the
  // same clock source even when PTP/gPTP/GPS sync is not enabled.
  return time.stamp;
}
```

**效果：**
- `/livox/lidar` 10 Hz，`/livox/imu` 200 Hz，数据正常。
- FAST-LIVO2 **LIO-only 模式 (`img_en: 0`) 成功初始化**，`/aft_mapped_to_init` 稳定输出 ~10 Hz。
- 但 `/livox/lidar` 与 `/livox/imu` 的 header stamp 仍在 Livox 自己的硬件时间域（从设备启动开始的秒数），与 RealSense 图像的系统时间域相差约 `1.78e9` 秒，导致 **VIO 无法同步图像与 LiDAR/IMU**。

#### 3.2.2 第二次补丁（对齐到系统时间 + 分别校准 IMU/LiDAR）

为了让 Livox 时间戳与 RealSense 图像在同一域，在 `pub_handler.cpp` 中增加辅助函数，对 IMU 和 LiDAR 分别计算 `offset = system_time - hardware_time`：

```cpp
#include <atomic>
...
std::atomic<uint64_t> PubHandler::imu_time_offset_ns_;
std::atomic<uint64_t> PubHandler::lidar_time_offset_ns_;

static uint64_t GetSystemTimeNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

static uint64_t GetAlignedTimestamp(uint8_t* time_stamp, uint8_t size,
                                    std::atomic<uint64_t>& offset_ns) {
  LdsStamp time;
  memcpy(time.stamp_bytes, time_stamp, size);
  uint64_t hw_ns = time.stamp;
  uint64_t expected_offset = 0;
  if (offset_ns.compare_exchange_strong(expected_offset, GetSystemTimeNs() - hw_ns)) {
    // First packet for this stream: stored the hardware-to-system offset.
  }
  return hw_ns + offset_ns.load();
}
```

调用处改为：
```cpp
// IMU
imu_data.time_stamp = GetAlignedTimestamp(data->timestamp, sizeof(data->timestamp),
                                          imu_time_offset_ns_);

// LiDAR
packet.time_stamp = GetAlignedTimestamp(data->timestamp, sizeof(data->timestamp),
                                        lidar_time_offset_ns_);
```

并在 `pub_handler.h` 中声明：
```cpp
static std::atomic<uint64_t> imu_time_offset_ns_;
static std::atomic<uint64_t> lidar_time_offset_ns_;
```

**效果：**
- LiDAR header stamp 接近 ROS 系统时间。
- RealSense 图像 header stamp 也是系统时间。
- **FAST-LIVO2 VIO 模式 (`img_en: 1`) 成功进入视觉 EKF 流程**，`/aft_mapped_to_init` 输出 ~9–10 Hz，日志出现 `retrieveFromVisualSparseMap`、`computeJacobianAndUpdateEKF` 等 VIO 阶段。
- 但产生新的问题：**IMU 与 LiDAR 时间戳存在持续漂移**，见第 4 节。

### 3.3 `~/ws_livox/src/livox_ros_driver2/config/MID360_config.json`

把 `host_net_info` 中所有 `192.168.1.5` 改为 `192.168.1.50`：
```json
"host_net_info" : {
  "cmd_data_ip" : "192.168.1.50",
  ...
  "push_msg_ip": "192.168.1.50",
  ...
  "point_data_ip": "192.168.1.50",
  ...
  "imu_data_ip" : "192.168.1.50",
  ...
}
```

### 3.4 `~/ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`

```python
xfer_format = 1    # CustomMsg
output_type = 0
publish_freq = 10.0
```

### 3.5 `~/ws_livo/src/fast_livo/config/mid360_d435i.yaml`

```yaml
common:
  img_topic: "/camera/camera/infra1/image_rect_raw"
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"
  img_en: 1
  lidar_en: 1
  enable_image_processing: false
extrin_calib:
  Rcl: [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0]
  Pcl: [0.05, 0.0, -0.05]
preprocess:
  lidar_type: 7
time_offset:
  imu_time_offset: 0.0
```

> `Rcl`/`Pcl` 目前为粗略估计，后续需用 FAST-Calib 或类似工具做精确 LiDAR-Camera 标定。

---

## 4. 仍未解决的问题

### 4.1 IMU 与 LiDAR 时间戳漂移（最高优先级）

现象：
- 在 VIO 模式下，`fastlivo_mapping` 日志持续打印：
  ```
  [WARN] IMU and LiDAR not synced! delta time: 19.99
  [WARN] IMU and LiDAR not synced! delta time: 20.01
  ...
  ```
- 漂移量会随运行时间增大，停止前观察到从 ~20 s 增加到 ~49 s。
- 具体数值：
  - LiDAR header time ≈ ROS 系统时间（如 `1781670889.4`）
  - IMU header time ≈ 系统时间 − 漂移量（如 `1781670840.2`）

根因分析：
1. Livox MID360 内部 LiDAR 与 IMU 的硬件时钟似乎不是同一个源头，或者驱动/SDK 层没有做好同步。
2. 分别把 IMU、LiDAR 的硬件时间戳对齐到系统时间后，二者之间的 **相对时钟差** 被保留下来，并随时间持续放大。
3. FAST-LIVO2 的 `sync_packages` 要求 IMU 时间覆盖 LiDAR scan 结束时间，过大的时间差导致：
   - 大量 `IMU and LiDAR not synced!` 警告；
   - IMU buffer 持续堆积（日志中 `imu size` 可达数万）；
   - 虽然 LIO/VIO 仍能部分运行，但 fusion 质量不可靠。

已尝试/待尝试方案：
- [x] 强制 Livox IMU/LiDAR 都使用硬件时间戳（解决 42 s 不同源问题）。
- [x] 分别把 IMU/LiDAR 硬件时间对齐到系统时间（解决与 RealSense 图像的域差异）。
- [ ] 在 Livox 驱动层统一 IMU/LiDAR 的 offset，例如：用 LiDAR 的 offset 校准 IMU，或在收到第一个 LiDAR packet 后重新对齐 IMU offset。
- [ ] 在 FAST-LIVO2 层尝试 `common.ros_driver_bug_fix: true`，该参数会把 IMU 时间戳按 `std::round(last_timestamp_lidar - timestamp)` 取整秒对齐，可能作为临时 workaround。
- [ ] 启用 Livox 的 PTP/gPTP 时间同步（需要外部 PTP master，如 Linux PTP）。
- [ ] 检查 Livox SDK 是否有更合适的 MID360 时间同步配置。

### 4.2 LiDAR-Camera 外参未标定

当前 `Rcl`/`Pcl` 为粗略手动值。VIO 能跑起来，但视觉重投影、地图颜色融合精度取决于精确外参。建议使用：
- FAST-Calib（若项目中有）
- 或 livox_camera_calib + 标定板

### 4.3 Livox 驱动退出段错误

`livox_ros_driver2_node` 在 SIGINT 时偶尔报 exit code `-11`（segfault）。目前看来是析构顺序问题，不影响正常运行，但不够干净。

### 4.4 RealSense `control_transfer returned error`

启动后 RealSense 节点会周期性打印 `control_transfer returned error`，但图像流正常。可通过更新 librealsense 固件/驱动缓解。

### 4.5 `pcl::VoxelGrid::applyFilter` 警告

日志中偶发：
```
[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.
```
通常是因为点云下采样 leaf size 相对于点云范围过小，或坐标系/外参导致点云坐标异常大。需检查 `filter_size_surf`/`filter_size_map` 以及 `Rcl`/`Pcl` 是否合理。

---

## 5. 关键日志解读

### 5.1 LIO-only 成功初始化
```
[ LIO ] Update Voxel Map
[ LIO ] Raw feature num: 19296, downsampled feature num:19296 effective feature num: 10394 average residual: 0.014
```
同时 `/aft_mapped_to_init` 有稳定输出。

### 5.2 VIO 进入视觉流程
```
| retrieveFromVisualSparseMap   | 0.004770                    |
| computeJacobianAndUpdateEKF   | 0.006356                    |
| generateVisualMapPoints       | 0.000611                    |
```
说明 VIO manager 已在对图像做前端跟踪与 EKF 更新。

### 5.3 IMU/LiDAR 不同步
```
[WARN] IMU and LiDAR not synced! delta time: 20.013895 .
[INFO] get imu at time: 1781670817.076549
[INFO] Get LiDAR, its header time: 1781670837.090445
```
`delta time = last_timestamp_lidar - timestamp_imu`。若持续 > 0.5 s，FAST-LIVO2 会 warn。

---

## 6. 复现步骤（当前）

### 6.1 单次启动

```bash
# 1. 确保 mid360-full.service 没抢话题
sudo systemctl stop mid360-full.service

# 2. source 工作空间
source /home/lingzhilab/ws_livox/install/setup.bash
source /home/lingzhilab/vins/install/setup.bash
source ~/ws_livo/install/setup.bash

# 3. 启动
ros2 launch fast_livo mapping_mid360_d435i.launch.py use_rviz:=False
```

### 6.2 仅测试 LIO

把 `~/ws_livo/src/fast_livo/config/mid360_d435i.yaml` 和 install 下的同名文件里的 `img_en: 1` 改为 `img_en: 0`，重新 `colcon build --packages-select fast_livo` 或直接改 install 配置后启动。

### 6.3 重建 fast_livo

```bash
cd ~/ws_livo
source /home/lingzhilab/ws_livox/install/setup.bash
source /home/lingzhilab/vins/install/setup.bash
source ~/ws_livo/install/setup.bash
colcon build --packages-select fast_livo --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/home/lingzhilab/ws_livo/install/sophus
```

### 6.4 重建 livox_ros_driver2

```bash
cd /home/lingzhilab/ws_livox
source install/setup.bash
colcon build --packages-select livox_ros_driver2 --cmake-args -DCMAKE_BUILD_TYPE=Release --allow-overriding livox_ros_driver2
```

---

## 7. 建议的下一步

1. **解决 IMU/LiDAR 时间漂移**
   - 首选：在 `livox_ros_driver2` 内部，让 IMU 时间戳以 LiDAR 为基准做二次对齐（保存各自 first packet 的硬件时间和系统时间，计算固定差值后统一偏移）。
   - 次选：开启 `common.ros_driver_bug_fix: true` 做整数秒 workaround，并观察 VIO 稳定性。
2. **精确标定 LiDAR-Camera 外参**。
3. **调参**：根据实际场景调整 `filter_size_surf`、`filter_size_map`、`vio` 相关参数、`img_time_offset` 等。
4. **处理 RealSense 控制传输错误** 和 Livox 驱动退出段错误，提升系统鲁棒性。

---

## 8. 参考文件清单

| 文件 | 说明 |
|------|------|
| `~/ws_livo/src/fast_livo/CMakeLists.txt` | 修复 vikit 链接 |
| `~/ws_livo/src/fast_livo/config/mid360_d435i.yaml` | FAST-LIVO2 主配置 |
| `~/ws_livo/src/fast_livo/config/camera_d435i.yaml` | D435i 相机内参 |
| `~/ws_livo/src/fast_livo/launch/mapping_mid360_d435i.launch.py` | 统一启动文件 |
| `~/ws_livox/src/livox_ros_driver2/src/comm/pub_handler.cpp` | 时间戳对齐补丁 |
| `~/ws_livox/src/livox_ros_driver2/src/comm/pub_handler.h` | 新增 static 成员声明 |
| `~/ws_livox/src/livox_ros_driver2/config/MID360_config.json` | host IP 修正 |
| `~/ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` | CustomMsg 配置 |
| `/etc/systemd/system/mid360-full.service` | 已禁用 |
| `/etc/systemd/system/mid360-network.service` | 仍启用 |

---

*记录时间：2026-06-17*
