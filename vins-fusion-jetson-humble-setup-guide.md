# VINS-Fusion Jetson Humble 适配完全指南

> 本文档记录了将 VINS-Fusion 适配到 NVIDIA Jetson (JetPack 6.0) + ROS 2 Humble 的完整过程，以及 RealSense D435i 的驱动配置与使用说明。
>
> 适配日期：2026-05-31  
> 目标平台：NVIDIA Jetson Orin/Xavier, JetPack 6.0 (L4T R36.4.7), Ubuntu 22.04, ROS 2 Humble

---

## 目录

1. [系统环境概览](#1-系统环境概览)
2. [安装系统依赖](#2-安装系统依赖)
   - 2.1 Ceres Solver
   - 2.2 Intel RealSense SDK (librealsense)
   - 2.3 ROS 2 Humble (预装确认)
3. [构建 RealSense ROS2 驱动](#3-构建-realsense-ros2-驱动)
4. [创建 vins-fusion-jetson-humble 项目](#4-创建-vins-fusion-jetson-humble-项目)
   - 4.1 项目来源与策略
   - 4.2 复制与标识更新
   - 4.3 ROS1 算法修复同步确认
5. [编译 VINS-Fusion](#5-编译-vins-fusion)
   - 5.1 编译参数选择（线程控制）
   - 5.2 重复包名处理
   - 5.3 逐个包编译过程
6. [配置文件适配](#6-配置文件适配)
   - 6.1 RealSense D435i 配置修改
   - 6.2 话题名映射
7. [运行与验证](#7-运行与验证)
   - 7.1 启动 RealSense 节点
   - 7.2 启动 VINS 节点
   - 7.3 启动 Loop Closure（可选）
   - 7.4 启动 RViz2 可视化
   - 7.5 验证结果
8. [已知问题与注意事项](#8-已知问题与注意事项)
9. [GPU 模式说明](#9-gpu-模式说明)
10. [附录：完整命令速查](#10-附录完整命令速查)

---

## 1. 系统环境概览

| 组件 | 版本 / 规格 |
|------|------------|
| 开发板 | NVIDIA Jetson (aarch64) |
| JetPack | R36.4.7 (JetPack 6.0+) |
| OS | Ubuntu 22.04.5 LTS (Jammy Jellyfish) |
| ROS 2 | Humble Hawksbill |
| CPU | 6-core ARM64 Cortex-A78AE |
| 内存 | 7.4 GB |
| OpenCV | 4.8.0 (JetPack 自带，无 CUDA 模块) |
| Eigen3 | 3.4.0 |
| GCC | 11.4.0 |
| CMake | 3.22+ |

### 工作区结构

```
ros2_vins/
├── src/
│   ├── VINS-Fusion/              # 原始 ROS1 上游（参考用，不编译）
│   ├── VINS-Fusion-ROS2/         # Foxy 时代 ROS2 移植版（参考用，不编译）
│   ├── vins-fusion-jetson-humble/ # 本项目：Humble + Jetson 适配版
│   ├── librealsense/             # Intel RealSense SDK v2.58.1
│   ├── realsense-ros/            # RealSense ROS2 wrapper v4.57.7
│   └── tutorial/                 # 本文档
├── build/
├── install/
└── log/
```

---

## 2. 安装系统依赖

### 2.1 Ceres Solver 2.1.0

**为什么必须安装？**  
VINS-Fusion 的核心优化器（滑动窗口 BA、IMU 预积分、边缘化）重度依赖 Ceres Solver。Jetson 的 apt 仓库中没有预编译的 arm64 版本，必须从源码构建。

#### 步骤 1：安装依赖包

```bash
sudo apt-get update
sudo apt-get install -y \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev
```

#### 步骤 2：下载源码

```bash
cd /tmp
wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.tar.gz
tar -xzf 2.1.0.tar.gz
cd ceres-solver-2.1.0
```

#### 步骤 3：CMake 配置

```bash
mkdir build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF
```

**关键参数说明：**
- `-DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF`：禁用测试和示例编译，大幅缩短编译时间（否则需要编译数十个测试用例，在 Jetson 上容易超时）
- `-DBUILD_SHARED_LIBS=ON`：构建动态链接库，方便 VINS 链接

#### 步骤 4：编译与安装

```bash
# Jetson 6 核 / 7.4GB 内存，使用 3 线程避免 OOM
make -j3
sudo make install
sudo ldconfig
```

**验证：**
```bash
ls /usr/local/lib/libceres*
# 应显示 libceres.so, libceres.so.2.1.0, libceres.so.3
```

> **踩坑记录：** 第一次编译时启用了测试（默认），后台任务在 15 分钟后超时中断（编译到 48% 的测试阶段）。重新配置 `-DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF` 后，编译在 2 分钟内完成。另外，手动拷贝库文件时遗漏了 `CeresConfig.cmake`，导致 `find_package(Ceres REQUIRED)` 报错。解决方式是重新运行 `make install`，确保 CMake 配置文件一并安装到 `/usr/local/lib/cmake/Ceres/`。

---

### 2.2 Intel RealSense SDK (librealsense) v2.58.1

**为什么从源码构建？**  
Jetson 的 apt 仓库中没有官方 librealsense2 包。工作区中已经包含了 v2.58.1 源码，直接构建即可。

#### 步骤 1：CMake 配置

```bash
cd /home/lyx/ros2_vins/src/librealsense
mkdir -p build && cd build

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/local
```

**关键参数说明：**
- `-DFORCE_RSUSB_BACKEND=ON`：**Jetson 必需**。使用纯用户态 USB 后端（RSUSB），避免内核模块 patch 的复杂性。JetPack 6.0 的内核（5.15.148-tegra）与 librealsense 的原生内核模块不兼容，RSUSB 是官方推荐的 Jetson 方案。
- `-DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF`：不构建示例工具和 GUI（Jetson 通常无显示器或不需要 Viewer）

#### 步骤 2：编译与安装

```bash
make -j3
sudo make install
sudo ldconfig
```

编译时间约 14 分钟（`-j3`）。

#### 步骤 3：安装 udev 规则

```bash
cd /home/lyx/ros2_vins/src/librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### 步骤 4：验证

```bash
rs-enumerate-devices --version
# 输出: rs-enumerate-devices version: 2.58.1.0
```

---

### 2.3 ROS 2 Humble（预装确认）

Jetson 上已预装 ROS 2 Humble，验证：
```bash
echo $ROS_DISTRO        # humble
which ros2              # /opt/ros/humble/bin/ros2
ros2 --version
```

---

## 3. 构建 RealSense ROS2 驱动

realsense-ros 的源码已在工作区中（`src/realsense-ros`，版本约 v4.57.7）。

### 构建命令

```bash
cd /home/lyx/ros2_vins
source /opt/ros/humble/setup.bash

colcon build \
    --packages-select realsense2_camera_msgs realsense2_camera realsense2_description \
    --symlink-install \
    --parallel-workers 3
```

> **注意：** `librealsense2` ROS 包（`src/librealsense/package.xml`）是 librealsense 的 ROS cmake 包装，但由于我们已经系统安装了 librealsense2，为了避免 colcon 重复编译整个 SDK，我们在 `src/librealsense` 目录下放置了 `COLCON_IGNORE` 文件跳过它。realsense2_camera 的 CMakeLists.txt 中直接使用 `find_package(realsense2 2.56.6)` 查找系统安装的库，不依赖 ROS 包形式的 librealsense2。

### 验证编译

```bash
source install/setup.bash
ros2 pkg list | grep realsense2_camera
# 应显示 realsense2_camera, realsense2_camera_msgs, realsense2_description
```

---

## 4. 创建 vins-fusion-jetson-humble 项目

### 4.1 项目来源与策略

**原始需求：** 将 ROS1 版本的 VINS-Fusion 适配到 Jetson + ROS 2 Humble。

**策略选择：**
- 工作区中已有两个参考版本：
  - `VINS-Fusion`：HKUST 原始 ROS1 版本（catkin / C++11）
  - `VINS-Fusion-ROS2`：社区移植的 ROS2 版本（最初为 Foxy 适配）

- **关键发现：** `VINS-Fusion-ROS2` 的 git 历史中已有 commit `fb32d2f "Updated to support ROS2 Humble"`，说明该仓库**已经原生支持 Humble**。

- **最终策略：** 以 `VINS-Fusion-ROS2` 为起点复制出新项目 `vins-fusion-jetson-humble`，再确认 ROS1 原版的最新算法修复是否已同步。这是工作量最小、风险最低的方案。

### 4.2 复制与标识更新

```bash
cd /home/lyx/ros2_vins/src
cp -r VINS-Fusion-ROS2 vins-fusion-jetson-humble
```

更新所有 `package.xml` 的描述信息，将占位符替换为 Jetson Humble 专用标识：

| 包名 | 更新内容 |
|------|---------|
| `vins` | description → "VINS-Fusion main estimator for ROS 2 Humble on Jetson" |
| `camera_models` | description → "Camera models library for VINS-Fusion Jetson Humble" |
| `loop_fusion` | description → "Loop closure for VINS-Fusion Jetson Humble" |
| `global_fusion` | description → "GPS fusion for VINS-Fusion Jetson Humble" |
| 全部 | maintainer → `jetson@example.com`, license → `GPLv3` |

更新 `README.md` 为 Jetson Humble 专用文档。

### 4.3 ROS1 算法修复同步确认

ROS1 原版 `VINS-Fusion` 有三个重要的后期修复：

| Commit | 描述 | 文件 |
|--------|------|------|
| `be55a93` | Memory issue fix：条件创建 IMU/stereo subscriber | `rosNodeTest.cpp` |
| `0c32069` | Extrinsic lock bug：Quaternion `.normalized()` | `estimator.cpp` |
| `ae69746` | Propagation issue fix：添加 `mPropagate` 锁，修复初始化后 IMU 传播竞争条件 | `estimator.cpp`, `estimator.h` |

**验证结果：** 以上三个修复在 `VINS-Fusion-ROS2` 中**已经全部同步**，无需额外 patch。

```bash
grep -n "mPropagate" vins/src/estimator/estimator.h      # 已存在
grep -n "normalized()" vins/src/estimator/estimator.cpp   # 已存在
grep -n "if(USE_IMU)" vins/src/rosNodeTest.cpp           # 已存在
```

---

## 5. 编译 VINS-Fusion

### 5.1 编译参数选择（线程控制）

Jetson 的瓶颈是 **内存（7.4GB）**，而非 CPU 核心数（6 核）。多次后台编译任务因内存不足或 timeout 导致系统崩溃。

**最终策略：**
- 单包顺序编译（`--parallel-workers 1`）
- 每个包内部使用 2-3 线程（Jetson 实际可用内存决定）

| 项目 | 推荐线程 | 原因 |
|------|---------|------|
| Ceres Solver | `-j3` | 禁用测试后编译很快 |
| librealsense | `-j3` | 大项目，7.4GB 内存紧张 |
| colcon build | `--parallel-workers 1` | 单包顺序，避免多包同时编译导致 OOM |

### 5.2 重复包名处理

工作区中存在三个同名包集合：
- `VINS-Fusion/vins_estimator`（包名 `vins`）
- `VINS-Fusion-ROS2/vins`（包名 `vins`）
- `vins-fusion-jetson-humble/vins`（包名 `vins`）
- 同理 `camera_models`, `loop_fusion`, `global_fusion`

colcon 不允许重复包名。解决方案：在不需要编译的包目录下放置 `COLCON_IGNORE`：

```bash
# 给参考版本放置 COLCON_IGNORE，只保留 vins-fusion-jetson-humble
for pkg in vins camera_models loop_fusion global_fusion; do
    touch src/VINS-Fusion-ROS2/$pkg/COLCON_IGNORE
done
touch src/VINS-Fusion/vins_estimator/COLCON_IGNORE
touch src/VINS-Fusion/camera_models/COLCON_IGNORE
touch src/VINS-Fusion/loop_fusion/COLCON_IGNORE
touch src/VINS-Fusion/global_fusion/COLCON_IGNORE
```

### 5.3 逐个包编译过程

```bash
cd /home/lyx/ros2_vins
source /opt/ros/humble/setup.bash
source install/setup.bash

# 1. camera_models（依赖最少）
colcon build --packages-select camera_models --symlink-install --parallel-workers 1

# 2. global_fusion（依赖 camera_models）
colcon build --packages-select global_fusion --symlink-install --parallel-workers 1

# 3. loop_fusion（依赖 camera_models）
colcon build --packages-select loop_fusion --symlink-install --parallel-workers 1

# 4. vins（主包，依赖以上全部 + OpenCV + Ceres）
colcon build --packages-select vins --symlink-install --parallel-workers 1
```

**编译结果：**
- `camera_models`：编译通过，有若干 C++11 → C++14 的 deprecation warning（不影响运行）
- `global_fusion`：编译通过
- `loop_fusion`：编译通过
- `vins`：编译通过（1 分 30 秒），有 `ConstPtr` deprecated warning（ROS2 Humble 的消息类型变化）

**验证可执行文件：**
```bash
ls install/vins/lib/vins/vins_node
ls install/loop_fusion/lib/loop_fusion/loop_fusion_node
ls install/global_fusion/lib/global_fusion/global_fusion_node
```

---

## 6. 配置文件适配

### 6.1 RealSense D435i 配置修改

原始配置文件：`src/vins-fusion-jetson-humble/config/realsense_d435i/realsense_stereo_imu_config.yaml`

**必须修改项：**

| 配置项 | 原始值 | 修改后 | 原因 |
|--------|--------|--------|------|
| `imu` | `0` | `1` | 启用 IMU 模式 |
| `imu_topic` | `/camera/imu` | `/camera/camera/imu` | RealSense ROS2 默认 namespace 为 `camera`，node name 为 `camera`，完整话题为 `/camera/camera/imu` |
| `image0_topic` | `/camera/infra1/image_rect_raw` | `/camera/camera/infra1/image_rect_raw` | 同上，添加 namespace |
| `image1_topic` | `/camera/infra2/image_rect_raw` | `/camera/camera/infra2/image_rect_raw` | 同上 |
| `output_path` | `/home/dji/output/` | `/home/lyx/output/` | 匹配当前用户 |
| `pose_graph_save_path` | `/home/dji/output/pose_graph/` | `/home/lyx/output/pose_graph/` | 同上 |

**创建输出目录：**
```bash
mkdir -p /home/lyx/output/pose_graph
```

### 6.2 相机标定参数说明

VINS-Fusion 需要左右相机内参（`left.yaml` / `right.yaml`）和 IMU-相机外参（`body_T_cam0` / `body_T_cam1`）。**当前默认启用的是修正版配置**，基于本机 RealSense D435i 实测值调整，原版作为备用。

**文件对应关系：**

| 文件 | 内容 | 用途 |
|------|------|------|
| `realsense_stereo_imu_config.yaml` | **修正版（默认）** | `body_T_cam1` 基线 49.95 mm |
| `realsense_stereo_imu_config_default.yaml` | 原版（GitHub 默认） | `body_T_cam1` 基线 60.86 mm |
| `left.yaml` / `right.yaml` | **实测值（默认）** | fx=384.60, cx=316.43, cy=239.29 |
| `left_default.yaml` / `right_default.yaml` | 原版（GitHub 默认） | fx=382.84, cx=317.48, cy=235.34 |

**默认配置（已启用）：**

`left.yaml` / `right.yaml`：
```yaml
projection_parameters:
   fx: 384.6004638671875
   fy: 384.6004638671875
   cx: 316.4323425292969
   cy: 239.29095458984375
```

`body_T_cam1`（修正版，基线 49.95 mm）：
```
┌                                                          ┐
│ -0.001002   0.000363   0.999999   0.016190 │
│ -0.999992  -0.003830  -0.001001  -0.041615 │
│  0.003830  -0.999993   0.000367   0.007647 │
│  0.000000   0.000000   0.000000   1.000000 │
└                                                          ┘
```

**原版配置（备用）：**

`left_default.yaml` / `right_default.yaml`：
```yaml
projection_parameters:
   fx: 382.83856201171875
   fy: 382.83856201171875
   cx: 317.4849853515625
   cy: 235.34381103515625
```

`body_T_cam1`（原版，基线 60.86 mm）：
```
┌                                                          ┐
│ -0.001002   0.000363   0.999999   0.015286 │
│ -0.999992  -0.003830  -0.001001  -0.052436 │
│  0.003830  -0.999993   0.000367   0.008693 │
│  0.000000   0.000000   0.000000   1.000000 │
└                                                          ┘
```

**差异对比：**

| 参数 | 原版 | 修正版 | 差异 |
|------|------|--------|------|
| fx | 382.84 | 384.60 | 1.76 px (0.46%) |
| cx | 317.48 | 316.43 | 1.05 px (0.33%) |
| cy | 235.34 | 239.29 | 3.95 px (1.65%) |
| 基线 | 60.86 mm | **49.95 mm** | **与 camera_info 一致** |

**为什么默认用修正版？**

1. **内参匹配本机设备**：`ros2 topic echo /camera/camera/infra1/camera_info` 读出的 `K` 矩阵与修正版一致
2. **基线匹配 camera_info**：从 `P[0,3] = -19.211` / `fx = 384.6005` 推算基线 ≈ **50 mm**，修正版基线 49.95 mm 与之吻合；原版 60.86 mm 偏差约 10 mm
3. **初始化更稳定**：精确的内参和基线有助于 VINS 初始化阶段快速收敛

**回退到原版的方法：**

如果修正版运行异常，修改 `realsense_stereo_imu_config.yaml` 中的两行：
```yaml
cam0_calib: "left_default.yaml"
cam1_calib: "right_default.yaml"
```
并将 `body_T_cam1` 替换为 `realsense_stereo_imu_config_default.yaml` 中的值。

**获取自己设备标定参数的方法：**
```bash
# 启动 RealSense 后，读取相机内参
ros2 topic echo /camera/camera/infra1/camera_info
# 查看 K 矩阵 [fx, 0, cx, 0, fy, cy, 0, 0, 1]
# 查看 P 矩阵 [fx, 0, cx, -fx * baseline, 0, fy, cy, 0, 0, 0, 1, 0]
```

**关键参数解读：**
- `image_width / image_height`：必须与 `depth_module.infra_profile` 中指定的分辨率一致（当前为 **640×480**）
- `distortion_parameters`：全部为 **0.0**（使用 `image_rect_raw`，已是去畸变后的图像）
- **基线（Baseline）**：从 `infra2/camera_info` 的 `P[0,3] = -19.211` 推算，基线 = `|P[0,3]| / fx` ≈ **50 mm**，与 D435i 官方规格一致

> ⚠️ **注意**：`body_T_cam0` / `body_T_cam1`（IMU-相机外参）**不能**直接从 `camera_info` 读取。修正版通过对默认值的 `body_T_cam1` 平移向量进行缩放，使基线从 60.86 mm 修正为 49.95 mm，旋转矩阵保持不变。`estimate_extrinsic: 1` 会进一步在线优化。

### 6.3 话题名映射

| VINS 配置中的话题 | RealSense 实际发布的话题 | 消息类型 |
|------------------|------------------------|---------|
| `/camera/camera/imu` | `/camera/camera/imu` | `sensor_msgs/msg/Imu` |
| `/camera/camera/infra1/image_rect_raw` | `/camera/camera/infra1/image_rect_raw` | `sensor_msgs/msg/Image` (Y8, 640x480@30Hz) |
| `/camera/camera/infra2/image_rect_raw` | `/camera/camera/infra2/image_rect_raw` | `sensor_msgs/msg/Image` (Y8, 640x480@30Hz) |

---

## 7. 运行与验证

### 7.1 启动 RealSense 节点

**推荐参数（双目红外 + IMU，关闭 color/depth）：**

```bash
cd /home/lyx/ros2_vins
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
    enable_color:=false \
    enable_depth:=false \
    enable_infra1:=true \
    enable_infra2:=true \
    enable_gyro:=true \
    enable_accel:=true \
    unite_imu_method:=2 \
    depth_module.infra_profile:="640,480,30"
```

**参数说明：**
- `enable_color:=false`：关闭 RGB 相机。D435i 在 Jetson 上同时开启 color + motion module 时，某些固件版本会触发 `Motion Module force pause` 硬件错误。
- `enable_depth:=false`：关闭深度计算，节省 USB 带宽和 GPU 资源。
- `enable_infra1:=true / enable_infra2:=true`：启用左右红外相机。VINS-Fusion 使用这两个作为双目输入。
- `enable_gyro:=true / enable_accel:=true`：启用陀螺仪和加速度计。
- `unite_imu_method:=2`：将 gyro (200Hz) 和 accel (100Hz) 融合为统一 IMU 消息 (200Hz)。
- `depth_module.infra_profile:="640,480,30"`：**强制指定红外相机分辨率为 640×480@30Hz**。⚠️ **必须使用逗号分隔**，若写成 `640x480x30` 会被忽略并回退到默认 848×480。降低分辨率可显著减少 Jetson 的 CPU 负载。

**验证 RealSense 话题：**
```bash
ros2 topic list | grep -E "infra|imu"
ros2 topic hz /camera/camera/imu
ros2 topic hz /camera/camera/infra1/image_rect_raw
```

### 7.2 启动 VINS 节点

在另一个终端中：

```bash
cd /home/lyx/ros2_vins
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run vins vins_node \
    src/vins-fusion-jetson-humble/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

**正常启动输出：**
```
[INFO] []: init begins
config_file: src/vins-fusion-jetson-humble/config/realsense_d435i/realsense_stereo_imu_config.yaml
USE_IMU: 1
IMU_TOPIC: /camera/camera/imu
[WARN] []:  Optimize extrinsic param around initial guess!
[INFO] []: Unsynchronized sensors, online estimate time offset, initial td: 0.000000
[INFO] []: ROW: 480 COL: 640
[INFO] []: frame_ids: world=world body=body camera=camera
camera number 2
 exitrinsic cam 0
  ... (4x4 外参矩阵)
 exitrinsic cam 1
  ... (4x4 外参矩阵)
set g     0     0 9.805
[INFO] []: reading paramerter of camera src/vins-fusion-jetson-humble/config/realsense_d435i/left.yaml
[INFO] []: reading paramerter of camera src/vins-fusion-jetson-humble/config/realsense_d435i/right.yaml
[WARN] []: waiting for image and imu...
```

当相机开始移动后，VINS 会进入初始化流程，输出 `Initialization finish!`。

### 7.3 启动 Loop Closure（可选）

```bash
ros2 run loop_fusion loop_fusion_node \
    src/vins-fusion-jetson-humble/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

### 7.4 启动 RViz2 可视化

```bash
ros2 launch vins vins_rviz.launch.py
```

### 7.5 验证结果

| 验证项 | 状态 |
|--------|------|
| RealSense 节点识别 D435I | ✅ Serial: 243122070322 |
| infra1 图像流 | ✅ 640x480@30Hz, Y8 |
| infra2 图像流 | ✅ 640x480@30Hz, Y8 |
| IMU 数据流 | ✅ 200Hz |
| VINS 节点启动 | ✅ 读取配置正常，等待数据 |
| loop_fusion 编译 | ✅ 可执行文件存在 |
| global_fusion 编译 | ✅ 可执行文件存在 |

---

## 8. 已知问题与注意事项

### 8.1 D435i Motion Module force pause

**现象：** RealSense 日志中出现：
```
Hardware Notification:Motion Module force pause, ... ,Error,Hardware Error
```

**原因：** D435i 的 Motion Module 与 Color Camera 同时启用时，在某些固件版本（如当前 5.17.0.10）和特定平台上会出现硬件级冲突。

**解决方案：**
- 关闭 color 流（`enable_color:=false`），只使用 infra1/infra2 + IMU
- 若必须使用 color，可尝试更新 D435i 固件到最新版本
- 该警告通常不影响 IMU 数据输出，RealSense 会自动恢复

### 8.2 USB control_transfer 警告

**现象：**
```
(messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: Resource temporarily unavailable
```

**原因：** Jetson 的 USB 控制器与 RealSense 的 bulk/control 传输交互时的正常行为。

**影响：** 通常不影响数据流传输，可忽略。若出现频繁掉流，可尝试更换 USB 端口或使用带供电的 USB Hub。

### 8.3 IMU Calibration 缺失

**现象：**
```
(ds-calib-parsers.cpp:36) IMU Calibration is not available, default intrinsic and extrinsic will be used.
```

**原因：** D435i 出厂时 IMU 内参存储在设备内部，但 librealsense 的 RSUSB 后端有时读取不到。

**影响：** VINS-Fusion 的 `estimate_extrinsic: 1` 配置会让优化器在线估计 IMU-Camera 外参，因此默认参数也可以工作。若需要更高精度，可使用 Intel 的 `rs-imu-calibration.py` 工具进行标定。

### 8.4 编译线程与内存

Jetson 7.4GB 内存在并行编译大项目时非常容易耗尽，导致编译进程被 OOM Killer 终止或系统无响应。

**推荐原则：**
- `make` 使用 `-j3`（不超过 3 线程）
- `colcon build` 使用 `--parallel-workers 1`（单包顺序编译）
- 避免在编译时同时运行其他内存密集型程序

### 8.5 USB 设备占用（RS2_USB_STATUS_BUSY）

**现象：** 启动 RealSense 节点时失败，日志中出现：
```
RS2_USB_STATUS_BUSY
```
或
```
control_transfer returned error, index: 768, error: Resource temporarily unavailable
```

**原因：** 之前的 realsense 进程未正确退出，仍占用 USB 设备。

**解决方案：**
每次启动 RealSense 前，先清理残留进程：
```bash
pkill -f realsense2_camera_node
sleep 2
ros2 launch realsense2_camera rs_launch.py ...
```

若仍然失败，检查并强制终止：
```bash
ps aux | grep realsense | grep -v grep
# 手动 kill -9 <PID>
pkill -9 -f "realsense2_camera"
```

### 8.6 VINS-Fusion 的 `vins/CMakeLists.txt` 硬编码路径

```cmake
include_directories("../camera_models/include")
```

该相对路径假设 `vins/` 和 `camera_models/` 在同一父目录下。若单独移动 `vins` 包会导致编译失败。

---

## 9. GPU 模式说明

### 当前状态：CPU 模式

VINS-Fusion 的 `GPU_MODE` 宏位于：
```
src/vins-fusion-jetson-humble/vins/src/featureTracker/feature_tracker.h
```

当前状态：
```cpp
// #define GPU_MODE 1   ← 已注释掉
```

### 为什么默认关闭 GPU？

JetPack 自带的 OpenCV 4.8.0 **没有编译 CUDA 模块**：
```python
import cv2
cv2.cuda.getCudaEnabledDeviceCount()  # 返回 0
```

VINS 的 GPU_MODE 依赖 OpenCV CUDA optical flow（`cv::cuda::OpticalFlowDual_TVL1`），需要 `opencv2/cudaoptflow.hpp` 头文件。当前系统缺少这些组件。

### 未来启用 GPU 的步骤

若后续需要 GPU 加速特征跟踪：

1. **重新编译 OpenCV with CUDA**
   ```bash
   # 需要先安装 CUDA Toolkit（当前系统有 CUDA 运行时但无 nvcc）
   sudo apt install cuda-toolkit-12-2   # 或对应 JetPack 版本的 CUDA
   
   # 然后重新编译 OpenCV
   cmake -DWITH_CUDA=ON -DCUDA_ARCH_BIN=8.7 ...  # Jetson Orin 的 SM 8.7
   ```

2. **取消注释 GPU_MODE**
   ```bash
   # 修改 feature_tracker.h 第 14 行
   #define GPU_MODE 1
   ```

3. **重新编译 VINS**
   ```bash
   colcon build --packages-select vins --symlink-install --parallel-workers 1
   ```

> **注意：** Jetson 上重编 OpenCV with CUDA 通常需要 1-2 小时，且需要确保 CUDA Toolkit 版本与 JetPack 兼容。

---

## 10. 附录：完整命令速查

### 环境初始化（每个终端必做）

```bash
cd /home/lyx/ros2_vins
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 一键编译 VINS（已编译过的可以跳过）

```bash
cd /home/lyx/ros2_vins
source /opt/ros/humble/setup.bash
colcon build \
    --packages-select camera_models global_fusion loop_fusion vins \
    --symlink-install --parallel-workers 1
```

### 完整运行流程（4 个终端）

**Terminal 1 - RealSense：**
```bash
cd /home/lyx/ros2_vins
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=false enable_depth:=false \
    enable_infra1:=true enable_infra2:=true \
    enable_gyro:=true enable_accel:=true unite_imu_method:=2 \
    depth_module.infra_profile:="640,480,30"
```

**Terminal 2 - VINS Estimator：**
```bash
cd /home/lyx/ros2_vins
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run vins vins_node \
    src/vins-fusion-jetson-humble/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

**Terminal 3 - Loop Closure（可选）：**
```bash
cd /home/lyx/ros2_vins
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run loop_fusion loop_fusion_node \
    src/vins-fusion-jetson-humble/config/realsense_d435i/realsense_stereo_imu_config.yaml
```

**Terminal 4 - RViz2：**
```bash
cd /home/lyx/ros2_vins
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vins vins_rviz.launch.py
```

### 话题检查

```bash
ros2 topic list | grep -E "infra|imu"
ros2 topic hz /camera/camera/imu
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/infra2/image_rect_raw
```

### 清理残留 RealSense 进程

```bash
ps aux | grep realsense | grep -v grep
# 若有残留，kill -9 <PID>
pkill -9 -f "realsense2_camera"
```

**推荐做法（启动前自动清理）：**
```bash
pkill -f realsense2_camera_node; sleep 2
ros2 launch realsense2_camera rs_launch.py ...
```

---

*文档结束。如有问题，检查各包的 `README.md` 或查看 ROS2 日志：`~/.ros/log/`*
