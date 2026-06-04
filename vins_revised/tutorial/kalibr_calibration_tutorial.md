# Kalibr 标定教程 —— RealSense D435i 双目 + IMU 联合标定

> **目标**：使用 Kalibr 标定工具箱，获取精确的 D435i 内参、外参和 IMU 噪声参数，填入 VINS 配置文件。

---

## 1. Kalibr 简介

**Kalibr** 是 ETH Zurich（苏黎世联邦理工学院）开源的标定工具箱，支持：

| 功能 | 说明 |
|------|------|
| **多相机标定** | 单目、双目、多目相机联合标定（内参 + 相对外参） |
| **相机-IMU 联合标定** | IMU 与相机之间的精确旋转、平移和时间偏移 |
| **IMU 噪声标定** | 加速度计和陀螺仪的测量噪声、随机游走参数 |

**官网**：https://github.com/ethz-asl/kalibr

---

## 2. 环境准备

### 2.1 安装 Docker

Kalibr 原生基于 ROS1（catkin），与 ROS2 Rolling 存在依赖冲突。**推荐用 Docker 运行**。

```bash
# 安装 Docker
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER
newgrp docker

# 验证
docker --version
```

### 2.2 拉取 Kalibr Docker 镜像

```bash
# 官方预构建镜像（ROS Noetic + Kalibr）
docker pull stereolabs/kalibr:latest

# 或从源码构建（推荐，确保最新版）
git clone https://github.com/ethz-asl/kalibr.git
cd kalibr
bash build-docker.sh
```

### 2.3 安装 rosbags（ROS2 → ROS1 bag 转换工具）

Kalibr 只读 ROS1 bag（`.bag`），ROS2 bag（`.db3`）需要先转换。

```bash
pip3 install rosbags
```

---

## 3. 标定板准备

Kalibr 支持两种标定板：

### 方案 A：Aprilgrid（推荐）

Aprilgrid 是 Kalibr 推荐的高精度标定板，角点检测更鲁棒。

```bash
# 下载官方 Aprilgrid（6x6，40mm 大格，8mm 小格）
wget https://raw.githubusercontent.com/ethz-asl/kalibr/master/aprilgrid/pdf/april_6x6_40x40cm.pdf
```

打印要求：
- **A3 或 A2 尺寸**（A4 太小，检测角点不足）
- 用激光打印机，确保方格尺寸精确
- 贴在**平整的刚性板**上（木板、亚克力板），不能弯曲

### 方案 B：Checkerboard（传统棋盘格）

```bash
# 下载官方 Checkerboard（12x8 格，80mm 大格）
wget https://raw.githubusercontent.com/ethz-asl/kalibr/master/checkerboard/pdf/checkerboard_12x8_80x80mm.pdf
```

创建 YAML 描述文件 `checkerboard.yaml`：

```yaml
target_type: 'checkerboard'
targetCols: 12           # 内角点列数
targetRows: 8            # 内角点行数
rowSpacingMeters: 0.08   # 每个格子的行间距（m）
colSpacingMeters: 0.08   # 每个格子的列间距（m）
```

---

## 4. 数据采集

### 4.1 录制 ROS2 bag（VINS 实际运行的话题）

```bash
source /opt/ros/rolling/setup.bash
source ~/ros2_ws/install/setup.bash

# 创建输出目录
mkdir -p ~/kalibr_data
cd ~/kalibr_data

# 启动 RealSense（640x480，关闭深度和 RGB，只保留红外 + IMU）
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true \
  enable_infra1:=true enable_infra2:=true \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2 \
  depth_module.infra_profile:=640x480x30 \
  enable_depth:=false enable_color:=false
```

在**另一个终端**录制 bag：

```bash
cd ~/kalibr_data

ros2 bag record \
  /camera/camera/imu \
  /camera/camera/infra1/image_rect_raw \
  /camera/camera/infra2/image_rect_raw \
  -o d435i_calib
```

### 4.2 采集动作要求

录制时长：**约 2 分钟**（太短数据不够，太长计算太慢）。

**相机标定部分**（前 30 秒）：
- 缓慢移动相机，让标定板出现在画面各个角落
- 上下、左右、前后平移
- 绕三个轴轻微旋转（俯仰、横滚、偏航）
- 标定板占画面 1/4 ~ 1/2 为宜

**IMU 标定部分**（后 90 秒）：
- 手持相机做**三轴旋转**（类似画"8"字）
- 每个轴至少旋转 2 圈
- 动作要**平稳、持续**，不能突然抖动或静止
- 避免纯平移运动（IMU 需要角速度激励）

> ⚠️ **不要做的事**：
> - 动作太快（图像模糊，角点检测失败）
> - 静止不动（IMU 无法激励，外参不可观）
> - 只在一个平面运动（自由度不足）

停止录制：`Ctrl+C`

---

## 5. ROS2 bag → ROS1 bag 转换

Kalibr 只接受 ROS1 bag，需要用 `rosbags` 转换。

```bash
cd ~/kalibr_data

# 转换
trosbags-convert d435i_calib/ -o d435i_calib.bag

# 验证
rosbag info d435i_calib.bag
```

应看到类似输出：

```
topics:     /camera/imu                           12000 msgs    sensor_msgs/Imu
            /camera/infra1/image_rect_raw          1800 msgs    sensor_msgs/Image
            /camera/infra2/image_rect_raw          1800 msgs    sensor_msgs/Image
```

---

## 6. 运行标定

### 6.1 双目相机标定（可选，验证出厂内参）

如果你怀疑出厂内参不准，可以先跑相机标定：

```bash
cd ~/kalibr_data

docker run -it --rm \
  -v $(pwd):/data \
  stereolabs/kalibr:latest \
  bash -c "cd /data && kalibr_calibrate_cameras \
    --target checkerboard.yaml \
    --bag d435i_calib.bag \
    --topics /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw \
    --models pinhole-radtan pinhole-radtan \
    --approx-sync 0.05"
```

参数说明：
| 参数 | 含义 |
|------|------|
| `--target` | 标定板描述文件 |
| `--bag` | ROS1 bag 文件 |
| `--topics` | 左右目图像话题 |
| `--models` | 相机模型（`pinhole-radtan` = 针孔 + 径向/切向畸变）|
| `--approx-sync` | 时间戳近似同步容差（秒）|

输出文件：
- `d435i_calib-camchain.yaml` — 相机内参和双目外参

---

### 6.2 相机-IMU 联合标定（重点）

这是获取 VINS 外参的核心步骤。

**步骤 1：创建 IMU 描述文件 `imu.yaml`**

D435i 的 IMU 参数（粗略初始值，Kalibr 会优化）：

```yaml
rostopic: /camera/imu
update_rate: 200.0        # Hz

# Accelerometer
accelerometer_noise_density: 0.01   # m/s^2/sqrt(Hz)  (噪声连续时间)
accelerometer_random_walk: 0.001    # m/s^3/sqrt(Hz)  (随机游走)

# Gyroscope
gyroscope_noise_density: 0.005      # rad/s/sqrt(Hz)
gyroscope_random_walk: 0.0005       # rad/s^2/sqrt(Hz)
```

> 这些初始值来自 D435i BMI085 数据手册，Kalibr 会根据数据在线估计精确值。

**步骤 2：创建相机链描述文件 `camchain.yaml`**

用之前相机标定的结果，或直接用出厂值创建初始文件：

```yaml
cam0:
  camera_model: pinhole
  intrinsics: [384.6005, 384.6005, 316.4323, 239.2910]  # fx fy cx cy
  distortion_model: radtan
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]                # k1 k2 p1 p2
  resolution: [640, 480]
  rostopic: /camera/infra1/image_rect_raw

cam1:
  camera_model: pinhole
  intrinsics: [384.6005, 384.6005, 316.4323, 239.2910]
  distortion_model: radtan
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]
  resolution: [640, 480]
  rostopic: /camera/infra2/image_rect_raw
  T_cn_cnm1:                              # cam0 → cam1 的变换
    - [1.0, 0.0, 0.0, 0.04995]           # 双目基线 ~50mm
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
```

**步骤 3：运行联合标定**

```bash
cd ~/kalibr_data

docker run -it --rm \
  -v $(pwd):/data \
  stereolabs/kalibr:latest \
  bash -c "cd /data && kalibr_calibrate_imu_camera \
    --target checkerboard.yaml \
    --cam camchain.yaml \
    --imu imu.yaml \
    --bag d435i_calib.bag \
    --bag-from-to 5 110 \
    --approx-sync 0.05"
```

参数说明：
| 参数 | 含义 |
|------|------|
| `--target` | 标定板描述 |
| `--cam` | 相机链初始参数 |
| `--imu` | IMU 初始参数 |
| `--bag` | 数据 bag |
| `--bag-from-to` | 只使用 bag 中 5~110 秒的数据（跳过首尾抖动） |
| `--approx-sync` | 图像与 IMU 近似同步容差 |

---

## 7. 结果解析

标定完成后，当前目录会生成以下文件：

| 文件 | 内容 |
|------|------|
| `d435i_calib-imucam.yaml` | **IMU-相机联合标定结果**（外参 + 时间偏移） |
| `d435i_calib-imucam-imu.yaml` | **IMU 噪声标定结果**（acc_n, gyr_n, acc_w, gyr_w） |
| `d435i_calib-imucam-report.pdf` | 可视化报告（残差、协方差、重投影误差） |

### 7.1 外参结果（`d435i_calib-imucam.yaml`）

```yaml
transform:
  q_IC: [0.001, -0.004, 0.001, 1.000]   # 四元数：IMU → Camera0
  p_IC: [0.021, 0.008, 0.003]           # 平移 (m)：IMU → Camera0

time_offset:
  imu0: 0.003                             # 时间偏移 td = 3 ms
```

转换成 VINS 的 `body_T_cam0`（4×4 齐次矩阵）：
- $R$ = quat2mat(q_IC)
- $t$ = p_IC
- 填入 `body_T_cam0` 的 16 个数

### 7.2 IMU 噪声结果（`d435i_calib-imucam-imu.yaml`）

```yaml
imu0:
  accelerometer_noise_density: 0.0082     # → VINS: acc_n
  accelerometer_random_walk: 0.0009       # → VINS: acc_w
  gyroscope_noise_density: 0.0041         # → VINS: gyr_n
  gyroscope_random_walk: 0.0003           # → VINS: gyr_w
```

### 7.3 时间偏移

```yaml
time_offset:
  imu0: 0.003
```

→ VINS 配置中 `td: 0.003`

---

## 8. 填入 VINS 配置文件

用 Kalibr 结果更新 `realsense_d435i_vio/realsense_stereo_imu_config.yaml`：

### 8.1 更新 IMU 噪声参数

```yaml
acc_n: 0.0082           # Kalibr 标定结果
gyr_n: 0.0041
acc_w: 0.0009
gyr_w: 0.0003
```

### 8.2 更新外参 `body_T_cam0`

将 Kalibr 的四元数 + 平移转换为 4×4 矩阵。

例如 Kalibr 输出 q_IC = [x, y, z, w] = [0.001, -0.004, 0.001, 1.000]，p_IC = [0.021, 0.008, 0.003]：

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

q = [0.001, -0.004, 0.001, 1.000]  # x, y, z, w
p = [0.021, 0.008, 0.003]

Rmat = R.from_quat(q).as_matrix()  # 3x3
T = np.eye(4)
T[:3, :3] = Rmat
T[:3, 3] = p
print(T.flatten())
```

填入 YAML：

```yaml
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ R11, R12, R13, tx, R21, R22, R23, ty, R31, R32, R33, tz, 0., 0., 0., 1. ]
```

### 8.3 固定外参（不再在线优化）

标定精度足够高后，关闭在线优化：

```yaml
estimate_extrinsic: 0   # 信任 Kalibr 标定值，不再优化
estimate_td: 0          # 信任 Kalibr 时间偏移，不再优化
td: 0.003               # 填入 Kalibr 标定的时间偏移
```

> `estimate_extrinsic: 0` 的前提是 Kalibr 标定足够精确。如果标定环境不理想（光照差、运动不足），建议保持 `estimate_extrinsic: 1` 让 VINS 在线微调。

---

## 9. 常见问题

### Q1: Kalibr 报错 "not enough image corners"

**原因**：标定板在画面中出现太少，或角点检测失败。

**解决**：
- 打印更大的标定板（A3 以上）
- 确保光线充足，没有反光
- 移动更慢，减少运动模糊

### Q2: Kalibr 报错 "gyroscope gravity sensitivity"

**原因**：IMU 数据缺少足够的旋转激励。

**解决**：
- 录制时做更大幅度的旋转运动
- 每个轴至少旋转 2 圈
- 避免长时间静止

### Q3: `rosbags-convert` 报错

**原因**：缺少 `rosbags` 库或 bag 格式不支持。

**解决**：
```bash
pip3 install rosbags --upgrade
# 或使用 rosbags 命令行
rosbags-convert d435i_calib/ -o d435i_calib.bag
```

### Q4: 时间偏移标定不稳定

**原因**：图像帧率太低（< 20 Hz）或 IMU 频率不够。

**解决**：
- 使用 30 Hz 图像 + 200 Hz IMU
- 增加 `--approx-sync` 容差到 0.1

---

## 10. 总结

| 参数 | VINS 默认值 | Kalibr 标定后 | 精度提升 |
|------|-----------|--------------|---------|
| `acc_n` | 0.1 | ~0.008 | **12×** |
| `gyr_n` | 0.01 | ~0.004 | **2.5×** |
| `body_T_cam0` | 近似值 | 精确值 | **显著** |
| `td` | 0.00 | ~0.003 | **消除时滞** |

**建议**：
- 内参：直接用 D435i 出厂值（足够精确）
- 外参：先用 `estimate_extrinsic: 1` 跑 VIO，有时间再用 Kalibr 离线标定
- IMU 噪声：强烈建议用 Kalibr 标定，默认值和实际值可能差一个数量级
