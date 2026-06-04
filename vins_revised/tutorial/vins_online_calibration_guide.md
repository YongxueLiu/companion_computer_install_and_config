# VINS-Fusion D435i 在线外参标定完整指南

> 基于 VINS-Fusion `estimate_extrinsic` 功能，无需 Kalibr，直接在线优化相机-IMU 外参。

---

## 一、前置知识：D435i 坐标系与外参推导

### 1.1 关键发现——IMU 数据已在相机坐标系下

RealSense 官方文档明确说明：

> *"each IMU sample is multiplied internally by the extrinsic matrix"*  
> *"The resulting orientation angles and acceleration vectors share the coordinate system with the depth sensor"*

**验证方法**（相机平放静止时）：
```bash
ros2 topic echo /camera/camera/imu --once | grep linear_acceleration -A 3
# 预期输出: y ≈ -9.8（Y 轴向下，重力向下，读数为负）
```

**结论**：通过 `/camera/camera/imu` 获取的 IMU 数据，坐标轴和相机完全一致：
- **X-right, Y-down, Z-forward**（OpenCV 惯例）

这意味着 **IMU 和相机的旋转外参接近单位矩阵 I**，不是 Euroc 那种 -90° 大角度。之前用 Euroc 默认值直接导致标定发散到 1.2m。

---

### 1.2 外参推导——从 RealSense 出厂标定到 VINS

#### 1.2.1 读取出厂标定

RealSense D435i 的 IMU-相机外参已经预存在设备中，通过 ROS 话题读取：

```bash
# T_accel←depth: 从 depth(左目) 到 accel(IMU) 的变换
ros2 topic echo /camera/camera/extrinsics/depth_to_accel --once
```

输出示例：
```yaml
rotation:
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
translation:
- -0.005520000122487545
- 0.005100000184029341
- 0.011739999987185001
```

即：**R = I**, **t = [-0.00552, 0.00510, 0.01174] m**

#### 1.2.2 RealSense extrinsics 的数学定义

RealSense SDK 中，`depth_to_accel` 表示变换 **T_accel←depth**：

```
p_accel = R · p_depth + t
```

其中：
- `p_depth` = 点在 depth（左目相机）坐标系中的坐标
- `p_accel` = 同一点在 accel（IMU）坐标系中的坐标

#### 1.2.3 VINS body_T_cam0 的推导

VINS 配置中的 `body_T_cam0` 表示 **T_body←cam0**（从 cam0 到 body 的变换）。

验证：VINS 源码 `feature_manager.cpp` 中
```cpp
Eigen::Vector3d ptsInCam = ric[0] * point + tic[0];
// ric = R_body←cam, tic = t_body←cam
```

由于 body = IMU = accel，cam0 = depth：

```
T_body←cam0 = T_IMU←cam0 = T_accel←depth = [ I | t_da ]
```

直接代入出厂值：

```yaml
body_T_cam0:  R=I,  t=[-0.00552, 0.00510, 0.01174]
```

#### 1.2.4 VINS body_T_cam1 的推导

首先读取双目基线：

```bash
# T_infra2←depth: 从 depth(左目) 到 infra2(右目) 的变换
ros2 topic echo /camera/camera/extrinsics/depth_to_infra2 --once
```

输出：
```yaml
rotation:
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
- 0.0
- 0.0
- 0.0
- 1.0
translation:
- -0.05014605075120926
- 0.0
- 0.0
```

即 **T_cam1←cam0 = [ I | [-0.05015, 0, 0] ]**

这意味着：
```
p_cam1 = p_cam0 + [-0.05015, 0, 0]
```

反过来，**T_cam0←cam1**（cam0 原点在 cam1 坐标系中的坐标）：
```
p_cam0 = p_cam1 + [0.05015, 0, 0]
→ T_cam0←cam1 = [ I | [0.05015, 0, 0] ]
```

VINS 的 `body_T_cam1` = T_body←cam1：
```
T_body←cam1 = T_body←cam0 · T_cam0←cam1
            = [ I | t_body_cam0 ] · [ I | [0.05015, 0, 0] ]
            = [ I | t_body_cam0 + [0.05015, 0, 0] ]
            = [ I | [-0.00552+0.05015, 0.00510, 0.01174] ]
            = [ I | [0.04463, 0.00510, 0.01174] ]
```

验证基线：
```
baseline = t_body_cam1 - t_body_cam0 = [0.05015, 0, 0]
||baseline|| = 0.05015 m ≈ 5.0 cm
```

与 `camera_info` 中 `baseline = |P[0,3]|/fx = 19.211/384.6 = 0.04995m` 一致。

#### 1.2.5 汇总：正确的外参初值

| 矩阵 | 旋转 R | 平移 t (m) | 来源 |
|---|---|---|---|
| `body_T_cam0` | I | [-0.00552, 0.00510, 0.01174] | `depth_to_accel` 直接读取 |
| `body_T_cam1` | I | [0.04463, 0.00510, 0.01174] | `depth_to_accel` + `depth_to_infra2` 基线叠加 |

---

## 二、配置文件准备

编辑 `~/VINS-Fusion-ROS2/config/realsense_d435i_vio/realsense_stereo_imu_config.yaml`：

```yaml
# 在线优化外参（以上面推导的出厂值为初值进行微调）
estimate_extrinsic: 1

output_path: "/home/lyx/output/"

# ================================================================
# body_T_cam0 = T_IMU←cam0 = depth_to_accel
# ================================================================
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ 1.0, 0.0, 0.0, -5.5200001224875450e-03,
           0.0, 1.0, 0.0,  5.1000001840293407e-03,
           0.0, 0.0, 1.0,  1.1739999987185001e-02,
           0.0, 0.0, 0.0,  1.0 ]

# ================================================================
# body_T_cam1 = T_IMU←cam1 = T_IMU←cam0 + T_depth←cam1
#             = depth_to_accel + (-depth_to_infra2.translation)
# ================================================================
body_T_cam1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ 1.0, 0.0, 0.0,  4.4626050628721714e-02,
           0.0, 1.0, 0.0,  5.1000001840293407e-03,
           0.0, 0.0, 1.0,  1.1739999987185001e-02,
           0.0, 0.0, 0.0,  1.0 ]
```

> **如果你之前跑过发散的标定，先删旧结果**：
> ```bash
> rm ~/output/extrinsic_parameter.csv ~/output/vio.csv
> ```

---

## 三、启动步骤

### 3.1 终端 1：启动 RealSense

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

确认输出：
- `Device USB type: 3.2`
- `Infra(1), Format: Y8, Width: 640, Height: 480`
- `Infra(2), Format: Y8, Width: 640, Height: 480`
- `RealSense Node Is Up!`

### 3.2 终端 2：启动 VINS

```bash
cd ~/ros2_ws && source install/setup.bash

ros2 run vins vins_node \
  /home/lyx/VINS-Fusion-ROS2/config/realsense_d435i_vio/realsense_stereo_imu_config.yaml
```

等待出现：
```
Initialization finish!
```

> 如果一直 `waiting for image and imu...`，检查：
> ```bash
> ros2 topic hz /camera/camera/imu
> ros2 topic hz /camera/camera/infra1/image_rect_raw
> ```

---

## 四、标定运动方法（最关键）

`Initialization finish!` 出现后，**立刻开始运动**。

### 4.1 推荐运动序列（约 3~5 分钟）

| 阶段 | 动作 | 目的 | 持续时间 |
|---|---|---|---|
| 1 | **静止 3 秒** → 绕 Z 轴缓慢左右转头 | 激发偏航角旋转 | 30 秒 |
| 2 | **静止 3 秒** → 绕 X 轴缓慢抬头-低头 | 激发俯仰角旋转 | 30 秒 |
| 3 | **静止 3 秒** → 绕 Y 轴缓慢左右倾斜 | 激发横滚角旋转 | 30 秒 |
| 4 | **3D 八字形**：手持相机走动着画大 8 字 | 同时激发旋转+平移 | 60 秒 |
| 5 | 相机平行于桌面/地面，平移移动 | 激发纯平移（对 tic 敏感）| 30 秒 |
| 6 | 重复阶段 4（3D 八字形） | 增加样本量 | 60 秒 |

### 4.2 动作要点

1. **慢而稳**：运动太快会导致图像模糊、光流跟踪失败。**建议角速度约 30°/s**
2. **静止间隔**：每个动作之间静止 2~3 秒，让 VINS 重新三角化
3. **覆盖全空间**：尽量让相机朝向各个方向（上、下、左、右、前、后）
4. **平移不可少**：纯旋转只能标定 `ric`，**平移运动才能标定 `tic`**

### 4.3 禁忌动作

- ❌ 剧烈甩动（> 90°/s）：图像糊掉，前端跟踪失败
- ❌ 遮挡镜头：特征点丢失，初始化失败
- ❌ 纯旋转不移动：只能标定旋转外参，平移外参无法观测
- ❌ 重复同方向运动：信息矩阵缺秩，部分轴无法收敛

---

## 五、判断标定是否收敛

### 5.1 终端 2 观察 VINS 输出

关注以下几个信号：

```
# 好信号：初始化成功
Initialization finish!

# 好信号：优化正在进行
solver costs: X ms

# 好信号：IMU 积分正常（位置在小范围内波动）
time: X, t: 0.0XX 0.0XX 0.0XX

# 坏信号：图像跟踪丢失（偶尔出现正常，频繁出现说明运动太快）
throw img0

# 坏信号：IMU 激励不足（运动太慢或静止太久）
not enough IMU excitation
```

### 5.2 查看标定结果文件

外参实时写入 `~/output/extrinsic_parameter.csv`。**每隔 10~20 秒新开一个终端查看**：

```bash
cat ~/output/extrinsic_parameter.csv
```

**收敛标志**：连续查看 5 次以上，`body_T_cam0` 的数据**小数点后 4 位不再变化**。

示例（未收敛，还在变）：
```yaml
body_T_cam0: !!opencv-matrix
   data: [ 0.9999, 0.0046, 0.0042, -0.0108,
           -0.0047, 0.9999, 0.0107,  0.0104,
           -0.0042, -0.0108, 0.9999, 0.0269, ... ]
```

### 5.3 物理合理性检查

D435i 的 IMU 和左目物理距离约 **1~2 cm**，即 `||tic|| ≈ 0.01~0.02 m`。

| 检查项 | 合理范围 | 说明 |
|---|---|---|
| `\|tic\|` | 0.010 ~ 0.030 m | D435i IMU 靠近左目 |
| 旋转矩阵行列式 | ≈ 1.0 | 必须是合法旋转矩阵 |
| 俯仰/横滚角 | < 5° | IMU 和相机安装面基本平行 |

**如果 `|tic| > 0.1 m` 或旋转矩阵明显离谱，说明标定发散，立刻停止，检查外参初值。**

---

## 六、提取并比较标定结果

标定收敛后，提取结果并与初值比较：

```bash
python3 ~/VINS-Fusion-ROS2/scripts/extract_calibration_result.py
```

### 6.1 手动比较（如果不用脚本）

```bash
python3 -c "
import cv2
import numpy as np

# 读取标定结果
fs = cv2.FileStorage('/home/lyx/output/extrinsic_parameter.csv', cv2.FILE_STORAGE_READ)
T = fs.getNode('body_T_cam0').mat()
fs.release()

R, t = T[:3, :3], T[:3, 3]
print('Translation:', t)
print('|t| (m):', np.linalg.norm(t))
print('det(R):', np.linalg.det(R))
"
```

### 6.2 填入最终配置

把标定结果写入配置文件，然后把 `estimate_extrinsic` 改回 `0`：

```yaml
estimate_extrinsic: 0

body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ ... 填入标定结果 ... ]

body_T_cam1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ ... 填入标定结果 ... ]
```

---

## 七、完整速查流程

```bash
# 1. 清理旧结果
rm ~/output/extrinsic_parameter.csv ~/output/vio.csv

# 2. 确认配置文件 estimate_extrinsic: 1，且 body_T_cam0/cam1 用出厂值

# 3. 终端 1：启动 RealSense
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true enable_infra1:=true enable_infra2:=true \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2 \
  depth_module.infra_profile:=640x480x30 \
  enable_depth:=false enable_color:=false

# 4. 终端 2：启动 VINS
cd ~/ros2_ws && source install/setup.bash
ros2 run vins vins_node \
  /home/lyx/VINS-Fusion-ROS2/config/realsense_d435i_vio/realsense_stereo_imu_config.yaml

# 5. 等待 Initialization finish!，开始运动（3~5 分钟）

# 6. 终端 3：每隔 10~20 秒查看外参文件
cat ~/output/extrinsic_parameter.csv

# 7. 收敛后提取结果
python3 ~/VINS-Fusion-ROS2/scripts/extract_calibration_result.py

# 8. 填入配置，estimate_extrinsic 改 0，重启 VINS 验证
```

---

## 八、常见问题

### Q1: 标定过程中轨迹飞掉？
**A**: 外参初值错误。检查 `body_T_cam0` 的旋转是否接近单位矩阵 I（D435i 的 IMU 和相机坐标系方向一致）。

### Q2: `extrinsic_parameter.csv` 一直不变？
**A**: 运动幅度太小或太单一。加大 3D 八字形的幅度，确保同时有旋转和平移。

### Q3: 标定结果每次重启都不一样？
**A**: 正常。在线标定受运动轨迹影响。建议标定 2~3 次取平均，或选择最稳定的那个结果。

### Q4: 如何提高精度？
**A**: 
- 运动时间加长到 5~10 分钟
- 确保场景纹理丰富（不要对着白墙）
- 光照均匀，避免过曝/欠曝
- USB 3.2 必须，不要有其他高带宽设备抢占

### Q5: IMU 噪声参数需要标定吗？
**A**: 在线标定只能标外参（ric/tic），IMU 噪声参数（acc_n, gyr_n 等）需要单独用 Allan 方差标定。D435i 经验值：
```yaml
acc_n: 0.2
acc_w: 0.002
gyr_n: 0.02
gyr_w: 4.0e-5
```

---

*最后更新：2025-06-04*
