# 从 RealSense 出厂标定推导 VINS-Fusion `body_T_cam0` / `body_T_cam1`

> 本文档说明如何从 Intel RealSense D435i 的**出厂标定数据**（通过 ROS2 topic 读取），推导出 VINS-Fusion 配置文件中的 `body_T_cam0` 和 `body_T_cam1`。

---

## 1. 核心问题

VINS-Fusion 的 `body_T_cam` 参数定义了 **相机坐标系** 到 **IMU（body）坐标系** 的变换。对于 D435i，这个参数不能照搬 Euroc 数据集的默认值，必须从 RealSense 的出厂标定数据中推导。

---

## 2. 读取出厂标定数据

启动 RealSense 后，通过 ROS2 topic 读取两个关键的外参：

### 2.1 启动 RealSense

```bash
source /opt/ros/rolling/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_sync:=true \
  enable_infra1:=true enable_infra2:=true \
  enable_gyro:=true enable_accel:=true \
  unite_imu_method:=2 \
  depth_module.infra_profile:=640x480x30 \
  enable_depth:=false enable_color:=false
```

### 2.2 读取 IMU 到左目的外参

```bash
ros2 topic echo /camera/camera/extrinsics/depth_to_accel --once
```

**预期输出：**

```yaml
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: depth
rotation: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
translation:
  x: -0.005520000122487545
  y: 0.005100000184029341
  z: 0.011739999987185001
```

### 2.3 读取左目到右目的外参

```bash
ros2 topic echo /camera/camera/extrinsics/depth_to_infra2 --once
```

**预期输出：**

```yaml
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: depth
translation:
  x: -0.05014999955892563
  y: 0.0
  z: 0.0
```

> 注意：D435i 的左右红外相机（infra1/infra2）与 depth 流共享同一个光学模组，因此 `depth_to_infra2` 就是左右目的基线。

### 2.4 读取相机内参（验证分辨率）

```bash
# 左目
ros2 topic echo /camera/camera/infra1/camera_info --once
# 预期：k: [384.6005, 0, 316.4323, 0, 384.6005, 239.2910, 0, 0, 1]

# 右目
ros2 topic echo /camera/camera/infra2/camera_info --once
# 预期：p[0,3] = -19.2110
```

基线验证：

$$
\text{baseline} = \frac{|P[0,3]|}{f_x} = \frac{19.2110}{384.6005} \approx 0.04995 \text{ m} = 49.95 \text{ mm}
$$

---

## 3. librealsense Extrinsics 约定（关键！）

RealSense 的 `extrinsics` 消息遵循 librealsense 的标准约定：

> **`translation` 表示：source 坐标系的原点，在 destination 坐标系中的坐标。**

用数学语言表达，如果消息名为 `frame_A_to_frame_B`：

$$
\mathbf{p}_B = \mathbf{R} \cdot \mathbf{p}_A + \mathbf{t}
$$

其中：
- $\mathbf{p}_A$：点在 frame A 中的坐标
- $\mathbf{p}_B$：点在 frame B 中的坐标
- $\mathbf{t}$：**frame A 的原点在 frame B 中的坐标**
- $\mathbf{R}$：frame A 到 frame B 的旋转

### 应用到具体话题

| 话题名 | source | destination | translation 含义 |
|---|---|---|---|
| `depth_to_accel` | depth (= infra1 = 左目) | accel (= IMU) | **cam0 原点在 IMU 系中的坐标** |
| `depth_to_infra2` | depth (= infra1 = 左目) | infra2 (= 右目) | **cam0 原点在 cam1 系中的坐标** |

---

## 4. 推导 `body_T_cam0`

### 4.1 VINS-Fusion 中 `body_T_cam` 的定义

在 VINS-Fusion 源码中（`vins/src/estimator/parameters.cpp`）：

```cpp
cv::Mat cv_T;
fsSettings["body_T_cam0"] >> cv_T;
Eigen::Matrix4d T;
cv::cv2eigen(cv_T, T);
RIC.push_back(T.block<3, 3>(0, 0));  // 旋转
TIC.push_back(T.block<3, 1>(0, 3));  // 平移
```

使用时：

```cpp
// vins/src/estimator/feature_manager.cpp
t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
```

这表示：

$$
\mathbf{p}_{body} = \mathbf{R}_{bc} \cdot \mathbf{p}_{cam} + \mathbf{t}_{bc}
$$

其中：
- $\mathbf{R}_{bc}$：cam → body 的旋转
- $\mathbf{t}_{bc}$：**cam 原点在 body 系中的坐标**

### 4.2 从 `depth_to_accel` 提取

从 `ros2 topic echo` 输出：

```yaml
translation:
  x: -0.005520000122487545
  y: 0.005100000184029341
  z: 0.011739999987185001
```

根据 librealsense 约定：

$$
\mathbf{t}_{depth\to accel} = \text{cam0 原点在 IMU 系中的坐标}
$$

由于 VINS 的 body frame 就是 IMU frame，所以：

$$
\mathbf{t}_{bc} = \mathbf{t}_{depth\to accel} = [-0.00552,\ 0.00510,\ 0.01174]^T \text{ (m)}
$$

旋转矩阵 `rotation` 输出是 `[1,0,0, 0,1,0, 0,0,1]`（单位矩阵），表示 cam0 和 IMU 的坐标轴方向已通过 SDK 内部对齐。

因此：

```yaml
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ 1.0, 0.0, 0.0, -5.5200001224875450e-03,
           0.0, 1.0, 0.0,  5.1000001840293407e-03,
           0.0, 0.0, 1.0,  1.1739999987185001e-02,
           0., 0., 0., 1. ]
```

### 4.3 物理意义验证

| 分量 | 值 | 含义 |
|---|---|---|
| x = -5.52 mm | 负 | cam0 在 body 的**左侧**（body X = right） |
| y = +5.10 mm | 正 | cam0 在 body 的**下方**（body Y = down） |
| z = +11.74 mm | 正 | cam0 在 body 的**后方**（body Z = forward） |

这与 RealSense D435i 的机械结构一致：IMU（BMI085）芯片位于左目相机的右上方、前方。

---

## 5. 推导 `body_T_cam1`

### 5.1 从 `depth_to_infra2` 提取基线

从 `ros2 topic echo` 输出：

```yaml
translation:
  x: -0.05014999955892563
  y: 0.0
  z: 0.0
```

根据 librealsense 约定：

$$
\mathbf{t}_{depth\to infra2} = \text{cam0 原点在 cam1 系中的坐标} = [-0.05015,\ 0,\ 0]^T \text{ (m)}
$$

这意味着：**cam0 在 cam1 左侧 5.015 cm**，反过来就是 **cam1 在 cam0 右侧 5.015 cm**。

### 5.2 数学推导

我们知道：
- `body_T_cam0` 把 cam0 中的点转换到 body 系：$\mathbf{p}_{body} = \mathbf{R}_{bc0} \mathbf{p}_{c0} + \mathbf{t}_{bc0}$
- `body_T_cam1` 把 cam1 中的点转换到 body 系：$\mathbf{p}_{body} = \mathbf{R}_{bc1} \mathbf{p}_{c1} + \mathbf{t}_{bc1}$

同时，`depth_to_infra2` 把 cam0 中的点转换到 cam1 系：

$$
\mathbf{p}_{c1} = \mathbf{R}_{c0\to c1} \mathbf{p}_{c0} + \mathbf{t}_{c0\to c1}
$$

其中：
- $\mathbf{R}_{c0\to c1} = \mathbf{I}$（左右目光轴平行，无旋转）
- $\mathbf{t}_{c0\to c1} = [-0.05015,\ 0,\ 0]^T$

将 $\mathbf{p}_{c1}$ 代入 `body_T_cam1` 的公式：

$$
\mathbf{p}_{body} = \mathbf{R}_{bc1} (\mathbf{R}_{c0\to c1} \mathbf{p}_{c0} + \mathbf{t}_{c0\to c1}) + \mathbf{t}_{bc1}
$$

而直接通过 `body_T_cam0`：

$$
\mathbf{p}_{body} = \mathbf{R}_{bc0} \mathbf{p}_{c0} + \mathbf{t}_{bc0}
$$

令两式相等（对任意 $\mathbf{p}_{c0}$）：

$$
\mathbf{R}_{bc1} \mathbf{R}_{c0\to c1} = \mathbf{R}_{bc0}
$$

$$
\mathbf{R}_{bc1} \mathbf{t}_{c0\to c1} + \mathbf{t}_{bc1} = \mathbf{t}_{bc0}
$$

解得：

$$
\mathbf{R}_{bc1} = \mathbf{R}_{bc0} \mathbf{R}_{c0\to c1}^{-1} = \mathbf{I} \cdot \mathbf{I} = \mathbf{I}
$$

$$
\mathbf{t}_{bc1} = \mathbf{t}_{bc0} - \mathbf{R}_{bc1} \mathbf{t}_{c0\to c1} = \mathbf{t}_{bc0} - \mathbf{t}_{c0\to c1}
$$

代入数值：

$$
\mathbf{t}_{bc1} = [-0.00552,\ 0.00510,\ 0.01174]^T - [-0.05015,\ 0,\ 0]^T
$$

$$
\mathbf{t}_{bc1} = [0.04463,\ 0.00510,\ 0.01174]^T \text{ (m)}
$$

### 5.3 验证

```python
import numpy as np

t_bc0 = np.array([-0.00552, 0.00510, 0.01174])
t_c0_to_c1 = np.array([-0.05015, 0, 0])
t_bc1 = t_bc0 - t_c0_to_c1
print(t_bc1)  # [0.04463, 0.0051, 0.01174]
```

因此：

```yaml
body_T_cam1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ 1.0, 0.0, 0.0, 4.4626050628721714e-02,
           0.0, 1.0, 0.0, 5.1000001840293407e-03,
           0.0, 0.0, 1.0, 1.1739999987185001e-02,
           0., 0., 0., 1. ]
```

### 5.4 物理意义验证

| 分量 | 值 | 含义 |
|---|---|---|
| x = +44.63 mm | 正 | cam1 在 body 的**右侧**（比 cam0 更靠右） |
| y = +5.10 mm | 正 | cam1 在 body 的**下方**（与 cam0 相同） |
| z = +11.74 mm | 正 | cam1 在 body 的**后方**（与 cam0 相同） |

cam0 与 cam1 的 x 差：$0.04463 - (-0.00552) = 0.05015$ m = 5.015 cm，与基线一致 ✅

---

## 6. 常见误区

### 误区 1：认为 `translation` 是 destination 原点在 source 中的坐标

**错误理解：** `depth_to_accel.translation` = IMU 原点在 cam0 系中的坐标。

**正确理解：** `depth_to_accel.translation` = **cam0 原点在 IMU 系中的坐标**。

如果反过来用，会把 `body_T_cam0.t` 的符号弄反，导致 VIO 初始化失败或发散。

### 误区 2：直接把 `depth_to_infra2.translation` 当作 `body_T_cam1.t`

**错误做法：** `body_T_cam1.t = depth_to_infra2.t = [-0.05015, 0, 0]`

**正确做法：** `body_T_cam1.t = body_T_cam0.t - depth_to_infra2.t = [0.04463, ...]`

`depth_to_infra2` 描述的是 **cam0 到 cam1** 的相对关系，不是 **body 到 cam1** 的绝对关系。必须先通过 `body_T_cam0` 建立 body 和 cam0 的关联，再叠加基线。

### 误区 3：认为 `body_T_cam0` 的旋转不应该是单位矩阵

D435i 的 IMU 芯片（BMI085）和相机传感器的坐标轴方向确实不同（见下表）：

| 轴 | 红外相机 (infra) | IMU (BMI085) |
|---|---|---|
| X | right（右）| forward（前）|
| Y | down（下）| left（左）|
| Z | forward（前）| up（上）|

但由于 **librealsense SDK 内部已自动将 IMU 数据转换到相机坐标系**，ROS 发布的 `/camera/camera/imu` 已经使用 camera frame（X-right, Y-down, Z-forward）。因此 `body_T_cam0` 的旋转是单位矩阵，不需要额外的坐标旋转。

如果把 Euroc 的 ~90° 旋转矩阵搬过来，会导致 IMU 数据被**二次投影**，VIO 必然发散。

---

## 7. 完整速查表

| 步骤 | 命令 | 输出字段 | 对应 VINS 参数 |
|---|---|---|---|
| 1 | `ros2 topic echo /camera/camera/extrinsics/depth_to_accel --once` | `rotation`, `translation` | `body_T_cam0` |
| 2 | `ros2 topic echo /camera/camera/extrinsics/depth_to_infra2 --once` | `translation` | 用于推导 `body_T_cam1` |
| 3 | `ros2 topic echo /camera/camera/infra1/camera_info --once` | `k[0]` (fx), `k[2]` (cx) | `left.yaml` |
| 4 | `ros2 topic echo /camera/camera/infra2/camera_info --once` | `p[0,3]` (Tx) | 验证基线 |

---

## 8. 配置文件模板

```yaml
# ================================================================
# body_T_cam0 (IMU → 左目相机)
# ================================================================
# 数据来源：ros2 topic echo /camera/camera/extrinsics/depth_to_accel
#   rotation: [1,0,0, 0,1,0, 0,0,1]  (SDK 内部已对齐)
#   translation: [-0.00552, 0.00510, 0.01174]  (cam0 原点在 IMU 系中的坐标)
# ================================================================
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ 1.0, 0.0, 0.0, -5.5200001224875450e-03,
           0.0, 1.0, 0.0,  5.1000001840293407e-03,
           0.0, 0.0, 1.0,  1.1739999987185001e-02,
           0., 0., 0., 1. ]

# ================================================================
# body_T_cam1 (IMU → 右目相机)
# ================================================================
# 数据来源：body_T_cam0 + depth_to_infra2
#
# librealsense 约定：
#   depth_to_infra2.translation = cam0 原点在 cam1 系中的坐标
#
# 推导：
#   body_T_cam1.t = body_T_cam0.t - depth_to_infra2.translation
#                = [-0.00552, 0.00510, 0.01174] - [-0.05015, 0, 0]
#                = [ 0.04463, 0.00510, 0.01174]
# ================================================================
body_T_cam1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ 1.0, 0.0, 0.0, 4.4626050628721714e-02,
           0.0, 1.0, 0.0, 5.1000001840293407e-03,
           0.0, 0.0, 1.0, 1.1739999987185001e-02,
           0., 0., 0., 1. ]
```

---

## 参考源码

| 文件 | 关键代码 |
|---|---|
| `vins/src/estimator/parameters.cpp` | `body_T_cam0` / `body_T_cam1` 读取与解析 |
| `vins/src/estimator/feature_manager.cpp` | `tic[0]` / `tic[1]` 在位姿三角化中的使用 |
| `loop_fusion/src/pose_graph.cpp` | 回环检测与位姿图优化 |
