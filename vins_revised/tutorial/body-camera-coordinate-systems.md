# Body 系与 Camera 系的关系：以 Intel RealSense D435i 为例

## 1. 引言

在视觉惯性里程计（VIO）系统中，IMU 和相机是两种最核心的传感器。IMU 测量的是载体坐标系（Body Frame）下的角速度和比力，相机测量的是相机坐标系（Camera Frame）下的图像。为了将这两种观测融合在一起，必须明确知道 **Body 系和 Camera 系之间的几何关系**。

本文以 **Intel RealSense D435i** 为例，结合其官方文档，系统性地解释 Body 系与 Camera 系的物理关系、坐标约定、外参变换以及在 VINS 中的配置方法。

---

## 2. Body 系与 Camera 系的物理关系

### 2.1 什么是 Body 系？

**Body 系（$\mathcal{B}$，又称 IMU 坐标系）** 是固连在 IMU 上的右手直角坐标系：
- **原点**：通常位于 IMU 芯片的中心或传感器的几何中心；
- **坐标轴**：与 IMU 内部加速度计和陀螺仪的敏感轴对齐；
- **测量量**：角速度 $\boldsymbol{\omega}$ 和比力（specific force）$\mathbf{a}$ 都在该坐标系下表示。

### 2.2 什么是 Camera 系？

**Camera 系（$\mathcal{C}$）** 是固连在相机上的右手直角坐标系：
- **原点**：位于相机的光心（pinhole 模型的投影中心）；
- **坐标轴**：通常遵循 OpenCV 约定——$x$ 轴向右，$y$ 轴向下，$z$ 轴沿光轴向前（指向场景）；
- **测量量**：图像平面上的像素坐标 $(u, v)$ 以及通过内参反投影得到的三维射线方向。

### 2.3 两者的关系：刚性固连

在同一个传感器套件中（如 RealSense D435i），IMU 和相机被固定在同一块电路板或同一个机械结构上。只要结构不发生形变，Body 系和 Camera 系之间的相对关系就是**恒定不变**的。这个恒定关系在数学上称为**外参（Extrinsic Parameters）**。

用刚体变换表示：

$$
\mathbf{P}^C = \mathbf{R}_B^C \, \mathbf{P}^B + \mathbf{p}_B^C
$$

其中：
- $\mathbf{R}_B^C \in SO(3)$：将向量从 Body 系旋转到 Camera 系；
- $\mathbf{p}_B^C \in \mathbb{R}^3$：Body 系原点在 Camera 系下的坐标。

在 VINS 的配置文件中，这个外参以 $4 \times 4$ 齐次变换矩阵 `body_T_cam` 的形式给出：

```yaml
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [r11, r12, r13, tx,
          r21, r22, r23, ty,
          r31, r32, r33, tz,
          0,   0,   0,   1]
```

前 $3 \times 3$ 块即为 $\mathbf{R}_B^C$，第 4 列前 3 行即为 $\mathbf{p}_B^C$。

---

## 3. RealSense D435i 的坐标系约定

### 3.1 D435i 的硬件结构

Intel RealSense D435i 在 D435 深度相机的基础上，集成了一个 **Bosch BMI055 6 轴惯性传感器**（3 轴加速度计 + 3 轴陀螺仪）。IMU 芯片位于相机模组内部，与深度传感器刚性固定。

根据官方文档，D435i 的坐标系遵循以下约定：

| 轴 | 方向 | 说明 |
|----|------|------|
| **+x** | 向右 | 相机图像平面的水平向右方向 |
| **+y** | 向下 | 相机图像平面的垂直向下方向 |
| **+z** | 向前 | 沿光轴指向被拍摄场景 |

这个坐标系与 **OpenCV 针孔相机模型** 完全兼容。

### 3.2 关键：D435i 的 Body 系与 Camera 系是同一个吗？

**不是同一个原点，但坐标轴方向经过对齐后是一致的。**

RealSense SDK（`librealsense2`）内部做了以下处理：

> "Each IMU sample is multiplied internally by the extrinsic matrix... The resulting orientation angles and acceleration vectors share the coordinate system with the depth sensor."

也就是说，当你通过 RealSense SDK 读取 IMU 数据时，SDK 已经在内部把原始 IMU 测量值乘以了存储在设备中的外参矩阵，**输出的加速度向量和角速度向量已经和深度传感器（Camera 系）共享同一套坐标系**。

**但这并不意味着 Body 系原点和 Camera 系原点重合。** 它们之间仍然有厘米级的平移偏移。只是在方向（旋转）上，SDK 已经帮你对齐了。

### 3.3 静止时加速度计读数为什么是负的？

官方文档特别指出：

> "The accelerometer is an inertial sensor and it measures inertial force. So when the camera is idle, the accelerometer doesn't measure G-force, but rather the force that resists to G."

物理含义：
- 重力加速度 $\mathbf{g}$ 的方向是**竖直向下**（指向地心）；
- 当相机静止放在桌面上时，桌面给相机的支持力方向是**竖直向上**；
- 加速度计测量的是**比力**（单位质量所受的非引力外力），即支持力；
- D435i 的 $+y$ 轴指向**下方**，所以向上的支持力在 $y$ 轴上的投影为**负值**；
- 因此静止时加速度计的 $y$ 轴读数约为 $\mathbf{a}_y \approx -9.8 \, \text{m/s}^2$。

这正是 VINS 中重力向量 $\mathbf{g}^W = [0, 0, g]^T$（或类似形式）与 IMU 测量互补的体现：IMU 测到的是 $-\mathbf{g}$ 方向（以传感器坐标系表示），而世界系下的运动方程需要把这一项还原回去。

### 3.4 D435i 的外参来源

官方文档明确说明：

> "The depth <-> IMU sensor extrinsic (rigid body transformation) is precalculated based on mechanical drawings and cannot be modified."

这意味着：
1. **旋转外参** $\mathbf{R}_B^C$：由机械图纸预先计算，出厂时已经确定；
2. **平移外参** $\mathbf{p}_B^C$：同样是机械设计的固定值；
3. 两者都存储在设备的固件中，SDK 读取时会自动应用。

对于 VINS 用户来说，如果你使用 RealSense SDK 发布的 `/camera/imu` 话题，那么你拿到的角速度和加速度已经是**在 Camera 坐标系下表示的**。此时在 VINS 配置中，Body 系和 Camera 系的旋转外参 $\mathbf{R}_B^C$ 可以近似为单位矩阵（或一个非常小的旋转），而平移外参 $\mathbf{p}_B^C$ 仍然需要填入 IMU 芯片到相机光心的物理偏移。

然而，在 VINS-Fusion-ROS2 的 RealSense D435i 配置文件中，我们看到 `body_T_cam0` 并不是一个严格的单位矩阵：

```yaml
body_T_cam0:
   data: [ -5.76e-03, -4.05e-03,  9.9998e-01,  2.03e-02,
           -9.9998e-01, -1.02e-03, -5.76e-03,   7.93e-03,
            1.05e-03,   -9.9999e-01, -4.04e-03,  2.86e-03,
            0., 0., 0., 1. ]
```

这说明该配置可能基于未经 SDK 坐标变换的原始 IMU 数据流，或者使用了更精确的外参标定结果（如通过 Kalibr 标定得到）。

---

## 4. 外参在 VINS 中的作用

### 4.1 为什么要用外参？

VINS 的状态估计是在 **Body（IMU）坐标系** 下进行的。所有 IMU 积分、预积分、速度更新都发生在 Body 系下。但视觉观测——特征点在图像上的像素坐标——是在 **Camera 坐标系** 下的。

为了计算视觉重投影误差，必须将 Body 系下的位姿转换到 Camera 系：

$$
\mathbf{P}^C = \mathbf{R}_B^C \left( \mathbf{R}_W^B (\mathbf{P}^W - \mathbf{p}_B^W) \right) + \mathbf{p}_B^C
$$

如果外参错了，即使 Body 系位姿完全正确，投影到图像上的点也会偏移，导致视觉约束给优化器错误的反馈，最终使整个系统漂移甚至发散。

### 4.2 VINS 中如何处理外参？

VINS 提供了三种外参处理策略，通过 `estimate_extrinsic` 参数控制：

| 取值 | 含义 | 适用场景 |
|------|------|---------|
| **0** | 完全信任给定的 `body_T_cam`，固定不优化 | 已用 Kalibr 等工具精确标定 |
| **1** | 以给定值为初始猜测，在线微调 | 标定值有少量误差，允许优化修正 |
| **2** | 没有任何先验，在线标定 | 外参完全未知，VINS 启动时估计旋转外参 |

对于 RealSense D435i，由于 SDK 已经提供了出厂外参，且旋转部分通常非常准确，推荐：
- 如果使用 SDK 发布的已对齐 IMU 数据：`estimate_extrinsic: 0`，`body_T_cam` 设为单位矩阵（仅保留平移偏移）；
- 如果使用原始 IMU 数据或需要更高精度：`estimate_extrinsic: 1`，填入出厂外参作为初始值，让 VINS 在线微调。

### 4.3 D435i 的 IMU 内参校准

官方文档提到：

> "The D435i IMU sensor does not include internal calibration... A complementary calibration tool has been developed and published as part of the SDK."

这说明：
- **外参**（Body <-> Camera 的刚体变换）是出厂预设的，用户无需干预；
- **内参**（加速度计和陀螺仪的零偏、尺度因子、轴间耦合）需要用户自行校准。

RealSense SDK 提供了专门的 IMU 校准工具，校准结果存储在设备的 NVRAM 中。VINS 用户应确保在校准后使用设备，否则 IMU 零偏会比较大，影响初始化成功率。

---

## 5. 标定的层次：内参与外参

在 VIO 系统中，涉及到多种标定参数，容易混淆。以 D435i 为例：

| 标定类型 | 参数内容 | D435i 状态 | VINS 中的对应配置 |
|---------|---------|-----------|-----------------|
| **相机内参** | 焦距 $f_x, f_y$、主点 $c_x, c_y$、畸变系数 | 出厂已标定，存于固件 | `cam0_calib`、`cam1_calib` 引用的 YAML 文件 |
| **相机外参**（双目） | 左相机到右相机的变换 $\mathbf{T}_{C_0}^{C_1}$ | 出厂已标定 | 双目配置中隐式使用 |
| **IMU 内参** | 零偏、噪声密度、随机游走 | **需要用户校准** | `acc_n`、`gyr_n`、`acc_w`、`gyr_w` |
| **IMU-Camera 外参** | Body 系到 Camera 系的刚体变换 $\mathbf{T}_B^C$ | 出厂基于机械图纸预计算 | `body_T_cam0`、`body_T_cam1` |

**关键结论：**
- D435i 的 **IMU-Camera 外参** 通常足够准确，可以直接使用；
- D435i 的 **IMU 内参（零偏）** 必须校准，否则 VINS 初始化很难成功；
- 如果你追求极致精度，可以用 **Kalibr** 对整套传感器（双目 + IMU）做一次联合标定，得到更精确的内外参。

---

## 6. 总结

1. **Body 系（IMU）和 Camera 系是刚性固连的**，它们之间的相对关系就是外参 $(\mathbf{R}_B^C, \mathbf{p}_B^C)$。

2. **RealSense D435i 的坐标约定**：$x$ 向右，$y$ 向下，$z$ 向前。SDK 内部已经将 IMU 数据对齐到 Camera 坐标系，但平移偏移仍然存在。

3. **静止时加速度计读数约为 $-9.8$**（$y$ 轴），这是由坐标系方向（$+y$ 向下）和比力的物理定义（测量支持力而非重力）共同决定的。

4. **D435i 的外参由机械图纸预先计算**，存储在设备固件中，用户一般无需重新标定。但 **IMU 内参（零偏）需要校准**。

5. **在 VINS 中**，外参通过 `body_T_cam` 配置，`estimate_extrinsic` 控制是否在线优化。对于 D435i，推荐先使用出厂外参，若精度不够再启用在线微调。

理解 Body 系和 Camera 系的关系，是正确配置 VINS 并获得良好轨迹估计的前提。
