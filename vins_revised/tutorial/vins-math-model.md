# VINS-Fusion 数学模型详解

## 1. 引言

VINS（Visual-Inertial Navigation System，视觉惯性导航系统）是一种将相机视觉观测与惯性测量单元（IMU）数据进行紧耦合融合的位姿估计方法。VINS-Fusion 在此基础上进一步支持了多传感器融合（如双目相机、GPS 等）以及回路闭合检测。

本文从**程序执行顺序**的角度，系统性地介绍 VINS 的核心算法原理，并在每个章节开头标注对应的**源码文件路径**。

### 1.1 节点与源码对应关系

| 节点 | 可执行文件 | 主源码文件 |
|------|-----------|-----------|
| **vins_node** | `vins/src/rosNodeTest.cpp` | `vins/src/rosNodeTest.cpp` |
| **loop_fusion_node** | `loop_fusion/src/pose_graph_node.cpp` | `loop_fusion/src/pose_graph_node.cpp` |

### 1.2 程序执行顺序总览

```
【vins_node 启动】
  ├─> vins/src/rosNodeTest.cpp:main()
  │    ├─> readParameters()          → 第 3 章：系统启动与参数读取
  │    ├─> estimator.setParameter()  → 第 3 章：系统启动与参数读取
  │    └─> executor.spin()
  │
  ├─> 【视觉前端】vins/src/featureTracker/feature_tracker.cpp:trackImage()
  │    └─> 输出特征点 /feature_tracker/feature    → 第 4 章：视觉前端——特征跟踪
  │
  ├─> 【主线程】vins/src/estimator/estimator.cpp:processMeasurements()
  │    ├─> 【IMU 预积分】IntegrationBase         → 第 5 章：IMU 预积分模型
  │    ├─> 【初始化】initialStructure()           → 第 6 章：视觉-惯性初始化
  │    ├─> 【滑动窗口优化】processImage() → optimization()  → 第 7 章：滑动窗口状态估计与优化
  │    ├─> 【边缘化】slideWindow() → marginalization()     → 第 8 章：边缘化
  │    └─> 【可视化】visualization.cpp               → 第 9 章：可视化输出
  │
  └─> /vins_estimator/odometry, /keyframe_pose, ...

【loop_fusion_node 启动】
  ├─> loop_fusion/src/pose_graph_node.cpp:main()
  │    └─> measurement_process = std::thread(process)
  │
  ├─> 【回环检测】loop_fusion/src/pose_graph.cpp:detectLoop()
  │    ├─> db.query()      → DBoW2 词袋查询
  │    └─> db.add()        → 加入词袋数据库
  │
  ├─> 【几何验证】loop_fusion/src/keyframe.cpp:findConnection()
  │    ├─> searchByBRIEFDes()   → BRIEF 特征匹配
  │    └─> PnPRANSAC()          → PnP 几何验证
  │
  └─> 【位姿图优化】loop_fusion/src/pose_graph.cpp:optimize6DoF()
       └─> Ceres Solver 全局优化  → 第 10 章：回环检测与位姿图优化
```

---

## 2. 坐标系与符号约定

> 📁 **源码位置**：`vins/src/estimator/parameters.h`, `vins/src/estimator/estimator.h`
>
> 所有坐标系定义和符号约定在参数读取和状态估计器初始化时统一使用。

- $\mathcal{W}$：世界坐标系（World Frame）
- $\mathcal{B}$：IMU（载体）坐标系（Body Frame）
- $\mathcal{C}$：相机坐标系（Camera Frame）

**姿态相对约定：** 本文统一使用上下角标表示坐标系间的相对变换。$\mathbf{R}_a^b$ 表示从坐标系 $a$ 到坐标系 $b$ 的旋转矩阵，其作用是将向量从 $a$ 坐标系旋转到 $b$ 坐标系：

$$
\mathbf{v}^b = \mathbf{R}_a^b \, \mathbf{v}^a
$$

$\mathbf{q}_a^b$ 为对应的单位四元数。$(\mathbf{R}_a^b, \mathbf{p}_a^b)$ 共同构成从 $a$ 到 $b$ 的刚体变换，满足：

$$
\mathbf{p}^b = \mathbf{R}_a^b \, \mathbf{p}^a + \mathbf{p}_a^b
$$

其中 $\mathbf{p}_a^b$ 为 $a$ 系原点在 $b$ 系下的坐标。

据此，文中关键符号定义如下：
- $\mathbf{p}_B^W \in \mathbb{R}^3$：IMU（body）原点在世界坐标系下的位置
- $\mathbf{R}_B^W \in SO(3)$：IMU 在世界坐标系下的姿态，即将 body 系向量转到 world 系
- $\mathbf{v}_B^W \in \mathbb{R}^3$：IMU 在世界坐标系下的速度
- $\mathbf{b}_a, \mathbf{b}_g \in \mathbb{R}^3$：加速度计和陀螺仪的零偏

IMU 坐标系到相机坐标系的外参变换记为 $(\mathbf{R}_B^C, \mathbf{p}_B^C)$，即将点从 IMU 坐标系变换到相机坐标系。

---

## 3. 视觉前端——特征跟踪与测量模型

> 📁 **源码位置**：`vins/src/featureTracker/feature_tracker.cpp:trackImage()`
>
> 程序执行顺序：图像 → `img0_callback()` / `img1_callback()` → `feature_callback()` → `trackImage()` → `/feature_tracker/feature`

---

### 3.1 前端特征跟踪流程

特征跟踪是 VINS 的**第一步**。相机图像进入系统后，`FeatureTracker::trackImage()` 执行以下操作：

1. **LK 光流跟踪**：`cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err)`
2. **反向验证**（`FLOW_BACK`）：从当前帧反向跟踪到前一帧，验证一致性
3. **F 矩阵 RANSAC 剔除**：`rejectWithF()` 用基础矩阵剔除误匹配
4. **新特征点检测**：`cv::goodFeaturesToTrack()` 检测新角点
5. **双目匹配**：`cv::calcOpticalFlowPyrLK(cur_img, rightImg, ...)` 在右目中找对应点
6. **输出**：打包为 `featureFrame`，通过 `/feature_tracker/feature` 话题发布

---

> **💡 概念澄清：特征点 vs 路标点**
>
> | | **特征点（Feature Point）** | **路标点 / 地图点（Landmark / Map Point）** |
> |---|---------------------------|------------------------------------------|
> | **维度** | 2D（像素坐标 $u, v$） | 3D（空间坐标 $X, Y, Z$） |
> | **来源** | 图像上检测/跟踪得到（`goodFeaturesToTrack`、`calcOpticalFlowPyrLK`） | 多帧特征点**三角化**（`triangulate()`）恢复 |
> | **数量** | 每帧 100~150 个 | 滑动窗口内通常数百个 |
> | **关系** | 是路标点在**图像上的投影观测** | 是特征点对应的**三维空间位置** |
>
> **一句话：一个路标点可以在多帧中被观测到，每次观测对应图像上的一个特征点。**
>
> ```
> 路标点 P^W (3D) ──► [投影到帧0] ──► 特征点 p_0 (2D)
>     │              [投影到帧1] ──► 特征点 p_1 (2D)
>     │              [投影到帧2] ──► 特征点 p_2 (2D)
> ```

---

### 3.2 针孔相机模型

...

## 4. IMU 预积分模型

> 📁 **源码位置**：`vins/src/factor/integration_base.h`
>
> 程序执行顺序：`processMeasurements()` → `processIMU()` → `push_back()` → 中值积分更新 `alpha, beta, gamma`

### 4.1 IMU 测量模型

IMU 的原始测量包括加速度计读数 $\hat{\mathbf{a}}_t$ 和陀螺仪读数 $\hat{\boldsymbol{\omega}}_t$。在忽略地球自转（低成本 MEMS IMU）的前提下，测量值与真实值之间的关系为：

$$
\hat{\mathbf{a}}_t = \mathbf{a}_t^B + \mathbf{b}_{a_t} + \mathbf{n}_a
$$

$$
\hat{\boldsymbol{\omega}}_t = \boldsymbol{\omega}_t^B + \mathbf{b}_{g_t} + \mathbf{n}_g
$$

其中：
- $\mathbf{a}_t^B$ 为 IMU 坐标系下的真实加速度（包含重力分量）；
- $\boldsymbol{\omega}_t^B$ 为 IMU 坐标系下的真实角速度；
- $\mathbf{n}_a \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\sigma}_a^2)$，$\mathbf{n}_g \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\sigma}_g^2)$ 为白噪声；
- $\mathbf{b}_{a_t}$ 和 $\mathbf{b}_{g_t}$ 为随机游走零偏，其导数服从高斯分布：

$$
\dot{\mathbf{b}}_a = \mathbf{n}_{b_a}, \quad \dot{\mathbf{b}}_g = \mathbf{n}_{b_g}
$$

其中 $\mathbf{n}_{b_a} \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\sigma}_{b_a}^2)$，$\mathbf{n}_{b_g} \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\sigma}_{b_g}^2)$。

### 4.2 运动学方程

设世界坐标系 $\mathcal{W}$ 为惯性参考系，IMU 在世界系下的状态由位置 $\mathbf{p}_B^W$、速度 $\mathbf{v}_B^W$ 和姿态 $\mathbf{q}_B^W$（或 $\mathbf{R}_B^W$）描述。根据牛顿力学，运动学方程为：

$$
\dot{\mathbf{p}}_B^W = \mathbf{v}_B^W
$$

$$
\dot{\mathbf{v}}_B^W = \mathbf{R}_B^W \, \mathbf{a}_t^B = \mathbf{R}_B^W \left( \hat{\mathbf{a}}_t - \mathbf{b}_{a_t} - \mathbf{n}_a \right) + \mathbf{g}^W
$$

$$
\dot{\mathbf{R}}_B^W = \mathbf{R}_B^W \, \left[ \boldsymbol{\omega}_t^B \right]_\times = \mathbf{R}_B^W \left[ \hat{\boldsymbol{\omega}}_t - \mathbf{b}_{g_t} - \mathbf{n}_g \right]_\times
$$

其中：
- $\mathbf{g}^W = [0, 0, g]^T$ 为重力加速度在世界系下的向量（通常取 $g \approx 9.81 \, \text{m/s}^2$）；
- $[\cdot]_\times$ 表示将三维向量映射为反对称矩阵（叉乘矩阵）；
- 姿态也可以用四元数表示：

$$
\dot{\mathbf{q}}_B^W = \frac{1}{2} \mathbf{q}_B^W \otimes \begin{bmatrix} 0 \\ \hat{\boldsymbol{\omega}}_t - \mathbf{b}_{g_t} - \mathbf{n}_g \end{bmatrix}
$$

其中 $\otimes$ 表示四元数乘法。

### 4.3 预积分的动机与定义

在基于优化的 VIO 中，IMU 测量需要在相邻图像帧之间进行积分，以提供帧间运动约束。然而，如果直接在世界系下积分：

$$
\mathbf{v}_j^W = \mathbf{v}_i^W + \int_{t_i}^{t_j} \left( \mathbf{R}_B^W(t) \hat{\mathbf{a}}_t + \mathbf{g}^W \right) dt
$$

$$
\mathbf{p}_j^W = \mathbf{p}_i^W + \int_{t_i}^{t_j} \mathbf{v}_B^W(t) \, dt
$$

每当优化更新 $\mathbf{R}_B^W$、$\mathbf{v}_B^W$ 或 $\mathbf{p}_B^W$ 时，都需要重新积分，计算代价高昂。

**预积分（Pre-integration）** 的核心思想是：将积分量与绝对位姿分离，使得积分结果仅依赖于两帧之间的 IMU 测量，而与世界系下的绝对位姿无关。

**详细推导：**

从速度的运动方程出发，将 $\mathbf{v}_j^W - \mathbf{v}_i^W$ 写为：

$$
\mathbf{v}_j^W - \mathbf{v}_i^W = \int_{t_i}^{t_j} \left( \mathbf{R}_B^W(\tau) (\hat{\mathbf{a}}_\tau - \mathbf{b}_{a_\tau} - \mathbf{n}_a) + \mathbf{g}^W \right) d\tau
$$

两边同时左乘 $\mathbf{R}_W^B(t_i) = (\mathbf{R}_B^W(t_i))^{-1}$：

$$
\mathbf{R}_W^B(t_i) (\mathbf{v}_j^W - \mathbf{v}_i^W) = \int_{t_i}^{t_j} \left( \mathbf{R}_W^B(t_i) \mathbf{R}_B^W(\tau) (\hat{\mathbf{a}}_\tau - \mathbf{b}_{a_\tau} - \mathbf{n}_a) + \mathbf{R}_W^B(t_i) \mathbf{g}^W \right) d\tau
$$

定义相对旋转 $\mathbf{R}_i^\tau \triangleq \mathbf{R}_W^B(t_i) \mathbf{R}_B^W(\tau)$，它将向量从 $t_\tau$ 时刻的 body 系旋转到 $t_i$ 时刻的 body 系。上式右端第一项的被积函数中的 $\mathbf{R}_i^\tau (\hat{\mathbf{a}}_\tau - \cdots)$ 完全是在 $t_i$ 时刻的 body 坐标系下表示的，与绝对姿态 $\mathbf{R}_B^W(t_i)$ 无关。

然而，上式左端仍然含有 $\mathbf{v}_j^W$ 和 $\mathbf{v}_i^W$，右端含有 $\mathbf{g}^W$ 的积分项。为了彻底分离，我们直接**定义**以下三个仅依赖于区间 $[t_i, t_j]$ 内 IMU 测量的量：

**位置预积分：**

$$
\boldsymbol{\alpha}_i^j \triangleq \iint_{t_i}^{t_j} \mathbf{R}_i^\tau \left( \hat{\mathbf{a}}_\tau - \mathbf{b}_{a_i} \right) d\tau^2
$$

**速度预积分：**

$$
\boldsymbol{\beta}_i^j \triangleq \int_{t_i}^{t_j} \mathbf{R}_i^\tau \left( \hat{\mathbf{a}}_\tau - \mathbf{b}_{a_i} \right) d\tau
$$

**姿态预积分（旋转增量）：**

$$
\boldsymbol{\gamma}_i^j \triangleq \int_{t_i}^{t_j} \frac{1}{2} \boldsymbol{\gamma}_i^\tau \otimes \begin{bmatrix} 0 \\ \hat{\boldsymbol{\omega}}_\tau - \mathbf{b}_{g_i} \end{bmatrix} d\tau
$$

其中：
- $\mathbf{R}_i^\tau \triangleq \mathbf{R}_W^B(t_i) \mathbf{R}_B^W(\tau)$，表示从时刻 $t_\tau$ 的 body 系到 $t_i$ 时刻 body 系的相对旋转；
- $\boldsymbol{\gamma}_i^\tau$ 为对应的相对旋转四元数；
- 注意预积分量是在**第 $i$ 帧的 IMU 坐标系**下表示的；
- 零偏取了区间起始时刻的值 $\mathbf{b}_{a_i}$、$\mathbf{b}_{g_i}$，因为在短时间内零偏变化很小。

### 4.4 预积分更新公式

利用预积分量，可以从状态 $i$ 递推得到状态 $j$。我们从运动方程直接积分推导：

**速度推导：**

$$
\mathbf{v}_j^W = \mathbf{v}_i^W + \int_{t_i}^{t_j} \left( \mathbf{R}_B^W(\tau) (\hat{\mathbf{a}}_\tau - \mathbf{b}_{a_i}) + \mathbf{g}^W \right) d\tau
$$

将 $\mathbf{R}_B^W(\tau) = \mathbf{R}_B^W(t_i) \mathbf{R}_i^\tau$ 代入：

$$
\mathbf{v}_j^W = \mathbf{v}_i^W + \mathbf{R}_i^W \int_{t_i}^{t_j} \mathbf{R}_i^\tau (\hat{\mathbf{a}}_\tau - \mathbf{b}_{a_i}) d\tau + \mathbf{g}^W \Delta t_{ij}
$$

$$= \mathbf{v}_i^W + \mathbf{R}_i^W \boldsymbol{\beta}_i^j + \mathbf{g}^W \Delta t_{ij}
$$

注意 VINS 代码中取 $\mathbf{g}^W = [0, 0, g]^T$，因此上式写为：

$$
\mathbf{v}_j^W = \mathbf{v}_i^W - \mathbf{g}^W \Delta t_{ij} + \mathbf{R}_i^W \boldsymbol{\beta}_i^j
$$

**位置推导：**

$$
\mathbf{p}_j^W = \mathbf{p}_i^W + \int_{t_i}^{t_j} \mathbf{v}_B^W(s) \, ds
$$

将速度公式代入并积分：

$$
\mathbf{p}_j^W = \mathbf{p}_i^W + \mathbf{v}_i^W \Delta t_{ij} - \frac{1}{2} \mathbf{g}^W \Delta t_{ij}^2 + \mathbf{R}_i^W \boldsymbol{\alpha}_i^j
$$

**姿态推导：**

四元数直接相乘：

$$
\mathbf{q}_j^W = \mathbf{q}_i^W \otimes \boldsymbol{\gamma}_i^j
$$

或等价地用旋转矩阵表示为 $\mathbf{R}_j^W = \mathbf{R}_i^W \mathbf{R}_i^j$。

**关键观察：** 预积分量 $\boldsymbol{\alpha}_i^j$、$\boldsymbol{\beta}_i^j$、$\boldsymbol{\gamma}_i^j$ 仅依赖于区间 $[t_i, t_j]$ 内的 IMU 测量和零偏 $\mathbf{b}_{a_i}$、$\mathbf{b}_{g_i}$，与绝对位姿 $\mathbf{p}_i^W$、$\mathbf{v}_i^W$、$\mathbf{R}_i^W$ 无关。因此，当优化更新位姿时，预积分量无需重新计算。

### 4.5 预积分的离散化与中值积分

实际系统中 IMU 以离散频率采样（如 200 Hz 或 400 Hz），需要对上述连续积分进行离散化。设时间区间 $[t_i, t_j]$ 内包含 $N$ 个 IMU 测量区间，每个区间长度为 $\Delta t$。

VINS 采用**中值积分（Mid-point Integration）** 进行离散化，即在一个 IMU 采样间隔内假设加速度计和陀螺仪的测量值为该区间两端读数的平均值。

**详细推导：**

考虑单个采样间隔 $[t_k, t_{k+1}]$，设该间隔起始时的预积分量为 $\boldsymbol{\alpha}_k$、$\boldsymbol{\beta}_k$、$\boldsymbol{\gamma}_k$。

**姿态更新推导：**

陀螺仪中值角速度：

$$
\bar{\boldsymbol{\omega}} = \frac{1}{2} \left[ (\hat{\boldsymbol{\omega}}_k - \mathbf{b}_{g_k}) + (\hat{\boldsymbol{\omega}}_{k+1} - \mathbf{b}_{g_k}) \right]
$$

在时间间隔 $\Delta t$ 内，假设角速度恒定为 $\bar{\boldsymbol{\omega}}$，则旋转增量为 $\bar{\boldsymbol{\omega}} \Delta t$。对应的四元数更新（小角度近似）：

$$
\delta\mathbf{q} = \begin{bmatrix} 1 \\ \frac{1}{2} \bar{\boldsymbol{\omega}} \Delta t \end{bmatrix}
$$

更精确的归一化形式：

$$
\delta\mathbf{q} = \begin{bmatrix} \cos\left(\frac{\|\bar{\boldsymbol{\omega}}\| \Delta t}{2}\right) \\ \frac{\bar{\boldsymbol{\omega}}}{\|\bar{\boldsymbol{\omega}}\|} \sin\left(\frac{\|\bar{\boldsymbol{\omega}}\| \Delta t}{2}\right) \end{bmatrix}
$$

姿态预积分更新：

$$
\boldsymbol{\gamma}_{k+1} = \boldsymbol{\gamma}_k \otimes \delta\mathbf{q}
$$

**速度更新推导：**

加速度在 $t_k$ 时刻的 body 系下为 $\mathbf{a}_k = \hat{\mathbf{a}}_k - \mathbf{b}_{a_k}$，在 $t_{k+1}$ 时刻为 $\mathbf{a}_{k+1} = \hat{\mathbf{a}}_{k+1} - \mathbf{b}_{a_k}$。它们在世界系（或更准确地说是 $t_i$ 时刻 body 系）下的方向由对应的旋转决定。

利用中值定理，速度增量为平均加速度乘以 $\Delta t$：

$$
\boldsymbol{\beta}_{k+1} = \boldsymbol{\beta}_k + \frac{1}{2} \left( \mathbf{R}_k \mathbf{a}_k + \mathbf{R}_{k+1} \mathbf{a}_{k+1} \right) \Delta t
$$

其中 $\mathbf{R}_k$ 为 $\boldsymbol{\gamma}_k$ 对应的旋转矩阵。

**位置更新推导：**

位置增量由平均速度乘以 $\Delta t$ 得到。在一个间隔内，速度从 $\boldsymbol{\beta}_k$ 线性增加到 $\boldsymbol{\beta}_{k+1}$，因此平均速度为 $\frac{1}{2}(\boldsymbol{\beta}_k + \boldsymbol{\beta}_{k+1})$。但更直接的推导是利用积分：

$$
\boldsymbol{\alpha}_{k+1} = \boldsymbol{\alpha}_k + \boldsymbol{\beta}_k \Delta t + \int_0^{\Delta t} \int_0^s \mathbf{a}(\tau) \, d\tau \, ds
$$

对于恒定加速度（中值），双重积分产生 $\frac{1}{2} \bar{\mathbf{a}} \Delta t^2$。结合两端加速度的中值近似，得到：

$$
\boldsymbol{\alpha}_{k+1} = \boldsymbol{\alpha}_k + \boldsymbol{\beta}_k \Delta t + \frac{1}{4} \left( \mathbf{R}_k \mathbf{a}_k + \mathbf{R}_{k+1} \mathbf{a}_{k+1} \right) \Delta t^2
$$

### 4.6 预积分的误差状态方程

预积分量受 IMU 测量噪声和零偏随机游走的影响。为了量化不确定性并用于后续优化的信息矩阵，需要推导预积分误差的状态传播方程。

**详细推导：**

定义预积分误差状态：

$$
\delta \boldsymbol{\alpha}_k, \quad \delta \boldsymbol{\beta}_k, \quad \delta \boldsymbol{\theta}_k, \quad \delta \mathbf{b}_{a_k}, \quad \delta \mathbf{b}_{g_k}
$$

其中 $\delta \boldsymbol{\theta}_k$ 为姿态误差对应的李代数（旋转向量），满足 $\mathbf{R}_k \approx \mathbf{R}_k^{true} \exp\left([\delta\boldsymbol{\theta}_k]_\times\right)$ 或等价地 $\delta\mathbf{R}_k \approx \mathbf{I} + [\delta\boldsymbol{\theta}_k]_\times$。

考虑一个 IMU 采样间隔 $[t_k, t_{k+1}]$，设真实状态为 $\mathbf{x}^{true}$，预积分计算用的名义状态为 $\mathbf{x}$，误差为 $\delta\mathbf{x} = \mathbf{x}^{true} - \mathbf{x}$（对于旋转用乘法误差）。

**位置误差的推导：**

真实位置预积分满足：

$$
\boldsymbol{\alpha}_{k+1}^{true} = \boldsymbol{\alpha}_k^{true} + \boldsymbol{\beta}_k^{true} \Delta t + \frac{1}{2} \mathbf{R}_k^{true} (\hat{\mathbf{a}}_k - \mathbf{b}_{a_k}^{true}) \Delta t^2
$$

（这里用欧拉积分简化推导，结果与中值积分的一阶近似一致）

名义位置预积分：

$$
\boldsymbol{\alpha}_{k+1} = \boldsymbol{\alpha}_k + \boldsymbol{\beta}_k \Delta t + \frac{1}{2} \mathbf{R}_k (\hat{\mathbf{a}}_k - \mathbf{b}_{a_k}) \Delta t^2
$$

相减并保留一阶项：

$$\delta\boldsymbol{\alpha}_{k+1} = \delta\boldsymbol{\alpha}_k + \delta\boldsymbol{\beta}_k \Delta t + \frac{1}{2} \delta\mathbf{R}_k (\hat{\mathbf{a}}_k - \mathbf{b}_{a_k}) \Delta t^2 - \frac{1}{2} \mathbf{R}_k \delta\mathbf{b}_{a_k} \Delta t^2$$

利用 $\delta\mathbf{R}_k \approx [\delta\boldsymbol{\theta}_k]_\times \mathbf{R}_k$，有 $\delta\mathbf{R}_k \mathbf{a} \approx [\delta\boldsymbol{\theta}_k]_\times \mathbf{R}_k \mathbf{a} = -(\mathbf{R}_k \mathbf{a}) \times \delta\boldsymbol{\theta}_k$（利用叉乘矩阵性质 $[\mathbf{u}]_\times \mathbf{v} = -\mathbf{v} \times \mathbf{u} = -[\mathbf{v}]_\times \mathbf{u}$）。

因此：

$$\delta\boldsymbol{\alpha}_{k+1} \approx \delta\boldsymbol{\alpha}_k + \delta\boldsymbol{\beta}_k \Delta t - \frac{1}{2} \mathbf{R}_k [\hat{\mathbf{a}}_k - \mathbf{b}_{a_k}]_\times \delta\boldsymbol{\theta}_k \Delta t^2 - \frac{1}{2} \mathbf{R}_k \delta\mathbf{b}_{a_k} \Delta t^2$$

**速度误差的推导：**

类似地：

$$\delta\boldsymbol{\beta}_{k+1} \approx \delta\boldsymbol{\beta}_k - \mathbf{R}_k [\hat{\mathbf{a}}_k - \mathbf{b}_{a_k}]_\times \delta\boldsymbol{\theta}_k \Delta t - \mathbf{R}_k \delta\mathbf{b}_{a_k} \Delta t + \mathbf{R}_k \mathbf{n}_{a_k} \Delta t$$

**姿态误差的推导：**

真实旋转：$\mathbf{R}_{k+1}^{true} = \mathbf{R}_k^{true} \exp\left([(\hat{\boldsymbol{\omega}}_k - \mathbf{b}_{g_k}^{true}) \Delta t + \mathbf{n}_{g_k} \Delta t]_\times\right)$

名义旋转：$\mathbf{R}_{k+1} = \mathbf{R}_k \exp\left([(\hat{\boldsymbol{\omega}}_k - \mathbf{b}_{g_k}) \Delta t]_\times\right)$

定义乘法误差：$\mathbf{R}_k^{true} = \mathbf{R}_k \exp\left([\delta\boldsymbol{\theta}_k]_\times\right)$

经过一阶近似推导（利用 BCH 公式的线性近似），得到：

$$\delta\boldsymbol{\theta}_{k+1} \approx \delta\boldsymbol{\theta}_k - [\hat{\boldsymbol{\omega}}_k - \mathbf{b}_{g_k}]_\times \delta\boldsymbol{\theta}_k \Delta t - \delta\mathbf{b}_{g_k} \Delta t + \mathbf{n}_{g_k} \Delta t$$

$$= \left( \mathbf{I} - [\hat{\boldsymbol{\omega}}_k - \mathbf{b}_{g_k}]_\times \Delta t \right) \delta\boldsymbol{\theta}_k - \delta\mathbf{b}_{g_k} \Delta t + \mathbf{n}_{g_k} \Delta t$$

**零偏误差的推导：**

零偏是随机游走：

$$\delta\mathbf{b}_{a_{k+1}} = \delta\mathbf{b}_{a_k} + \mathbf{n}_{b_{a_k}} \Delta t$$

$$\delta\mathbf{b}_{g_{k+1}} = \delta\mathbf{b}_{g_k} + \mathbf{n}_{b_{g_k}} \Delta t$$

**组合成矩阵形式：**

$$
\begin{bmatrix}
\delta \boldsymbol{\alpha}_{k+1} \\
\delta \boldsymbol{\beta}_{k+1} \\
\delta \boldsymbol{\theta}_{k+1} \\
\delta \mathbf{b}_{a_{k+1}} \\
\delta \mathbf{b}_{g_{k+1}}
\end{bmatrix}
=
\mathbf{F}_k
\begin{bmatrix}
\delta \boldsymbol{\alpha}_k \\
\delta \boldsymbol{\beta}_k \\
\delta \boldsymbol{\theta}_k \\
\delta \mathbf{b}_{a_k} \\
\delta \mathbf{b}_{g_k}
\end{bmatrix}
+
\mathbf{G}_k
\begin{bmatrix}
\mathbf{n}_{a_k} \\
\mathbf{n}_{g_k} \\
\mathbf{n}_{b_{a_k}} \\
\mathbf{n}_{b_{g_k}}
\end{bmatrix}
$$

其中 $\mathbf{F}_k$ 和 $\mathbf{G}_k$ 分别为 $15 \times 15$ 和 $15 \times 12$ 的矩阵。

**欧拉积分下的状态转移矩阵 $\mathbf{F}_k$：**

$$
\mathbf{F}_k = \begin{bmatrix}
\mathbf{I} & \mathbf{I} \Delta t & -\frac{1}{2} \mathbf{R}_k [\hat{\mathbf{a}}_k - \mathbf{b}_{a_k}]_\times \Delta t^2 & -\frac{1}{2} \mathbf{R}_k \Delta t^2 & \mathbf{0} \\
\mathbf{0} & \mathbf{I} & -\mathbf{R}_k [\hat{\mathbf{a}}_k - \mathbf{b}_{a_k}]_\times \Delta t & -\mathbf{R}_k \Delta t & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{I} - [\hat{\boldsymbol{\omega}}_k - \mathbf{b}_{g_k}]_\times \Delta t & \mathbf{0} & -\mathbf{I} \Delta t \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}
\end{bmatrix}
$$

**噪声输入矩阵 $\mathbf{G}_k$：**

$$
\mathbf{G}_k = \begin{bmatrix}
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{R}_k \Delta t & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{I} \Delta t & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{I} \Delta t & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} \Delta t
\end{bmatrix}
$$

### 4.7 预积分的协方差传播

设第 $k$ 步的预积分协方差矩阵为 $\boldsymbol{\Sigma}_k \in \mathbb{R}^{15 \times 15}$，则通过线性误差传播公式：

$$
\boldsymbol{\Sigma}_{k+1} = \mathbf{F}_k \, \boldsymbol{\Sigma}_k \, \mathbf{F}_k^T + \mathbf{G}_k \, \mathbf{Q} \, \mathbf{G}_k^T
$$

其中 $\mathbf{Q} = \text{diag}(\boldsymbol{\sigma}_a^2, \boldsymbol{\sigma}_g^2, \boldsymbol{\sigma}_{b_a}^2, \boldsymbol{\sigma}_{b_g}^2)$ 为噪声协方差矩阵。初始条件为 $\boldsymbol{\Sigma}_0 = \mathbf{0}$。通过迭代上式，可以得到区间 $[t_i, t_j]$ 内预积分量的最终协方差矩阵 $\boldsymbol{\Sigma}_{ij}$。

**数值验证：**

假设一个采样间隔 $\Delta t = 0.1$ s，加速度计噪声 $\sigma_a = 0.1 \, \text{m/s}^2$，初始 $\boldsymbol{\Sigma}_0 = \mathbf{0}$。则一步传播后速度预积分的方差：

$$\text{Var}(\beta_x) = (\Delta t)^2 \sigma_a^2 = 0.01 \times 0.01 = 0.0001 \, (\text{m/s})^2$$

即标准差约 $0.01$ m/s。若两帧间隔 1 s（10 个采样），速度预积分的标准差累积到约 $0.032$ m/s。

### 4.8 预积分对零偏的一阶修正

在优化过程中，零偏 $\mathbf{b}_a$ 和 $\mathbf{b}_g$ 会不断更新。如果每次零偏变化都重新进行预积分，计算开销较大。VINS 采用**一阶泰勒展开**对预积分量进行近似修正。

**详细推导：**

设预积分时的零偏为 $\bar{\mathbf{b}}_a$ 和 $\bar{\mathbf{b}}_g$，优化更新后的零偏为 $\mathbf{b}_a = \bar{\mathbf{b}}_a + \delta\mathbf{b}_a$，$\mathbf{b}_g = \bar{\mathbf{b}}_g + \delta\mathbf{b}_g$。

对于位置和速度预积分，它们关于零偏的依赖关系近似为线性（因为积分内部是 $-(\hat{\mathbf{a}} - \mathbf{b}_a)$ 形式）。因此：

$$
\boldsymbol{\alpha}_i^j(\mathbf{b}_a) \approx \hat{\boldsymbol{\alpha}}_i^j + \mathbf{J}_{b_a}^{\alpha} \, \delta\mathbf{b}_a
$$

$$
\boldsymbol{\beta}_i^j(\mathbf{b}_a) \approx \hat{\boldsymbol{\beta}}_i^j + \mathbf{J}_{b_a}^{\beta} \, \delta\mathbf{b}_a
$$

其中雅可比矩阵 $\mathbf{J}_{b_a}^{\alpha} = \frac{\partial \boldsymbol{\alpha}}{\partial \mathbf{b}_a}$，$\mathbf{J}_{b_a}^{\beta} = \frac{\partial \boldsymbol{\beta}}{\partial \mathbf{b}_a}$。从 $\mathbf{F}$ 矩阵的结构可知：

$$\frac{\partial \delta\boldsymbol{\alpha}}{\partial \delta\mathbf{b}_a} \approx -\frac{1}{2} \mathbf{R}_k \Delta t^2, \quad \frac{\partial \delta\boldsymbol{\beta}}{\partial \delta\mathbf{b}_a} \approx -\mathbf{R}_k \Delta t$$

对于姿态预积分，旋转关于陀螺仪零偏的依赖通过指数映射描述。利用李群上的微分：

$$\frac{\partial}{\partial \mathbf{b}_g} \exp\left([(\hat{\boldsymbol{\omega}} - \mathbf{b}_g) \Delta t]_\times\right) \approx -\mathbf{A}(\boldsymbol{\phi}) \Delta t$$

其中 $\mathbf{A}(\boldsymbol{\phi})$ 为 SO(3) 的伴随矩阵（或右雅可比矩阵），$\boldsymbol{\phi} = (\hat{\boldsymbol{\omega}} - \mathbf{b}_g) \Delta t$。对于小角度，$\mathbf{A}(\boldsymbol{\phi}) \approx \mathbf{I}$，因此：

$$
\boldsymbol{\gamma}_i^j(\mathbf{b}_g) \approx \hat{\boldsymbol{\gamma}}_i^j \otimes \begin{bmatrix} 1 \\ -\frac{1}{2} \mathbf{J}_{b_g}^{\gamma} \, \delta\mathbf{b}_g \end{bmatrix}
$$

其中 $\mathbf{J}_{b_g}^{\gamma}$ 可通过误差状态方程中的对应块提取。

**具体地，雅可比矩阵的更新公式为：**

$$
\mathbf{J}_{k+1} = \mathbf{F}_k \, \mathbf{J}_k
$$

初始条件为 $\mathbf{J}_0 = \mathbf{I}_{15}$。提取其中的对应块即可得到各预积分量关于零偏的雅可比。

**数值验证：**

假设两帧间隔 $\Delta t = 0.1$ s，预积分时猜测 $b_a = 0.1$ m/s²，实际应为 $b_a = 0.2$ m/s²，IMU 测量 $\hat{a} = 1.0$ m/s²。

预积分用的加速度：$a_{\text{used}} = 1.0 - 0.1 = 0.9$ m/s²，得到 $\hat{\alpha} = 0.9 \times 0.005 = 0.0045$ m。

零偏偏差：$\delta b_a = 0.1$ m/s²。

雅可比：$J_{b_a}^{\alpha} \approx -\frac{1}{2} \Delta t^2 = -0.005$。

修正量：$\Delta \alpha = -0.005 \times 0.1 = -0.0005$ m。

修正后：$\alpha = 0.0045 - 0.0005 = 0.0040$ m。

直接用正确零偏重算：$a_{\text{true}} = 1.0 - 0.2 = 0.8$ m/s²，$\alpha = 0.8 \times 0.005 = 0.0040$ m。

**完全一致！** 验证了零偏修正的有效性。

### 4.9 预积分残差及其雅可比

在优化中，IMU 残差衡量预积分预测与状态估计之间的一致性。将 3.4 节的递推公式改写为残差形式：

$$
\mathbf{r}_{\mathcal{B}}(\hat{\mathbf{z}}_{i}^{j}, \mathcal{X}) =
\begin{bmatrix}
\mathbf{R}_W^i \left( \mathbf{p}_j^W - \mathbf{p}_i^W - \mathbf{v}_i^W \Delta t + \frac{1}{2} \mathbf{g}^W \Delta t^2 \right) - \boldsymbol{\alpha}_i^j \\
\mathbf{R}_W^i \left( \mathbf{v}_j^W - \mathbf{v}_i^W + \mathbf{g}^W \Delta t \right) - \boldsymbol{\beta}_i^j \\
2 \left[ (\mathbf{q}_i^W)^{-1} \otimes \mathbf{q}_j^W \otimes (\boldsymbol{\gamma}_i^j)^{-1} \right]_{xyz} \\
\mathbf{b}_{a_j} - \mathbf{b}_{a_i} \\
\mathbf{b}_{g_j} - \mathbf{b}_{g_i}
\end{bmatrix}
$$

其中：
- 第一行为位置残差，在 $i$ 时刻的 body 坐标系下度量；
- 第二行为速度残差，同样在 $i$ 时刻 body 坐标系下度量；
- 第三行为姿态残差，利用四元数虚部表示旋转误差；
- 最后两行为零偏随机游走约束。

**状态估计值 $\mathcal{X}$ 的来源：**

残差公式中的 $\mathbf{p}_i^W, \mathbf{v}_i^W, \mathbf{q}_i^W$ 和 $\mathbf{p}_j^W, \mathbf{v}_j^W, \mathbf{q}_j^W$ 并不是从某个独立传感器直接读到的，而是**滑动窗口中当前待优化的状态变量**。在优化开始前，它们有一个**初始猜测值**，来源包括：

1. **上一帧优化结果的传播**：前一帧优化结束后得到最优状态，利用 IMU 运动方程（3.2 节）向前积分递推到当前帧，作为初始值；
2. **视觉初始化**：系统启动阶段，通过纯视觉 SFM 得到初始位姿，再与 IMU 预积分对齐得到速度和零偏（见第 7 节）；
3. **先验信息**：边缘化（第 6 节）留下的历史约束为状态提供了先验分布，也参与构成初始估计。

**优化的本质**就是不断调整这些状态变量，使得 IMU 残差（要求相邻帧状态差与预积分匹配）和视觉残差（要求重投影与观测匹配）同时最小。因此，状态估计值在优化前后是变化的：优化前是"猜测"，优化后是"当前最优估计"。

**残差对状态变量的雅可比推导：**

对于位置残差 $\mathbf{r}_p = \mathbf{R}_W^i (\mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i \Delta t + \frac{1}{2} \mathbf{g} \Delta t^2) - \boldsymbol{\alpha}$：

- 对 $\mathbf{p}_i$ 的雅可比：$\frac{\partial \mathbf{r}_p}{\partial \mathbf{p}_i} = -\mathbf{R}_W^i$
- 对 $\mathbf{p}_j$ 的雅可比：$\frac{\partial \mathbf{r}_p}{\partial \mathbf{p}_j} = \mathbf{R}_W^i$
- 对 $\mathbf{v}_i$ 的雅可比：$\frac{\partial \mathbf{r}_p}{\partial \mathbf{v}_i} = -\mathbf{R}_W^i \Delta t$
- 对 $\mathbf{R}_i$ 的雅可比：利用旋转矩阵的链式法则，涉及 $[\mathbf{R}_W^i (\cdots)]_\times$

对于姿态残差 $\mathbf{r}_q = 2 [(\mathbf{q}_i)^{-1} \otimes \mathbf{q}_j \otimes \gamma^{-1}]_{xyz}$：

- 对 $\mathbf{q}_i$ 的雅可比涉及左乘四元数逆的雅可比矩阵
- 对 $\mathbf{q}_j$ 的雅可比涉及右乘四元数逆的雅可比矩阵

Ceres Solver 也支持自动求导，但在 VINS 的代码实现中，IMU 残差的雅可比是手动推导并解析计算的，以提高效率。

---



### 3.2 针孔相机模型

VINS 支持多种相机模型，最常用的是针孔相机模型。对于三维空间点 $\mathbf{P}^C = [X, Y, Z]^T$ 在相机坐标系下的坐标，其投影到归一化平面为：

$$
\mathbf{p}_n = \begin{bmatrix} x \\ y \end{bmatrix} = \begin{bmatrix} X/Z \\ Y/Z \end{bmatrix}
$$

经过畸变校正和像素缩放后，像素坐标为：

$$
\mathbf{p}_{pix} = \begin{bmatrix} f_x x + c_x \\ f_y y + c_y \end{bmatrix} = \mathbf{K} \, \mathbf{p}_n
$$

其中 $\mathbf{K} = \text{diag}(f_x, f_y)$ 为内参矩阵的焦距部分，$f_x, f_y$ 为焦距，$c_x, c_y$ 为主点坐标。

### 3.3 重投影误差

对于滑动窗口中一个被多帧观测到的路标点 $\mathbf{P}^W$，其在第 $k$ 帧图像中的重投影误差定义为：

$$
\mathbf{r}_{C_{kj}} = \mathbf{p}_{kj}^{obs} - \pi_c \left( \mathbf{P}^C \right)
$$

其中三维点在第 $k$ 帧相机坐标系下的坐标通过以下链式变换得到：

首先，从 world 系变换到第 $k$ 帧的 body 系：

$$
\mathbf{P}^{B_k} = \mathbf{R}_W^k \, \mathbf{P}^W + \mathbf{p}_W^k = \mathbf{R}_W^k \left( \mathbf{P}^W - \mathbf{p}_k^W \right)
$$

这里利用了 $\mathbf{p}_W^k = -\mathbf{R}_W^k \mathbf{p}_k^W$（world 原点在 body 系下的坐标）。

然后，从 body 系变换到相机系：

$$
\mathbf{P}^C = \mathbf{R}_B^C \, \mathbf{P}^{B_k} + \mathbf{p}_B^C = \mathbf{R}_B^C \, \mathbf{R}_W^k \left( \mathbf{P}^W - \mathbf{p}_k^W \right) + \mathbf{p}_B^C
$$

其中：
- $\mathbf{p}_{kj}^{obs}$ 为第 $k$ 帧中第 $j$ 个特征点的观测像素坐标；
- $\pi_c(\cdot)$ 为相机投影函数（包含去畸变和内参变换）；
- $(\mathbf{R}_B^C, \mathbf{p}_B^C)$ 为 IMU 到相机的外参（将点从 body 系转到 camera 系）。

### 3.4 逆深度参数化

为了减少参数数量并提高数值稳定性，VINS 在特征管理器中采用 **逆深度（Inverse Depth）** 参数化。将特征点在第 $i$ 次观测帧中的深度 $d$ 表示为 $\lambda = 1/d$，则三维点坐标为：

$$
\mathbf{P}^W = \mathbf{R}_i^W \left( \frac{1}{\lambda} \mathbf{m}_i \right) + \mathbf{p}_i^W
$$

其中 $\mathbf{m}_i$ 为由像素坐标反投影得到的单位方向向量（在 $i$ 时刻 body 坐标系下）：

$$
\mathbf{m}_i = \frac{1}{\sqrt{x^2 + y^2 + 1}} \begin{bmatrix} x \\ y \\ 1 \end{bmatrix}
$$

这里 $(x, y)$ 为归一化平面坐标。这种参数化在特征点位于无穷远处（$\lambda \to 0$）时具有更好的数值特性。

---

## 5. 状态估计与滑动窗口优化

> 📁 **源码位置**：`vins/src/estimator/estimator.cpp`
>
> 程序执行顺序：`processMeasurements()` → `processImage()` → `initialStructure()` / `optimization()` → `slideWindow()`

### 5.1 什么是滑动窗口

相机持续运动时，如果每帧都对**所有历史帧**和**所有地图点**做 Bundle Adjustment（BA），状态向量的维度会随时间无限增长，计算量迅速爆炸，无法在实时系统中运行。

**滑动窗口（Sliding Window）** 的核心思想是：
> 只保留最近 **N 帧** 的状态进行联合优化，当新帧到来时，**最旧的一帧被移出窗口**。但被移出的帧不是直接丢弃，而是通过**边缘化（Marginalization）**将其携带的约束信息转化为**先验（Prior）**保留下来，继续影响窗口内的优化。

在 VINS-Fusion 中，`WINDOW_SIZE = 10`，即窗口内始终保持 **11 帧**（索引 0~10）。

**直观比喻：**

想象一列长度为 11 节的火车在轨道上行驶：

```
时间 t:   [帧0][帧1][帧2]...[帧9][帧10]  ← 当前窗口，联合优化
           ↑                         ↑
         最旧帧                   最新帧

时间 t+1:      [帧1][帧2]...[帧9][帧10][帧11]  ← 窗口滑动
               ↑                              ↑
             最旧帧                         最新帧

帧0 被边缘化 → 变成先验约束 → 继续约束帧1~帧10
```

**滑动窗口 vs 全量 BA：**

| | 滑动窗口 | 全量 BA |
|--|---------|--------|
| 优化帧数 | 固定（11 帧） | 无限增长 |
| 计算复杂度 | O(1)，实时可行 | O(n³)，不可实时 |
| 历史信息 | 通过先验保留 | 全部保留 |
| 长期漂移 | 有（需回环修正） | 无（全局最优） |

---

### 5.2 状态向量

滑动窗口中第 $k$ 个 IMU 状态定义为：

$$
\mathbf{x}_k = \left[ \mathbf{p}_k^W, \mathbf{v}_k^W, \mathbf{q}_k^W, \mathbf{b}_{a_k}, \mathbf{b}_{g_k} \right]
$$

整个滑动窗口的状态向量为：

$$
\mathcal{X} = \left[ \mathbf{x}_0, \mathbf{x}_1, \dots, \mathbf{x}_n, \lambda_0, \lambda_1, \dots, \lambda_m \right]
$$

其中包含 $n+1$ 个 IMU 位姿状态和 $m+1$ 个特征点的逆深度。

**纯双目 VO 模式（`imu: 0`）下的简化：**

当关闭 IMU 时，状态向量退化为仅含位姿和逆深度：

$$
\mathbf{x}_k^{VO} = \left[ \mathbf{p}_k^W, \mathbf{q}_k^W \right], \quad \mathcal{X}^{VO} = \left[ \mathbf{x}_0^{VO}, \dots, \mathbf{x}_n^{VO}, \lambda_0, \dots, \lambda_m \right]
$$

此时没有速度、零偏预积分约束，目标函数退化为纯视觉 BA：

$$
\min_{\mathcal{X}^{VO}} \left\{ \sum_{(k,j) \in \mathcal{C}} \rho \left( \|\mathbf{r}_{C_{kj}}\|_{\boldsymbol{\Sigma}_{C_{kj}}}^2 \right) + \|\mathbf{r}_p\|_{\boldsymbol{\Sigma}_p}^2 \right\}
$$

但滑动窗口的机制和边缘化策略保持不变。

---

### 5.3 目标函数

VINS 通过最小化以下目标函数来估计最优状态：

$$
\min_{\mathcal{X}} \left\{ \sum_{(i,j) \in \mathcal{B}} \|\mathbf{r}_{\mathcal{B}}(\hat{\mathbf{z}}_{i}^{j}, \mathcal{X})\|_{\boldsymbol{\Sigma}_{ij}}^2 + \sum_{(k,j) \in \mathcal{C}} \rho \left( \|\mathbf{r}_{C_{kj}}(\hat{\mathbf{z}}_{k}^{j}, \mathcal{X})\|_{\boldsymbol{\Sigma}_{C_{kj}}}^2 \right) + \|\mathbf{r}_p\|_{\boldsymbol{\Sigma}_p}^2 \right\}
$$

其中：
- 第一项为 **IMU 残差**，衡量预积分预测与状态估计之间的一致性；
- 第二项为 **视觉残差**，衡量特征点重投影误差；
- 第三项为 **先验残差**（来自边缘化）；
- $\|\mathbf{r}\|_{\boldsymbol{\Sigma}}^2 = \mathbf{r}^T \boldsymbol{\Sigma}^{-1} \mathbf{r}$ 为马氏距离；
- $\rho(\cdot)$ 为 Huber 鲁棒核函数，用于降低外点（outliers）的影响；
- $\mathcal{B}$ 为所有 IMU 预积分约束的集合；
- $\mathcal{C}$ 为所有视觉观测约束的集合。

**目标函数中的状态估计值 $\mathcal{X}$ 具体指什么？**

目标函数中的 $\mathcal{X}$ 就是 5.2 节定义的状态向量，包含滑动窗口内所有帧的位姿、速度、IMU 零偏和路标点逆深度。在优化迭代过程中，Ceres Solver 会不断调整 $\mathcal{X}$ 中的每一个变量，使得所有残差的加权平方和最小。

**状态估计值的初始来源可概括为：**

| 状态分量 | 初始值来源 | 说明 |
|---------|-----------|------|
| 位姿 $\mathbf{p}^W, \mathbf{q}^W$ | 视觉 SFM / 上一帧传播 | 初始化时由纯视觉恢复；运行时由上一帧最优结果通过 IMU 积分预测 |
| 速度 $\mathbf{v}^W$ | IMU 积分 / 视觉-惯性对齐 | 初始化时通过预积分与视觉位姿对齐求解；运行时由上一帧速度积分预测 |
| 零偏 $\mathbf{b}_a, \mathbf{b}_g$ | 初始化估计 / 上一帧结果 | 短时间内变化缓慢，可直接用上一帧优化结果作为初值 |
| 逆深度 $\lambda$ | 三角测量 | 由两帧或多帧的观测通过三角化恢复 |

优化开始后，这些初始值被代入残差公式计算残差大小，再通过 LM 算法求解增量 $\delta\mathcal{X}$，更新状态：$\mathcal{X} \leftarrow \mathcal{X} + \delta\mathcal{X}$。重复迭代直到收敛，得到的就是当前滑动窗口的**最优状态估计**。

---

### 5.4 优化求解

VINS 使用 **Ceres Solver** 进行非线性最小二乘优化。Ceres 自动计算雅可比矩阵，并采用列文伯格-马夸特（Levenberg-Marquardt）算法迭代求解：

$$
\left( \mathbf{J}^T \boldsymbol{\Sigma}^{-1} \mathbf{J} + \lambda \mathbf{I} \right) \delta \mathcal{X} = -\mathbf{J}^T \boldsymbol{\Sigma}^{-1} \mathbf{r}
$$

其中 $\mathbf{J}$ 为残差关于状态的雅可比矩阵，$\lambda$ 为阻尼因子。对于视觉残差，Ceres 使用自动求导；对于 IMU 残差，VINS 提供了手写的解析雅可比以提高计算效率。

**滑动窗口中的残差构成（以 11 帧窗口为例）：**

```
帧0 ──[IMU]──► 帧1 ──[IMU]──► 帧2 ──...──► 帧10
  │              │              │              │
  └─[视觉]─► 路标点A        └─[视觉]─► 路标点B
  └─[视觉]─► 路标点C           └─[视觉]─► 路标点D
  └─[先验] (来自帧-1的边缘化)   
```

> 图中的 **路标点 A/B/C/D** 是 3D 空间点，由多帧观测到的 2D **特征点** 三角化恢复。帧 0 观测到路标点 A 和 C（对应图像上的两个特征点），帧 2 也观测到路标点 A（对应另一个特征点）。

每优化一次，所有帧的位姿、所有路标点的深度、所有 IMU 零偏同时被调整，使得：
- IMU 预积分约束（帧间运动）满足
- 视觉重投影约束（像素对应）满足
- 先验约束（历史边缘化信息）满足

优化完成后，**只有最新帧的位姿被输出为当前里程计**，其余帧作为历史参考继续留在窗口中参与后续优化。

---

## 6. 边缘化（Marginalization）

> 📁 **源码位置**：`vins/src/estimator/estimator.cpp:slideWindow()` → `marginalization()`
>
> 程序执行顺序：滑动窗口满 → `marginalization_flag = MARGIN_OLD` → 舒尔补计算先验信息矩阵

滑动窗口的容量有限，当新帧到来时需要移除旧帧以维持固定的计算复杂度。直接丢弃旧帧会丢失其携带的约束信息，因此 VINS 采用 **舒尔补（Schur Complement）** 进行边缘化，将被移除状态所蕴含的信息转化为先验约束保留在系统中。

### 6.1 信息矩阵的舒尔补

假设高斯-牛顿法中的正规方程为：

$$
\begin{bmatrix}
\boldsymbol{\Lambda}_{mm} & \boldsymbol{\Lambda}_{mr} \\
\boldsymbol{\Lambda}_{rm} & \boldsymbol{\Lambda}_{rr}
\end{bmatrix}
\begin{bmatrix}
\delta \mathbf{x}_m \\
\delta \mathbf{x}_r
\end{bmatrix}
=
\begin{bmatrix}
\mathbf{b}_m \\
\mathbf{b}_r
\end{bmatrix}
$$

其中下标 $m$ 表示待边缘化的状态（marginalized），$r$ 表示保留的状态（remaining）。

对 $\delta \mathbf{x}_m$ 进行舒尔补消元，得到仅关于保留状态的先验：

$$
\left( \boldsymbol{\Lambda}_{rr} - \boldsymbol{\Lambda}_{rm} \boldsymbol{\Lambda}_{mm}^{-1} \boldsymbol{\Lambda}_{mr} \right) \delta \mathbf{x}_r = \mathbf{b}_r - \boldsymbol{\Lambda}_{rm} \boldsymbol{\Lambda}_{mm}^{-1} \mathbf{b}_m
$$

定义边缘化后的信息矩阵和向量：

$$
\boldsymbol{\Lambda}_{prior} = \boldsymbol{\Lambda}_{rr} - \boldsymbol{\Lambda}_{rm} \boldsymbol{\Lambda}_{mm}^{-1} \boldsymbol{\Lambda}_{mr}
$$

$$
\mathbf{b}_{prior} = \mathbf{b}_r - \boldsymbol{\Lambda}_{rm} \boldsymbol{\Lambda}_{mm}^{-1} \mathbf{b}_m
$$

### 6.2 边缘化策略

VINS 采用以下边缘化策略：

1. **当滑动窗口满时：** 如果次新帧（second newest frame）不是关键帧，则丢弃该帧的视觉观测，但保留其 IMU 约束，将其与最新帧进行预积分合并。

2. **如果次新帧是关键帧：** 则边缘化最旧帧（oldest frame），将其状态移出滑动窗口，同时将其关联的视觉特征和 IMU 约束转化为先验信息保留在系统中。

### 6.3 边缘化中的 FEJ 问题

由于非线性优化中雅可比矩阵的线性化点会随着迭代改变，直接对信息矩阵进行舒尔补会导致 **线性化点不一致** 的问题。VINS 采用 **First-Estimate Jacobian（FEJ）** 策略，即始终使用第一次估计的线性化点计算雅可比矩阵，从而保证信息矩阵的一致性和正定性。

---

## 7. 视觉-惯性初始化

> 📁 **源码位置**：`vins/src/estimator/estimator.cpp:initialStructure()`
>
> 程序执行顺序：`processImage()` 检测到 `solver_flag == INITIAL` → `initialStructure()` → SFM → 视觉-惯性对齐

VINS 在系统启动时需要进行初始化，以估计初始的尺度、重力方向、速度以及 IMU 零偏。初始化过程分为两个主要阶段：**纯视觉 SFM（Structure from Motion）** 和 **视觉-惯性对齐**。

### 7.1 纯视觉 SFM

利用滑动窗口内的连续图像帧，首先进行纯视觉的运动恢复结构：

1. **本质矩阵估计：** 对相邻帧提取特征点，利用五点法（5-point algorithm）或本质矩阵（Essential Matrix）估计相对位姿。

2. **三角测量：** 根据估计的相对位姿，对匹配的特征点进行三角化，恢复三维点云。

3. **PnP 与全局 BA：** 利用已恢复的三维点，通过 PnP（Perspective-n-Point）求解后续帧的位姿，最后进行全局 Bundle Adjustment 优化所有帧的位姿和路标点。

纯视觉 SFM 得到的是 **无尺度（scale-ambiguous）** 的相机运动轨迹和地图。设视觉 SFM 得到的相机位姿为 $(\mathbf{R}_C^W, \mathbf{p}_C^W)$，由于尺度未知，实际恢复的是 $s \cdot \mathbf{p}_C^W$，其中 $s$ 为未知的尺度因子。

### 7.2 视觉-惯性对齐

将 IMU 预积分与视觉 SFM 结果进行对齐，求解以下未知量：

#### 7.2.1 陀螺仪零偏标定

假设视觉 SFM 得到的第 $i$ 帧到第 $j$ 帧的相对旋转为 $\mathbf{R}_{C_i}^{C_j}$，IMU 预积分得到的相对旋转为 $\boldsymbol{\gamma}_i^j$。两者通过外参旋转 $\mathbf{R}_B^C$ 关联：

$$
\mathbf{R}_B^C \, \mathbf{R}_i^j \, (\mathbf{R}_B^C)^T = \mathbf{R}_{C_i}^{C_j}
$$

或等价地用四元数表示为：

$$
\mathbf{q}_B^C \otimes \boldsymbol{\gamma}_i^j = \mathbf{q}_{C_i}^{C_j} \otimes \mathbf{q}_B^C
$$

由于陀螺仪零偏 $\mathbf{b}_g$ 的存在，预积分旋转存在误差。对 $\boldsymbol{\gamma}$ 关于 $\mathbf{b}_g$ 进行一阶线性化，构建最小二乘问题：

$$
\min_{\delta \mathbf{b}_g} \sum_{(i,j)} \left\| 2 \left[ (\mathbf{q}_{C_i}^{C_j})^{-1} \otimes \mathbf{q}_B^C \otimes \boldsymbol{\gamma}_i^j(\mathbf{b}_g) \otimes (\mathbf{q}_B^C)^{-1} \right]_{xyz} \right\|^2
$$

该问题为线性最小二乘问题，解析求解得到 $\delta\mathbf{b}_g$ 后更新 $\mathbf{b}_g \leftarrow \mathbf{b}_g + \delta\mathbf{b}_g$，然后重新进行 IMU 预积分。

#### 7.2.2 速度、重力向量与尺度的估计

定义待估计量：

- $\mathbf{v}_k^W$：第 $k$ 帧对应 IMU 的速度（在视觉坐标系下）
- $\mathbf{g}^W$：重力向量（在视觉坐标系下）
- $s$：视觉地图的尺度因子

将预积分位置公式与视觉位姿关联。注意视觉 SFM 得到的相机位置与 IMU 位置的关系为：

$$
\mathbf{p}_C^W = \mathbf{p}_B^W + \mathbf{R}_B^W \mathbf{p}_C^B = \mathbf{p}_B^W - \mathbf{R}_B^W \mathbf{R}_B^C \mathbf{p}_B^C
$$

（因为 $\mathbf{p}_C^B = -\mathbf{R}_B^C \mathbf{p}_B^C$，即 camera 原点在 body 系下的坐标）

为简化推导，假设外参平移已知的条件下，可以建立 IMU 位置与相机位置的关系。将预积分位置公式：

$$
\mathbf{p}_j^W = \mathbf{p}_i^W + \mathbf{v}_i^W \Delta t - \frac{1}{2} \mathbf{g}^W \Delta t^2 + \mathbf{R}_i^W \boldsymbol{\alpha}_i^j
$$

转换为相机坐标（忽略外参平移的微小影响或将其合并到方程一侧），得到关于 $\mathbf{v}_k^W$、$\mathbf{g}^W$ 和 $s$ 的线性方程：

$$
s \left( \bar{\mathbf{p}}_{C_j}^W - \bar{\mathbf{p}}_{C_i}^W \right) = \mathbf{v}_i^W \Delta t - \frac{1}{2} \mathbf{g}^W \Delta t^2 + \mathbf{R}_{C_i}^W \boldsymbol{\alpha}_i^j
$$

其中 $\bar{\mathbf{p}}_{C_k}^W$ 为视觉 SFM 得到的未缩放相机位置（即 $s=1$ 时的位置）。该方程对每对相邻帧成立，构成线性方程组。

将所有帧的约束堆叠，写成矩阵形式 $\mathbf{A}\mathbf{x} = \mathbf{b}$，其中：

$$
\mathbf{x} = \left[ (\mathbf{v}_0^W)^T, (\mathbf{v}_1^W)^T, \dots, (\mathbf{v}_n^W)^T, (\mathbf{g}^W)^T, s \right]^T
$$

通过最小二乘法求解该线性系统，得到所有帧的速度、重力方向和尺度因子。

#### 7.2.3 重力方向精化

由于重力向量的模长已知（$|\mathbf{g}^W| = g \approx 9.81$），可以进一步对重力方向进行精化。将重力向量表示为：

$$
\mathbf{g}^W = g \cdot \bar{\hat{\mathbf{g}}}^W + w_1 \mathbf{b}_1 + w_2 \mathbf{b}_2
$$

其中 $\bar{\hat{\mathbf{g}}}^W$ 为之前估计的重力方向单位向量，$\mathbf{b}_1, \mathbf{b}_2$ 为垂直于 $\bar{\hat{\mathbf{g}}}^W$ 的切平面基底，$w_1, w_2$ 为小量修正。重新求解线性系统，得到更精确的重力方向和更准确的尺度估计。

#### 7.2.4 加速度计零偏标定

将加速度计零偏也纳入考虑，位置预积分公式修正为：

$$
\boldsymbol{\alpha}_i^j(\mathbf{b}_a) \approx \hat{\boldsymbol{\alpha}}_i^j + \mathbf{J}_{b_a}^{\alpha} \, \delta\mathbf{b}_a
$$

其中 $\mathbf{J}_{b_a}^{\alpha}$ 为预积分位置关于加速度计零偏的雅可比矩阵（在预积分过程中同步计算）。将修正后的预积分代入线性系统，可以进一步估计加速度计零偏 $\mathbf{b}_a$。

### 7.3 初始化后的状态转换

完成初始化后，将视觉坐标系下的状态转换到以重力方向为 $Z$ 轴的世界坐标系中，并将尺度缩放到真实尺度。此时系统进入正常的滑动窗口优化模式。

---

## 8. 回环检测与位姿图优化

> 📁 **源码位置**：
> - `loop_fusion/src/pose_graph_node.cpp:process()` — 主线程
> - `loop_fusion/src/pose_graph.cpp:detectLoop()` — DBoW2 回环检测
> - `loop_fusion/src/keyframe.cpp:findConnection()` — PnP 几何验证
> - `loop_fusion/src/pose_graph.cpp:optimize6DoF()` — Ceres 位姿图优化
>
> 程序执行顺序：`process()` 接收关键帧 → `detectLoop()` 词袋查询 → `findConnection()` 几何验证 → `optimize6DoF()` 全局优化 → `/odometry_rect`

### 8.1 回路检测

VINS-Fusion 的 `loop_fusion` 模块使用 **DBoW2（Bag of Words）** 进行回路检测。关键帧提取 BRIEF 描述子，与词袋数据库中的历史关键帧进行匹配。当检测到回路候选帧时，通过几何验证（PnP + RANSAC）确认回路闭合。

### 8.2 4-DoF 位姿图优化

由于视觉惯性系统能够准确估计重力方向（roll 和 pitch），回路闭合后的全局优化仅在水平面上进行 **4-DoF（x, y, z, yaw）** 位姿图优化：

$$
\min_{\{\mathbf{p}, \psi\}} \left\{ \sum_{(i,j) \in \mathcal{S}} \|\mathbf{r}_{ij}\|^2 + \sum_{(i,j) \in \mathcal{L}} \|\mathbf{r}_{loop}\|^2 \right\}
$$

其中：
- $\mathcal{S}$ 为相邻关键帧之间的序列约束（由 VIO 提供）；
- $\mathcal{L}$ 为回路约束（由回路检测提供）。

位姿图优化显著抑制了长距离运行中的累积漂移。

---

## 9. 可视化输出

> 📁 **源码位置**：`vins/src/utility/visualization.cpp`
>
> 程序执行顺序：`optimization()` 完成后 → `pubOdometry()` / `pubKeyPoses()` / `pubPointCloud()` / `pubTF()` → ROS2 话题发布

| 话题名 | 类型 | 说明 | 对应函数 |
|--------|------|------|---------|
| `/vins_estimator/odometry` | `nav_msgs/Odometry` | 实时位姿 | `pubOdometry()` |
| `/vins_estimator/path` | `nav_msgs/Path` | 轨迹 | `pubKeyPoses()` |
| `/vins_estimator/point_cloud` | `sensor_msgs/PointCloud` | 地图点 | `pubPointCloud()` |
| `/vins_estimator/image_track` | `sensor_msgs/Image` | 特征跟踪图 | `pubTrackImage()` |
| `/tf` | `tf2_msgs/TFMessage` | TF 变换 | `pubTF()` |

---

## 10. 总结

本文从数学角度系统梳理了 VINS-Fusion 的核心模型：

| 模块 | 核心数学工具 | 作用 |
|------|-------------|------|
| IMU 预积分 | 微分方程、中值积分、误差状态传播 | 避免重复积分，提供帧间运动约束 |
| 视觉重投影 | 针孔投影、逆深度参数化 | 提供特征点几何约束 |
| 滑动窗口优化 | 非线性最小二乘、Ceres Solver | 融合多源观测，估计最优状态 |
| 边缘化 | 舒尔补、FEJ | 维持固定计算量，保留历史信息 |
| 初始化 | SFM、线性最小二乘 | 恢复尺度、重力方向和 IMU 零偏 |
| 回路检测 | DBoW2、4-DoF 位姿图 | 消除长期累积漂移 |

VINS 的精髓在于将高频率的 IMU 数据与低频率的视觉观测进行紧耦合融合，利用 IMU 提供良好的短时运动预测，利用视觉提供长时无漂移的位姿校正，两者互补实现了鲁棒、高精度的实时位姿估计。
