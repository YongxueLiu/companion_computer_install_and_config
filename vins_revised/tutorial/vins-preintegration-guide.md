# VINS 预积分：从零开始的数值推导

预积分（IMU Pre-integration）是 VINS 中最抽象、也最难理解的模块之一。本文抛弃繁琐的公式堆砌，用**具体的数值例子**一步一步带你算清楚：预积分到底在算什么？为什么它能避免重复积分？协方差和零偏修正又是怎么回事？

---

## 1. 一句话总结

**预积分 = 在两帧图像之间，用 IMU 原始测量算出的"相对运动量"（相对位移 + 相对速度变化 + 相对旋转）。**

它的核心好处是：**这个"相对运动量"只取决于这两帧之间的 IMU 数据，和你从哪出发、初速度多少、朝向如何完全无关。** 因此优化更新位姿时，它不需要重新计算。

---

## 2. 直接积分的痛点：为什么要搞预积分？

假设你有一个机器人，IMU 测得它在 x 方向以 $2 \, \text{m/s}^2$ 加速。你想知道 0.1 秒后它到哪了、速度多少。

**直接积分（在世界坐标系下）：**

$$
v(t) = v(0) + \int_0^t a(\tau) \, d\tau
$$

$$
p(t) = p(0) + \int_0^t v(\tau) \, d\tau
$$

如果初始速度 $v(0) = 1 \, \text{m/s}$，那么：
- $v(0.1) = 1 + 2 \times 0.1 = 1.2 \, \text{m/s}$
- $p(0.1) = 0 + 1 \times 0.1 + \frac{1}{2} \times 2 \times 0.1^2 = 0.11 \, \text{m}$

现在问题来了：在优化过程中，算法发现初始速度其实不是 $1.0$，而是 $1.5 \, \text{m/s}$。

**直接积分的后果：** 你必须把上面的积分**全部重算一遍**：
- $v(0.1) = 1.5 + 0.2 = 1.7 \, \text{m/s}$
- $p(0.1) = 0 + 1.5 \times 0.1 + 0.01 = 0.16 \, \text{m}$

每次优化更新初速度、初始位置或初始姿态，都要重新积分一次，计算量爆炸。

**预积分的思路：** 把"相对运动"从积分结果里**剥离**出来，让它只和 IMU 测量有关。

---

## 3. 数值例子 1：无旋转的直线运动（建立直觉）

### 3.1 场景设定

为了最简单，假设：
- 机器人沿直线（x 轴）运动，**没有旋转**；
- 世界坐标系和 IMU 坐标系永远对齐（$\mathbf{R}_B^W = \mathbf{I}$）；
- 运动发生在水平面，重力在 z 方向，与 x 方向无关，我们可以暂时忽略重力；
- 图像帧在 $t_i = 0$ 和 $t_j = 0.1 \, \text{s}$ 拍摄；
- IMU 采样频率 100 Hz，两帧之间有 10 个 IMU 采样，每个间隔 $\Delta t = 0.01 \, \text{s}$；
- 为了更简单，假设 IMU 在这 0.1 s 内测得的加速度恒定：$\hat{a}_x = 2 \, \text{m/s}^2$；
- 陀螺仪测得角速度为 0；
- 零偏和噪声都暂时忽略（后面再加）。

### 3.2 直接积分（在世界系下）

IMU 的加速度测量直接就是世界系下的加速度（因为没有旋转）。运动方程：

$$
\dot{v} = \hat{a} = 2
$$

$$
\dot{p} = v
$$

从 $t=0$ 积分到 $t=0.1$：

| 时刻 | 速度 $v$ (m/s) | 位置 $p$ (m) |
|------|---------------|-------------|
| $t=0$ | $v_i = 1.0$ | $p_i = 0.0$ |
| $t=0.1$ | $v_j = 1.0 + 2 \times 0.1 = \mathbf{1.2}$ | $p_j = 0.0 + 1.0 \times 0.1 + \frac{1}{2} \times 2 \times 0.1^2 = \mathbf{0.11}$ |

### 3.3 预积分计算

**预积分量的定义（连续形式）：**

$$
\boldsymbol{\beta}_i^j = \int_0^{0.1} \mathbf{R}_i^\tau \hat{a}_\tau \, d\tau, \quad
\boldsymbol{\alpha}_i^j = \int_0^{0.1} \int_0^s \mathbf{R}_i^\tau \hat{a}_\tau \, d\tau \, ds, \quad
\boldsymbol{\gamma}_i^j = \mathbf{I}
$$

因为没有旋转（$\mathbf{R}_i^\tau = \mathbf{I}$），且 $\hat{a} = 2$ 恒定：

$$
\boldsymbol{\beta}_i^j = \int_0^{0.1} 2 \, d\tau = \mathbf{0.2} \, \text{m/s}
$$

$$
\boldsymbol{\alpha}_i^j = \int_0^{0.1} 2s \, ds = \left[ s^2 \right]_0^{0.1} = \mathbf{0.01} \, \text{m}
$$

$$
\boldsymbol{\gamma}_i^j = \mathbf{I} \quad \text{（无旋转）}
$$

**关键：这两个数只和 IMU 测到的加速度有关，和初速度 $v_i$、初始位置 $p_i$ 完全无关。**

### 3.4 用预积分更新状态

预积分给出的递推公式：

$$
v_j = v_i + \boldsymbol{\beta}_i^j = 1.0 + 0.2 = \mathbf{1.2} \, \text{m/s}
$$

$$
p_j = p_i + v_i \Delta t + \boldsymbol{\alpha}_i^j = 0.0 + 1.0 \times 0.1 + 0.01 = \mathbf{0.11} \, \text{m}
$$

结果和直接积分**完全一致**。

### 3.5 为什么预积分不需要重算？

现在优化更新了，发现初始速度不是 $1.0$，而是 $v_i = 1.5 \, \text{m/s}$。

**直接积分：** 必须从头重算：

$$
p_j = 0.0 + 1.5 \times 0.1 + \frac{1}{2} \times 2 \times 0.1^2 = 0.16 \, \text{m}
$$

**预积分：** $\boldsymbol{\alpha}_i^j = 0.01$ 和 $\boldsymbol{\beta}_i^j = 0.2$ **完全不变**，直接代入新速度：

$$
p_j = 0.0 + 1.5 \times 0.1 + 0.01 = 0.16 \, \text{m}
$$

**省掉了积分的全过程！** 这就是预积分的威力。

---

## 4. 数值例子 2：带旋转的平面运动（真正的预积分）

上面的例子太简单，没有旋转。一旦物体旋转，加速度的方向就会变，预积分的价值才真正体现。

### 4.1 场景设定

- 机器人从静止开始，在水平面内运动；
- 绕 z 轴以恒定角速度 $\omega_z = \pi/2 \, \text{rad/s}$（即 $90°/\text{s}$）旋转；
- 同时，IMU 在 body 系的 x 轴方向测得恒定加速度 $a_x = 1 \, \text{m/s}^2$（即机器人一直在"往前冲"）；
- 图像帧在 $t_i = 0$ 和 $t_j = 0.1 \, \text{s}$ 拍摄；
- 为了手算方便，假设 0.1 s 内只有**一个 IMU 采样间隔**（即 IMU 频率为 10 Hz，不现实但便于演示）；
- 初始状态：$\mathbf{p}_i^W = [0, 0, 0]^T$，$\mathbf{v}_i^W = [0, 0, 0]^T$，$\mathbf{R}_i^W = \mathbf{I}$（body 与世界对齐）；
- 忽略零偏和噪声。

### 4.2 手算预积分（中值积分）

VINS 使用**中值积分**。在一个采样间隔内，用两端测量的平均值作为该区间内的恒定值。

**陀螺仪测量：**
- 两端角速度都是 $\hat{\boldsymbol{\omega}} = [0, 0, \pi/2]^T$
- 中值：$\boldsymbol{\omega} = [0, 0, \pi/2]^T$

**旋转角度：**

$$
\theta = \|\boldsymbol{\omega}\| \Delta t = \frac{\pi}{2} \times 0.1 = 0.15708 \, \text{rad} \approx 9°
$$

**姿态预积分 $\boldsymbol{\gamma}_i^j$（四元数）：**

四元数更新公式（小角度近似下）：

$$
\delta\mathbf{q} = \begin{bmatrix} \cos(\theta/2) \\ \frac{\boldsymbol{\omega}}{\|\boldsymbol{\omega}\|} \sin(\theta/2) \end{bmatrix}
= \begin{bmatrix} \cos(0.0785) \\ 0 \\ 0 \\ \sin(0.0785) \end{bmatrix}
\approx \begin{bmatrix} 0.9969 \\ 0 \\ 0 \\ 0.0785 \end{bmatrix}
$$

$$
\boldsymbol{\gamma}_i^j = [1, 0, 0, 0] \otimes \delta\mathbf{q} = \begin{bmatrix} 0.9969 \\ 0 \\ 0 \\ 0.0785 \end{bmatrix}
$$

对应的旋转矩阵（手算验证）：

$$
\mathbf{R}_i^j \approx \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}
= \begin{bmatrix} 0.9877 & -0.1564 & 0 \\ 0.1564 & 0.9877 & 0 \\ 0 & 0 & 1 \end{bmatrix}
$$

**加速度计测量：**
- 两端加速度都是 $\hat{\mathbf{a}} = [1, 0, 0]^T$（body 系下）
- 但注意：body 系在旋转，所以同一方向的加速度在世界系下的方向在变化！

$t=0$ 时，body x 轴与世界 x 轴对齐，所以 body 系下的 $[1, 0, 0]$ 在世界系下也是 $[1, 0, 0]$。

$t=0.1$ 时，body 系已旋转了 $9°$，body x 轴在世界系下变为：

$$
\mathbf{R}_i^j \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix}
= \begin{bmatrix} 0.9877 \\ 0.1564 \\ 0 \end{bmatrix}
$$

**速度预积分 $\boldsymbol{\beta}_i^j$：**

中值积分取两个方向上的平均值：

$$
\text{平均方向} = \frac{1}{2} \left( \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix} + \begin{bmatrix} 0.9877 \\ 0.1564 \\ 0 \end{bmatrix} \right)
= \begin{bmatrix} 0.9939 \\ 0.0782 \\ 0 \end{bmatrix}
$$

$$
\boldsymbol{\beta}_i^j = \text{平均方向} \times a_x \times \Delta t
= \begin{bmatrix} 0.9939 \\ 0.0782 \\ 0 \end{bmatrix} \times 1 \times 0.1
= \begin{bmatrix} \mathbf{0.0994} \\ \mathbf{0.0078} \\ 0 \end{bmatrix} \, \text{m/s}
$$

**位置预积分 $\boldsymbol{\alpha}_i^j$：**

中值积分的位置更新公式：

$$
\boldsymbol{\alpha}_i^j = \boldsymbol{\beta}_{\text{old}} \Delta t + \frac{1}{4} \left( \mathbf{R}_k \mathbf{a}_k + \mathbf{R}_{k+1} \mathbf{a}_{k+1} \right) \Delta t^2
$$

由于初始速度为 0，$\boldsymbol{\beta}_{\text{old}} = [0, 0, 0]^T$：

$$
\boldsymbol{\alpha}_i^j = \mathbf{0} + \frac{1}{4} \left( \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix} + \begin{bmatrix} 0.9877 \\ 0.1564 \\ 0 \end{bmatrix} \right) \times 0.1^2
= \frac{1}{4} \begin{bmatrix} 1.9877 \\ 0.1564 \\ 0 \end{bmatrix} \times 0.01
= \begin{bmatrix} \mathbf{0.00497} \\ \mathbf{0.00039} \\ 0 \end{bmatrix} \, \text{m}
$$

### 4.3 用预积分更新状态

有了预积分量，直接代入递推公式（假设重力不影响水平面运动，暂不考虑）：

**位置：**

$$
\mathbf{p}_j^W = \mathbf{p}_i^W + \mathbf{v}_i^W \Delta t + \mathbf{R}_i^W \boldsymbol{\alpha}_i^j
= \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix} + \mathbf{0} + \mathbf{I} \cdot \begin{bmatrix} 0.00497 \\ 0.00039 \\ 0 \end{bmatrix}
= \begin{bmatrix} \mathbf{0.00497} \\ \mathbf{0.00039} \\ 0 \end{bmatrix} \, \text{m}
$$

**速度：**

$$
\mathbf{v}_j^W = \mathbf{v}_i^W + \mathbf{R}_i^W \boldsymbol{\beta}_i^j
= \begin{bmatrix} \mathbf{0.0994} \\ \mathbf{0.0078} \\ 0 \end{bmatrix} \, \text{m/s}
$$

**姿态：**

$$
\mathbf{R}_j^W = \mathbf{R}_i^W \mathbf{R}_i^j
= \mathbf{I} \cdot \begin{bmatrix} 0.9877 & -0.1564 & 0 \\ 0.1564 & 0.9877 & 0 \\ 0 & 0 & 1 \end{bmatrix}
= \begin{bmatrix} \mathbf{0.9877} & \mathbf{-0.1564} & 0 \\ \mathbf{0.1564} & \mathbf{0.9877} & 0 \\ 0 & 0 & 1 \end{bmatrix}
$$

### 4.4 验证：直接积分法

我们验证一下这个结果是否合理。

在 0.1 s 内，机器人从静止开始，一直沿着自己的 x 轴加速，同时绕 z 轴转了 $9°$。

- 如果不旋转，0.1 s 后的速度应该是 $1 \times 0.1 = 0.1$ m/s，沿 x 轴；
- 由于转了 $9°$，速度方向也偏了约 $4.5°$（平均角度），所以速度应该偏向 x 和 y 的正方向；
- 我们算得 $[0.0994, 0.0078, 0]$，确实 x 方向约 $0.1$，y 方向有个小的正分量，符合直觉。

位置同理：平均速度约为 $[0.05, 0.004, 0]$，乘以 0.1 s 得 $[0.005, 0.0004, 0]$，和我们算的 $[0.00497, 0.00039, 0]$ **几乎完全一致**。

### 4.5 预积分的真正威力

假设优化发现初始姿态不是 $\mathbf{I}$，而是绕 z 轴偏了 $5°$。直接积分需要从新的初始姿态开始，重新计算每一时刻 body 系在世界系下的方向，再把 body 系加速度一次次转到世界系，积分求速度和位置——**全部重算**。

预积分法：**$\boldsymbol{\alpha}_i^j$、$\boldsymbol{\beta}_i^j$、$\boldsymbol{\gamma}_i^j$ 完全不需要重算！** 直接代入新的 $\mathbf{R}_i^W$ 即可：

$$
\mathbf{p}_j^W = \mathbf{p}_i^W + \mathbf{v}_i^W \Delta t + \mathbf{R}_i^W \boldsymbol{\alpha}_i^j
$$

只有最后乘了一个初始姿态 $\mathbf{R}_i^W$，预积分量本身纹丝不动。

---

## 5. 预积分的协方差传播（直观理解 + 数值）

IMU 测量有噪声，预积分量也不可能完全准确。我们需要知道预积分结果的"不确定度"有多大，这样才能在优化中给它们合适的权重（协方差越大，权重越低）。

### 5.1 误差状态的定义

真实值 = 预积分值 + 误差。定义 15 维误差向量：

$$
\delta\mathbf{x} = \begin{bmatrix} \delta\boldsymbol{\alpha} & \delta\boldsymbol{\beta} & \delta\boldsymbol{\theta} & \delta\mathbf{b}_a & \delta\mathbf{b}_g \end{bmatrix}^T
$$

其中：
- $\delta\boldsymbol{\alpha}$：位置预积分的误差（3维）
- $\delta\boldsymbol{\beta}$：速度预积分的误差（3维）
- $\delta\boldsymbol{\theta}$：姿态预积分的误差（3维，旋转向量）
- $\delta\mathbf{b}_a, \delta\mathbf{b}_g$：加速度计和陀螺仪零偏的误差（各3维）

### 5.2 线性递推公式

误差状态满足线性递推：

$$
\delta\mathbf{x}_{k+1} = \mathbf{F}_k \, \delta\mathbf{x}_k + \mathbf{G}_k \, \mathbf{n}_k
$$

其中 $\mathbf{n}_k = [\mathbf{n}_a, \mathbf{n}_g, \mathbf{n}_{b_a}, \mathbf{n}_{b_g}]^T$ 是噪声向量，$\mathbf{F}_k$ 和 $\mathbf{G}_k$ 是 $15 \times 15$ 和 $15 \times 12$ 的矩阵。

### 5.3 数值例子：协方差怎么传播？

继续上面的**带旋转例子**，假设：
- 加速度计噪声标准差：$\sigma_a = 0.1 \, \text{m/s}^2$
- 陀螺仪噪声标准差：$\sigma_g = 0.01 \, \text{rad/s}$
- 零偏随机游走标准差：$\sigma_{b_a} = 0.001$，$\sigma_{b_g} = 0.0001$
- 采样间隔：$\Delta t = 0.1 \, \text{s}$（为了简化，只有一个间隔）

**初始协方差：** $\boldsymbol{\Sigma}_0 = \mathbf{0}_{15 \times 15}$（开始时完全确定）

**计算 $\mathbf{F}$ 矩阵（取关键行块）：**

对于姿态误差：

$$
\delta\boldsymbol{\theta}_{k+1} \approx \delta\boldsymbol{\theta}_k - \Delta t \, \delta\mathbf{b}_{g_k} + \Delta t \, \mathbf{n}_{g_k}
$$

这说明：陀螺仪噪声 $\mathbf{n}_g$ 和零偏误差 $\delta\mathbf{b}_g$ 都会让姿态预积分越来越"飘"。

对于速度误差：

$$
\delta\boldsymbol{\beta}_{k+1} \approx \delta\boldsymbol{\beta}_k + \Delta t \, \mathbf{n}_{a_k}
$$

（简化版，忽略了姿态误差对加速度投影的影响）

**协方差更新：**

$$
\boldsymbol{\Sigma}_{k+1} = \mathbf{F} \boldsymbol{\Sigma}_k \mathbf{F}^T + \mathbf{G} \mathbf{Q} \mathbf{G}^T
$$

其中 $\mathbf{Q} = \text{diag}(\sigma_a^2, \sigma_g^2, \sigma_{b_a}^2, \sigma_{b_g}^2)$。

经过一步传播后，速度预积分的方差约为：

$$
\text{Var}(\beta_x) \approx (\Delta t)^2 \sigma_a^2 = 0.01 \times 0.01 = 0.0001 \, (\text{m/s})^2
$$

即标准差约 $0.01$ m/s。

姿态预积分的方差约为：

$$
\text{Var}(\theta_z) \approx (\Delta t)^2 \sigma_g^2 = 0.01 \times 0.0001 = 1 \times 10^{-6} \, \text{rad}^2
$$

即标准差约 $0.001$ rad（约 $0.06°$）。

**随着时间延长，协方差会不断累积。** 0.1 s 内还好，但如果两帧图像间隔 1 秒，预积分的误差就会大得多。这就是为什么 VIO 需要高频率的图像帧来"修正"IMU 的累积漂移。

---

## 6. 预积分的零偏修正（数值例子）

在实际优化中，IMU 的零偏 $\mathbf{b}_a$ 和 $\mathbf{b}_g$ 是待优化的状态量，会不断更新。每次零偏变了都重新做预积分，太慢了。VINS 用**一阶泰勒展开**来近似修正。

### 6.1 问题设定

继续**带旋转的例子**。假设：
- 做预积分时，我们猜测加速度计零偏为 $\bar{\mathbf{b}}_a = [0.1, 0, 0]^T$；
- 优化更新后发现，真实零偏应该是 $\mathbf{b}_a = [0.2, 0, 0]^T$；
- 偏差量：$\delta\mathbf{b}_a = \mathbf{b}_a - \bar{\mathbf{b}}_a = [0.1, 0, 0]^T$。

### 6.2 预积分时用了错误的零偏

预积分时，我们把 IMU 原始测量 $\hat{a} = [1, 0, 0]$ 扣除了猜测的零偏：

$$
a_{\text{used}} = \hat{a} - \bar{b}_a = [1, 0, 0] - [0.1, 0, 0] = [0.9, 0, 0]
$$

基于 $a = 0.9$ 算出的预积分位置：

$$
\hat{\boldsymbol{\alpha}} = \int\int [0.9, 0, 0]^T \, dt^2 = [0.0045, 0, 0]^T \, \text{m}
$$

（对比真实值：如果用正确的 $a = 0.8$，应该是 $[0.0040, 0, 0]^T$）

### 6.3 一阶修正

预积分位置对零偏的雅可比矩阵（在预积分过程中同步算出）：

$$
\mathbf{J}_{b_a}^{\alpha} = \frac{\partial \boldsymbol{\alpha}}{\partial \mathbf{b}_a} \approx -\frac{1}{2} \mathbf{I} \, \Delta t^2
= -\begin{bmatrix} 0.005 & 0 & 0 \\ 0 & 0.005 & 0 \\ 0 & 0 & 0.005 \end{bmatrix}
$$

（这是无旋转时的近似；有旋转时是类似的带旋转矩阵的积分形式）

修正量：

$$
\Delta \boldsymbol{\alpha} = \mathbf{J}_{b_a}^{\alpha} \, \delta\mathbf{b}_a
= -\begin{bmatrix} 0.005 & 0 & 0 \\ 0 & 0.005 & 0 \\ 0 & 0 & 0.005 \end{bmatrix}
\begin{bmatrix} 0.1 \\ 0 \\ 0 \end{bmatrix}
= \begin{bmatrix} -0.0005 \\ 0 \\ 0 \end{bmatrix}
$$

修正后的预积分位置：

$$
\boldsymbol{\alpha}_{\text{corrected}} = \hat{\boldsymbol{\alpha}} + \Delta\boldsymbol{\alpha}
= \begin{bmatrix} 0.0045 \\ 0 \\ 0 \end{bmatrix} + \begin{bmatrix} -0.0005 \\ 0 \\ 0 \end{bmatrix}
= \begin{bmatrix} 0.0040 \\ 0 \\ 0 \end{bmatrix}
$$

**和直接用正确零 bias 重算的结果 $[0.0040, 0, 0]$ 完全一致！**

### 6.4 为什么一阶修正有效？

因为预积分量关于零偏**几乎是线性的**（位置预积分 $\approx$ 加速度 $\times t^2$，加速度 $\approx$ 测量值 $-$ 零偏）。在零偏变化不大的情况下（通常每次优化只更新一点点），一阶泰勒展开非常准确，完全不需要重新积分。

---

## 7. 总结：预积分的完整计算流程

至此，我们已经从数值上完整走通了预积分的全过程。总结如下：

```
输入：两帧图像之间的所有 IMU 测量 {â_k, ω̂_k}
输出：预积分量 (α, β, γ) 及其协方差 Σ、零偏雅可比 J

1. 初始化：
   α = 0, β = 0, γ = [1,0,0,0]（单位四元数）
   Σ = 0_{15×15}, J = I_{15}

2. 对每个 IMU 采样间隔 [t_k, t_{k+1}]：
   a. 中值积分：取 a_k 和 a_{k+1} 的平均，ω_k 和 ω_{k+1} 的平均
   b. 更新姿态预积分 γ（四元数乘法）
   c. 更新速度预积分 β
   d. 更新位置预积分 α
   e. 计算 F 和 G 矩阵
   f. 更新协方差：Σ ← F·Σ·F^T + G·Q·G^T
   g. 更新雅可比：J ← F·J

3. 输出：α, β, γ, Σ, J_{b_a}^α, J_{b_a}^β, J_{b_g}^γ
```

**优化的使用方式：**

```
IMU 残差 = [  R_W^i (p_j - p_i - v_i Δt + ½ g Δt²) - α
             R_W^i (v_j - v_i + g Δt) - β
             2[(q_i)^{-1} ⊗ q_j ⊗ γ^{-1}]_{xyz}
             b_{a_j} - b_{a_i}
             b_{g_j} - b_{g_i} ]
```

当零偏变化时：
```
α_corrected = α + J_{b_a}^α · δb_a
β_corrected = β + J_{b_a}^β · δb_a
γ_corrected = γ ⊗ [1, ½ J_{b_g}^γ · δb_g]
```

---

## 8. 关键要点回顾

| 问题 | 答案 |
|------|------|
| 预积分到底是什么？ | 两帧图像之间，IMU 测出的**相对运动量**（位移增量、速度增量、旋转增量） |
| 为什么能避免重算？ | 它只和 IMU 数据有关，和绝对位姿、初速度无关 |
| 旋转为什么麻烦？ | 旋转时加速度的方向一直在变，不能简单做标量积分 |
| 中值积分是干嘛的？ | 用一个采样间隔内的平均测量值近似连续积分，减小离散化误差 |
| 协方差传播是干嘛的？ | 告诉优化器"这个预积分结果有多可信"，噪声越大权重越低 |
| 零偏修正是干嘛的？ | 优化更新零偏后，用一阶近似快速修正预积分结果，避免重新积分 |

理解预积分的关键，就是反复问自己一句话：**"这个东西和初始位姿有关吗？"** 如果有关，就不是预积分；如果无关，就是预积分。
