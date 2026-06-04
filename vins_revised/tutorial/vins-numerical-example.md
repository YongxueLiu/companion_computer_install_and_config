# VINS 完整计算流程数值示例

本文通过一个**可以手算**的极简场景，一步一步带你走完 VINS 的核心计算流程：从传感器数据生成，到初始化，再到滑动窗口优化的一次完整 LM 迭代。所有步骤都给出**具体数值**，方便对照验证。

---

## 1. 场景设定

### 1.1 坐标系与世界参数

- **世界系 $\mathcal{W}$**：$x$ 轴向前，$y$ 轴向左，$z$ 轴**向下**（与 VINS 代码约定一致）
- **重力向量**：$\mathbf{g}^W = [0, 0, 9.81]^T \, \text{m/s}^2$
- **Body 系**（IMU）与 **Camera 系** 完全重合：$\mathbf{R}_B^C = \mathbf{I}$，$\mathbf{p}_B^C = \mathbf{0}$
- **相机内参**：$f_x = f_y = 500$，$c_x = c_y = 320$，无畸变

### 1.2 真实运动

机器人沿 $x$ 轴做**匀速直线运动**，速度 $v_x = 1.0 \, \text{m/s}$，无旋转。

| 帧 | 时刻 | 位置 $\mathbf{p}^W$ | 速度 $\mathbf{v}^W$ | 姿态 $\mathbf{R}_B^W$ |
|----|------|---------------------|---------------------|-----------------------|
| 0  | $t=0.0$ | $[0.00, 0, 0]$ | $[1.00, 0, 0]$ | $\mathbf{I}$ |
| 1  | $t=0.1$ | $[0.10, 0, 0]$ | $[1.00, 0, 0]$ | $\mathbf{I}$ |
| 2  | $t=0.2$ | $[0.20, 0, 0]$ | $[1.00, 0, 0]$ | $\mathbf{I}$ |

### 1.3 路标点

世界系下有一个静止特征点：

$$
\mathbf{P}^W = [1.0, 0, 5.0]^T \, \text{m}
$$

在 camera 系下的真实坐标（第 $k$ 帧）：$\mathbf{P}^{C_k} = \mathbf{P}^W - \mathbf{p}_k^W$。

---

## 2. 传感器数据生成

### 2.1 IMU 测量

机器人匀速运动，真实运动加速度为 $\mathbf{0}$。IMU 测量的是**比力**（specific force），即非引力外力。当 $z$ 轴向下时，静止或匀速时 IMU 感受到的是向上的支持力，故比力为：

$$
\hat{\mathbf{a}} = [0, 0, 9.81]^T \, \text{m/s}^2
$$

陀螺仪测量（无旋转）：

$$
\hat{\boldsymbol{\omega}} = [0, 0, 0]^T \, \text{rad/s}
$$

零偏：$\mathbf{b}_a = \mathbf{0}$，$\mathbf{b}_g = \mathbf{0}$，噪声暂忽略。

IMU 采样频率 100 Hz，两帧图像间有 10 个采样，$\Delta t_{\text{imu}} = 0.01 \, \text{s}$。

### 2.2 视觉测量（特征点像素坐标）

针孔投影：$u = f_x \frac{X}{Z} + c_x$，$v = f_y \frac{Y}{Z} + c_y$。

| 帧 | $\mathbf{P}^{C_k}$ | 像素 $(u, v)$ |
|----|---------------------|---------------|
| 0  | $[1.00, 0, 5.00]$ | $(420, 320)$ |
| 1  | $[0.90, 0, 5.00]$ | $(410, 320)$ |
| 2  | $[0.80, 0, 5.00]$ | $(400, 320)$ |

验证（帧 0）：$u = 500 \times \frac{1}{5} + 320 = 420$，$v = 500 \times \frac{0}{5} + 320 = 320$。✓

---

## 3. 初始化阶段

### 3.1 纯视觉 SFM（Structure from Motion）

利用帧 0 和帧 1 的对应特征点 $(420, 320)$ 和 $(410, 320)$：

**本质矩阵：** 由于运动是纯平移且无旋转，本质矩阵 $\mathbf{E} = [\mathbf{t}]_\times \mathbf{R} = [\mathbf{t}]_\times$，其中平移方向为 $x$ 轴。

**三角化求深度：**

帧 0 的归一化射线方向：$\mathbf{m}_0 = \left[\frac{420-320}{500}, \frac{320-320}{500}, 1\right]^T = [0.2, 0, 1]^T$

帧 1 的归一化射线方向：$\mathbf{m}_1 = \left[\frac{410-320}{500}, 0, 1\right]^T = [0.18, 0, 1]^T$

设尺度因子为 $s$，深度为 $d_0$，则：

$$
d_0 \mathbf{m}_0 - d_1 \mathbf{m}_1 = s \cdot [1, 0, 0]^T
$$

由 $z$ 分量：$d_0 = d_1$；由 $x$ 分量：$0.2 d_0 - 0.18 d_0 = s \Rightarrow d_0 = 50s$。

真实深度为 $5.0 \, \text{m}$，故 $50s = 5.0 \Rightarrow \mathbf{s = 0.1}$。

由此恢复出相机位姿（以帧 0 为世界原点）：

- $\mathbf{p}_0^W = [0, 0, 0]$，$\mathbf{R}_0^W = \mathbf{I}$
- $\mathbf{p}_1^W = s \cdot [1, 0, 0] = [0.1, 0, 0]$，$\mathbf{R}_1^W = \mathbf{I}$

### 3.2 视觉-惯性对齐

**陀螺仪零偏标定：**

视觉得到的相对旋转 $\mathbf{R}_0^1 = \mathbf{I}$，IMU 预积分得到的相对旋转 $\boldsymbol{\gamma}_0^1 = \mathbf{I}$（因为角速度为 0）。两者一致，故：

$$
\mathbf{b}_g = \mathbf{0}
$$

**预积分计算（帧 0 → 帧 1）：**

由于无旋转，$\mathbf{R}_0^\tau = \mathbf{I}$，加速度恒定：

$$
\boldsymbol{\alpha}_0^1 = \iint_0^{0.1} [0, 0, 9.81]^T \, dt^2 = [0, 0, 0.04905]^T \, \text{m}
$$

$$
\boldsymbol{\beta}_0^1 = \int_0^{0.1} [0, 0, 9.81]^T \, dt = [0, 0, 0.981]^T \, \text{m/s}
$$

$$
\boldsymbol{\gamma}_0^1 = \mathbf{I}
$$

**速度估计：**

将预积分与视觉位姿代入位置更新公式，求解 $v_0$：

$$
\mathbf{p}_1^W = \mathbf{p}_0^W + \mathbf{v}_0^W \Delta t - \frac{1}{2} \mathbf{g}^W \Delta t^2 + \mathbf{R}_0^W \boldsymbol{\alpha}_0^1
$$

$$
[0.1, 0, 0] = [0, 0, 0] + \mathbf{v}_0^W \times 0.1 - [0, 0, 0.04905] + [0, 0, 0.04905]
$$

$$
0.1 \cdot \mathbf{v}_0^W = [0.1, 0, 0] \Rightarrow \mathbf{v}_0^W = [1.0, 0, 0]^T \, \text{m/s}
$$

验证通过。同理 $\mathbf{v}_1^W = [1.0, 0, 0]^T$。

**初始化结果总结：**

| 状态 | 帧 0 | 帧 1 | 帧 2 |
|------|------|------|------|
| $\mathbf{p}^W$ | $[0, 0, 0]$ | $[0.1, 0, 0]$ | $[0.2, 0, 0]$ |
| $\mathbf{v}^W$ | $[1, 0, 0]$ | $[1, 0, 0]$ | $[1, 0, 0]$ |
| $\mathbf{R}_B^W$ | $\mathbf{I}$ | $\mathbf{I}$ | $\mathbf{I}$ |
| $\mathbf{b}_a$ | $\mathbf{0}$ | $\mathbf{0}$ | $\mathbf{0}$ |
| $\mathbf{b}_g$ | $\mathbf{0}$ | $\mathbf{0}$ | $\mathbf{0}$ |

---

## 4. 滑动窗口优化

### 4.1 状态向量

滑动窗口包含 3 帧，每帧 IMU 状态 15 维，加 1 个逆深度，共 46 维。为手算可行，本节做一个合理简化：**假设帧 0 和帧 2 的状态已固定为真实值，只优化帧 1 的位置 $p_{1x}$ 和速度 $v_{1x}$。**

这样优化变量降为 2 维：

$$
\delta\mathcal{X} = [\delta p_{1x}, \delta v_{1x}]^T
$$

（在实际 VINS 中，Ceres 会同时优化所有状态，但原理完全相同。）

**故意引入误差的初值：**

$$
p_{1x}^{(0)} = 0.11 \, \text{m} \quad (\text{真实值} \, 0.10, \text{误差} +1\,\text{cm})
$$

$$
v_{1x}^{(0)} = 1.05 \, \text{m/s} \quad (\text{真实值} \, 1.00, \text{误差} +5\,\text{cm/s})
$$

### 4.2 残差计算

#### 4.2.1 IMU 残差（帧 0 → 帧 1）

**位置残差：**

$$
r_{px} = p_{1x} - p_{0x} - v_{0x} \Delta t + \frac{1}{2} g_x \Delta t^2 - \alpha_x
$$

$$
= 0.11 - 0 - 1.0 \times 0.1 + 0 - 0 = \mathbf{0.01} \, \text{m}
$$

**速度残差：**

$$
r_{vx} = v_{1x} - v_{0x} + g_x \Delta t - \beta_x
$$

$$
= 1.05 - 1.0 + 0 - 0 = \mathbf{0.05} \, \text{m/s}
$$

#### 4.2.2 视觉残差（帧 1）

将路标点变换到帧 1 的 camera 系：

$$
\mathbf{P}^{C_1} = \mathbf{P}^W - \mathbf{p}_1^W = [1.0 - 0.11, 0, 5.0] = [0.89, 0, 5.0]
$$

投影到像素平面：

$$
u = 500 \times \frac{0.89}{5.0} + 320 = 409.0
$$

观测值为 $410.0$，故残差：

$$
r_u = 410.0 - 409.0 = \mathbf{1.0} \, \text{pixel}
$$

### 4.3 雅可比矩阵

#### IMU 残差对状态的雅可比

- $\frac{\partial r_{px}}{\partial p_{1x}} = 1$，$\frac{\partial r_{px}}{\partial v_{1x}} = 0$
- $\frac{\partial r_{vx}}{\partial p_{1x}} = 0$，$\frac{\partial r_{vx}}{\partial v_{1x}} = 1$

#### 视觉残差对状态的雅可比

视觉残差 $r_u = u^{obs} - \left(f_x \frac{P_x^{C_1}}{P_z^{C_1}} + c_x\right)$，其中 $P_x^{C_1} = 1.0 - p_{1x}$，$P_z^{C_1} = 5.0$。

$$
\frac{\partial r_u}{\partial p_{1x}} = -f_x \cdot \frac{-1}{P_z} = \frac{500}{5.0} = \mathbf{100}
$$

（$v_{1x}$ 不直接影响视觉残差，故为 0。）

#### 组装雅可比

$$
\mathbf{J} = \begin{bmatrix}
\frac{\partial r_{px}}{\partial p_{1x}} & \frac{\partial r_{px}}{\partial v_{1x}} \\
\frac{\partial r_{vx}}{\partial p_{1x}} & \frac{\partial r_{vx}}{\partial v_{1x}} \\
\frac{\partial r_u}{\partial p_{1x}} & \frac{\partial r_u}{\partial v_{1x}}
\end{bmatrix}
= \begin{bmatrix}
1 & 0 \\
0 & 1 \\
100 & 0
\end{bmatrix}
$$

残差向量：

$$
\mathbf{r} = \begin{bmatrix} 0.01 \\ 0.05 \\ 1.0 \end{bmatrix}
$$

### 4.4 LM 一次迭代

为简化，设所有信息矩阵为单位阵（即各残差等权重）。高斯-牛顿正规方程为：

$$
\mathbf{J}^T \mathbf{J} \, \delta\mathcal{X} = -\mathbf{J}^T \mathbf{r}
$$

**计算 $\mathbf{J}^T \mathbf{J}$：**

$$
\mathbf{J}^T \mathbf{J} = \begin{bmatrix} 1 & 0 & 100 \\ 0 & 1 & 0 \end{bmatrix} \begin{bmatrix} 1 & 0 \\ 0 & 1 \\ 100 & 0 \end{bmatrix} = \begin{bmatrix} 1 + 10000 & 0 \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} 10001 & 0 \\ 0 & 1 \end{bmatrix}
$$

**计算 $\mathbf{J}^T \mathbf{r}$：**

$$
\mathbf{J}^T \mathbf{r} = \begin{bmatrix} 1 & 0 & 100 \\ 0 & 1 & 0 \end{bmatrix} \begin{bmatrix} 0.01 \\ 0.05 \\ 1.0 \end{bmatrix} = \begin{bmatrix} 0.01 + 100 \\ 0.05 \end{bmatrix} = \begin{bmatrix} 100.01 \\ 0.05 \end{bmatrix}
$$

**求解增量：**

$$
\begin{bmatrix} 10001 & 0 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} \delta p_{1x} \\ \delta v_{1x} \end{bmatrix} = -\begin{bmatrix} 100.01 \\ 0.05 \end{bmatrix}
$$

$$
\delta p_{1x} = -\frac{100.01}{10001} \approx \mathbf{-0.0100} \, \text{m}
$$

$$
\delta v_{1x} = -\frac{0.05}{1} = \mathbf{-0.05} \, \text{m/s}
$$

**状态更新：**

$$
p_{1x}^{(1)} = p_{1x}^{(0)} + \delta p_{1x} = 0.11 - 0.01 = \mathbf{0.10} \, \text{m}
$$

$$
v_{1x}^{(1)} = v_{1x}^{(0)} + \delta v_{1x} = 1.05 - 0.05 = \mathbf{1.00} \, \text{m/s}
$$

### 4.5 结果验证

更新后的残差：

- $r_{px} = 0.10 - 0.10 = \mathbf{0}$
- $r_{vx} = 1.00 - 1.00 = \mathbf{0}$
- 帧 1 投影：$u = 500 \times \frac{1.0-0.10}{5.0} + 320 = 410$，$r_u = 410 - 410 = \mathbf{0}$

**所有残差收敛到 0，状态精确恢复到真实值。**

---

## 5. 边缘化（简述）

当帧 3 到来时，滑动窗口已满（假设最大容量为 3 帧），需要边缘化最旧帧（帧 0）。

**边缘化前，系统的信息矩阵包含了帧 0 的约束：**
- 帧 0 与帧 1 之间的 IMU 预积分约束；
- 帧 0 对特征点的视觉观测约束；
- 帧 0 的先验信息（来自更早的边缘化）。

**舒尔补操作：** 将帧 0 的状态从信息矩阵中消去，保留其约束对剩余状态（帧 1、2、3 和路标点）的影响，转化为**先验残差** $\mathbf{r}_p$。

**数值上，** 假设边缘化后帧 0 的位置信息被压缩为一个关于帧 1 位置的线性约束：

$$
\mathbf{r}_p \approx \mathbf{p}_1^W - [0.1, 0, 0] \approx \mathbf{0}
$$

（实际是先验信息矩阵和先验向量的形式，见主文档第 6 节。）

---

## 6. 完整流程总结

| 步骤 | 输入 | 输出 | 本文数值结果 |
|------|------|------|-------------|
| **传感器数据** | 真实运动 | IMU + 图像 | â=[0,0,9.81], 像素=(420,320)/(410,320)/(400,320) |
| **纯视觉 SFM** | 特征匹配 | 相对位姿 + 尺度 | $s=0.1$，$\mathbf{R}=\mathbf{I}$，$\mathbf{t}=[0.1,0,0]$ |
| **预积分** | IMU 原始数据 | $\alpha, \beta, \gamma$ | $\alpha=[0,0,0.04905]$，$\beta=[0,0,0.981]$，$\gamma=\mathbf{I}$ |
| **视觉-惯性对齐** | SFM 结果 + 预积分 | 速度、重力、零偏 | $\mathbf{v}=[1,0,0]$，$\mathbf{g}=[0,0,9.81]$，$\mathbf{b}=\mathbf{0}$ |
| **滑动窗口优化** | 状态初值 + 残差 | 最优状态 | 一次 LM 迭代修正 $p_{1x}: 0.11 \to 0.10$，$v_{1x}: 1.05 \to 1.00$ |
| **边缘化** | 最旧帧约束 | 先验信息 | 帧 0 的信息被压缩为关于剩余状态的先验 |

---

## 7. 关键观察

1. **视觉约束的"尺度放大效应"：** 在本例中，$\frac{\partial r_u}{\partial p_{1x}} = 100$（像素/米），这意味着 1 cm 的位置误差会导致 1 像素的重投影误差。高焦距相机对这个系数更敏感，因此视觉位姿估计精度更高。

2. **IMU 与视觉的互补性：**
   - IMU 残差对 $p_{1x}$ 的雅可比是 1，对 $v_{1x}$ 的雅可比也是 1；
   - 视觉残差对 $p_{1x}$ 的雅可比是 100，但对 $v_{1x}$ 的雅可比是 0。
   - 这意味着：**视觉主要约束位置，IMU 主要约束速度和位置的变化率**，两者互补。

3. **正规方程中的信息矩阵：** $\mathbf{J}^T\mathbf{J}$ 的对角元 $10001$ 和 $1$ 分别反映了位置 $p_{1x}$ 和速度 $v_{1x}$ 的**信息量**。位置的信息量远大于速度（因为视觉提供了强约束），因此位置增量 $\delta p_{1x}$ 被精确修正到 $-0.01$ m。

4. **预积分的价值：** 本例中预积分量 $\alpha$ 和 $\beta$ 完全由 IMU 测量决定，与状态初值无关。无论初值怎么猜，预积分不变，只需代入残差公式即可。
