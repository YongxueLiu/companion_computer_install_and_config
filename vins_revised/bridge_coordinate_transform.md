# VINS-Fusion / FAST-LIO → PX4 坐标转换逻辑详解

> 本文目标：从源码和实测两个维度，说清楚 VINS-Fusion 与 FAST-LIO 各自的 body/world 坐标系约定，以及把它们接入 PX4 `vehicle_visual_odometry` 时每一步转换的数学依据。避免再用"VINS 输出就是 ENU"这种不严谨的表述。

---

## 1. 先下结论：两个 SLAM 的 world 都不是真正的 ENU

| 项目 | VINS-Fusion (VIO) | FAST-LIO |
|---|---|---|
| **World Z** | 垂直向上（重力对齐） | 垂直向上（重力对齐） |
| **World X/Y** | 水平面内正交，方向由初始化时 body 的 right/forward 决定 | 水平面内正交，方向由初始化时 body 的 forward/left 决定 |
| **是否等于 ENU** | **仅当**初始化时 body right 朝东、forward 朝北 | **仅当**初始化时 body forward 朝东、left 朝北 |
| **真北参考** | 无（需外部 magnetometer/GNSS 对齐） | 无（需外部 magnetometer/GNSS 对齐） |

更准确的说法是：**两者都输出一个"重力对齐的初始水平坐标系"**，而不是真正的 ENU。只有在初始化姿态满足特定条件时，它才偶然与 ENU 重合。

---

## 2. 坐标系约定总览

### 2.1 PX4 使用的坐标系

| 坐标系 | 轴定义 | 用途 |
|---|---|---|
| **FRD** (body) | X=Forward, Y=Right, Z=Down | 飞行器的机体坐标系 |
| **NED** (world) | X=North, Y=East, Z=Down | 飞行器的导航坐标系 |

PX4 `VehicleOdometry` 消息要求：
- `position` 在 NED 下
- `q` 是 `FRD → NED` 的旋转（Hamilton 四元数，`[w,x,y,z]`）
- 时间戳为 Unix 微秒（uXRCE-DDS client 会内部转换成 PX4 boot 时间）

### 2.2 VINS-Fusion 实际使用的坐标系

**Body = `camera_imu_optical_frame`**（RealSense D435i 默认 IMU 帧）

| 轴 | 方向 | 备注 |
|---|---|---|
| X | right（右）| 图像水平向右 |
| Y | down（下）| 图像垂直向下 |
| Z | forward（前）| 镜头光轴朝前 |

静止时的典型加速度（水平放置）：
```yaml
linear_acceleration:
  x: -0.33
  y: -9.66
  z:  1.23
```
frame_id: `camera_imu_optical_frame`

重力主要在 **Y 轴负方向**，符合 optical frame "Y-down" 的约定。

**World**（VIO 初始化后）

| 轴 | 方向 | 来源 |
|---|---|---|
| X | 初始化时 body 的 right 在水平面投影 | `g2R()` + yaw 归零 |
| Y | 初始化时 body 的 forward 在水平面投影 | 右手定则 |
| Z | up（重力反方向） | `g2R()` |

**实测验证**（运行中手持相机移动）：
- 往**右**移动 → `/vins_estimator/odometry.pose.pose.position.x` **增大**
- 往**前**移动 → `position.y` **增大**
- 往**上**移动 → `position.z` **增大**

这与 World X=right、Y=forward、Z=up 完全吻合。

### 2.3 FAST-LIO 实际使用的坐标系

**Body = 传感器制造商约定的 IMU 坐标系**

以 Livox Avia 为例，厂家文档定义的是 **FLU**：

| 轴 | 方向 |
|---|---|
| X | forward（前）|
| Y | left（左）|
| Z | up（上）|

静止时典型加速度（水平放置）：
```yaml
linear_acceleration:
  x:  0.0
  y:  0.0
  z: -9.8
```

重力在 **Z 轴负方向**，符合 FLU "Z-up" 的约定。

**World**（初始化后）

FAST-LIO 的 IKFoM 状态里 `R_wb(0) = I`，也就是说世界坐标系在初始化瞬间**直接等于 body 坐标系**。重力方向单独存放在 `state.grav` 中（`[0, 0, G]`），不通过旋转矩阵来对齐。

| 轴 | 方向 |
|---|---|
| X | 初始化时 body forward |
| Y | 初始化时 body left |
| Z | 初始化时 body up |

因此 FAST-LIO world 同样只是"Z-up 的初始水平系"，不是自动 ENU。

---

## 3. 源码验证

### 3.1 VINS-Fusion：`g2R()` 与 yaw 归零

```cpp
// vins/src/utility/utility.cpp
Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();      // body 下的重力方向
    Eigen::Vector3d ng2{0, 0, 1.0};            // world Z = up
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    return R0;
}
```

对 D435i，`g ≈ [0, -G, 0]`，`FromTwoVectors([0,-1,0], [0,0,1])` 等价于绕 X 轴转 -90°，得到：

```
R0 = [1  0  0]
     [0  0  1]
     [0 -1  0]
```

映射关系：
- body X (right)    → world X (right)
- body Y (down)     → world -Z
- body Z (forward)  → world Y (forward)

然后 `yaw = R2ypr(R0 * Rs[0]).x()` 取出的是 **body X 轴** 在水平面的方位角，归零后 world X 就与 body right 对齐。最终 world 约定：

> **X = right，Y = forward，Z = up**

完整初始化代码在 `estimator.cpp`：

```cpp
Matrix3d R0 = Utility::g2R(g);
double yaw = Utility::R2ypr(R0 * Rs[0]).x();
R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
g = R0 * g;
Matrix3d rot_diff = R0;
for (int i = 0; i <= frame_count; i++) {
    Ps[i] = rot_diff * Ps[i];
    Rs[i] = rot_diff * Rs[i];
    Vs[i] = rot_diff * Vs[i];
}
```

### 3.2 FAST-LIO：保持 `rot = I`

```cpp
// FAST_LIO_ROS2/src/IMU_Processing.hpp
state_ikfom init_state = kf_state.get_x();
init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
init_state.bg  = mean_gyr;
// ... 没有修改 init_state.rot，默认保持 I
kf_state.change_x(init_state);
```

`SO3` 的默认值是单位矩阵，因此：

> **World = 初始化时的 body 坐标系**

过程模型里重力直接以状态量形式参与，而不是通过 world 旋转来对齐：

```cpp
// FAST_LIO_ROS2/src/use-ikfom.hpp
res(i+12) = a_inertial[i] + s.grav[i];   // i = 0,1,2
```

这里 `s.grav` 是世界坐标系下的重力向量 `[0, 0, G]`，正好对应 Z-up。

---

## 4. Bridge 转换逻辑

两个 SLAM 的 world 都是"水平 Z-up"，所以**位置/速度的 ENU→NED 映射公式完全相同**。差异只在于 body frame 不同，因此姿态转换链中的 `R_FRD→body` 这一项要分别处理。

### 4.1 位置 / 速度转换

通用的 ENU → NED 映射：

```
NED_X =  ENU_Y   (North)
NED_Y =  ENU_X   (East)
NED_Z = -ENU_Z   (Down)
```

代码中就是数组索引交换加取反：

```python
vo.position = [
    float(p_enu[1]),   # North
    float(p_enu[0]),   # East
    float(-p_enu[2]),  # Down
]
```

这个公式对 VINS 和 FAST-LIO 都成立，因为它们的 world 都是 X/Y 水平、Z 向上。

### 4.2 姿态转换链

PX4 需要 `q_FRD→NED`。完整的转换链是：

```
q_FRD→NED = q_ENU→NED  ⊗  q_body→ENU  ⊗  q_FRD→body
```

其中：
- `q_ENU→NED`：world 水平系 → NED，对两者都相同
- `q_body→ENU`：SLAM 输出的 body→world 四元数
- `q_FRD→body`：取决于 SLAM 的 body frame 约定

#### `q_ENU→NED` 的值

ENU → NED 的旋转矩阵：

```
[0  1  0]
[1  0  0]
[0  0 -1]
```

这是 180° 旋转，四元数（Hamilton, `[w,x,y,z]`）为：

```python
q_enu_to_ned = [0.0, sqrt(2)/2, sqrt(2)/2, 0.0]
```

#### VINS 的 `q_FRD→body`

VINS body = optical frame (X-right, Y-down, Z-forward)。需要把 FRD 的三个轴映射到 VINS body：

| FRD | VINS body |
|---|---|
| Forward (X) | Z |
| Right (Y)   | X |
| Down (Z)    | Y |

对应的旋转矩阵：

```
[0  1  0]
[0  0  1]
[1  0  0]
```

四元数为：

```python
q_frd_to_opencv = [-0.5, 0.5, 0.5, 0.5]   # 或 [0.5, -0.5, -0.5, -0.5]，等价
```

#### FAST-LIO 的 `q_FRD→body`

FAST-LIO body = FLU (X-forward, Y-left, Z-up)。映射关系：

| FRD | FLU |
|---|---|
| Forward (X) | X |
| Right (Y)   | -Y |
| Down (Z)    | -Z |

对应的旋转矩阵：

```
[1  0  0]
[0 -1  0]
[0  0 -1]
```

这是绕 X 轴 180°，四元数为：

```python
q_frd_to_flu = [0.0, 1.0, 0.0, 0.0]
```

### 4.3 四元数乘法约定

两个 bridge 都使用 Hamilton 约定，函数实现如下：

```python
def quat_multiply(q1, q2):
    """q1 ⊗ q2，先应用 q2，再应用 q1。"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])
```

几何意义：
```python
q_virtual = quat_multiply(q_enu_to_ned,
                          quat_multiply(q_body_to_enu, q_frd_to_body))
```

等价的旋转矩阵形式：

```
R_FRD→NED = R_ENU→NED · R_body→ENU · R_FRD→body
```

### 4.4 代码对照

| 项目 | FAST-LIO bridge | VINS bridge（当前代码） |
|---|---|---|
| `q_enu_to_ned` | `[0, √2/2, √2/2, 0]` | `[0, √2/2, √2/2, 0]` |
| `q_frd_to_body` | `[0, 1, 0, 0]`（FLU） | `[0, 1, 0, 0]`（FLU） |
| `body_frame` 默认 | `FLU` | `FLU` |

**关键区别**：FAST-LIO 的 body 确实是 FLU，所以 `q_frd_to_flu` 是对的；但 VINS 的实际 body 是 optical frame（X-right, Y-down, Z-forward），应该用 `q_frd_to_opencv = [-0.5, 0.5, 0.5, 0.5]`。当前 `vins_px4_bridge` 里写死 `body_frame='FLU'`，与 VINS 真实 body 不一致。

> 如果不改代码，只想快速验证姿态是否正确：可以临时把 VINS 的相机 rigid mount 成"FLU"（X-forward, Y-left, Z-up），但这对 D435i 几乎不可能，因为它的图像坐标系和 IMU 坐标系都是 optical frame。

---

## 5. 实测验证方法

### 5.1 验证 World 方向（VINS）

1. 启动 RealSense + VINS
2. `ros2 topic echo /vins_estimator/odometry --once`
3. 手持相机分别向**右 / 前 / 上**移动，观察 position：
   - 右移 → x 增大
   - 前移 → y 增大
   - 上移 → z 增大

这与源码推导的 World X=right、Y=forward、Z=up 一致。

### 5.2 验证 Body 方向（VINS）——从 odometry 四元数反推

直接拿 `/vins_estimator/odometry` 的四元数来算 body 各轴在 world 中的投影：

```bash
ros2 topic echo /vins_estimator/odometry --once
```

实测 quaternion `[w, x, y, z]`（镜头朝前正常放置）：

```
w =  0.7149,  x = -0.6988,  y = -0.0136,  z =  0.0194
```

转成旋转矩阵 `R = q_to_matrix(q)`，取三列：

| Body 轴 | 在 World 中的方向 | 近似等于 |
|---|---|---|
| X (col 0) | `[0.999, 0.047, -0.008]` | **World X** (right) |
| Y (col 1) | `[-0.009, 0.023, -1.000]` | **World -Z** (down) |
| Z (col 2) | `[-0.047, 0.999, 0.023]` | **World Y** (forward) |

姿态角（VINS `R2ypr` 约定）：
- Yaw = 2.68°
- Pitch = 0.44°
- Roll = **-88.69°**（≈ -90°，body X 几乎水平朝右，body Z 几乎水平朝前）

**结论**：body X≈world right，body Y≈world down，body Z≈world forward。这就是 optical frame，不是 FLU。

也可以交叉验证 IMU：
```bash
ros2 topic echo /camera/camera/imu --once
```
- `header.frame_id == camera_imu_optical_frame`
- 水平静止时 `linear_acceleration.y ≈ -9.8`

body Y 向下承受重力，进一步确认 optical frame。

### 5.3 验证 FAST-LIO Body 方向

```bash
ros2 topic echo /livox/imu --once
```

Livox Avia 水平静止时：
- `linear_acceleration.z ≈ -9.8`

说明 body Z 向上，是 FLU。

---

## 6. Yaw 对齐：从虚拟 NED 到真北

即使姿态转换完全正确，VINS/FAST-LIO 的 world 也只是"初始水平系"，X/Y 轴不一定对准东/北。PX4 如果需要与 magnetometer/GNSS 融合，必须进行 yaw 对齐。

Bridge 中支持三种模式：

| 模式 | 说明 |
|---|---|
| `none` | 直接把虚拟 NED 发给 PX4，不保证真北 |
| `px4_mag` | 订阅 `/fmu/out/vehicle_attitude`，取 PX4 magnetometer yaw，计算 `δ_yaw = yaw_px4 - yaw_virtual`，然后锁定这个偏移量 |
| `manual` | 手动设置一个固定的 `manual_yaw_offset_rad` |

`px4_mag` 模式的实现：

```python
if self.yaw_offset is None:
    self.yaw_offset = self.px4_yaw - yaw_virtual

half = self.yaw_offset / 2.0
delta_q = np.array([math.cos(half), 0.0, 0.0, math.sin(half)])
q_out = quat_multiply(delta_q, q_virtual)
```

注意：这个 offset 只在**第一次**收到有效 PX4 attitude 时锁定。如果 VINS 发生位置跳变（reset），offset 会被清除并重新锁定。

---

## 7. 时间戳：不要自作聪明加 offset

PX4 uXRCE-DDS client 在反序列化 `vehicle_visual_odometry` 时会自动做时间转换：

```cpp
// PX4-Autopilot src/modules/uxrce_dds_client/utilities/conversions.cpp
topic.timestamp = msg.timestamp - session->time_offset;
```

因此 bridge 里直接发 Unix 微秒即可：

```python
vo.timestamp = int(msg.header.stamp.sec * 1_000_000
                   + msg.header.stamp.nanosec // 1000)
```

**千万不要**在 bridge 里再减一个 offset，否则会造成双重转换，导致时间戳错误。

---

## 8. 常见误区

### 误区 1："VINS world 就是 ENU"

错。VINS world 只是重力对齐的局部水平系，X/Y 轴方向由初始化时的 body right/forward 决定。ENU 是一种特殊情况（body right 朝东 + forward 朝北）。

### 误区 2："VINS body 是 FLU"

错。对于 RealSense D435i 默认配置，body 是 `camera_imu_optical_frame`：X-right, Y-down, Z-forward。只有 Livox/FAST-LIO 这种厂商文档明确写 FLU 的才是 FLU。

### 误区 3："位置和速度的转换公式不一样"

错。只要 SLAM 的 world 是 X/Y 水平、Z 向上，位置/速度的转换公式就是统一的 `(x,y,z)→(y,x,-z)`。差异只在姿态的 `q_frd_to_body`。

### 误区 4："姿态图中 roll≈-180° 说明相机倒置"

不一定。可能是可视化脚本里的四元数→Euler 角实现与 VINS 的 `R2ypr` 约定不一致。要以源码和实际安装方式为准。例如 `realtime_odom_plot.py` 若使用 `transformations.euler_from_quaternion` 的默认轴序，可能与 VINS 的 Z-Y-X 不同，导致显示出来的 roll/pitch 看起来"倒置"。

---

## 9. 总结

1. **VINS-Fusion** 的 world 是 `g2R()` 生成的重力对齐水平系：
   - `World X = 初始化时 body 的 right`
   - `World Y = 初始化时 body 的 forward`
   - `World Z = up`

2. **FAST-LIO** 的 world 等于初始化时的 body 坐标系（通常是 FLU）：
   - `World X = forward`
   - `World Y = left`
   - `World Z = up`

3. **位置/速度转换** 对两者通用：
   ```
   NED = (ENU_Y, ENU_X, -ENU_Z)
   ```

4. **姿态转换** 需要区分 body frame：
   - FAST-LIO：`q_frd_to_flu = [0, 1, 0, 0]`
   - VINS (D435i)：`q_frd_to_opencv = [-0.5, 0.5, 0.5, 0.5]`

5. **真北对齐** 必须额外做，两种 SLAM 本身都不提供地球参考方向。

6. **时间戳** 直接发 Unix µs，不要加 offset。

---

## 10. 参考文件

- VINS-Fusion 初始化：`src/VINS-Fusion-ROS2/vins/src/estimator/estimator.cpp`（`visualInitialAlign`）
- VINS-Fusion `g2R`：`src/VINS-Fusion-ROS2/vins/src/utility/utility.cpp`
- VINS-Fusion 可视化：`src/VINS-Fusion-ROS2/vins/src/utility/visualization.cpp`（`pubOdometry`）
- FAST-LIO IMU 初始化：`ws_livox/src/FAST_LIO_ROS2/src/IMU_Processing.hpp`
- FAST-LIO 过程模型：`ws_livox/src/FAST_LIO_ROS2/src/use-ikfom.hpp`
- VINS bridge：`src/vins_px4_bridge/vins_px4_bridge/bridge_node.py`
- FAST-LIO bridge：`ws_livox/src/fastlio_px4_bridge/fastlio_px4_bridge/bridge_node.py`
- PX4 uXRCE 时间转换：`PX4-Autopilot/src/modules/uxrce_dds_client/utilities/conversions.cpp`
