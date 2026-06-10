# VINS-PX4 Bridge 字段映射与填充详解

本文档详细说明 `vins_px4_bridge` 中每一个输入字段如何映射到输出字段，包括数据来源、转换公式、单位、以及常见错误。

---

## 1. 输入消息：`nav_msgs/Odometry`

VINS-Fusion 发布的话题：`/vins_estimator/odometry`

### 1.1 有效字段（VINS 实际填充）

| 字段路径 | 数据类型 | 说明 | 示例值 |
|---|---|---|---|
| `header.stamp` | `builtin_interfaces/Time` | 时间戳 | sec=1780900573, nanosec=792221307 |
| `header.frame_id` | string | 固定为 `"world"` | `"world"` |
| `child_frame_id` | string | 固定为 `"body"` | `"body"` |
| `pose.pose.position.x` | float64 | World 坐标系 X | 0.622 |
| `pose.pose.position.y` | float64 | World 坐标系 Y | 0.341 |
| `pose.pose.position.z` | float64 | World 坐标系 Z | -0.106 |
| `pose.pose.orientation.x` | float64 | Body→World 四元数 X | -0.699 |
| `pose.pose.orientation.y` | float64 | Body→World 四元数 Y | -0.014 |
| `pose.pose.orientation.z` | float64 | Body→World 四元数 Z | 0.019 |
| `pose.pose.orientation.w` | float64 | Body→World 四元数 W | 0.715 |
| `twist.twist.linear.x` | float64 | World 坐标系线速度 X | -0.003 |
| `twist.twist.linear.y` | float64 | World 坐标系线速度 Y | 0.001 |
| `twist.twist.linear.z` | float64 | World 坐标系线速度 Z | -0.000 |

### 1.2 无效字段（VINS 未填充）

| 字段路径 | 值 | 说明 |
|---|---|---|
| `pose.covariance[36]` | 全 0 | 协方差矩阵未填充 |
| `twist.twist.angular.x/y/z` | 0.0 | 角速度未填充 |
| `twist.covariance[36]` | 全 0 | 速度协方差未填充 |

---

## 2. 输出消息：`px4_msgs/VehicleOdometry`

Bridge 发布的话题：`/fmu/in/vehicle_visual_odometry`

### 2.1 字段总览

```
uint64 timestamp              # 必需
uint64 timestamp_sample       # 必需
uint8 pose_frame              # 固定为 1 (NED)
uint8 velocity_frame          # 固定为 1 (NED)
float32[3] position           # NED 位置
float32[4] q                  # FRD→NED 四元数 [w,x,y,z]
float32[3] velocity           # NED 线速度
float32[3] angular_velocity   # NaN (VINS 不提供)
float32[3] position_variance
float32[3] orientation_variance
float32[3] velocity_variance
uint8 reset_counter           # 跳变计数
```

---

## 3. 逐字段详解

### 3.1 `timestamp` 与 `timestamp_sample`

**数据来源**：
- `timestamp_sample`：VINS 原始采样时间 `msg.header.stamp`
- `timestamp`：bridge 接收到消息时的 ROS2 当前时间

**转换公式**：
```python
vins_time_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000
ros_time_us = self.get_clock().now().nanoseconds // 1000

vo.timestamp_sample = vins_time_us   # EKF2 实际使用
vo.timestamp = ros_time_us           # 仅用于日志/调试
```

**单位**：微秒（µs），Unix 纪元时间

**重要说明**：
- `timestamp_sample` 是 EKF2 做传感器融合时真正用的时间戳
- `timestamp` 只是记录消息到达 PX4 侧的时间，用于日志分析和延迟统计
- 两者的差值 ≈ DDS 传输延迟 + bridge 处理延迟（通常几毫秒到几十毫秒）
- 不要加任何偏移量，PX4 uXRCE-DDS client 内部会自动 `msg.timestamp - session->time_offset`
- 手动修正会导致双重转换

**示例**：
```
输入: sec=1780900573, nanosec=792221307
输出: timestamp=1780900573792221
```

---

### 3.2 `pose_frame`

**固定值**：`VehicleOdometry.POSE_FRAME_NED = 1`

**说明**：告诉 PX4 position 和 q 都是在 NED 坐标系下。

---

### 3.3 `position[3]`

**数据来源**：`msg.pose.pose.position`

**坐标转换**：VINS World → NED

VINS World 约定（实测）：
- X = right（右）
- Y = forward（前）
- Z = up（上）

NED 约定：
- X = North
- Y = East
- Z = Down

**转换公式**：
```python
p_enu = [msg.pose.pose.position.x,
         msg.pose.pose.position.y,
         msg.pose.pose.position.z]

vo.position = [
    float(p_enu[1]),   # NED X = ENU Y = forward → North
    float(p_enu[0]),   # NED Y = ENU X = right → East
    float(-p_enu[2]),  # NED Z = -ENU Z = -up → Down
]
```

**映射关系表**：

| VINS World | NED | 物理含义 |
|---|---|---|
| X (right) | Y (East) | 右移 → East 增 |
| Y (forward) | X (North) | 前移 → North 增 |
| Z (up) | -Z (Down) | 上移 → Down 减 |

**示例**：
```
输入: position=[0.622, 0.341, -0.106]
输出: position=[0.341, 0.622, 0.106]
```

---

### 3.4 `q[4]`（四元数）

**数据来源**：`msg.pose.pose.orientation`

**最终目标**：`q_FRD→NED`，Hamilton 约定 `[w, x, y, z]`

**完整转换链**：
```
q_FRD→NED = q_ENU→NED ⊗ q_body→ENU ⊗ q_FRD→body
```

#### 3.4.1 第一步：规范化输入四元数

```python
q_body_to_enu = np.array([w, x, y, z])
q_body_to_enu /= np.linalg.norm(q_body_to_enu)
```

#### 3.4.2 第二步：选择 `q_FRD→body`

根据 `body_frame` 参数：

| body_frame | Body 约定 | q_FRD→body [w,x,y,z] | 适用场景 |
|---|---|---|---|
| `OPENCV` | X-right, Y-down, Z-forward | `[-0.5, 0.5, 0.5, 0.5]` | VINS + RealSense D435i |
| `FLU` | X-forward, Y-left, Z-up | `[0, 1, 0, 0]` | FAST-LIO + Livox |
| `FRD` | X-forward, Y-right, Z-down | `[1, 0, 0, 0]` | 已经是 FRD |

#### 3.4.3 第三步：`q_ENU→NED`

固定值：`[0, √2/2, √2/2, 0]`

对应旋转矩阵：
```
[0  1  0]
[1  0  0]
[0  0 -1]
```

#### 3.4.4 第四步：四元数乘法

```python
def quat_multiply(q1, q2):
    """Hamilton 乘法 q1 ⊗ q2，先应用 q2，再应用 q1"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

# 完整链
q_virtual = quat_multiply(q_enu_to_ned,
                          quat_multiply(q_body_to_enu, q_frd_to_body))
q_virtual /= np.linalg.norm(q_virtual)
```

#### 3.4.5 第五步（可选）：Yaw 对齐

当 `yaw_alignment_mode == 'px4_mag'`：

```python
# 从 q_virtual 提取 yaw
yaw_virtual = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

# 与 PX4 magnetometer yaw 比较
if yaw_offset is None:
    yaw_offset = px4_yaw - yaw_virtual  # 首次锁定

# 应用偏移
delta_q = [cos(yaw_offset/2), 0, 0, sin(yaw_offset/2)]
q_out = quat_multiply(delta_q, q_virtual)
```

**示例**（body right=East, forward=North，使用 OPENCV）：
```
输入 q_body_to_enu:  [-0.707, 0.707, 0, 0]
q_FRD→NED 输出:     [1, 0, 0, 0]  (identity)
含义: FRD = NED，机头朝北
```

---

### 3.5 `velocity_frame`

**固定值**：`VehicleOdometry.VELOCITY_FRAME_NED = 1`

---

### 3.6 `velocity[3]`

**数据来源**：`msg.twist.twist.linear`

**坐标转换**：与 position 完全相同（VINS 的 twist 也是在 World 坐标系下）

```python
v_enu = [msg.twist.twist.linear.x,
         msg.twist.twist.linear.y,
         msg.twist.twist.linear.z]

vo.velocity = [
    float(v_enu[1]),   # NED X = ENU Y
    float(v_enu[0]),   # NED Y = ENU X
    float(-v_enu[2]),  # NED Z = -ENU Z
]
```

**注意**：VINS 的 twist 在 `nav_msgs/Odometry` 中定义为 child_frame（body）下的速度。但 VINS 实际填充的是 World 坐标系下的速度（与 position 同系），所以转换方式与 position 一致。

---

### 3.7 `angular_velocity[3]`

**固定值**：`[NaN, NaN, NaN]`

**原因**：VINS-Fusion 不输出角速度。

**PX4 行为**：当 angular_velocity 为 NaN 时，EKF2 会忽略该字段，只融合位置和姿态。

---

### 3.8 `position_variance[3]`

**数据来源**：`msg.pose.covariance`

```python
cov = msg.pose.covariance
pos_var = [float(cov[0]), float(cov[7]), float(cov[14])]
```

** fallback**：如果 VINS 未填充（全 0），使用 `default_position_variance`：
```python
vo.position_variance = [0.01, 0.01, 0.01]  # 默认值
```

**注意**：covariance 矩阵在 NED 下的顺序与 ENU 不同，但因为 VINS 输出全 0，所以这里直接透传前三个对角元素。

---

### 3.9 `orientation_variance[3]`

**数据来源**：`msg.pose.covariance[21]`, `[28]`, `[35]`

```python
ori_var = [float(cov[21]), float(cov[28]), float(cov[35])]
```

** fallback**：使用 `default_orientation_variance`

---

### 3.10 `velocity_variance[3]`

**数据来源**：无（VINS 不填充 twist covariance）

**固定值**：`default_velocity_variance`（默认 `[0.01, 0.01, 0.01]`）

---

### 3.11 `reset_counter`

**行为**：当检测到位置跳变时自动递增

```python
if jump > position_jump_threshold:
    reset_counter += 1
    yaw_offset = None  # 强制重新锁定 yaw
```

**用途**：通知 PX4 EKF2 外部定位源发生了 reset，需要重新收敛。

---

## 4. 完整数据流示例

### 4.1 输入（VINS Odometry）

```yaml
header:
  stamp:
    sec: 1780900573
    nanosec: 792221307
  frame_id: "world"
child_frame_id: "body"
pose:
  pose:
    position:
      x: 0.6220584271813309
      y: 0.3407979416439078
      z: -0.10580004884643243
    orientation:
      x: -0.6988333258318888
      y: -0.013644887992229881
      z: 0.019398660208685435
      w: 0.714891244680286
twist:
  twist:
    linear:
      x: -0.0031056791006096015
      y: 0.0012222675855154761
      z: -0.0003334886177494799
```

### 4.2 输出（PX4 VehicleOdometry）

```yaml
timestamp: 1780900573792221
timestamp_sample: 1780900573792221
pose_frame: 1
position:
  - 0.3407979416439078      # NED X = ENU Y = forward
  - 0.6220584271813309      # NED Y = ENU X = right
  - 0.10580004884643243     # NED Z = -ENU Z = -(-0.106) = 0.106
q:
  - -0.9995473623275757     # w
  - -0.004998687654733658   # x
  - -0.029600119218230247   # y
  - 0.001967926509678364    # z
velocity_frame: 1
velocity:
  - 0.0012222675855154761   # NED X
  - -0.0031056791006096015  # NED Y
  - 0.0003334886177494799   # NED Z
angular_velocity:
  - .nan
  - .nan
  - .nan
position_variance: [0.01, 0.01, 0.01]
orientation_variance: [0.01, 0.01, 0.01]
velocity_variance: [0.01, 0.01, 0.01]
reset_counter: 0
```

---

## 5. 坐标转换速查表

### 5.1 位置/速度：World → NED

| World (VINS) | NED (PX4) | 公式 |
|---|---|---|
| X (right) | Y (East) | `NED[1] = World[0]` |
| Y (forward) | X (North) | `NED[0] = World[1]` |
| Z (up) | -Z (Down) | `NED[2] = -World[2]` |

### 5.2 姿态：四元数转换链

```
q_out = q_enu_to_ned ⊗ q_body_to_enu ⊗ q_frd_to_body
```

| 参数 | OPENCV | FLU | FRD |
|---|---|---|---|
| `q_frd_to_body` | `[-0.5, 0.5, 0.5, 0.5]` | `[0, 1, 0, 0]` | `[1, 0, 0, 0]` |
| `q_enu_to_ned` | `[0, √2/2, √2/2, 0]`（固定） | 同上 | 同上 |

### 5.3 从四元数提取 Yaw

```python
yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
```

- 0° = 朝北（NED X）
- 90° = 朝东（NED Y）
- 180° = 朝南
- -90° = 朝西

---

## 6. 常见错误与修正

### 错误 1：使用 `body_frame:=FLU` 运行 VINS+RealSense

**现象**：地面站 yaw 显示偏差约 90°

**原因**：VINS body 是 optical frame，不是 FLU

**修正**：
```bash
ros2 run vins_px4_bridge bridge_node --ros-args -p body_frame:=OPENCV
```

### 错误 2：手动减去时间偏移

**现象**：PX4 报时间戳异常，拒绝融合

**原因**：uXRCE-DDS client 已经内部转换了

**修正**：bridge 直接发送原始 Unix µs，不要加 offset

### 错误 3：初始化时 body 方向未对准

**现象**：yaw 始终有一个固定偏差

**原因**：VINS world 的 X/Y 轴由初始化时的 body right/forward 决定，不是真北

**修正**：
- 方案 A：初始化时让相机 right=East, forward=North
- 方案 B：使用 `yaw_alignment_mode:=px4_mag` 自动对齐
- 方案 C：使用 `yaw_alignment_mode:=manual` 手动补偿

---

## 7. 参考

- `src/vins_px4_bridge/vins_px4_bridge/bridge_node.py`
- `tutorial/bridge_coordinate_transform.md`
- PX4 `VehicleOdometry` 消息定义：`px4_msgs/msg/VehicleOdometry`
- PX4 EKF2 外部视觉融合文档：https://docs.px4.io/main/en/ros/external_position_estimation.html
