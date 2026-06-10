# PX4–ROS2 时间同步机制与视觉定位延时分析

> 本文档从 VINS-Fusion、PX4-Autopilot、`fastlio_px4_bridge` 三个代码库的源码角度，解释时间戳在整条视觉定位管道中的真实含义、转换逻辑，并给出延时测量方法与参数推荐。
>
> 适用环境：ROS2 Humble + PX4 via uXRCE-DDS + Jetson Orin Nano

---

## 1. 时间戳全链路（源码级 walkthrough）

```
[RealSense D435i] --(USB)--> [librealsense2 RSUSB] --(ROS2)--> [realsense2_camera]
                                                                    |
                                                                    v
[VINS-Fusion] <--(image+imu)-- [rosNodeTest.cpp] --(publish)--> /vins_estimator/odometry
                                                                    |
                                                                    v
[vins_px4_bridge] <--(subscribe)-- [bridge_node.py] --(publish)--> /fmu/in/vehicle_visual_odometry
                                                                    |
                                                                    v
[PX4 uXRCE-DDS client] <--(DDS)-- [deserialize] --(uORB)--> vehicle_visual_odometry
                                                                    |
                                                                    v
[PX4 EKF2] <--(融合)-- [ekf2] --(状态估计)--> vehicle_local_position
```

### 1.1 源头：RealSense 图像时间戳

RealSense D435i 的 RSUSB 后端在收到每帧图像时，会调用 `global_timestamp_reader` 获取系统时间（Unix epoch）。这个时间戳通过 `sensor_msgs::Image::header.stamp` 发布到 ROS2。

> **关键属性**：时间戳代表 **USB 帧数据到达 librealsense2 用户态驱动的时刻**，不是曝光开始时刻，也不是 VINS solver 完成时刻。

### 1.2 VINS-Fusion：`header.stamp` 的传递

**源码位置**：`vins/src/rosNodeTest.cpp`

```cpp
// 图像回调中，header 直接从 ROS2 Image 消息复制
std_msgs::msg::Header header;
header = img0_msg->header;

// 该 header 被传入 estimator，最终通过 pubOdometry() 发布到 /vins_estimator/odometry
pubOdometry(estimator, header);
```

**源码位置**：`vins/src/utility/visualization.cpp` 第 137–189 行

```cpp
void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    nav_msgs::msg::Odometry odometry;
    odometry.header = header;  // <-- 直接复用图像 header，不做任何修改
    // ... 填充 pose, twist ...
    pub_odometry->publish(odometry);
}
```

**结论**：`/vins_estimator/odometry` 的 `header.stamp` **就是图像采集/到达时刻**（Unix epoch），不是 VINS 优化完成时刻。VINS solver 在 ~30–40 ms 后才完成计算，但时间戳保持不变。

### 1.3 Bridge：Unix epoch → 微秒整数

**源码位置**：`vins_px4_bridge/bridge_node.py`

```python
vo.timestamp = int(msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000)
vo.timestamp_sample = vo.timestamp
```

**关键设计**：**不要应用任何 timesync_offset**。

`fastlio_px4_bridge` 的源码注释明确说明：
> "Do NOT apply timesync_offset here. PX4 uXRCE-DDS client automatically converts timestamps during deserialization. Manual offset would cause double-conversion."

Bridge 只做一个纯单位转换（秒+纳秒 → 微秒整数），时间基准仍然是 Unix epoch。

### 1.4 PX4 uXRCE-DDS Client：自动时区转换

**源码位置**：`PX4-Autopilot/src/modules/uxrce_dds_client/uxrce_dds_client.cpp`

```cpp
static void on_time(uxrSession *session, int64_t current_time, int64_t client_transmit_timestamp,
                    int64_t agent_receive_timestamp, int64_t originate_timestamp, void *args)
{
    Timesync *timesync = static_cast<Timesync *>(args);
    timesync->update(current_time / 1000, agent_receive_timestamp, originate_timestamp);
    session->time_offset = -timesync->offset() * 1000; // us -> ns
}
```

- `session->time_offset` 存储在 **纳秒**，是一个很大的**负数**（≈ -1.78×10¹⁵ ns），因为 PX4 boot 时间远小于 Unix epoch。
- 该 offset 由 uXRCE 内置的 timesync 协议自动维护（类似 NTP，见下文）。

**源码位置**：`build/px4_sitl_default/uORB/ucdr/vehicle_odometry.h`（生成代码）

```cpp
if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
```

**转换逻辑**：
1. 收到 `VehicleOdometry` 消息，`timestamp` 字段是 Unix epoch 微秒。
2. `time_offset_us = session->time_offset / 1000`（ns → µs）。
3. `timestamp_px4 = timestamp_unix - time_offset_us`。
4. 由于 `time_offset_us` 是一个很大的正数（≈ +1.78×10¹⁵ µs，即 ~50+ 年），减法结果就是把 Unix 时间映射到 **PX4 boot-relative 微秒**。
5. `math::min(..., hrt_absolute_time())` 防止未来时间戳（如果时钟不同步）。

**结论**：PX4 uXRCE client **自动完成** Unix epoch → PX4 boot time 的转换。Bridge 端**绝对不要**手动减去任何 offset。

### 1.5 PX4 EKF2：时间戳的融合语义

**源码位置**：`PX4-Autopilot/src/modules/ekf2/EKF/ev_control.cpp`

EKF2 收到 `vehicle_visual_odometry` 后，用 `timestamp` 计算 **innovation delay**：

```
delay = hrt_absolute_time() - measurement.timestamp
```

如果 `delay` 超过 `EKF2_EV_DELAY` 参数设置的阈值，测量会被**延迟融合**（放到延迟补偿缓冲区中），而不是立即丢弃。

> 这意味着时间戳的准确性直接决定了 EKF2 的融合时序。如果时间戳被错误地提前或延后，EKF2 会把测量值映射到错误的状态历史时刻，导致估计发散。

---

## 2. 时间戳的真实含义总结

| 阶段 | 时间戳代表的时刻 | 数值基准 | 关键源码 |
|------|----------------|---------|---------|
| RealSense 图像发布 | USB 帧到达用户态驱动的时刻 | Unix epoch | `global_timestamp_reader` |
| VINS `/odometry` | 同上（直接透传图像 header） | Unix epoch | `visualization.cpp: odometry.header = header` |
| Bridge `/fmu/in/...` | 同上（仅做单位转换） | Unix epoch µs | `bridge_node.py: sec*1e6 + nsec//1000` |
| PX4 uORB `vehicle_visual_odometry` | 同上（自动转换为 boot-relative） | PX4 boot µs | `ucdr_deserialize_vehicle_odometry` |
| EKF2 融合 | 该时刻对应的状态历史被更新 | — | `ev_control.cpp` |

**核心认知**：整条链路上的时间戳始终指向 **图像采集/到达时刻**，不是 VINS 计算完成时刻。EKF2 通过 `EKF2_EV_DELAY` 参数补偿从图像采集到 EKF2 收到消息之间的物理延迟。

---

## 3. PX4 Timesync 算法（源码级）

**源码位置**：`PX4-Autopilot/src/lib/timesync/Timesync.hpp`, `Timesync.cpp`

### 3.1 算法概述

PX4 的 timesync 是一个**在线双指数平滑滤波器**（alpha-beta filter），带异常值剔除：

```cpp
// 伪代码
offset_estimate = alpha * (t_remote - t_local - one_way_delay) + (1 - alpha) * offset_estimate
one_way_delay   = beta  * (RTT / 2) + (1 - beta) * one_way_delay
```

- `alpha`, `beta` 是平滑增益，从 `0.05` 逐渐收敛到 `0.003`。
- 需要约 **500 个样本**（即 ~500 次往返）才能完全收敛。

### 3.2 异常值剔除

```cpp
if (RTT > 10 ms) drop_sample();
if (deviation_from_estimate > 100 ms) drop_sample();
if (consecutive_bad_deviations > 10) reset_filter();
```

- RTT 超过 10 ms 的样本被直接丢弃。
- 与当前估计偏差超过 100 ms 的样本被丢弃。
- 连续 10 个异常样本触发滤波器完全重置。

### 3.3 实际表现

在 localhost（uXRCE Agent 和 PX4 在同一台 Jetson 上）环境下：
- RTT 通常在 **0.1–1 ms** 量级。
- 收敛速度：**< 5 秒**（500 样本 @ 100 Hz）。
- 收敛后的 offset 精度：**< 1 ms**。

---

## 4. 端到端延时分析

### 4.1 各组件延时

| 组件 | 典型延时 | 来源/说明 |
|------|---------|----------|
| RealSense RSUSB 采集+传输 | 5–10 ms | USB 3.2 帧传输 + 用户态驱动处理 |
| `rosNodeTest.cpp` 图像同步 | ≤ 15 ms | `sync_tolerance = 0.015 s`，左右目时间对齐 |
| 特征跟踪前端 | 5–10 ms | `max_cnt=150`, `min_dist=30`, 640×480 |
| VINS 非线性优化 | 30–40 ms | `max_solver_time=0.04 s`（日志实测 32–36 ms）|
| Bridge 处理 | < 1 ms | Python numpy 四元数运算 |
| uXRCE-DDS 传输 (localhost) | 1–5 ms | BEST_EFFORT QoS，共享内存/loopback |
| **总端到端** | **~55–70 ms** | 图像采集 → PX4 EKF2 收到 |

### 4.2 为什么时间戳不反映真实延迟

时间戳是图像到达时刻（t₀），但 EKF2 收到消息的时刻是 t₀ + 70 ms。EKF2 通过以下逻辑处理：

```
measurement_delay = hrt_absolute_time() - measurement.timestamp
                    = 70 ms  (approx)
```

如果 `EKF2_EV_DELAY = 60 ms`，EKF2 会把该测量值放到 **延迟补偿缓冲区** 中，等状态历史回溯到 t₀ + 60 ms 时再融合。剩余的 10 ms 误差由 EKF 的过程噪声模型吸收。

### 4.3 延时测量方法

**方法 1：ROS2 侧粗测**

在 bridge 节点中打印：
```python
now_us = self.get_clock().now().nanoseconds // 1000
latency_ms = (now_us - vo.timestamp) / 1000.0
self.get_logger().info(f'Latency: {latency_ms:.1f} ms')
```

**方法 2：PX4 侧精确测**

在 PX4 中订阅 `vehicle_visual_odometry` uORB 话题（通过 MAVLink shell 或 custom module）：
```cpp
px4_us = hrt_absolute_time();
delay_us = px4_us - vehicle_visual_odometry.timestamp;
```

**方法 3：EKF2 日志分析**

飞行后查看 `.ulg` 日志中的 `ekf2_innovations` 话题，`ev_hgt`, `ev_pos` 等 innovation 的符号和大小可以间接反映时间同步是否准确。如果 innovation 出现系统性偏置，通常是 `EKF2_EV_DELAY` 设置不当。

---

## 5. 推荐配置

### 5.1 PX4 参数

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `EKF2_EV_DELAY` | **60 ms** | 视觉定位端到端延迟补偿。默认 0 ms，必须手动增加。 |
| `EKF2_HGT_REF` | **3** (Vision) | 高度参考源设为视觉。 |
| `EKF2_EV_CTRL` | **15** (0b1111) | bit0=位置, bit1=速度, bit2=高度, bit3= yaw。全部启用。 |
| `EKF2_EV_NOISE_MD` | 根据环境 | 室内结构化环境用 `0` (constant)，室外/纹理差用 `1` (dynamic)。 |
| `EKF2_EV_GATE` | **5.0** | 视觉测量 innovation gate，默认 3.0，可适当放宽到 5.0。 |

### 5.2 VINS 配置优化（降低延迟）

| 参数 | 当前值 | 优化方向 |
|------|--------|---------|
| `max_solver_time` | 0.04 s | 已接近极限，Jetson Orin Nano 实测 32–36 ms。不建议再减小。 |
| `max_num_iterations` | 8 | 可尝试减小到 6–7，牺牲少量精度换 5–10 ms 延迟。 |
| `max_cnt` | 150 | 可减小到 120，减少前端跟踪计算量。 |
| `image_width/height` | 640/480 | 已是最小实用分辨率，不建议再降。 |

### 5.3 Bridge 配置

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `default_position_variance` | `[0.01, 0.01, 0.01]` | 0.1 m 标准差。VINS 精度好时可用 `[0.001, 0.001, 0.001]`。 |
| `default_orientation_variance` | `[0.01, 0.01, 0.01]` | ~0.1 rad 标准差。 |
| `default_velocity_variance` | `[0.01, 0.01, 0.01]` | 速度信噪比通常较好。 |
| `position_jump_threshold` | **0.5** | 检测到 SLAM 跳变时 increment reset_counter。 |
| `yaw_alignment_mode` | `none` 或 `px4_mag` | 如果不需要真北对齐，`none` 最简单；需要 GPS/磁北融合时用 `px4_mag`。 |

---

## 6. 常见误区

### 误区 1：在 bridge 中手动减去 timesync_offset

**错误做法**：
```python
vo.timestamp = int(...) - timesync_offset_us  # ❌ 绝对不要
```

**后果**：PX4 uXRCE client 会再次减去 offset，导致时间戳变成 boot time 的 **两倍差值**，EKF2 会把测量值映射到 50 年前的状态历史，直接拒绝融合。

### 误区 2：用 solver 完成时刻作为时间戳

**错误做法**：在 bridge 中用 `rospy.Time.now()` 或 `rclpy.clock.now()` 覆盖 VINS header stamp。

**后果**：时间戳被延后了 30–40 ms，EKF2 认为测量值来自 "未来"（因为 `delay = now - timestamp` 变小），导致融合时序错乱。

### 误区 3：`EKF2_EV_DELAY = 0`

**后果**：EKF2 假设视觉测量是 "即时" 的，但实际上有 60+ ms 的物理延迟。EKF2 会把测量值融合到 **错误的状态历史时刻**，表现为 innovation 系统性偏置、估计漂移甚至发散。

### 误区 4：忽略 `reset_counter`

**后果**：VINS 在光照变化、快速运动或纹理丢失时可能发生位置跳变。如果 `reset_counter` 不递增，PX4 EKF2 会把跳变解释为真实运动，导致姿态/速度估计突变。

---

## 7. 快速检查清单

在首次飞行前确认：

- [ ] VINS 运行稳定，`/vins_estimator/odometry` 有数据输出
- [ ] Bridge 运行，`ros2 topic echo /fmu/in/vehicle_visual_odometry` 有数据
- [ ] PX4 参数 `EKF2_EV_DELAY` 已设为 **60 ms**
- [ ] PX4 参数 `EKF2_HGT_REF = 3`, `EKF2_EV_CTRL = 15`
- [ ] `vehicle_visual_odometry.timestamp` 是 Unix epoch µs（约 2.0×10⁶ 秒级，即 2026 年）
- [ ] uXRCE Agent 已连接，QGroundControl 中 `vehicle_local_position` 的 xy/z 与 VINS 输出一致
- [ ] 手持无人机缓慢平移+旋转，QGC 的 HUD 姿态响应平滑、无跳变
