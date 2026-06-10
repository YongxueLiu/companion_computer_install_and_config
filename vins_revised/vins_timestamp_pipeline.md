# VINS-Fusion → PX4 时间戳 Pipeline 详解

> 本文从 VINS-Fusion 源码一路追踪到 PX4 EKF2，说清楚 `timestamp_sample` 到底是什么时刻、EKF2 如何处理它、以及 `EKF2_EV_DELAY` 到底在补什么。

---

## 1. 核心结论（先放前面）

| 问题 | 答案 |
|---|---|
| VINS 的 `timestamp_sample` 是什么时刻？ | **图像采集时刻**（RealSense 硬件时间戳） |
| VINS 输出的位姿对应什么时刻？ | **同一图像采集时刻** |
| EKF2 融合用的是哪个时间？ | `timestamp_sample`（`ev_data.time_us = ev_odom.timestamp_sample`） |
| VINS 前后端处理延迟要补吗？ | **不需要**，EKF2 会自动回溯到 `timestamp_sample` 融合 |
| `EKF2_EV_DELAY` 到底补什么？ | 仅补偿硬件时间戳偏差 + uXRCE time_offset 误差（约 0~20 ms） |
| 推荐设置 | **`EKF2_EV_DELAY = 0`** 或最多 **10~20 ms** |

---

## 2. 完整溯源：从 RealSense 到 EKF2

### 2.1 第一步：RealSense 驱动发布图像

RealSense ROS 驱动（`realsense2_camera_node`）从 D435i 的红外相机读取帧，并在 `sensor_msgs/Image` 的 `header.stamp` 中填入**相机硬件时间戳**（基于 USB 帧的 SOF/EOF 时间或固件时间）。

```
Image message:
  header:
    stamp: 图像采集时刻（相机硬件时钟）
    frame_id: "camera_infra1_optical_frame"
```

> 注意：librealsense SDK 内部做了 IMU 和图像的硬件同步，`header.stamp` 与对应 IMU 样本的时间是同步的。

### 2.2 第二步：VINS 接收图像

`rosNodeTest.cpp` 中的图像回调：

```cpp
// vins/src/rosNodeTest.cpp
void sync_process()
{
    while (true)
    {
        // 从缓冲区取出同步后的双目图像
        time = img0_buf.front()->header.stamp.sec
             + img0_buf.front()->header.stamp.nanosec * (1e-9);
        header = img0_buf.front()->header;  // ← 直接保留原始 header
        image0 = getImageFromMsg(img0_buf.front());
        img0_buf.pop();
        ...
        estimator.inputImage(time, image0, image1);  // ← time = header.stamp
    }
}
```

`time` 直接取自图像消息的 `header.stamp`，**没有任何修改或偏移**。

### 2.3 第三步：VINS 前端特征提取

`estimator.inputImage()` 把 `time` 原封不动传给 `featureTracker`：

```cpp
// vins/src/estimator/estimator.cpp
void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    featureFrame = featureTracker.trackImage(t, _img, _img1);
    // ...
    featureBuf.push(make_pair(t, featureFrame));  // ← t 就是图像采集时间
}
```

`featureTracker.trackImage()` 只负责提取特征点，`t` 被保存在 `featureFrame` 中，用于后续与 IMU 数据对齐。

### 2.4 第四步：VINS 后端优化与发布

`processMeasurements()` 从 `featureBuf` 中取出特征帧，执行 `processImage()` 进行滑动窗口优化：

```cpp
// vins/src/estimator/estimator.cpp
void Estimator::processMeasurements()
{
    while (true)
    {
        // 取出图像和特征
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> feature;
        feature = featureBuf.front();
        featureBuf.pop();

        // IMU 预积分（从 prevTime 到 curTime）
        getIMUInterval(prevTime, curTime, accVector, gyrVector);
        for(size_t i = 0; i < accVector.size(); i++)
        {
            processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
        }

        // 视觉-惯性联合优化
        processImage(feature.second, feature.first);  // feature.first = t

        // 构造 header，直接复用图像时间戳
        std_msgs::msg::Header header;
        header.frame_id = WORLD_FRAME_ID;
        header.stamp.sec = (int)feature.first;
        header.stamp.nanosec = (uint)((feature.first - sec_ts) * 1e9);

        // 发布 odometry
        pubOdometry(*this, header);   // ← header.stamp = 图像采集时刻
    }
}
```

关键点：
- `feature.first` 就是 `inputImage()` 传入的 `t`，即图像采集时间
- `processImage()` 优化后，`Ps[WINDOW_SIZE]` 和 `Rs[WINDOW_SIZE]` 是**该图像采集时刻**的位姿
- `header.stamp` 直接设为 `feature.first`，**不添加任何处理延迟**

### 2.5 第五步：VINS 发布 Odometry

```cpp
// vins/src/utility/visualization.cpp
void pubOdometry(const Estimator &estimator, const std_msgs::msg::Header &header)
{
    nav_msgs::msg::Odometry odometry;
    odometry.header = header;  // ← 直接使用传入的 header（图像采集时间）
    odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
    odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
    odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
    odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
    // ...
    pub_odometry->publish(odometry);
}
```

### 2.6 第六步：Bridge 转发到 PX4

Bridge 中：

```python
# vins_px4_bridge/bridge_node.py
vins_time_us = int(msg.header.stamp.sec * 1_000_000
                   + msg.header.stamp.nanosec // 1000)
vo.timestamp_sample = vins_time_us   # EKF2 实际用于融合的时间
vo.timestamp = ros_time_us           # ROS2 当前时间（仅日志记录）
```

`timestamp_sample` 就是 VINS `header.stamp` 的原始值，即**图像采集时刻**。

### 2.7 第七步：PX4 EKF2 接收与融合

```cpp
// PX4-Autopilot/src/modules/ekf2/EKF2.cpp
void EKF2::UpdateEvOdometrySample(ekf2_timestamps_s &ekf2_timestamps)
{
    vehicle_odometry_s ev_odom;
    if (_ev_odom_sub.update(&ev_odom)) {
        extVisionSample ev_data{};
        // ... 提取 position, velocity, quaternion ...

        // 关键：EKF2 用 timestamp_sample 作为融合时间
        ev_data.time_us = ev_odom.timestamp_sample;
        ev_data.reset_counter = ev_odom.reset_counter;
        ev_data.quality = ev_odom.quality;

        if (new_ev_odom) {
            _ekf.setExtVisionData(ev_data);  // ← 放入 EKF2 缓冲区
        }
    }
}
```

```cpp
// PX4-Autopilot/src/modules/ekf2/EKF/estimator_interface.cpp
void EstimatorInterface::setExtVisionData(const extVisionSample &evdata)
{
    // 计算数据在缓冲区中的存放位置
    const int64_t time_us = evdata.time_us
                - static_cast<int64_t>(_params.ekf2_ev_delay * 1000)
                - static_cast<int64_t>(_dt_ekf_avg * 5e5f);

    // 放入 TimestampedRingBuffer
    extVisionSample ev_sample_new{evdata};
    ev_sample_new.time_us = time_us;
    _ext_vision_buffer->push(ev_sample_new);
}
```

---

## 3. EKF2 延迟模型拆解

EKF2 收到 EV 数据后，执行以下时间修正：

```
buffer_time = timestamp_sample
            - EKF2_EV_DELAY × 1000
            - dt_ekf_avg / 2 × 1e6
```

### 3.1 `_dt_ekf_avg * 5e5f`：EKF 预测周期中点对齐

这一项对所有传感器都相同（mag、baro、airspeed、range、EV）：

```cpp
// Mag
const int64_t time_us = mag_sample.time_us
            - static_cast<int64_t>(_params.ekf2_mag_delay * 1000)
            - static_cast<int64_t>(_dt_ekf_avg * 5e5f);

// Baro
const int64_t time_us = baro_sample.time_us
            - static_cast<int64_t>(_params.ekf2_baro_delay * 1000)
            - static_cast<int64_t>(_dt_ekf_avg * 5e5f);

// EV
const int64_t time_us = evdata.time_us
            - static_cast<int64_t>(_params.ekf2_ev_delay * 1000)
            - static_cast<int64_t>(_dt_ekf_avg * 5e5f);
```

**含义**：把传感器数据的时间戳对齐到 EKF 预测步的中点，减少离散时间步带来的量化误差。

- `_dt_ekf_avg` 是 EKF 平均预测周期（约 10 ms，对应 100 Hz IMU）
- `_dt_ekf_avg * 5e5f` = `_dt_ekf_avg / 2 * 1e6` ≈ **5 ms**
- 这一项**始终存在**，与 `EKF2_EV_DELAY` 无关

### 3.2 `EKF2_EV_DELAY`：传感器特有的额外延迟

这是各传感器特有的参数，用于补偿从**物理采样**到**EKF2 处理**之间的延迟。

| 传感器 | 默认延迟 | 物理原因 |
|---|---|---|
| Barometer | 0 ms | 气压计几乎没有处理延迟 |
| Magnetometer | 0 ms | 磁力计几乎没有处理延迟 |
| Range Finder | 5 ms | 激光测距非常快 |
| Optical Flow | 5 ms | 光流计算很快 |
| Airspeed | 100 ms | 空速管有管道延迟 |
| **External Vision** | **175 ms** | **覆盖外部视觉系统的端到端延迟** |

**关键问题**：175 ms 的默认值是怎么来的？

PX4 的设计假设是：外部视觉系统（如 MAVROS + T265）发送的 `timestamp_sample` **不够准确**，通常是**发送时间**或**处理完成时间**，而不是**图像采集时间**。

如果 `timestamp_sample` 是发送时间（比图像采集时间晚了 100+ ms），EKF2 需要把这个时间往回退，才能得到图像采集时刻。

但对于 **VINS-Fusion + RealSense**：
- `timestamp_sample` = 图像采集时刻（硬件时间戳）
- VINS 位姿 = 该图像采集时刻的位姿
- EKF2 收到数据后，会自动回溯到 `timestamp_sample` 融合

**所以 VINS 的前端特征提取 + 后端优化延迟，不需要 `EKF2_EV_DELAY` 来补偿。**

---

## 4. 为什么 VINS 处理延迟无需补偿

### 4.1 EKF2 的融合机制

EKF2 不是"收到数据立即融合"，而是：

1. 所有传感器数据先放入各自的 `TimestampedRingBuffer`
2. IMU 数据驱动预测循环，按时间顺序推进
3. 当预测循环推进到某个传感器的 `time_us` 时，才执行融合/更新
4. 如果传感器数据的时间在 EKF2 当前时间之前，EKF2 会**回溯**（rewind）到那个时间点，融合数据，然后重新预测到当前时间

这意味着：
- 即使 EV 数据在采集后 60 ms 才到达 EKF2
- EKF2 仍然会把数据放在 `timestamp_sample` 时刻
- EKF2 的融合循环会在推进到 `timestamp_sample` 时正确处理它

### 4.2 时序图

```
时间轴 →

T0          T1=T0+30ms     T2=T0+60ms      T3=T0+65ms
|              |               |                |
图像采集    VINS前端完成    VINS后端完成     EKF2收到数据
   |              |               |                |
   |←——30ms——→|←——30ms——→|←——5ms——→|
   |              |               |                |
   └──────────────┴───────────────┴────────────────┘
                    VINS 总处理延迟 ≈ 60 ms

EKF2 缓冲区：
  ...  [T0-5ms]  [T0]  [T0+5ms]  ...  [T3]  ...
         ↑
    EV 数据放在这里
    (timestamp_sample - dt_ekf_avg/2)

EKF2 融合循环：
  推进到 T0 时 → 发现 EV 数据 → 回溯到 T0-5ms → 融合 EV → 重新预测到当前
```

VINS 的处理延迟（60 ms）完全不影响 EKF2 的正确性，因为 EKF2 是按**时间戳**融合，不是按**到达时间**融合。

---

## 5. `EKF2_EV_DELAY` 到底该设多少

### 5.1 理论值：0 ms

如果满足以下条件：
1. `timestamp_sample` 是准确的图像采集时间（VINS+RealSense 满足）
2. RealSense 硬件时间戳与系统时间偏差很小（< 5 ms）
3. uXRCE-DDS `time_offset` 校准准确

那么 `EKF2_EV_DELAY = 0` 是理论最优值。

此时 EKF2 中的 `time_us`：
```
time_us = timestamp_sample - 0 - dt_ekf_avg/2
        ≈ timestamp_sample - 5 ms
```

### 5.2 实际推荐值：0 ~ 20 ms

考虑实际系统的微小误差：

| 误差来源 | 典型大小 |
|---|---|
| RealSense 硬件时间戳偏差 | ~5 ms |
| uXRCE-DDS time_offset 校准误差 | ~5 ms |
| 图像 USB 传输延迟（已含在硬件时间戳中）| ~0-2 ms |
| Jetson 系统时间漂移 | ~1-3 ms |
| 合计 | ~10-15 ms |

**推荐设置**：
- 如果追求最优：`EKF2_EV_DELAY = 0`
- 保守设置：`EKF2_EV_DELAY = 10 ~ 20`
- 如果试飞 innovaton 检查偶尔失败：`EKF2_EV_DELAY = 20 ~ 30`

### 5.3 对比：默认值 175 ms 的问题

175 ms 是 PX4 针对"外部视觉时间戳不准确"的保守设置。对 VINS+RealSense 来说：

```
time_us = timestamp_sample - 175 ms - 5 ms
        = timestamp_sample - 180 ms
```

这意味着 EKF2 把视觉数据放在**图像采集前 180 ms** 的位置。虽然 EKF2 的缓冲区可以处理回溯，但这个值明显偏大，可能导致：
- 缓冲区需要保存更长时间的历史数据
- 视觉数据与 IMU 数据的时间对齐出现不必要的偏差
- 极端情况下，如果处理延迟超过 175 ms，数据会被丢弃

---

## 6. 验证方法

### 6.1 检查 VINS 时间戳是否为图像采集时间

```bash
# 同时查看图像和 odometry 的时间戳
ros2 topic echo /camera/camera/infra1/image_rect_raw --once | grep stamp
ros2 topic echo /vins_estimator/odometry --once | grep stamp
```

两者的时间戳应该非常接近（差异 < 1 ms），说明 VINS 直接复用了图像时间戳。

### 6.2 检查 EKF2 是否使用 timestamp_sample

在 PX4 日志（ULog）中查看：

```python
import pyulog
ulog = pyulog.ULog('logfile.ulg')
ev = ulog.get_dataset('vehicle_visual_odometry')

# timestamp_sample 应该与 VINS 的 header.stamp 一致
# timestamp 是 PX4 收到消息的时间（较新）
```

### 6.3 地面站查看 innovation

在 QGroundControl 的 MAVLink Inspector 中查看：
- `estimator_status` → `vel_innov` / `pos_innov`
- 如果 `EKF2_EV_DELAY` 设置合理，innovation 应该稳定且较小
- 如果设置过大（如 175 ms），可能会出现 innovation 偏大或不稳定

---

## 7. 总结

| 要点 | 结论 |
|---|---|
| VINS `timestamp_sample` 来源 | RealSense 图像硬件时间戳 → 图像采集时刻 |
| VINS 位姿对应时刻 | **同一图像采集时刻**（`Ps[WINDOW_SIZE]` 的时间 = `feature.first`） |
| EKF2 融合机制 | 按 `timestamp_sample` 回溯融合，不是按到达时间 |
| VINS 处理延迟是否需要补偿 | **不需要**，EKF2 自动处理 |
| `_dt_ekf_avg * 5e5f` | 始终存在（约 5 ms），用于 IMU 积分中点对齐 |
| `EKF2_EV_DELAY` 实际作用 | 仅补偿硬件时间戳偏差 + uXRCE time_offset 误差 |
| 推荐值 | **0 ~ 20 ms**（默认 175 ms 对 VINS+RealSense 偏大） |

---

## 8. 参考源码

- VINS 图像回调：`src/VINS-Fusion-ROS2/vins/src/rosNodeTest.cpp`
- VINS 特征跟踪：`src/VINS-Fusion-ROS2/vins/src/estimator/estimator.cpp`（`inputImage`）
- VINS 优化发布：`src/VINS-Fusion-ROS2/vins/src/estimator/estimator.cpp`（`processMeasurements`）
- VINS 可视化：`src/VINS-Fusion-ROS2/vins/src/utility/visualization.cpp`（`pubOdometry`）
- EKF2 EV 接收：`PX4-Autopilot/src/modules/ekf2/EKF2.cpp`（`UpdateEvOdometrySample`）
- EKF2 缓冲区：`PX4-Autopilot/src/modules/ekf2/EKF/estimator_interface.cpp`（`setExtVisionData`）
- EKF2 参数定义：`PX4-Autopilot/src/modules/ekf2/params_external_vision.yaml`
- EKF2 默认值：`PX4-Autopilot/src/modules/ekf2/EKF/common.h`
