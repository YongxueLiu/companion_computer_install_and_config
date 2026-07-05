# FAST-LIVO2 双线程数据同步机制

> 本文基于 `FAST-LIVO2-ROS2` 源码，说明在 ROS2 Humble 环境下，回调线程与主线程如何协作完成 LiDAR / IMU / Image 数据的接收、缓冲、同步与消费。
>
> 相关源码文件：
> - `src/LIVMapper.cpp`
> - `include/LIVMapper.h`
> - `include/common_lib.h`

## 1. 为什么需要双线程

FAST-LIVO2 的主线程要做非常重的计算：
- IMU 预积分与点云去畸变
- LIO（LiDAR-Inertial Odometry）状态估计与 voxel map 更新
- VIO（Visual-Inertial Odometry）特征跟踪与视觉地图点管理
- 各种点云、图像、里程计发布

如果这些计算全部跑在 ROS2 executor 的回调线程里，DDS 层接收新消息时会被阻塞，导致：
- IMU 数据丢失或滞后
- LiDAR 帧丢失
- RealSense 图像流因回调不及时而 USB 断开

因此代码将 **ROS2 spin（回调）** 与 **算法主循环（消费）** 拆成两个线程：

```
+-------------------+      deque buffers      +------------------------+
|  ROS2 spin thread |  -------------------->  |   Mapping main thread  |
|  (DDS callbacks)  |   mtx_buffer protected  |  (LIO/VIO/state est.)  |
+-------------------+                         +------------------------+
        |                                                |
        v                                                v
   接收 / 预处理数据                                sync_packages()
   push 到缓冲队列                                  processImu()
                                                    stateEstimationAndMapping()
```

## 2. 线程启动：run()

线程分裂发生在 `LIVMapper::run()`：

```cpp
void LIVMapper::run(rclcpp::Node::SharedPtr &node) 
{
  // 1. 创建单线程 executor，并挂载当前节点
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(this->node);

  // 2. 在独立线程中运行 executor.spin()，专门处理 DDS 回调
  std::thread spin_thread([&executor]() { executor.spin(); });

  // 3. 主线程进入自己的循环，消费数据
  rclcpp::Rate rate(5000);
  while (rclcpp::ok()) 
  {
    if (!sync_packages(LidarMeasures)) 
    {
      rate.sleep();
      continue;
    }
    handleFirstFrame();
    processImu();
    stateEstimationAndMapping();
  }

  // 4. 退出时安全 join 回调线程
  executor.cancel();
  if (spin_thread.joinable()) spin_thread.join();
}
```

关键设计：
- `executor.spin()` 只负责触发 subscription / timer 回调，不做任何算法计算。
- 主线程以 5000 Hz 的频率轮询 `sync_packages()`，数据未就绪时 `rate.sleep()` 让出 CPU。

## 3. 回调线程：数据接收与缓冲

### 3.1 共享缓冲区定义

所有跨线程共享的缓冲区都定义在 `LIVMapper.h` 中，使用 `std::deque`：

```cpp
std::mutex mtx_buffer, mtx_buffer_imu_prop;

deque<PointCloudXYZI::Ptr>       lid_raw_data_buffer;   // LiDAR 点云
deque<double>                    lid_header_time_buffer; // LiDAR 帧头时间
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer; // IMU 数据
deque<cv::Mat>                   img_buffer;            // 图像帧
deque<double>                    img_time_buffer;       // 图像时间戳
deque<sensor_msgs::msg::Imu>     prop_imu_buffer;       // IMU 预测专用
```

### 3.2 回调函数的工作流程

以 Livox CustomMsg 点云回调为例：

```cpp
void LIVMapper::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in)
{
  if (!lidar_en) return;

  double cur_head_time = stamp2Sec(msg_in->header.stamp);

  // ---- 1. 预处理在加锁之外 ----
  livox_ros_driver2::msg::CustomMsg::SharedPtr msg(new livox_ros_driver2::msg::CustomMsg(*msg_in));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);   // 解析 / 降采样等，可能较耗时

  if (!ptr || ptr->empty()) { /* error */ return; }

  // ---- 2. 加锁，写入共享队列 ----
  mtx_buffer.lock();
  if (abs(last_timestamp_imu.load() - cur_head_time) > 1.0 && !imu_buffer.empty())
  {
    // 发现 IMU 与 LiDAR 时间差过大，打印提示供自同步使用
  }
  if (cur_head_time < last_timestamp_lidar.load())
  {
    RCLCPP_ERROR(..., "lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar.store(cur_head_time);
  mtx_buffer.unlock();
}
```

IMU 回调也类似：

```cpp
void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
  if (!imu_en) return;
  if (last_timestamp_lidar.load() < 0.0) return;  // 等待第一帧 LiDAR 到来

  // 时间戳修正
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  msg->header.stamp = sec2Stamp(stamp2Sec(msg->header.stamp) - imu_time_offset);
  double timestamp = stamp2Sec(msg->header.stamp);

  // loop back / jump 检查 ...

  mtx_buffer.lock();
  last_timestamp_imu.store(timestamp);
  imu_buffer.push_back(msg);
  mtx_buffer.unlock();

  // IMU 预测使用独立的锁，避免阻塞主数据流
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (!p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
}
```

图像回调 `img_cbk()` 同样：转换 `cv::Mat` 后加锁写入 `img_buffer` / `img_time_buffer`。

### 3.3 预处理在锁外的设计意义

注意 `p_pre->process()` 是在 `mtx_buffer.lock()` 之前调用的。这是因为：
- 点云预处理可能耗时（解析 Livox CustomMsg、降采样、格式转换）。
- 如果在锁内执行，会阻塞同一 `mtx_buffer` 保护下的 IMU / Image 回调。
- 预处理只需要原始消息，不需要访问共享缓冲区，因此可以安全地放在锁外。

只有“指针/时间戳入队”这一最小操作才需要加锁。

## 4. 主线程：数据同步与消费

### 4.1 主循环

```cpp
while (rclcpp::ok()) 
{
  if (!sync_packages(LidarMeasures)) 
  {
    rate.sleep();
    continue;
  }
  handleFirstFrame();
  processImu();
  stateEstimationAndMapping();
}
```

- `sync_packages()`：尝试从缓冲队列中构造一个完整的时间同步测量组 `LidarMeasureGroup`。
- 成功返回 `true` 后，主线程才会进入 `processImu()` 和 `stateEstimationAndMapping()`。
- 失败则 sleep，等待回调线程继续收数据。

### 4.2 sync_packages() 的核心逻辑

`sync_packages(LidarMeasureGroup &meas)` 是数据同步的核心。它按照当前 `slam_mode_` 分支：

```cpp
bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en)   return false;
  if (imu_buffer.empty() && imu_en)   return false;

  switch (slam_mode_)
  {
    case ONLY_LIO: ...
    case LIVO:     ...   // 本文重点
    case ONLY_LO:  ...
  }
}
```

#### 4.2.1 LIVO 模式：LIO 与 VIO 交替

LIVO 模式下，`meas.lio_vio_flg` 在 `WAIT -> LIO -> VIO -> LIO -> VIO -> ...` 之间切换：

- **LIO 阶段**：以图像捕获时间为基准，收集截至该时刻的 IMU 和 LiDAR 点，构建一帧 LIO 测量。
- **VIO 阶段**：消费一张图像，触发视觉前端处理。

```cpp
case LIVO:
{
  switch (meas.lio_vio_flg)
  {
    case WAIT:
    case VIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;

      // 检查数据是否足够新
      double lid_newest_time = lid_header_time_buffer.back()
                               + lid_raw_data_buffer.back()->points.back().curvature / 1000.0;
      double imu_newest_time = stamp2Sec(imu_buffer.back()->header.stamp);

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
        return false;   // 数据还没齐，继续等

      struct MeasureGroup m;
      m.lio_time = img_capture_time;

      // 取截至 img_capture_time 的 IMU
      mtx_buffer.lock();
      while (!imu_buffer.empty())
      {
        if (stamp2Sec(imu_buffer.front()->header.stamp) > m.lio_time) break;
        if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.last_lio_update_time)
          m.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
      }
      mtx_buffer.unlock();

      // 按时间切分 LiDAR 点云到 pcl_proc_cur / pcl_proc_next
      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl = lid_raw_data_buffer.front()->points;
        double frame_header_time = lid_header_time_buffer.front();
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (auto pt : pcl)
        {
          if (pt.curvature < max_offs_time_ms)
            meas.pcl_proc_cur->points.push_back(pt);   // 当前 LIO 用
          else
            meas.pcl_proc_next->points.push_back(pt);  // 下一帧用
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      meas.measures.clear();

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img    = img_buffer.front();

      mtx_buffer.lock();
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();

      meas.measures.push_back(m);
      return true;
    }
  }
}
```

关键要点：
1. **以图像为时间基准**：LIO 的处理时刻对齐到图像曝光时刻，这样 VIO 和 LIO 使用同一参考时间。
2. **LiDAR 点云切分**：一帧 LiDAR 扫描可能跨越图像时刻，因此按 `pt.curvature`（相对帧头的时间偏移，单位 ms）把点拆到 `pcl_proc_cur` 和 `pcl_proc_next`。
3. **IMU 窗口**：只取时间戳小于等于当前处理时刻的 IMU，超出的留给下一帧。
4. **所有队列操作都加 `mtx_buffer` 锁**，但持锁时间尽量短。

### 4.3 完整示例：走一遍 LIVO 同步流程

下面用一个具体例子把上面代码串起来。假设传感器频率：
- 图像：30 Hz，每帧曝光时刻为 `0.033, 0.067, 0.100, 0.133, 0.167, 0.200, ...`
- LiDAR：10 Hz，每 **100 ms** 发布一帧点云，帧头时间为 `0.000, 0.100, 0.200, ...`
- **每帧 LiDAR 覆盖本帧帧头开始的约 100 ms 扫描数据**，所以点云中 `pt.curvature` 范围约为 `0 ~ 100 ms`
- IMU：200 Hz，时间戳为 `0.000, 0.005, 0.010, 0.015, ...`

`exposure_time_init` 设为 0，简化计算。

#### 初始状态

回调线程已经收了若干数据，缓冲区为：

```text
img_time_buffer:        [0.033, 0.067, 0.100, 0.133, 0.167, ...]
lid_header_time_buffer: [0.000, 0.100, 0.200, ...]
imu_buffer:             [0.000, 0.005, 0.010, ..., 0.150, ...]
```

`meas.lio_vio_flg` 初始为 `WAIT`，`meas.last_lio_update_time = -1.0`。

---

#### 第 1 轮：LIO + VIO 处理图像 0.033 s

##### LIO 阶段

`sync_packages()` 进入 `case WAIT/case VIO` 分支：

```cpp
double img_capture_time = img_time_buffer.front();  // 0.033
```

检查数据是否够新：

```cpp
lid_newest_time = lid_header_time_buffer.back()   // 0.200
                + lid_raw_data_buffer.back()->points.back().curvature / 1000.0;  // ~100 ms
              ≈ 0.300
```

`0.033 < 0.300`，LiDAR 数据足够新，继续。

取 IMU：

```cpp
m.lio_time = 0.033;

while (!imu_buffer.empty())
{
  if (stamp2Sec(imu_buffer.front()->header.stamp) > 0.033) break;
  if (stamp2Sec(imu_buffer.front()->header.stamp) > -1.0)
    m.imu.push_back(imu_buffer.front());
  imu_buffer.pop_front();
}
```

结果：IMU `0.000 ~ 0.030` 进入 `m.imu`。

切分 LiDAR：

```cpp
frame_header_time = 0.000
max_offs_time_ms  = (0.033 - 0.000) * 1000.0f;  // = 33 ms
```

当前 LiDAR 帧的 `curvature` 范围是 `0 ~ 100 ms`，所以 `curvature < 33 ms` 的点给 `pcl_proc_cur`，`curvature >= 33 ms` 的点给 `pcl_proc_next`。

结果：
- `pcl_proc_cur`：LiDAR 第 1 帧中 `0.000 ~ 0.033 s` 的点
- `pcl_proc_next`：LiDAR 第 1 帧中 `0.033 ~ 0.100 s` 的点

然后消费掉这帧 LiDAR。

返回并执行 LIO：

```cpp
meas.measures.push_back(m);
meas.lio_vio_flg = LIO;
return true;
```

主线程调用 `processImu()` + `stateEstimationAndMapping()` → `handleLIO()`。

`processImu()` 中：
- `prop_beg_time = 0`（`last_prop_end_time` 初始值）
- `prop_end_time = m.lio_time = 0.033`
- IMU 从 0 传播到 0.033 s，给 LiDAR 点去畸变

`handleLIO()` 中：
- 用去畸变后的点云更新 `_state`
- **调用 `publish_odometry(pubOdomAftMapped)` 发布 `/aft_mapped_to_init`**
- `_state` 现在对应 0.033 s

##### VIO 阶段

下一次 `sync_packages()` 进入 `case LIO` 分支（上一轮是 LIO，本轮执行 VIO）：

```cpp
double img_capture_time = img_time_buffer.front();  // 0.033
meas.lio_vio_flg = VIO;

struct MeasureGroup m;
m.vio_time = 0.033;
m.lio_time = meas.last_lio_update_time;  // 0.033，上一次 LIO 更新时刻
m.img = img_buffer.front();              // 图像 1

mtx_buffer.lock();
img_buffer.pop_front();                  // 消费图像 1
img_time_buffer.pop_front();
mtx_buffer.unlock();

meas.measures.push_back(m);
return true;
```

注意：VIO 测量组**没有放新的 IMU**（代码里没有 `m.imu.push_back`）。

主线程调用 `processImu()` + `stateEstimationAndMapping()` → `handleVIO()`。

`processImu()` 中：
- `prop_beg_time = last_prop_end_time = 0.033`
- `prop_end_time = m.vio_time = 0.033`
- 传播区间长度为 0，且 `meas.imu` 为空，所以状态保持为上一次 LIO 后的 `_state`

`handleVIO()` 中：
- 直接用 0.033 s 的 `_state` 处理图像 1
- **不发布 odom**

---

#### 第 2 轮：LIO + VIO 处理图像 0.067 s

##### LIO 阶段

`sync_packages()` 再次进入 `case WAIT/case VIO` 分支，`img_capture_time = 0.067`。

缓冲区：

```text
img_time_buffer:        [0.067, 0.100, 0.133, ...]
lid_header_time_buffer: [0.100, 0.200, ...]
```

上一帧 LiDAR（0.000）已被消费。

队首 LiDAR 帧头 `0.100 > 0.067`，不消费新 LiDAR 帧。

但代码先把 `pcl_proc_next` 搬进 `pcl_proc_cur`：

```cpp
*(meas.pcl_proc_cur) = *(meas.pcl_proc_next);     // 0.033 ~ 0.100 s 的点
PointCloudXYZI().swap(*meas.pcl_proc_next);
```

然后检查新 LiDAR 帧：`0.100 > 0.067`，break。

再切分 `pcl_proc_cur`：

```cpp
// 此时 pcl_proc_cur 里的点相对于 last_lio_update_time=0.033 的偏移
// 需要保留 0.033 ~ 0.067 s 的部分
```

实际上代码在 `processImu()` 里会根据 `prop_beg_time` 和 `prop_end_time` 切分。`prop_beg_time = 0.033`，`prop_end_time = 0.067`，所以 0.033~0.067 s 的点参与当前 LIO。

取 IMU：`0.035 ~ 0.065` 进入 `m.imu`。

执行 LIO：
- `_state` 从 0.033 传播到 0.067 s
- 用 0.033~0.067 s 的 LiDAR 点更新 `_state`
- **发布 `/aft_mapped_to_init`**

##### VIO 阶段

消费图像 2，`m.vio_time = 0.067`，`m.lio_time = 0.067`。

`handleVIO()` 用 0.067 s 的 `_state` 处理图像 2。

---

#### 第 3 轮：LIO + VIO 处理图像 0.100 s

##### LIO 阶段

`img_capture_time = 0.100`。

队首 LiDAR 帧头 `0.100 <= 0.100`，消费这帧 LiDAR。

先把 `pcl_proc_next`（上一轮剩下的 0.067~0.100 s）搬进 `pcl_proc_cur`。

然后切分新 LiDAR 帧（帧头 0.100，覆盖 0.100~0.200 s）：

```cpp
max_offs_time_ms = (0.100 - 0.100) * 1000.0f;  // = 0 ms
```

所以：
- `curvature < 0 ms`：没有
- `curvature >= 0 ms`：整帧新 LiDAR 给 `pcl_proc_next`

本轮 LIO 用：
- `pcl_proc_cur`：上一轮 LiDAR 剩下的 `0.067 ~ 0.100 s`

执行 LIO，状态更新到 0.100 s，发布 odom。

##### VIO 阶段

消费图像 3，执行 VIO。

---

#### 第 4 轮：LIO + VIO 处理图像 0.133 s

##### LIO 阶段

`img_capture_time = 0.133`。

新 LiDAR 帧还没来（下一帧在 0.200），不消费新 LiDAR。

把 `pcl_proc_next`（0.100~0.200 s）搬进 `pcl_proc_cur`。

切分：

```cpp
max_offs_time_ms = (0.133 - 0.100) * 1000.0f;  // = 33 ms
```

结果：
- `pcl_proc_cur`：`0.100 ~ 0.133 s` 的点
- `pcl_proc_next`：`0.133 ~ 0.200 s` 的点

执行 LIO，状态更新到 0.133 s，发布 odom。

##### VIO 阶段

消费图像 4，执行 VIO。

---

#### 第 5 轮：LIO + VIO 处理图像 0.167 s

类似第 2 轮：
- 无新 LiDAR
- `pcl_proc_cur` = 上一轮的 `pcl_proc_next` 中 `0.133 ~ 0.167 s` 的部分
- 执行 LIO，发布 odom
- VIO 消费图像 5

---

#### 第 6 轮：LIO + VIO 处理图像 0.200 s

`img_capture_time = 0.200`。

新 LiDAR 帧（帧头 0.200）到达，消费它。

`pcl_proc_cur` = 上一轮剩下的 `0.167 ~ 0.200 s`。

新 LiDAR 帧（0.200~0.300 s）因为 `max_offs_time_ms = 0`，全部给 `pcl_proc_next`。

执行 LIO，发布 odom。

---

#### 关于 VIO 阶段 IMU 传播的源码细节

在 `IMU_Processing.cpp` 的 `UndistortPcl()` 里：

```cpp
const double prop_beg_time = last_prop_end_time;
const double prop_end_time = lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;
```

所以 VIO 阶段：
- `prop_beg_time = last_prop_end_time`（上一次 LIO/VIO 结束时刻）
- `prop_end_time = m.vio_time`（当前图像时刻）

但是，`sync_packages()` 的 VIO 分支里没有给 `m.imu` 放新的 IMU 数据，所以 `v_imu` 只有上一帧最后一个 IMU：

```cpp
auto v_imu = meas.imu;       // 空
v_imu.push_front(last_imu);  // 只有 1 个元素

for (int i = 0; i < v_imu.size() - 1; i++)  // 循环不执行
```

因此 VIO 阶段虽然传播区间存在，但实际的 IMU 传播循环不会执行，`_state` 保持在上一次 LIO 更新后的状态，直接用于视觉前端。

---

#### 小结

| 时刻 | 事件 | odom 发布 |
|------|------|----------|
| 0.000 | LiDAR 第 1 帧帧头；扫描覆盖 0.000~0.100 s | — |
| 0.033 | **图像 1 曝光**；LIO 用 LiDAR 0.000~0.033 + IMU 0~0.030；**发布 odom** | ✅ |
| 0.033 | VIO 消费图像 1 | ❌ |
| 0.067 | **图像 2 曝光**；LIO 用 LiDAR 0.033~0.067 + IMU 0.035~0.065；**发布 odom** | ✅ |
| 0.067 | VIO 消费图像 2 | ❌ |
| 0.100 | **图像 3 曝光**；LiDAR 第 2 帧（0.100~0.200）到达；LIO 用 LiDAR 0.067~0.100；**发布 odom** | ✅ |
| 0.100 | VIO 消费图像 3 | ❌ |
| 0.133 | **图像 4 曝光**；LIO 用 LiDAR 0.100~0.133；**发布 odom** | ✅ |
| 0.133 | VIO 消费图像 4 | ❌ |
| 0.167 | **图像 5 曝光**；LIO 用 LiDAR 0.133~0.167；**发布 odom** | ✅ |
| 0.167 | VIO 消费图像 5 | ❌ |
| 0.200 | **图像 6 曝光**；LiDAR 第 3 帧（0.200~0.300）到达；LIO 用 LiDAR 0.167~0.200；**发布 odom** | ✅ |

从这个时间线可以看出：
- **每帧 LiDAR（100 ms）大约支撑 3 次 LIO 更新**。
- LIO 频率 ≈ 10 Hz × 3 = **30 Hz 理论上限**，但实际受处理耗时和同步等待影响，会低一些。
- **odom 只在 LIO 后发布**，VIO 阶段不发布 odom。

### 4.4 消费数据：processImu() + stateEstimationAndMapping()

`sync_packages` 返回后，`LidarMeasures` 中已经准备好：
- `LidarMeasures.measures.back().imu`：当前帧需要的 IMU 序列
- `LidarMeasures.pcl_proc_cur`：当前 LIO 需要的点云
- `LidarMeasures.measures.back().img`：VIO 需要的图像（VIO 阶段）
- `LidarMeasures.lio_vio_flg`：当前是 LIO 还是 VIO

主线程调用：

```cpp
void LIVMapper::processImu() 
{
  p_imu->Process2(LidarMeasures, _state, feats_undistort);
  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO: handleVIO(); break;
    case LIO:
    case LO:  handleLIO(); break;
  }
}
```

- `handleLIO()`：使用 `feats_undistort` 做 voxel map 状态估计。
- `handleVIO()`：使用 `LidarMeasures.measures.back().img` 做视觉前端跟踪。

## 5. IMU 预测线程/回调

除了主数据流，还有一个 `imu_prop_timer`（每 4 ms 触发）用于高频 IMU 传播，发布高频里程计：

```cpp
imu_prop_timer = this->node->create_wall_timer(
    0.004s, std::bind(&LIVMapper::imu_prop_callback, this));
```

```cpp
void LIVMapper::imu_prop_callback()
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) return;

  mtx_buffer_imu_prop.lock();
  new_imu = false;
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    // 丢弃已经过时的 IMU
    while (!prop_imu_buffer.empty() &&
           stamp2Sec(prop_imu_buffer.front().header.stamp) < latest_ekf_time)
      prop_imu_buffer.pop_front();

    // 用最新 EKF 状态做 IMU 前向传播
    for (int i = 0; i < prop_imu_buffer.size(); i++)
    {
      // prop_imu_once(...)
    }
  }
  mtx_buffer_imu_prop.unlock();
}
```

注意：
- 它使用独立的 `mtx_buffer_imu_prop`，与主数据缓冲区的 `mtx_buffer` 分开。
- 这样高频 IMU 传播不会和 LIO/VIO 主线程抢同一把锁，也不会阻塞 DDS 回调。

## 6. 从同步到状态估计的完整数据流

很多人会问：`sync_packages()` 拿到数据之后，到 LIO/VIO 真正更新状态，中间到底发生了什么？这里把流程再拆开讲一遍。

### 6.1 你理解的流程，大部分是对的

你的直觉大致正确：

1. 找到图像曝光时刻；
2. 收集该时刻之前的 LiDAR 和 IMU；
3. 用 IMU 做前向传播，得到每个 IMU 时刻的位姿；
4. 用后向传播给 LiDAR 点去畸变；
5. 做 LIO 更新；
6. VIO 用已经传播到图像时刻的状态处理图像。

### 6.2 但有几个细节需要纠正

#### （1）不是找“LiDAR 扫描开始/结束时刻的 IMU”，而是用整段 IMU 连续传播

`sync_packages()` 取的是**时间戳 ≤ 当前目标时刻**的所有 IMU，组成一个序列。`UndistortPcl()` 里会把上一帧的最后一个 IMU 也接在当前帧前面：

```cpp
auto v_imu = meas.imu;
v_imu.push_front(last_imu);   // 保证传播连续性
```

然后以 `prop_beg_time = last_prop_end_time`（上一次 LIO/VIO 结束时刻）为起点，`prop_end_time = meas.lio_time`（当前图像时刻）为终点，逐段做 IMU 前向传播。

所以 IMU 传播区间是连续的：`[last_prop_end_time, img_capture_time]`，而不是只在 LiDAR 开始/结束两个瞬间采样。

#### （2）LIO 的目标时间已经是图像曝光时间

在 LIVO 模式下，`sync_packages()` 的 LIO 分支设置：

```cpp
m.lio_time = img_capture_time;   // LIO 传播到这个时刻
```

所以 LIO 阶段的 IMU 前向传播，**终点就是图像曝光时刻**。LIO 更新完后，系统状态 `_state` 已经被推进到图像时刻了。

#### （3）VIO 阶段不“继续往前传播”，而是直接用 LIO 后的状态

VIO 分支：

```cpp
m.vio_time = img_capture_time;
m.lio_time = meas.last_lio_update_time;  // 也就是上一次的图像时刻
```

在 `processImu()` 里：

```cpp
prop_beg_time = last_prop_end_time;       // 上一次 LIO/VIO 结束时刻
prop_end_time = meas.vio_time;            // 当前图像时刻
```

所以 VIO 阶段的传播区间是 `[last_prop_end_time, img_capture_time]`，理论上不为零。**但是**，VIO 测量组里没有放新的 IMU 数据（`sync_packages` 的 VIO 分支没有 `m.imu.push_back`），所以 `v_imu` 只有上一帧最后一个 IMU，实际的 IMU 传播循环不会执行，`_state` 保持在上一次 LIO 更新后的状态。

然后 `handleVIO()` 直接用 `_state` 处理图像。

所以正确的顺序是：

```
图像 0.033 s 曝光
   ↓
sync_packages 组织 LIO 数据（lio_time = 0.033）
   ↓
processImu() → UndistortPcl()
   - 前向传播：last_prop_end_time (0.000) → 0.033
   - 后向去畸变：把 LiDAR 点投影到 0.033 时刻
   ↓
stateEstimationAndMapping() → handleLIO()
   - 用去畸变后的点云更新状态 _state
   - 发布 /aft_mapped_to_init
   - _state 现在对应 0.033 s
   ↓
下一次 sync_packages 进入 VIO 分支
   ↓
processImu() → UndistortPcl()
   - prop_end_time = 0.033，但 meas.imu 为空，传播循环不执行
   ↓
stateEstimationAndMapping() → handleVIO()
   - 直接用 0.033 s 的 _state 做视觉前端
   - 不发布 odom
```

### 6.3 为什么要让 LIO 对齐到图像时刻？

如果不这样，就会出现：
- LIO 在 LiDAR 帧尾时刻（比如 0.010）更新一次；
- VIO 在图像时刻（比如 0.033）更新一次；
- 两次状态更新之间差了 23 ms，视觉和激光的状态不同步。

把 LIO 也拉到图像时刻后，LIO 和 VIO 共享同一个状态基准，后续做激光-视觉联合优化时才不会打架。

### 6.4 关键变量时间线

| 变量 | 含义 | LIO 阶段取值 | VIO 阶段取值 |
|------|------|-------------|-------------|
| `meas.lio_time` | IMU 传播终点 / 状态更新时间 | `img_capture_time` | `last_lio_update_time` |
| `meas.vio_time` | 图像曝光时间 | 未使用 | `img_capture_time` |
| `prop_beg_time` | IMU 前向传播起点 | `last_prop_end_time` | `last_prop_end_time` |
| `prop_end_time` | IMU 前向传播终点 | `meas.lio_time` | `meas.vio_time` |
| `last_prop_end_time` | 记录给下一轮 | 更新为 `prop_end_time` | 更新为 `prop_end_time` |

### 6.5 一个容易混淆的点

注意 `meas.lio_time` 在 LIO 阶段和 VIO 阶段的含义不同：
- **LIO 阶段**：`lio_time` 就是要传播的终点，等于当前图像时刻。
- **VIO 阶段**：`lio_time` 记录的是上一次 LIO 更新的时刻，`vio_time` 才是当前图像时刻。此时传播区间为零，仅用于保持状态。

### 6.6 `stateEstimationAndMapping()` 的执行频率与多 LiDAR 帧处理

#### （1）执行频率是否和图像帧率一致？

主线程循环是 `rclcpp::Rate(5000)`，但只有当 `sync_packages()` 返回 `true` 时，才会真正调用一次 `stateEstimationAndMapping()`。在 LIVO 模式下，`sync_packages()` 每成功一次只产生**一个测量组**（LIO 或 VIO 二选一），并且这两个阶段是**交替出现**的：

```
图像 0.033 s 曝光
   ↓
sync_packages() 返回 LIO 测量组
   ↓
stateEstimationAndMapping() → handleLIO()
   ↓
下一循环
   ↓
sync_packages() 返回 VIO 测量组
   ↓
stateEstimationAndMapping() → handleVIO()
   ↓
图像 0.067 s 曝光
   ↓
LIO → VIO → ...
```

所以：
- `stateEstimationAndMapping()` 被**有效调用**的频率约为 **2 × 图像帧率**（每帧图像触发一次 LIO + 一次 VIO）。
- 但 LIO 状态更新频率 = 图像帧率，VIO 状态更新频率 = 图像帧率。
- 如果你把 LIO 和 VIO 合起来算作一次“完整 LIVO 更新”，那完整更新频率才和图像帧率一致。

#### （2）连续两帧图像之间出现多个 LiDAR 帧怎么办？

 Livox MID360 典型帧率是 10 Hz（100 ms 一帧），图像 30 Hz（33 ms 一帧），所以经常会出现**一帧图像间隔内包含多帧 LiDAR** 的情况。

在 LIVO 的 LIO 分支里，`sync_packages()` 会一次性消费缓冲区中所有**帧头时间 ≤ 当前图像时刻**的 LiDAR 帧：

```cpp
while (!lid_raw_data_buffer.empty())
{
  if (lid_header_time_buffer.front() > img_capture_time) break;

  auto pcl = lid_raw_data_buffer.front()->points;
  double frame_header_time = lid_header_time_buffer.front();
  float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

  for (int i = 0; i < pcl.size(); i++)
  {
    auto pt = pcl[i];
    if (pcl[i].curvature < max_offs_time_ms)
    {
      // 该点在当前图像时刻之前，参与本次 LIO
      pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
      meas.pcl_proc_cur->points.push_back(pt);
    }
    else
    {
      // 该点在当前图像时刻之后，留给下一次 LIO
      pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
      meas.pcl_proc_next->points.push_back(pt);
    }
  }

  lid_raw_data_buffer.pop_front();
  lid_header_time_buffer.pop_front();
}
```

处理规则：
1. **整帧取出**：所有帧头在图像时刻之前的 LiDAR 帧都会被从缓冲区弹出。
2. **按点切分**：对每一帧，按 `curvature`（相对帧头的时间偏移，单位 ms）判断单个点属于当前 LIO 还是下一次 LIO。
3. **累积使用**：多个 LiDAR 帧中、落在 `[last_lio_update_time, img_capture_time]` 区间内的点，会合并进 `pcl_proc_cur`，一次性用于当前 LIO 更新。
4. **跨帧缓存**：图像时刻之后的点进入 `pcl_proc_next`，在下一帧图像时刻被继续消费。

也就是说，**LIO 不是按 LiDAR 帧来处理，而是按“图像时刻”来切分点云**。一个图像周期内可能消费多帧 LiDAR，但只产生一次 LIO 状态更新。

#### （3）实际意义

- 如果图像 30 Hz、LiDAR 10 Hz：大约每 3 帧图像对应 1 帧 LiDAR，多数 LIO 阶段只会消费 1 帧 LiDAR。
- 如果图像 30 Hz、LiDAR 50/100 Hz（如部分固态 LiDAR）：一个图像间隔内会有多帧 LiDAR，LIO 会把它们累积并按图像时刻切分。
- 这种设计的优点是 LIO 和 VIO 严格共享同一时间基准；缺点是如果某段 LiDAR 数据堆积过多，单次 LIO 的计算量会变大。

## 7. LIO 与 VIO 的更新机制

### 7.1 LIO 使用平面假设（point-to-plane）更新

`handleLIO()` 调用 `voxelmap_manager->StateEstimation(state_propagat)`，核心测量模型是**点到局部平面的距离残差**。

流程：
1. 把去畸变后的 LiDAR 点云 `feats_down_body` 按当前状态 `_state` 转到世界系。
2. 对每个点，在 voxel map 中找最近的平面（`BuildResidualListOMP`）。
3. 构造残差：

```
residual = -dis_to_plane
         = -(normal · (point_world - plane_center) + d)
```

4. 对状态（旋转 + 平移，6 DOF）求 Jacobian，构建 `H` 和 `R_inv`。
5. 用迭代 EKF 更新 `_state`。

所以 LIO 确实是**基于平面假设**的状态更新。voxel map 里的每个叶子节点都会拟合一个局部平面，满足平面条件才参与残差计算。

### 7.2 VIO 使用光度残差（photometric residual）更新

`handleVIO()` 调用 `vio_manager->processFrame(...)`，核心测量模型是**直接法视觉前端**。

流程：
1. 从 voxel map 中投影一批视觉地图点到当前图像。
2. 对每个地图点，取参考帧的小块（patch），用仿射变换 warp 到当前帧。
3. 计算光度误差：

```
error = ref_intensity * (ref_expo / cur_expo) - cur_intensity
```

4. 对图像块误差关于相机位姿（以及曝光时间、特征点逆深度）求 Jacobian。
5. 用 EKF 更新 `_state`。

所以 VIO **不使用平面假设**，它用的是像素亮度一致性。

### 7.3 它们是在同一个时间点更新吗？

**不是“都在 LIO 时刻更新”，而是都对齐到同一个图像曝光时刻更新。**

LIO 阶段 `m.lio_time = img_capture_time`，VIO 阶段 `m.vio_time = img_capture_time`。所以两个阶段的**目标时刻都是图像曝光时刻**，不是 LiDAR 扫描结束时刻，也不是某个独立的 “LIO 时刻”。

**它们是同一个时间基准，但不是同一个函数调用里同时更新。**

执行顺序：

```
图像 0.033 s
   ↓
LIO 阶段
   - IMU 传播到 0.033 s
   - LiDAR 点去畸变到 0.033 s
   - point-to-plane EKF 更新 _state
   - _state 现在对应 0.033 s
   ↓
VIO 阶段
   - 直接用 0.033 s 的 _state
   - photometric EKF 再次更新 _state
   - _state 仍对应 0.033 s
```

也就是说：
- LIO 和 VIO 都在**图像曝光时刻 0.033 s** 这一时间基准上更新状态。
- 但它们是**分两步**完成的：先 LIO 更新，再 VIO 更新。
- VIO 用的是 LIO 更新之后的 `_state`，而不是 IMU 传播完但还没做 LIO 的状态。

这种设计的优点是：
- LIO 提供稳定的位姿初值和局部地图。
- VIO 在此基础上用图像做精细修正，同时更新曝光参数。

缺点是：
- 如果 LIO 在这帧有偏差，VIO 会在偏差基础上继续更新。
- 两步 EKF 不是联合优化，没有同时考虑激光平面残差和视觉光度残差。

### 7.4 代码对应关系

| 阶段 | 入口函数 | 核心类 | 测量模型 |
|------|---------|--------|---------|
| LIO | `handleLIO()` | `VoxelMapManager::StateEstimation()` | point-to-plane residual |
| VIO | `handleVIO()` | `VIOManager::processFrame()` | photometric residual |

## 8. 同步机制总结

| 组件 | 线程 | 保护锁 | 说明 |
|------|------|--------|------|
| `lid_raw_data_buffer` / `lid_header_time_buffer` | 回调线程写，主线程读 | `mtx_buffer` | LiDAR 点云帧缓冲 |
| `imu_buffer` | 回调线程写，主线程读 | `mtx_buffer` | IMU 测量缓冲 |
| `img_buffer` / `img_time_buffer` | 回调线程写，主线程读 | `mtx_buffer` | 图像帧缓冲 |
| `prop_imu_buffer` / `newest_imu` | IMU 回调写，IMU prop timer 读 | `mtx_buffer_imu_prop` | 高频 IMU 传播 |
| `LidarMeasures` | 主线程独占 | 无（单生产者单消费者，主线程内使用） | 当前处理测量组 |

## 9. 关键设计经验

1. **回调线程只做最轻量的事**：接收、拷贝、时间戳修正、入队。复杂预处理在锁外完成。
2. **一把粗粒度锁保护所有主缓冲区**：`mtx_buffer` 同时保护 LiDAR / IMU / Image 队列。虽然粒度粗，但持锁时间极短，简化了时序同步逻辑。
3. **主线程轮询而非条件变量**：使用 `rclcpp::Rate(5000)` 周期性检查数据是否就绪。实现简单，但在数据稀疏时会有轻微空转。
4. **以图像时刻为 LIO/VIO 统一基准**：保证激光和视觉测量在时间上对齐。
5. **LiDAR 点云跨帧切分**：利用 Livox 点云中 `curvature` 字段存储的相对时间偏移，实现按图像时刻精确切分。
6. **高频 IMU 传播独立锁**：避免 IMU 传播阻塞主数据流。

## 10. 调试建议

如果发现 VIO Append 点数低或 IMU sync warning 多，可从同步角度检查：

- **回调线程是否被饿死**：用 `ros2 topic hz` 检查 `/livox/lidar`、`/camera/camera/infra1/image_rect_raw` 在算法运行期间是否掉频率。
- **缓冲区是否堆积**：在 `sync_packages()` 前后打印 `lid_raw_data_buffer.size()` / `img_buffer.size()` / `imu_buffer.size()`，看是否持续增长。
- **时间戳是否对齐**：检查 `last_timestamp_imu - last_timestamp_lidar` 是否长期大于 0.1 s。
- **主线程耗时**：`LIVMapper::run()` 中已经打印 `stateEstimationAndMapping()` 耗时，若单帧超过 100 ms，说明主线程太忙，需要考虑降采样或换更强算力。
