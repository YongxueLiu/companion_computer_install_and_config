# FAST-LIVO2 Dual-Thread Data Synchronization

> This article explains how FAST-LIVO2-ROS2 separates ROS2 DDS callbacks from the heavy mapping loop, and how the two threads synchronize over shared sensor buffers.
>
> Relevant source files:
> - `src/LIVMapper.cpp`
> - `include/LIVMapper.h`
> - `include/common_lib.h`

## 1. Why Two Threads?

The FAST-LIVO2 main thread performs expensive computations:
- IMU pre-integration and LiDAR point undistortion
- LIO (LiDAR-inertial odometry) state estimation and voxel-map updates
- VIO (visual-inertial odometry) feature tracking and visual-map management
- Publishing point clouds, images, and odometry

If these computations ran inside the ROS2 executor callback thread, DDS message reception would be blocked, causing:
- IMU lag or dropped messages
- Missed LiDAR scans
- RealSense USB disconnections due to stalled image callbacks

Therefore the code splits **ROS2 spin (callbacks)** and **the mapping loop (consumption)** into two threads:

```
+-------------------+      deque buffers      +------------------------+
|  ROS2 spin thread |  -------------------->  |   Mapping main thread  |
|  (DDS callbacks)  |   mtx_buffer protected  |  (LIO/VIO/state est.)  |
+-------------------+                         +------------------------+
        |                                                |
        v                                                v
   receive / preprocess data                       sync_packages()
   push to buffers                                 processImu()
                                                   stateEstimationAndMapping()
```

## 2. Thread Startup: run()

The split happens in `LIVMapper::run()`:

```cpp
void LIVMapper::run(rclcpp::Node::SharedPtr &node) 
{
  // 1. Single-threaded executor attached to this node
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(this->node);

  // 2. Run executor.spin() in its own thread for DDS callbacks
  std::thread spin_thread([&executor]() { executor.spin(); });

  // 3. Main thread enters its own consumption loop
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

  // 4. Gracefully stop and join the spin thread
  executor.cancel();
  if (spin_thread.joinable()) spin_thread.join();
}
```

Key design points:
- `executor.spin()` only triggers subscriptions and timers; it performs no algorithmic work.
- The main thread polls `sync_packages()` at 5000 Hz and sleeps when data is not ready.

## 3. Callback Thread: Reception and Buffering

### 3.1 Shared buffers

All cross-thread buffers are declared in `LIVMapper.h` as `std::deque`s:

```cpp
std::mutex mtx_buffer, mtx_buffer_imu_prop;

deque<PointCloudXYZI::Ptr>       lid_raw_data_buffer;    // LiDAR scans
deque<double>                    lid_header_time_buffer; // LiDAR frame timestamps
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer; // IMU messages
deque<cv::Mat>                   img_buffer;             // image frames
deque<double>                    img_time_buffer;        // image timestamps
deque<sensor_msgs::msg::Imu>     prop_imu_buffer;        // high-rate IMU propagation
```

### 3.2 Callback workflow

The Livox CustomMsg callback is representative:

```cpp
void LIVMapper::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in)
{
  if (!lidar_en) return;

  double cur_head_time = stamp2Sec(msg_in->header.stamp);

  // ---- 1. Preprocess outside the lock ----
  livox_ros_driver2::msg::CustomMsg::SharedPtr msg(new livox_ros_driver2::msg::CustomMsg(*msg_in));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);   // parsing / downsampling can be heavy

  if (!ptr || ptr->empty()) { /* error */ return; }

  // ---- 2. Lock and write to shared queue ----
  mtx_buffer.lock();
  if (abs(last_timestamp_imu.load() - cur_head_time) > 1.0 && !imu_buffer.empty())
  {
    // Large IMU-LiDAR time gap; print info used for self-sync
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

The IMU callback is similar, and also copies the IMU into the independent `prop_imu_buffer` under `mtx_buffer_imu_prop`:

```cpp
mtx_buffer.lock();
last_timestamp_imu.store(timestamp);
imu_buffer.push_back(msg);
mtx_buffer.unlock();

if (imu_prop_enable)
{
  mtx_buffer_imu_prop.lock();
  if (!p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
  newest_imu = *msg;
  new_imu = true;
  mtx_buffer_imu_prop.unlock();
}
```

### 3.3 Preprocessing outside the lock

Notice that `p_pre->process()` is called **before** `mtx_buffer.lock()`. This is intentional:
- LiDAR preprocessing can be expensive (parsing Livox CustomMsg, filtering, format conversion).
- Holding `mtx_buffer` during preprocessing would block IMU and image callbacks that share the same mutex.
- Preprocessing only needs the raw message, so it is safe to run outside the critical section.

Only the minimal "push pointer/timestamp" operation is protected by the lock.

## 4. Main Thread: Synchronization and Consumption

### 4.1 Main loop

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

- `sync_packages()` tries to assemble a complete, time-synchronized `LidarMeasureGroup`.
- On success, the main thread proceeds to `processImu()` and `stateEstimationAndMapping()`.
- On failure, it sleeps and waits for the callback thread to collect more data.

### 4.2 sync_packages() core logic

`sync_packages(LidarMeasureGroup &meas)` branches on `slam_mode_`:

```cpp
bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en)   return false;
  if (imu_buffer.empty() && imu_en)   return false;

  switch (slam_mode_)
  {
    case ONLY_LIO: ...
    case LIVO:     ...   // focus of this article
    case ONLY_LO:  ...
  }
}
```

#### 4.2.1 LIVO mode: alternating LIO and VIO

In LIVO mode, `meas.lio_vio_flg` switches among `WAIT -> LIO -> VIO -> LIO -> VIO -> ...`:

- **LIO phase**: collect IMU and LiDAR points up to the current image capture time.
- **VIO phase**: consume one image and run the visual front-end.

```cpp
case LIVO:
{
  switch (meas.lio_vio_flg)
  {
    case WAIT:
    case VIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;

      // Ensure LiDAR and IMU have caught up to the image time
      double lid_newest_time = lid_header_time_buffer.back()
                               + lid_raw_data_buffer.back()->points.back().curvature / 1000.0;
      double imu_newest_time = stamp2Sec(imu_buffer.back()->header.stamp);

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
        return false;   // not enough data yet

      struct MeasureGroup m;
      m.lio_time = img_capture_time;

      // Pop IMU messages up to img_capture_time
      mtx_buffer.lock();
      while (!imu_buffer.empty())
      {
        if (stamp2Sec(imu_buffer.front()->header.stamp) > m.lio_time) break;
        if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.last_lio_update_time)
          m.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
      }
      mtx_buffer.unlock();

      // Split LiDAR points across the image time boundary
      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl = lid_raw_data_buffer.front()->points;
        double frame_header_time = lid_header_time_buffer.front();
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (auto pt : pcl)
        {
          if (pt.curvature < max_offs_time_ms)
            meas.pcl_proc_cur->points.push_back(pt);   // current LIO
          else
            meas.pcl_proc_next->points.push_back(pt);  // next frame
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

Key points:
1. **Image time is the common reference**: LIO is aligned to the image exposure moment so LIO and VIO share the same time base.
2. **LiDAR point splitting**: a single LiDAR scan may straddle the image time. The code uses `pt.curvature` (relative time offset in ms) to split points into `pcl_proc_cur` and `pcl_proc_next`.
3. **IMU windowing**: only IMU messages up to the current processing time are popped; later ones remain for the next frame.
4. **All queue operations hold `mtx_buffer`**, but only for short critical sections.

### 4.3 Complete Example: Walking Through a LIVO Sync Cycle

The following concrete example ties the code above together. Assume sensor rates:
- Camera: 30 Hz, exposure times at `0.033, 0.067, 0.100, 0.133, 0.167, 0.200, ...`
- LiDAR: 10 Hz, one point-cloud message every **100 ms**, frame header times at `0.000, 0.100, 0.200, ...`
- **Each LiDAR frame covers about 100 ms of scan data starting from its header**, so `pt.curvature` inside the frame is roughly `0 ~ 100 ms`
- IMU: 200 Hz, timestamps at `0.000, 0.005, 0.010, 0.015, ...`

Set `exposure_time_init = 0` for simplicity.

#### Initial state

The callback thread has already buffered some data:

```text
img_time_buffer:        [0.033, 0.067, 0.100, 0.133, 0.167, ...]
lid_header_time_buffer: [0.000, 0.100, 0.200, ...]
imu_buffer:             [0.000, 0.005, 0.010, ..., 0.150, ...]
```

`meas.lio_vio_flg` starts at `WAIT`, and `meas.last_lio_update_time = -1.0`.

---

#### Round 1: LIO + VIO for image 0.033 s

##### LIO phase

`sync_packages()` enters the `WAIT / VIO` branch:

```cpp
double img_capture_time = img_time_buffer.front();  // 0.033
```

Check whether sensor data is new enough:

```cpp
lid_newest_time = lid_header_time_buffer.back()   // 0.200
                + lid_raw_data_buffer.back()->points.back().curvature / 1000.0;  // ~100 ms
              ≈ 0.300
```

`0.033 < 0.300`, so LiDAR data is new enough.

Collect IMU:

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

Result: IMU messages from `0.000 ~ 0.030` go into `m.imu`.

Split the LiDAR frame:

```cpp
frame_header_time = 0.000
max_offs_time_ms  = (0.033 - 0.000) * 1000.0f;  // = 33 ms
```

The current LiDAR frame has `curvature` in `0 ~ 100 ms`, so points with `curvature < 33 ms` go to `pcl_proc_cur`, and points with `curvature >= 33 ms` go to `pcl_proc_next`.

Result:
- `pcl_proc_cur`: points from LiDAR frame 1 in `0.000 ~ 0.033 s`
- `pcl_proc_next`: points from LiDAR frame 1 in `0.033 ~ 0.100 s`

Then this LiDAR frame is consumed.

Return and run LIO:

```cpp
meas.measures.push_back(m);
meas.lio_vio_flg = LIO;
return true;
```

The main thread calls `processImu()` and `stateEstimationAndMapping()` → `handleLIO()`.

Inside `processImu()`:
- `prop_beg_time = 0` (initial `last_prop_end_time`)
- `prop_end_time = m.lio_time = 0.033`
- IMU propagates from 0 to 0.033 s and undistorts the LiDAR points

Inside `handleLIO()`:
- Update `_state` with the undistorted point cloud
- **Call `publish_odometry(pubOdomAftMapped)` to publish `/aft_mapped_to_init`**
- `_state` now corresponds to 0.033 s

##### VIO phase

The next `sync_packages()` enters the `LIO` branch (last round was LIO, so this round runs VIO):

```cpp
double img_capture_time = img_time_buffer.front();  // 0.033
meas.lio_vio_flg = VIO;

struct MeasureGroup m;
m.vio_time = 0.033;
m.lio_time = meas.last_lio_update_time;  // 0.033, last LIO update time
m.img = img_buffer.front();              // image 1

mtx_buffer.lock();
img_buffer.pop_front();                  // consume image 1
img_time_buffer.pop_front();
mtx_buffer.unlock();

meas.measures.push_back(m);
return true;
```

Note: the VIO branch **does not push any new IMU messages** into `m.imu`.

The main thread calls `processImu()` and `stateEstimationAndMapping()` → `handleVIO()`.

Inside `processImu()`:
- `prop_beg_time = last_prop_end_time = 0.033`
- `prop_end_time = m.vio_time = 0.033`
- The propagation interval has length 0, and `meas.imu` is empty, so the state stays at the last LIO state

Inside `handleVIO()`:
- Process image 1 directly with the 0.033 s `_state`
- **Does not publish odom**

---

#### Round 2: LIO + VIO for image 0.067 s

##### LIO phase

`sync_packages()` enters the `WAIT / VIO` branch again, `img_capture_time = 0.067`.

Buffers:

```text
img_time_buffer:        [0.067, 0.100, 0.133, ...]
lid_header_time_buffer: [0.100, 0.200, ...]
```

The previous LiDAR frame (0.000) has been consumed. The next LiDAR header `0.100 > 0.067`, so no new LiDAR frame is consumed.

But the code first moves `pcl_proc_next` into `pcl_proc_cur`:

```cpp
*(meas.pcl_proc_cur) = *(meas.pcl_proc_next);     // 0.033 ~ 0.100 s
PointCloudXYZI().swap(*meas.pcl_proc_next);
```

Then it checks the new LiDAR frame: `0.100 > 0.067`, break.

Inside `processImu()`, `prop_beg_time = 0.033` and `prop_end_time = 0.067`, so points in `0.033 ~ 0.067 s` participate in this LIO.

Collect IMU: `0.035 ~ 0.065` go into `m.imu`.

Run LIO:
- Propagate `_state` from 0.033 to 0.067 s
- Update `_state` with LiDAR points from `0.033 ~ 0.067 s`
- **Publish `/aft_mapped_to_init`**

##### VIO phase

Consume image 2, `m.vio_time = 0.067`, `m.lio_time = 0.067`.

`handleVIO()` processes image 2 with the 0.067 s `_state`.

---

#### Round 3: LIO + VIO for image 0.100 s

##### LIO phase

`img_capture_time = 0.100`.

The front LiDAR header `0.100 <= 0.100`, so this LiDAR frame is consumed.

First, move `pcl_proc_next` (leftover `0.067 ~ 0.100 s`) into `pcl_proc_cur`.

Then split the new LiDAR frame (header 0.100, covering `0.100 ~ 0.200 s`):

```cpp
max_offs_time_ms = (0.100 - 0.100) * 1000.0f;  // = 0 ms
```

Result:
- `curvature < 0 ms`: none
- `curvature >= 0 ms`: the entire new LiDAR frame goes to `pcl_proc_next`

This LIO uses:
- `pcl_proc_cur`: leftover `0.067 ~ 0.100 s` from the previous LiDAR frame

Run LIO, update state to 0.100 s, publish odom.

##### VIO phase

Consume image 3, run VIO.

---

#### Round 4: LIO + VIO for image 0.133 s

##### LIO phase

`img_capture_time = 0.133`.

No new LiDAR frame yet (next one at 0.200), so no new LiDAR is consumed.

Move `pcl_proc_next` (`0.100 ~ 0.200 s`) into `pcl_proc_cur`.

Split:

```cpp
max_offs_time_ms = (0.133 - 0.100) * 1000.0f;  // = 33 ms
```

Result:
- `pcl_proc_cur`: points in `0.100 ~ 0.133 s`
- `pcl_proc_next`: points in `0.133 ~ 0.200 s`

Run LIO, update state to 0.133 s, publish odom.

##### VIO phase

Consume image 4, run VIO.

---

#### Round 5: LIO + VIO for image 0.167 s

Similar to round 2:
- No new LiDAR
- `pcl_proc_cur` = part of the previous `pcl_proc_next` in `0.133 ~ 0.167 s`
- Run LIO, publish odom
- VIO consumes image 5

---

#### Round 6: LIO + VIO for image 0.200 s

`img_capture_time = 0.200`.

New LiDAR frame (header 0.200) arrives and is consumed.

`pcl_proc_cur` = leftover `0.167 ~ 0.200 s`.

New LiDAR frame (`0.200 ~ 0.300 s`) has `max_offs_time_ms = 0`, so it all goes to `pcl_proc_next`.

Run LIO, publish odom.

---

#### Source-level detail: IMU propagation in the VIO phase

In `IMU_Processing.cpp`, `UndistortPcl()` sets:

```cpp
const double prop_beg_time = last_prop_end_time;
const double prop_end_time = lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;
```

So in the VIO phase:
- `prop_beg_time = last_prop_end_time` (the end time of the previous LIO/VIO step)
- `prop_end_time = m.vio_time` (the current image time)

However, the VIO branch in `sync_packages()` does **not** put new IMU data into `m.imu`. Therefore:

```cpp
auto v_imu = meas.imu;       // empty
v_imu.push_front(last_imu);  // only one element

for (int i = 0; i < v_imu.size() - 1; i++)  // loop does not execute
```

So although the propagation interval exists in the VIO phase, the actual IMU propagation loop does not run, and `_state` remains at the state produced by the previous LIO update, ready for the visual frontend.

---

#### Summary

| Time | Event | Odom published |
|------|-------|----------------|
| 0.000 | LiDAR frame 1 header; covers scan `0.000 ~ 0.100 s` | — |
| 0.033 | **Image 1 exposure**; LIO uses LiDAR `0.000 ~ 0.033` + IMU `0 ~ 0.030`; **publishes odom** | ✅ |
| 0.033 | VIO consumes image 1 | ❌ |
| 0.067 | **Image 2 exposure**; LIO uses LiDAR `0.033 ~ 0.067` + IMU `0.035 ~ 0.065`; **publishes odom** | ✅ |
| 0.067 | VIO consumes image 2 | ❌ |
| 0.100 | **Image 3 exposure**; LiDAR frame 2 (`0.100 ~ 0.200`) arrives; LIO uses LiDAR `0.067 ~ 0.100`; **publishes odom** | ✅ |
| 0.100 | VIO consumes image 3 | ❌ |
| 0.133 | **Image 4 exposure**; LIO uses LiDAR `0.100 ~ 0.133`; **publishes odom** | ✅ |
| 0.133 | VIO consumes image 4 | ❌ |
| 0.167 | **Image 5 exposure**; LIO uses LiDAR `0.133 ~ 0.167`; **publishes odom** | ✅ |
| 0.167 | VIO consumes image 5 | ❌ |
| 0.200 | **Image 6 exposure**; LiDAR frame 3 (`0.200 ~ 0.300`) arrives; LIO uses LiDAR `0.167 ~ 0.200`; **publishes odom** | ✅ |

From this timeline:
- **Each LiDAR frame (100 ms) supports about 3 LIO updates**.
- The theoretical upper bound for LIO frequency is 10 Hz × 3 = **30 Hz**, but the real rate is lower due to processing latency and synchronization waits.
- **Odom is published only after LIO**; VIO does not publish odom.

### 4.4 Consuming the synchronized data

After `sync_packages()` returns, `LidarMeasures` contains:
- `LidarMeasures.measures.back().imu`: IMU sequence for the current frame
- `LidarMeasures.pcl_proc_cur`: point cloud for the current LIO
- `LidarMeasures.measures.back().img`: image for VIO (VIO phase)
- `LidarMeasures.lio_vio_flg`: whether this is an LIO or VIO step

The main thread then calls:

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

- `handleLIO()` performs voxel-map state estimation using `feats_undistort`.
- `handleVIO()` runs the visual front-end using `LidarMeasures.measures.back().img`.

## 5. High-Rate IMU Propagation Timer

Besides the main data flow, an `imu_prop_timer` fires every 4 ms to propagate the latest EKF state at IMU rate:

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
    // Drop obsolete IMU messages
    while (!prop_imu_buffer.empty() &&
           stamp2Sec(prop_imu_buffer.front().header.stamp) < latest_ekf_time)
      prop_imu_buffer.pop_front();

    // Forward-propagate from the latest EKF state
    for (int i = 0; i < prop_imu_buffer.size(); i++)
    {
      // prop_imu_once(...)
    }
  }
  mtx_buffer_imu_prop.unlock();
}
```

Notes:
- It uses a separate mutex `mtx_buffer_imu_prop`.
- This prevents high-frequency propagation from contending with the main data buffers and DDS callbacks.

## 6. From Synchronization to State Estimation

After `sync_packages()` returns data, what happens before LIO/VIO actually update the state? This section walks through the full pipeline.

### 6.1 Your intuition is mostly correct

The overall flow is:

1. Find the image exposure time.
2. Collect LiDAR and IMU data before that time.
3. Use IMU measurements to forward-propagate and obtain poses at each IMU time.
4. Use backward propagation to undistort LiDAR points.
5. Run LIO update.
6. VIO processes the image using the state already propagated to the image time.

### 6.2 A few details need correction

#### (1) It is not "IMU at LiDAR start/end times"; it is continuous propagation over the whole IMU sequence

`sync_packages()` collects **all IMU messages with timestamps ≤ the current target time**. `UndistortPcl()` also prepends the last IMU message from the previous frame:

```cpp
auto v_imu = meas.imu;
v_imu.push_front(last_imu);   // keep propagation continuous
```

Then it forward-propagates from `prop_beg_time = last_prop_end_time` (the previous LIO/VIO end time) to `prop_end_time = meas.lio_time` (the current image time), step by step over every IMU interval.

So the propagation interval is continuous: `[last_prop_end_time, img_capture_time]`.

#### (2) The LIO target time is already the image exposure time

In LIVO mode, the LIO branch sets:

```cpp
m.lio_time = img_capture_time;   // LIO propagates to this moment
```

Therefore, during the LIO phase, IMU forward propagation already ends at the image exposure time. After LIO, `_state` corresponds to that image moment.

#### (3) VIO does not "continue propagating"; it directly uses the LIO-updated state

In the VIO branch:

```cpp
m.vio_time = img_capture_time;
m.lio_time = meas.last_lio_update_time;  // the previous image time
```

Inside `processImu()`:

```cpp
prop_beg_time = last_prop_end_time;       // end time of the previous LIO/VIO step
prop_end_time = meas.vio_time;            // current image time
```

So the VIO propagation interval is `[last_prop_end_time, img_capture_time]`, which is theoretically non-zero. **However**, the VIO branch in `sync_packages()` does not put new IMU data into `m.imu`, so `v_imu` contains only the last IMU of the previous frame. The actual IMU propagation loop does not execute, and `_state` remains at the state produced by the previous LIO update.

Then `handleVIO()` processes the image with `_state`.

Correct ordering:

```
Image at 0.033 s exposed
   ↓
sync_packages builds LIO data (lio_time = 0.033)
   ↓
processImu() → UndistortPcl()
   - forward propagate: last_prop_end_time (0.000) → 0.033
   - backward undistort: project LiDAR points to 0.033
   ↓
stateEstimationAndMapping() → handleLIO()
   - update _state with undistorted point cloud
   - publish /aft_mapped_to_init
   - _state now corresponds to 0.033 s
   ↓
Next sync_packages enters VIO branch
   ↓
processImu() → UndistortPcl()
   - prop_end_time = 0.033, but meas.imu is empty, so the propagation loop does not run
   ↓
stateEstimationAndMapping() → handleVIO()
   - process image with _state at 0.033 s
   - does not publish odom
```

### 6.3 Why align LIO to the image time?

Without this alignment:
- LIO updates at the LiDAR scan end (e.g., 0.010 s).
- VIO updates at the image time (e.g., 0.033 s).
- The two states would be 23 ms apart, making LiDAR-visual fusion inconsistent.

By pulling LIO to the image time, LIO and VIO share the same state reference.

### 6.4 Key-variable timeline

| Variable | Meaning | LIO phase value | VIO phase value |
|----------|---------|-----------------|-----------------|
| `meas.lio_time` | IMU propagation end / state update time | `img_capture_time` | `last_lio_update_time` |
| `meas.vio_time` | Image exposure time | unused | `img_capture_time` |
| `prop_beg_time` | IMU propagation start | `last_prop_end_time` | `last_prop_end_time` |
| `prop_end_time` | IMU propagation end | `meas.lio_time` | `meas.vio_time` |
| `last_prop_end_time` | Carried to next iteration | updated to `prop_end_time` | updated to `prop_end_time` |

### 6.5 A subtle point

`meas.lio_time` has different meanings in LIO and VIO phases:
- **LIO phase**: it is the propagation target, equal to the current image time.
- **VIO phase**: it records the previous LIO update time, while `vio_time` is the current image time. Propagation is zero-length.

### 6.6 Execution frequency of `stateEstimationAndMapping()` and handling multiple LiDAR frames

#### (1) Is the execution frequency equal to the image frame rate?

The main loop runs at `rclcpp::Rate(5000)`, but `stateEstimationAndMapping()` is only called when `sync_packages()` returns `true`. In LIVO mode, each successful `sync_packages()` call produces **only one measurement group** (either LIO or VIO), and the two phases **alternate**:

```
image exposed at 0.033 s
   ↓
sync_packages() returns a LIO group
   ↓
stateEstimationAndMapping() → handleLIO()
   ↓
next loop
   ↓
sync_packages() returns a VIO group
   ↓
stateEstimationAndMapping() → handleVIO()
   ↓
image exposed at 0.067 s
   ↓
LIO → VIO → ...
```

Therefore:
- The **effective call rate** of `stateEstimationAndMapping()` is about **2 × image frame rate** (one LIO and one VIO per image).
- However, the LIO state update rate equals the image frame rate, and the VIO state update rate also equals the image frame rate.
- If you treat one LIO + one VIO as a single “complete LIVO update”, then the complete update rate matches the image frame rate.

#### (2) What if multiple LiDAR frames arrive between two consecutive images?

A Livox MID360 typically runs at 10 Hz (100 ms per scan), while a camera at 30 Hz has a 33 ms interval. Thus, **multiple LiDAR scans can fall within one image interval**.

In the LIVO LIO branch, `sync_packages()` consumes **all LiDAR frames whose header time is ≤ the current image time** in one shot:

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
      // this point is before the image moment, use it now
      pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
      meas.pcl_proc_cur->points.push_back(pt);
    }
    else
    {
      // this point is after the image moment, save for next LIO
      pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
      meas.pcl_proc_next->points.push_back(pt);
    }
  }

  lid_raw_data_buffer.pop_front();
  lid_header_time_buffer.pop_front();
}
```

Rules:
1. **Whole frames are popped**: every LiDAR frame with a header time before the image moment is removed from the buffer.
2. **Per-point splitting**: within each frame, `curvature` (per-point relative offset in ms) decides whether a point belongs to the current LIO or the next LIO.
3. **Accumulation**: points from multiple LiDAR frames that fall inside `[last_lio_update_time, img_capture_time]` are merged into `pcl_proc_cur` and used in the current LIO update.
4. **Cross-frame caching**: points after the image moment go into `pcl_proc_next` and are consumed at the next image moment.

In short, **LIO processes point clouds by image-time slicing, not by whole LiDAR frames**. One image interval may consume several LiDAR scans, but it produces only one LIO state update.

#### (3) Practical implications

- Image 30 Hz + LiDAR 10 Hz: roughly one LiDAR scan per three image frames; most LIO phases consume a single scan.
- Image 30 Hz + LiDAR 50/100 Hz (some solid-state sensors): multiple scans per image interval; LIO accumulates and slices them by image time.
- The benefit is a strict common time base for LIO and VIO. The cost is that heavy LiDAR accumulation can make a single LIO step more expensive.

## 7. LIO and VIO Update Mechanisms

### 7.1 LIO uses the planar hypothesis (point-to-plane)

`handleLIO()` calls `voxelmap_manager->StateEstimation(state_propagat)`. Its measurement model is a **point-to-local-plane distance residual**.

Pipeline:
1. Transform the undistorted LiDAR points `feats_down_body` to the world frame with the current `_state`.
2. For each point, find the nearest plane in the voxel map (`BuildResidualListOMP`).
3. Build the residual:

```
residual = -dis_to_plane
         = -(normal · (point_world - plane_center) + d)
```

4. Compute the Jacobian w.r.t. the state (rotation + translation, 6 DOF) and form `H` and `R_inv`.
5. Update `_state` with an iterated EKF.

So LIO is indeed a **plane-hypothesis-based** state update. Each voxel-map leaf fits a local plane, and only planes meeting the planarity condition contribute residuals.

### 7.2 VIO uses the photometric residual

`handleVIO()` calls `vio_manager->processFrame(...)`. Its measurement model is a **direct visual frontend**.

Pipeline:
1. Project visual map points from the voxel map into the current image.
2. For each map point, take a reference-frame patch and warp it into the current frame with an affine transform.
3. Compute the photometric error:

```
error = ref_intensity * (ref_expo / cur_expo) - cur_intensity
```

4. Compute the Jacobian w.r.t. camera pose (plus exposure time and feature inverse depth).
5. Update `_state` with an EKF.

So VIO **does not use the planar hypothesis**; it relies on pixel intensity consistency.

### 7.3 Do they update at the same time instant?

**They are not “both updated at the LIO time”; they are both aligned to the same image exposure time.**

In the LIO phase `m.lio_time = img_capture_time`; in the VIO phase `m.vio_time = img_capture_time`. So the target time for both phases is the **image exposure time**, not the LiDAR scan end time and not a separate “LIO time”.

**They share the same time reference, but are not updated in the same function call.**

Execution order:

```
image at 0.033 s
   ↓
LIO phase
   - propagate IMU to 0.033 s
   - undistort LiDAR points to 0.033 s
   - point-to-plane EKF update of _state
   - _state now corresponds to 0.033 s
   ↓
VIO phase
   - reuse _state at 0.033 s
   - photometric EKF update of _state
   - _state still corresponds to 0.033 s
```

In short:
- Both LIO and VIO update the state at the **image exposure time, e.g. 0.033 s**.
- But they are **two sequential steps**: LIO first, VIO second.
- VIO uses `_state` **after** the LIO update, not the raw IMU-propagated state.

Advantages:
- LIO provides a stable pose prior and a local map.
- VIO refines it with visual information and estimates exposure parameters.

Drawbacks:
- If LIO drifts in this frame, VIO continues from a biased state.
- The two-step EKF is not a joint optimization; laser plane residuals and visual photometric residuals are not minimized together.

### 7.4 Code mapping

| Phase | Entry function | Core class | Measurement model |
|-------|---------------|------------|-------------------|
| LIO | `handleLIO()` | `VoxelMapManager::StateEstimation()` | point-to-plane residual |
| VIO | `handleVIO()` | `VIOManager::processFrame()` | photometric residual |

## 8. Synchronization Summary

| Component | Thread | Mutex | Purpose |
|-----------|--------|-------|---------|
| `lid_raw_data_buffer` / `lid_header_time_buffer` | callback writes, main reads | `mtx_buffer` | LiDAR scan buffer |
| `imu_buffer` | callback writes, main reads | `mtx_buffer` | IMU measurement buffer |
| `img_buffer` / `img_time_buffer` | callback writes, main reads | `mtx_buffer` | image frame buffer |
| `prop_imu_buffer` / `newest_imu` | IMU callback writes, prop timer reads | `mtx_buffer_imu_prop` | high-rate IMU propagation |
| `LidarMeasures` | main thread only | none | current synchronized measurement group |

## 9. Design Takeaways

1. **Callbacks stay lightweight**: receive, copy, timestamp correction, enqueue. Heavy preprocessing runs outside the lock.
2. **One coarse mutex for the main buffers**: `mtx_buffer` protects all LiDAR / IMU / image queues. The critical sections are very short, so a single mutex keeps the synchronization logic simple.
3. **Polling, not condition variables**: the main thread uses `rclcpp::Rate(5000)` to periodically check for data. This is simple but introduces slight idle spinning when data is sparse.
4. **Image time as the common reference**: LIO and VIO are aligned to the image capture moment.
5. **Split LiDAR scans by point timestamp**: Livox `curvature` stores per-point relative time, enabling precise splitting around the image boundary.
6. **Independent mutex for IMU propagation**: keeps high-frequency propagation from blocking the main data flow.

## 10. Debugging Hints

If VIO appends are low or IMU sync warnings persist, inspect the synchronization path:

- **Callback starvation**: check topic rates with `ros2 topic hz` while the algorithm runs.
- **Buffer growth**: print `lid_raw_data_buffer.size()`, `img_buffer.size()`, and `imu_buffer.size()` inside `sync_packages()` to see if queues keep growing.
- **Timestamp drift**: monitor `last_timestamp_imu - last_timestamp_lidar`; a persistent gap > 0.1 s indicates sync issues.
- **Main-thread latency**: `LIVMapper::run()` already logs `stateEstimationAndMapping()` duration. If it exceeds ~100 ms, consider downsampling or using more compute.
