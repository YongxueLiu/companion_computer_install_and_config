# VINS-Fusion-ROS2 程序执行过程与数据流详解

本文基于 VINS-Fusion-ROS2 源码，系统梳理从传感器数据输入到最终位姿输出的完整执行链路，帮助开发者快速定位关键代码并理解系统内部的数据流转机制。

---

## 1. 系统架构概览

VINS-Fusion-ROS2 包含 **4 个 ROS2 功能包**，分别承担不同的职责：

| 包名 | 职责 | 主要可执行文件 |
|------|------|--------------|
| `vins` | 核心视觉惯性里程计（VIO） | `vins_node` |
| `loop_fusion` | 回环检测与位姿图优化 | `loop_fusion_node` |
| `global_fusion` | GPS 与 VIO 融合，全局位姿优化 | `global_fusion_node` |
| `camera_models` | 相机模型与标定工具 | `Calibrations` |

**最小运行依赖**：仅需启动 `vins_node` 即可输出实时 VIO 位姿。`loop_fusion_node` 与 `global_fusion_node` 为可选模块，分别用于消除长期漂移和提供地理坐标系下的全局一致性。

---

## 2. `vins_node` 的启动与线程模型

### 2.1 主函数入口

入口位于 `vins/src/rosNodeTest.cpp` 的 `main()` 函数，执行流程如下：

```
rclcpp::init(argc, argv)
  └─> 读取 YAML 配置文件路径（通过 rclcpp::remove_ros_arguments 过滤 ROS 参数）
      └─> readParameters(config_file)      // 加载全局参数
          └─> estimator.setParameter()     // 初始化 Estimator
              └─> featureTracker.readIntrinsicParameter() // 加载相机内参
                  └─> 若 MULTIPLE_THREAD=1，启动 processThread
                      └─> Estimator::processMeasurements()
```

### 2.2 订阅的 ROS2 Topic

| Topic | 消息类型 | 回调函数 | 说明 |
|-------|---------|---------|------|
| `IMU_TOPIC` (配置中指定) | `sensor_msgs/Imu` | `imu_callback` | IMU 原始数据 |
| `IMAGE0_TOPIC` | `sensor_msgs/Image` | `img0_callback` | 左目/单目图像 |
| `IMAGE1_TOPIC` (双目时) | `sensor_msgs/Image` | `img1_callback` | 右目图像 |
| `/vins_restart` | `std_msgs/Bool` | `restart_callback` | 系统重启指令 |
| `/vins_imu_switch` | `std_msgs/Bool` | `imu_switch_callback` | IMU 开关 |
| `/vins_cam_switch` | `std_msgs/Bool` | `cam_switch_callback` | 单双目切换 |

> 注：VINS-Fusion-ROS2 版本中，`feature_callback`（订阅 `/feature_tracker/feature`）已被标记为 deprecated，特征跟踪改在 `vins_node` 内部完成。

### 2.3 多线程结构

`vins_node` 内部至少存在 **3 条并发执行线**：

1. **ROS2 Executor 线程**：处理 ROS2 订阅回调（`imu_callback`、`img0_callback`、`img1_callback`），负责将外部数据写入缓冲队列。
2. **Sync 线程**（`sync_process`）：将左右目图像按时间戳对齐（容忍 3ms），取到同步帧后调用 `estimator.inputImage()`。
3. **Process 线程**（`processMeasurements`，仅在 `MULTIPLE_THREAD=1` 时启动）：从 `featureBuf` 中取出特征帧，联合 IMU 数据进行滑动窗口优化。

若 `MULTIPLE_THREAD=0`，`inputImage()` 在同步线程中直接调用 `processMeasurements()`，变为单线程串行处理。

---

## 3. 核心数据流：从传感器到位姿输出

以下按数据到达的时间顺序，拆解 VIO 内部的关键处理阶段。

### 3.1 Stage 1：图像接收与同步 (`sync_process`)

```
[ROS2 图像消息]
    │
    ├─> img0_callback ──> img0_buf (queue)
    └─> img1_callback ──> img1_buf (queue)
            │
            ▼
    sync_process 线程循环：
        ├─ 比较 img0_buf.front 与 img1_buf.front 的时间戳
        ├─ 若差值 < 0.003s，认为同步成功
        ├─ 取出图像并 cv_bridge 转换为 cv::Mat
        ├─ 丢弃旧帧（防止实时性滞后）
        └─> estimator.inputImage(time, image0, image1)
```

代码路径：`vins/src/rosNodeTest.cpp`（Line 74~146）

### 3.2 Stage 2：前端特征跟踪 (`FeatureTracker::trackImage`)

`inputImage()` 首先调用 `featureTracker.trackImage()` 进行前端处理：

```
cv::Mat (当前帧灰度图)
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│  FeatureTracker::trackImage()                               │
│  ├─ 1. 若存在 prev_pts，执行 cv::calcOpticalFlowPyrLK        │
│  │      （金字塔 LK 光流跟踪上一帧特征点到当前帧）            │
│  │      可选：FLOW_BACK 反向光流剔除外点                    │
│  │      可选：GPU_MODE 使用 CUDA 加速                       │
│  ├─ 2. 若存在右目图像，对左目跟踪成功的点进行右目 LK 匹配    │
│  ├─ 3. setMask()：按跟踪次数排序，在 MIN_DIST 半径内做 NMS  │
│  ├─ 4. cv::goodFeaturesToTrack：补充新特征点               │
│  ├─ 5. 去畸变并计算像素速度 (ptsVelocity)                   │
│  └─ 6. 返回 featureFrame: map<feature_id, vector<cam_id,   │
│         Eigen::Matrix<double,7,1>>>                          │
│         [x,y,z, u,v, vx,vy]                                 │
└─────────────────────────────────────────────────────────────┘
    │
    ▼
estimator.inputImage() 将 featureFrame 压入 featureBuf
```

代码路径：`vins/src/featureTracker/feature_tracker.cpp`（Line 104 起）

### 3.3 Stage 3：IMU 数据接收与预积分 (`inputIMU / processIMU`)

```
[ROS2 Imu 消息]
    │
    ▼
imu_callback ──> accBuf / gyrBuf (queue)
    │
    ▼
processMeasurements 线程中：
    getIMUInterval(prevTime, curTime) ──> 提取两帧图像间的所有 IMU 数据
        │
        ▼
    for each IMU sample:
        processIMU(t, dt, acc, gyr)
        ├─ 若 frame_count != 0:
        │   pre_integrations[frame_count]->push_back(dt, acc, gyr)
        │   tmp_pre_integration->push_back(...)
        └─ 中值积分传播当前位姿（用于高频 IMU 预测输出）
```

- `IntegrationBase`（`vins/src/factor/integration_base.h`）负责维护两帧之间的 IMU 预积分量 $\alpha, \beta, \gamma$（位置、速度、旋转增量）以及对应的协方差。
- 在 `solver_flag == NON_LINEAR` 阶段，`inputIMU()` 还会调用 `fastPredictIMU()` 高频发布 `imu_propagate` 话题，用于实时预测位姿。

### 3.4 Stage 4：滑动窗口与初始化 (`processImage`)

`processMeasurements` 在拿到同步后的 `feature` 与 `IMU` 数据后，执行 `processImage()`：

```
processImage(featureFrame, header)
    │
    ├─> f_manager.addFeatureCheckParallax(frame_count, image, td)
    │       判断当前帧是否为关键帧（视差足够大 -> MARGIN_OLD）
    │
    ├─> imageframe.pre_integration = tmp_pre_integration
    │       将当前帧的 IMU 预积分绑定到图像帧
    │       tmp_pre_integration = new IntegrationBase(...) // 为下一帧新建
    │
    ├─> [若 ESTIMATE_EXTRINSIC == 2] 在线标定外参旋转 Ric
    │
    ├─> 若 solver_flag == INITIAL（初始化阶段）：
    │       ├─ Mono + IMU:
    │       │   frame_count == WINDOW_SIZE 时
    │       │   └─> initialStructure()  // 视觉 SfM + IMU 对齐
    │       ├─ Stereo + IMU:
    │       │   frame_count == WINDOW_SIZE 时
    │       │   └─> PnP 初始化位姿 -> solveGyroscopeBias -> optimization
    │       └─ Stereo only:
    │           └─> PnP + triangulate -> optimization
    │
    └─> 若 solver_flag == NON_LINEAR（正常运行阶段）：
            ├─> f_manager.triangulate(...)   // 三角化新特征点深度
            ├─> optimization()                // Ceres 滑动窗口优化
            ├─> outliersRejection(...)        // 外点剔除
            ├─> slideWindow()                 // 边缘化滑窗
            └─> updateLatestStates()          // 更新最新状态用于 IMU 预测
```

**初始化细节**：
- `initialStructure()`（`vins/src/estimator/estimator.cpp` Line 600+）：
  1. 检查视差，选取参考帧；
  2. `solveRelativeRT`（5 点法求相对位姿）；
  3. `construct`（全局 SfM，`initial_sfm.cpp`）；
  4. `visualInitialAlign()`（`initial_alignment.cpp`）：求解重力方向、尺度、速度、陀螺仪偏置 `Bgs`。

代码路径：`vins/src/estimator/estimator.cpp`（Line 454 起）

### 3.5 Stage 5：后端优化 (`optimization`)

`optimization()` 使用 **Ceres Solver** 构建滑动窗口中的最大后验估计问题：

```
待优化参数块（Ceres ParameterBlock）：
    ├─ para_Pose[WINDOW_SIZE+1]      // 7 DoF: [x,y,z, qx,qy,qz,qw]
    ├─ para_SpeedBias[WINDOW_SIZE+1] // 9 DoF: [vx,vy,vz, Bax,Bay,Baz, Bgx,Bgy,Bgz]
    ├─ para_Ex_Pose[2]               // 相机-IMU 外参
    ├─ para_Feature[NUM_OF_F]        // 逆深度 (1 DoF)
    └─ para_Td[1]                    // IMU-Camera 时间同步误差 td

残差项（ResidualBlock）：
    ├─ 1. MarginalizationFactor      // 先验（上一帧边缘化后的信息）
    ├─ 2. IMUFactor                  // 相邻帧间 IMU 预积分残差
    │      pre_integrations[j]->delta_p, delta_q, delta_v
    ├─ 3. ProjectionTwoFrameOneCamFactor   // 单目：两帧+单相机重投影
    ├─ 4. ProjectionTwoFrameTwoCamFactor   // 双目：两帧+双相机重投影
    └─ 5. ProjectionOneFrameTwoCamFactor   // 双目：单帧+双相机重投影

Ceres 配置：
    ├─ linear_solver: DENSE_SCHUR（或 CUDA，若 USE_GPU_CERES=1）
    ├─ trust_region_strategy: DOGLEG
    └─ max_solver_time: SOLVER_TIME（Keyframe 时放宽到 0.8*SOLVER_TIME）
```

优化完成后调用 `double2vector()` 将参数数组写回 `Ps`、`Rs`、`Vs`、`Bas`、`Bgs` 等状态变量。

代码路径：`vins/src/estimator/estimator.cpp`（Line 1064 起）

### 3.6 Stage 6：边缘化 (`slideWindow` / `Marginalization`)

根据关键帧判断结果，边缘化策略分为两类：

| 策略 | 触发条件 | 操作 |
|------|---------|------|
| `MARGIN_OLD` | 视差大（关键帧） | 边缘化掉滑动窗口中最老的一帧，将其视觉与 IMU 信息转为先验 |
| `MARGIN_SECOND_NEW` | 视差小（非关键帧） | 边缘化掉次新帧，保留最老帧，仅丢弃该帧的视觉观测 |

边缘化通过 Schur Complement 实现，构造 `MarginalizationFactor` 作为下一优化的先验约束。

代码路径：`vins/src/estimator/estimator.cpp`（Line 1198 起）及 `vins/src/factor/marginalization_factor.cpp`

### 3.7 Stage 7：结果输出与可视化

`processMeasurements` 在每次 `processImage` 后调用 `pubOdometry()` 等函数发布结果：

| 发布 Topic | 消息类型 | 内容 | 代码位置 |
|-----------|---------|------|---------|
| `odometry` | `nav_msgs/Odometry` | 滑动窗口最新帧位姿 `Ps[WINDOW_SIZE]`, `Rs[WINDOW_SIZE]` | `utility/visualization.cpp` |
| `imu_propagate` | `nav_msgs/Odometry` | IMU 高频预测位姿 | `utility/visualization.cpp` |
| `path` | `nav_msgs/Path` | 历史轨迹 | `utility/visualization.cpp` |
| `point_cloud` | `sensor_msgs/PointCloud` | 当前帧三角化的 3D 地图点 | `utility/visualization.cpp` |
| `margin_cloud` | `sensor_msgs/PointCloud` | 被边缘化掉的关键帧地图点 | `utility/visualization.cpp` |
| `keyframe_pose` | `nav_msgs/Odometry` | 关键帧位姿（供 loop_fusion 订阅） | `utility/visualization.cpp` |
| `keyframe_point` | `sensor_msgs/PointCloud` | 关键帧特征点（供 loop_fusion 订阅） | `utility/visualization.cpp` |
| `image_track` | `sensor_msgs/Image` | 前端跟踪可视化图像 | `utility/visualization.cpp` |
| `extrinsic` | `nav_msgs/Odometry` | 相机-IMU 外参 | `utility/visualization.cpp` |

---

## 4. 可选模块的数据流

### 4.1 `loop_fusion_node` — 回环检测与位姿图优化

启动命令：
```bash
ros2 run loop_fusion loop_fusion_node [config_file]
```

**输入 Topic**：
| Topic | 来源 | 说明 |
|-------|------|------|
| `IMAGE0_TOPIC` | 相机驱动 | 原始图像 |
| `keyframe_pose` | `vins_node` | 关键帧位姿 |
| `keyframe_point` | `vins_node` | 关键帧地图点（含 3D 坐标、归一化坐标、UV、feature_id） |
| `extrinsic` | `vins_node` | 相机外参 |

**内部处理**：

```
process 线程：
    ├─ 同步 image / point / pose，时间戳对齐
    ├─ 构建 KeyFrame 对象（图像、位姿、BRIEF 描述子、3D 点）
    ├─> PoseGraph::addKeyFrame()
            ├─ detectLoop()   // DBoW2 词袋查询历史关键帧
            ├─ 若检测到回环 (loop_index != -1):
            │      findConnection()  // BRIEF 描述子匹配 + PnP 几何验证
            │      计算相对位姿 relative_t, relative_q
            │      将当前帧索引压入 optimize_buf
            └─ 应用当前 drift 修正后发布 path

optimize4DoF / optimize6DoF 线程：
    ├─ 从 optimize_buf 取出需要优化的关键帧
    └─ Ceres 位姿图优化（VIO 提供相邻帧约束，回环提供闭环约束）
```

**输出 Topic**：
- `pose_graph_path`：回环修正后的全局轨迹
- `base_path`：未修正的 VIO 原始轨迹
- `odometry_rect`：drift 修正后的实时位姿（供 RViz 显示）

代码路径：`loop_fusion/src/pose_graph_node.cpp`、`loop_fusion/src/pose_graph.cpp`

### 4.2 `global_fusion_node` — GPS 融合

启动命令：
```bash
ros2 run global_fusion global_fusion_node
```

**输入 Topic**：
| Topic | 来源 | 说明 |
|-------|------|------|
| `/vins_estimator/odometry` | `vins_node` | VIO 位姿 |
| `/gps` | GPS 驱动 | `sensor_msgs/NavSatFix` |

**内部处理**：

```
vio_callback:
    ├─> globalEstimator.inputOdom(t, vio_t, vio_q)
    ├─ 同步 GPS 队列（10ms 容差）
    │   └─> globalEstimator.inputGPS(t, lat, lon, alt, posAccuracy)
    └─ 获取当前 global 位姿并发布

optimize 线程（后台循环）：
    ├─ 当收到新 GPS 时触发
    ├─ Ceres 优化问题：
    │   ├─ VIO 因子 (RelativeRTError)：相邻帧相对位姿约束
    │   └─ GPS 因子 (TError)：全局平移约束
    └─ 更新 WGPS_T_WVIO（VIO 坐标系到 GPS 坐标系的变换）
```

**输出 Topic**：
- `global_odometry`：GPS 对齐后的全局位姿
- `global_path`：全局轨迹
- `car_model`：车辆模型 MarkerArray（RViz 可视化）

代码路径：`global_fusion/src/globalOptNode.cpp`、`global_fusion/src/globalOpt.cpp`

---

## 5. 关键数据结构与坐标系

### 5.1 滑动窗口状态

在 `Estimator` 中，滑动窗口大小为 `WINDOW_SIZE = 10`，状态数组如下：

```cpp
Vector3d  Ps[WINDOW_SIZE + 1];   // 世界系下 IMU 位置
Vector3d  Vs[WINDOW_SIZE + 1];   // 世界系下 IMU 速度
Matrix3d  Rs[WINDOW_SIZE + 1];   // 世界系下 IMU 旋转
Vector3d  Bas[WINDOW_SIZE + 1];  // 加速度计偏置
Vector3d  Bgs[WINDOW_SIZE + 1];  // 陀螺仪偏置
```

- **世界系（World Frame）**：初始化时以第一帧 IMU 的重力方向为 Z 轴建立的ENU/ENU-like 坐标系。
- **Body Frame**：IMU 坐标系。
- **Camera Frame**：相机坐标系，通过外参 `ric[0/1]`、`tic[0/1]` 与 Body Frame 关联。

### 5.2 Feature 的数据表示

`featureFrame` 是特征跟踪器传给估计器的核心数据结构：

```cpp
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
//   ^ feature_id
//            ^ camera_id (0: 左目, 1: 右目)
//                      ^ [x, y, z, u, v, vx, vy]
//                        x,y,z: 归一化平面坐标 (z=1)
//                        u,v:   像素坐标
//                        vx,vy: 像素速度（用于 td 估计）
```

---

## 6. 执行流程总结图

```
┌─────────────┐     ┌─────────────┐
│  IMU Driver │     │ Cam Driver  │
│ (ROS2 Node) │     │ (ROS2 Node) │
└──────┬──────┘     └──────┬──────┘
       │                   │
       ▼                   ▼
  imu_callback        img0_callback / img1_callback
       │                   │
       ▼                   ▼
  accBuf/gyrBuf       img0_buf/img1_buf
       │                   │
       │              sync_process 线程
       │                   │
       │                   ▼
       │           estimator.inputImage(t, img0, img1)
       │                   │
       │              ┌─────────────────────────────┐
       │              │ FeatureTracker::trackImage  │
       │              │  - LK 光流跟踪              │
       │              │  - 双目匹配 (若 STEREO=1)   │
       │              │  - 生成 featureFrame        │
       │              └─────────────────────────────┘
       │                   │
       │              featureBuf (若 MULTIPLE_THREAD)
       │                   │
       └───────────────> processMeasurements 线程
                           │
           ┌───────────────┼───────────────┐
           ▼               ▼               ▼
     getIMUInterval    processIMU      processImage
     (提取 IMU 数据)   (预积分/传播)   (后端核心)
                           │               │
                           │          ┌──────────────────────────┐
                           │          │ solver_flag == INITIAL   │
                           │          │   -> initialStructure()  │
                           │          │   -> visualInitialAlign()│
                           │          └──────────────────────────┘
                           │          ┌──────────────────────────┐
                           │          │ solver_flag == NON_LINEAR│
                           │          │   -> triangulate()       │
                           │          │   -> optimization()      │
                           │          │   -> outliersRejection() │
                           │          │   -> slideWindow()       │
                           │          └──────────────────────────┘
                           │               │
                           ▼               ▼
                    updateLatestStates()  pubOdometry()
                           │               │
                    fastPredictIMU()      │
                           │               │
                           ▼               ▼
                    pub_latest_odometry  odometry / path / point_cloud
                           │               │
                           ▼               ▼
                     (高频 IMU 预测)   (下游: RViz / loop_fusion / global_fusion)
```

---

## 7. 代码速查表

| 功能 | 文件路径 |
|------|---------|
| 节点入口、Topic 订阅 | `vins/src/rosNodeTest.cpp` |
| 参数读取 | `vins/src/estimator/parameters.cpp` |
| 滑动窗口估计器主逻辑 | `vins/src/estimator/estimator.cpp` |
| 特征跟踪器 | `vins/src/featureTracker/feature_tracker.cpp` |
| 初始化：SfM | `vins/src/initial/initial_sfm.cpp` |
| 初始化：IMU 对齐 | `vins/src/initial/initial_aligment.cpp` |
| IMU 预积分 | `vins/src/factor/integration_base.h` |
| Ceres 优化问题构建 | `vins/src/estimator/estimator.cpp` (`optimization()`) |
| 边缘化因子 | `vins/src/factor/marginalization_factor.cpp` |
| 可视化发布 | `vins/src/utility/visualization.cpp` |
| 回环节点 | `loop_fusion/src/pose_graph_node.cpp` |
| 词袋/回环检测 | `loop_fusion/src/pose_graph.cpp` |
| GPS 融合节点 | `global_fusion/src/globalOptNode.cpp` |
| GPS/VIO 联合优化 | `global_fusion/src/globalOpt.cpp` |

---

## 8. 调试建议

1. **确认图像时间同步**：查看终端是否频繁输出 `throw img0` 或 `throw img1`，说明左右目时间戳不同步。
2. **确认 IMU 数据到达**：若终端持续输出 `wait for imu ...`，检查 IMU_TOPIC 配置与驱动是否正常发布。
3. **查看初始化状态**：`solver_flag` 从 `INITIAL` 切换到 `NON_LINEAR` 时，终端会打印 `Initialization finish!`。
4. **前端跟踪可视化**：将 `SHOW_TRACK` 设为 1，订阅 `/image_track` 可在 RViz 中查看光流跟踪效果。
5. **性能分析**：终端会输出 `solver costs: xxx [ms]`，若该值持续接近 `SOLVER_TIME`，说明后端计算吃紧，可考虑降低 `MAX_CNT` 或减少 `NUM_ITERATIONS`。

---

> **作者注**：本文基于 VINS-Fusion-ROS2 源码梳理，核心逻辑与原始 VINS-Fusion（ROS1 版本）保持一致，差异主要在于 ROS2 的节点创建、QoS、参数解析及 `ament` 构建系统。若需深入某一模块（如预积分推导、边缘化数学原理），建议配合原论文 *VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator* 及 *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry* 阅读。
