# VINS-Fusion 世界坐标系说明

## `/odometry` 中的 x, y, z 是什么

**是的，`/odometry.pose.pose.position.x/y/z` 就是相机（body/camera0）在世界坐标系下的三维位置。**

VINS-Fusion 的滑动窗口优化完成后，会将当前帧的位姿发布到 `/odometry` 话题上。这个位姿是相对于**世界坐标系（world frame）**的，不是相对于上一帧的。

---

## 世界坐标系的原点在哪里

**原点 = 初始化完成瞬间的 camera0（左红外相机）位置。**

代码中体现在：
```cpp
// estimator.cpp visualInitialAlign()
Ps[0] = 0;   // 第一帧位置强制归零，作为世界原点
```

**关键点：每次重启 VINS，世界原点都会变。**

- 你启动 VINS → 相机静止等待初始化 → 初始化成功 → 此时相机所在位置 = 世界原点
- 下次重启 → 初始化时的位置不同 → 原点又变了

所以 VINS 的世界坐标系是**局部坐标系**，不是全局固定的。如果要与真实物理空间对齐，需要在初始化时让相机对准某个已知方向，并记录初始化时的物理位置。

---

## 世界坐标系的三个轴分别指向什么方向

VINS-Fusion 的世界坐标系建立规则如下（源码 + 实测验证）。

### 前提：body / IMU 坐标系

RealSense D435i 的 IMU 数据发布在 `camera_imu_optical_frame` 中：

| 轴 | 方向 |
|---|---|
| **X** | right（右）|
| **Y** | down（下）|
| **Z** | forward（前）|

静止时典型加速度读数（水平放置）：
```
linear_acceleration: x≈0, y≈-9.8, z≈0
```

VINS 配置文件中的 `body_T_cam0` 旋转设为 `I`，意味着 VINS 直接把这个 optical frame 当作 body frame 使用。

### Z 轴 —— 垂直向上（与重力对齐）

```cpp
// utility.cpp g2R()
Eigen::Vector3d ng1 = g.normalized();        // 重力方向（body系下）
Eigen::Vector3d ng2{0, 0, 1.0};              // 世界Z轴正方向
R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
```

`g2R` 的作用：把重力向量旋转到世界坐标系的 Z 轴正方向。对 D435i 而言，body 的 Y 轴向下，重力加速度 `g ≈ [0, -G, 0]`，`FromTwoVectors([0,-1,0], [0,0,1])` 相当于把 body 的 Y-down 转到 world 的 Z-up。因此无论初始化时相机是斜放还是倒放，最终世界坐标系的 **Z 轴永远垂直向上**。

### X / Y 轴 —— 由初始 body 朝向和 yaw 归零共同决定

```cpp
// estimator.cpp visualInitialAlign()
Matrix3d R0 = Utility::g2R(g);
double yaw = Utility::R2ypr(R0 * Rs[0]).x();
R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
```

`R2ypr(R).x()` 取的是 `R` 的**第 0 列**在 world XY 平面上的方位角，也就是 **body X 轴（right）**的水平朝向。把 yaw 归零意味着：

- **World X 轴 = 初始化时 body 的 right 方向在水平面的投影**
- **World Y 轴 = 初始化时 body 的 forward 方向在水平面的投影**（与 X 垂直，满足 X × Y = Z）

**实测验证**：运行 VINS 并手持相机移动
- 往**右**移动 → `/odometry.pose.pose.position.x` **增大**
- 往**前**移动 → `/odometry.pose.pose.position.y` **增大**
- 往**上**移动 → `z` **增大**

这与 World X=right、Y=forward、Z=up 完全吻合。

### 与 ENU 的关系

如果初始化时相机满足：
- body right（X）指向**正东**
- body forward（Z）指向**正北**

那么 VINS world 就是标准的 ENU（East-North-Up）。

**但 VINS 本身不保证这一点**。`g2R` + yaw 归零只保证 **Z=up 且 body right 投影到 X 轴**，不提供真北参考。因此更准确的描述是：

> VINS world 是一个**重力对齐的局部水平坐标系**，Z=up，X=初始化时 body 的 right，Y=初始化时 body 的 forward。只有在初始化 body right 朝东、forward 朝北时，它才是 ENU。

---

## 总结

| 轴 | 方向 | 确定方式 |
|---|---|---|
| **X** | 初始化时 body 的 right 方向在水平面的投影 | yaw 角基于 body X 归零 |
| **Y** | 初始化时 body 的 forward 方向在水平面的投影 | 右手坐标系，与 X 垂直 |
| **Z** | 垂直向上 | 与重力反方向对齐 |
| **原点** | 初始化完成时的相机位置 | 每次重启都会变 |

**实测口诀**：
- 右移 → x 增
- 前移 → y 增
- 上移 → z 增

---

## 实际使用中的注意事项

1. **不要跨重启对比位姿数值**  
   每次重启 VINS，原点和方向都可能不同，位姿数值没有可比性。

2. **如果想让坐标系与物理空间对齐**  
   初始化时把相机的 right（X）对准正东、forward（Z）对准正北，这样 world 就接近 ENU。否则只能得到"局部水平系"，没有真北。

3. **Z 值不是海拔高度**  
   VINS 的 Z 是相对于初始化原点的垂直高度，不是 GPS 意义上的海拔。相机往上抬，Z 增加；往下放，Z 减小。

4. **纯视觉模式没有绝对尺度**  
   stereo 模式下尺度来自双目基线，是固定的。如果是单目模式，尺度来自初始化时的运动，可能不稳定。

5. **body frame 不是 FLU**  
   对于 RealSense D435i + librealsense 的默认配置，VINS body frame 是 optical frame（X-right, Y-down, Z-forward），不是航空常用的 FLU（X-forward, Y-left, Z-up）。在写 PX4 bridge 的姿态转换时务必注意这一点，不能把 FLU 的 `q_frd_to_flu` 直接套用到 VINS 上。
