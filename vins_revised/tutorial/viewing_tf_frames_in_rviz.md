# 在 RViz 中显示 TF 坐标系

本文档介绍如何在 RViz 中可视化 RealSense 和 VINS 的各坐标系（TF Frames），帮助理解传感器安装位姿与算法输出的空间关系。

---

## 1. 基本概念

**TF (Transform Frame)** 是 ROS 中描述坐标系之间变换关系的工具。每个坐标系都有一个唯一的 `frame_id`，TF 树描述了这些坐标系之间的父子关系。

在 RViz 中，坐标系显示为 **红(X)-绿(Y)-蓝(Z)** 的三轴箭头：
- **红色箭头** 指向 X 轴正方向
- **绿色箭头** 指向 Y 轴正方向  
- **蓝色箭头** 指向 Z 轴正方向

---

## 2. RealSense 驱动发布的 TF

当 RealSense 节点启动时（默认 `publish_tf:=true`），会自动发布以下 TF 树：

```
camera_link                          # 相机基座标系（物理外壳中心）
├── camera_depth_frame               # 深度模组机械中心
│   └── camera_depth_optical_frame   # 深度图像坐标系（Z轴向前，X向右，Y向下）
├── camera_color_frame               # RGB模组机械中心
│   └── camera_color_optical_frame   # RGB图像坐标系
├── camera_infra1_frame              # 左红外模组机械中心
│   └── camera_infra1_optical_frame  # 左红外图像坐标系
├── camera_infra2_frame              # 右红外模组机械中心
│   └── camera_infra2_optical_frame  # 右红外图像坐标系
├── camera_gyro_frame                # 陀螺仪坐标系
└── camera_accel_frame               # 加速度计坐标系
```

> **关键区别**：
> - `*_frame` 是机械安装坐标系，通常 Z 轴指向上方
> - `*_optical_frame` 是图像坐标系，**Z 轴指向镜头前方**（光轴方向），X 向右，Y 向下
> - VINS 使用的图像话题，其 `header.frame_id` 通常是 `camera_infra1_optical_frame`

---

## 3. VINS 发布的 TF

启动 VINS 后，`visualization.cpp` 中的 `pubTF()` 会发布：

```
world                 # VINS 初始化时建立的世界坐标系（重力对齐，初始位置为原点）
└── body              # IMU/机体坐标系（随设备移动）
    └── camera        # 相机坐标系（相对于 body 的固定外参变换）
```

| Frame | 含义 | 动态/静态 |
|-------|------|-----------|
| `world` | 世界坐标系，Z轴朝上（重力方向），原点在初始化位置 | 固定 |
| `body` | IMU/机体坐标系，通常前-右-下或前-左-上 | 实时跟踪 |
| `camera` | 相机坐标系，与 `body` 有固定外参关系 | 实时跟踪 |

---

## 4. RViz 中显示 TF 的操作步骤

### 4.1 添加 TF 显示

1. 在 RViz 左下角点击 **Add**
2. 选择 **TF**（位于 rviz 分类下）
3. 左侧 Displays 列表会出现 **TF** 项

### 4.2 配置 Fixed Frame

TF 显示需要一个根坐标系作为参考。根据你想观察的内容选择：

| 观察目标 | Fixed Frame 建议 |
|---------|-----------------|
| 只看 RealSense 各传感器相对关系 | `camera_link` |
| 看 VINS 轨迹和位姿 | `world` |
| 看点云内部结构 | `camera_depth_optical_frame` |

### 4.3 显示/隐藏特定坐标系

展开 **TF** 项，你会看到所有可用的 frame_id 列表：
- 勾选/取消勾选前面的复选框来控制显示/隐藏
- 被勾选的坐标系会在 3D 视图中显示为三轴箭头
- **Frame Timeout** 默认 15 秒，如果 TF 太久没更新会变灰色

### 4.4 调整显示样式

在 TF 的设置中：
- **Marker Scale**: 调整坐标轴箭头的大小（建议 0.3~1.0）
- **Show Names**: 勾选后会在每个坐标系旁边显示 frame_id 文字标签
- **Show Arrows**: 控制是否显示箭头（如果只想看文字可以关掉）

---

## 5. 常见使用场景

### 场景 1：验证 RealSense 内外参

启动 RealSense：
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_infra1:=true enable_infra2:=true \
  enable_depth:=true enable_color:=true
```

RViz 配置：
- Fixed Frame: `camera_link`
- 添加 TF，勾选 `camera_infra1_optical_frame`、`camera_infra2_optical_frame`、`camera_depth_optical_frame`
- 添加两个 Image 显示左右目图像

效果：你可以直观看到两个红外相机的光轴有微小夹角（基线约 55mm），以及深度相机和 RGB 相机的位置差异。

### 场景 2：验证 VINS 初始化与世界坐标系

同时启动 RealSense + VINS：
```bash
# Terminal 1: RealSense
ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true

# Terminal 2: VINS
ros2 run vins vins_node /path/to/realsense_stereo_imu_config.yaml
```

RViz 配置：
- Fixed Frame: `world`
- 添加 TF，勾选 `world`、`body`、`camera`
- 添加 Path，Topic 选 `/vins_estimator/path`

效果：
- `world` 固定在初始化位置
- `body` 和 `camera` 会随着你移动相机而实时变化
- Path 显示历史轨迹

### 场景 3：检查相机-IMU 外参（如果不一致）

如果你想检查 VINS 中设定的相机外参是否与实际硬件 TF 一致：
- Fixed Frame 设为 `camera_link`
- 同时显示 `camera_infra1_optical_frame`（RealSense 驱动发布）和 `camera`（VINS 发布）
- 如果两者不重合，说明 VINS 配置文件中的外参与实际不符

---

## 6. 常见问题

### 6.1 "Frame [xxx] does not exist"

- 该 frame 没有节点发布 TF
- 检查对应的节点是否在运行
- 使用 `ros2 run tf2_tools view_frames` 查看完整 TF 树（需要安装 `ros-$ROS_DISTRO-tf2-tools`）

### 6.2 TF 树断裂（不连通）

TF 树必须是连通的。例如 `world` 和 `camera_link` 之间如果没有变换关系，你就不能在一个 RViz 里同时以 `world` 为 Fixed Frame 显示 `camera_link` 下的数据。

**解决方案**：
- 如果需要同时显示，可以手动发布一个静态 TF：
  ```bash
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world camera_link
  ```
- 或者根据实际安装位置填入正确的外参

### 6.3 坐标轴方向理解错误

- **机械坐标系** (`*_frame`)：通常 Z 向上，X 向前，Y 向左（或右），遵循 ROS 标准
- **光学坐标系** (`*_optical_frame`)：**Z 向前（光轴）**，X 向右，Y 向下，遵循图像处理惯例

不要混淆两者！VINS 处理图像时使用的是光学坐标系。

### 6.4 VINS 的 `world` 和 `camera` 在哪？

VINS 只有在完成初始化后才会发布这些 TF。如果相机静止不动或纹理不足，初始化可能失败，此时不会看到 `world`/`body`/`camera`。

---

## 7. 实用命令

```bash
# 查看当前所有 TF 话题
ros2 topic echo /tf

# 查看 TF 树结构图（需要安装 tf2_tools）
ros2 run tf2_tools view_frames

# 查找两个坐标系之间的变换
ros2 run tf2_ros tf2_echo world body

# 手动发布静态 TF（用于测试或补齐外参）
ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll parent_frame child_frame
```
