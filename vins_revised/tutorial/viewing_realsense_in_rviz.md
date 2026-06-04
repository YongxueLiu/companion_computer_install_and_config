# 在 RViz 中查看 RealSense D435i 原始数据

本文档介绍如何在不启动 VINS 的情况下，单独使用 RViz 可视化 RealSense ROS2 驱动发布的原始数据流（红外图像、深度点云等）。

---

## 前提条件

- RealSense SDK (`librealsense2`) 已安装
- RealSense ROS2 包 (`realsense-ros`) 已编译
- 相机已连接且识别正常

---

## 1. 启动 RealSense 节点

**必须开启 `enable_color` 和 `pointcloud.enable`**，否则深度点云无法生成。

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_depth:=true \
  enable_color:=true \
  pointcloud.enable:=true
```

> **注意**：`enable_color:=false` 时，PointCloud2 模块虽然能运行，但生成的 `/camera/camera/depth/color/points` 点云数据为空或异常，导致 RViz 中看不到点。

启动成功后，应能看到如下日志：

```
Open profile: stream_type: Infra(1), Format: Y8, Width: 848, Height: 480, FPS: 30
Open profile: stream_type: Infra(2), Format: Y8, Width: 848, Height: 480, FPS: 30
Open profile: stream_type: Depth(0), Format: Z16, Width: 848, Height: 480, FPS: 30
Open profile: stream_type: Color(0), Format: RGB8, Width: 1280, Height: 720, FPS: 30
RealSense Node Is Up!
```

---

## 2. 检查发布的话题

新开一个终端，source 环境后执行：

```bash
ros2 topic list | grep camera
```

确认以下关键话题存在：

| 话题 | 说明 |
|------|------|
| `/camera/camera/infra1/image_rect_raw` | 左目红外图像 |
| `/camera/camera/infra2/image_rect_raw` | 右目红外图像 |
| `/camera/camera/color/image_raw` | RGB 彩色图像 |
| `/camera/camera/depth/image_rect_raw` | 深度图 |
| `/camera/camera/depth/color/points` | 深度点云 (PointCloud2) |

---

## 3. 启动 RViz

```bash
rviz2
```

---

## 4. 配置 Fixed Frame

在左侧 **Displays → Global Options → Fixed Frame** 中填入：

```
camera_link
```

> RealSense 驱动默认会发布 `camera_link` 到各个传感器坐标系（`camera_depth_optical_frame`、`camera_color_optical_frame` 等）的 TF 变换。如果 Fixed Frame 填错，所有显示项都会报错 `Transform [...] does not exist`。

---

## 5. 添加显示项

### 5.1 查看红外图像

1. 点击左下角 **Add** → 选择 **Image**
2. 在 Image 的 **Topic** 中选择 `/camera/camera/infra1/image_rect_raw`
3. 左侧应实时显示左目红外灰度图

### 5.2 查看深度点云（推荐：PointCloud2）

1. 点击 **Add** → 选择 **PointCloud2**
2. 在 PointCloud2 的 **Topic** 中填入：
   ```
   /camera/camera/depth/color/points
   ```
3. **Size (m)** 建议调为 `0.05`（默认 `0.01` 太小，可能看不清）
4. **Style** 可选 `Flat Squares` 或 `Points`
5. 如果点云出现在相机正前方但视角太远，在右侧 **Views** 面板中将 **Target Frame** 改为 `camera_depth_optical_frame`，即可进入点云内部观察

### 5.3 查看深度图（DepthCloud 插件）

如果不方便开启 `pointcloud.enable`，也可以用 RViz 内置的 **DepthCloud** 插件实时渲染点云：

1. 点击 **Add** → 选择 **DepthCloud**
2. **Depth Map Topic**: `/camera/camera/depth/image_rect_raw`
3. **CameraInfo Topic**: `/camera/camera/depth/camera_info`
4. **Fixed Frame**: `camera_link` 或 `camera_depth_optical_frame`

> DepthCloud 是客户端实时从深度图反投影生成的，不依赖驱动的 PointCloud2 输出，但配置稍繁琐。

---

## 6. 常见问题排查

### 6.1 "Fixed Frame [camera_link] does not exist"

- 检查 RealSense 启动参数是否包含 `publish_tf:=false`（默认是 true，不要手动关闭）
- 检查是否有其他节点发布了冲突的 TF
- 临时解决方案：Fixed Frame 改为 `camera_depth_optical_frame`

### 6.2 PointCloud2 Status: Ok，但看不到点

- **最常见原因**：`enable_color:=false`。必须开启 color 才能正常生成 `/camera/camera/depth/color/points`。
- **视角问题**：点云在相机前方 0.3~5 米处，RViz 默认视角在 (10, 10, 10) 远处。把 Target Frame 切到 `camera_depth_optical_frame` 即可。
- **点太小**：Size (m) 设为 0.05 或更大。

### 6.3 "No messages received" / Status: Error

- 确认 RealSense 节点仍在运行
- 用 `ros2 topic hz /camera/camera/depth/color/points` 检查是否有数据流
- 检查话题名是否匹配（v4.58.1 驱动的话题前缀为 `/camera/camera/...`，不是 `/camera/...`）

### 6.4 红外图显示全黑

- 红外图本身是灰度图，且室内场景可能较暗。RViz 的 Image 插件会自动归一化显示，通常能看到内容。
- 如果完全黑屏，检查 `enable_infra1:=true` 是否生效，以及相机镜头盖是否打开。

---

## 7. 完整可用配置总结

| 目的 | 启动命令 | RViz 配置 |
|------|---------|-----------|
| 只看双目红外 | `enable_infra1/2:=true`, 其余 false | Image ×2, Fixed Frame=`camera_link` |
| 看点云（推荐） | `enable_color:=true`, `pointcloud.enable:=true` | PointCloud2=`/camera/camera/depth/color/points`, Fixed Frame=`camera_link` |
| 看深度图反投影 | `enable_depth:=true` | DepthCloud + Depth Map Topic |
| VINS-VO 模式 | `enable_infra1/2:=true`, `enable_depth:=false` | 无需 RViz，直接跑 VINS |
