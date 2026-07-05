# FAST-LIVO2 外参标定手册（Livox MID360 + RealSense D435i）

## 一、前提条件

- 已编译安装 `calib_ros2`（包名），位于 `/home/lingzhilab/ws_livo/install/calib_ros2`
- 已打过 Ceres 2.0 补丁（源码在 `/home/lingzhilab/ws_livo/src/livox_camera_calib`）
- RealSense 已连接，且 `lsusb | grep -i realsense` 能看到设备
- Livox MID360 已连接，且 `ping 192.168.1.135` 通

## 二、录制标定数据

### 1. 启动 Livox（先启）

```bash
source /home/lingzhilab/ws_livox/install/setup.bash
source /home/lingzhilab/vins/install/setup.bash
source /home/lingzhilab/ws_livo/install/setup.bash
export ROS_RSUSB_BACKEND=1
```

使用已准备的 launch 文件：

```bash
ros2 launch /home/lingzhilab/fast_liov/launch/calib/livox_only.launch.py
```

该文件内容：

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_livox = get_package_share_directory("livox_ros_driver2")
    livox_config_path = os.path.join(pkg_livox, "config", "MID360_config.json")
    return LaunchDescription([
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "lvx_file_path": "/home/livox/livox_test.lvx",
                "user_config_path": livox_config_path,
                "cmdline_input_bd_code": "livox0000000001",
            }],
        ),
    ])
```

**等待 15 秒**，让 Livox 稳定。

### 2. 启动 RealSense（后启）

使用已准备的 launch 文件：

```bash
ros2 launch /home/lingzhilab/fast_liov/launch/calib/realsense_only.launch.py
```

该文件内容：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            namespace="camera",
            output="screen",
            parameters=[{
                "camera_name": "camera",
                "camera_namespace": "camera",
                "enable_color": False,
                "enable_depth": False,
                "enable_infra1": True,
                "enable_infra2": False,
                "enable_gyro": False,
                "enable_accel": False,
                "enable_sync": False,
                "initial_reset": False,
                "depth_module.infra_profile": "640,480,30",
                "depth_module.infra1_format": "Y8",
            }],
        ),
    ])
```

**等待 15 秒**，确认图像流正常：

```bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
```

应输出约 30 Hz。

### 3. 录制 bag

在第三个终端：

```bash
ros2 bag record /livox/lidar /camera/camera/infra1/image_rect_raw -o /tmp/calib_bag
```

录制 **10 秒** 即可。确保场景有丰富直线边缘（门框、柜子、墙角、桌椅边缘），传感器**保持静止**。

录完后按 `Ctrl+C` 停止。

## 三、提取图像和点云

```bash
source /home/lingzhilab/ws_livo/install/setup.bash
mkdir -p /tmp/calib_extract/pcd /tmp/calib_extract/img

ros2 launch calib_ros2 bag_to_pcd.launch.py \
  bags_dir:=/tmp/calib_bag \
  pcds_dir:=/tmp/calib_extract/pcd \
  images_dir:=/tmp/calib_extract/img \
  lidar_topic:=/livox/lidar \
  image_topic:=/camera/camera/infra1/image_rect_raw \
  is_custom_msg:=true
```

`bag_to_pcd` 保存完 PCD 和图像后不会自动退出，按 `Ctrl+C` 停止即可。

输出应为：
- `/tmp/calib_extract/img/0.bmp`
- `/tmp/calib_extract/pcd/0.pcd`

## 四、运行 livox_camera_calib

### 1. 修改配置文件

编辑 `/home/lingzhilab/ws_livo/install/calib_ros2/share/calib_ros2/config/calib.yaml`：

```yaml
/**:
  ros__parameters:
    common:
        image_file: "/tmp/calib_extract/img/0.bmp"
        pcd_file: "/tmp/calib_extract/pcd/0.pcd"
        result_file: "/tmp/calib_extract/extrinsic.txt"

    camera:
        camera_matrix: [384.600464,      0.0    ,  316.432343,
                           0.0     , 384.600464,  239.290955,
                           0.0     ,    0.0     ,   1.0     ]
        dist_coeffs: [0.0, 0.0, 0.0, 0.0]

    calib:
        calib_config_file: "/home/lingzhilab/ws_livo/install/calib_ros2/share/calib_ros2/config/config_mid360_d435i.yaml"
        use_rough_calib: true
```

`config_mid360_d435i.yaml` 已经存在，内容如下（初始外参可不动）：

```yaml
%YAML:1.0
PointCloudTopic: "/livox/lidar"
ImageTopic: "/camera/camera/infra1/image_rect_raw"

ExtrinsicMat: !!opencv-matrix
  rows: 4
  cols: 4
  dt: d
  data: [0.0, -1.0,  0.0,  0.0,
         0.0,  0.0, -1.0, -0.15,
         1.0,  0.0,  0.0, -0.095,
         0.0,  0.0,  0.0,  1.0]

Canny.gray_threshold: 10
Canny.len_threshold: 200
Voxel.size: 0.5
Voxel.down_sample_size: 0.02
Plane.min_points_size: 30
Plane.normal_theta_min: 45
Plane.normal_theta_max: 135
Plane.max_size: 8
Ransac.dis_threshold: 0.02
Edge.min_dis_threshold: 0.03
Edge.max_dis_threshold: 0.06
```

### 2. 运行标定

```bash
source /home/lingzhilab/ws_livo/install/setup.bash
ros2 launch calib_ros2 calib.launch.py
```

会弹出 rviz 窗口显示优化过程。优化收敛后终端提示：

```
push enter to publish again
```

按 `Ctrl+C` 退出。

## 五、结果转换

标定输出文件 `/tmp/calib_extract/extrinsic.txt` 格式为 4×4 矩阵：

```
r11,r12,r13,t1
r21,r22,r23,t2
r31,r32,r33,t3
0,0,0,1
```

**注意**：livox_camera_calib 输出的是 **LiDAR → Camera** 变换，可以直接填入 FAST-LIVO2，**不需要转置**。

- `Rcl` = 输出矩阵左上 3×3 旋转部分
- `Pcl` = 输出矩阵右上 3×1 平移部分（即 LiDAR 原点在 Camera 系下的坐标）

示例：若标定输出为

```
-0.00522824,-0.998807,0.048561,0.0290129
0.0186039,-0.0486504,-0.998643,-0.0872397
0.999813,-0.00431771,0.0188361,-0.198226
0,0,0,1
```

则 FAST-LIVO2 推荐配置为（标定 R + 实测 P）：

```yaml
Rcl: [-0.005228, -0.998807,  0.048561,
       0.018604, -0.048650, -0.998643,
       0.999813, -0.004318,  0.018836]
Pcl: [0.029013, -0.150000, -0.095000]
```

如果标定得到的平移和你的实际安装测量偏差较大，说明单帧标定对平移不够鲁棒。实践经验表明：
- 优先信任标定得到的旋转部分（Rcl）
- 平移部分（Pcl）按实际测量值填写效果更好
- 当前推荐组合：`Rcl` 取自 `run_1782119262` 的初始值，`Pcl = [0.029, -0.15, -0.095]`

## 六、填入 FAST-LIVO2

编辑 `/home/lingzhilab/fast_liov/src/FAST-LIVO2-ROS2/config/mid360_d435i.yaml`，替换 `extrin_calib` 部分：

```yaml
extrin_calib:
  extrinsic_T: [-0.011, -0.02329, 0.04412]
  extrinsic_R: [1., 0., 0.,
                0., 1., 0.,
                0., 0., 1.]
  Rcl: [-0.005228, -0.998807,  0.048561,
         0.018604, -0.048650, -0.998643,
         0.999813, -0.004318,  0.018836]
  Pcl: [0.029013, -0.087240, -0.198226]
```

重新编译：

```bash
cd /home/lingzhilab/ws_livo
source install/setup.bash
colcon build --packages-select fast_livo --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/home/lingzhilab/ws_livo/install/sophus
```

## 七、验证

```bash
source /home/lingzhilab/ws_livo/install/setup.bash
export ROS_RSUSB_BACKEND=1
ros2 launch fast_livo mapping_mid360_d435i.launch.py use_rviz:=False
```

观察（75 s 运行参考值）：
- 启动后无持续 IMU/LiDAR sync warning
- 日志中 `VIO ] Append XX new visual map points` 频繁出现
- 推荐组合（标定 R + 实测 P）验证：`SEM 3623 次 / VIO Append 1207 次 / 平均 59.2 点 / 总点数 71461 / USB 0 次断开`
- `/cloud_registered` 点云颜色与 `/rgb_img` 图像对齐

若 VIO Append 点数明显偏低，说明标定平移可能不够准确，可尝试旋转用标定结果、平移用实测值。

## 八、一键脚本

已准备 `/home/lingzhilab/run_calib.sh`，可直接运行：

```bash
/home/lingzhilab/run_calib.sh
```

该脚本会自动完成步骤二、三、四，并打印标定结果。输出矩阵可直接填入 FAST-LIVO2。

脚本使用的 launch 文件位于：
- `/home/lingzhilab/fast_liov/launch/calib/livox_only.launch.py`
- `/home/lingzhilab/fast_liov/launch/calib/realsense_only.launch.py`

脚本会自动将每次标定结果保存到 `/home/lingzhilab/calib_data/run_<timestamp>/`，其中包含：
- `from_bag/extrinsic.txt`：4×4 外参矩阵
- `from_bag/init.png`：初始外参投影图
- `from_bag/rough.png`：粗优化后投影图
- `from_bag/opt.png`：最终优化结果投影图
- `from_bag/opt_iter_*.png`：优化过程中间结果
- `calibration_report.docx`：包含投影图和颜色说明的 Word 报告

投影图说明：
- 背景为 RealSense 红外图像
- 彩色点为按当前外参投影到图像上的 LiDAR 点云
- 颜色使用 jet 彩虹色映射，表示该点到 LiDAR 原点的深度：
  - 深蓝/黑色：最近
  - 青色/蓝绿：较近
  - 绿色/黄色：中等距离
  - 橙色：较远
  - 深红：最远
- 颜色只表示深度，不表示误差。判断效果好坏应看点云边缘是否与图像中的墙边、门框、桌椅轮廓重合。

## 关键提醒

1. **必须先启 Livox，后启 RealSense**，否则 RealSense 容易掉线。
2. **必须设置 `export ROS_RSUSB_BACKEND=1`**，否则 RealSense 在 Jetson 上不稳定。
3. **标定输出直接作为 LiDAR → Camera 填入 FAST-LIVO2，不需要转置**。
4. 如果标定结果与上一次的旋转差几十度，说明场景不好或初始外参偏差大，换到直线边缘更丰富的位置重新录。
5. 单帧标定旋转通常可靠，平移可能受场景深度分布影响。若平移与实测差距大，可旋转用标定结果、平移用实测值。
