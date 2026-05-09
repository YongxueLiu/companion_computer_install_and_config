# MID360 + FAST_LIO 机载里程计 完整使用说明

> **目标平台**：Jetson Orin Nano / Ubuntu 22.04 / ROS2 Humble  
> **功能**：通过 Livox MID360 激光雷达输出实时里程计（`/Odometry`）与点云地图  
> **适用场景**：无人机/无人车机载定位、SLAM 建图

---

## 目录

- [一、文件清单](#一文件清单)
- [二、硬件连接](#二硬件连接)
- [三、软件安装](#三软件安装)
- [四、网络配置](#四网络配置)
- [五、雷达 IP 配置核对](#五雷达-ip-配置核对)
- [六、日常使用](#六日常使用)
- [七、获取里程计数据](#七获取里程计数据)
- [八、保存点云地图](#八保存点云地图)
- [九、机载场景建议](#九机载场景建议)
- [十、故障排查](#十故障排查)

---

## 一、文件清单

所有脚本位于 `~/mid_360_install/` 目录下：

| 脚本文件 | 作用 |
|---------|------|
| `install_mid360_slam.sh` | 安装 Livox-SDK2 + livox_ros_driver2 + FAST_LIO_ROS2 |
| `setup_mid360_network.sh` | 手动配置 MID360 静态 IP |
| `install_mid360_service.sh` | 将网络配置注册为 systemd 开机自启 |

---

## 二、硬件连接

```
MID360 雷达  ←——网线——→  Jetson Orin Nano 有线网口
                
要求：
- 电脑网口与雷达 IP 必须在同一网段（如 192.168.1.x）
- 建议雷达直连电脑，不经过路由器
- 雷达默认 IP 通常为 192.168.1.135（具体看机身标签）
```

查看 Jetson 网口名：

```bash
ip addr
# 有线网口通常为 eth0 或 enpxxx
# 无线网口通常为 wlxxx（不要选这个）
```

---

## 三、软件安装

只需执行一次：

```bash
bash ~/mid_360_install/install_mid360_slam.sh
```

脚本会严格按照官方 README 完成以下安装：

1. **Livox-SDK2** → `cmake .. && make -j && sudo make install`
2. **livox_ros_driver2** → `./build.sh humble`
3. **FAST_LIO_ROS2** → `git clone --recursive` → `rosdep install` → `colcon build --symlink-install`

安装完成后，环境变量会自动写入 `~/.bashrc`。

---

## 四、网络配置

### 4.1 手动配置（调试时使用）

```bash
# 自动检测网口，使用默认 IP 192.168.1.50
bash ~/mid_360_install/setup_mid360_network.sh

# 或手动指定网口和 IP
bash ~/mid_360_install/setup_mid360_network.sh eth0 192.168.1.50
```

### 4.2 开机自启（机载推荐）

```bash
# 自动检测网口并注册 systemd 开机自启
bash ~/mid_360_install/install_mid360_service.sh

# 或手动指定
bash ~/mid_360_install/install_mid360_service.sh eth0 192.168.1.50
```

注册完成后，每次开机自动配置网络。常用命令：

```bash
sudo systemctl status mid360-network.service   # 查看状态
sudo systemctl start mid360-network.service    # 手动启动
sudo systemctl stop mid360-network.service     # 手动停止
sudo systemctl disable mid360-network.service  # 禁用自启
```

---

## 五、雷达 IP 配置核对

**这一步必须做，否则雷达连不上。**

打开配置文件：

```bash
nano ~/ws_livox/src/livox_ros_driver2/config/MID360_config.json
```

确认以下字段：

```json
{
  "host_net_info" : {
    "cmd_data_ip" : "192.168.1.50",   // <-- 必须和电脑网口 IP 完全一致
    ...
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.135"           // <-- MID360 雷达实际 IP（看机身标签）
    }
  ]
}
```

**修改后需要重新编译：**

```bash
cd ~/ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
```

---

## 六、日常使用

### 6.1 手动启动（调试用）

开 **3 个终端**，每个都先执行：

```bash
source ~/ws_livox/install/setup.sh
```

**终端 1 — 启动雷达驱动：**

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**终端 2 — 启动 FAST_LIO 建图 + 里程计：**

```bash
# 有显示器（自动打开 RViz）
ros2 launch fast_lio mapping.launch.py

# 无显示器（机载无头模式，关闭 RViz）
ros2 launch fast_lio mapping.launch.py rviz:=false
```

**终端 3 — 查看实时里程计：**

```bash
ros2 topic echo /Odometry
```

### 6.2 开机全自启（机载无头模式）

如果希望 Jetson 上电后全自动运行，不需要 SSH 手动开终端：

**Step 1 — 创建雷达驱动自启服务：**

```bash
sudo tee /etc/systemd/system/mid360-driver.service > /dev/null << 'EOF'
[Unit]
Description=MID360 LiDAR Driver
After=mid360-network.service
Wants=mid360-network.service

[Service]
Type=simple
User=lyx
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.sh && source /home/lyx/ws_livox/install/setup.sh && ros2 launch livox_ros_driver2 msg_MID360_launch.py'
Restart=always

[Install]
WantedBy=multi-user.target
EOF
```

**Step 2 — 创建 FAST_LIO 自启服务：**

```bash
sudo tee /etc/systemd/system/mid360-slam.service > /dev/null << 'EOF'
[Unit]
Description=FAST_LIO SLAM
After=mid360-driver.service
Wants=mid360-driver.service

[Service]
Type=simple
User=lyx
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.sh && source /home/lyx/ws_livox/install/setup.sh && sleep 15 && ros2 launch fast_lio mapping.launch.py rviz:=false'
Restart=always

[Install]
WantedBy=multi-user.target
EOF
```

**Step 3 — 启用并启动：**

```bash
sudo systemctl daemon-reload
sudo systemctl enable mid360-driver.service
sudo systemctl enable mid360-slam.service

# 立即启动测试
sudo systemctl start mid360-driver.service
sudo systemctl start mid360-slam.service
```

**查看状态：**

```bash
sudo systemctl status mid360-driver.service --no-pager
sudo systemctl status mid360-slam.service --no-pager
```

---

## 七、获取里程计数据

### 7.1 命令行实时查看

```bash
ros2 topic echo /Odometry

# 只看一次
ros2 topic echo /Odometry --once
```

### 7.2 话题信息

```bash
ros2 topic info /Odometry
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `/Odometry` | `nav_msgs/Odometry` | 实时位姿（位置 + 姿态四元数） |
| `/path` | `nav_msgs/Path` | 运动轨迹 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 实时点云地图 |

### 7.3 Python 订阅示例

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.sub = self.create_subscription(
            Odometry, '/Odometry', self.callback, 10)

    def callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.get_logger().info(
            f"Pos: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

def main():
    rclpy.init()
    node = OdomListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 7.4 录制数据包

```bash
# 录制里程计 + 点云
ros2 bag record /Odometry /cloud_registered -o ~/mid360_bag

# 只录里程计
ros2 bag record /Odometry -o ~/odom_bag
```

---

## 八、保存点云地图

### 8.1 修改配置

```bash
nano ~/ws_livox/src/FAST_LIO_ROS2/config/mid360.yaml
```

找到并修改：

```yaml
pcd_save:
    pcd_save_en: true       # false -> true
    interval: -1            # -1 表示保存为一个文件
```

### 8.2 重新编译

```bash
cd ~/ws_livox
colcon build --packages-select fast_lio --symlink-install
```

### 8.3 获取地图

运行 FAST_LIO 后，地图文件 `test.pcd` 保存在**你启动终端时的当前目录**下（通常是 `~`）。

查看地图：

```bash
pcl_viewer ~/test.pcd
```

---

## 九、机载场景建议

| 项目 | 操作建议 |
|------|---------|
| **功耗模式** | `sudo nvpmodel -m 0 && sudo jetson_clocks`（开启最大性能） |
| **无显示器** | FAST_LIO 启动时加 `rviz:=false` |
| **网络** | 使用 systemd 自启，避免每次手动配置 |
| **外参标定** | 机载必须精确标定 IMU-LiDAR 外参，修改 `mid360.yaml` 中的 `extrinsic_T` 和 `extrinsic_R` |
| **日志记录** | 飞行前启动 `ros2 bag record`，便于事后分析 |
| **TF 发布** | FAST_LIO 默认不发布 `/tf`，如需可用 `static_transform_publisher` 或修改源码 |

---

## 十、故障排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| `ros2 launch` 报错找不到雷达 | 网口 IP 配置错误或雷达未通电 | 检查 `ip addr` 和 `MID360_config.json` 中的 IP |
| 雷达驱动启动但无点云数据 | 雷达 IP 不对 | 确认雷达机身标签上的 IP，修改 json 后重新编译 |
| FAST_LIO 不输出 `/Odometry` | 话题名不匹配 | 检查 `mid360.yaml` 中 `lid_topic: /livox/lidar` |
| 编译报错缺库 | 系统依赖未安装 | `sudo apt install libapr1-dev libboost-all-dev libpcl-dev libeigen3-dev` |
| 重启后连不上雷达 | 网络配置丢失 | `sudo systemctl status mid360-network.service` 检查服务状态 |
| 里程计漂移严重 | 外参不准或运动过快 | 精确标定 `extrinsic_T` / `extrinsic_R`，开启 `extrinsic_est_en: true` |
| 编译 livox_ros_driver2 报找不到 sdk | Livox-SDK2 未安装 | 重新运行 `install_mid360_slam.sh` 的前半部分 |

---

## 附录：系统架构图

```
┌─────────────────────────────────────────────────────────┐
│                  Jetson Orin Nano                       │
│  ┌─────────────┐    ┌──────────────┐    ┌───────────┐  │
│  │ systemd     │───→│ livox_driver │───→│ FAST_LIO  │  │
│  │ 网络自启    │    │ /livox/lidar │    │ /Odometry │  │
│  └─────────────┘    └──────────────┘    └───────────┘  │
│         ↑                    ↑                 ↑        │
│    enp108s0/eth0       自定义点云          里程计输出    │
│         │                                          │    │
│    静态IP: 192.168.1.50                      ROS2 Topic │
└─────────┼──────────────────────────────────────────┘    │
          │                                               │
    ┌─────┴─────┐                                         │
    │  MID360   │  IP: 192.168.1.135（默认）               │
    │  激光雷达  │                                         │
    └───────────┘                                         │
```

---

*文档版本: 2025-05-03*  
*对应仓库: https://github.com/Livox-SDK/Livox-SDK2, https://github.com/Livox-SDK/livox_ros_driver2, https://github.com/Ericsii/FAST_LIO_ROS2*
