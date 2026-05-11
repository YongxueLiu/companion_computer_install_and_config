# FAST-LIO → PX4 外部视觉定位系统

## 系统概述

本系统实现 **Livox MID360 激光雷达 + FAST-LIO2 SLAM → PX4 飞控外部视觉定位** 的完整链路，支持开机自启动。

```
┌──────────────┐    ETH      ┌──────────────┐    DDS     ┌──────────────┐
│  Livox MID360 │ ───────────→ │  Jetson Orin │ ─────────→ │   PX4 V5+    │
│  (LiDAR+IMU)  │   192.168.1.x│   Nano       │  uXRCE-DDS │   (CUAV)     │
└──────────────┘              └──────────────┘            └──────────────┘
                                      │
                                      ▼
                              ┌──────────────┐
                              │  QGroundControl
                              │  (监控/调参)
                              └──────────────┘
```

---

## 一、系统架构与组件

### 1.1 软件栈

| 组件 | 功能 | 启动方式 |
|---|---|---|
| `mid360-network.service` | 配置静态 IP `192.168.1.50/24` | systemd oneshot (root) |
| `mid360-full.service` | 统一 orchestrator | systemd simple (user) |
| `MicroXRCEAgent` | DDS ↔ 串口桥接 | `start_all.sh` |
| `livox_ros_driver2` | 雷达驱动，输出点云+IMU | `start_all.sh` |
| `fast_lio` | SLAM，输出 `/Odometry` | `start_all.sh` |
| `fastlio_px4_bridge` | 坐标转换，输出 `VehicleOdometry` | `start_all.sh` |

### 1.2 话题链路

```
/livox/lidar  ──→  fast_lio  ──→  /Odometry  ──→  fastlio_px4_bridge
/livox/imu    ──→              (nav_msgs/Odometry)       │
                                                          ▼
                                             /fmu/in/vehicle_visual_odometry
                                                          │
                                                          ▼
                                                        PX4 EKF2
                                                          │
                                                          ▼
                                            /fmu/out/vehicle_local_position
```

---

## 二、核心实现：坐标系转换

### 2.1 坐标系定义

| 坐标系 | X | Y | Z | 用途 |
|---|---|---|---|---|
| **FLU** | 前 | 左 | 上 | Livox body（假设） |
| **FRD** | 前 | 右 | 下 | PX4 body |
| **ENU** | 东 | 北 | 上 | FAST-LIO world |
| **NED** | 北 | 东 | 下 | PX4 world |

### 2.2 位置转换

ENU → NED（固定矩阵，与初始化朝向无关）：
```
p_ned = [p_enu_y, p_enu_x, -p_enu_z]
```

### 2.3 姿态四元数转换（链式法则）

```
q_FRD→NED = q_ENU→NED ⊗ q_body→ENU ⊗ q_FRD→body
```

**固定常数四元数**（Hamiltonian w,x,y,z）：
- `q_ENU→NED = [0, √2/2, √2/2, 0]` （ENU→NED 旋转）
- `q_FRD→FLU = [0, 1, 0, 0]` （FRD→FLU，绕 X 轴 180°）

**参数化选择**：
- `body_frame='FLU'`：应用 `q_FRD→FLU` 修正
- `body_frame='FRD'`：跳过 body 修正

### 2.4 可选：偏航角对齐

支持三种模式：

| 模式 | 原理 | 适用场景 |
|---|---|---|
| `none` | 不修正，SLAM 虚拟北 = 初始化机头方向 | 室内纯视觉飞行 |
| `px4_mag` | 订阅 PX4 `vehicle_attitude`，计算 `delta_yaw = yaw_mag - yaw_slam`，锁定后持续修正 | 需要与真实北对齐、融合 GPS |
| `manual` | 固定 `manual_yaw_offset_rad` | 已知固定偏置 |

**对齐公式**：
```
delta_q = [cos(delta_yaw/2), 0, 0, sin(delta_yaw/2)]  # 绕 Z 轴
q_truth = delta_q ⊗ q_FRD→NED_virtual
```

### 2.5 时间同步与 `timestamp`

桥接节点发布 `VehicleOdometry` 时，时间戳不是直接用 ROS2 的 Unix 时间，而是对齐到 **PX4 飞控的内部时钟**：

```python
ros_time_us = int(now.nanoseconds // 1000)
vo.timestamp = ros_time_us + self.timesync_offset
```

**`timesync_offset` 不是参数，不需要手动设置**。它是 PX4 通过 uXRCE-DDS 自动计算并发布到 `/fmu/out/timesync_status` 的 `estimated_offset` 字段，桥接节点订阅后直接使用：

```python
self.timesync_offset = int(msg.estimated_offset)
```

#### 为什么 `estimated_offset` 的绝对值很大？

PX4 使用 **boot-relative 时间**（开机以来的微秒数），而 ROS2 使用 **Unix epoch 时间**（1970 年以来的微秒数）。两者基准相差约 50 多年，所以 `estimated_offset` 绝对值通常在 `-10¹⁵` µs 量级（约 -20000 天）。这**完全正常**。

代码通过 `ros_time + offset` 把 Unix 时间对齐到 PX4 的 boot 时间，EKF2 接收到的就是正确的飞控本地时间戳。

#### 实际数据示例

```bash
ros2 topic echo /fmu/out/timesync_status --qos-reliability best_effort --no-daemon --once
```

```yaml
timestamp: 1778487602608726
remote_timestamp: 1778487602548824
observed_offset: -1778487188495505
estimated_offset: -1778487188551609   # ← 绝对值约 -1.78×10¹⁵ µs，正常
round_trip_time: 7583                # ← 串口往返约 7.6 ms，正常
```

#### 健康判断标准

| 指标 | 正常范围 | 实际值 | 状态 |
|---|---|---|---|
| `estimated_offset` 绝对值 | `-10¹⁵` 量级 | `-1.78×10¹⁵` µs | ✅ 正常 |
| `estimated_offset` 波动 | 稳定，相邻差 < 1 ms | 约 180 µs | ✅ **非常稳定** |
| `round_trip_time` | 3 ~ 20 ms | 3.7 ~ 11 ms | ✅ 正常 |

> ⚠️ 如果 `estimated_offset` 剧烈跳动（每次变化几毫秒以上）或 `round_trip_time` 飙升到几十毫秒，说明串口延迟不稳定，需检查 `/dev/ttyTHS1` 线缆或波特率设置。

---

## 三、文件结构

```
~/ws_livox/
├── src/
│   ├── livox_ros_driver2/          # 官方 Livox ROS2 驱动
│   ├── FAST_LIO_ROS2/              # FAST-LIO2 SLAM
│   ├── fastlio_px4_bridge/         # 本桥接节点
│   │   ├── fastlio_px4_bridge/
│   │   │   └── bridge_node.py      # 主节点（Python）
│   │   ├── launch/
│   │   │   └── bridge.launch.py
│   │   ├── config/
│   │   │   └── bridge_config.yaml  # 参数配置
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── USAGE.md                # 使用说明
│   └── px4_msgs -> ~/ros2_px4/src/px4_msgs  # 软链接
├── start_all.sh                     # 统一启动脚本
├── mid360-full.service              # systemd 统一服务定义
└── install/                          # colcon 安装空间
```

---

## 四、开机自启动配置

### 4.1 双服务架构

由于网络配置需要 **root 权限**，而 ROS2 节点应以 **普通用户** 运行，采用**分离式架构**：

| 服务 | 类型 | 用户 | 作用 |
|---|---|---|---|
| `mid360-network.service` | `oneshot` | root | 配置网卡 IP |
| `mid360-full.service` | `simple` | lingzhilab | 启动 ROS2 全栈 |

**依赖关系**：`mid360-full.service` `After=` `mid360-network.service`

### 4.2 安装命令

```bash
# 1. 创建网络服务
sudo tee /etc/systemd/system/mid360-network.service > /dev/null << 'EOF'
[Unit]
Description=MID360 Network Setup
After=network.target

[Service]
Type=oneshot
ExecStartPre=-/bin/bash -c 'nmcli dev set enP8p1s0 managed no 2>/dev/null || true'
ExecStartPre=/bin/bash -c 'ip addr flush dev enP8p1s0'
ExecStartPre=/bin/bash -c 'ip link set enP8p1s0 down'
ExecStartPre=/bin/bash -c 'sleep 1'
ExecStartPre=/bin/bash -c 'ip link set enP8p1s0 up'
ExecStart=/bin/bash -c 'ip addr add 192.168.1.50/24 dev enP8p1s0'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

# 2. 创建统一服务
sudo cp /home/lingzhilab/ws_livox/mid360-full.service /etc/systemd/system/

# 3. 启用开机自启
sudo systemctl daemon-reload
sudo systemctl enable mid360-network.service mid360-full.service

# 4. 立即启动
sudo systemctl start mid360-network.service
sudo systemctl start mid360-full.service
```

### 4.3 状态查看

```bash
sudo systemctl status mid360-network.service --no-pager
sudo systemctl status mid360-full.service --no-pager
sudo journalctl -u mid360-full.service -f
```

---

## 五、启动脚本 `start_all.sh` 逻辑

```
启动
  │
  ▼
Step 1: MicroXRCEAgent (后台)
  │
  ▼
Step 2: Livox Driver (后台)
  │
  ├── 等待 /livox/lidar 话题就绪（轮询 ros2 topic info）
  │
  ▼
Step 3: FAST-LIO (后台)
  │
  ├── 等待 /Odometry 话题就绪
  │
  ▼
Step 4: fastlio_px4_bridge (后台)
  │
  ├── 等待 /fmu/in/vehicle_visual_odometry 就绪
  │
  ▼
监控循环（每 5 秒检查）
  ├── Agent 存活？
  ├── Driver 存活？
  ├── SLAM 存活？
  └── Bridge 存活？
      └── 任一死亡 → 脚本退出 → systemd 自动重启
```

---

## 六、PX4 飞控参数（QGroundControl）

### 6.1 外部视觉融合（必须）

| 参数 | 值 | 说明 |
|---|---|---|
| `EKF2_EV_CTRL` | `15` | 位掩码：`1(位置)+2(速度)+4(高度)+8(偏航)` |
| `EKF2_HGT_REF` | `3` | 高度参考源 = Vision |
| `EKF2_MAG_TYPE` | `5` | 使用 Vision/EV 偏航，禁用磁罗盘融合（避免室内磁干扰） |
| `EKF2_EV_DELAY` | `20`~`50` | 视觉延迟补偿（ms） |

### 6.2 uXRCE-DDS 串口（必须）

| 参数 | 值 |
|---|---|
| `UXRCE_DDS_CFG` | `TELEM1` 或 `TELEM2` |
| `SER_TEL1_BAUD` | `921600` |
| `SER_TEL2_BAUD` | `921600` |

### 6.3 故障保护（推荐）

| 参数 | 值 |
|---|---|
| `COM_POS_FS_DELAY` | `5` |
| `NAV_RCL_ACT` | `2` (Land) |
| `NAV_DLL_ACT` | `2` (Land) |

---

## 七、已知问题与规避

### 7.1 Fast-DDS 大消息限制

`EstimatorStatusFlags`（104 字节）超过 Fast-DDS 默认 payload（95 字节），`ros2 topic echo` 时可能报错。

**规避**：
```bash
# 禁用 ros2 daemon，使用 --no-daemon 参数
mkdir -p ~/.config/ros2
echo "daemon=False" > ~/.config/ros2/cli.ini

ros2 topic echo /fmu/out/estimator_status_flags --qos-reliability best_effort --no-daemon
```

### 7.2 多服务冲突

旧的分散服务（`mid360-driver.service`、`mid360-slam.service`、`microxrce.service`）必须**全部删除**，否则会和统一服务争抢资源。

---

## 八、验证步骤

### 8.1 地面手持测试

开机自启动完成后（约 30~60 秒）：

```bash
# 1. 确认所有进程在跑
ps aux | grep -E "MicroXRCE|livox|fastlio|bridge_node" | grep -v grep

# 2. 确认话题存在
ros2 topic list | grep -E "Odometry|vehicle_visual_odometry"

# 3. 查看桥接输出
ros2 topic echo /fmu/in/vehicle_visual_odometry --qos-reliability best_effort --once

# 4. 查看 PX4 融合状态（加 --no-daemon 规避 Fast-DDS 限制）
ros2 topic echo /fmu/out/estimator_status_flags --qos-reliability best_effort --no-daemon --once | grep cs_ev
```

期望看到：
- `cs_ev_pos: true`
- `cs_ev_hgt: true`
- `cs_ev_yaw: true`

### 8.2 方向检查

手持无人机，观察 QGC 本地位置：
- 机头朝前 → X（北）增大
- 向右走 → Y（东）增大
- 向上抬 → Z（下）减小

编辑 `config/bridge_config.yaml` 调整参数：

```yaml
body_frame: 'FLU'                # 'FLU' 或 'FRD'
yaw_alignment_mode: 'none'       # 'none' | 'px4_mag' | 'manual'
publish_rate: 20.0               # 发布频率(Hz)。默认 20Hz，太高会导致飞控 CPU 负载过高
```

> **注意 `publish_rate`**：默认 **20 Hz**。PX4 EKF2 处理外部视觉的舒适区间是 10~30 Hz，超过 50 Hz 会导致飞控 CPU 负载过高（QGC 报 `CPU load running high`）。

修改后重新编译：
```bash
cd ~/ws_livox
colcon build --packages-select fastlio_px4_bridge --symlink-install
```

如果方向反了，修改 `body_frame`（`'FLU'` ↔ `'FRD'`），重新编译。

---

## 九、重建命令

如果修改了源码：

```bash
cd ~/ws_livox
source /opt/ros/humble/setup.bash
colcon build --packages-select fastlio_px4_bridge --symlink-install
```

---

## 十、技术要点总结

1. **坐标系转换**：ENU→NED 不是简单的 Z 反向，而是完整的 `R = [0 1 0; 1 0 0; 0 0 -1]` 旋转，对应四元数 `[0, √2/2, √2/2, 0]`。
2. **四元数链式法则**：`q_out = q_ENU→NED ⊗ q_body→ENU ⊗ q_FRD→body`，乘法顺序不可颠倒。
3. **QoS 匹配**：Micro-ROS Agent 使用 `BEST_EFFORT`，订阅 `/fmu/out/*` 时必须显式指定 `--qos-reliability best_effort`。
4. **systemd 权限分离**：网络配置（root）和 ROS2 节点（user）必须分开服务，不可合并到同一个 `User=` 下。
5. **启动时序**：驱动必须在网络就绪后启动，SLAM 必须在驱动就绪后启动，桥接必须在 SLAM 就绪后启动。`start_all.sh` 通过轮询话题或固定延时保证顺序。
