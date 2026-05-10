# fastlio_px4_bridge 使用说明

## 一、启动顺序（严格按序）

```bash
# 1. 启动雷达驱动
cd ~/ws_livox && source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 2. 等出现 successfully change work mode 后，启动 FAST-LIO
ros2 launch fast_lio mapping.launch.py rviz:=false

# 3. 等 IMU Initial Done 后，启动桥接节点
ros2 launch fastlio_px4_bridge bridge.launch.py
```

---

## 二、坐标转换（自动完成，无需干预）

本节点**始终执行完整 ENU→NED 转换**。

**位置**：
```
p_ned = [p_enu_y, p_enu_x, -p_enu_z]
```

**姿态**（链式法则）：
```
q_FRD->NED = q_ENU->NED ⊗ q_body->ENU ⊗ q_FRD->body
```

其中 `q_ENU->NED = [0, √2/2, √2/2, 0]` 为固定常数。

---

## 三、参数配置

编辑 `~/ws_livox/src/fastlio_px4_bridge/config/bridge_config.yaml`：

```yaml
body_frame: 'FLU'                # 'FLU' 或 'FRD'
yaw_alignment_mode: 'none'       # 'none' | 'px4_mag' | 'manual'
manual_yaw_offset_rad: 0.0       # manual 模式用，单位弧度
```

### body_frame
- `'FLU'`：Livox IMU 是 Forward-Left-Up（默认，应用 R_x(180) 修正）
- `'FRD'`：Livox IMU 已经是 Forward-Right-Down（与 PX4 一致，不修正）

### yaw_alignment_mode
| 模式 | 作用 |
|---|---|
| `'none'` | 不修正偏航。SLAM 虚拟北 = 初始化时机头方向。 |
| `'px4_mag'` | **自动对齐真实北**：订阅 PX4 `vehicle_attitude`，计算 `delta_yaw = yaw_磁力计 - yaw_SLAM`，锁定后持续修正。 |
| `'manual'` | 固定偏置角，使用 `manual_yaw_offset_rad`。 |

**修改后重新编译**：
```bash
cd ~/ws_livox
colcon build --packages-select fastlio_px4_bridge --symlink-install
```

---

## 四、验证

### 查看桥接输出
```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry --qos-reliability best_effort
```

### 查看 PX4 融合状态
```bash
ros2 topic echo /fmu/out/estimator_status_flags --qos-reliability best_effort | grep cs_ev
```
融合成功标志：`cs_ev_pos: true`, `cs_ev_hgt: true`, `cs_ev_yaw: true`

### 查看偏航锁定（px4_mag 模式）
桥接日志会打印：
```
Yaw offset locked: 90.50 deg (PX4=91.20, SLAM=0.70)
```

---

## 五、地面手持方向检查

| 动作 | QGC 期望变化 |
|---|---|
| 机头朝前走 | X（北）增大 |
| 向右走 | Y（东）增大 |
| 向上抬 | Z（下）减小，高度增大 |
| 左倾 | Roll 正 |
| 抬头 | Pitch 正 |

方向反了 → 改 `body_frame`（`'FLU'` ↔ `'FRD'`）后重新编译。

---

## 六、PX4 参数（QGC）

| 参数 | 值 |
|---|---|
| `EKF2_EV_CTRL` | `15` |
| `EKF2_HGT_REF` | `3` (Vision) |
| `EKF2_GPS_CTRL` | `0`（室内）|

---

## 七、常见问题

**Q: `ros2 topic echo` 收不到数据**
> Micro-ROS 话题 QoS 是 `BEST_EFFORT`，必须加 `--qos-reliability best_effort`。

**Q: 偏航不对（机头朝前但 QGC 显示朝东）**
> 设 `yaw_alignment_mode: 'px4_mag'`，让磁力计自动修正。

**Q: 桥接报 `Position jump`**
> FAST-LIO 丢定位。`reset_counter` 自动递增通知 PX4 重置，同时 `yaw_offset` 自动重新锁定。
