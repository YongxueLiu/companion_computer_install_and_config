# PX4 地面站（QGC）参数设置指南

适用于 **FAST-LIO → PX4 外部视觉定位** 场景。

---

## 一、外部视觉核心参数（必须设置）

| 参数 | 值 | 说明 |
|---|---|---|
| `EKF2_EV_CTRL` | `15` | 启用 EV 控制位掩码：`1(位置) + 2(速度) + 4(高度) + 8(偏航)` = 15 |
| `EKF2_HGT_REF` | `3` | 高度参考源设为 **Vision**（外部视觉） |
| `EKF2_MAG_TYPE` | `5` | 使用 **Vision/EV 偏航**，禁用磁罗盘融合（避免室内磁干扰） |
| `EKF2_EV_DELAY` | `20` ~ `50` | 外部视觉延迟（ms）。FAST-LIO 通常 <50ms，设 20~30 即可。 |
| `EKF2_EV_NOISE_MD` | `0` | 使用桥接节点提供的 covariance（手动噪声模型） |
| `EKF2_EV_GATE` | `5.0` | EV 卡方检验门限。可适当放宽到 5.0~10.0。 |

> **EKF2_EV_CTRL 说明**：
> - bit 0 (1): 水平位置融合
> - bit 1 (2): 水平速度融合
> - bit 2 (4): 高度融合
> - bit 3 (8): 偏航融合
> - `1+2+4+8 = 15`

---

## 二、GPS 相关（室内/室外策略）

### 纯室内（无 GPS）
| 参数 | 值 | 说明 |
|---|---|---|
| `EKF2_GPS_CTRL` | `0` | 完全禁用 GPS 融合 |

### 室外（GPS + 视觉融合）
| 参数 | 值 | 说明 |
|---|---|---|
| `EKF2_GPS_CTRL` | `7` | 启用 GPS 位置+速度+高度 |
| `EKF2_HGT_REF` | `1` | 高度源改为 **Baro**（GPS 气压计），避免 GPS 和 Vision 高度冲突 |
| `EKF2_EV_CTRL` | `11` | 关闭 EV 高度控制：`1+2+8 = 11`，只留位置+偏航 |

> 注意：GPS 和视觉同时开启时，EKF2 会自动加权融合。但高度源只能选一个，建议室外用 Baro/GPS，室内用 Vision。

---

## 三、故障保护（Failsafe）

| 参数 | 推荐值 | 说明 |
|---|---|---|
| `COM_POS_FS_DELAY` | `5` | 位置丢失后延迟 5 秒才触发故障保护 |
| `COM_POS_LOW_EPH` | `-1` | 禁用水平位置精度检查（EV 没有 Eph 概念） |
| `COM_POS_LOW_EPV` | `-1` | 禁用垂直位置精度检查 |
| `COM_FAIL_ACT_T` | `5` | 动作超时 5 秒 |
| `NAV_RCL_ACT` | `2` | 遥控丢失后 **Land**（降落） |
| `NAV_DLL_ACT` | `2` | 数据链丢失后 **Land** |

---

## 四、Micro-XRCE-DDS / 串口参数

确保飞控的 uXRCE-DDS 客户端已启用：

| 参数 | 值 | 说明 |
|---|---|---|
| `UXRCE_DDS_CFG` | `TELEM1` 或 `TELEM2` | 启用 DDS 的串口端口 |
| `SER_TEL1_BAUD` | `921600` | 如果接 TELEM1，波特率设为 921600 |
| `SER_TEL2_BAUD` | `921600` | 如果接 TELEM2，波特率设为 921600 |

> 注意：`UXRCE_DDS_CFG` 不能是 `Disabled`。如果对应串口被 MAVLink 占用（`MAV_0_CONFIG` 或 `MAV_1_CONFIG`），需要先释放。

---

## 五、磁罗盘（配合 px4_mag 偏航对齐）

如果使用 `yaw_alignment_mode: 'px4_mag'`，确保磁罗盘工作正常：

| 参数 | 值 | 说明 |
|---|---|---|
| `CAL_MAG0_EN` | `1` | 启用主磁罗盘 |
| `EKF2_MAG_TYPE` | `0` | 自动选择（ fusion 模式） |
| `EKF2_MAG_CHECK` | `0` | 禁用磁罗盘强度检查（室内可能有干扰） |

> 如果室内磁干扰大，建议 `yaw_alignment_mode` 用 `'none'` 或 `'manual'`，不要依赖 `px4_mag`。

---

## 六、安全与解锁

| 参数 | 值 | 说明 |
|---|---|---|
| `COM_PREARM_MODE` | `0` | 禁用预解锁检查（调试用，正式飞行建议保留） |
| `CBRK_SUPPLY_CHK` | `894281` | 禁用电源检查（如果使用非标准供电） |
| `CBRK_USB_CHK` | `197848` | 禁用 USB 连接检查（调试时防止无法解锁） |

> ⚠️ 以上三项仅用于**地面调试验证**，正式飞行前建议恢复默认值。

---

## 七、快速检查清单（解锁前）

在 QGC 中确认：

- [ ] `EKF2_EV_CTRL` = `15`
- [ ] `EKF2_HGT_REF` = `3` (Vision)
- [ ] `EKF2_MAG_TYPE` = `5`
- [ ] `UXRCE_DDS_CFG` = `TELEM1` 或 `TELEM2`（非 Disabled）
- [ ] 对应串口波特率 = `921600`
- [ ] 桥接节点已启动，`/fmu/in/vehicle_visual_odometry` 有数据
- [ ] `ros2 topic echo /fmu/out/estimator_status_flags` 显示 `cs_ev_pos: true`
- [ ] 地面手持测试：前后左右移动时 QGC 本地位置同步变化
- [ ] 故障保护参数已设置（`COM_POS_FS_DELAY` 等）

---

## 八、参考命令

```bash
# 查看 EKF2 是否融合了外部视觉
ros2 topic echo /fmu/out/estimator_status_flags --qos-reliability best_effort | grep cs_ev

# 查看飞控当前姿态（真实北向）
ros2 topic echo /fmu/out/vehicle_attitude --qos-reliability best_effort

# 查看本地位置估计
ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort
```
