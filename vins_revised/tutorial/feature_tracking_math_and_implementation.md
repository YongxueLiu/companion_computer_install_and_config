# VINS-Fusion 特征点跟踪（Feature Tracking）数学模型与实现详解

本文档深入解析 VINS-Fusion 中 `FeatureTracker` 类的数学原理和代码实现，涵盖角点检测、LK 光流跟踪、外点剔除、双目匹配等核心算法。

---

## 1. 概述

`FeatureTracker` 是 VINS-Fusion 的**前端视觉模块**，负责从图像中提取并跟踪特征点。它的输出是一组带 ID 的特征点，供后端（Estimator）进行位姿估计和地图构建。

### 1.1 输入输出

| 输入 | 说明 |
|------|------|
| `_img` (左目图像) | 当前帧的灰度图 (`CV_8UC1`) |
| `_img1` (右目图像, 可选) | 当前帧右目灰度图，双目模式下使用 |
| `_cur_time` | 当前图像的时间戳 |

| 输出 | 说明 |
|------|------|
| `featureFrame` | `map<int, vector<pair<int, Matrix<double,7,1>>>>` |
| | 键：`feature_id` |
| | 值：`[camera_id, [x, y, z, u, v, vx, vy]]` |

### 1.2 核心流程

```
图像输入
  ├─→ 时序光流跟踪 (prev → cur)
  ├─→ 反向验证 (cur → prev)
  ├─→ F 矩阵 RANSAC 外点剔除
  ├─→ 角点检测 (补充丢失的特征点)
  ├─→ 双目匹配 (左目 → 右目, 可选)
  └─→ 坐标归一化 + 速度计算
```

---

## 2. 角点检测：Shi-Tomasi 算法

### 2.1 数学原理

Shi-Tomasi 角点检测的核心是计算图像每个像素邻域内的**梯度自相关矩阵**（Structure Tensor）：

$$M = \sum_{(x,y) \in W} w(x,y) \begin{bmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{bmatrix}$$

其中：
- $I_x, I_y$ 是图像在 $x$ 和 $y$ 方向的梯度
- $W$ 是像素邻域窗口
- $w(x,y)$ 是窗口权重（通常用高斯权重）

计算 $M$ 的**两个特征值** $\lambda_1, \lambda_2$：

| 区域类型 | $\lambda_1$ | $\lambda_2$ | 特征 |
|---------|------------|------------|------|
| 平坦区域 | 小 | 小 | 梯度接近 0 |
| 边缘 | 大 | 小 | 沿边缘方向梯度变化小 |
| **角点** | **大** | **大** | **两个方向梯度变化都大** |

**Shi-Tomasi 角点响应函数**：

$$R = \min(\lambda_1, \lambda_2)$$

只有当 $\min(\lambda_1, \lambda_2) > \text{threshold}$ 时，才被认为是角点。

> **与 Harris 角点的区别**：Harris 使用 $\det(M) - k \cdot \text{trace}^2(M)$，Shi-Tomasi 直接用较小的特征值，更稳定。

### 2.2 代码实现

```cpp
cv::goodFeaturesToTrack(cur_img, n_pts, 
    MAX_CNT - cur_pts.size(),  // 最大角点数量
    0.01,                        // 质量等级 (qualityLevel)
    MIN_DIST,                    // 最小欧氏距离
    mask);                       // 掩码，限制检测区域
```

**参数说明**：

| 参数 | 值 | 含义 |
|------|---|------|
| `MAX_CNT` | 150 | 每帧最多保留 150 个特征点 |
| `qualityLevel` | 0.01 | 角点质量阈值，相对于最大响应值的比例 |
| `MIN_DIST` | 30 | 两个角点之间的最小像素距离，避免过于密集 |
| `mask` | CV_8UC1 | 限制在特定区域检测（见 2.3 节） |

### 2.3 非极大值抑制：setMask()

`goodFeaturesToTrack` 返回的角点可能过于密集。`setMask()` 实现了一个**带优先级的非极大值抑制**：

```cpp
void FeatureTracker::setMask()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));  // 初始全白

    // 1. 按跟踪时长排序（跟踪越久越优先保留）
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), 
        [](const auto &a, const auto &b) { return a.first > b.first; });

    // 2. 保留特征点，并在其周围画黑圆（禁止新角点进入）
    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);  // 半径 MIN_DIST 的黑圆
        }
    }
}
```

**算法逻辑**：
1. 按 `track_cnt`（跟踪成功次数）降序排序，老特征点优先
2. 从得分最高的特征点开始，在 mask 上以该点为中心画一个半径为 `MIN_DIST`（30px）的黑色圆
3. 后续特征点如果落在黑色圆内，则被丢弃

这样保证了特征点在图像上**均匀分布**，且**长期跟踪的特征点优先保留**。

---

## 3. 光流跟踪：Lucas-Kanade 算法

### 3.1 数学原理

**光流假设**：相邻帧之间，同一个特征点的亮度不变：

$$I(x, y, t) = I(x + dx, y + dy, t + dt)$$

对 $I(x + dx, y + dy, t + dt)$ 做一阶泰勒展开：

$$I(x + dx, y + dy, t + dt) \approx I(x, y, t) + I_x dx + I_y dy + I_t dt$$

由亮度不变假设 $I(x + dx, y + dy, t + dt) = I(x, y, t)$，得：

$$I_x \frac{dx}{dt} + I_y \frac{dy}{dt} + I_t = 0$$

即：

$$I_x u + I_y v + I_t = 0$$

这是**光流约束方程**，有两个未知数 $(u, v)$，只有一个方程，属于**欠定问题**（孔径问题）。

**Lucas-Kanade 解法**：假设特征点邻域内的所有像素具有相同的光流 $(u, v)$，建立超定方程组，用最小二乘法求解：

$$\begin{bmatrix} \sum I_x^2 & \sum I_x I_y \\ \sum I_x I_y & \sum I_y^2 \end{bmatrix} \begin{bmatrix} u \\ v \end{bmatrix} = -\begin{bmatrix} \sum I_x I_t \\ \sum I_y I_t \end{bmatrix}$$

### 3.2 金字塔 LK 光流

**问题**：当相机运动较大时，特征点在两帧之间的位移可能超过 LK 算法的收敛范围（通常只有几个像素）。

**解决**：使用**图像金字塔（Image Pyramid）**，从 coarse 到 fine 逐层估计：

```
Level 3 (最小): 先在低分辨率图像上估计大致位移
      ↓
Level 2: 在中等分辨率上 refine
      ↓
Level 1 (最大): 在原始分辨率上得到精确位移
```

这样即使原始图像上的位移有 20~30 像素，在金字塔顶层可能只有 3~4 像素，LK 算法可以稳定收敛。

### 3.3 代码实现

```cpp
// 前向光流：prev_img → cur_img
cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, 
    cv::Size(21, 21),  // 窗口大小
    3,                  // 金字塔层数
    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
```

| 参数 | 值 | 含义 |
|------|---|------|
| `winSize` | 21×21 | LK 算法的局部窗口大小 |
| `maxLevel` | 3 | 金字塔层数（0 表示只用原图） |
| `TermCriteria` | COUNT(30) + EPS(0.01) | 最大迭代 30 次，或误差 < 0.01 |

**输出**：
- `status[i] = 1`：跟踪成功
- `status[i] = 0`：跟踪失败（超出图像、光流未收敛等）
- `err[i]`：跟踪误差（像素级）

---

## 4. 反向验证（Forward-Backward Check）

### 4.1 为什么需要反向验证？

LK 光流是**局部搜索**，可能陷入局部最优。例如：
- 场景中有重复的纹理（如砖墙、网格）
- 光流可能错误地匹配到相邻的相似图案上

### 4.2 数学原理

**正向光流**：$p_{t} \xrightarrow{\text{LK}} p_{t+1}$

**反向光流**：$p_{t+1} \xrightarrow{\text{LK}} \hat{p}_{t}$

**一致性检验**：如果正向和反向都正确，应该满足：

$$\|p_t - \hat{p}_t\| < \epsilon$$

VINS 中 $\epsilon = 0.5$ 像素。

### 4.3 代码实现

```cpp
// 正向光流
 cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, ...);

// 反向光流
vector<cv::Point2f> reverse_pts = prev_pts;
cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, ...);

// 一致性检验
for(size_t i = 0; i < status.size(); i++)
{
    if(status[i] && reverse_status[i] && 
       distance(prev_pts[i], reverse_pts[i]) <= 0.5)
        status[i] = 1;   // ✅ 双向一致，保留
    else
        status[i] = 0;   // ❌ 不一致，丢弃
}
```

**效果**：剔除约 5%~15% 的错误匹配，显著提高跟踪精度。

---

## 5. 外点剔除：F 矩阵 RANSAC

### 5.1 为什么需要 RANSAC？

即使经过反向验证，仍可能有少量外点（outliers）残留。这些外点会严重影响后端优化的精度。

### 5.2 数学原理：对极几何

对于两个视角下的同一对匹配点 $p = (u, v, 1)^T$ 和 $p' = (u', v', 1)^T$，它们满足**对极约束**：

$$p'^T F p = 0$$

其中 $F$ 是 $3 \times 3$ 的**基础矩阵（Fundamental Matrix）**。

**RANSAC 流程**：
1. 随机抽取 8 对匹配点
2. 用 8 点法求解 $F$ 矩阵
3. 计算所有点对的 Sampson 距离
4. 统计内点（距离 < 阈值）数量
5. 重复 1~4 步，保留内点最多的 $F$ 矩阵
6. 最终用所有内点重新优化 $F$

### 5.3 代码实现

```cpp
void FeatureTracker::rejectWithF()
{
    if (cur_pts.size() >= 8)
    {
        // 1. 将像素坐标转换为归一化坐标（用虚拟焦距 FOCAL_LENGTH）
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            m_camera[0]->liftProjective(
                Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
            // ... 对 prev_pts 同样处理
        }

        // 2. RANSAC 求解 F 矩阵
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, 
            cv::FM_RANSAC,    // RANSAC 方法
            F_THRESHOLD,      // 阈值：1.0 像素
            0.99,             // 置信度：99%
            status);          // 输出：内点标记

        // 3. 只保留内点
        reduceVector(cur_pts, status);
        reduceVector(prev_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
    }
}
```

**参数**：
- `F_THRESHOLD = 1.0`：Sampson 距离阈值（像素）
- 置信度 `0.99`：RANSAC 迭代次数自动计算，99% 概率找到正确解

---

## 6. 双目匹配

### 6.1 原理

在双目模式下，VINS 不仅要做**时序跟踪**（帧间跟踪），还要做**空间匹配**（左右目匹配）。

对于左目检测到的每个特征点 $p_l$，在右目图像上找到对应的 $p_r$。

### 6.2 为什么能用光流做双目匹配？

D435i 的左右目是**水平对齐**的（经过 stereo rectification），同一个 3D 点在左右目上的 $y$ 坐标相同，只有 $x$ 坐标不同（视差）。

因此，左右目匹配可以看作是一个**一维的光流问题**，LK 算法可以胜任。

### 6.3 代码实现

```cpp
// 1. 左目 → 右目光流
cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, 
    cv::Size(21, 21), 3);

// 2. 反向验证：右目 → 左目
if(FLOW_BACK)
{
    cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, 
        statusRightLeft, err, cv::Size(21, 21), 3);

    for(size_t i = 0; i < status.size(); i++)
    {
        if(status[i] && statusRightLeft[i] && 
           inBorder(cur_right_pts[i]) && 
           distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
            status[i] = 1;
        else
            status[i] = 0;
    }
}

// 3. 只保留成功匹配的点
ids_right = ids;
reduceVector(cur_right_pts, status);
reduceVector(ids_right, status);
```

**注意**：左右目匹配失败的点**不会从 `cur_pts` 中删除**（注释掉的代码），只会从 `cur_right_pts` 中删除。这意味着即使右目匹配失败，左目的特征点仍然会被保留用于单目跟踪。

---

## 7. 坐标归一化与速度计算

### 7.1 像素坐标 → 归一化坐标

特征点的像素坐标 $(u, v)$ 需要通过相机内参转换为归一化平面坐标 $(x, y, 1)$：

```cpp
vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);  // 像素坐标
        Eigen::Vector3d b;
        cam->liftProjective(a, b);               // 去畸变 + 归一化
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}
```

`liftProjective` 内部执行：
- 如果是 **PINHOLE** 模型：$x = (u - c_x) / f_x, \quad y = (v - c_y) / f_y$
- 如果是 **MEI**（鱼眼）模型：使用迭代法去畸变

### 7.2 特征点速度

VINS 还计算了特征点在**归一化坐标系**下的运动速度，用于时间同步补偿：

```cpp
vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, 
    vector<cv::Point2f> &pts, 
    map<int, cv::Point2f> &cur_id_pts, 
    map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++)
        cur_id_pts.insert(make_pair(ids[i], pts[i]));

    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;  // 时间差
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            auto it = prev_id_pts.find(ids[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    else
        pts_velocity.push_back(cv::Point2f(0, 0));

    return pts_velocity;
}
```

**用途**：后端估计相机和 IMU 之间的时间偏移 `td` 时，用特征点速度来补偿图像和 IMU 的时间不同步。

---

## 8. 可视化：drawTrack()

```cpp
void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts,
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    // 左右图拼接
    if (!imRight.empty() && stereo_cam)
        cv::hconcat(imLeft, imRight, imTrack);
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, cv::COLOR_GRAY2RGB);

    // 左目特征点：颜色表示跟踪时长
    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, 
            cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }

    // 右目特征点：绿色
    if (!imRight.empty() && stereo_cam)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;  // 右目图像在拼接后的右半部分
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
        }
    }
}
```

**颜色编码**：
- **蓝色** (`len = 0`)：新检测到的特征点，刚跟踪了 1 帧
- **紫色/品红** (`len = 0.5`)：跟踪了约 10 帧
- **红色** (`len = 1`)：跟踪了 20+ 帧的老特征点

---

## 9. 参数配置

`config/realsense_d435i/realsense_stereo_imu_config.yaml` 中的前端参数：

```yaml
max_cnt: 150            # 每帧最大特征点数量
min_dist: 30            # 特征点之间的最小像素距离
freq: 10                # 跟踪结果发布频率 (Hz)
F_threshold: 1.0        # F 矩阵 RANSAC 阈值 (像素)
show_track: 1           # 是否发布跟踪可视化图像
flow_back: 1            # 是否启用反向光流验证
```

---

## 10. 总结

VINS-Fusion 的特征点跟踪模块是一个精心设计的**多级筛选系统**：

| 阶段 | 算法 | 作用 | 剔除率 |
|------|------|------|--------|
| **角点检测** | Shi-Tomasi + setMask | 提取纹理丰富的均匀分布角点 | — |
| **时序跟踪** | 金字塔 LK 光流 | 帧间特征点跟踪 | ~10% 失败 |
| **反向验证** | Forward-Backward Check | 剔除错误匹配 | ~5-15% |
| **几何约束** | F 矩阵 RANSAC | 剔除违反对极几何的外点 | ~1-5% |
| **空间匹配** | 双目 LK 光流 | 左右目特征点匹配 | ~10-20% 失败 |

最终输出的特征点具有：
- ✅ **全局唯一 ID**：从初始化到丢失，同一个 3D 点有固定 ID
- ✅ **归一化坐标**：消除相机内参和畸变影响
- ✅ **速度信息**：用于时间同步补偿
- ✅ **左右目关联**：双目模式下有右目对应点

这些特征点是 VINS 后端进行位姿估计和地图构建的**唯一视觉输入**，前端跟踪的质量直接决定了整个系统的精度和鲁棒性。
