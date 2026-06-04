# VINS-Fusion-ROS2 迁移至 ROS2 Rolling 适配指南

本文档详细记录将 VINS-Fusion-ROS2 从 ROS2 Humble 迁移到 **ROS2 Rolling**（Ubuntu 22.04）所做的全部代码修改、兼容层设计和运行时修复。

> **环境信息**
> - OS: Ubuntu 22.04
> - ROS2 Distro: Rolling
> - librealsense2: v2.58.1
> - realsense-ros: v4.58.1
> - OpenCV: 4.x (Rolling 附带)
> - CMake: 3.28.3

---

## 1. 背景：Rolling 与 Humble 的关键差异

ROS2 Rolling 是滚动发行版，会不断引入破坏性 API 变更。本次迁移遇到的核心问题：

| 变更项 | Humble (可用) | Rolling (已移除/变更) |
|--------|--------------|---------------------|
| `ament_target_dependencies` | ✅ 宏可用 | ❌ **完全移除** |
| `rclcpp::Node::make_shared` | ✅ 静态工厂方法 | ❌ 已移除，改用 `std::make_shared` |
| `rclcpp::remove_ros_arguments(argc, argv)` | ✅ 接受 `int, char**` | ❌ 仅接受 `vector<string>` |
| 头文件后缀 | `.h` | `.hpp`（部分包强制迁移） |
| C++ 标准 | 14（默认可用） | 17（推荐，部分依赖要求） |

---

## 2. CMake 构建系统适配

### 2.1 `ament_target_dependencies` 的替代方案

**问题**：`ament_target_dependencies` 宏在 Rolling 中已完全移除，直接编译会报错：
```
Unknown CMake command "ament_target_dependencies"
```

**解决方案**：在所有 4 个包的 `CMakeLists.txt` 中引入自定义兼容函数 `ament_target_dependencies_compat`，用现代 CMake 的 `target_link_libraries` + `target_include_directories` 替代。

```cmake
function(ament_target_dependencies_compat target)
  foreach(pkg ${ARGN})
    if(${pkg} STREQUAL "OpenCV")
      if(OpenCV_INCLUDE_DIRS)
        target_include_directories(${target} PUBLIC ${OpenCV_INCLUDE_DIRS})
      endif()
      target_link_libraries(${target} ${OpenCV_LIBS})
    elseif(TARGET ${pkg}::${pkg})
      target_link_libraries(${target} ${pkg}::${pkg})
    else()
      if(DEFINED ${pkg}_INCLUDE_DIRS AND ${pkg}_INCLUDE_DIRS)
        target_include_directories(${target} PUBLIC ${${pkg}_INCLUDE_DIRS})
      endif()
      if(DEFINED ${pkg}_LIBRARIES AND ${pkg}_LIBRARIES)
        target_link_libraries(${target} ${${pkg}_LIBRARIES})
      endif()
    endif()
  endforeach()
endfunction()
```

**涉及文件**：
- `vins/CMakeLists.txt`
- `camera_models/CMakeLists.txt`
- `loop_fusion/CMakeLists.txt`
- `global_fusion/CMakeLists.txt`

**使用方式**：
```cmake
# 旧代码（Humble）
ament_target_dependencies(vins_node rclcpp std_msgs ...)

# 新代码（Rolling）
ament_target_dependencies_compat(vins_node rclcpp std_msgs ...)
```

### 2.2 C++ 标准升级

所有 4 个包的 `CMakeLists.txt` 中：
```cmake
-set(CMAKE_CXX_STANDARD 14)
+set(CMAKE_CXX_STANDARD 17)
```

Rolling 的某些系统依赖（如 `rclcpp`）已开始使用 C++17 特性，不升级会导致编译失败。

---

## 3. ROS2 API 代码适配

### 3.1 节点创建方式

**问题**：`rclcpp::Node::make_shared("node_name")` 在 Rolling 中已移除。

**修改**：所有节点入口统一改为 `std::make_shared`：

```cpp
// 旧代码
auto n = rclcpp::Node::make_shared("vins_estimator");

// 新代码
auto n = std::make_shared<rclcpp::Node>("vins_estimator");
```

**涉及文件**：
- `vins/src/rosNodeTest.cpp`
- `vins/src/KITTIOdomTest.cpp`
- `loop_fusion/src/pose_graph_node.cpp`
- `global_fusion/src/globalOptNode.cpp`

### 3.2 命令行参数解析

**问题**：`rclcpp::remove_ros_arguments(argc, argv)` 的旧重载（接受 `int, char**`）在 Rolling 中已移除。必须使用接受 `std::vector<std::string>` 的版本。

**修改**：

```cpp
// 旧代码
string config_file = argv[1];

// 新代码
std::vector<std::string> raw_args(argv + 1, argv + argc);
auto non_ros_args = rclcpp::remove_ros_arguments(raw_args);
string config_file = non_ros_args[1];  // 或 non_ros_args[0]，取决于索引
```

**涉及文件**：
- `vins/src/rosNodeTest.cpp`
- `loop_fusion/src/pose_graph_node.cpp`

### 3.3 头文件 `.h` → `.hpp` 迁移

Rolling 对部分核心包强制迁移了头文件后缀。不修改会导致 `file not found` 错误。

| 旧头文件 | 新头文件 |
|---------|---------|
| `cv_bridge/cv_bridge.h` | `cv_bridge/cv_bridge.hpp` |
| `tf2/LinearMath/Quaternion.h` | `tf2/LinearMath/Quaternion.hpp` |
| `tf2/LinearMath/Transform.h` | `tf2/LinearMath/Transform.hpp` |

**涉及文件**：
- `vins/src/rosNodeTest.cpp`
- `vins/src/KITTIOdomTest.cpp`
- `vins/src/utility/visualization.cpp`
- `vins/src/utility/visualization.h`
- `loop_fusion/src/pose_graph_node.cpp`

---

## 4. 运行时崩溃修复

### 4.1 `TransformBroadcaster` 未初始化导致 SegFault

**问题**：`visualization.cpp` 中的 `pubTF()` 函数在旧代码中有严重 bug：

```cpp
// 旧代码
std::shared_ptr<tf2_ros::TransformBroadcaster> br;
// ... 设置 transform ...
br->sendTransform(transform);  // ❌ br 是空指针，SegFault!
```

同时，`pubTF()` 开头有一个 `return;` 被注释掉了，导致 TF 从未发布。

**修复**：
1. 将 `TransformBroadcaster` 移到全局作用域，在 `registerPub()` 中初始化
2. 移除 `pubTF()` 中的 `return;` 和调试用的 `cout`
3. 使用全局 `tf_broadcaster` 发送 TF

```cpp
// visualization.cpp 全局
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

// registerPub() 中初始化
tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(n);

// pubTF() 中使用
tf_broadcaster->sendTransform(transform);
```

---

## 5. 话题名规范化

**问题**：旧代码中所有 publisher 的话题名没有统一前缀，如 `"path"`、`"odometry"`、`"point_cloud"`。在 ROS2 中，这会导致话题散落在根命名空间下，与 RViz 配置和 `ros2 topic list` 的预期不一致。

**修复**：所有话题统一加上 `/vins_estimator/` 前缀：

```cpp
// 旧代码
pub_path = n->create_publisher<nav_msgs::msg::Path>("path", 10);

// 新代码
pub_path = n->create_publisher<nav_msgs::msg::Path>("/vins_estimator/path", 10);
```

**完整话题列表**：
- `/vins_estimator/path`
- `/vins_estimator/odometry`
- `/vins_estimator/point_cloud`
- `/vins_estimator/margin_cloud`
- `/vins_estimator/key_poses`
- `/vins_estimator/camera_pose`
- `/vins_estimator/camera_pose_visual`
- `/vins_estimator/keyframe_pose`
- `/vins_estimator/keyframe_point`
- `/vins_estimator/extrinsic`
- `/vins_estimator/image_track`
- `/vins_estimator/imu_propagate`

---

## 6. 新增功能：PointCloud2 转换节点

**背景**：VINS 内部使用的是旧版 `sensor_msgs/PointCloud` 格式，而 RViz2 对 `sensor_msgs/PointCloud2` 的支持更好。

**新增文件**：`vins/src/pointcloud_converter.cpp`

功能：订阅 VINS 发布的旧版 PointCloud，自动转换为 PointCloud2 格式再发布。

| 订阅 (旧版) | 发布 (新版) |
|------------|------------|
| `/vins_estimator/point_cloud` | `/point_cloud2` |
| `/vins_estimator/margin_cloud` | `/margin_cloud2` |
| `/vins_estimator/keyframe_point` | `/keyframe_point2` |

**构建配置**：在 `vins/CMakeLists.txt` 中新增：
```cmake
add_executable(pointcloud_converter src/pointcloud_converter.cpp)
ament_target_dependencies_compat(pointcloud_converter rclcpp std_msgs sensor_msgs)
target_link_libraries(pointcloud_converter ${sensor_msgs_LIBRARIES})
```

---

## 7. RealSense D435i 配置适配

### 7.1 话题名双重前缀

RealSense ROS v4.58.1 在 Rolling 下的话题前缀为 `/camera/camera/...`（namespace + node name），而非旧版的 `/camera/...`。

**修正**：
```yaml
image0_topic: "/camera/camera/infra1/image_rect_raw"
image1_topic: "/camera/camera/infra2/image_rect_raw"
```

### 7.2 分辨率匹配

RealSense 默认红外输出为 848x480，但 VINS 的标定文件 `left.yaml` / `right.yaml` 原本是 640x480 的内参。

**修正方案**：
- 将 `left.yaml` 和 `right.yaml` 更新为 848x480 的实际出厂内参
- `image_width: 848`, `image_height: 480`

### 7.3 IMU 兼容性

D435i 的 IMU 在 `librealsense2 v2.58.1` + Ubuntu 22.04 上不稳定，频繁触发 `Motion Module failure`。

**Workaround**：暂时使用纯双目 VO 模式（`imu: 0`），规避 IMU 问题。

---

## 8. 修改文件汇总

### 8.1 核心代码修改

| 文件 | 修改类型 | 说明 |
|------|---------|------|
| `vins/CMakeLists.txt` | 构建适配 | C++17、`ament_target_dependencies_compat`、新增 `pointcloud_converter` |
| `vins/src/rosNodeTest.cpp` | API 适配 | `std::make_shared`、`remove_ros_arguments`、`.hpp` 头文件 |
| `vins/src/utility/visualization.cpp` | 运行时修复 + 话题规范 | `TransformBroadcaster` 初始化、话题加前缀、移除 `return;` |
| `vins/src/utility/visualization.h` | 头文件迁移 | `.h` → `.hpp` |
| `vins/src/KITTIOdomTest.cpp` | 头文件迁移 | `cv_bridge.hpp` |
| `camera_models/CMakeLists.txt` | 构建适配 | C++17、`ament_target_dependencies_compat` |
| `loop_fusion/CMakeLists.txt` | 构建适配 | C++17、`ament_target_dependencies_compat` |
| `loop_fusion/src/pose_graph_node.cpp` | API 适配 | `std::make_shared`、`remove_ros_arguments`、`.hpp` |
| `loop_fusion/src/parameters.h` | 头文件迁移 | `.h` → `.hpp` |
| `global_fusion/CMakeLists.txt` | 构建适配 | C++17、`ament_target_dependencies_compat` |
| `global_fusion/src/globalOptNode.cpp` | API 适配 | `std::make_shared` |

### 8.2 配置文件修改

| 文件 | 修改说明 |
|------|---------|
| `config/realsense_d435i/realsense_stereo_imu_config.yaml` | 话题名、分辨率改回 848x480 |
| `config/realsense_d435i/left.yaml` | 更新为 848x480 真实内参 |
| `config/realsense_d435i/right.yaml` | 更新为 848x480 真实内参 |

### 8.3 新增文件

| 文件 | 说明 |
|------|------|
| `vins/src/pointcloud_converter.cpp` | PointCloud → PointCloud2 转换节点 |
| `config/realsense_d435i/start_realsense_for_vins.sh` | RealSense 启动脚本 |

---

## 9. 编译验证

适配完成后，4 个包全部编译通过：

```bash
cd ~/ros2_ws
colcon build --packages-select camera_models vins loop_fusion global_fusion
```

输出：
```
Summary: 4 packages finished
```

---

## 10. 已知限制与未来工作

| 问题 | 状态 | 建议 |
|------|------|------|
| D435i IMU 驱动不稳定 | ⚠️ 临时规避 (`imu: 0`) | 降级 `librealsense2` 至 v2.54.2 |
| 640x480 模式下图像同步异常 | ❌ 已放弃 | 使用默认 848x480 |
| `ament_target_dependencies_compat` 是临时方案 | ⚠️ 可用 | 长期应改用现代 CMake targets（如 `cv_bridge::cv_bridge`） |
| `sensor_msgs/PointCloud` 旧格式 | ⚠️ 已加转换层 | 长期应修改 VINS 内部直接发布 PointCloud2 |
