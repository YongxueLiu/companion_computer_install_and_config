#!/bin/bash
set -e

# ============================================================================
# MID360 + FAST_LIO 完整安装脚本（严格按各自官方 README，支持断点续传）
# 官方仓库：
#   - Livox-SDK2:     https://github.com/Livox-SDK/Livox-SDK2
#   - livox_ros_driver2: https://github.com/Livox-SDK/livox_ros_driver2
#   - FAST_LIO_ROS2:  https://github.com/Ericsii/FAST_LIO_ROS2
# ============================================================================

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

WS_DIR="${HOME}/ws_livox"
SDK2_DIR="${HOME}/Livox-SDK2"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  MID360 + FAST_LIO 一键安装脚本${NC}"
echo -e "${GREEN}  支持断点续传（已完成的步骤自动跳过）${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# ----------------------------------------------------------------------------
# 0. 检查 ROS2 Humble
# ----------------------------------------------------------------------------
if [ ! -f /opt/ros/humble/setup.sh ]; then
    echo -e "${RED}[错误] ROS2 Humble 未安装${NC}"
    exit 1
fi

# ----------------------------------------------------------------------------
# 1. 安装系统编译依赖（幂等，重复运行安全）
# ----------------------------------------------------------------------------
echo -e "${YELLOW}[1/6] 安装/检查系统依赖...${NC}"
sudo apt-get update
sudo apt-get install -y \
    git \
    cmake \
    build-essential \
    libapr1-dev \
    libboost-all-dev \
    libpcl-dev \
    libeigen3-dev \
    python3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions

# ----------------------------------------------------------------------------
# 2. Livox-SDK2（官方步骤，已安装则跳过）
# ----------------------------------------------------------------------------
if [ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]; then
    echo -e "${BLUE}[2/6] Livox-SDK2 已安装，跳过${NC}"
else
    echo -e "${YELLOW}[2/6] 下载并编译 Livox-SDK2...${NC}"
    if [ ! -d "${SDK2_DIR}" ]; then
        git clone https://github.com/Livox-SDK/Livox-SDK2.git "${SDK2_DIR}"
    fi
    cd "${SDK2_DIR}"
    mkdir -p build && cd build
    cmake .. && make -j
    sudo make install
    sudo ldconfig
fi

# ----------------------------------------------------------------------------
# 3. 创建工作空间，下载 livox_ros_driver2（已下载则跳过）
# ----------------------------------------------------------------------------
mkdir -p "${WS_DIR}/src"

if [ -d "${WS_DIR}/src/livox_ros_driver2" ]; then
    echo -e "${BLUE}[3/6] livox_ros_driver2 已下载，跳过${NC}"
else
    echo -e "${YELLOW}[3/6] 下载 livox_ros_driver2...${NC}"
    cd "${WS_DIR}/src"
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git
fi

# ----------------------------------------------------------------------------
# 4. 编译 livox_ros_driver2（官方 build.sh humble，已编译则跳过）
# ----------------------------------------------------------------------------
if [ -d "${WS_DIR}/install/livox_ros_driver2" ]; then
    echo -e "${BLUE}[4/6] livox_ros_driver2 已编译，跳过${NC}"
else
    echo -e "${YELLOW}[4/6] 编译 livox_ros_driver2（官方 ./build.sh humble）...${NC}"
    cd "${WS_DIR}/src/livox_ros_driver2"
    source /opt/ros/humble/setup.sh
    ./build.sh humble
fi

# ----------------------------------------------------------------------------
# 5. 下载 FAST_LIO_ROS2（官方 --recursive，已下载则更新子模块）
# ----------------------------------------------------------------------------
if [ -d "${WS_DIR}/src/FAST_LIO_ROS2" ]; then
    echo -e "${BLUE}[5/6] FAST_LIO_ROS2 已下载，更新子模块...${NC}"
    cd "${WS_DIR}/src/FAST_LIO_ROS2"
    git submodule update --init --recursive || true
else
    echo -e "${YELLOW}[5/6] 下载 FAST_LIO_ROS2（--recursive）...${NC}"
    cd "${WS_DIR}/src"
    git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
fi

# ----------------------------------------------------------------------------
# 6. 编译 FAST_LIO_ROS2（官方 colcon build --symlink-install，已编译则跳过）
# ----------------------------------------------------------------------------
if [ -d "${WS_DIR}/install/fast_lio" ]; then
    echo -e "${BLUE}[6/6] FAST_LIO_ROS2 已编译，跳过${NC}"
else
    echo -e "${YELLOW}[6/6] 编译 FAST_LIO_ROS2（官方 colcon build --symlink-install）...${NC}"
    cd "${WS_DIR}"

    # source livox_ros_driver before build（FAST_LIO 官方要求）
    source "${WS_DIR}/install/setup.sh"

    # rosdep（若未初始化则先初始化）
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init || true
    fi
    rosdep update || true
    rosdep install --from-paths src --ignore-src -y || true

    # 官方编译命令（只编译 fast_lio，避免 livox_ros_driver2 因参数变化被重新编译）
    colcon build --packages-select fast_lio --symlink-install
fi

# ----------------------------------------------------------------------------
# 7. 写入环境变量（幂等）
# ----------------------------------------------------------------------------
if ! grep -q "ws_livox/install/setup.sh" "${HOME}/.bashrc"; then
    echo -e "${YELLOW}配置环境变量...${NC}"
    echo "" >> "${HOME}/.bashrc"
    echo "# === MID360 + FAST_LIO 工作空间 ===" >> "${HOME}/.bashrc"
    echo "source /opt/ros/humble/setup.sh" >> "${HOME}/.bashrc"
    echo "export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH}:/usr/local/lib" >> "${HOME}/.bashrc"
    echo "source ${WS_DIR}/install/setup.sh" >> "${HOME}/.bashrc"
else
    echo -e "${BLUE}环境变量已配置，跳过${NC}"
fi

# ----------------------------------------------------------------------------
# 完成
# ----------------------------------------------------------------------------
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}        安装完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "【目录结构】"
echo "  Livox-SDK2        : ${SDK2_DIR}"
echo "  ROS2 工作空间     : ${WS_DIR}"
echo ""
echo "【重要：网络配置】"
echo "  MID360 需要电脑网口设置静态IP（如 192.168.1.50/24），"
echo "  且需和 ${WS_DIR}/src/livox_ros_driver2/config/MID360_config.json 里的"
echo "  cmd_data_ip 一致。"
echo ""
echo "【官方使用方式】"
echo ""
echo "1) 启动雷达驱动（终端 A）："
echo "   source ${WS_DIR}/install/setup.sh"
echo "   ros2 launch livox_ros_driver2 msg_MID360_launch.py"
echo ""
echo "2) 启动 FAST_LIO 建图+里程计（终端 B）："
echo "   source ${WS_DIR}/install/setup.sh"
echo "   ros2 launch fast_lio mapping.launch.py"
echo ""
echo "3) 查看里程计："
echo "   ros2 topic echo /Odometry"
echo ""
echo "【如需强制重新编译某一步】"
echo "  - Livox-SDK2     : sudo rm -f /usr/local/lib/liblivox_lidar_sdk_shared.so"
echo "  - livox_ros_driver2: rm -rf ${WS_DIR}/install/livox_ros_driver2"
echo "  - FAST_LIO_ROS2  : rm -rf ${WS_DIR}/install/fast_lio"
echo ""
