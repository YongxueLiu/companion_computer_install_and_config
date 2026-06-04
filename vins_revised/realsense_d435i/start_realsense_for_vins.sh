#!/bin/bash
# 启动 RealSense D435i，输出默认 848x480 红外图像

source /opt/ros/rolling/setup.bash
source ~/ros2_ws/install/setup.bash

echo "🚀 Starting RealSense D435i in default 848x480 mode for VINS-Fusion..."

ros2 launch realsense2_camera rs_launch.py \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_depth:=false \
  enable_color:=false \
  enable_gyro:=false \
  enable_accel:=false
