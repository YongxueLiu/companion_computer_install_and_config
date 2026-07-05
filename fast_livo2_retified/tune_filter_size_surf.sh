#!/bin/bash
# 自动扫描 FAST-LIVO2 的 filter_size_surf 参数
# 适用于 70 平米小房间，在 Jetson Orin Nano 上测试
# 用法：./tune_filter_size_surf.sh

set -e

WORKSPACE="/home/lingzhilab/ws_livo"
CONFIG_SRC="/home/lingzhilab/fast_liov/src/FAST-LIVO2-ROS2/config/mid360_d435i.yaml"
CONFIG_INSTALL="${WORKSPACE}/install/fast_livo/share/fast_livo/config/mid360_d435i.yaml"
BACKUP="${CONFIG_SRC}.backup.$(date +%s)"
RESULT_DIR="/home/lingzhilab/fast_liov/tune_results/run_$(date +%s)"
DURATION=60  # 每组参数跑 60 秒

# 参数组合 (filter_size_surf, voxel_size)
PARAMS=(
  "0.05 0.3"
  "0.10 0.3"
  "0.15 0.4"
  "0.20 0.5"
  "0.25 0.5"
)

mkdir -p "${RESULT_DIR}"
cp "${CONFIG_SRC}" "${BACKUP}"
echo "[INFO] 备份原配置: ${BACKUP}"
echo "[INFO] 结果保存到: ${RESULT_DIR}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"

for i in "${!PARAMS[@]}"; do
  read -r FS VS <<< "${PARAMS[$i]}"
  echo ""
  echo "=========================================="
  echo "[TEST $((i+1))/${#PARAMS[@]}] filter_size_surf=${FS}, voxel_size=${VS}"
  echo "=========================================="

  # 修改参数（兼容不同缩进）
  sed -i "s/^\([[:space:]]*filter_size_surf:\).*/\1 ${FS}/" "${CONFIG_SRC}"
  sed -i "s/^\([[:space:]]*voxel_size:\).*/\1 ${VS}/" "${CONFIG_SRC}"

  # 同步到 install（如果 launch 读的是 install 里的配置）
  if [ -f "${CONFIG_INSTALL}" ]; then
    cp "${CONFIG_SRC}" "${CONFIG_INSTALL}"
  fi

  # 保存当前配置副本
  cp "${CONFIG_SRC}" "${RESULT_DIR}/config_fs${FS}_vs${VS}.yaml"

  LOG_FILE="${RESULT_DIR}/fs${FS}_vs${VS}.log"

  echo "[INFO] 启动 FAST-LIVO2，运行 ${DURATION} 秒..."
  echo "[INFO] 日志: ${LOG_FILE}"

  # 启动 launch
  timeout "${DURATION}" ros2 launch fast_livo mapping_mid360_d435i.launch.py use_rviz:=False > "${LOG_FILE}" 2>&1 || true

  # 确保上一轮进程完全退出
  sleep 3
  pkill -f "fastlivo_mapping" 2>/dev/null || true
  pkill -f "livox_ros_driver2_node" 2>/dev/null || true
  pkill -f "realsense2_camera_node" 2>/dev/null || true
  sleep 2

  # 提取关键指标
  echo "[INFO] 统计结果:"
  echo "  LIO 平均耗时:"
  grep -oP 'Average Total Time\s*\|\s*\K[0-9.]+' "${LOG_FILE}" | tail -1 || echo "  N/A"
  echo "  stateEstimationAndMapping() 最后几次耗时:"
  grep -oP 'stateEstimationAndMapping\(\) took \K[0-9.]+' "${LOG_FILE}" | tail -5 || echo "  N/A"
  echo "  VIO Raw feature num 最后几次:"
  grep -oP '\[ VIO \] Raw feature num: \K[0-9]+' "${LOG_FILE}" | tail -5 || echo "  N/A"
  echo "  LIO 帧数:"
  grep -c '\[ LIO \] Update Voxel Map' "${LOG_FILE}" || echo "  0"
  echo "  VIO 帧数:"
  grep -c '\[ VIO \] Raw feature num' "${LOG_FILE}" || echo "  0"

done

# 恢复原始配置
cp "${BACKUP}" "${CONFIG_SRC}"
if [ -f "${CONFIG_INSTALL}" ]; then
  cp "${CONFIG_SRC}" "${CONFIG_INSTALL}"
fi
echo ""
echo "[INFO] 测试完成，已恢复原始配置"
echo "[INFO] 结果目录: ${RESULT_DIR}"
echo "[INFO] 请把 ${RESULT_DIR} 打包发给我分析"
