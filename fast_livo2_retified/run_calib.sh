#!/bin/bash
set -e

source /home/lingzhilab/ws_livox/install/setup.bash
source /home/lingzhilab/vins/install/setup.bash
source /home/lingzhilab/ws_livo/install/setup.bash
export ROS_RSUSB_BACKEND=1

TIMESTAMP=$(date +%s)
BAG_DIR=/tmp/calib_bag_${TIMESTAMP}
FINAL_DIR=/home/lingzhilab/calib_data/run_${TIMESTAMP}
mkdir -p "$FINAL_DIR"

LIVOX_LAUNCH=/home/lingzhilab/fast_liov/launch/calib/livox_only.launch.py
RS_LAUNCH=/home/lingzhilab/fast_liov/launch/calib/realsense_only.launch.py

echo "=============================================="
echo "  LiDAR-Camera Calibration for MID360 + D435i"
echo "=============================================="
echo "Final output dir: $FINAL_DIR"

# Check RealSense device
echo ""
echo "[1/8] Checking RealSense device..."
if ! lsusb | grep -qi realsense; then
    echo "ERROR: No RealSense device found. Please check USB connection."
    exit 1
fi
echo "OK: RealSense detected."

# Launch Livox
echo ""
echo "[2/8] Starting Livox driver..."
ros2 launch "$LIVOX_LAUNCH" > /tmp/livox_calib.log 2>&1 &
LIVOX_PID=$!
echo "  Livox PID: $LIVOX_PID"
sleep 15

# Launch RealSense
echo ""
echo "[3/8] Starting RealSense driver..."
ros2 launch "$RS_LAUNCH" > /tmp/realsense_calib.log 2>&1 &
RS_PID=$!
echo "  RealSense PID: $RS_PID"
sleep 15

# Check image topic
echo ""
echo "[4/8] Checking image stream..."
ros2 topic hz /camera/camera/infra1/image_rect_raw --window 20 > /tmp/image_hz_calib.log 2>&1 &
HZ_PID=$!
sleep 5
kill $HZ_PID || true
if grep -q "average rate" /tmp/image_hz_calib.log; then
    cat /tmp/image_hz_calib.log | tail -n 5
    echo "OK: Image stream is active."
else
    echo "ERROR: Image stream not active. See /tmp/realsense_calib.log"
    kill $RS_PID $LIVOX_PID || true
    exit 1
fi

# Record bag
echo ""
echo "[5/8] Recording 10 s bag..."
ros2 bag record /livox/lidar /camera/camera/infra1/image_rect_raw -o "$BAG_DIR" --max-bag-duration 12 > /tmp/ros2_bag_record_calib.log 2>&1 &
BAG_PID=$!
echo "  Bag record PID: $BAG_PID"
sleep 10

kill $BAG_PID || true
wait $BAG_PID || true

# Stop drivers
kill $RS_PID $LIVOX_PID || true
wait $RS_PID $LIVOX_PID || true

# Copy bag
echo ""
echo "[6/8] Saving bag to $FINAL_DIR..."
cp -r "$BAG_DIR" "$FINAL_DIR/"
ros2 bag info "$BAG_DIR" > "$FINAL_DIR/bag_info.txt"

# Extract image and PCD
echo ""
echo "[7/8] Extracting image and PCD..."
mkdir -p "$FINAL_DIR/from_bag/pcd" "$FINAL_DIR/from_bag/img"
timeout --signal=INT 180 ros2 launch calib_ros2 bag_to_pcd.launch.py \
  bags_dir:="$BAG_DIR" \
  pcds_dir:="$FINAL_DIR/from_bag/pcd" \
  images_dir:="$FINAL_DIR/from_bag/img" \
  lidar_topic:=/livox/lidar \
  image_topic:=/camera/camera/infra1/image_rect_raw \
  is_custom_msg:=true > /tmp/bag_to_pcd_calib.log 2>&1 || true

# Update calib config
echo ""
echo "[8/8] Running livox_camera_calib..."
python3 - <<PYEOF
import yaml, os
final_dir = "$FINAL_DIR"
config_path = "/home/lingzhilab/ws_livo/install/calib_ros2/share/calib_ros2/config/calib.yaml"
with open(config_path, 'r') as f:
    cfg = yaml.safe_load(f)
# Ensure calib_config_file points to the MID360+D435i config in this workspace
cfg['/**']['ros__parameters']['calib']['calib_config_file'] = "/home/lingzhilab/ws_livo/install/calib_ros2/share/calib_ros2/config/config_mid360_d435i.yaml"
cfg['/**']['ros__parameters']['calib']['use_rough_calib'] = True
cfg['/**']['ros__parameters']['common']['image_file'] = os.path.join(final_dir, "from_bag", "img", "0.bmp")
cfg['/**']['ros__parameters']['common']['pcd_file'] = os.path.join(final_dir, "from_bag", "pcd", "0.pcd")
cfg['/**']['ros__parameters']['common']['result_file'] = os.path.join(final_dir, "from_bag", "extrinsic.txt")
# Camera intrinsics for D435i infrared 640x480
if 'camera' not in cfg['/**']['ros__parameters']:
    cfg['/**']['ros__parameters']['camera'] = {}
cfg['/**']['ros__parameters']['camera']['camera_matrix'] = [384.600464, 0.0, 316.432343, 0.0, 384.600464, 239.290955, 0.0, 0.0, 1.0]
cfg['/**']['ros__parameters']['camera']['dist_coeffs'] = [0.0, 0.0, 0.0, 0.0]
with open(config_path, 'w') as f:
    yaml.safe_dump(cfg, f, default_flow_style=False)
print("Updated calib.yaml")
PYEOF

timeout --signal=INT 180 ros2 launch calib_ros2 calib.launch.py > "$FINAL_DIR/calib_run.log" 2>&1 || true

echo ""
echo "=============================================="
echo "  Calibration finished"
echo "=============================================="
echo "Result file: $FINAL_DIR/from_bag/extrinsic.txt"
echo "Run log:     $FINAL_DIR/calib_run.log"
if [ -f "$FINAL_DIR/from_bag/extrinsic.txt" ]; then
    echo ""
    echo "Calibration output (LiDAR -> Camera, use directly in FAST-LIVO2):"
    cat "$FINAL_DIR/from_bag/extrinsic.txt"
else
    echo "ERROR: Result file not found. Check $FINAL_DIR/calib_run.log"
fi
