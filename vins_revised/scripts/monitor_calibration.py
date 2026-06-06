#!/usr/bin/env python3
"""
Real-time monitor for VINS online extrinsic calibration convergence.
Usage:
    python3 monitor_calibration.py [extrinsic_file]

Press Ctrl+C to stop.
"""

import sys
import os
import time
import cv2
import numpy as np

DEFAULT_PATH = os.path.expanduser("~/output/extrinsic_parameter.csv")


def rotation_error(R1, R2):
    """Return rotation error in degrees."""
    dR = R1.T @ R2
    trace = np.trace(dR)
    trace = min(3.0, max(-1.0, trace))
    return np.degrees(np.arccos((trace - 1.0) / 2.0))


def load_extrinsic(path):
    if not os.path.exists(path):
        return None
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        return None
    T = fs.getNode("body_T_cam0").mat()
    fs.release()
    return T


def main():
    filepath = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
    prev_T = None
    stable_count = 0
    print("=" * 70)
    print("  VINS Calibration Monitor")
    print("=" * 70)
    print(f"Watching: {filepath}")
    print("Waiting for file to appear...")
    print()

    try:
        while True:
            T = load_extrinsic(filepath)
            if T is None:
                time.sleep(1)
                continue

            R = T[:3, :3]
            t = T[:3, 3]

            if prev_T is None:
                print("File detected! Starting monitoring...\n")
                print(f"{'Time':>8}  {'|t| (m)':>10}  {'t_x':>10}  {'t_y':>10}  {'t_z':>10}  {'dR(deg)':>10}  {'dt(mm)':>10}  {'Stable':>8}")
                print("-" * 90)
                prev_T = T.copy()
                stable_count = 0

            dR = rotation_error(prev_T[:3, :3], R)
            dt = np.linalg.norm(prev_T[:3, 3] - t) * 1000.0  # mm

            if dR < 0.01 and dt < 0.1:
                stable_count += 1
                status = f"YES ({stable_count})"
            else:
                stable_count = 0
                status = "NO"

            now = time.strftime("%H:%M:%S")
            print(f"\r{now:>8}  {np.linalg.norm(t):>10.4f}  {t[0]:>10.4f}  {t[1]:>10.4f}  {t[2]:>10.4f}  {dR:>10.4f}  {dt:>10.2f}  {status:>8}", end="", flush=True)

            prev_T = T.copy()
            time.sleep(2)

    except KeyboardInterrupt:
        print("\n\nMonitoring stopped.")
        if stable_count >= 5:
            print(f"Calibration appears STABLE ({stable_count} consecutive readings < 0.01 deg / 0.1 mm)")
            print("You can now extract the result with:")
            print(f"  python3 ~/VINS-Fusion-ROS2/scripts/extract_calibration_result.py {filepath}")
        else:
            print("Calibration not yet stable. Continue moving the camera.")


if __name__ == "__main__":
    main()
