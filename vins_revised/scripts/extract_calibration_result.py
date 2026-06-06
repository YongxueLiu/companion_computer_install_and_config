#!/usr/bin/env python3
"""
Extract and validate VINS online calibration results.
Usage:
    python3 extract_calibration_result.py [extrinsic_file]

Default reads: ~/output/extrinsic_parameter.csv
Outputs:
    - Human-readable R, t, Euler angles
    - Physical sanity checks
    - Generated body_T_cam1 for stereo setup
    - Ready-to-copy YAML snippets
"""

import sys
import os
import cv2
import numpy as np

DEFAULT_PATH = os.path.expanduser("~/output/extrinsic_parameter.csv")


def rotation_matrix_to_euler_angles(R):
    """Convert rotation matrix to XYZ Euler angles in degrees."""
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.degrees(np.array([x, y, z]))


def format_matrix_yaml(name, T, indent=3):
    """Format 4x4 matrix as opencv-matrix YAML block."""
    lines = [f"{' ' * indent}{name}: !!opencv-matrix"]
    lines.append(f"{' ' * (indent + 3)}rows: 4")
    lines.append(f"{' ' * (indent + 3)}cols: 4")
    lines.append(f"{' ' * (indent + 3)}dt: d")
    data_str = ", ".join(f"{v:.16e}" for v in T.flatten())
    lines.append(f"{' ' * (indent + 3)}data: [ {data_str} ]")
    return "\n".join(lines)


def main():
    filepath = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
    if not os.path.exists(filepath):
        print(f"ERROR: File not found: {filepath}")
        print(f"Make sure VINS has run with estimate_extrinsic=1 or 2")
        sys.exit(1)

    fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        print(f"ERROR: Cannot open {filepath} as YAML")
        sys.exit(1)

    T0 = fs.getNode("body_T_cam0").mat()
    T1_node = fs.getNode("body_T_cam1")
    T1 = T1_node.mat() if not T1_node.empty() else None
    fs.release()

    if T0 is None:
        print("ERROR: body_T_cam0 not found in file")
        sys.exit(1)

    R0 = T0[:3, :3]
    t0 = T0[:3, 3]
    euler0 = rotation_matrix_to_euler_angles(R0)

    print("=" * 70)
    print("  VINS Online Calibration Result Extractor")
    print("=" * 70)
    print(f"\nSource: {filepath}")
    print()
    print("-" * 70)
    print("  body_T_cam0 (IMU -> Left Camera)")
    print("-" * 70)
    print("\nFull 4x4 matrix:")
    for row in T0:
        print("  [" + ", ".join(f"{v: .6e}" for v in row) + " ]")

    print(f"\nRotation R:")
    for row in R0:
        print("  [" + ", ".join(f"{v: .6e}" for v in row) + " ]")

    print(f"\nTranslation t (m): [{t0[0]:.6f}, {t0[1]:.6f}, {t0[2]:.6f}]")
    print(f"Translation norm:    {np.linalg.norm(t0):.6f} m")
    print(f"Euler angles (deg):  roll={euler0[0]:.3f}, pitch={euler0[1]:.3f}, yaw={euler0[2]:.3f}")

    # Sanity checks
    print("\n" + "-" * 70)
    print("  Physical Sanity Checks")
    print("-" * 70)
    det = np.linalg.det(R0)
    print(f"  det(R) = {det:.6f}  {'PASS' if abs(det - 1.0) < 0.01 else 'FAIL (should be ~1.0)'}")
    print(f"  |t|    = {np.linalg.norm(t0):.4f} m  {'PASS' if 0.015 <= np.linalg.norm(t0) <= 0.035 else 'WARN (expect 0.015~0.035 m for D435i)'}")
    print(f"  |pitch| = {abs(euler0[1]):.2f} deg  {'PASS' if abs(euler0[1]) < 10 else 'WARN (expect < 10 deg)'}")
    print(f"  |roll|  = {abs(euler0[0]):.2f} deg  {'PASS' if abs(euler0[0]) < 10 else 'WARN (expect < 10 deg)'}")

    # body_T_cam1
    print("\n" + "-" * 70)
    print("  body_T_cam1 (IMU -> Right Camera)")
    print("-" * 70)

    if T1 is not None:
        print("\nFound in calibration file:")
        for row in T1:
            print("  [" + ", ".join(f"{v: .6e}" for v in row) + " ]")
    else:
        # Approximate from baseline
        baseline = 0.04995  # D435i 640x480 baseline in meters
        t1 = t0 + R0 @ np.array([baseline, 0.0, 0.0])
        T1 = np.eye(4)
        T1[:3, :3] = R0
        T1[:3, 3] = t1
        print(f"\nApproximated from baseline = {baseline:.5f} m:")
        print(f"Translation t1 (m): [{t1[0]:.6f}, {t1[1]:.6f}, {t1[2]:.6f}]")
        for row in T1:
            print("  [" + ", ".join(f"{v: .6e}" for v in row) + " ]")

    # YAML snippets
    print("\n" + "=" * 70)
    print("  Ready-to-copy YAML snippets for config file")
    print("=" * 70)
    print("\n# Set estimate_extrinsic to 0 after calibration")
    print("estimate_extrinsic: 0")
    print()
    print(format_matrix_yaml("body_T_cam0", T0, indent=0))
    print()
    print(format_matrix_yaml("body_T_cam1", T1, indent=0))
    print()
    print("# Also update td if it was estimated")
    print("# td: 0.0")
    print()


if __name__ == "__main__":
    main()
