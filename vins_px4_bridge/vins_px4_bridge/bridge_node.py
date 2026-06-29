#!/usr/bin/env python3
"""
VINS-Fusion to PX4 VehicleVisualOdometry bridge node.

Full ENU->NED conversion with optional PX4 magnetometer yaw alignment.

Coordinate chain (always applied):
    R_FRD->NED_virtual = R_ENU->NED * R_body->ENU * R_FRD->body

Optional yaw alignment (when yaw_alignment_mode == 'px4_mag'):
    delta_yaw = yaw_px4_mag - yaw_NED_virtual
    delta_q   = R_z(delta_yaw)   # NED_virtual -> NED_truth
    q_truth   = delta_q ⊗ q_FRD->NED_virtual

Reference frames:
    FLU: Forward-Left-Up   (VINS IMU body frame)
    FRD: Forward-Right-Down (PX4 body)
    ENU: East-North-Up      (VINS VIO world frame)
    NED: North-East-Down    (PX4 world)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, VehicleAttitude


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamiltonian quaternion multiplication q1 ⊗ q2. q = [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dtype=np.float64)


def quat_to_yaw(q: np.ndarray) -> float:
    """
    Extract yaw (rotation about Z) from a Hamiltonian quaternion q = [w,x,y,z].
    Assumes q represents body -> NED/ENU rotation (Z-Y-X Tait-Bryan order).
    """
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class VinsPx4Bridge(Node):
    def __init__(self):
        super().__init__('vins_px4_bridge')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('odometry_topic', '/vins_estimator/odometry')

        # body_frame:
    #   'OPENCV' -> VINS body = optical frame (X-right, Y-down, Z-forward)
    #   'FLU'    -> body = FLU (X-forward, Y-left, Z-up)
    #   'FRD'    -> body = FRD (passthrough)
    # RealSense D435i default IMU frame is camera_imu_optical_frame -> use OPENCV
        self.declare_parameter('body_frame', 'OPENCV')

        # yaw_alignment_mode:
        #   'none'      -> no yaw correction; output is in virtual NED
        #   'px4_mag'   -> subscribe to PX4 /fmu/out/vehicle_attitude,
        #                  compute delta_yaw = yaw_mag - yaw_virtual,
        #                  align SLAM yaw to magnetometer North
        #   'manual'    -> use manual_yaw_offset_rad as fixed delta_yaw
        self.declare_parameter('yaw_alignment_mode', 'none')
        self.declare_parameter('manual_yaw_offset_rad', 0.0)

        self.declare_parameter('position_jump_threshold', 0.5)
        self.declare_parameter('default_position_variance', [0.01, 0.01, 0.01])
        self.declare_parameter('default_orientation_variance', [0.01, 0.01, 0.01])
        self.declare_parameter('default_velocity_variance', [0.01, 0.01, 0.01])

        # ------------------------------------------------------------------
        # Precompute fixed constant quaternions (w, x, y, z)
        # ------------------------------------------------------------------
        # R_ENU->NED:  ENU (East-North-Up) -> NED (North-East-Down)
        #   Matrix: [0 1 0; 1 0 0; 0 0 -1]
        #   Quaternion: [0, sqrt(2)/2, sqrt(2)/2, 0]
        self.q_enu_to_ned = np.array([0.0, np.sqrt(2)/2, np.sqrt(2)/2, 0.0])

    # R_FRD->FLU:  FRD (Forward-Right-Down) -> FLU (Forward-Left-Up)
        #   Matrix: [1 0 0; 0 -1 0; 0 0 -1]
        #   Quaternion: [0, 1, 0, 0]  (180 deg about X)
        self.q_frd_to_flu = np.array([0.0, 1.0, 0.0, 0.0])

    # R_FRD->OPENCV: FRD -> optical frame (X-right, Y-down, Z-forward)
        #   FRD Forward -> optical Z
        #   FRD Right   -> optical X
        #   FRD Down    -> optical Y
        #   Matrix: [0 1 0; 0 0 1; 1 0 0]
        #   Quaternion: [-0.5, 0.5, 0.5, 0.5]
        self.q_frd_to_opencv = np.array([-0.5, 0.5, 0.5, 0.5])

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self.last_pos = None
        self.reset_counter = 0

        # Yaw alignment state
        self.px4_yaw = None
        self.yaw_offset = None  # delta_yaw = yaw_px4 - yaw_virtual (locked at first match)

        # ------------------------------------------------------------------
        # QoS: BEST_EFFORT to match Micro-ROS Agent
        # ------------------------------------------------------------------
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        odom_topic = self.get_parameter('odometry_topic').value
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        # NOTE: Do NOT apply timesync_offset here. PX4 uXRCE-DDS client
        # automatically converts timestamps during deserialization:
        #   topic.timestamp = msg.timestamp - session->time_offset
        # Manual offset would cause double-conversion.

        yaw_mode = self.get_parameter('yaw_alignment_mode').value
        if yaw_mode == 'px4_mag':
            self.sub_attitude = self.create_subscription(
                VehicleAttitude, '/fmu/out/vehicle_attitude',
                self.px4_attitude_callback, qos_best_effort)
            self.get_logger().info('Yaw alignment: subscribed to /fmu/out/vehicle_attitude')

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self.pub_vo = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)

        body = self.get_parameter('body_frame').value
        self.get_logger().info(
            f'VINS-PX4 bridge started | odom_topic={odom_topic} | '
            f'body_frame={body} | yaw_alignment={yaw_mode}')

    def px4_attitude_callback(self, msg: VehicleAttitude):
        """Receive PX4 true attitude (FRD -> true NED) and extract yaw."""
        q_px4 = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]], dtype=np.float64)
        q_px4 /= np.linalg.norm(q_px4)
        self.px4_yaw = quat_to_yaw(q_px4)

    def odom_callback(self, msg: Odometry):
        vo = VehicleOdometry()

        # ---- Timestamp (us) ------------------------------------------------
        # timestamp_sample: VINS original sample time -> EKF2 uses this for fusion
        # timestamp: current ROS2 time -> used only for logging/debugging
        vins_time_us = int(msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000)
        now = self.get_clock().now()
        ros_time_us = int(now.nanoseconds // 1000)

        vo.timestamp_sample = vins_time_us
        vo.timestamp = ros_time_us

        # ---- Pose frame -----------------------------------------------------
        vo.pose_frame = VehicleOdometry.POSE_FRAME_NED

        # ------------------------------------------------------------------
        # POSITION: full ENU -> NED
        #   NED X =  ENU Y   (North)
        #   NED Y =  ENU X   (East)
        #   NED Z = -ENU Z   (Down)
        # ------------------------------------------------------------------
        p_enu = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ], dtype=np.float64)

        vo.position = [
            float(p_enu[1]),   # North = East (ENU Y)
            float(p_enu[0]),   # East  = North (ENU X)
            float(-p_enu[2]),  # Down  = -Up
        ]

        # ------------------------------------------------------------------
        # ORIENTATION (quaternion chain): body -> ENU -> NED_virtual
        # ------------------------------------------------------------------
        q_body_to_enu = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ], dtype=np.float64)
        q_body_to_enu /= np.linalg.norm(q_body_to_enu)

        body_frame = self.get_parameter('body_frame').value
        if body_frame == 'OPENCV':
            # Chain: FRD -> optical -> ENU -> NED
            q_virtual = quat_multiply(
                self.q_enu_to_ned,
                quat_multiply(q_body_to_enu, self.q_frd_to_opencv)
            )
        elif body_frame == 'FLU':
            # Chain: FRD -> FLU -> ENU -> NED
            q_virtual = quat_multiply(
                self.q_enu_to_ned,
                quat_multiply(q_body_to_enu, self.q_frd_to_flu)
            )
        elif body_frame == 'FRD':
            # Chain: FRD -> ENU -> NED
            q_virtual = quat_multiply(self.q_enu_to_ned, q_body_to_enu)
        else:
            self.get_logger().warn(f'Unknown body_frame: {body_frame}, using FRD passthrough')
            q_virtual = quat_multiply(self.q_enu_to_ned, q_body_to_enu)

        q_virtual /= np.linalg.norm(q_virtual)

        # ------------------------------------------------------------------
        # YAW ALIGNMENT (optional)
        # ------------------------------------------------------------------
        yaw_mode = self.get_parameter('yaw_alignment_mode').value
        q_out = q_virtual.copy()

        if yaw_mode == 'px4_mag':
            if self.px4_yaw is not None:
                yaw_virtual = quat_to_yaw(q_virtual)

                if self.yaw_offset is None:
                    # Lock delta_yaw on first valid pair
                    self.yaw_offset = self.px4_yaw - yaw_virtual
                    self.get_logger().info(
                        f'Yaw offset locked: {math.degrees(self.yaw_offset):.2f} deg '
                        f'(PX4={math.degrees(self.px4_yaw):.2f}, SLAM={math.degrees(yaw_virtual):.2f})')

                # Apply locked offset: delta_q = R_z(yaw_offset)
                half = self.yaw_offset / 2.0
                delta_q = np.array([math.cos(half), 0.0, 0.0, math.sin(half)])
                q_out = quat_multiply(delta_q, q_virtual)
                q_out /= np.linalg.norm(q_out)
            else:
                self.get_logger().warn('Waiting for PX4 /fmu/out/vehicle_attitude...', throttle_duration_sec=5.0)

        elif yaw_mode == 'manual':
            offset = self.get_parameter('manual_yaw_offset_rad').value
            half = offset / 2.0
            delta_q = np.array([math.cos(half), 0.0, 0.0, math.sin(half)])
            q_out = quat_multiply(delta_q, q_virtual)
            q_out /= np.linalg.norm(q_out)

        vo.q = [float(q_out[0]), float(q_out[1]), float(q_out[2]), float(q_out[3])]

        # ------------------------------------------------------------------
        # VELOCITY: VINS twist is in World-ENU, transform to NED
        # ------------------------------------------------------------------
        v_enu = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ], dtype=np.float64)

        vo.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        vo.velocity = [
            float(v_enu[1]),   # North = East (ENU Y)
            float(v_enu[0]),   # East  = North (ENU X)
            float(-v_enu[2]),  # Down  = -Up
        ]

        # Angular velocity: VINS does not provide -> NaN
        vo.angular_velocity = [float('nan'), float('nan'), float('nan')]

        # ------------------------------------------------------------------
        # Reset detection
        # ------------------------------------------------------------------
        p_ned_arr = np.array(vo.position)
        if self.last_pos is not None:
            jump = np.linalg.norm(p_ned_arr - np.array(self.last_pos))
            thresh = self.get_parameter('position_jump_threshold').value
            if jump > thresh:
                self.reset_counter += 1
                self.yaw_offset = None  # force re-lock yaw offset after jump
                self.get_logger().warn(
                    f'Position jump: {jump:.3f} m, reset_counter={self.reset_counter}, '
                    f'yaw_offset will be re-locked')
        self.last_pos = p_ned_arr.copy()
        vo.reset_counter = self.reset_counter

        # ------------------------------------------------------------------
        # Variances
        # ------------------------------------------------------------------
        cov = msg.pose.covariance
        pos_var = [float(cov[0]), float(cov[7]), float(cov[14])]
        ori_var = [float(cov[21]), float(cov[28]), float(cov[35])]

        vo.position_variance = pos_var if any(v > 0.0 for v in pos_var) else \
            self.get_parameter('default_position_variance').value
        vo.orientation_variance = ori_var if any(v > 0.0 for v in ori_var) else \
            self.get_parameter('default_orientation_variance').value

        # Velocity variance: VINS does not provide -> use default or NaN
        vel_var = self.get_parameter('default_velocity_variance').value
        vo.velocity_variance = vel_var if any(v > 0.0 for v in vel_var) else \
            [float('nan'), float('nan'), float('nan')]

        vo.quality = 1
        self.pub_vo.publish(vo)


def main(args=None):
    rclpy.init(args=args)
    node = VinsPx4Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
