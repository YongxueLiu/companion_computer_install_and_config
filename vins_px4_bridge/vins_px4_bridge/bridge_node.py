#!/usr/bin/env python3
"""
VINS-Fusion to PX4 VehicleVisualOdometry bridge node.

Default mode: VIO (Visual-Inertial Odometry).
Can also run in VO mode with larger default variances.

Coordinate chain (VIO mode, body_frame='OPENCV'):
    q_frd_to_ned = q_enu_to_ned ⊗ q_body_to_enu ⊗ q_frd_to_opencv

    q_enu_to_ned    = [0.0,  sqrt(2)/2,  sqrt(2)/2,  0.0]   # [w, x, y, z]
    q_frd_to_opencv = [-0.5, 0.5,        0.5,        0.5]   # [w, x, y, z]

In matrix form:
    R_frd^ned = R_enu^ned * R_body^enu * R_frd^opencv

Fixed matrices:

              [0 1  0]                    [0 1 0]
    R_enu^ned=[1 0  0]    R_frd^opencv = [0 0 1]
              [0 0 -1]                    [1 0 0]

Let R_body^enu = [[r00 r01 r02],
                  [r10 r11 r12],
                  [r20 r21 r22]]

Full expansion:

    R_frd^ned = [[ r12,  r10,  r11],
                 [ r02,  r00,  r01],
                 [-r22, -r20, -r21]]

Verification (camera horizontal, facing North):

    R_body^enu = [[1,  0, 0],
                  [0,  0, 1],
                  [0, -1, 0]]

    R_frd^ned = I

i.e. FRD aligns with NED (Forward=North, Right=East, Down=Down).

Coordinate chain (VO mode):
    q_frd_to_ned = q_edn_to_ned ⊗ q_rdf_to_edn ⊗ q_frd_to_rdf

    q_edn_to_ned    = [ 0.5,  0.5,  0.5,  0.5]   # [w, x, y, z]
    q_frd_to_rdf    = [ 0.5, -0.5, -0.5, -0.5]   # [w, x, y, z]

In matrix form:
    R_frd^ned = R_edn^ned * R_rdf^edn * R_frd^rdf

Fixed matrices:

              [0 0 1]                    [0 1 0]
    R_edn^ned=[1 0 0]    R_frd^rdf    = [0 0 1]
              [0 1 0]                    [1 0 0]

VO mode assumes:
    - camera forward = virtual North
    - camera right   = virtual East
    - camera down    = Down

VINS VO output (RDF / OpenCV camera):
    X = right, Y = down, Z = forward

which is the same as EDN (East-Down-North).

Verification (camera facing North, t=0):
    q_rdf_to_edn = identity
    q_frd_to_ned = q_edn_to_ned ⊗ q_frd_to_rdf = identity

i.e. FRD aligns with NED.

For VINS + RealSense D435i:
    body_frame = 'OPENCV' (X-right, Y-down, Z-forward)
    VIO: VINS World is ENU-like: X=East/right, Y=North/forward, Z=Up
    VO:  VINS World is the first camera frame (RDF/EDN)

Reference: vins_config_reference/bridge_coordinate_transform.md
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
    """Extract yaw (rotation about Z) from a Hamiltonian quaternion q = [w,x,y,z]."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class VinsPx4Bridge(Node):
    def __init__(self):
        super().__init__('vins_px4_bridge')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        # mode: 'vio' (default) or 'vo'
        #        affects coordinate conversion, velocity handling and default variances
        self.declare_parameter('mode', 'vio')

        # odometry_topic: VINS output topic
        self.declare_parameter('odometry_topic', '/vins_estimator/odometry')

        # body_frame: 'OPENCV' for VINS + RealSense D435i
        self.declare_parameter('body_frame', 'OPENCV')

        # yaw_alignment_mode: 'none' / 'px4_mag' / 'manual'
        self.declare_parameter('yaw_alignment_mode', 'none')
        self.declare_parameter('manual_yaw_offset_rad', 0.0)

        self.declare_parameter('position_jump_threshold', 0.5)
        self.declare_parameter('publish_rate', 20.0)

        # Default variances; mode='vo' overrides to larger values if not set
        self.declare_parameter('default_position_variance', [0.01, 0.01, 0.01])
        self.declare_parameter('default_orientation_variance', [0.01, 0.01, 0.01])
        self.declare_parameter('default_velocity_variance', [0.01, 0.01, 0.01])

        mode = self.get_parameter('mode').value
        if mode == 'vo':
            # VO has no absolute scale and larger drift -> larger variances
            self.position_variance = self.get_parameter('default_position_variance').value
            if self.position_variance == [0.01, 0.01, 0.01]:
                self.position_variance = [0.1, 0.1, 0.1]
            self.orientation_variance = self.get_parameter('default_orientation_variance').value
            if self.orientation_variance == [0.01, 0.01, 0.01]:
                self.orientation_variance = [0.05, 0.05, 0.05]
            self.velocity_variance = self.get_parameter('default_velocity_variance').value
            if self.velocity_variance == [0.01, 0.01, 0.01]:
                self.velocity_variance = [0.1, 0.1, 0.1]
            self.get_logger().info('Mode: VO (larger default variances)')
        else:
            self.position_variance = self.get_parameter('default_position_variance').value
            self.orientation_variance = self.get_parameter('default_orientation_variance').value
            self.velocity_variance = self.get_parameter('default_velocity_variance').value
            self.get_logger().info('Mode: VIO')

        # ------------------------------------------------------------------
        # Precompute fixed constant quaternions (w, x, y, z)
        # ------------------------------------------------------------------
        # All verified by quaternion -> rotation matrix formula.
        # q = [w, x, y, z] and -q represent the same rotation.

        # R_ENU->NED: ENU (East-North-Up) -> NED (North-East-Down)
        #   Mapping: ENU X (East)  -> NED Y (East)
        #            ENU Y (North) -> NED X (North)
        #            ENU Z (Up)    -> NED Z (Down)
        #   Matrix: [[0, 1,  0],
        #            [1, 0,  0],
        #            [0, 0, -1]]
        #   Quaternion: [0, sqrt(2)/2, sqrt(2)/2, 0]
        self.q_enu_to_ned = np.array([0.0, np.sqrt(2)/2, np.sqrt(2)/2, 0.0])

        # R_FRD->OPENCV: FRD (Forward-Right-Down) -> OpenCV (X-right, Y-down, Z-forward)
        #   Mapping: FRD X (Forward) -> OPENCV Z (Forward)
        #            FRD Y (Right)   -> OPENCV X (Right)
        #            FRD Z (Down)    -> OPENCV Y (Down)
        #   Matrix: [[0, 1, 0],
        #            [0, 0, 1],
        #            [1, 0, 0]]
        #   Quaternion: [-0.5, 0.5, 0.5, 0.5]  (equiv. [0.5, -0.5, -0.5, -0.5])
        self.q_frd_to_opencv = np.array([-0.5, 0.5, 0.5, 0.5])

        # R_FRD->FLU: FRD (Forward-Right-Down) -> FLU (Forward-Left-Up)
        #   Mapping: FRD X (Forward) -> FLU X (Forward)
        #            FRD Y (Right)   -> FLU Y (Left)   (reversed)
        #            FRD Z (Down)    -> FLU Z (Up)     (reversed)
        #   Matrix: [[1,  0,  0],
        #            [0, -1,  0],
        #            [0,  0, -1]]
        #   Quaternion: [0, 1, 0, 0]  (180 deg about X)
        self.q_frd_to_flu = np.array([0.0, 1.0, 0.0, 0.0])

        # VO mode constants (RDF = OpenCV camera frame = EDN)

        # R_FRD->RDF: FRD -> RDF (Right-Down-Forward)
        #   (RDF is the same physical frame as OPENCV)
        #   Mapping: FRD X (Forward) -> RDF Z (Forward)
        #            FRD Y (Right)   -> RDF X (Right)
        #            FRD Z (Down)    -> RDF Y (Down)
        #   Matrix: [[0, 1, 0],
        #            [0, 0, 1],
        #            [1, 0, 0]]
        #   Quaternion: [0.5, -0.5, -0.5, -0.5]  (equiv. [-0.5, 0.5, 0.5, 0.5])
        self.q_frd_to_rdf = np.array([0.5, -0.5, -0.5, -0.5])

        # R_EDN->NED: EDN (East-Down-North) -> NED (North-East-Down)
        #   Mapping: EDN X (East)  -> NED Y (East)
        #            EDN Y (Down)  -> NED Z (Down)
        #            EDN Z (North) -> NED X (North)
        #   Matrix: [[0, 0, 1],
        #            [1, 0, 0],
        #            [0, 1, 0]]
        #   Quaternion: [0.5, 0.5, 0.5, 0.5]
        self.q_edn_to_ned = np.array([0.5, 0.5, 0.5, 0.5])

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self.last_pos = None
        self.reset_counter = 0
        self.last_publish_time = self.get_clock().now()

        # Yaw alignment state
        self.px4_yaw = None
        self.yaw_offset = None

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
            f'Bridge started | mode={mode} | body_frame={body} | '
            f'yaw_alignment={yaw_mode} | odom_topic={odom_topic}')

    def px4_attitude_callback(self, msg: VehicleAttitude):
        """Receive PX4 true attitude (FRD -> true NED) and extract yaw."""
        q_px4 = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]], dtype=np.float64)
        q_px4 /= np.linalg.norm(q_px4)
        self.px4_yaw = quat_to_yaw(q_px4)

    def odom_callback(self, msg: Odometry):
        now = self.get_clock().now()
        dt = (now - self.last_publish_time).nanoseconds * 1e-9
        publish_period = 1.0 / self.get_parameter('publish_rate').value
        if dt < publish_period:
            return
        self.last_publish_time = now

        vo = VehicleOdometry()

        # ---- Timestamp (us) -----------------------------------------------
        # PX4 uXRCE-DDS client auto-converts timestamps during deserialization.
        # Do NOT apply any offset here.
        vo.timestamp_sample = int(msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000)
        vo.timestamp = int(now.nanoseconds // 1000)

        # ---- Pose frame -----------------------------------------------------
        vo.pose_frame = VehicleOdometry.POSE_FRAME_NED

        mode = self.get_parameter('mode').value
        is_vo = (mode == 'vo')

        # ------------------------------------------------------------------
        # POSITION: VINS frame -> NED
        # ------------------------------------------------------------------
        p_world = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ], dtype=np.float64)

        if is_vo:
            # VO: VINS output is in RDF/EDN camera frame (X=right/East, Y=down/Down, Z=forward/North)
            vo.position = [
                float(p_world[2]),   # North = forward (RDF Z)
                float(p_world[0]),   # East  = right   (RDF X)
                float(p_world[1]),   # Down  = down    (RDF Y)
            ]
        else:
            # VIO: VINS World is ENU-like (X=right/East, Y=forward/North, Z=up)
            vo.position = [
                float(p_world[1]),   # North = forward (World Y)
                float(p_world[0]),   # East  = right   (World X)
                float(-p_world[2]),  # Down  = -up
            ]

        # ------------------------------------------------------------------
        # ORIENTATION (quaternion chain): body -> World -> NED
        # ------------------------------------------------------------------
        q_body_to_world = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ], dtype=np.float64)
        q_body_to_world /= np.linalg.norm(q_body_to_world)

        body_frame = self.get_parameter('body_frame').value
        if is_vo:
            # VO: VINS output is already in RDF/EDN camera frame.
            # q_body_to_world = q_rdf_to_edn (current camera vs initial camera).
            q_virtual = quat_multiply(
                self.q_edn_to_ned,
                quat_multiply(q_body_to_world, self.q_frd_to_rdf)
            )
        elif body_frame == 'OPENCV':
            q_virtual = quat_multiply(
                self.q_enu_to_ned,
                quat_multiply(q_body_to_world, self.q_frd_to_opencv)
            )
        elif body_frame == 'FLU':
            q_virtual = quat_multiply(
                self.q_enu_to_ned,
                quat_multiply(q_body_to_world, self.q_frd_to_flu)
            )
        elif body_frame == 'FRD':
            q_virtual = quat_multiply(self.q_enu_to_ned, q_body_to_world)
        else:
            self.get_logger().warn(f'Unknown body_frame: {body_frame}, using FRD passthrough')
            q_virtual = quat_multiply(self.q_enu_to_ned, q_body_to_world)

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
                    self.yaw_offset = self.px4_yaw - yaw_virtual
                    self.get_logger().info(
                        f'Yaw offset locked: {math.degrees(self.yaw_offset):.2f} deg '
                        f'(PX4={math.degrees(self.px4_yaw):.2f}, VINS={math.degrees(yaw_virtual):.2f})')

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
        # PX4 VehicleOdometry.reset_counter is uint8; keep it in [0, 255]
        vo.reset_counter = int(self.reset_counter) % 256

        # ------------------------------------------------------------------
        # Velocity: VINS frame -> NED
        # ------------------------------------------------------------------
        v_world = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ], dtype=np.float64)

        vo.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        if is_vo:
            # VO: VINS does NOT optimize velocity (Vs stays at zero).
            # Send NaN with huge variance so EKF2 ignores it.
            vo.velocity = [float('nan'), float('nan'), float('nan')]
        else:
            # VIO: VINS fills twist in World frame, same as position
            vo.velocity = [
                float(v_world[1]),
                float(v_world[0]),
                float(-v_world[2]),
            ]

        # Angular velocity not provided by VINS
        vo.angular_velocity = [float('nan'), float('nan'), float('nan')]

        # ------------------------------------------------------------------
        # Variances
        # ------------------------------------------------------------------
        cov = msg.pose.covariance
        pos_var = [float(cov[0]), float(cov[7]), float(cov[14])]
        ori_var = [float(cov[21]), float(cov[28]), float(cov[35])]

        vo.position_variance = pos_var if any(v > 0.0 for v in pos_var) else self.position_variance
        vo.orientation_variance = ori_var if any(v > 0.0 for v in ori_var) else self.orientation_variance
        if is_vo:
            # VO velocity is invalid; tell EKF2 not to fuse it
            vo.velocity_variance = [999.0, 999.0, 999.0]
        else:
            vo.velocity_variance = self.velocity_variance

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
