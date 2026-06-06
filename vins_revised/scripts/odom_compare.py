#!/usr/bin/env python3
"""
odom_compare.py
===============
实时对比 VINS 原始里程计 (/vins_estimator/odometry) 与
Loop Fusion 回环修正里程计 (/odometry_rect) 的差异。

用法：
    source /opt/ros/rolling/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 run vins vins_node <config>          # 终端 A
    ros2 run loop_fusion loop_fusion_node <config>  # 终端 B
    python3 odom_compare.py                   # 终端 C

可选参数：
    --save csv_file.csv    将对比结果保存到 CSV
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R
import csv
import sys
import signal


class OdomCompareNode(Node):
    def __init__(self, save_path=None):
        super().__init__('odom_compare')
        
        self.vins_odom = None
        self.loop_odom = None
        self.vins_time = 0.0
        self.loop_time = 0.0
        
        self.save_path = save_path
        self.csv_file = None
        self.csv_writer = None
        
        if save_path:
            self.csv_file = open(save_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'timestamp', 'vins_x', 'vins_y', 'vins_z',
                'loop_x', 'loop_y', 'loop_z',
                'pos_diff_m', 'angle_diff_deg'
            ])
            self.get_logger().info(f'Saving results to {save_path}')
        
        self.create_subscription(Odometry, '/vins_estimator/odometry',
                                 self.vins_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry_rect',
                                 self.loop_cb, rclpy.qos.qos_profile_sensor_data)
        
        # 0.5 Hz 打印对比结果
        self.create_timer(0.5, self.compare)
        
        self.get_logger().info('Waiting for /vins_estimator/odometry and /odometry_rect ...')
    
    def vins_cb(self, msg):
        self.vins_odom = msg
        self.vins_time = self.get_clock().now().nanoseconds / 1e9
    
    def loop_cb(self, msg):
        self.loop_odom = msg
        self.loop_time = self.get_clock().now().nanoseconds / 1e9
    
    def quat_angle_deg(self, q1, q2):
        """计算两个四元数 [x,y,z,w] 之间的角度差（度）"""
        dot = abs(np.dot(q1, q2))
        dot = min(dot, 1.0)
        return 2.0 * np.degrees(np.arccos(dot))
    
    def compare(self):
        if self.vins_odom is None:
            self.get_logger().info('Waiting for /vins_estimator/odometry ...')
            return
        if self.loop_odom is None:
            self.get_logger().info('Waiting for /odometry_rect ...')
            return
        
        vins_p = np.array([
            self.vins_odom.pose.pose.position.x,
            self.vins_odom.pose.pose.position.y,
            self.vins_odom.pose.pose.position.z
        ])
        loop_p = np.array([
            self.loop_odom.pose.pose.position.x,
            self.loop_odom.pose.pose.position.y,
            self.loop_odom.pose.pose.position.z
        ])
        
        pos_diff = float(np.linalg.norm(vins_p - loop_p))
        
        vins_q = [
            self.vins_odom.pose.pose.orientation.x,
            self.vins_odom.pose.pose.orientation.y,
            self.vins_odom.pose.pose.orientation.z,
            self.vins_odom.pose.pose.orientation.w
        ]
        loop_q = [
            self.loop_odom.pose.pose.orientation.x,
            self.loop_odom.pose.pose.orientation.y,
            self.loop_odom.pose.pose.orientation.z,
            self.loop_odom.pose.pose.orientation.w
        ]
        
        angle_diff = self.quat_angle_deg(vins_q, loop_q)
        
        # 终端彩色输出
        t = self.get_clock().now().nanoseconds / 1e9
        status = "🔄 LOOP CORRECTED" if pos_diff > 0.05 or angle_diff > 2.0 else "✅ MATCHED"
        
        print(f'\n[{t:.2f}s] {status}')
        print(f'  Position diff : {pos_diff:.4f} m')
        print(f'  Angle diff    : {angle_diff:.3f} deg')
        print(f'  VINS raw      : ({vins_p[0]:.3f}, {vins_p[1]:.3f}, {vins_p[2]:.3f})')
        print(f'  Loop corrected: ({loop_p[0]:.3f}, {loop_p[1]:.3f}, {loop_p[2]:.3f})')
        
        if self.csv_writer:
            self.csv_writer.writerow([
                f'{t:.3f}',
                f'{vins_p[0]:.6f}', f'{vins_p[1]:.6f}', f'{vins_p[2]:.6f}',
                f'{loop_p[0]:.6f}', f'{loop_p[1]:.6f}', f'{loop_p[2]:.6f}',
                f'{pos_diff:.6f}', f'{angle_diff:.6f}'
            ])
            self.csv_file.flush()
    
    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f'CSV saved to {self.save_path}')
        super().destroy_node()


def main():
    save_path = None
    if '--save' in sys.argv:
        idx = sys.argv.index('--save')
        if idx + 1 < len(sys.argv):
            save_path = sys.argv[idx + 1]
    
    rclpy.init()
    node = OdomCompareNode(save_path=save_path)
    
    def sigint_handler(sig, frame):
        node.get_logger().info('Shutting down...')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, sigint_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
