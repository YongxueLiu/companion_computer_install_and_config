#!/usr/bin/env python3
"""
realtime_odom_plot.py
=====================
实时可视化 VINS 原始轨迹 vs Loop Fusion 回环修正轨迹。

用法：
    source /opt/ros/rolling/setup.bash
    source ~/ros2_ws/install/setup.bash
    
    # 显示 VINS + Loop Fusion 对比（默认）
    python3 realtime_odom_plot.py
    
    # 只显示 VINS 原始值（不启动 Loop Fusion 时用）
    python3 realtime_odom_plot.py --no-loop

功能：
    1. 实时订阅 /vins_estimator/odometry 和 /odometry_rect
    2. 动态绘制 X/Y/Z 时间序列（最近 400 个数据点）
    3. 实时显示位置漂移量和角度漂移量（对比模式）
    4. 实时显示 Roll / Pitch / Yaw 姿态角
    5. 坐标轴根据当前数据范围自动缩放

按 Ctrl+C 或关闭窗口退出。
"""

import sys
import argparse
import threading
from collections import deque

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_to_rpy(q_cam):
    """
    将 camera frame (X=right, Y=down, Z=forward) 下的四元数
    转换为 PX4 / MAVLink FRD body frame (X=forward, Y=right, Z=down) 的 Roll/Pitch/Yaw（度，ZYX顺序）。
    
    物理含义（转换后，与 PX4 统一）：
        Roll  = 绕 X(forward) 轴 = 右侧下沉为正
        Pitch = 绕 Y(right)   轴 = 低头为正（nose down）
        Yaw   = 绕 Z(down)    轴 = 向右转为正（从上方看顺时针）
    """
    # 从 camera frame 到 PX4 FRD body frame 的旋转四元数
    # q_body2cam = [-0.5, -0.5, -0.5, 0.5]
    x1, y1, z1, w1 = -0.5, -0.5, -0.5, 0.5
    x2, y2, z2, w2 = q_cam
    
    # 四元数乘法: q_std = q_cam * q_body2cam
    # （camera body→world 后再叠一个 standard→camera 的旋转 = standard→world）
    w = w2*w1 - x2*x1 - y2*y1 - z2*z1
    x = w2*x1 + x2*w1 + y2*z1 - z2*y1
    y = w2*y1 - x2*z1 + y2*w1 + z2*x1
    z = w2*z1 + x2*y1 - y2*x1 + z2*w1
    
    # 标准 ZYX Euler 角
    roll = np.degrees(np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)))
    yaw = np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return roll, pitch, yaw


class RealtimeOdomPlot(Node):
    def __init__(self, max_points=400, show_loop=True):
        super().__init__('realtime_odom_plot')
        
        self.max_points = max_points
        self.show_loop = show_loop
        self.lock = threading.Lock()
        
        # 数据缓冲区（滑动窗口）
        self.t = deque(maxlen=max_points)
        self.vins_x = deque(maxlen=max_points)
        self.vins_y = deque(maxlen=max_points)
        self.vins_z = deque(maxlen=max_points)
        self.vins_roll = deque(maxlen=max_points)
        self.vins_pitch = deque(maxlen=max_points)
        self.vins_yaw = deque(maxlen=max_points)
        
        if show_loop:
            self.loop_x = deque(maxlen=max_points)
            self.loop_y = deque(maxlen=max_points)
            self.loop_z = deque(maxlen=max_points)
            self.loop_roll = deque(maxlen=max_points)
            self.loop_pitch = deque(maxlen=max_points)
            self.loop_yaw = deque(maxlen=max_points)
            self.pos_diff = deque(maxlen=max_points)
            self.angle_diff = deque(maxlen=max_points)
        
        self.start_time = None
        self.frame_count = 0
        
        qos = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(Odometry, '/vins_estimator/odometry', self.vins_cb, qos)
        
        if show_loop:
            self.create_subscription(Odometry, '/odometry_rect', self.loop_cb, qos)
            self.get_logger().info('Waiting for /vins_estimator/odometry and /odometry_rect ...')
        else:
            self.get_logger().info('Waiting for /vins_estimator/odometry ... (--no-loop mode)')
    
    def quat_angle_deg(self, q1, q2):
        """四元数 [x,y,z,w] 夹角（度）"""
        dot = abs(np.dot(q1, q2))
        dot = min(dot, 1.0)
        return 2.0 * np.degrees(np.arccos(dot))
    
    def vins_cb(self, msg):
        with self.lock:
            now = self.get_clock().now().nanoseconds / 1e9
            if self.start_time is None:
                self.start_time = now
            
            self.t.append(now - self.start_time)
            self.vins_x.append(msg.pose.pose.position.x)
            self.vins_y.append(msg.pose.pose.position.y)
            self.vins_z.append(msg.pose.pose.position.z)
            
            q = [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
            self._vins_q = q
            roll, pitch, yaw = quat_to_rpy(q)
            self.vins_roll.append(roll)
            self.vins_pitch.append(pitch)
            self.vins_yaw.append(yaw)
    
    def loop_cb(self, msg):
        with self.lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now().nanoseconds / 1e9
            
            self.loop_x.append(msg.pose.pose.position.x)
            self.loop_y.append(msg.pose.pose.position.y)
            self.loop_z.append(msg.pose.pose.position.z)
            
            q_loop = [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
            roll, pitch, yaw = quat_to_rpy(q_loop)
            self.loop_roll.append(roll)
            self.loop_pitch.append(pitch)
            self.loop_yaw.append(yaw)
            
            if hasattr(self, '_vins_q'):
                angle = self.quat_angle_deg(self._vins_q, q_loop)
                self.angle_diff.append(angle)
            else:
                self.angle_diff.append(0.0)
            
            if len(self.vins_x) > 0 and len(self.loop_x) > 0:
                dx = self.vins_x[-1] - self.loop_x[-1]
                dy = self.vins_y[-1] - self.loop_y[-1]
                dz = self.vins_z[-1] - self.loop_z[-1]
                self.pos_diff.append(np.sqrt(dx*dx + dy*dy + dz*dz))
            else:
                self.pos_diff.append(0.0)
    
    def get_data_copy(self):
        with self.lock:
            if self.show_loop:
                min_len = min(
                    len(self.t),
                    len(self.vins_x), len(self.vins_y), len(self.vins_z),
                    len(self.vins_roll), len(self.vins_pitch), len(self.vins_yaw),
                    len(self.loop_x), len(self.loop_y), len(self.loop_z),
                    len(self.loop_roll), len(self.loop_pitch), len(self.loop_yaw),
                    len(self.pos_diff), len(self.angle_diff)
                )
                return {
                    't': np.array(self.t)[:min_len],
                    'vins_x': np.array(self.vins_x)[:min_len],
                    'vins_y': np.array(self.vins_y)[:min_len],
                    'vins_z': np.array(self.vins_z)[:min_len],
                    'vins_roll': np.array(self.vins_roll)[:min_len],
                    'vins_pitch': np.array(self.vins_pitch)[:min_len],
                    'vins_yaw': np.array(self.vins_yaw)[:min_len],
                    'loop_x': np.array(self.loop_x)[:min_len],
                    'loop_y': np.array(self.loop_y)[:min_len],
                    'loop_z': np.array(self.loop_z)[:min_len],
                    'loop_roll': np.array(self.loop_roll)[:min_len],
                    'loop_pitch': np.array(self.loop_pitch)[:min_len],
                    'loop_yaw': np.array(self.loop_yaw)[:min_len],
                    'pos_diff': np.array(self.pos_diff)[:min_len],
                    'angle_diff': np.array(self.angle_diff)[:min_len],
                }
            else:
                min_len = min(
                    len(self.t),
                    len(self.vins_x), len(self.vins_y), len(self.vins_z),
                    len(self.vins_roll), len(self.vins_pitch), len(self.vins_yaw),
                )
                return {
                    't': np.array(self.t)[:min_len],
                    'vins_x': np.array(self.vins_x)[:min_len],
                    'vins_y': np.array(self.vins_y)[:min_len],
                    'vins_z': np.array(self.vins_z)[:min_len],
                    'vins_roll': np.array(self.vins_roll)[:min_len],
                    'vins_pitch': np.array(self.vins_pitch)[:min_len],
                    'vins_yaw': np.array(self.vins_yaw)[:min_len],
                }


def setup_plots(node, show_loop):
    """初始化 matplotlib 图形"""
    fig, axes = plt.subplots(3, 2, figsize=(14, 11))
    
    if show_loop:
        fig.suptitle('VINS vs Loop Fusion — Real-time Comparison', fontsize=13, fontweight='bold')
    else:
        fig.suptitle('VINS Odometry — Real-time', fontsize=13, fontweight='bold')
    
    lines = {}
    
    if show_loop:
        # ========== 对比模式：3x2 布局 ==========
        ax_xt = axes[0, 0]
        ax_yt = axes[0, 1]
        ax_zt = axes[1, 0]
        ax_pos = axes[1, 1]
        ax_ang = axes[2, 0]
        ax_rpy = axes[2, 1]
        
        # ---- X vs Time ----
        line_vins_xt, = ax_xt.plot([], [], 'b--', linewidth=2.0, alpha=0.9, label='VINS raw')
        line_loop_xt, = ax_xt.plot([], [], 'r-', linewidth=1.5, alpha=0.8, label='Loop corrected')
        ax_xt.set_xlabel('Time (s)', fontsize=10)
        ax_xt.set_ylabel('X (m)', fontsize=10)
        ax_xt.set_title('X Position vs Time', fontsize=11, fontweight='bold')
        ax_xt.legend(loc='best', fontsize=8)
        ax_xt.grid(True, alpha=0.3)
        
        # ---- Y vs Time ----
        line_vins_yt, = ax_yt.plot([], [], 'b--', linewidth=2.0, alpha=0.9, label='VINS raw')
        line_loop_yt, = ax_yt.plot([], [], 'r-', linewidth=1.5, alpha=0.8, label='Loop corrected')
        ax_yt.set_xlabel('Time (s)', fontsize=10)
        ax_yt.set_ylabel('Y (m)', fontsize=10)
        ax_yt.set_title('Y Position vs Time', fontsize=11, fontweight='bold')
        ax_yt.legend(loc='best', fontsize=8)
        ax_yt.grid(True, alpha=0.3)
        
        # ---- Z vs Time ----
        line_vins_zt, = ax_zt.plot([], [], 'b--', linewidth=2.0, alpha=0.9, label='VINS raw')
        line_loop_zt, = ax_zt.plot([], [], 'r-', linewidth=1.2, alpha=0.8, label='Loop corrected')
        ax_zt.set_xlabel('Time (s)', fontsize=10)
        ax_zt.set_ylabel('Z (m)', fontsize=10)
        ax_zt.set_title('Z Position vs Time', fontsize=11, fontweight='bold')
        ax_zt.legend(loc='best', fontsize=8)
        ax_zt.grid(True, alpha=0.3)
        
        # ---- 位置漂移 ----
        line_pos, = ax_pos.plot([], [], 'g-', linewidth=1.5)
        ax_pos.axhline(y=0.05, color='orange', linestyle='--', alpha=0.6, label='5 cm')
        ax_pos.set_xlabel('Time (s)', fontsize=10)
        ax_pos.set_ylabel('Position Diff (m)', fontsize=10)
        ax_pos.set_title('Position Drift', fontsize=11, fontweight='bold')
        ax_pos.legend(loc='best', fontsize=8)
        ax_pos.grid(True, alpha=0.3)
        
        # ---- 角度漂移 ----
        line_ang, = ax_ang.plot([], [], 'm-', linewidth=1.5)
        ax_ang.axhline(y=2.0, color='orange', linestyle='--', alpha=0.6, label='2°')
        ax_ang.set_xlabel('Time (s)', fontsize=10)
        ax_ang.set_ylabel('Angle Diff (deg)', fontsize=10)
        ax_ang.set_title('Angular Drift (Quaternion)', fontsize=11, fontweight='bold')
        ax_ang.legend(loc='best', fontsize=8)
        ax_ang.grid(True, alpha=0.3)
        
        # ---- Roll / Pitch / Yaw ----
        line_vins_r,  = ax_rpy.plot([], [], 'b--', linewidth=1.5, alpha=0.9, label='VINS Roll')
        line_vins_p,  = ax_rpy.plot([], [], 'c--', linewidth=1.5, alpha=0.9, label='VINS Pitch')
        line_vins_yw, = ax_rpy.plot([], [], 'm--', linewidth=1.5, alpha=0.9, label='VINS Yaw')
        line_loop_r,  = ax_rpy.plot([], [], 'r-',  linewidth=1.2, alpha=0.8, label='Loop Roll')
        line_loop_p,  = ax_rpy.plot([], [], 'orange',  linewidth=1.2, alpha=0.8, linestyle='-', label='Loop Pitch')
        line_loop_yw, = ax_rpy.plot([], [], 'purple',  linewidth=1.2, alpha=0.8, linestyle='-', label='Loop Yaw')
        ax_rpy.set_xlabel('Time (s)', fontsize=10)
        ax_rpy.set_ylabel('Angle (deg)', fontsize=10)
        ax_rpy.set_title('Roll / Pitch / Yaw', fontsize=11, fontweight='bold')
        ax_rpy.legend(loc='best', fontsize=7, ncol=2)
        ax_rpy.grid(True, alpha=0.3)
        
        lines = {
            'vins_xt': line_vins_xt, 'loop_xt': line_loop_xt,
            'vins_yt': line_vins_yt, 'loop_yt': line_loop_yt,
            'vins_zt': line_vins_zt, 'loop_zt': line_loop_zt,
            'pos': line_pos, 'ang': line_ang,
            'vins_r': line_vins_r, 'vins_p': line_vins_p, 'vins_yw': line_vins_yw,
            'loop_r': line_loop_r, 'loop_p': line_loop_p, 'loop_yw': line_loop_yw,
        }
        
    else:
        # ========== 单 VINS 模式：3x2 布局，6个独立图 ==========
        ax_xt = axes[0, 0]
        ax_yt = axes[0, 1]
        ax_zt = axes[1, 0]
        ax_roll = axes[1, 1]
        ax_pitch = axes[2, 0]
        ax_yaw = axes[2, 1]
        
        # ---- X vs Time ----
        line_vins_xt, = ax_xt.plot([], [], 'b-', linewidth=1.5, alpha=0.9, label='VINS raw')
        ax_xt.set_xlabel('Time (s)', fontsize=10)
        ax_xt.set_ylabel('X (m)', fontsize=10)
        ax_xt.set_title('X Position vs Time', fontsize=11, fontweight='bold')
        ax_xt.legend(loc='best', fontsize=8)
        ax_xt.grid(True, alpha=0.3)
        
        # ---- Y vs Time ----
        line_vins_yt, = ax_yt.plot([], [], 'b-', linewidth=1.5, alpha=0.9, label='VINS raw')
        ax_yt.set_xlabel('Time (s)', fontsize=10)
        ax_yt.set_ylabel('Y (m)', fontsize=10)
        ax_yt.set_title('Y Position vs Time', fontsize=11, fontweight='bold')
        ax_yt.legend(loc='best', fontsize=8)
        ax_yt.grid(True, alpha=0.3)
        
        # ---- Z vs Time ----
        line_vins_zt, = ax_zt.plot([], [], 'b-', linewidth=1.5, alpha=0.9, label='VINS raw')
        ax_zt.set_xlabel('Time (s)', fontsize=10)
        ax_zt.set_ylabel('Z (m)', fontsize=10)
        ax_zt.set_title('Z Position vs Time', fontsize=11, fontweight='bold')
        ax_zt.legend(loc='best', fontsize=8)
        ax_zt.grid(True, alpha=0.3)
        
        # ---- Roll vs Time ----
        line_vins_r, = ax_roll.plot([], [], 'r-', linewidth=1.5, alpha=0.9, label='VINS Roll')
        ax_roll.set_xlabel('Time (s)', fontsize=10)
        ax_roll.set_ylabel('Roll (deg)', fontsize=10)
        ax_roll.set_title('Roll vs Time', fontsize=11, fontweight='bold')
        ax_roll.legend(loc='best', fontsize=8)
        ax_roll.grid(True, alpha=0.3)
        
        # ---- Pitch vs Time ----
        line_vins_p, = ax_pitch.plot([], [], 'g-', linewidth=1.5, alpha=0.9, label='VINS Pitch')
        ax_pitch.set_xlabel('Time (s)', fontsize=10)
        ax_pitch.set_ylabel('Pitch (deg)', fontsize=10)
        ax_pitch.set_title('Pitch vs Time', fontsize=11, fontweight='bold')
        ax_pitch.legend(loc='best', fontsize=8)
        ax_pitch.grid(True, alpha=0.3)
        
        # ---- Yaw vs Time ----
        line_vins_yw, = ax_yaw.plot([], [], 'm-', linewidth=1.5, alpha=0.9, label='VINS Yaw')
        ax_yaw.set_xlabel('Time (s)', fontsize=10)
        ax_yaw.set_ylabel('Yaw (deg)', fontsize=10)
        ax_yaw.set_title('Yaw vs Time', fontsize=11, fontweight='bold')
        ax_yaw.legend(loc='best', fontsize=8)
        ax_yaw.grid(True, alpha=0.3)
        
        lines = {
            'vins_xt': line_vins_xt,
            'vins_yt': line_vins_yt,
            'vins_zt': line_vins_zt,
            'vins_r': line_vins_r,
            'vins_p': line_vins_p,
            'vins_yw': line_vins_yw,
        }
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    return fig, axes, lines


def update(frame, node, lines, axes, show_loop):
    """动画更新回调"""
    data = node.get_data_copy()
    
    if len(data['t']) < 2:
        return list(lines.values())
    
    t = data['t']
    
    # ---- 更新 X vs Time ----
    lines['vins_xt'].set_data(t, data['vins_x'])
    axes[0, 0].set_xlim(t[0], max(t[-1], t[0] + 1))
    if len(data['vins_x']) > 0:
        if show_loop and len(data.get('loop_x', [])) > 0:
            x_min = min(np.min(data['vins_x']), np.min(data['loop_x']))
            x_max = max(np.max(data['vins_x']), np.max(data['loop_x']))
        else:
            x_min, x_max = np.min(data['vins_x']), np.max(data['vins_x'])
        margin = max((x_max - x_min) * 0.1, 0.1)
        axes[0, 0].set_ylim(x_min - margin, x_max + margin)
    
    # ---- 更新 Y vs Time ----
    lines['vins_yt'].set_data(t, data['vins_y'])
    axes[0, 1].set_xlim(t[0], max(t[-1], t[0] + 1))
    if len(data['vins_y']) > 0:
        if show_loop and len(data.get('loop_y', [])) > 0:
            y_min = min(np.min(data['vins_y']), np.min(data['loop_y']))
            y_max = max(np.max(data['vins_y']), np.max(data['loop_y']))
        else:
            y_min, y_max = np.min(data['vins_y']), np.max(data['vins_y'])
        margin = max((y_max - y_min) * 0.1, 0.1)
        axes[0, 1].set_ylim(y_min - margin, y_max + margin)
    
    # ---- 更新 Z vs Time ----
    lines['vins_zt'].set_data(t, data['vins_z'])
    axes[1, 0].set_xlim(t[0], max(t[-1], t[0] + 1))
    if len(data['vins_z']) > 0:
        if show_loop and len(data.get('loop_z', [])) > 0:
            z_min = min(np.min(data['vins_z']), np.min(data['loop_z']))
            z_max = max(np.max(data['vins_z']), np.max(data['loop_z']))
            axes[1, 0].set_ylim(z_min - 0.2, z_max + 0.2)
        else:
            z_min, z_max = np.min(data['vins_z']), np.max(data['vins_z'])
            axes[1, 0].set_ylim(z_min - 0.2, z_max + 0.2)
    
    if show_loop:
        # ---- 更新 Loop 位置 ----
        lines['loop_xt'].set_data(t, data['loop_x'])
        lines['loop_yt'].set_data(t, data['loop_y'])
        lines['loop_zt'].set_data(t, data['loop_z'])
        
        # ---- 更新位置漂移 ----
        lines['pos'].set_data(t, data['pos_diff'])
        axes[1, 1].set_xlim(t[0], max(t[-1], t[0] + 1))
        if len(data['pos_diff']) > 0:
            pmax = np.max(data['pos_diff'])
            axes[1, 1].set_ylim(0, max(pmax * 1.2, 0.1))
        
        # ---- 更新角度漂移 ----
        lines['ang'].set_data(t, data['angle_diff'])
        axes[2, 0].set_xlim(t[0], max(t[-1], t[0] + 1))
        if len(data['angle_diff']) > 0:
            amax = np.max(data['angle_diff'])
            axes[2, 0].set_ylim(0, max(amax * 1.2, 2.0))
        
        # ---- 更新 Roll/Pitch/Yaw ----
        lines['vins_r'].set_data(t, data['vins_roll'])
        lines['vins_p'].set_data(t, data['vins_pitch'])
        lines['vins_yw'].set_data(t, data['vins_yaw'])
        lines['loop_r'].set_data(t, data['loop_roll'])
        lines['loop_p'].set_data(t, data['loop_pitch'])
        lines['loop_yw'].set_data(t, data['loop_yaw'])
        axes[2, 1].set_xlim(t[0], max(t[-1], t[0] + 1))
        
        all_rpy = np.concatenate([
            data['vins_roll'], data['vins_pitch'], data['vins_yaw'],
            data['loop_roll'], data['loop_pitch'], data['loop_yaw']
        ])
        if len(all_rpy) > 0:
            rpy_min, rpy_max = np.min(all_rpy), np.max(all_rpy)
            margin = max((rpy_max - rpy_min) * 0.1, 5.0)
            axes[2, 1].set_ylim(rpy_min - margin, rpy_max + margin)
        
        # 更新窗口标题
        if len(data['pos_diff']) > 0:
            fig = axes[0, 0].figure
            fig.suptitle(
                f'VINS vs Loop Fusion — Real-time | '
                f'Points: {len(t)} | '
                f'Max Pos: {np.max(data["pos_diff"]):.3f}m | '
                f'Max Ang: {np.max(data["angle_diff"]):.2f}°',
                fontsize=13, fontweight='bold'
            )
    else:
        # ---- 更新 Roll/Pitch/Yaw（单 VINS 模式） ----
        lines['vins_r'].set_data(t, data['vins_roll'])
        lines['vins_p'].set_data(t, data['vins_pitch'])
        lines['vins_yw'].set_data(t, data['vins_yaw'])
        
        axes[1, 1].set_xlim(t[0], max(t[-1], t[0] + 1))
        if len(data['vins_roll']) > 0:
            r_min, r_max = np.min(data['vins_roll']), np.max(data['vins_roll'])
            margin = max((r_max - r_min) * 0.1, 5.0)
            axes[1, 1].set_ylim(r_min - margin, r_max + margin)
        
        axes[2, 0].set_xlim(t[0], max(t[-1], t[0] + 1))
        if len(data['vins_pitch']) > 0:
            p_min, p_max = np.min(data['vins_pitch']), np.max(data['vins_pitch'])
            margin = max((p_max - p_min) * 0.1, 5.0)
            axes[2, 0].set_ylim(p_min - margin, p_max + margin)
        
        axes[2, 1].set_xlim(t[0], max(t[-1], t[0] + 1))
        if len(data['vins_yaw']) > 0:
            y_min, y_max = np.min(data['vins_yaw']), np.max(data['vins_yaw'])
            margin = max((y_max - y_min) * 0.1, 5.0)
            axes[2, 1].set_ylim(y_min - margin, y_max + margin)
        
        # 更新窗口标题
        fig = axes[0, 0].figure
        fig.suptitle(
            f'VINS Odometry — Real-time | Points: {len(t)}',
            fontsize=13, fontweight='bold'
        )
    
    return list(lines.values())


def main():
    parser = argparse.ArgumentParser(description='Real-time VINS odometry plotter')
    parser.add_argument('--no-loop', action='store_true',
                        help='Only show VINS raw values, hide Loop Fusion correction')
    args = parser.parse_args()
    
    show_loop = not args.no_loop
    
    rclpy.init()
    node = RealtimeOdomPlot(max_points=400, show_loop=show_loop)
    
    # ROS2 spin 放在后台线程
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    fig, axes, lines = setup_plots(node, show_loop=show_loop)
    
    # 绑定关闭事件
    def on_close(event):
        node.get_logger().info('Window closed, shutting down...')
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)
        sys.exit(0)
    
    fig.canvas.mpl_connect('close_event', on_close)
    
    # 启动动画
    ani = FuncAnimation(
        fig, update, fargs=(node, lines, axes, show_loop),
        interval=200,  # 200ms = 5 Hz 刷新
        blit=False, cache_frame_data=False
    )
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
