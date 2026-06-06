#!/usr/bin/env python3
"""
plot_odom_compare.py
====================
离线可视化 VINS 原始轨迹 vs Loop Fusion 回环修正轨迹。

用法：
    python3 plot_odom_compare.py ~/odom_compare_result.csv
    
功能：
    1. 2D 俯视图（X-Y 平面）—— 最直观观察回环修正效果
    2. 高度 Z 随时间变化
    3. 位置漂移量随时间变化
    4. 角度漂移量随时间变化
    
坐标轴自动根据数据范围缩放，无需手动设置范围。
"""

import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


def load_csv(path):
    """读取 odom_compare.py 生成的 CSV 文件"""
    data = {
        't': [], 'vins_x': [], 'vins_y': [], 'vins_z': [],
        'loop_x': [], 'loop_y': [], 'loop_z': [],
        'pos_diff': [], 'angle_diff': []
    }
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data['t'].append(float(row['timestamp']))
            data['vins_x'].append(float(row['vins_x']))
            data['vins_y'].append(float(row['vins_y']))
            data['vins_z'].append(float(row['vins_z']))
            data['loop_x'].append(float(row['loop_x']))
            data['loop_y'].append(float(row['loop_y']))
            data['loop_z'].append(float(row['loop_z']))
            data['pos_diff'].append(float(row['pos_diff_m']))
            data['angle_diff'].append(float(row['angle_diff_deg']))
    return {k: np.array(v) for k, v in data.items()}


def plot_comparison(data):
    """绘制四幅子图"""
    t = data['t']
    t0 = t[0]
    t_rel = t - t0  # 相对时间，从 0 开始
    
    fig = plt.figure(figsize=(14, 10))
    gs = GridSpec(2, 2, figure=fig, hspace=0.3, wspace=0.25)
    
    # ---------- 1. 2D 俯视图 (X-Y) ----------
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(data['vins_x'], data['vins_y'], 'b-', linewidth=1.2, alpha=0.8,
             label='VINS raw (no loop)')
    ax1.plot(data['loop_x'], data['loop_y'], 'r-', linewidth=1.2, alpha=0.8,
             label='Loop Fusion corrected')
    
    # 标记起点和终点
    ax1.scatter(data['vins_x'][0], data['vins_y'][0], c='green', s=80, marker='o',
                zorder=5, label='Start')
    ax1.scatter(data['vins_x'][-1], data['vins_y'][-1], c='blue', s=80, marker='x',
                zorder=5, label='VINS end')
    ax1.scatter(data['loop_x'][-1], data['loop_y'][-1], c='red', s=80, marker='x',
                zorder=5, label='Loop end')
    
    ax1.set_xlabel('X (m)', fontsize=11)
    ax1.set_ylabel('Y (m)', fontsize=11)
    ax1.set_title('Top View (X-Y Trajectory)', fontsize=12, fontweight='bold')
    ax1.legend(loc='best', fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal', adjustable='datalim')  # 等比例缩放
    
    # ---------- 2. 高度 Z 随时间变化 ----------
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t_rel, data['vins_z'], 'b-', linewidth=1.2, alpha=0.8, label='VINS raw')
    ax2.plot(t_rel, data['loop_z'], 'r-', linewidth=1.2, alpha=0.8, label='Loop corrected')
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Z (m)', fontsize=11)
    ax2.set_title('Height (Z) over Time', fontsize=12, fontweight='bold')
    ax2.legend(loc='best', fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    # ---------- 3. 位置漂移量 ----------
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(t_rel, data['pos_diff'], 'g-', linewidth=1.5)
    ax3.axhline(y=0.05, color='orange', linestyle='--', alpha=0.7, label='5 cm threshold')
    ax3.fill_between(t_rel, data['pos_diff'], alpha=0.2, color='green')
    ax3.set_xlabel('Time (s)', fontsize=11)
    ax3.set_ylabel('Position Diff (m)', fontsize=11)
    ax3.set_title('Position Drift (Loop vs VINS)', fontsize=12, fontweight='bold')
    ax3.legend(loc='best', fontsize=9)
    ax3.grid(True, alpha=0.3)
    
    # 标记最大漂移点
    max_idx = np.argmax(data['pos_diff'])
    ax3.scatter(t_rel[max_idx], data['pos_diff'][max_idx], c='red', s=60, zorder=5)
    ax3.annotate(f'Max: {data["pos_diff"][max_idx]:.3f} m\n@ {t_rel[max_idx]:.1f} s',
                 xy=(t_rel[max_idx], data['pos_diff'][max_idx]),
                 xytext=(10, 10), textcoords='offset points',
                 fontsize=9, color='red',
                 arrowprops=dict(arrowstyle='->', color='red', alpha=0.7))
    
    # ---------- 4. 角度漂移量 ----------
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(t_rel, data['angle_diff'], 'm-', linewidth=1.5)
    ax4.axhline(y=2.0, color='orange', linestyle='--', alpha=0.7, label='2° threshold')
    ax4.fill_between(t_rel, data['angle_diff'], alpha=0.2, color='magenta')
    ax4.set_xlabel('Time (s)', fontsize=11)
    ax4.set_ylabel('Angle Diff (deg)', fontsize=11)
    ax4.set_title('Angular Drift (Loop vs VINS)', fontsize=12, fontweight='bold')
    ax4.legend(loc='best', fontsize=9)
    ax4.grid(True, alpha=0.3)
    
    # 标记最大角度漂移点
    max_a_idx = np.argmax(data['angle_diff'])
    ax4.scatter(t_rel[max_a_idx], data['angle_diff'][max_a_idx], c='red', s=60, zorder=5)
    ax4.annotate(f'Max: {data["angle_diff"][max_a_idx]:.2f}°\n@ {t_rel[max_a_idx]:.1f} s',
                 xy=(t_rel[max_a_idx], data['angle_diff'][max_a_idx]),
                 xytext=(10, 10), textcoords='offset points',
                 fontsize=9, color='red',
                 arrowprops=dict(arrowstyle='->', color='red', alpha=0.7))
    
    # 总标题
    duration = t_rel[-1]
    fig.suptitle(f'VINS vs Loop Fusion Comparison\nDuration: {duration:.1f}s | '
                 f'Max Pos Drift: {np.max(data["pos_diff"]):.3f}m | '
                 f'Max Angle Drift: {np.max(data["angle_diff"]):.2f}°',
                 fontsize=14, fontweight='bold', y=0.98)
    
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    
    # 保存图片
    out_png = sys.argv[1].replace('.csv', '.png')
    plt.savefig(out_png, dpi=150, bbox_inches='tight')
    print(f'Plot saved to: {out_png}')
    
    plt.show()


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 plot_odom_compare.py <csv_file>')
        print('Example: python3 plot_odom_compare.py ~/odom_compare_result.csv')
        sys.exit(1)
    
    csv_path = sys.argv[1]
    print(f'Loading {csv_path} ...')
    data = load_csv(csv_path)
    
    print(f'Loaded {len(data["t"])} samples, '
          f'duration: {data["t"][-1] - data["t"][0]:.1f}s')
    print(f'Max position drift : {np.max(data["pos_diff"]):.4f} m')
    print(f'Max angle drift    : {np.max(data["angle_diff"]):.3f} deg')
    
    plot_comparison(data)


if __name__ == '__main__':
    main()
