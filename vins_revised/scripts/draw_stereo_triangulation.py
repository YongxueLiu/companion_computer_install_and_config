#!/usr/bin/env python3
"""
Draw a diagram explaining stereo triangulation:
    Z = f_x * b / d
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Arc

fig, ax = plt.subplots(1, 1, figsize=(10, 7))

# ------------------------------------------------------------
# Geometry parameters (in some abstract units)
# ------------------------------------------------------------
b = 3.0       # baseline
Z = 5.0       # depth to point P
f = 2.5       # focal length (abstract)
img_w = 2.0   # image plane half-width

# Camera centers
OL = np.array([0, 0])
OR = np.array([b, 0])

# 3D point P (in front of cameras)
# We place P such that it projects to the image planes nicely
# Let P_x = b/2  (centered between cameras)
# Then from similar triangles:
#   u_L = f * (P_x - 0) / Z   => P_x = u_L * Z / f
#   u_R = f * (P_x - b) / Z
# Let's pick u_L = 0.6, then P_x = 0.6 * 5 / 2.5 = 1.2
uL = 0.6
P_x = uL * Z / f          # 1.2
uR = f * (P_x - b) / Z    # 2.5 * (1.2 - 3) / 5 = -0.9

P = np.array([P_x, Z])

# Image planes (placed at y = f for clarity, though in reality
# they could be behind the cameras; we just need the geometry)
img_y = f

# Projection points on image planes
pL = np.array([OL[0] + uL, img_y])
pR = np.array([OR[0] + uR, img_y])

# ------------------------------------------------------------
# Draw cameras
# ------------------------------------------------------------
cam_size = 0.25
def draw_camera(ax, center, color='k'):
    # Simple box + lens representation
    cx, cy = center
    # body
    rect = plt.Rectangle((cx - cam_size/2, cy - cam_size/3),
                          cam_size, cam_size*0.7,
                          fill=True, facecolor=color, edgecolor='k', linewidth=1.5)
    ax.add_patch(rect)
    # lens
    lens = plt.Rectangle((cx - cam_size/4, cy + cam_size*0.3),
                          cam_size/2, cam_size*0.25,
                          fill=True, facecolor='#888888', edgecolor='k')
    ax.add_patch(lens)
    # optical axis dot
    ax.plot(cx, cy, 'ko', markersize=4)

draw_camera(ax, OL, color='#4472C4')
draw_camera(ax, OR, color='#C5504B')

# Labels for optical centers
ax.text(OL[0], -0.45, r'$O_L$ (left camera)', ha='center', fontsize=12, fontweight='bold')
ax.text(OR[0], -0.45, r'$O_R$ (right camera)', ha='center', fontsize=12, fontweight='bold')

# ------------------------------------------------------------
# Draw image planes
# ------------------------------------------------------------
ax.plot([OL[0] - img_w, OL[0] + img_w], [img_y, img_y], 'b--', linewidth=1.5, alpha=0.7)
ax.plot([OR[0] - img_w, OR[0] + img_w], [img_y, img_y], 'r--', linewidth=1.5, alpha=0.7)
ax.text(OL[0] - img_w - 0.15, img_y, 'image plane', ha='right', va='center', fontsize=9, color='b')
ax.text(OR[0] + img_w + 0.15, img_y, 'image plane', ha='left', va='center', fontsize=9, color='r')

# ------------------------------------------------------------
# Draw point P and projection rays
# ------------------------------------------------------------
ax.plot(P[0], P[1], 'g*', markersize=18, zorder=5)
ax.text(P[0] + 0.15, P[1] + 0.15, r'$P(X, Y, Z)$', fontsize=13, color='darkgreen', fontweight='bold')

# Rays: P -> OL, P -> OR
ax.plot([P[0], OL[0]], [P[1], OL[1]], 'g-', linewidth=1.2, alpha=0.6)
ax.plot([P[0], OR[0]], [P[1], OR[1]], 'g-', linewidth=1.2, alpha=0.6)

# Continue rays to image planes
ax.plot([OL[0], pL[0]], [OL[1], pL[1]], 'g--', linewidth=1.0, alpha=0.4)
ax.plot([OR[0], pR[0]], [OR[1], pR[1]], 'g--', linewidth=1.0, alpha=0.4)

# Projection points
ax.plot(pL[0], pL[1], 'bo', markersize=8, zorder=5)
ax.plot(pR[0], pR[1], 'ro', markersize=8, zorder=5)
ax.text(pL[0], pL[1] + 0.25, r'$p_L$', fontsize=12, color='b', ha='center', fontweight='bold')
ax.text(pR[0], pR[1] + 0.25, r'$p_R$', fontsize=12, color='r', ha='center', fontweight='bold')

# ------------------------------------------------------------
# Draw baseline and depth annotations
# ------------------------------------------------------------
# Baseline b
ax.annotate('', xy=(OR[0], -0.9), xytext=(OL[0], -0.9),
            arrowprops=dict(arrowstyle='<->', color='purple', lw=2))
ax.text((OL[0]+OR[0])/2, -1.15, r'Baseline $b$', ha='center', fontsize=12,
        color='purple', fontweight='bold')

# Depth Z
ax.annotate('', xy=(P[0] + 0.8, P[1]), xytext=(P[0] + 0.8, OL[1]),
            arrowprops=dict(arrowstyle='<->', color='darkgreen', lw=2))
ax.text(P[0] + 1.0, Z/2, r'Depth $Z$', ha='left', va='center', fontsize=12,
        color='darkgreen', fontweight='bold', rotation=90)

# ------------------------------------------------------------
# Draw disparity d on the image plane
# ------------------------------------------------------------
# Draw horizontal bracket for d = uL - uR (projected onto a common line)
# For clarity, draw a reference line at y = img_y + 0.5
d_y = img_y + 0.6
ax.plot([pL[0], pR[0]], [d_y, d_y], 'k-', linewidth=1.0)
ax.plot([pL[0], pL[0]], [d_y - 0.08, d_y + 0.08], 'k-', linewidth=1.5)
ax.plot([pR[0], pR[0]], [d_y - 0.08, d_y + 0.08], 'k-', linewidth=1.5)
ax.text((pL[0] + pR[0])/2, d_y + 0.25,
        r'Disparity $d = u_L - u_R$', ha='center', fontsize=12,
        fontweight='bold')

# ------------------------------------------------------------
# Draw focal length f
# ------------------------------------------------------------
ax.annotate('', xy=(OL[0] - 0.5, img_y), xytext=(OL[0] - 0.5, OL[1]),
            arrowprops=dict(arrowstyle='<->', color='gray', lw=1.5))
ax.text(OL[0] - 0.75, img_y/2, r'$f_x$', ha='center', va='center',
        fontsize=12, color='gray', fontweight='bold', rotation=90)

# ------------------------------------------------------------
# Highlight similar triangles for derivation
# ------------------------------------------------------------
# Left camera: triangle (OL, projection on axis at y=Z, P)
# The key insight: triangle (OL, pL, point directly above OL at y=img_y)
#                  is similar to triangle (OL, P_proj_on_axis, P)
# We'll draw the similar triangles lightly shaded

from matplotlib.patches import Polygon

# Left similar triangle (simplified: just highlight the two triangles)
tri_L1 = np.array([[OL[0], OL[1]], [pL[0], pL[1]], [OL[0], pL[1]]])
tri_L2 = np.array([[OL[0], OL[1]], [P[0], P[1]], [OL[0], P[1]]])

ax.add_patch(Polygon(tri_L1, closed=True, fill=True, facecolor='blue', alpha=0.08, edgecolor='b', linewidth=0.5))
ax.add_patch(Polygon(tri_L2, closed=True, fill=True, facecolor='blue', alpha=0.08, edgecolor='b', linewidth=0.5))

# Small arc to indicate equal angles
arc1 = Arc((OL[0], OL[1]), 0.6, 0.6, angle=0, theta1=90, theta2=np.degrees(np.arctan2(uL, f)), color='b', lw=1)
ax.add_patch(arc1)
arc2 = Arc((OL[0], OL[1]), 1.2, 1.2, angle=0, theta1=90, theta2=np.degrees(np.arctan2(P[0]-OL[0], Z)), color='b', lw=1)
ax.add_patch(arc2)

# ------------------------------------------------------------
# Formula box
# ------------------------------------------------------------
formula_text = (
    r'Similar triangles:  $\frac{Z}{b} = \frac{f_x}{d}$' + '\n' +
    r'$\Downarrow$' + '\n' +
    r'$Z = \frac{f_x \cdot b}{u_L - u_R}$'
)
ax.text(0.02, 0.98, formula_text, transform=ax.transAxes,
        fontsize=14, verticalalignment='top',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9, edgecolor='black', linewidth=1.5),
        family='monospace')

# ------------------------------------------------------------
# Extra annotation explaining the principle
# ------------------------------------------------------------
principle_text = (
    r'Key insight: A point closer to the camera (smaller $Z$)'
    '\n'
    r'produces larger disparity $d$ on the image plane.'
)
ax.text(0.98, 0.02, principle_text, transform=ax.transAxes,
        fontsize=10, verticalalignment='bottom', horizontalalignment='right',
        style='italic', color='dimgray')

# ------------------------------------------------------------
# Axis setup
# ------------------------------------------------------------
ax.set_aspect('equal')
ax.set_xlim(-1.5, 5.5)
ax.set_ylim(-1.8, 6.5)
ax.set_xlabel('X (camera coordinate)', fontsize=11)
ax.set_ylabel('Z (depth, forward)', fontsize=11)
ax.set_title('Stereo Triangulation Geometry', fontsize=14, fontweight='bold', pad=15)
ax.grid(True, alpha=0.2)

# Remove top and right spines for cleaner look
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

plt.tight_layout()
plt.savefig('/home/lyx/VINS-Fusion-ROS2/tutorial/stereo_triangulation.png', dpi=200, bbox_inches='tight')
print("Saved to tutorial/stereo_triangulation.png")
