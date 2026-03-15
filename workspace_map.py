# workspace_map.py — วาด workspace ของแขนกล บันทึกเป็น workspace.png
#
# วิธีใช้:
#   python workspace_map.py

import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

from ik_solver import solve_ik
from config import L1, L2, L3, L4

# ── sweep พิกัด ───────────────────────────────────────────────────────────
r_range = range(0, 420, 5)    # ระยะแนวนอนจากแกน J1 (mm)
z_range = range(0, 280, 5)    # ความสูง (mm)

reach, no_reach = [], []

for z in z_range:
    for r in r_range:
        result = solve_ik(r, 0, z)
        if result:
            reach.append((r, z))
        else:
            no_reach.append((r, z))

rx, rz = zip(*reach) if reach else ([], [])
nx, nz = zip(*no_reach) if no_reach else ([], [])

# ── plot ──────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(10, 7))

ax.scatter(nx, nz, c='#e0e0e0', s=8, label='ไม่ถึง')
ax.scatter(rx, rz, c='#2196F3', s=8, label='ถึงได้')

# วาดแขน ที่ home pose (ทุก joint 90°)
ax.plot([0, 0], [0, L1], 'k-', lw=3)                          # base
ax.plot([0, 0], [L1, L1+L2], color='#F44336', lw=4)           # L2
ax.plot([0, 0], [L1+L2, L1+L2+L3], color='#FF9800', lw=4)     # L3
ax.plot([0, 0], [L1+L2+L3, L1+L2+L3+L4], color='#4CAF50', lw=4) # L4
ax.plot(0, 0, 'ks', ms=10, zorder=5)
ax.plot(0, L1+L2+L3+L4, 'g^', ms=10, zorder=5, label=f'home tip (0, {L1+L2+L3+L4})')

# แกน
ax.axhline(0, color='gray', lw=0.5)
ax.axvline(0, color='gray', lw=0.5)

# labels
ax.set_xlabel('x / r  (mm)  ระยะแนวนอนจากแกน J1', fontsize=11)
ax.set_ylabel('z  (mm)  ความสูงจากฐาน', fontsize=11)
ax.set_title('Workspace — มุมมองด้านข้าง (y=0, gripper แนวนอน)', fontsize=13)
ax.set_xlim(-20, 420)
ax.set_ylim(-10, 280)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.legend(fontsize=10)

# annotation ข้อมูล link
info = (f'L1={L1}  L2={L2}  L3={L3}  L4={L4} mm\n'
        f'max reach = {L2+L3+L4} mm จาก J2')
ax.text(0.02, 0.97, info, transform=ax.transAxes,
        fontsize=9, va='top', family='monospace',
        bbox=dict(boxstyle='round', fc='white', alpha=0.8))

plt.tight_layout()
plt.savefig('workspace.png', dpi=150)
print("บันทึกแล้ว → workspace.png")
print(f"จุดที่ถึงได้: {len(reach)}  |  ถึงไม่ได้: {len(no_reach)}")
