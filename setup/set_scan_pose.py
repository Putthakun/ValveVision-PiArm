# set_scan_pose.py — จัดท่า scan pose แบบ interactive แล้วบันทึกลง config.py
#
# วิธีใช้:  python set_scan_pose.py
#
# ปุ่ม:
#   1 2 3 4   → เลือก joint (J1/J2/J3/J4)
#   + หรือ =  → เพิ่มมุม
#   - หรือ _  → ลดมุม
#   [ ] \     → step 1° / 5° / 10°
#   s         → บันทึกลง config.py และออก
#   q         → ออกโดยไม่บันทึก

import re
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import tty
import termios
from servo_controller import ServoController
from config import SCAN_POSE

# ── helpers ────────────────────────────────────────────────────────────────

def getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def update_config(pose: dict):
    with open('config.py', 'r') as f:
        content = f.read()

    new_block = (
        "SCAN_POSE = {\n"
        f"    'J1': {pose['J1']:.1f},\n"
        f"    'J2': {pose['J2']:.1f},\n"
        f"    'J3': {pose['J3']:.1f},\n"
        f"    'J4': {pose['J4']:.1f},\n"
        f"    'J5': 90.0,\n"
        f"    'J6': 90.0,\n"
        "}"
    )

    content = re.sub(
        r'SCAN_POSE\s*=\s*\{[^}]*\}',
        new_block,
        content,
        flags=re.DOTALL,
    )

    with open('config.py', 'w') as f:
        f.write(content)


def show(pose, current, step):
    joints = ['J1', 'J2', 'J3', 'J4']
    print(f"\r  step={step:2}°  |  ", end='')
    for j in joints:
        mark = '►' if j == current else ' '
        print(f"{mark}{j}={pose[j]:5.1f}°", end='  ')
    print(' ' * 5, end='', flush=True)


# ── main ───────────────────────────────────────────────────────────────────

arm  = ServoController()
pose = {k: float(v) for k, v in SCAN_POSE.items()}

arm.move_smooth(pose)

print("=" * 55)
print("SET SCAN POSE")
print("=" * 55)
print("  1 2 3 4   เลือก joint")
print("  + / -     เพิ่ม / ลด มุม")
print("  [ ] \\     step  1° / 5° / 10°")
print("  s         บันทึก")
print("  q         ออกไม่บันทึก")
print()

joints  = ['J1', 'J2', 'J3', 'J4']
current = 'J1'
step    = 5

show(pose, current, step)

while True:
    ch = getch()

    if ch == 'q':
        print("\nออกโดยไม่บันทึก")
        break

    elif ch == 's':
        print(f"\n\nบันทึก SCAN_POSE → {pose}")
        update_config(pose)
        print("อัปเดต config.py เรียบร้อย ✅")
        break

    elif ch in ('1', '2', '3', '4'):
        current = f'J{ch}'

    elif ch in ('+', '='):
        pose[current] = min(180.0, pose[current] + step)
        arm.move_smooth(pose)

    elif ch in ('-', '_'):
        pose[current] = max(0.0, pose[current] - step)
        arm.move_smooth(pose)

    elif ch == '[':
        step = 1
    elif ch == ']':
        step = 5
    elif ch == '\\':
        step = 10

    show(pose, current, step)

arm.move_to_home()
