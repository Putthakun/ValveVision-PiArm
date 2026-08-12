# tools/find_joint_limits.py
# หาขีดจำกัดจริงของแต่ละ joint (ของโครงแขน ไม่ใช่ของ servo)
#
# วิธีใช้ (รันบน Pi เท่านั้น เพราะต้องสั่ง servo จริง):
#   python3 tools/find_joint_limits.py
#
# ปุ่มบังคับ (ขณะปรับแต่ละ joint):
#   +  หรือ =   ขยับ +step
#   -           ขยับ -step
#   [           ตั้งขนาดก้าวเป็น 1°
#   ]           ตั้งขนาดก้าวเป็น 5°
#   l           บันทึกขีดล่าง ณ ตำแหน่งปัจจุบัน (จุดที่แขนเริ่มชน/ฝืน)
#   u           บันทึกขีดบน ณ ตำแหน่งปัจจุบัน
#   n           ไป joint ถัดไป (ต้องบันทึกครบทั้ง l และ u ก่อน ไม่งั้นจะเตือน)
#   q           หยุดกลางคันแล้วสรุปผลเท่าที่มี
#
# ★ สคริปต์จะหักระยะปลอดภัย 10° เข้ามาจากจุดที่ชนจริงให้อัตโนมัติ
#   (ขีดล่าง += 10, ขีดบน -= 10) ก่อนพิมพ์ค่าสุดท้าย — ไม่ต้องหักเอง

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import termios
import tty

from servo_controller import ServoController
from config import LIMITS as OLD_LIMITS

SAFETY_MARGIN = 10  # องศา — หักเข้ามาจากจุดที่ชนจริง

# J5, J6 ไม่ได้ใช้ในงานนี้ ปล่อยไว้ตามค่าเดิม (0,180) — ดู Task 1 ใน PLAN.md
JOINTS = ['J1', 'J2', 'J3', 'J4']
JOINT_NAMES = {
    'J1': 'Base (หมุนฐาน)',
    'J2': 'Shoulder (ยกไหล่)',
    'J3': 'Elbow (งอศอก)',
    'J4': 'Wrist Pitch (ก้มเงย)',
}

ctrl = ServoController()


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def park_others(active_joint: str):
    """ส่ง joint อื่นๆ ไปที่ 90° (ท่ากลาง) ให้นิ่งไว้ระหว่างวัดแกนนี้"""
    for j in ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']:
        if j != active_joint:
            ctrl.set_joint(j, 90)


print("=" * 60)
print("หาขีดจำกัดจริงของแต่ละ joint (Task 1)")
print("ขยับช้าๆ ทีละ 1° เมื่อใกล้จุดชน — ถ้า servo เริ่มคราง ให้หยุดทันที")
print("=" * 60)

results = {}  # joint -> (lower, upper) หลังหักระยะปลอดภัยแล้ว

quit_early = False

for joint in JOINTS:
    if quit_early:
        break

    print(f"\n{'─' * 60}")
    print(f"[{joint}] {JOINT_NAMES[joint]}")
    print(f"  ขีดจำกัดเดิมใน config.py: {OLD_LIMITS[joint]}")
    input("  กด Enter เพื่อเริ่มปรับ (แขนจะกลับท่ากลาง 90° ก่อน) ... ")

    park_others(joint)
    current = 90.0
    step = 1.0
    ctrl.set_joint(joint, current)

    lower_raw = None
    upper_raw = None

    print("  + - ขยับ | [ ] ตั้งก้าว 1°/5° | l/u บันทึกขีดล่าง/บน | n ถัดไป | q หยุด")
    print(f"  ตำแหน่งปัจจุบัน: {current:.0f}°   ก้าว: {step:.0f}°   "
          f"ล่าง: -   บน: -")

    while True:
        key = getch()

        if key in ('+', '='):
            current = max(0.0, min(180.0, current + step))
        elif key == '-':
            current = max(0.0, min(180.0, current - step))
        elif key == '[':
            step = 1.0
        elif key == ']':
            step = 5.0
        elif key in ('l', 'L'):
            lower_raw = current
            print(f"\n  [บันทึกขีดล่าง] ที่ {lower_raw:.0f}° (ดิบ ยังไม่หักระยะปลอดภัย) → กลับ 90°")
            current = 90.0
        elif key in ('u', 'U'):
            upper_raw = current
            print(f"\n  [บันทึกขีดบน]   ที่ {upper_raw:.0f}° (ดิบ ยังไม่หักระยะปลอดภัย) → กลับ 90°")
            current = 90.0
        elif key in ('n', 'N'):
            if lower_raw is None or upper_raw is None:
                print(f"\n  ⚠ ยังไม่ได้บันทึกครบ (ล่าง={lower_raw}, บน={upper_raw}) "
                      f"— กด l และ u ให้ครบก่อนไปแกนถัดไป")
                continue
            break
        elif key in ('q', 'Q'):
            print(f"\n  [หยุด] ออกกลางคัน")
            quit_early = True
            break
        else:
            continue

        ctrl.set_joint(joint, current)
        l_str = f"{lower_raw:.0f}°" if lower_raw is not None else "-"
        u_str = f"{upper_raw:.0f}°" if upper_raw is not None else "-"
        sys.stdout.write(
            f"\r  ตำแหน่งปัจจุบัน: {current:5.0f}°   ก้าว: {step:.0f}°   "
            f"ล่าง: {l_str}   บน: {u_str}    "
        )
        sys.stdout.flush()

    if lower_raw is not None and upper_raw is not None:
        # เรียงค่าเอง ไม่พึ่งว่าผู้ใช้กด l หรือ u ก่อน — ทิศทาง +/- (logic angle)
        # กับทิศทางที่แขนขยับจริงอาจสวนทางกันเมื่อ INVERT[joint] เป็น True
        # ทำให้กดสลับด้านได้ง่ายโดยไม่รู้ตัว
        lo_raw, hi_raw = sorted((lower_raw, upper_raw))
        if lower_raw > upper_raw:
            print(f"  ⚠ ค่าที่บันทึกสลับด้าน (จุดที่กด l มีมุมมากกว่าจุดที่กด u) — เรียงให้ใหม่อัตโนมัติ")

        lo = round(lo_raw) + SAFETY_MARGIN
        hi = round(hi_raw) - SAFETY_MARGIN

        if hi - lo < 20:
            print(f"  ✗ {joint}: ช่วงหลังหักระยะปลอดภัยเหลือแค่ {hi - lo}° (แคบผิดปกติ) "
                  f"— ไม่บันทึกค่านี้ ใช้ค่าเดิมจาก config.py แทน กรุณาวัดแกนนี้ใหม่")
        else:
            results[joint] = (lo, hi)
            print(f"  → {joint}: ดิบ ({lo_raw:.0f}, {hi_raw:.0f}) "
                  f"→ หักระยะปลอดภัย {SAFETY_MARGIN}° → LIMITS['{joint}'] = ({lo}, {hi})")

# กลับท่ากลางก่อนจบ
for j in ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']:
    ctrl.set_joint(j, 90)

# ── สรุปผล ──────────────────────────────────────────────────────────────
print("\n\n" + "=" * 60)
print("ผลลัพธ์ LIMITS -- คัดลอกไปใส่ใน config.py:")
print("=" * 60)
print("LIMITS = {")
for j in ['J1', 'J2', 'J3', 'J4']:
    if j in results:
        lo, hi = results[j]
        print(f"    '{j}': ({lo},   {hi}),")
    else:
        lo, hi = OLD_LIMITS[j]
        print(f"    '{j}': ({lo},   {hi}),   # ยังไม่ได้วัด ใช้ค่าเดิม")
print(f"    'J5': {OLD_LIMITS['J5']},")
print(f"    'J6': {OLD_LIMITS['J6']},")
print("}")

if len(results) < len(JOINTS):
    missing = [j for j in JOINTS if j not in results]
    print(f"\n⚠ ยังไม่ได้วัดครบ: {missing} — รันสคริปต์นี้ใหม่เพื่อวัดแกนที่เหลือ")
