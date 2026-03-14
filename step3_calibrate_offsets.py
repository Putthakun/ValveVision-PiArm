# step3_calibrate_offsets.py
# ปรับ ZERO_OFFSET ทีละ joint จนแขนตรง 90° จริง
#
# วิธีใช้:
#   python3 step3_calibrate_offsets.py
#
# ปุ่มบังคับ (ขณะปรับแต่ละ joint):
#   +  หรือ =   เพิ่ม 1°
#   -           ลด 1°
#   Shift++     เพิ่ม 5°  (กด shift++ หรือพิมพ์ >)
#   Shift+-     ลด 5°    (พิมพ์ <)
#   Enter       บันทึก joint นี้ → ไป joint ถัดไป
#   s           ข้าม joint นี้ (ใช้ค่าเดิม)
#   q           หยุดและแสดงผล

import sys
import time
import termios
import tty

from adafruit_servokit import ServoKit
from config import INVERT, ZERO_OFFSET, CHANNEL

kit = ServoKit(channels=16)
for ch in range(6):
    kit.servo[ch].set_pulse_width_range(500, 2500)

JOINTS = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
JOINT_NAMES = {
    'J1': 'Base (หมุนฐาน)',
    'J2': 'Shoulder (ยกไหล่)',
    'J3': 'Elbow (งอศอก)',
    'J4': 'Wrist Pitch (ก้มเงย)',
    'J5': 'Wrist Roll (หมุนข้อมือ)',
    'J6': 'Gripper (คีบ)',
}

TARGET_LOGIC = 90  # logic angle ที่ต้องการ (90° = neutral/กลาง)


def logic_to_servo(joint: str, logic: float, offset: float) -> float:
    """แปลง logic angle → servo angle โดยใช้ INVERT + offset ที่กำลังทดสอบ"""
    servo = (180 - logic) if INVERT[joint] else logic
    servo += offset
    return max(0, min(180, servo))


def send(joint: str, offset: float):
    ch = CHANNEL[joint]
    angle = logic_to_servo(joint, TARGET_LOGIC, offset)
    kit.servo[ch].angle = angle


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def park_others(active_joint: str, offsets: dict):
    """ส่ง joint อื่นๆ ไปที่ 90° (ใช้ค่า offset ล่าสุด)"""
    for j in JOINTS:
        if j != active_joint:
            send(j, offsets[j])


print("=" * 55)
print("STEP 3 -- Calibrate ZERO_OFFSET")
print("ปรับแต่ละ joint จนแขนอยู่ที่ 90° (ท่ากลาง) จริงๆ")
print("=" * 55)
print()
print("ปุ่ม:  + / -   = ±1°     > / <   = ±5°")
print("       Enter   = บันทึก  s       = ข้าม  q = หยุด")
print()

# โหลด offset เริ่มต้นจาก config
current_offsets = dict(ZERO_OFFSET)  # copy
final_offsets   = dict(ZERO_OFFSET)

# กลับ home ก่อน
print("กำลัง home ทุก joint ...")
for j in JOINTS:
    send(j, current_offsets[j])
time.sleep(2)

quit_early = False

for joint in JOINTS:
    if quit_early:
        break

    ch = CHANNEL[joint]
    offset = current_offsets[joint]

    print(f"\n{'─' * 55}")
    print(f"[{joint}] {JOINT_NAMES[joint]}   (channel {ch})")
    print(f"  INVERT = {INVERT[joint]}   offset เริ่มต้น = {offset:+.0f}°")
    print(f"  servo จะถูกส่งไปที่ {logic_to_servo(joint, TARGET_LOGIC, offset):.0f}°")
    print(f"  ดูว่าแขน joint นี้อยู่ที่ 90° (ท่ากลาง) ไหม")
    input("  กด Enter เพื่อเริ่มปรับ ... ")

    park_others(joint, final_offsets)
    send(joint, offset)
    time.sleep(0.5)

    print(f"  offset ปัจจุบัน: {offset:+.0f}°   servo = {logic_to_servo(joint, TARGET_LOGIC, offset):.0f}°")
    print("  ปรับด้วย + - > <  |  Enter=บันทึก  s=ข้าม  q=หยุด")

    while True:
        key = getch()

        if key in ('+', '='):
            offset += 1
        elif key == '-':
            offset -= 1
        elif key in ('>', '.'):
            offset += 5
        elif key in ('<', ','):
            offset -= 5
        elif key in ('\r', '\n'):
            final_offsets[joint] = round(offset)
            print(f"\r  [บันทึก] {joint} offset = {round(offset):+d}°           ")
            break
        elif key in ('s', 'S'):
            print(f"\r  [ข้าม]   {joint} ใช้ค่าเดิม = {current_offsets[joint]:+d}°      ")
            break
        elif key in ('q', 'Q'):
            print(f"\r  [หยุด]                                              ")
            quit_early = True
            break
        else:
            continue

        # ส่งค่าใหม่ทันที
        angle = logic_to_servo(joint, TARGET_LOGIC, offset)
        angle = max(0, min(180, angle))
        kit.servo[ch].angle = angle
        sys.stdout.write(f"\r  offset = {offset:+.0f}°   servo = {angle:.0f}°   ")
        sys.stdout.flush()

# ────────────────────────────────────────────────
print("\n\n" + "=" * 55)
print("ผลลัพธ์ ZERO_OFFSET -- คัดลอกไปใส่ใน config.py:")
print("=" * 55)
print("ZERO_OFFSET = {")
keys = list(final_offsets.keys())
line1 = ", ".join(f"'{k}': {final_offsets[k]:d}" for k in keys[:3])
line2 = ", ".join(f"'{k}': {final_offsets[k]:d}" for k in keys[3:])
print(f"    {line1},")
print(f"    {line2},")
print("}")
print()

# เปรียบเทียบ
changed = [(j, ZERO_OFFSET[j], final_offsets[j]) for j in JOINTS if ZERO_OFFSET[j] != final_offsets[j]]
if changed:
    print("ค่าที่เปลี่ยนจากเดิม:")
    for j, old, new in changed:
        print(f"  {j}: {old:+d}° → {new:+d}°  (เปลี่ยน {new-old:+d}°)")
else:
    print("ไม่มีการเปลี่ยนแปลง")
