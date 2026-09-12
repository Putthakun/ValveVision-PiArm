#!/usr/bin/env python3
# tools/hold_at_valve.py — พาแขนไปท่าที่ gripper ชี้จุ๊บ ให้คนจูนด้วยตาจนตรงจริง
# แล้วค้างไว้สำหรับ "ปรับมุมกล้อง" ให้มองตามแกน gripper
#
# ★ ที่มา (2026-09-12): ทุกตำแหน่งที่ทดสอบ จุ๊บอยู่เหนือปลาย gripper ในเฟรม
#   250-450px เสมอ ทั้งที่เรขาคณิตชี้ gripper ตรงจุ๊บแล้ว = กล้องก้มลงเทียบกับ
#   แกน gripper ~25-30° ทำให้เฟสละเอียดต้องขยับแขนมากจนหลุดเฟรม/ชนขีดจำกัด
#
# ★ ทำไมไม่ใช้ coarse_locate() (เลือก pitch จากภาพ): เพราะภาพจากกล้องที่เอียง
#   คือสิ่งที่กำลังจะปรับ ใช้มันเลือกท่าจะได้ท่าที่ "ภาพดูดี" แต่ปลายก้ามจริง
#   ไม่ตรงจุ๊บ (เจอจริง: เลือก pitch=-25° เงยมือขึ้นชดเชยกล้องที่ก้ม ปลายก้าม
#   จริงอยู่ต่ำกว่าจุ๊บ) — เครื่องมือนี้ใช้ pitch ใกล้ 0° แล้วให้ตาคนตัดสินแทน
#
# วิธีใช้:
#   1. หมุนล้อไป 9 นาฬิกา (เอื้อมสบาย pitch ใกล้ 0 ได้)
#   2. python3 tools/hold_at_valve.py
#      แขนไปท่าชี้จุ๊บตามเรขาคณิต แล้วให้กดปุ่มจูนจน "ปลายก้ามคีบชี้ตรงจุ๊บ
#      จริงด้วยตา" (ระดับเดียวกัน ซ้าย-ขวาตรง):
#        w / x   ยกขึ้น / กดลง 5 มม.       a / d   หันซ้าย / หันขวา 1°
#        [ / ]   ก้าว 1 / 5                q       เสร็จ (ค้างท่าไว้)
#      ตอนจบจะบอกว่าต้องชดเชยไปเท่าไหร่ = แขนตก/เรขาคณิตคลาดที่ตำแหน่งนี้
#      (ห้ามเอาไปใส่เป็นค่าคงที่ใน config — กฎข้อ 1 — เฟสละเอียดต้องแก้เอง
#       ด้วยกล้อง ซึ่งจะทำได้ก็ต่อเมื่อกล้องมองตามแกน gripper)
#   3. python3 tools/preview_detect.py   ← ดูภาพสดที่ http://<ip>:8082/
#   4. คลายสกรูกล้อง ปรับให้ "จุ๊บ (กรอบเขียว) อยู่เหนือปลายก้ามคีบนิดเดียว"
#      (ปลายก้ามอยู่ราว y≈520 ของเฟรม 720) ระวังอย่าให้หมุนซ้าย-ขวา แล้วขันแน่น
#   5. ปิด preview แล้ว python3 -m fine --debug — err_y รอบ 1 ควร < ~100px
#
# ★ ไม่กลับท่าสแกนตอนจบโดยตั้งใจ — ต้องค้างท่าไว้ให้ปรับกล้อง
#   ถ้าต้องการกลับท่าสแกน: python3 -c "from arm import Arm; Arm().go_scan_pose()"

import os
import sys
import termios
import tty

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm import Arm
from camera import WristCamera
from coarse import CONFIRM_FRAMES_DEFAULT, _scan_from_pose
from geometry import valve_pose
from valve_detector import load_model


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    arm = Arm()
    cam = WristCamera()
    session = load_model()
    try:
        print("กำลังหาวาล์ว...")
        clock = _scan_from_pose(cam, session, arm, upper=False, confirm_frames=CONFIRM_FRAMES_DEFAULT)
        if clock is None:
            clock = _scan_from_pose(cam, session, arm, upper=True, confirm_frames=CONFIRM_FRAMES_DEFAULT)
    finally:
        cam.close()   # ปล่อยกล้องให้ preview_detect.py ใช้ต่อ

    if clock is None:
        print("ไม่เจอวาล์ว — ลองใหม่")
        return

    r, theta, z = valve_pose(clock)
    pitches = arm.reachable_pitches(r, theta, z)
    if not pitches:
        print(f"ที่ {clock:.2f} นาฬิกา เอื้อมไม่ถึง — ลองตำแหน่งอื่น (แนะนำ 9 นาฬิกา)")
        return
    pitch = min(pitches, key=abs)   # ใกล้ 0° ที่สุด ไม่ใช่ margin ดีสุด ไม่ใช่จากภาพ
    if not arm.move_to(r, theta, z, pitch):
        print("move_to ล้มเหลวทั้งที่เพิ่งเช็คว่าเอื้อมถึง — ผิดปกติ")
        return

    print(f"\nเจอวาล์วที่ {clock:.2f} นาฬิกา → แขนชี้ตามเรขาคณิต r={r:.0f} theta={theta:.0f}° z={z:.0f} pitch={pitch:+.0f}°")
    print("ดูด้วยตา: ปลายก้ามคีบชี้ตรงจุ๊บไหม? จูนจนตรง")
    print("  w / x  ขึ้น / ลง     a / d  ซ้าย / ขวา     [ / ]  ก้าว 1 / 5     q  เสร็จ")

    step = 1.0
    dz_total = 0.0
    dth_total = 0.0
    while True:
        k = getch()
        dz = dth = 0.0
        if k == 'w':
            dz = 5.0 * step
        elif k == 'x':
            dz = -5.0 * step
        elif k == 'a':
            dth = -1.0 * step
        elif k == 'd':
            dth = 1.0 * step
        elif k == '[':
            step = 1.0
        elif k == ']':
            step = 5.0
        elif k == 'q':
            break
        else:
            continue
        if dz or dth:
            if arm.nudge(dth, dz):
                dz_total += dz
                dth_total += dth
            else:
                print("  ✗ ขยับต่อไม่ได้ (ชนขีดจำกัด) — ลองทิศอื่น")
        cr, cth, cz, cp = arm.current()
        sys.stdout.write(f"\r  ตอนนี้ theta={cth:6.1f}°  z={cz:6.1f}   (ชดเชยรวม theta {dth_total:+.0f}° z {dz_total:+.0f}mm)  ก้าว {step:.0f}   ")
        sys.stdout.flush()

    print(f"\n\nชดเชยจากเรขาคณิต: theta {dth_total:+.0f}°  z {dz_total:+.0f}mm  (ที่ {clock:.2f} นาฬิกา pitch={pitch:+.0f}°)")
    print("  → ค่านี้คือแขนตก/เรขาคณิตคลาดที่ตำแหน่งนี้ ห้ามใส่เป็นค่าคงที่ (กฎข้อ 1) เฟสละเอียดต้องแก้เองด้วยกล้อง")
    print("\nแขนค้างท่าไว้แล้ว ตอนนี้เปิด  python3 tools/preview_detect.py")
    print("แล้วปรับกล้องให้จุ๊บ (กรอบเขียว) อยู่เหนือปลายก้ามคีบนิดเดียว ระวังอย่าให้หมุนซ้าย-ขวา")


if __name__ == "__main__":
    main()
