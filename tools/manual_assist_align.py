#!/usr/bin/env python3
# tools/manual_assist_align.py — โหมดช่วยแมนวล: พาแขนไปให้ใกล้สุดเท่าที่เอื้อมได้จริง
# แล้วให้คนดันฐานเข้าไปเองส่วนที่เหลือ (อาจารย์อนุญาตให้ช่วยดันได้)
#
# ★ ไม่ใช่ทางลัดของกฎข้อ 2 — ไม่มีการ "หลอกว่าไปถึงแล้ว" ตรงไหนเลย
#   ทุกตำแหน่งที่สั่งผ่านมาจาก solve_ik จริง (arm.best_pitch/move_to ปกติ)
#   แค่หาตำแหน่งที่ "ไกลสุดเท่าที่ยังสั่งได้จริง" แทนตำแหน่งจุ๊บเป๊ะๆ ที่อาจ
#   เอื้อมไม่ถึง แล้วบอกตรงๆ ว่าขาดอีกกี่ มม. ให้คนดันเอง — ไม่ใช่ coarse_locate()
#   เวอร์ชันปกติที่ต้องซื่อสัตย์ 100% (จึงแยกเป็นเครื่องมือคนละตัว ไม่ปนกัน)
#
# วิธีใช้:
#   หมุนล้อไปตำแหน่งที่ต้องการแล้วรัน
#   python3 tools/manual_assist_align.py

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm import Arm
from camera import WristCamera
from coarse import _scan_from_pose, CONFIRM_FRAMES_DEFAULT
from geometry import valve_pose
from valve_detector import load_model

R_STEP_MM = 10.0   # ถอยทีละเท่านี้ตอนหาระยะไกลสุดที่ยังเอื้อมถึง


def find_max_reach(arm: Arm, r_touch: float, theta_deg: float, z: float):
    """ลดระยะ r ทีละ R_STEP_MM จากระยะสัมผัสจริง จนกว่าจะหา pitch ที่เอื้อมถึงได้

    คืน (r, pitch_deg) ที่ไกลสุดเท่าที่ยังสั่งได้จริง หรือ None ถ้าไม่ถึงแม้แต่ r=0
    """
    r = r_touch
    while r >= 0:
        pitch = arm.best_pitch(r, theta_deg, z)
        if pitch is not None:
            return r, pitch
        r -= R_STEP_MM
    return None


def main():
    arm = Arm()
    cam = WristCamera()
    session = load_model()
    try:
        print("กำลังหาวาล์ว...")
        clock = _scan_from_pose(cam, session, arm, upper=False, confirm_frames=CONFIRM_FRAMES_DEFAULT)
        if clock is None:
            clock = _scan_from_pose(cam, session, arm, upper=True, confirm_frames=CONFIRM_FRAMES_DEFAULT)
        if clock is None:
            print("ไม่เจอวาล์ว — ลองใหม่")
            return

        print(f"เจอวาล์วที่ประมาณ {clock:.2f} นาฬิกา")
        r_touch, theta_deg, z = valve_pose(clock)

        result = find_max_reach(arm, r_touch, theta_deg, z)
        if result is None:
            print("เอื้อมไม่ถึงเลยแม้แต่ r=0 — ตำแหน่งนี้ช่วยด้วยมือไม่ได้จริงๆ")
            return

        r, pitch = result
        if not arm.move_to(r, theta_deg, z, pitch):
            print("move_to ล้มเหลวทั้งที่เพิ่งเช็คว่าเอื้อมถึง — ผิดปกติ หยุดไว้ก่อน")
            return

        shortfall = r_touch - r
        print(f"\nไปถึง r={r:.0f}mm (theta={theta_deg:.0f}° z={z:.0f}mm pitch={pitch:+.0f}°) แล้ว")
        if shortfall < 1.0:
            print("เอื้อมถึงจุ๊บพอดี ไม่ต้องช่วยดัน!")
        else:
            print(f"ขาดอีก {shortfall:.0f}mm ถึงจะถึงจุ๊บ — ช่วยดันฐาน/ล้อเข้าหากันอีก {shortfall:.0f}mm "
                  f"ตามแนวที่แขนชี้อยู่ตอนนี้ แล้วปลาย gripper ควรไปตรงจุ๊บพอดี")
    finally:
        cam.close()


if __name__ == "__main__":
    main()
