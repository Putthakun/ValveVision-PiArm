#!/usr/bin/env python3
# tools/measure_touch_distance.py — วัดจริงว่า TOUCH state (Task 11) ต้องเดินหน้า
# อีกกี่ มม. หลังเฟสละเอียดลู่เข้าแล้ว ถึงจะแตะจุ๊บพอดี
#
# ★ ที่มา (2026-09-12): PLAN.md เดิมเขียนว่า TOUCH เดินหน้าอีก 12 ซม. — ตัวเลข
#   นั้นมาจากตอนยังมี STANDOFF_MM=120 (ถอยแขนไว้ก่อนเข้าใกล้) แต่ตอนนี้ลดเหลือ
#   0 แล้ว (พบว่า standoff ทำให้บางตำแหน่งเอื้อมไม่ถึง) coarse_locate() จึงพาแขน
#   ไปเกือบถึงระยะสัมผัสเต็มอยู่แล้ว ถ้าเดินหน้าอีก 12 ซม.ตามเดิมจะพุ่งทะลุเข้า
#   ล้อลึก 12 ซม. อันตรายมาก ขัดกฎข้อ 5 (ห้ามค้างดันของแข็ง) โดยตรง
#   ต้องวัดของจริงแทนเดา (เหมือน pixel_scale/measure_repeatability)
#
# วิธีใช้:
#   python3 tools/measure_touch_distance.py --clock 9
#   ทำ coarse_locate() + fine_align() ให้ลู่เข้าก่อน แล้วเดินหน้าทีละ STEP_MM
#   กด Enter ทุกก้าว ดู/สัมผัสว่าแตะจุ๊บหรือยัง พิมพ์ "y" เมื่อแตะแล้ว
#   → ถอยทันทีอัตโนมัติ (กฎข้อ 5) พิมพ์ระยะรวมที่เดินหน้าไป
#   ทำซ้ำที่ 2-3 ตำแหน่งเอาค่าเฉลี่ย+ความเผื่อ (margin ความปลอดภัย) มาตั้งเป็น
#   ค่าคงที่ TOUCH_DISTANCE_MM ใน run.py

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm import Arm
from camera import WristCamera
from coarse import coarse_locate
from fine import _load_scale_for_pitch, fine_align
from valve_detector import load_model

STEP_MM = 3.0   # เดินหน้าทีละเท่านี้ — เล็กพอให้หยุดทันเมื่อรู้สึกว่าใกล้แตะ


def main():
    arm = Arm()
    cam = WristCamera()
    session = load_model()
    try:
        print("เฟสหยาบ: หาวาล์ว...")
        coarse = coarse_locate(cam, session, arm)
        if not coarse.ok:
            print(f"เฟสหยาบไม่สำเร็จ — reason={coarse.reason}")
            return
        print(f"เฟสหยาบ ok — r={coarse.r:.0f} theta={coarse.theta_deg:.0f}° "
              f"z={coarse.z:.0f} pitch={coarse.pitch_deg:+.0f}°")

        scale = _load_scale_for_pitch(coarse.pitch_deg)
        print("เฟสละเอียด: ไล่ตำแหน่ง...")
        fine_res = fine_align(cam, session, arm, scale, debug=True)
        if not fine_res.converged:
            print(f"เฟสละเอียดไม่ลู่เข้า (reason={fine_res.reason}) — หยุด ไม่เดินหน้าต่อ")
            return
        print(f"เฟสละเอียด ok — err={fine_res.final_px_err:.1f}px")

        r0, theta, z, pitch = arm.current()
        print(f"\nพร้อมเดินหน้าทีละ {STEP_MM:.0f}mm จาก r={r0:.0f}")
        print("กด Enter เพื่อเดินหน้า 1 ก้าว · พิมพ์ y แล้ว Enter เมื่อแตะจุ๊บแล้ว · พิมพ์ q ยกเลิก")

        traveled = 0.0
        while True:
            ans = input(f"  [r={r0 + traveled:.0f}, เดินไปแล้ว {traveled:.0f}mm] ").strip().lower()
            if ans == 'q':
                print("ยกเลิก — ไม่ถอยเพราะยังไม่ได้แตะอะไร (แต่เช็คให้แน่ใจด้วยตาก่อนปล่อยมือ)")
                return
            if ans == 'y':
                print(f"\n★ แตะจุ๊บที่ระยะเดินหน้ารวม {traveled:.0f}mm จากจุดที่เฟสละเอียดลู่เข้า")
                break
            if not arm.move_to(r0 + traveled + STEP_MM, theta, z, pitch):
                print("  ✗ เดินหน้าต่อไม่ได้ (เอื้อมไม่ถึง) — หยุดตรงนี้ ถือว่าเกินระยะที่ปลอดภัย")
                break
            traveled += STEP_MM
    finally:
        print("ถอยทันที (กฎข้อ 5)...")
        arm.retreat()
        cam.close()


if __name__ == "__main__":
    main()
