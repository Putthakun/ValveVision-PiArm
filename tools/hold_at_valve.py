#!/usr/bin/env python3
# tools/hold_at_valve.py — พาแขนไปค้างท่าที่ gripper ชี้จุ๊บ (ตามเรขาคณิต) แล้วปล่อยไว้
# ใช้ตอน "ปรับมุมกล้อง" ให้มองตามแกน gripper
#
# ★ ที่มา (2026-09-12): ทุกตำแหน่งที่ทดสอบ จุ๊บอยู่เหนือปลาย gripper ในเฟรม
#   250-450px เสมอ ทั้งที่เรขาคณิตชี้ gripper ตรงจุ๊บแล้ว = กล้องก้มลงเทียบกับ
#   แกน gripper ~25-30° ทำให้เฟสละเอียดต้องขยับแขนมากจนหลุดเฟรม/ชนขีดจำกัด
#   ถ้ากล้องมองตามแกน gripper จริง จุ๊บจะอยู่ใกล้ปลายก้ามตั้งแต่แรก error เล็ก
#
# วิธีใช้:
#   1. หมุนล้อไปตำแหน่งไหนก็ได้ที่เอื้อมถึงสบาย (เช่น 9 นาฬิกา pitch≈0)
#   2. python3 tools/hold_at_valve.py        ← แขนไปค้างท่าชี้จุ๊บ แล้วสคริปต์จบ (servo ค้างท่าไว้)
#   3. python3 tools/preview_detect.py       ← ดูภาพสดที่ http://<ip>:8082/
#   4. คลายสกรูกล้อง ปรับให้ "จุ๊บ (กรอบเขียว) อยู่เหนือปลายก้ามคีบนิดเดียว"
#      (ปลายก้ามอยู่ราว y≈520 ของเฟรม 720) แล้วขันแน่น
#   5. python3 tools/preview_detect.py ปิด แล้วรัน python3 -m fine --debug ทดสอบ
#      ค่า err_y รอบ 1 ควรเล็กกว่า ~100px (เดิม 250-450)
#
# ★ ไม่กลับท่าสแกนตอนจบโดยตั้งใจ — ต้องค้างท่าไว้ให้ปรับกล้อง กด Ctrl+C ไม่ต้อง
#   ถ้าต้องการกลับท่าสแกน: python3 -c "from arm import Arm; Arm().go_scan_pose()"

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm import Arm
from camera import WristCamera
from coarse import coarse_locate
from valve_detector import load_model


def main():
    arm = Arm()
    cam = WristCamera()
    session = load_model()
    try:
        print("กำลังหาวาล์วแล้วพาแขนไปชี้...")
        res = coarse_locate(cam, session, arm)
    finally:
        cam.close()   # ปล่อยกล้องให้ preview_detect.py ใช้ต่อ

    if not res.ok:
        print(f"ไม่สำเร็จ — reason={res.reason} (แขนอยู่ท่าล่าสุดที่ไปถึง)")
        return

    print(f"แขนค้างอยู่ที่ r={res.r:.0f} theta={res.theta_deg:.0f}° z={res.z:.0f} pitch={res.pitch_deg:+.0f}°")
    print("ตอนนี้เปิด  python3 tools/preview_detect.py  แล้วปรับกล้องให้จุ๊บอยู่เหนือปลายก้ามคีบนิดเดียว")


if __name__ == "__main__":
    main()
