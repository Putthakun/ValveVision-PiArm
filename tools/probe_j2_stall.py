#!/usr/bin/env python3
# tools/probe_j2_stall.py — ไล่ขยับ J2 ทีละนิด หาว่าเริ่มฝืด/สตอลที่มุมไหน
# (สืบเนื่องจากอาการ servo สตอลตอนไปท่าสแกน B — ดู DESIGN.md หัวข้อ 7)
#
# วิธีใช้: python3 tools/probe_j2_stall.py
# ดูตอนแขนขยับแต่ละก้าว บอกว่ามันไปถึงจริงไหม หรือค้าง/ฝืด แล้วกด Enter ไปมุมถัดไป
# กด q แล้ว Enter เพื่อออกก่อนครบ

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from servo_controller import ServoController

BASE = {'J1': 85.0, 'J3': 180.0, 'J4': 180.0, 'J5': 90.0, 'J6': 90.0}
J2_STEPS = [48, 46, 44, 42, 40, 39]

ctrl = ServoController()
for j2 in J2_STEPS:
    pose = dict(BASE, J2=float(j2))
    ctrl.move_smooth(pose, steps=20, delay=0.02)
    ans = input(f'J2={j2}° — ถึงจริงไหม/ฝืดไหม? (Enter=ไปต่อ, q=ออก): ').strip().lower()
    if ans == 'q':
        break

print('จบการไล่ทดสอบ')
