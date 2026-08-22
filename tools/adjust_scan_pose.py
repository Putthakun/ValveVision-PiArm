#!/usr/bin/env python3
# tools/adjust_scan_pose.py — ปรับท่าสแกนแบบเห็นผลทันที แล้วบันทึกลง config.py
#
# วิธีใช้:
#   1. เปิดสตรีมกล้องไว้อีกหน้าต่าง:  python3 tools/preview_cameras.py
#   2. เปิดเบราว์เซอร์ดูที่          http://<TAILSCALE_IP>:8081/wrist
#   3. รันสคริปต์นี้แล้วกดปุ่มปรับ ดูภาพในเบราว์เซอร์ไปด้วย
#   4. พอใจแล้วกด p เพื่อบันทึกลง config.py
#
# ปุ่ม:
#   a / d   J1 ลด / เพิ่ม   (J1 น้อยลง = ล้อเลื่อนไปทางขวาในภาพ)
#   w / x   J2 ลด / เพิ่ม   (J2 น้อยลง = กล้องเงยขึ้น ล้อเลื่อนลงในภาพ)
#   e / c   J3 ลด / เพิ่ม   (J3 พับข้อศอก — ตอนนี้อยู่ที่ 180 คือพับสุดแล้ว ลดได้อย่างเดียว)
#   [ / ]   ตั้งขนาดก้าว 1° / 5°
#   p       บันทึกลง config.py
#   r       กลับไปค่าที่บันทึกไว้
#   q       ออกโดยไม่บันทึก

import os
import re
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from servo_controller import ServoController
from config import SCAN_POSE

CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config.py')


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def save_to_config(j1: float, j2: float, j3: float) -> bool:
    """เขียนค่า J1/J2/J3 ใหม่ลงบล็อก SCAN_POSE ใน config.py (ไม่แตะส่วนอื่น)"""
    src = open(CONFIG_PATH, encoding='utf-8').read()
    start = src.find('SCAN_POSE = {')
    if start == -1:
        print('  ✗ หา SCAN_POSE ใน config.py ไม่เจอ')
        return False
    end = src.find('}', start)
    block = src[start:end]

    new_block = re.sub(r"'J1':\s*[\d.]+", f"'J1': {j1:.1f}", block)
    new_block = re.sub(r"'J2':\s*[\d.]+", f"'J2': {j2:.1f}", new_block)
    new_block = re.sub(r"'J3':\s*[\d.]+", f"'J3': {j3:.1f}", new_block)

    open(CONFIG_PATH, 'w', encoding='utf-8').write(src[:start] + new_block + src[end:])
    return True


ctrl = ServoController()
saved_j1 = float(SCAN_POSE['J1'])
saved_j2 = float(SCAN_POSE['J2'])
saved_j3 = float(SCAN_POSE['J3'])
j1, j2, j3 = saved_j1, saved_j2, saved_j3
step = 1.0

print('=' * 64)
print('ปรับท่าสแกน — ดูภาพสดที่ http://<TAILSCALE_IP>:8081/wrist')
print()
print('  a / d   J1 ลด / เพิ่ม   (น้อยลง = ล้อเลื่อนไปทางขวาในภาพ)')
print('  w / x   J2 ลด / เพิ่ม   (น้อยลง = กล้องเงยขึ้น ล้อเลื่อนลงในภาพ)')
print('  e / c   J3 ลด / เพิ่ม   (พับข้อศอก · ตอนนี้ 180 = พับสุด ลดได้อย่างเดียว)')
print('  [ / ]   ก้าว 1° / 5°     p บันทึก    r คืนค่าเดิม    q ออก')
print()
print('★ หมุนจุ๊บไปตำแหน่งที่ชิดขอบสุด (12 กับ 6 นาฬิกา) แล้วเช็คว่ายังเห็น')
print('=' * 64)


def show():
    flag = '' if (j1, j2, j3) == (saved_j1, saved_j2, saved_j3) else '  *ยังไม่บันทึก*'
    sys.stdout.write(f'\r  J1={j1:6.1f}°  J2={j2:6.1f}°  J3={j3:6.1f}°   ก้าว {step:.0f}°{flag}       ')
    sys.stdout.flush()


def apply():
    pose = dict(SCAN_POSE)
    pose['J1'] = j1
    pose['J2'] = j2
    pose['J3'] = j3
    ctrl.move_smooth(pose, steps=25, delay=0.02, settle=0.1)


apply()
show()

while True:
    k = getch()

    if k == 'a':
        j1 = max(0.0, j1 - step)
    elif k == 'd':
        j1 = min(180.0, j1 + step)
    elif k == 'w':
        j2 = max(0.0, j2 - step)
    elif k == 'x':
        j2 = min(180.0, j2 + step)
    elif k == 'e':
        j3 = max(0.0, j3 - step)
    elif k == 'c':
        j3 = min(180.0, j3 + step)
    elif k == '[':
        step = 1.0
        show()
        continue
    elif k == ']':
        step = 5.0
        show()
        continue
    elif k == 'r':
        j1, j2, j3 = saved_j1, saved_j2, saved_j3
        print(f'\n  คืนค่าเดิม J1={j1:.0f} J2={j2:.0f} J3={j3:.0f}')
    elif k == 'p':
        if save_to_config(j1, j2, j3):
            saved_j1, saved_j2, saved_j3 = j1, j2, j3
            print(f'\n  ✓ บันทึกลง config.py แล้ว — SCAN_POSE J1={j1:.1f} J2={j2:.1f} J3={j3:.1f}')
            print('    (อย่าลืม git commit)')
        show()
        continue
    elif k == 'q':
        if (j1, j2, j3) != (saved_j1, saved_j2, saved_j3):
            print(f'\n  ออกโดยไม่บันทึก — config.py ยังเป็น J1={saved_j1:.0f} J2={saved_j2:.0f} J3={saved_j3:.0f}')
            print('  กำลังกลับไปท่าที่บันทึกไว้...')
            j1, j2, j3 = saved_j1, saved_j2, saved_j3
            apply()
        else:
            print('\n  ออก')
        break
    else:
        continue

    apply()
    show()
