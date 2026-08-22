# tools/test_reach_all_clock.py — ทดสอบว่าแขนไปถึงจุ๊บได้จริงทุกตำแหน่งนาฬิกาไหม (Task 5B Step 5)
#
# IK บอกได้แค่ว่ามุม joint อยู่ในขีดจำกัดไหม แต่ไม่รู้จักการชนทางกายภาพ
# สคริปต์นี้จึงสั่งแขนไปจริงทีละตำแหน่ง แล้วให้คนดูว่าชนอะไรหรือเปล่า
#
# ★ หยุดห่างหน้าล้อ STANDOFF มม. ไม่ให้ปลายแขนแตะขอบล้อ
# ★ รอกด Enter ก่อนขยับทุกครั้ง — ดูให้แน่ใจก่อนค่อยปล่อยไป
#
# รัน: python3 tools/test_reach_all_clock.py

import sys
import os
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm import Arm
from geometry import valve_pose

STANDOFF = 40.0          # มม. — ระยะเผื่อไม่ให้แตะล้อ
ORDER = [6, 7, 8, 9, 10, 11, 12, 1, 2, 3, 4, 5]   # เริ่มจากที่คุ้นแล้ว ไปหาที่ไม่เคยลอง
OUT_DIR = '/tmp/reach_test'
STREAM = 'http://localhost:8081/wrist.mjpg'


def grab(path):
    """ดึงภาพจากสตรีมกล้องที่มือ (ถ้า preview_cameras.py รันอยู่)"""
    try:
        r = urllib.request.urlopen(STREAM, timeout=5)
        buf = b''
        n = 0
        while n < 4:
            buf += r.read(8192)
            a = buf.find(b'\xff\xd8')
            b = buf.find(b'\xff\xd9', a)
            if a != -1 and b != -1:
                jpeg = buf[a:b + 2]
                buf = buf[b + 2:]
                n += 1
        open(path, 'wb').write(jpeg)
        return True
    except Exception as e:
        print(f'    (ดึงภาพไม่ได้: {e})')
        return False


os.makedirs(OUT_DIR, exist_ok=True)
arm = Arm()

print('=' * 62)
print('ทดสอบว่าแขนไปถึงจุ๊บได้จริงทุกตำแหน่งนาฬิกาไหม')
print(f'หยุดห่างหน้าล้อ {STANDOFF:.0f} มม. — ไม่แตะล้อ')
print()
print('  Enter = ไปตำแหน่งถัดไป   |   q + Enter = หยุดแล้วกลับท่าสแกน')
print('  ★ ดูว่าแขนชนยาง/ขอบล้อ/ตัวเองไหม  ★ ฟังว่า servo ครางไหม')
print('=' * 62)

results = {}

for c in ORDER:
    r, th, z = valve_pose(c)
    r_target = r - STANDOFF

    print(f'\n[{c} นาฬิกา]  r={r_target:.0f} มม.  theta={th:.0f}°  z={z:+.0f} มม.')

    pitch = arm.best_pitch(r_target, th, z)
    if pitch is None:
        print('  ✗ IK บอกว่าเอื้อมไม่ถึง — ข้าม')
        results[c] = 'เอื้อมไม่ถึง (IK)'
        continue

    print(f'  pitch ที่เลือก: {pitch:+.0f}°')
    key = input('  กด Enter เพื่อสั่งแขนไป (q=หยุด) ... ').strip().lower()
    if key == 'q':
        print('  หยุดตามคำสั่ง')
        break

    ok = arm.move_to(r_target, th, z, pitch)
    if not ok:
        print('  ✗ ชั้นความปลอดภัยปฏิเสธ')
        results[c] = 'ถูกปฏิเสธ'
        continue

    time.sleep(1.0)
    grab(os.path.join(OUT_DIR, f'clock_{c:02d}.jpg'))
    print('  ✓ ไปถึงแล้ว — ดูว่าชนอะไรไหม')
    results[c] = 'ไปถึง'

print('\nกลับท่าสแกน...')
arm.go_scan_pose()

print('\n' + '=' * 62)
print('สรุป')
print('=' * 62)
for c in ORDER:
    print(f'  {c:>2} นาฬิกา: {results.get(c, "ยังไม่ได้ทดสอบ")}')
print(f'\nภาพเก็บไว้ที่ {OUT_DIR}/')
