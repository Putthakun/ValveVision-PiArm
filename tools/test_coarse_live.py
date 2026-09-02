#!/usr/bin/env python3
# tools/test_coarse_live.py — ทดสอบ coarse_locate() บนแขนจริง (Task 9 เกณฑ์ผ่าน)
#
# วิธีใช้: หมุนล้อไปตำแหน่งนาฬิกาที่ต้องการ แล้วรัน
#   python3 tools/test_coarse_live.py
#
# สคริปต์จะ:
#   1. ไปท่าสแกน มองหาจุ๊บ ประเมินมุมนาฬิกา
#   2. พาแขนเข้าใกล้ (ตามที่ coarse_locate ตัดสินใจ)
#   3. ถ่ายภาพจากกล้องที่มือ ณ จุดที่พาไปถึง
#   4. ให้คุณดูภาพแล้วตอบว่าเห็นจุ๊บในเฟรมไหม (y/n) — นับเป็นผ่าน/ไม่ผ่าน
#
# ทำซ้ำหลายตำแหน่งนาฬิกาแล้วนับผลรวม เกณฑ์ผ่านของ Task 9 คือ
# เห็นจุ๊บได้ >= 8 จาก 10 ครั้ง

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2

from arm import Arm
from camera import WristCamera
from coarse import coarse_locate
from valve_detector import load_model, postprocess, preprocess

OUT_DIR = 'data/coarse_test'


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    arm = Arm()
    cam = WristCamera()
    for _ in range(15):      # warm-up
        cam.grab()
        time.sleep(0.1)
    session = load_model()

    results = []
    try:
        while True:
            clock_label = input('\nหมุนล้อไปตำแหน่งที่ต้องการแล้วกด Enter (พิมพ์ q ออก): ').strip()
            if clock_label.lower() == 'q':
                break

            print('  กำลังหา... ', end='', flush=True)
            res = coarse_locate(cam, session, arm)

            if not res.ok:
                print(f'ไม่สำเร็จ — reason={res.reason}')
                results.append((clock_label, False, res.reason))
                continue

            print(f'ok — r={res.r:.0f} theta={res.theta_deg:.0f}° z={res.z:.0f} pitch={res.pitch_deg:+.0f}°')
            time.sleep(0.5)
            cam.grab()
            frame = cam.grab()

            sess, inp, out = session
            h, w = frame.shape[:2]
            blob, sc, pl, pt = preprocess(frame)
            dets = postprocess(sess.run([out], {inp: blob})[0], w, h, sc, pl, pt)
            vis = frame.copy()
            for x1, y1, x2, y2, conf, _ in dets:
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            path = os.path.join(OUT_DIR, f'{clock_label}_{len(results):02d}.jpg')
            cv2.imwrite(path, vis)
            print(f'  ถ่ายเก็บไว้ที่ {path}')

            ans = input('  เห็นจุ๊บในภาพนี้ไหม? (y/n): ').strip().lower()
            ok = ans.startswith('y')
            results.append((clock_label, ok, 'เห็น' if ok else 'ไม่เห็น'))
    finally:
        cam.close()
        arm.go_scan_pose()

    print('\n' + '=' * 50)
    print('สรุปผล')
    print('=' * 50)
    n_ok = sum(1 for _, ok, _ in results if ok)
    for label, ok, why in results:
        print(f'  {label:>10}: {"✓ ผ่าน" if ok else "✗ ไม่ผ่าน"} ({why})')
    print(f'\nรวม {n_ok}/{len(results)} — เกณฑ์ผ่านของ Task 9 คือ >= 8/10')


if __name__ == '__main__':
    main()
