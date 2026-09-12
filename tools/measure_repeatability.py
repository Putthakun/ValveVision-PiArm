#!/usr/bin/env python3
# tools/measure_repeatability.py — วัดว่า "สั่งท่าเดิม แขนไปที่เดิมจริงไหม" (Task 13 การทดลองที่ 1)
#
# ใช้กล้องที่มือเป็นเครื่องมือวัด: สั่งท่าเดียวกันซ้ำ N รอบ (สลับไปท่าอื่นก่อนทุกรอบ
# ให้แขนต้องเดินทางมาใหม่จริง) แล้วดูว่าจุ๊บในภาพอยู่ที่เดิมไหม — กระจายกี่ px
#
# ★ ที่มา (2026-09-12): ที่ 9 นาฬิกา r=371 (ระยะเอื้อมสุด 372) สั่งท่าเดิมเป๊ะ
#   สองครั้ง จุ๊บห่างเป้า 48px กับ 296px — สงสัยว่า J2 รับน้ำหนักไม่ไหว (แขนตก)
#   + backlash เครื่องมือนี้ให้ตัวเลขตัดสินแทนการเดา และวัดซ้ำหลังแก้ฮาร์ดแวร์
#   (ใส่ยางยืดถ่วง J2 / เปลี่ยนไฟเลี้ยง) ว่าดีขึ้นจริงกี่ px
#
# วิธีใช้:
#   python3 tools/measure_repeatability.py --clock 9            # ท่าสัมผัสที่ 9 นาฬิกา
#   python3 tools/measure_repeatability.py --clock 9 --rounds 8
#   python3 tools/measure_repeatability.py --clock 9 --from below   # เข้าท่าจากล่างเสมอ
#   python3 tools/measure_repeatability.py --clock 9 --from above   # เข้าท่าจากบนเสมอ
#   (เทียบ below/above กัน: ถ้าต่างกันมาก = backlash · ถ้ากระจายมากทั้งคู่ = แขนตก/ไฟ)
#
# เกณฑ์อ่านผล (ประมาณ): กระจาย ≤ 20px ดีมาก · 20-60px ใช้ได้ · > 100px เฟสละเอียด
# ไม่มีทางลู่เข้า ต้องแก้ฮาร์ดแวร์ก่อน

import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm import Arm
from camera import WristCamera
from fine import _valve_px_in_frame
from geometry import valve_pose
from valve_detector import load_model

SETTLE_SEC = 1.5
DETOUR_MM = 40.0   # ท่าอ้อม: เข้าท่าเป้าจากล่าง/บนเท่านี้ทุกรอบ ให้ backlash ไปทางเดียวกัน


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clock', type=float, required=True)
    ap.add_argument('--rounds', type=int, default=6)
    ap.add_argument('--from', dest='approach', choices=['below', 'above', 'scan'], default='scan',
                    help='เข้าท่าเป้าจากไหนทุกรอบ: below/above = อ้อม z ±%.0fmm · scan = กลับท่าสแกนก่อน' % DETOUR_MM)
    ap.add_argument('--pitch', type=float, default=None, help='บังคับ pitch (ไม่ใส่ = ใกล้ 0° สุดที่เอื้อมถึง)')
    args = ap.parse_args()

    arm = Arm()
    r, theta, z = valve_pose(args.clock)
    pitches = arm.reachable_pitches(r, theta, z)
    if not pitches:
        print(f'ที่ {args.clock:g} นาฬิกา เอื้อมไม่ถึง'); sys.exit(1)
    pitch = args.pitch if args.pitch is not None else min(pitches, key=abs)

    print('=' * 62)
    print(f'วัดความทำซ้ำได้ — {args.clock:g} นาฬิกา r={r:.0f} theta={theta:.0f}° z={z:.0f} pitch={pitch:+.0f}°')
    print(f'  {args.rounds} รอบ · เข้าท่าจาก: {args.approach}')
    print('=' * 62)

    cam = WristCamera()
    for _ in range(15):
        cam.grab(); time.sleep(0.1)
    session = load_model()

    pts = []
    try:
        for rd in range(1, args.rounds + 1):
            # ── ออกจากท่าเป้าก่อน ให้แขนต้องเดินทางมาใหม่จริง ──
            if args.approach == 'scan':
                arm.go_scan_pose()
            else:
                dz = -DETOUR_MM if args.approach == 'below' else DETOUR_MM
                if not arm.move_to(r, theta, z + dz, pitch):
                    print(f'  รอบ {rd}: ท่าอ้อม (z{dz:+.0f}) เอื้อมไม่ถึง — ใช้ท่าสแกนแทน')
                    arm.go_scan_pose()
            time.sleep(0.8)

            if not arm.move_to(r, theta, z, pitch):
                print(f'  รอบ {rd}: move_to ล้มเหลว'); continue
            time.sleep(SETTLE_SEC)

            cam.grab()
            frame = cam.grab()
            p = _valve_px_in_frame(frame, session) if frame is not None else None
            if p is None:
                print(f'  รอบ {rd}: มองไม่เห็นจุ๊บ')
                continue
            pts.append(p)
            print(f'  รอบ {rd}: จุ๊บที่ ({p[0]:.0f}, {p[1]:.0f})')
    finally:
        cam.close()
        arm.go_scan_pose()

    if len(pts) < 2:
        print('\nได้ข้อมูลไม่พอ'); sys.exit(1)

    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    print('\n' + '=' * 62)
    print(f'เห็นจุ๊บ {len(pts)}/{args.rounds} รอบ')
    print(f'  x: ค่าเฉลี่ย {statistics.mean(xs):.0f}  ช่วง {min(xs):.0f}-{max(xs):.0f}  กระจาย {max(xs)-min(xs):.0f}px  SD {statistics.pstdev(xs):.1f}')
    print(f'  y: ค่าเฉลี่ย {statistics.mean(ys):.0f}  ช่วง {min(ys):.0f}-{max(ys):.0f}  กระจาย {max(ys)-min(ys):.0f}px  SD {statistics.pstdev(ys):.1f}')
    spread = max(max(xs) - min(xs), max(ys) - min(ys))
    verdict = 'ดีมาก' if spread <= 20 else 'ใช้ได้' if spread <= 60 else 'เฟสละเอียดลู่เข้าไม่ได้ ต้องแก้ฮาร์ดแวร์ก่อน'
    print(f'\nสรุป: กระจายสูงสุด {spread:.0f}px → {verdict}')


if __name__ == '__main__':
    main()
