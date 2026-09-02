#!/usr/bin/env python3
# tools/measure_pixel_scale.py — วัด "ขยับแขน 1 หน่วย → ภาพเลื่อนกี่พิกเซล" (Task 8)
#
# เฟสละเอียดต้องใช้ค่านี้ตัดสินว่าจะขยับเท่าไหร่
# ★ เครื่องหมายสำคัญกว่าขนาด — ถ้าทิศกลับด้าน ลูปจะวิ่งหนีเป้าแทนที่จะเข้าหา
#
# วิธีวัด (ทำซ้ำ 3 รอบเอาค่าเฉลี่ย):
#     ตรวจจับจุ๊บ                     → (u0, v0)
#     nudge(+DTHETA องศา, 0)          → (u1, v1)   deg_per_px_x = DTHETA / (u1-u0)
#     nudge(0, +DZ มม.)               → (u2, v2)   mm_per_px_y  = DZ / (v2-v1)
#
# ข้อดีของการวัดจากของจริง: มันกลืน "ความเอียงของกล้องจากแกน gripper" (~15°/22°)
# เข้าไปในค่าที่วัดได้เอง ไม่ต้องแยกคำนวณ
#
# วิธีใช้ (ต้องปิดสตรีมกล้องก่อน):
#   pkill -f preview_detect.py
#   python3 tools/measure_pixel_scale.py --clock 4.5
#
#   --dist 150     ระยะกล้องถึงจุ๊บที่จะไปวัด (มม.)
#   --rounds 3     จำนวนรอบ
#   --dry-run      คำนวณท่าอย่างเดียว ไม่ขยับแขน

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2

import valve_detector as vd
from arm import Arm, _polar_to_xy
from camera import WristCamera
from geometry import valve_pose
from ik_solver import solve_ik

OUT_JSON = 'pixel_scale.json'
CAM_BACK_MM = 78.0        # กล้องอยู่หลังปลาย gripper (ค่าเดียวกับ collect_dataset.py)
DTHETA_DEG = 12.0         # ขยับ theta เท่านี้ต่อการวัดหนึ่งครั้ง
                          # ★ เดิม 3° แต่ J1 ขยับภาพแนวนอนได้น้อยมาก (ดูหมายเหตุ
                          #   CAM_YAW_OFFSET_DEG) — วัดที่ 3° ได้สัญญาณแค่ 1-3 px
                          #   ซึ่งจมสัญญาณรบกวน ต้องขยับแรงขึ้นให้สัญญาณชัดพอวัดได้
DZ_MM = 10.0              # ขยับ z เท่านี้
SETTLE_SEC = 1.2          # รอให้แขนนิ่งก่อนถ่าย (servo สั่นต่อหลังหยุด)

# ★ ค่าชดเชยความเอียงของกล้องจากแกน gripper — วัดจริง 2026-08 ที่จุ๊บ 7.8 นาฬิกา
#
#   สูตรเรขาคณิตวางปลายแขนบนแนวแกน gripper ที่ลากผ่านจุ๊บ ซึ่งควรทำให้กล้อง
#   (ที่อยู่หลังบนแกนเดียวกัน) เล็งตรงไปที่จุ๊บ — แต่ของจริงกล้องเล็งไปที่ "ดุมล้อ"
#   เพราะตัวกล้องติดเอียงจากแกน gripper
#
#   กวาด theta หาค่าจริง: เจอจุ๊บที่ dtheta -5 ถึง -30 · ดีสุดที่ -20 (conf 0.73)
#   → ต้องเล็ง theta "น้อยกว่า" ตำแหน่งจุ๊บ 20 องศา กล้องถึงจะเห็น
#
#   ⚠️ ยังไม่ได้ยืนยันว่าค่านี้คงที่ทุกตำแหน่งนาฬิกาและทุกระยะ
#      ถ้าเป็นการ "เยื้องเชิงมุม" จะคงที่ · ถ้าเป็น "เยื้องเชิงระยะ" จะเปลี่ยนตามระยะ
#      ต้องวัดซ้ำที่ตำแหน่งอื่นก่อนเชื่อ (ดูหมายเหตุใน DESIGN.md)
# ★ 2026-08 ปรับกล้องด้วยมือให้ตรงแกน gripper แล้ว (ขันสกรูแน่นขึ้น แก้ปัญหา
#   กล้องหลวมที่ทำให้ตำแหน่งกระโดดสองโหมดห่างกัน 280 px) ยืนยันด้วยการยิงซ้ำ
#   5 รอบที่ท่าเดิม ได้ผลนิ่งในช่วง ±7 px รอบค่าเฉลี่ย x-39 y-45
#   ค่าเก่า -20/-40 เป็นค่าจากตอนกล้องยังเอียงมาก ใช้ไม่ได้อีกต่อไป
CAM_YAW_OFFSET_DEG = 0.0
CAM_DZ_OFFSET_MM = 0.0


def detect_valve(cam, sess, inp, out):
    """คืนจุดกึ่งกลางกรอบของจุ๊บที่มั่นใจที่สุด หรือ None ถ้าหาไม่เจอ"""
    cam.grab()                       # ทิ้งเฟรมค้างใน buffer
    img = cam.grab()
    if img is None:
        return None, None
    h, w = img.shape[:2]
    blob, sc, pl, pt = vd.preprocess(img)
    dets = vd.postprocess(sess.run([out], {inp: blob})[0], w, h, sc, pl, pt)
    if not dets:
        return None, img
    x1, y1, x2, y2, conf, _ = max(dets, key=lambda d: d[4])
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0, conf), img


def start_pose(clock: float, dist: float):
    """ท่าที่กล้องเล็งไปที่จุ๊บ ที่ระยะ dist — วิธีเดียวกับ collect_dataset.py

    วางปลายแขนบนแนวแกน gripper ที่ลากผ่านจุ๊บ กล้องซึ่งอยู่หลังบนแกนเดียวกัน
    จึงเล็งตรงไปที่จุ๊บ
    """
    r_v, th_v, z_v = valve_pose(clock)
    s = dist - CAM_BACK_MM
    for pitch in (0, 10, -10, 20, -20, 30, 40, 50):
        b = math.radians(pitch)
        r = r_v - s * math.cos(b)
        z = z_v + s * math.sin(b)
        th = th_v + CAM_YAW_OFFSET_DEG
        z += CAM_DZ_OFFSET_MM
        if solve_ik(*_polar_to_xy(r, th), z, float(pitch)) is not None:
            return r, th, z, float(pitch)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clock', type=float, required=True, help='ตำแหน่งนาฬิกาของจุ๊บตอนนี้')
    ap.add_argument('--dist', type=float, default=150.0, help='ระยะกล้องถึงจุ๊บ (มม.)')
    ap.add_argument('--rounds', type=int, default=3)
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    pose = start_pose(args.clock, args.dist)
    if pose is None:
        print(f'✗ ที่ {args.clock:g} นาฬิกา ระยะ {args.dist:.0f} มม. แขนเอื้อมไม่ถึง — ลอง --dist อื่น')
        sys.exit(1)
    r0, th0, z0, pitch = pose

    print('=' * 62)
    print(f'วัดอัตราส่วนพิกเซล — จุ๊บที่ {args.clock:g} นาฬิกา · ระยะ {args.dist:.0f} มม.')
    print(f'  ท่าเริ่มต้น: r={r0:.0f} theta={th0:.0f}° z={z0:.0f} pitch={pitch:+.0f}°')
    print(f'  แต่ละรอบ: nudge theta {DTHETA_DEG:+.0f}° แล้ว nudge z {DZ_MM:+.0f} มม.')
    print('=' * 62)
    if args.dry_run:
        print('(dry-run — ไม่ขยับแขน)')
        return

    arm = Arm()
    cam = WristCamera()
    for _ in range(15):
        cam.grab()
        time.sleep(0.1)
    sess, inp, out = vd.load_model()

    results = []
    try:
        # ★ รอบแรกของสคริปต์มักเลื่อนมากกว่ารอบอื่นอย่างมีนัยสำคัญ (วัดซ้ำ 2 ครั้ง
        #   เจอเหมือนกันทั้งคู่ — y เพี้ยนไป ~30% เทียบรอบหลังๆ) ทั้งที่สั่งท่าเดิมเป๊ะ
        #   คาดว่าเป็นการตกตะกอนของกล้อง/แขนที่ยังไม่นิ่งพอในรอบแรกหลังเปิดสคริปต์
        #   แก้ด้วยรอบอุ่นเครื่องทิ้ง 1 รอบก่อนเริ่มเก็บผลจริง
        print('[อุ่นเครื่อง]')
        arm.go_scan_pose()
        time.sleep(1.0)
        arm.move_to(r0, th0, z0, pitch)
        time.sleep(SETTLE_SEC * 2)

        for rd in range(1, args.rounds + 1):
            print(f'\n[รอบ {rd}]')
            if not arm.move_to(r0, th0, z0, pitch):
                print('  ✗ ไปท่าเริ่มต้นไม่ได้'); break
            time.sleep(SETTLE_SEC)

            p0, _ = detect_valve(cam, sess, inp, out)
            if p0 is None:
                print('  ✗ หาจุ๊บไม่เจอที่ท่าเริ่มต้น — ตรวจว่า --clock ตรงกับของจริงไหม')
                break
            print(f'  จุด 0: ({p0[0]:.0f}, {p0[1]:.0f}) conf={p0[2]:.2f}')

            # ── ขยับ theta อย่างเดียว → วัดแกน x ──────────────────────
            if not arm.nudge(DTHETA_DEG, 0.0):
                print('  ✗ nudge theta ถูกปฏิเสธ'); break
            time.sleep(SETTLE_SEC)
            p1, _ = detect_valve(cam, sess, inp, out)
            if p1 is None:
                print('  ✗ หาจุ๊บไม่เจอหลัง nudge theta'); break
            du = p1[0] - p0[0]
            print(f'  จุด 1: ({p1[0]:.0f}, {p1[1]:.0f}) conf={p1[2]:.2f}   ภาพเลื่อน x {du:+.0f} px')

            # ── ขยับ z อย่างเดียว → วัดแกน y ─────────────────────────
            if not arm.nudge(0.0, DZ_MM):
                print('  ✗ nudge z ถูกปฏิเสธ'); break
            time.sleep(SETTLE_SEC)
            p2, _ = detect_valve(cam, sess, inp, out)
            if p2 is None:
                print('  ✗ หาจุ๊บไม่เจอหลัง nudge z'); break
            dv = p2[1] - p1[1]
            print(f'  จุด 2: ({p2[0]:.0f}, {p2[1]:.0f}) conf={p2[2]:.2f}   ภาพเลื่อน y {dv:+.0f} px')

            if abs(du) < 3 or abs(dv) < 3:
                print('  ⚠ ภาพเลื่อนน้อยเกินไป ค่าที่ได้จะไม่นิ่ง — ข้ามรอบนี้')
                continue
            results.append((DTHETA_DEG / du, DZ_MM / dv, du, dv))
            print(f'  → deg_per_px_x = {DTHETA_DEG/du:+.5f}   mm_per_px_y = {DZ_MM/dv:+.4f}')
    finally:
        cam.close()
        arm.go_scan_pose()
        print('\nกลับท่าสแกนแล้ว')

    if not results:
        print('\n✗ วัดไม่สำเร็จสักรอบ')
        sys.exit(1)

    # ── ตรวจเครื่องหมายให้ตรงกันทุกรอบ (เกณฑ์ผ่านของ Task 8) ────────────
    sx = {(1 if a > 0 else -1) for a, _, _, _ in results}
    sy = {(1 if b > 0 else -1) for _, b, _, _ in results}
    print('\n' + '=' * 62)
    print(f'ได้ผล {len(results)} จาก {args.rounds} รอบ')
    for i, (a, b, du, dv) in enumerate(results, 1):
        print(f'  รอบ {i}: deg_per_px_x={a:+.5f}  mm_per_px_y={b:+.4f}  (du={du:+.0f} dv={dv:+.0f})')

    if len(sx) > 1 or len(sy) > 1:
        print('\n✗ เครื่องหมายไม่ตรงกันทุกรอบ — การตรวจจับไม่นิ่ง ห้ามใช้ค่านี้')
        print('  ลองเพิ่ม --dist หรือเช็คว่าจุ๊บอยู่ในเฟรมชัดเจนทุกครั้ง')
        sys.exit(1)

    ax = sum(a for a, _, _, _ in results) / len(results)
    ay = sum(b for _, b, _, _ in results) / len(results)
    spread_x = (max(abs(a) for a, _, _, _ in results) / min(abs(a) for a, _, _, _ in results) - 1) * 100
    spread_y = (max(abs(b) for _, b, _, _ in results) / min(abs(b) for _, b, _, _ in results) - 1) * 100
    print(f'\nเฉลี่ย: deg_per_px_x = {ax:+.5f}   mm_per_px_y = {ay:+.4f}')
    print(f'กระจาย: x {spread_x:.0f}%  y {spread_y:.0f}%   (เกณฑ์ผ่านคือไม่เกิน 20%)')

    # ★ ไม่เขียนไฟล์ถ้าไม่ผ่านเกณฑ์ — ค่าคาลิเบรตที่มั่วแล้วหลุดไปให้ fine.py ใช้
    #   คือบั๊กแบบเดียวกับ solve_ik_clamped เดิม: ระบบเดินต่อได้โดยไม่มีสัญญาณว่าผิด
    if spread_x > 20 or spread_y > 20:
        print(f'\n✗ กระจายเกินเกณฑ์ 20% — ไม่เขียน {OUT_JSON}')
        print('  ค่าที่ไม่นิ่งขนาดนี้ถ้าเอาไปใช้ ลูปเฟสละเอียดจะแกว่งหรือวิ่งหนีเป้า')
        print('  ลองวัดซ้ำที่ระยะอื่น หรือเพิ่ม --rounds')
        sys.exit(1)

    data = {
        'deg_per_px_x': round(ax, 6),
        'mm_per_px_y': round(ay, 5),
        'measured_at_cm': round(args.dist / 10.0, 1),
        'measured_at_clock': args.clock,
        'rounds': len(results),
        'spread_pct': {'x': round(spread_x, 1), 'y': round(spread_y, 1)},
        # เห็นปลาย gripper ในเฟรม (ยืนยันแล้วใน Task 4) และมันติดแน่นกับกล้อง
        # ตำแหน่งในภาพจึงคงที่เสมอ — fine.py ใช้จุดนี้เป็นเป้าให้จุ๊บมาทับ
        'aim_x': None,
        'aim_y': None,
        'aim_from': 'gripper_visible',
        'note': 'aim_x/aim_y ยังไม่ได้วัด — ดู Task 8 Step 4',
    }
    with open(OUT_JSON, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    print(f'\nเขียน {OUT_JSON} แล้ว')


if __name__ == '__main__':
    main()
