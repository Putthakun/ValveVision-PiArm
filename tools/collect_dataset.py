#!/usr/bin/env python3
# tools/collect_dataset.py — แขนกวาดเก็บภาพ dataset เอง (Task 6)
#
# คนทำแค่: หมุนล้อไปตำแหน่งนาฬิกา · สลับไฟ · ขยับล้อเล็กน้อยในกรอบเทป
# ที่เหลือสคริปต์ทำเอง
#
# วิธีใช้ — หมุนล้อครั้งเดียว เก็บครบทั้ง 4 สภาพแสง:
#   python3 tools/collect_dataset.py --clock 6
#
# เก็บเฉพาะบางสภาพแสง:
#   python3 tools/collect_dataset.py --clock 6 --lights day,room
#
# ดูว่าจะถ่ายกี่ภาพโดยไม่ขยับแขน:
#   python3 tools/collect_dataset.py --clock 6 --dry-run
#
# ภาพเก็บที่ data/raw/<clock>_<light>/  ชื่อไฟล์ <clock>_<light>_<ลำดับ>_<ชนิด>.jpg
#   ชนิด = scanA (ท่าสแกนครึ่งล่าง) · scanB (ครึ่งบน) · near (ระยะใกล้) · neg (ภาพลบ)
#
# ★ ชื่อโฟลเดอร์คือ "รอบการเก็บ" ซึ่งใช้แบ่ง train/val — ห้ามสุ่มแบ่งรายภาพ

import argparse
import math
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2

from arm import Arm, _polar_to_xy
from ik_solver import solve_ik
from camera import WristCamera
from geometry import valve_pose

OUT_ROOT = 'data/raw'
LIGHTS = ['day', 'room', 'dim', 'torch']
LIGHT_NAMES = {
    'day': 'กลางวัน (เปิดผ้าม่าน ปิดไฟห้อง)',
    'room': 'ไฟห้องปกติ',
    'dim': 'ไฟสลัว (ปิดไฟหลัก เหลือไฟอ่อน)',
    'torch': 'ไฟฉายส่องจากด้านข้าง',
}

CAM_BACK_MM = 78.0        # กล้องอยู่หลังปลาย gripper ประมาณเท่านี้ (ประมาณจากภาพ)

# ★ กล้องติดเอียงจากแกน gripper — วัดจริง 2026-08 ที่จุ๊บ 6 นาฬิกา 3 ระยะ
#   จุ๊บไปตกที่ (457, 587) แทนที่จะเป็นกลางเฟรม (640, 360) เหมือนกันทุกระยะ
#   = เยื้องเชิงมุม ~15° แนวนอน · ~22° แนวตั้ง (ไม่ใช่เยื้องเชิงระยะ)
#
#   ตอนเก็บ dataset ไม่ต้องแก้ — จุ๊บยังอยู่ในเฟรมและคมชัด อีกทั้งการที่จุ๊บ
#   ไม่อยู่กลางเฟรมเป๊ะทุกใบยังดีต่อการเทรนด้วย
#   ★ แต่ fine.py (Task 10) ต้องรู้ค่านี้ ไม่งั้นจะเล็งพลาดตั้งแต่ก้าวแรก
N_SCAN_EACH = 3           # ภาพต่อท่าสแกนหนึ่งท่า (A และ B อย่างละ 3 = 6)
N_NEAR_MAX = 25           # ภาพระยะใกล้สูงสุดต่อรอบ
N_NEG = 3                 # ภาพลบต่อรอบ
SCAN_JITTER_DEG = 1.5     # สั่นท่าสแกนเล็กน้อยเพื่อไม่ให้ได้ภาพซ้ำกันเป๊ะ

# ระยะกล้องถึงจุ๊บที่จะกวาด (มม.) — ตัวที่เอื้อมไม่ถึงจะถูกข้ามอัตโนมัติ
# ★ ตารางเดิมในแผน (350/280/220/180/150) เอื้อมไม่ถึงเกือบหมด
#   ระยะที่ทำได้จริงคือ ~90–220 มม. และที่ 6 นาฬิกาได้แค่ ~90–140
NEAR_DISTANCES = [100, 120, 140, 170, 200]

# มุม pitch ที่จะกวาด — แต่ละค่าคือ "มุมที่กล้องมองจุ๊บ" คนละมุมกัน
# กวาดหลายมุมเพื่อให้ dataset มีมุมมองหลากหลาย
NEAR_PITCHES = [-20, -10, 0, 10, 20, 30, 40, 50]

# เยื้องซ้าย-ขวาเล็กน้อย ให้จุ๊บไม่อยู่กลางเฟรมเป๊ะทุกใบ (องศาของ theta)
NEAR_DTHETA = [0, 5, -5]


def clock_tag(clock: float) -> str:
    """6 → '06' · 6.5 → '065' — ใช้เป็นส่วนหนึ่งของชื่อรอบ"""
    return f'{clock:g}'.replace('.', '')


def plan_near_poses(arm: Arm, clock: float):
    """สร้างรายการท่าระยะใกล้ที่ 'กล้องเล็งไปที่จุ๊บจริง' และเอื้อมถึง

    ★ ห้ามใช้ arm.best_pitch() ที่นี่ — มันเลือก pitch จากระยะขยับ joint ที่เหลือ
      ไม่ได้สนว่ากล้องจะหันไปทางไหน ผลคือ gripper (และกล้องที่อยู่หลัง) ชี้ก้มลงพื้น
      เก็บภาพมาได้แต่ขอบล้อกับพื้น ไม่เห็นจุ๊บ (เจอจริงตอนทดลองรอบแรก)

    วิธีที่ถูก: วางปลายแขนไว้ "บนแนวแกน gripper ที่ลากผ่านจุ๊บ" — กล้องซึ่งอยู่
    หลังปลายแขนบนแกนเดียวกันจึงเล็งตรงไปที่จุ๊บโดยอัตโนมัติ

        แกน gripper ที่ pitch = beta ชี้ไปทาง (cos beta, -sin beta) ในระนาบ (r, z)
        ถอยจากจุ๊บมาตามแกนนั้นเป็นระยะ s:
            r_tcp = r_valve - s*cos(beta)
            z_tcp = z_valve + s*sin(beta)
        กล้องอยู่ถัดไปอีก CAM_BACK_MM บนแกนเดียวกัน → ระยะกล้องถึงจุ๊บ = s + CAM_BACK_MM
    """
    r_v, th_v, z_v = valve_pose(clock)
    poses = []
    for dist in NEAR_DISTANCES:
        s = dist - CAM_BACK_MM          # ระยะจากปลาย gripper ถึงจุ๊บ
        if s < 15:                       # ใกล้เกินจนเสี่ยงชนจุ๊บ
            continue
        for pitch in NEAR_PITCHES:
            b = math.radians(pitch)
            r = r_v - s * math.cos(b)
            z = z_v + s * math.sin(b)
            for dth in NEAR_DTHETA:
                th = th_v + dth
                # ★ ต้องใช้ pitch ตัวเดียวกับที่ใช้คำนวณตำแหน่ง ไม่ใช่ best_pitch
                if solve_ik(*_polar_to_xy(r, th), z, float(pitch)) is not None:
                    poses.append((r, th, z, float(pitch), dist))
    random.shuffle(poses)
    return poses[:N_NEAR_MAX]


def plan_negative_poses(arm: Arm, clock: float):
    """ท่าเล็งไปที่ขอบล้อที่ 'ไม่มีจุ๊บ' — ได้ภาพรูระบายอากาศกับน็อตล้อเป็นภาพลบ

    เล็งไปตำแหน่งนาฬิกาที่ห่างจากจุ๊บ 4-5 ชั่วโมง เพื่อให้แน่ใจว่าจุ๊บไม่ติดมาในเฟรม
    """
    poses = []
    for offset in (4, -4, 5):
        r_v, th_v, z_v = valve_pose((clock + offset - 1) % 12 + 1)
        s = 130 - CAM_BACK_MM
        for pitch in (0, 15, -15, 30, -30, 45):     # ใช้มุมแรกที่เอื้อมถึง
            b = math.radians(pitch)
            r = r_v - s * math.cos(b)
            z = z_v + s * math.sin(b)
            if solve_ik(*_polar_to_xy(r, th_v), z, float(pitch)) is not None:
                poses.append((r, th_v, z, float(pitch)))
                break
    return poses[:N_NEG]


def grab_and_save(cam, path, warm=2):
    for _ in range(warm):
        cam.grab()
    frame = cam.grab()
    if frame is None:
        return False
    cv2.imwrite(path, frame)
    return True


def collect_round(arm, cam, clock, light, near_poses, neg_poses):
    session = f'{clock_tag(clock)}_{light}'
    out_dir = os.path.join(OUT_ROOT, session)
    os.makedirs(out_dir, exist_ok=True)
    n = 0

    def path(kind):
        nonlocal n
        n += 1
        return os.path.join(out_dir, f'{session}_{n:03d}_{kind}.jpg')

    # ── ภาพที่ท่าสแกน — ท่า A และ B อย่างละ N_SCAN_EACH ─────────────────
    for tag, upper in (('scanA', False), ('scanB', True)):
        for i in range(N_SCAN_EACH):
            # ใบแรกใช้ท่าจริง ใบต่อไปสั่นเล็กน้อยกันภาพซ้ำกันเป๊ะ
            arm.go_scan_pose(upper=upper, jitter_deg=0.0 if i == 0 else SCAN_JITTER_DEG)
            time.sleep(0.4)
            grab_and_save(cam, path(tag))
    print(f'    ท่าสแกน {N_SCAN_EACH * 2} ใบ', end='', flush=True)

    # ── ภาพระยะใกล้ ─────────────────────────────────────────────────────
    blurred = 0
    for i, (r, th, z, pitch, dist) in enumerate(near_poses):
        if not arm.move_to(r, th, z, pitch):
            continue
        # ทุกใบที่ 4 ถ่ายทันทีระหว่างแขนยังไม่นิ่ง → ได้ภาพเบลอที่ต้องมีใน dataset
        if i % 4 == 3:
            grab_and_save(cam, path('near'), warm=0)
            blurred += 1
        else:
            time.sleep(0.5)
            grab_and_save(cam, path('near'))
    print(f' · ระยะใกล้ {len(near_poses)} ใบ (เบลอ {blurred})', end='', flush=True)

    # ── ภาพลบ ───────────────────────────────────────────────────────────
    for r, th, z, pitch in neg_poses:
        if arm.move_to(r, th, z, pitch):
            time.sleep(0.5)
            grab_and_save(cam, path('neg'))
    print(f' · ภาพลบ {len(neg_poses)} ใบ')

    arm.go_scan_pose()
    return n, out_dir


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--clock', type=float, required=True, help='ตำแหน่งนาฬิกาของจุ๊บ (1-12 รับทศนิยม)')
    ap.add_argument('--lights', default=','.join(LIGHTS), help='สภาพแสงที่จะเก็บ คั่นด้วยคอมมา')
    ap.add_argument('--dry-run', action='store_true', help='คำนวณอย่างเดียว ไม่ขยับแขน ไม่ถ่ายภาพ')
    args = ap.parse_args()

    lights = [l.strip() for l in args.lights.split(',') if l.strip()]
    for l in lights:
        if l not in LIGHTS:
            print(f'✗ สภาพแสง "{l}" ไม่รู้จัก (ต้องเป็น {", ".join(LIGHTS)})')
            sys.exit(1)

    arm = Arm(simulate=True) if args.dry_run else Arm()
    near = plan_near_poses(arm, args.clock)
    neg = plan_negative_poses(arm, args.clock)
    per_round = N_SCAN_EACH * 2 + len(near) + len(neg)

    r0, th0, z0 = valve_pose(args.clock)
    print('=' * 62)
    print(f'เก็บ dataset — จุ๊บที่ {args.clock:g} นาฬิกา')
    print(f'  ตำแหน่งจุ๊บ: r={r0:.0f} มม. · theta={th0:.0f}° · z={z0:+.0f} มม.')
    print(f'  ท่าระยะใกล้ที่เอื้อมถึง {len(near)} จาก {len(NEAR_DISTANCES)*len(NEAR_PITCHES)*len(NEAR_DTHETA)} ท่าที่วางแผนไว้')
    if near:
        ds = sorted({p[4] for p in near})
        print(f'  ระยะที่ใช้ได้: {", ".join(str(d) for d in ds)} มม.')
    print(f'  ภาพต่อรอบ {per_round} ใบ × {len(lights)} สภาพแสง = {per_round*len(lights)} ใบ')
    print('=' * 62)

    if args.dry_run:
        print('\n(dry-run — ไม่ได้ขยับแขนและไม่ได้ถ่ายภาพ)')
        return
    if not near:
        print('\n✗ ไม่มีท่าระยะใกล้ที่เอื้อมถึงเลย — ตรวจเรขาคณิตใน geometry.py')
        sys.exit(1)

    try:
        cam = WristCamera()
    except Exception as e:
        print(f'\n✗ เปิดกล้องไม่ได้: {e}')
        print('  สาเหตุที่พบบ่อยที่สุด: มีสคริปต์อื่นจับกล้องอยู่ (เช่น tools/preview_cameras.py)')
        print('  ปิดด้วย:  pkill -f preview_cameras.py')
        sys.exit(1)

    for _ in range(15):     # warm-up ให้ auto-exposure ปรับตัว
        cam.grab()
        time.sleep(0.1)

    total = 0
    try:
        for light in lights:
            print(f'\n[{LIGHT_NAMES[light]}]')
            input('  1) ตั้งสภาพแสง  2) ขยับล้อเล็กน้อยในกรอบเทป  แล้วกด Enter ... ')
            n, out_dir = collect_round(arm, cam, args.clock, light, near, neg)
            total += n
            print(f'    → {n} ใบ ที่ {out_dir}')
    except KeyboardInterrupt:
        print('\n  หยุดกลางคัน')
    finally:
        cam.close()
        arm.go_scan_pose()

    print(f'\nรวม {total} ใบ · กลับท่าสแกนแล้ว')
    print(f'ตรวจภาพก่อนเก็บรอบต่อไป:  ls {OUT_ROOT}/{clock_tag(args.clock)}_*/')


if __name__ == '__main__':
    main()
