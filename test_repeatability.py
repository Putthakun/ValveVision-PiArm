# test_repeatability.py — เช็ค mechanical repeatability ของแขนกล
#
# หลักการ: สั่งแขนไปพิกัดเดิมซ้ำ N รอบ (กลับ scan pose ระหว่างรอบ)
# ถ่าย snapshot จาก camera_preview ทุกรอบ แล้วใช้ template matching
# วัดว่าแขนเลื่อนจากรอบแรกกี่ pixel → แปลงเป็น mm โดยประมาณ
#
# ต้องรัน camera_preview.py ค้างไว้ก่อน (ใช้ endpoint /snapshot)
# และ stop valvevision service ก่อน (กันชนกันคุม servo)
#
# วิธีใช้:
#   python test_repeatability.py                # default: (390, -100, 85) x 10 รอบ
#   python test_repeatability.py 390 -100 85 10

import math
import os
import sys
import time
import urllib.request

import cv2
import numpy as np

from servo_controller import ServoController
from ik_solver import solve_ik
from config import CAM_X, CAM_Y, CAM_Z, FOCAL_LENGTH

SNAPSHOT_URL = "http://localhost:8080/snapshot"
OUT_DIR      = "results/repeatability"
PATCH_SIZE   = 180   # px — ขนาด template patch รอบบริเวณแขน
SETTLE_SEC   = 1.5   # รอแขนนิ่งก่อนถ่าย


def fetch_snapshot() -> np.ndarray | None:
    try:
        with urllib.request.urlopen(SNAPSHOT_URL, timeout=5) as resp:
            data = np.frombuffer(resp.read(), dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)
    except Exception as e:
        print(f"[cam] snapshot ล้มเหลว: {e}")
        return None


def mm_per_px(x: float, y: float, z: float) -> float:
    """ประมาณ mm ต่อ pixel ที่ระยะของ target จากกล้อง"""
    dist = math.sqrt((x - CAM_X) ** 2 + (y - CAM_Y) ** 2 + (z - CAM_Z) ** 2)
    return dist / FOCAL_LENGTH


def find_arm_patch(base_gray: np.ndarray, ref_gray: np.ndarray) -> tuple[int, int]:
    """หาตำแหน่งแขนในภาพ ref: diff กับ baseline จะได้ 2 กลุ่ม —
    จุดที่แขนย้ายมา (มืด เพราะแขนสีดำ) กับจุดที่แขนย้ายออก (สว่าง เห็นล้อแทน)
    เลือกกลุ่มที่มืดกว่า = ตัวแขนจริง คืนมุมซ้ายบนของ patch"""
    diff = cv2.absdiff(base_gray, ref_gray)
    diff = cv2.GaussianBlur(diff, (9, 9), 0)
    _, mask = cv2.threshold(diff, 40, 255, cv2.THRESH_BINARY)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))

    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
    blobs = sorted(
        (
            (stats[j, cv2.CC_STAT_AREA], j)
            for j in range(1, n)
            if stats[j, cv2.CC_STAT_AREA] > 500
        ),
        reverse=True,
    )[:2]

    h, w = ref_gray.shape
    if not blobs:
        cx, cy = w // 2, h // 2
    else:
        # เลือก blob ที่มืดกว่าในภาพ ref (= ตัวแขน ไม่ใช่ล้อที่โผล่ออกมา)
        darkest = min(blobs, key=lambda b: ref_gray[labels == b[1]].mean())
        cx, cy = (int(v) for v in cents[darkest[1]])

    px = max(0, min(w - PATCH_SIZE, cx - PATCH_SIZE // 2))
    py = max(0, min(h - PATCH_SIZE, cy - PATCH_SIZE // 2))
    return px, py


def main():
    args = sys.argv[1:]
    if len(args) >= 3:
        x, y, z = float(args[0]), float(args[1]), float(args[2])
    else:
        x, y, z = 390.0, -100.0, 85.0
    n_trials = int(args[3]) if len(args) >= 4 else 10

    angles = solve_ik(x, y, z)
    if angles is None:
        print(f"({x:.0f}, {y:.0f}, {z:.0f}) อยู่นอก workspace — เลือกพิกัดใหม่")
        sys.exit(1)

    if fetch_snapshot() is None:
        print("ต่อ camera_preview ไม่ได้ — รัน python camera_preview.py ค้างไว้ก่อน")
        sys.exit(1)

    os.makedirs(OUT_DIR, exist_ok=True)
    scale = mm_per_px(x, y, z)
    print(f"เป้าหมาย ({x:.0f}, {y:.0f}, {z:.0f}) mm × {n_trials} รอบ")
    print(f"ระยะจากกล้อง ≈ {scale * FOCAL_LENGTH:.0f}mm → {scale:.2f} mm/px\n")

    arm = ServoController()
    print("[arm] ไป scan pose (baseline)...")
    arm.move_to_scan_pose()
    time.sleep(SETTLE_SEC)
    base_img = fetch_snapshot()
    cv2.imwrite(f"{OUT_DIR}/baseline_scanpose.jpg", base_img)

    snaps = []
    for i in range(1, n_trials + 1):
        print(f"[trial {i}/{n_trials}] ไป target...")
        arm.move_smooth(angles)
        time.sleep(SETTLE_SEC)
        img = fetch_snapshot()
        if img is None:
            print("  snapshot fail — ข้ามรอบนี้")
        else:
            cv2.imwrite(f"{OUT_DIR}/trial_{i:02d}.jpg", img)
            snaps.append((i, img))
        print(f"[trial {i}/{n_trials}] กลับ scan pose...")
        arm.move_to_scan_pose()
        time.sleep(0.5)

    if len(snaps) < 2:
        print("ภาพไม่พอวิเคราะห์")
        sys.exit(1)

    # ── วิเคราะห์: track ตัวแขนด้วย template matching ────────────────
    # ใช้รอบที่ 2 เป็น reference (รอบแรกอาจเพี้ยนจาก first-move transient)
    # จำกัด search window รอบตำแหน่งเดิม กันจับผิดลายล้อที่ซ้ำกัน
    ref_i, ref_img = snaps[1] if len(snaps) > 1 else snaps[0]
    base_gray = cv2.cvtColor(base_img, cv2.COLOR_BGR2GRAY)
    ref_gray  = cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY)
    px, py    = find_arm_patch(base_gray, ref_gray)
    template  = ref_gray[py:py + PATCH_SIZE, px:px + PATCH_SIZE]
    search    = 80   # px

    print(f"\ntemplate patch: ({px}, {py}) ขนาด {PATCH_SIZE}px (ref = trial {ref_i})")
    print(f"{'trial':>5} {'dx(px)':>8} {'dy(px)':>8} {'shift(px)':>10} {'≈mm':>7} {'conf':>6}")

    shifts_mm = []
    csv_lines = ["trial,dx_px,dy_px,shift_px,shift_mm,conf"]
    for i, img in snaps:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        x0, y0 = max(0, px - search), max(0, py - search)
        x1 = min(gray.shape[1], px + PATCH_SIZE + search)
        y1 = min(gray.shape[0], py + PATCH_SIZE + search)
        res = cv2.matchTemplate(gray[y0:y1, x0:x1], template, cv2.TM_CCOEFF_NORMED)
        _, conf, _, loc = cv2.minMaxLoc(res)
        dx, dy = (x0 + loc[0]) - px, (y0 + loc[1]) - py
        shift  = math.sqrt(dx * dx + dy * dy)
        flag   = ""
        if conf < 0.5:
            flag = "  ← match ไม่น่าเชื่อถือ (แขนไม่ถึงท่านี้?)"
        elif i != ref_i:
            shifts_mm.append(shift * scale)
        csv_lines.append(f"{i},{dx},{dy},{shift:.2f},{shift * scale:.2f},{conf:.2f}")
        print(f"{i:>5} {dx:>8} {dy:>8} {shift:>10.2f} {shift * scale:>7.2f} {conf:>6.2f}{flag}")

    if shifts_mm:
        mean_mm = sum(shifts_mm) / len(shifts_mm)
        max_mm  = max(shifts_mm)
        print(f"\nสรุป repeatability (เทียบ trial {ref_i}, เฉพาะรอบที่ track ได้):")
        print(f"  เฉลี่ย {mean_mm:.2f} mm | แย่สุด {max_mm:.2f} mm")
        print(f"  (measurement noise ~±{scale:.1f}mm จาก quantize 1px)")
        if max_mm <= 3.0:
            print("  ✅ นิ่งพอ (≤3mm) — ไป calibrate offset ต่อได้เลย")
        elif max_mm <= 6.0:
            print("  🟡 พอใช้ (3–6mm) — calibrate ได้ แต่อย่าคาดหวังแม่นกว่า scatter นี้")
        else:
            print("  ⚠️ เกิน 6mm — เช็ค horn/สกรูหลวม ก่อน calibrate")

    with open(f"{OUT_DIR}/summary.csv", "w") as f:
        f.write("\n".join(csv_lines) + "\n")
    print(f"\nบันทึกภาพ + summary.csv ไว้ที่ {OUT_DIR}/")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nยกเลิก")
