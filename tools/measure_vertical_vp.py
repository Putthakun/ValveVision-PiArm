#!/usr/bin/env python3
# tools/measure_vertical_vp.py — หา "จุดรวมสายตาแนวดิ่ง" ของแต่ละท่าสแกน
#
# ปัญหา: ที่ท่าสแกนกล้องก้มลงมองล้อ เส้นดิ่งจริงในฉากจึงไม่ตั้งตรงในภาพ แต่ลู่เข้าหา
# จุดเดียวใต้ภาพ เรียกว่า "จุดรวมสายตา" (vanishing point, VP) — เหมือนมองเสาตึก
# จากที่สูงแล้วเห็นเสาลู่เข้าหากัน coarse.py เคยถือว่า "ลงล่างในภาพ = ดิ่งจริง"
# เลยอ่านมุมนาฬิกาเพี้ยน ~10° (2026-09-13: ตั้ง 6 นาฬิกา อ่านได้ 5.62)
#
# ทิศ "ลงดิ่งจริง" ที่ดุมล้อ = ทิศจากดุมไปหา VP → ต้องรู้ VP ของแต่ละท่าสแกน
# ค่านี้เป็นเรขาคณิตของท่าสแกน (มุมกล้อง) ไม่ใช่ค่าชดเชยมุมตายตัว ถ้าท่าสแกนเปลี่ยนต้องวัดใหม่
#
# วิธีวัด: แขวนเส้นดิ่ง (ตลับเมตรห้อยด้วยน้ำหนักตัวตลับเอง) ให้พาดผ่านดุมล้อ
# และให้มีเส้นดิ่งอีกอย่างน้อย 1 เส้นในภาพที่ห่างออกไป (เช่น ขาโต๊ะ)
# สคริปต์หาเส้นเกือบดิ่งที่ยาว แล้วหาจุดตัดของทุกเส้นแบบ least squares
#
# วิธีใช้ (ต้องปิดสตรีมกล้องก่อน):
#   python3 tools/measure_vertical_vp.py                        # ขยับแขนไปท่า A และ B แล้วถ่าย
#   python3 tools/measure_vertical_vp.py --images a.jpg b.jpg   # ใช้ภาพที่ถ่ายไว้ ไม่ขยับแขน
#
# ★ เปิดดูภาพ /tmp/vp_A.jpg และ /tmp/vp_B.jpg ทุกครั้ง — ต้องเห็นเส้นเขียวทับเส้นดิ่งจริง
#   เท่านั้น ถ้าไปทับขอบยางหรือของอื่น ค่าที่ได้ใช้ไม่ได้

import argparse
import json
import math
import os
import sys
import time
from datetime import date

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

OUT_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scan_vp.json")

MAX_TILT_DEG = 25.0      # เส้นที่เอียงจากแนวตั้งของภาพเกินนี้ ไม่นับว่าเป็นเส้นดิ่ง
MIN_LEN_PX = 250         # สั้นกว่านี้มักเป็นขอบรูหรือตัวหนังสือ
MERGE_ANGLE_DEG = 3.0    # ส่วนของเส้นที่มุมต่างกันไม่เกินนี้ ...
MERGE_X_PX = 40          # ... และอยู่ห่างกันไม่เกินนี้ ถือเป็นเส้นเดียวกัน (เช่น ขอบซ้าย-ขวาของสายตลับเมตร)
MIN_SPREAD_DEG = 3.0     # เส้นดิ่งต้องเอียงต่างกันอย่างน้อยเท่านี้ ไม่งั้นจุดตัดไม่แม่น


def find_vertical_lines(frame: np.ndarray) -> list[dict]:
    """หาเส้นเกือบดิ่งที่ยาว แล้วรวมส่วนย่อยที่เป็นเส้นเดียวกันเข้าด้วยกัน"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 60, 160)
    segs = cv2.HoughLinesP(edges, 1, np.pi / 720, 150, minLineLength=MIN_LEN_PX, maxLineGap=15)
    if segs is None:
        return []
    h = frame.shape[0]

    lines: list[dict] = []
    for x1, y1, x2, y2 in np.asarray(segs).reshape(-1, 4).astype(float):
        if y2 < y1:
            x1, y1, x2, y2 = x2, y2, x1, y1
        tilt = math.degrees(math.atan2(x2 - x1, y2 - y1))   # + = ปลายล่างเบนขวา
        if abs(tilt) > MAX_TILT_DEG:
            continue
        x_mid = x1 + (x2 - x1) * (h / 2 - y1) / (y2 - y1)   # x ที่กลางความสูงภาพ
        for ln in lines:
            if abs(ln["tilt"] - tilt) <= MERGE_ANGLE_DEG and abs(ln["x_mid"] - x_mid) <= MERGE_X_PX:
                ln["pts"] += [(x1, y1), (x2, y2)]
                break
        else:
            lines.append({"tilt": tilt, "x_mid": x_mid, "pts": [(x1, y1), (x2, y2)]})

    for ln in lines:
        vx, vy, x0, y0 = cv2.fitLine(np.array(ln["pts"], np.float32), cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        ln["dir"] = (float(vx), float(vy))
        ln["p0"] = (float(x0), float(y0))
        ln["length"] = max(p[1] for p in ln["pts"]) - min(p[1] for p in ln["pts"])
    return lines


def vanishing_point(lines: list[dict]) -> tuple[float, float]:
    """จุดที่ใกล้ทุกเส้นที่สุด (least squares ถ่วงน้ำหนักด้วยความยาวเส้น)"""
    A = np.zeros((2, 2))
    b = np.zeros(2)
    for ln in lines:
        dx, dy = ln["dir"]
        n = np.array([-dy, dx])            # เวกเตอร์ตั้งฉากกับเส้น
        w = ln["length"]
        A += w * np.outer(n, n)
        b += w * n * np.dot(n, ln["p0"])
    vx, vy = np.linalg.solve(A, b)
    return float(vx), float(vy)


def measure(frame: np.ndarray, tag: str) -> tuple[float, float] | None:
    lines = find_vertical_lines(frame)
    tilts = [ln["tilt"] for ln in lines]
    print(f"[{tag}] เจอเส้นเกือบดิ่ง {len(lines)} เส้น: " + ", ".join(f"{t:+.1f}°" for t in tilts))

    vis = frame.copy()
    for ln in lines:
        (dx, dy), (x0, y0) = ln["dir"], ln["p0"]
        cv2.line(vis, (int(x0 - dx * 2000), int(y0 - dy * 2000)),
                 (int(x0 + dx * 2000), int(y0 + dy * 2000)), (0, 255, 0), 2)

    if len(lines) < 2 or max(tilts) - min(tilts) < MIN_SPREAD_DEG:
        cv2.imwrite(f"/tmp/vp_{tag}.jpg", vis)
        print(f"[{tag}] ❌ ต้องมีเส้นดิ่งอย่างน้อย 2 เส้นที่เอียงต่างกัน ≥ {MIN_SPREAD_DEG:.0f}° "
              f"— ตรวจภาพ /tmp/vp_{tag}.jpg")
        return None

    vp = vanishing_point(lines)
    print(f"[{tag}] จุดรวมสายตาแนวดิ่ง = ({vp[0]:.0f}, {vp[1]:.0f})")

    # ลูกศรจากกลางภาพชี้ทิศ "ลงดิ่งจริง" ให้ตรวจด้วยตา
    h, w = frame.shape[:2]
    c = np.array([w / 2, h / 2])
    d = np.array(vp) - c
    d = d if d[1] > 0 else -d
    tip = c + d / np.linalg.norm(d) * 200
    cv2.arrowedLine(vis, tuple(c.astype(int)), tuple(tip.astype(int)), (0, 0, 255), 4, tipLength=0.15)
    cv2.imwrite(f"/tmp/vp_{tag}.jpg", vis)
    return vp


def grab_at_scan_poses() -> dict:
    from arm import Arm
    from camera import WristCamera

    arm = Arm()
    cam = WristCamera()
    frames = {}
    try:
        for upper, tag in ((False, "A"), (True, "B")):
            arm.go_scan_pose(upper=upper)
            time.sleep(2.0)          # รอแขนนิ่ง
            for _ in range(5):       # ทิ้งเฟรมแรกๆ ที่ค่าแสงยังไม่นิ่ง
                frame = cam.grab()
            frames[tag] = frame
    finally:
        cam.close()
    return frames


def main() -> None:
    ap = argparse.ArgumentParser(description="หาจุดรวมสายตาแนวดิ่งของท่าสแกน A และ B")
    ap.add_argument("--images", nargs=2, metavar=("A_JPG", "B_JPG"), help="ใช้ภาพที่ถ่ายไว้แทนการขยับแขน")
    ap.add_argument("--dry-run", action="store_true", help="คำนวณอย่างเดียว ไม่เขียน scan_vp.json")
    args = ap.parse_args()

    if args.images:
        frames = {"A": cv2.imread(args.images[0]), "B": cv2.imread(args.images[1])}
    else:
        frames = grab_at_scan_poses()

    result = {}
    for tag in ("A", "B"):
        vp = measure(frames[tag], tag)
        if vp is None:
            sys.exit(1)
        result[tag] = {"vp": [round(vp[0], 1), round(vp[1], 1)], "measured": date.today().isoformat()}

    if args.dry_run:
        print("(dry-run — ไม่ได้บันทึก)")
        return
    with open(OUT_PATH, "w") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f"บันทึกแล้ว → {OUT_PATH}")


if __name__ == "__main__":
    main()
