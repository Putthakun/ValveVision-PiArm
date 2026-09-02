# coarse.py — เฟสหยาบ: หาว่าจุ๊บอยู่โซนไหน แล้วพาแขนเข้าใกล้ (Task 9)
#
# หน้าที่เดียว: พาแขนเข้าใกล้พอให้กล้องที่มือเห็นจุ๊บในเฟสละเอียด **ไม่ต้องแม่น**
#
# ★ ห้ามมีสูตรแปลง pixel → มิลลิเมตรในไฟล์นี้ (กฎข้อ 3 ของ CLAUDE.md)
#   ใช้แค่ "มุม" ของจุ๊บรอบดุมล้อในภาพ (ไม่ใช่ระยะพิกเซล) แล้วแปลงเป็นมุมนาฬิกา
#   แทนที่ geometry.valve_pose(clock) ทันที — ไม่มีอัตราส่วนพิกเซล/มม. เข้ามาเกี่ยวเลย
#
# ★ ทำไมตรวจจับดุมล้อสดทุกครั้ง แทนที่จะใช้พิกัดดุมคงที่จากการ calibrate ไว้ล่วงหน้า:
#   วันนี้พิสูจน์แล้วว่าตำแหน่งกล้องบนแขนไม่นิ่งสนิทข้ามเซสชัน (จุด J5 มีระยะหย่อน)
#   ถ้าจำพิกัดดุมเป็นค่าคงที่ พอกล้องขยับแม้เล็กน้อยค่าที่จำไว้จะผิดทันทีโดยไม่รู้ตัว
#   ดุมล้อเป็นวงกลมมืดเด่นชัดในทุกเฟรมอยู่แล้ว ตรวจจับสดทุกครั้งจึงเชื่อถือได้กว่า

import math
from dataclasses import dataclass

import cv2
import numpy as np

from arm import Arm
from camera import BaseCamera
from geometry import valve_pose

CONFIRM_FRAMES_DEFAULT = 3
MAX_ATTEMPTS_PER_POSE = 10   # ลองถ่ายกี่เฟรมก่อนเปลี่ยนไปท่าสแกนอีกท่า


@dataclass
class CoarseResult:
    ok: bool
    r: float = 0.0
    theta_deg: float = 0.0
    z: float = 0.0
    pitch_deg: float = 0.0
    reason: str = "ok"   # "ok" | "ไม่เจอวาล์ว" | "เอื้อมไม่ถึง" | "กล้องมีปัญหา"


def _find_hub(frame: np.ndarray) -> tuple[float, float] | None:
    """หาศูนย์กลางดุมล้อในภาพ — วงกลมมืดที่ใหญ่ที่สุดในเฟรม คืน (x, y) หรือ None

    วิธีเดียวกับที่ใช้ตรวจสอบด้วยมือมาตลอด Task 5B/8: threshold หาบริเวณมืด
    แล้วเลือกรูปทรงกลมที่สุดและใหญ่ที่สุด (ดุมล้อใหญ่กว่ารูระบายอากาศ/น็อตชัดเจน)
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, th = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for c in contours:
        (cx, cy), r = cv2.minEnclosingCircle(c)
        if not (40 < r < 120):
            continue
        circularity = cv2.contourArea(c) / (math.pi * r * r)
        if circularity < 0.55:
            continue
        if best is None or r > best[2]:
            best = (cx, cy, r)

    return (best[0], best[1]) if best else None


def _pixel_angle_to_clock(hub_xy: tuple[float, float], valve_xy: tuple[float, float]) -> float:
    """แปลงตำแหน่งจุ๊บในภาพ (เทียบดุม) → มุมนาฬิกา

    ใช้แค่ทิศทาง (atan2) ไม่ใช่ระยะพิกเซล จึงไม่ใช่สูตร pixel→mm
    เป็นเรขาคณิตกลับด้านของ geometry.valve_pose(): phi = radians((6-clock)*30)
    โดย phi=0 คือ "ขึ้นบน" (12 นาฬิกา) ในภาพ — ตรงกับนิยามใน CONTEXT.md
    (นับจากมุมมองกล้องที่มองเข้าหาหน้าล้อ)

    ★ ไม่ต้องแม่น — ค่านี้ป้อนต่อให้ geometry.valve_pose() หาตำแหน่งเข้าใกล้เฉยๆ
      ความคลาดเคลื่อนจากมุมมองกล้องไม่ตรงเป๊ะ/เลนส์บิดเบี้ยว จะถูกเฟสละเอียดแก้ทีหลัง
    """
    hx, hy = hub_xy
    vx, vy = valve_xy
    dx, dy = vx - hx, vy - hy
    # ⚠️ กับดักทิศ (แบบเดียวกับที่ geometry.valve_pose เตือนไว้) — เจอจริงจากการ
    # ทดสอบบนแขนจริง (2026-08): ตั้งจุ๊บไว้ 7 นาฬิกา แต่ระบบตีความเป็น 5 นาฬิกา
    # ซึ่งเป็นภาพสะท้อนซ้าย-ขวากันข้ามแกน 12-6 พอดี ต้องกลับเครื่องหมาย dx
    # (ทิศทางซ้าย-ขวาของภาพเทียบกับดุมล้อ ขึ้นกับว่ากล้องประกอบคว่ำ/หงายทางไหน
    #  ซึ่งอาจเปลี่ยนได้ทุกครั้งที่ถอด-ประกอบกล้องใหม่ ไม่ใช่ค่าคงที่ทางฟิสิกส์)
    phi = math.atan2(-dx, -dy)   # 0° = ขึ้นบน, +90° = ซ้ายในภาพ, 180°/-180° = ลงล่าง
    clock = (math.degrees(phi) / 30.0 + 12.0) % 12.0
    return 12.0 if clock == 0.0 else clock


def _detect_once(cam: BaseCamera, session) -> tuple[float, float] | None:
    """ถ่าย 1 เฟรม หาทั้งกล่องจุ๊บและดุมล้อ คืนตำแหน่งจุ๊บเทียบดุมเป็นมุมนาฬิกา หรือ None"""
    sess, input_name, output_name = session

    frame = cam.grab()
    if frame is None:
        return None

    from valve_detector import postprocess, preprocess

    h, w = frame.shape[:2]
    blob, scale, pad_left, pad_top = preprocess(frame)
    dets = postprocess(sess.run([output_name], {input_name: blob})[0], w, h, scale, pad_left, pad_top)
    if not dets:
        return None

    x1, y1, x2, y2, _, _ = max(dets, key=lambda d: d[4])
    valve_xy = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    hub_xy = _find_hub(frame)
    if hub_xy is None:
        return None

    return _pixel_angle_to_clock(hub_xy, valve_xy)


def _scan_from_pose(cam: BaseCamera, session, arm: Arm, upper: bool, confirm_frames: int) -> float | None:
    """ไปท่าสแกน (A หรือ B) แล้วลองหาจุ๊บจนกว่าจะเจอติดกัน confirm_frames เฟรม"""
    arm.go_scan_pose(upper=upper)

    streak = 0
    last_clock = None
    for _ in range(MAX_ATTEMPTS_PER_POSE):
        clock = _detect_once(cam, session)
        if clock is not None:
            streak += 1
            last_clock = clock
            if streak >= confirm_frames:
                return last_clock
        else:
            streak = 0
    return None


def coarse_locate(cam: BaseCamera, session, arm: Arm, *, confirm_frames: int = CONFIRM_FRAMES_DEFAULT) -> CoarseResult:
    """หาว่าจุ๊บอยู่โซนไหนจากกล้องที่มือ (ที่ท่าสแกน) แล้วพาแขนเข้าใกล้

    ลองท่าสแกน A (ครึ่งล่าง) ก่อน ไม่เจอค่อยลองท่า B (ครึ่งบน) — ดู config.py
    เหตุผลที่ต้องมี 2 ท่า (เฟรมเดียวครอบวงจุ๊บทั้งวงไม่ได้)
    """
    clock = _scan_from_pose(cam, session, arm, upper=False, confirm_frames=confirm_frames)
    if clock is None:
        clock = _scan_from_pose(cam, session, arm, upper=True, confirm_frames=confirm_frames)
    if clock is None:
        return CoarseResult(ok=False, reason="ไม่เจอวาล์ว")

    r, theta_deg, z = valve_pose(clock)
    pitch_deg = arm.best_pitch(r, theta_deg, z)
    if pitch_deg is None:
        return CoarseResult(ok=False, reason="เอื้อมไม่ถึง")

    if not arm.move_to(r, theta_deg, z, pitch_deg):
        return CoarseResult(ok=False, reason="เอื้อมไม่ถึง")

    return CoarseResult(ok=True, r=r, theta_deg=theta_deg, z=z, pitch_deg=pitch_deg, reason="ok")
