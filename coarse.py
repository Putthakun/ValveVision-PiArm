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

# ★ เว้นระยะก่อนถึงจุ๊บจริง — ไม่ใช่ค่าชดเชย error (ไม่ผิดกฎข้อ 1) แต่เป็นระยะที่
#   ดีไซน์ไว้ตั้งแต่แรกใน CONTEXT.md/DESIGN.md ให้กล้องยังโฟกัสเห็นจุ๊บได้ระหว่าง
#   เฟสละเอียด (กล้องโฟกัสใกล้กว่า ~12 ซม.ไม่ได้แล้ว)
#
#   ★ ลองใช้ 120 (ตามระยะที่ดีไซน์ไว้เป๊ะ) แล้วพบว่า "ยิ่งเว้นมาก ยิ่งเอื้อมไม่ถึง
#   ที่ตำแหน่งสูง/ไกลสุดขอบ" (12 นาฬิกา z สูง, 9 นาฬิกา r ไกล) เพราะการหักจาก r
#   ตรงๆ บังคับให้ J2 (ไหล่) ต้องเอนมากขึ้นเพื่อรักษาความสูง z เท่าเดิม จนเกิน
#   ขีดจำกัดจริง [70,170] — วัดเทียบแล้วที่ 60mm ทั้ง 9 และ 12 นาฬิกายังมี pitch
#   ให้เลือกอยู่ (2026-09) จึงลดเหลือ 60 — แลกกับระยะโฟกัสกล้องที่สั้นลงกว่าที่
#   ตั้งใจไว้ทีแรก ยังไม่ได้ยืนยันด้วยภาพจริงว่ากล้องยังโฟกัสจุ๊บชัดที่ระยะนี้
STANDOFF_MM = 60.0


@dataclass
class CoarseResult:
    ok: bool
    r: float = 0.0
    theta_deg: float = 0.0
    z: float = 0.0
    pitch_deg: float = 0.0
    reason: str = "ok"   # "ok" | "ไม่เจอวาล์ว" | "เอื้อมไม่ถึง" | "กล้องมีปัญหา"


def _find_hub(frame: np.ndarray) -> tuple[float, float] | None:
    """หาศูนย์กลางดุมล้อในภาพ — วงกลมมืดที่กลมที่สุดในเฟรม คืน (x, y) หรือ None

    วิธีเดียวกับที่ใช้ตรวจสอบด้วยมือมาตลอด Task 5B/8: threshold หาบริเวณมืด
    แล้วเลือกรูปทรงกลมที่สุด (ไม่ใช่ใหญ่ที่สุด — เคยเลือกผิดจากของในฉาก
    ที่ไม่ใช่ล้อเลยแต่รัศมีใหญ่กว่าดุมจริง เช่นเงา/ของวางพื้น 2026-09)
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
        # ★ เกณฑ์ 0.45 (ลดจาก 0.55 เดิม) กันไว้กรณีดุมล้อโดนขอบเฟรมตัด
        #   (2026-09, ตำแหน่ง 8 นาฬิกา ท่า B — รูกลางดุมที่ถูกต้องได้แค่ 0.51
        #   เพราะโดนตัดขอบ) แต่ตอนนี้ **เลือกตัวที่กลมที่สุด ไม่ใช่ใหญ่ที่สุด**
        #   ถึงจะปลอดภัย — ถ้ายังเลือกจากรัศมีเหมือนเดิม ตัวปลอมที่มีรัศมีใหญ่กว่า
        #   (แต่ผ่านเกณฑ์ 0.45 พอดี) จะชนะดุมจริงที่กลมกว่ามากแต่รัศมีเล็กกว่า
        if circularity < 0.45:
            continue
        if best is None or circularity > best[2]:
            best = (cx, cy, circularity)

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
    # ⚠️ กับดักทิศ (แบบเดียวกับที่ geometry.valve_pose เตือนไว้) — เคยลองกลับเครื่องหมาย
    # dx ไปรอบหนึ่ง (2026-08) จากคำบอกเล่าที่ยังไม่ได้ควบคุมตัวแปร (กด Enter เฉยๆ
    # ไม่ได้ป้อนตำแหน่งจริง) พอทดสอบใหม่แบบมี debug print เทียบกับตำแหน่งที่ตั้งจริง
    # (ตั้ง 7 นาฬิกา วัดได้ hub=601,82 valve=506,332) สูตรเดิม (ไม่กลับ dx) ให้ clock≈6.69
    # ใกล้ 7 กว่ามาก ส่วนสูตรที่กลับ dx ให้ 5.31 ผิดกว่าเดิม จึงกลับมาใช้สูตรเดิม
    phi = math.atan2(dx, -dy)   # 0° = ขึ้นบน, +90° = ขวาในภาพ, 180°/-180° = ลงล่าง
    clock = (math.degrees(phi) / 30.0 + 12.0) % 12.0
    clock = 12.0 if clock == 0.0 else clock

    print(f"[coarse] hub={hub_xy[0]:.0f},{hub_xy[1]:.0f} valve={valve_xy[0]:.0f},{valve_xy[1]:.0f} "
          f"dx={dx:.0f} dy={dy:.0f} → clock≈{clock:.2f}")

    return clock


def _detect_once(cam: BaseCamera, session, pose_tag: str = "") -> tuple[float, float] | None:
    """ถ่าย 1 เฟรม หาทั้งกล่องจุ๊บและดุมล้อ คืนตำแหน่งจุ๊บเทียบดุมเป็นมุมนาฬิกา หรือ None

    pose_tag : ใส่ "A"/"B" เพื่อแยกไฟล์ debug ของแต่ละท่าสแกน ไม่งั้นไฟล์จะทับกัน
               เห็นแค่ท่าล่าสุด (debug ชั่วคราว)
    """
    sess, input_name, output_name = session

    frame = cam.grab()
    if frame is None:
        return None

    from valve_detector import postprocess, preprocess

    h, w = frame.shape[:2]
    blob, scale, pad_left, pad_top = preprocess(frame)
    dets = postprocess(sess.run([output_name], {input_name: blob})[0], w, h, scale, pad_left, pad_top)
    if not dets:
        path = f"/tmp/coarse_debug_noval_{pose_tag}.jpg"
        cv2.imwrite(path, frame)   # ★ debug ชั่วคราว ดูว่าจุ๊บอยู่ในเฟรมไหม
        print(f"[coarse] เฟรมนี้ไม่เจอกล่องจุ๊บ (โมเดลตรวจไม่เจอ) — เก็บภาพไว้ที่ {path}")
        return None

    x1, y1, x2, y2, _, _ = max(dets, key=lambda d: d[4])
    valve_xy = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    hub_xy = _find_hub(frame)
    if hub_xy is None:
        path = f"/tmp/coarse_debug_nohub_{pose_tag}.jpg"
        cv2.imwrite(path, frame)   # ★ debug ชั่วคราว ดูว่าดุมล้อหน้าตาเป็นยังไงตอนหาไม่เจอ
        print(f"[coarse] เจอจุ๊บที่ {valve_xy} แต่หาดุมล้อไม่เจอ — เก็บภาพไว้ที่ {path}")
        return None

    return _pixel_angle_to_clock(hub_xy, valve_xy)


CLOCK_STABLE_TOL = 0.5   # ชม. — ค่าที่นับว่า "นิ่ง" ต้องห่างจากเฟรมก่อนหน้าไม่เกินนี้


def _scan_from_pose(cam: BaseCamera, session, arm: Arm, upper: bool, confirm_frames: int) -> float | None:
    """ไปท่าสแกน (A หรือ B) แล้วลองหาจุ๊บจนกว่าจะเจอ "นิ่ง" ติดกัน confirm_frames เฟรม

    ★ ต้องเช็คว่าค่านิ่งด้วย ไม่ใช่แค่เจอ (ไม่ None) ติดกัน — เจอจริงจากการทดสอบ
    (2026-09) ว่าดุมล้อที่ตรวจได้สลับไปเจอวัตถุคนละชิ้นกลางทาง (เช่น ขอบเฟรม)
    แต่ยัง "เจอ" ครบ 3 เฟรมติดกันพอดี ระบบเลยยืนยันค่าที่ผิดไปโดยไม่รู้ตัว
    """
    arm.go_scan_pose(upper=upper)
    pose_tag = "B" if upper else "A"

    streak = 0
    last_clock = None
    for _ in range(MAX_ATTEMPTS_PER_POSE):
        clock = _detect_once(cam, session, pose_tag)
        if clock is not None and (last_clock is None or _clock_diff(clock, last_clock) <= CLOCK_STABLE_TOL):
            streak += 1
            last_clock = clock
            if streak >= confirm_frames:
                return last_clock
        else:
            streak = 1 if clock is not None else 0
            last_clock = clock
    return None


def _clock_diff(a: float, b: float) -> float:
    """ระยะห่างระหว่าง 2 ตำแหน่งนาฬิกา แบบวนรอบ (11.9 กับ 0.1 ห่างกันแค่ 0.2 ไม่ใช่ 11.8)"""
    d = abs(a - b) % 12.0
    return min(d, 12.0 - d)


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

    r_touch, theta_deg, z = valve_pose(clock)
    r = max(0.0, r_touch - STANDOFF_MM)
    pitch_deg = arm.best_pitch(r, theta_deg, z)
    if pitch_deg is None:
        return CoarseResult(ok=False, reason="เอื้อมไม่ถึง")

    if not arm.move_to(r, theta_deg, z, pitch_deg):
        return CoarseResult(ok=False, reason="เอื้อมไม่ถึง")

    return CoarseResult(ok=True, r=r, theta_deg=theta_deg, z=z, pitch_deg=pitch_deg, reason="ok")
