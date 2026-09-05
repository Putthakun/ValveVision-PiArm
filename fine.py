# fine.py — เฟสละเอียด: ไล่ตำแหน่งจุ๊บให้ตรงปลาย gripper ทีละก้าวเล็กๆ (Task 10)
#
# หน้าที่เดียว: ให้จุ๊บในภาพเลื่อนมาอยู่ตำแหน่งเดียวกับปลาย gripper
#
# ★ ห้ามแปลงเป็นมิลลิเมตรเด็ดขาด (กฎข้อ 3 ของ CLAUDE.md) — คิดเป็นพิกเซลล้วนๆ
#   ตัวคูณ deg_per_px_x / mm_per_px_y ใน pixel_scale.json (Task 8) เป็นค่าที่วัด
#   จากของจริงมาแล้วว่า "ขยับแขนเท่านี้ → ภาพเลื่อนกี่พิกเซล" ลูปนี้แค่คูณกลับ
#   ไม่ได้แปลงพิกเซลเป็นระยะจริงเอง
#
# ★ ห้ามแตะ pitch ในลูปนี้ — arm.nudge() ไม่รับพารามิเตอร์ pitch อยู่แล้ว
#   (บังคับด้วยโครงสร้าง ไม่ใช่แค่ระเบียบ)
#
# ★ เป้า (เป้า_x, เป้า_y) คือตำแหน่งปลาย gripper ในภาพ ไม่ใช่จุดคงที่ — ตรวจจับสด
#   ทุกเฟรมเหมือน coarse.py ตรวจดุมล้อสด (เหตุผลเดียวกัน: กล้องขยับได้เล็กน้อย
#   ทุกครั้งที่ถอด-ประกอบ ค่าคงที่จะผิดโดยไม่รู้ตัว) ใช้ค่าคงที่ aim_x/aim_y ใน
#   pixel_scale.json เป็นทางเลือกสำรองเฉพาะกรณีมองไม่เห็นปลาย gripper (Task 4
#   ยืนยันแล้วว่าโปรเจ็คนี้เห็น จึงไม่ใช้ทางสำรองนี้ในทางปฏิบัติ)

import math
from dataclasses import dataclass

import cv2
import numpy as np

from arm import Arm
from camera import BaseCamera

MAX_STEPS_DEFAULT = 8
PX_THRESH_DEFAULT = 12.0
GAIN_DEFAULT = 0.5


@dataclass
class FineResult:
    converged: bool
    steps: int = 0
    final_px_err: float = 0.0
    reason: str = "เข้าเป้า"   # "เข้าเป้า" | "ครบรอบสูงสุด" | "มองไม่เห็นวาล์ว" | "แขนขยับต่อไม่ได้"


def _find_gripper_tip(frame: np.ndarray) -> tuple[float, float] | None:
    """หาปลาย gripper ในภาพ — ก้ามคีบสีเข้ม 2 อันติดอยู่หน้ากล้องเสมอ (rigid)

    ลักษณะที่ใช้แยกจากของอื่นในภาพ (เช่น รูล้อ/สติ๊กเกอร์ที่มืดเหมือนกัน):
    ก้ามคีบอยู่ค่อนไปทางล่างของเฟรมเสมอ ทรงสูงกว่ากว้าง และไม่ใหญ่ไม่เล็กเกินไป
    ค่าที่ใช้ตั้งจากการวัดจริงกับภาพที่เก็บไว้ตอน Task 9 (ก้ามคีบกว้าง ~80-220px
    สูง ~150-260px อัตราส่วนสูงต่อกว้าง ~1.1-2.3 เท่า)
    """
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, th = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for c in contours:
        x, y, cw, ch = cv2.boundingRect(c)
        cx = x + cw / 2.0
        if not (80 < cw < 220 and 150 < ch < 260):
            continue
        if not (1.1 < ch / cw < 2.3):
            continue
        if y + ch < h * 0.65:          # ต้องอยู่ค่อนไปทางล่างของเฟรม
            continue
        if not (0.3 * w < cx < 0.65 * w):
            continue
        area = cv2.contourArea(c)
        if best is None or area > best[1]:
            best = ((x, y, cw, ch), area)

    if best is None:
        return None
    (x, y, cw, ch), _ = best
    return (x + cw / 2.0, float(y))     # จุดบนสุดของก้ามคีบ = จุดที่วัตถุจะถูกคีบ


def _detect_valve_px(cam: BaseCamera, session) -> tuple[float, float] | None:
    """ถ่าย 1 เฟรม คืนตำแหน่งกึ่งกลางกล่องจุ๊บที่มั่นใจที่สุด หรือ None"""
    frame = cam.grab()
    if frame is None:
        return None

    from valve_detector import postprocess, preprocess

    sess, input_name, output_name = session
    h, w = frame.shape[:2]
    blob, scale, pad_left, pad_top = preprocess(frame)
    dets = postprocess(sess.run([output_name], {input_name: blob})[0], w, h, scale, pad_left, pad_top)
    if not dets:
        return None

    x1, y1, x2, y2, _, _ = max(dets, key=lambda d: d[4])
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


def fine_align(cam: BaseCamera, session, arm: Arm, scale: dict, *,
               max_steps: int = MAX_STEPS_DEFAULT,
               px_thresh: float = PX_THRESH_DEFAULT,
               gain: float = GAIN_DEFAULT,
               debug: bool = False) -> FineResult:
    """ไล่ nudge แขนทีละนิดจนจุ๊บในภาพมาอยู่ตำแหน่งเดียวกับปลาย gripper"""
    last_err = 0.0

    for step in range(max_steps):
        valve_xy = _detect_valve_px(cam, session)
        if valve_xy is None:
            return FineResult(False, step, last_err, "มองไม่เห็นวาล์ว")

        frame = cam.grab() if scale.get("aim_from") == "gripper_visible" or debug else None
        if scale.get("aim_from") == "gripper_visible":
            target_xy = _find_gripper_tip(frame) if frame is not None else None
            if target_xy is None:
                return FineResult(False, step, last_err, "มองไม่เห็นวาล์ว")
        else:
            target_xy = (scale["aim_x"], scale["aim_y"])

        err_x = valve_xy[0] - target_xy[0]
        err_y = valve_xy[1] - target_xy[1]
        last_err = math.hypot(err_x, err_y)

        if debug:
            print(f"  รอบ {step + 1}: {last_err:.0f}px  (err_x={err_x:+.0f} err_y={err_y:+.0f})")
            if frame is not None:
                vis = frame.copy()
                cv2.circle(vis, (int(valve_xy[0]), int(valve_xy[1])), 10, (0, 255, 0), 2)
                cv2.circle(vis, (int(target_xy[0]), int(target_xy[1])), 10, (0, 0, 255), 2)
                cv2.imwrite(f"/tmp/fine_debug_step{step + 1}.jpg", vis)

        if last_err < px_thresh:
            return FineResult(True, step, last_err, "เข้าเป้า")

        d_theta = err_x * scale["deg_per_px_x"] * gain
        d_z = err_y * scale["mm_per_px_y"] * gain
        if not arm.nudge(d_theta, d_z):
            return FineResult(False, step, last_err, "แขนขยับต่อไม่ได้")

    return FineResult(False, max_steps, last_err, "ครบรอบสูงสุด")


def _main():
    import argparse
    import json

    from camera import WristCamera
    from coarse import coarse_locate
    from valve_detector import load_model

    ap = argparse.ArgumentParser()
    ap.add_argument("--debug", action="store_true", help="พิมพ์ความคลาดเคลื่อนพิกเซลทุกรอบ")
    args = ap.parse_args()

    with open("pixel_scale.json", encoding="utf-8") as f:
        scale = json.load(f)

    arm = Arm()
    cam = WristCamera()
    session = load_model()
    try:
        print("เฟสหยาบ: หาวาล์ว...")
        coarse = coarse_locate(cam, session, arm)
        if not coarse.ok:
            print(f"เฟสหยาบไม่สำเร็จ — reason={coarse.reason}")
            return

        print(f"เฟสหยาบ ok — r={coarse.r:.0f} theta={coarse.theta_deg:.0f}° "
              f"z={coarse.z:.0f} pitch={coarse.pitch_deg:+.0f}°")
        print("เฟสละเอียด: ไล่ตำแหน่ง...")
        res = fine_align(cam, session, arm, scale, debug=args.debug)
        print(f"ผล: converged={res.converged} steps={res.steps} "
              f"final_px_err={res.final_px_err:.1f} reason={res.reason}")
    finally:
        cam.close()
        arm.go_scan_pose()


if __name__ == "__main__":
    _main()
