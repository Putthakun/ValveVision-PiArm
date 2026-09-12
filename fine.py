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
import time
from dataclasses import dataclass

import cv2
import numpy as np

from arm import Arm
from camera import BaseCamera

MAX_STEPS_DEFAULT = 15   # เพิ่มจาก 8 เพราะจำกัดขนาดก้าวแล้ว (MAX_STEP_*) ต้องใช้หลายรอบกว่าเดิม
PX_THRESH_DEFAULT = 12.0
GAIN_DEFAULT = 0.5

# ★ ต้องรอให้แขนนิ่งก่อนถ่าย — เจอจริงจากการทดสอบ (2026-09) ว่าเฟรมแรกหลัง
#   coarse_locate() ขยับเสร็จยังสั่นอยู่ ทำให้หาปลาย gripper ไม่เจอทั้งที่จริงๆ
#   เห็นอยู่ แค่เบลอ — ใช้ค่าเดียวกับ SETTLE_SEC ใน measure_pixel_scale.py
SETTLE_SEC = 1.2

# ★ ลองซ้ำกี่เฟรมก่อนจะสรุปว่า "มองไม่เห็นจริง" — ตรวจจับพลาดเป็นครั้งคราวได้
#   (โมเดล/heuristic ไม่ได้แม่น 100%) เหมือน coarse.py ที่ต้องลองหลายเฟรม
RETRIES_PER_STEP = 3

# ★ ค่าสำรองตำแหน่งปลาย gripper — ใช้เฉพาะตอนหาสดไม่เจอจริงๆ (เช่นเงายาง
#   ทับติดกับก้ามคีบเป็นก้อนเดียว แยกไม่ออก) เป็นค่าเฉลี่ยจากภาพที่เคยตรวจจับ
#   สำเร็จจริงหลายสิบภาพตอน Task 9 (ส่วนใหญ่อยู่ราว x=393-570, y=504-537)
#   ไม่แม่นเท่าตรวจจับสด แต่ดีกว่ายกเลิกทั้งรอบเฉยๆ
FALLBACK_GRIPPER_TIP_XY = (480.0, 520.0)

# ★ จำกัดขนาดก้าวต่อรอบ — เจอจริง (2026-09) ว่า error เริ่มต้นใหญ่เสมอ (~400px
#   เพราะปลาย gripper อยู่ล่างเฟรมแต่จุ๊บอยู่บนเฟรมโดยธรรมชาติ) ก้าวเดียว
#   ครึ่งหนึ่งของนั้น (gain 0.5) = ขยับ z ถึง 90mm ทีเดียว หลุดเฟรมทันทีที่
#   pitch ชัน ไม่ว่า scale จะถูกหรือผิด — ก้าวเล็กหลายรอบปลอดภัยกว่า และทำให้
#   เห็นชัดจาก log ว่า error ลดหรือเพิ่ม (ทิศถูกไหม) โดยไม่เสียวาล์วไปจากเฟรม
#   ★ นี่คือจำกัด "คำสั่งที่ขอ" ก่อนส่ง IK ไม่ใช่ clamp ผลของ IK — IK ยัง
#   ปฏิเสธเป้าที่เอื้อมไม่ถึงตรงๆ เหมือนเดิม (กฎข้อ 2 ไม่กระทบ)
MAX_STEP_THETA_DEG = 3.0
MAX_STEP_Z_MM = 15.0

# ★ "เงยหาขึ้นบน" เมื่อมองไม่เห็นจุ๊บ — เจอจริง (2026-09-12, 9 นาฬิกา r=371 ใกล้
#   ระยะเอื้อมสุด): J2 รับน้ำหนักปลายแขนไม่ไหว แขนตกลง กล้องเลยมองต่ำกว่าจุ๊บ
#   ทั้งที่ท่าเดียวกันตอนมีคนช่วยยกเห็นจุ๊บห่างเป้าแค่ 57px — สาเหตุเดียวที่ทำให้
#   จุ๊บหายไป "ด้านบน" อย่างเป็นระบบคือแขนตก จึงลองขยับขึ้นทีละก้าว (ไม่เกิน
#   MAX_STEP_Z_MM) จนกว่าจะเห็น หยุดทันทีที่เห็น = closed-loop ผ่านกล้อง ปรับตาม
#   แขนตกจริงในแต่ละท่า ไม่ใช่ค่าชดเชยตายตัว (กฎข้อ 1) ถ้าขยับขึ้นครบแล้วยัง
#   ไม่เห็นค่อยรายงาน "มองไม่เห็นวาล์ว" ตามเดิม
SEARCH_UP_MAX_STEPS = 4
RETRY_DELAY_SEC = 0.4   # เว้นช่วงระหว่างเฟรม retry ให้แขนหยุดส่าย (ภาพเบลอตอนยังขยับ)


def _clamp_step(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


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


def _aim_point(frame: np.ndarray, scale: dict) -> tuple[float, float]:
    """จุดเป้าในภาพที่จุ๊บต้องวิ่งไปทับ

    aim_from="calibrated"      → ใช้ aim_x/aim_y คงที่จาก pixel_scale.json
                                 (จุดที่จุ๊บอยู่ตอนปลายก้ามชี้ตรงจุ๊บจริง วัดด้วย
                                 tools/hold_at_valve.py) ★ ใช้โหมดนี้ตั้งแต่ 2026-09-12
                                 เพราะกล้องอยู่เหนือก้าม เกิด parallax: ปลายก้ามที่
                                 ยังห่างจุ๊บ 2-3 ซม. ดูต่ำกว่าจุ๊บในภาพ ~300px ทั้งที่
                                 ชี้ตรงกันแล้ว ปลายก้ามในภาพจึงไม่ใช่เป้าที่ถูก
    aim_from="gripper_visible" → หาปลายก้ามสด ไม่เจอใช้ FALLBACK_GRIPPER_TIP_XY
    """
    if scale.get("aim_from") == "gripper_visible":
        return _find_gripper_tip(frame) or FALLBACK_GRIPPER_TIP_XY
    return (float(scale["aim_x"]), float(scale["aim_y"]))


def _detect_valve_px(cam: BaseCamera, session) -> tuple[float, float] | None:
    """ถ่าย 1 เฟรม คืนตำแหน่งกึ่งกลางกล่องจุ๊บที่มั่นใจที่สุด หรือ None"""
    frame = cam.grab()
    if frame is None:
        return None
    return _valve_px_in_frame(frame, session)


def _valve_px_in_frame(frame: np.ndarray, session) -> tuple[float, float] | None:
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
               debug: bool = False,
               settle_sec: float = SETTLE_SEC,
               correct_x: bool = True,
               correct_y: bool = True) -> FineResult:
    """ไล่ nudge แขนทีละนิดจนจุ๊บในภาพมาอยู่ตำแหน่งเดียวกับปลาย gripper

    correct_x/correct_y : ปิดแกนไหนได้ถ้าไม่อยากแก้ (เช่นโหมดช่วยแมนวลที่คน
    จะดันความลึก/แนวตั้งเองอยู่แล้ว อยากให้ระบบแก้แค่ซ้าย-ขวาที่ปลอดภัยกว่า
    ไม่ต้องเสี่ยงชนขีดจำกัดจากการแก้แกนที่ไม่ได้ขอ) — ปิดแกนไหน แกนนั้นไม่นับ
    ตอนเช็คว่าเข้าเป้าด้วย
    """
    last_err = 0.0

    retry_delay = RETRY_DELAY_SEC if settle_sec > 0 else 0.0   # เทสปิดการหน่วงเวลาได้

    def _look():
        """ถ่ายจนกว่าจะเห็นจุ๊บ (ไม่เกิน RETRIES_PER_STEP เฟรม) คืน (frame, valve_xy, target_xy)"""
        frame = valve_xy = target_xy = None
        for attempt in range(RETRIES_PER_STEP):
            if attempt:
                time.sleep(retry_delay)
            frame = cam.grab()
            if frame is None:
                continue
            valve_xy = _valve_px_in_frame(frame, session)
            if valve_xy is not None:
                target_xy = _aim_point(frame, scale)   # ไม่มีทาง None (มีค่าสำรองในตัว)
                break
        return frame, valve_xy, target_xy

    for step in range(max_steps):
        time.sleep(settle_sec)   # รอแขนนิ่งก่อนถ่าย (เพิ่งขยับมาจาก coarse หรือ nudge รอบก่อน)
        frame, valve_xy, target_xy = _look()

        # ★ มองไม่เห็น → เงยหาขึ้นบนทีละก้าวก่อนยอมแพ้ (ดู SEARCH_UP_MAX_STEPS)
        for k in range(SEARCH_UP_MAX_STEPS):
            if valve_xy is not None:
                break
            if not arm.nudge(0.0, MAX_STEP_Z_MM):
                break   # ขึ้นต่อไม่ได้แล้ว (ชนขีดจำกัด) ไม่ฝืน
            if debug:
                print(f"  รอบ {step + 1}: มองไม่เห็นจุ๊บ → เงยหาขึ้น +{MAX_STEP_Z_MM:.0f}mm ({k + 1}/{SEARCH_UP_MAX_STEPS})")
            time.sleep(settle_sec)
            frame, valve_xy, target_xy = _look()

        if valve_xy is None:
            if debug and frame is not None:
                cv2.imwrite(f"/tmp/fine_debug_noval_{step + 1}.jpg", frame)
                print(f"  รอบ {step + 1}: ไม่เจอกล่องจุ๊บ (ลองแล้ว {RETRIES_PER_STEP} เฟรม + เงยหา) — "
                      f"เก็บภาพไว้ที่ /tmp/fine_debug_noval_{step + 1}.jpg")
            return FineResult(False, step, last_err, "มองไม่เห็นวาล์ว")

        err_x = valve_xy[0] - target_xy[0]
        err_y = valve_xy[1] - target_xy[1]
        # ★ ปิดแกนไหน ไม่นับ error ของแกนนั้นเลย ทั้งตอนเช็คเข้าเป้าและตอนคำนวณ nudge
        check_x, check_y = (err_x if correct_x else 0.0), (err_y if correct_y else 0.0)
        last_err = math.hypot(check_x, check_y)

        if debug:
            print(f"  รอบ {step + 1}: {last_err:.0f}px  (err_x={err_x:+.0f} err_y={err_y:+.0f})")
            if frame is not None:
                vis = frame.copy()
                cv2.circle(vis, (int(valve_xy[0]), int(valve_xy[1])), 10, (0, 255, 0), 2)
                cv2.circle(vis, (int(target_xy[0]), int(target_xy[1])), 10, (0, 0, 255), 2)
                cv2.imwrite(f"/tmp/fine_debug_step{step + 1}.jpg", vis)

        if last_err < px_thresh:
            return FineResult(True, step, last_err, "เข้าเป้า")

        # ★ ต้องมีเครื่องหมายลบ — scale ที่วัดไว้คือ "ขยับแขน +d → ภาพเลื่อน +px"
        #   (measure_pixel_scale.py เก็บ d/px ดิบๆ) ถ้าใช้ d = err×scale ตรงๆ
        #   ตามที่ PLAN.md เขียน จุ๊บจะเลื่อนไปอีก +err = ห่างเป้าเป็น 2 เท่า
        #   ยืนยันจากการทดสอบจริง (2026-09-12, 5.7 นาฬิกา ก้าวเล็ก): err_x
        #   -179 → -306 หลัง nudge ตามสูตรเดิม กลับทิศแล้วทุกค่าที่วัดได้ดิบๆ
        #   ทุก pitch สอดคล้องกันหมด ไม่ต้องกลับเครื่องหมายในไฟล์ด้วยมือ
        d_theta = _clamp_step(-check_x * scale["deg_per_px_x"] * gain, MAX_STEP_THETA_DEG)
        d_z = _clamp_step(-check_y * scale["mm_per_px_y"] * gain, MAX_STEP_Z_MM)
        if debug:
            print(f"         → nudge theta {d_theta:+.1f}° z {d_z:+.1f}mm")
        if not arm.nudge(d_theta, d_z):
            return FineResult(False, step, last_err, "แขนขยับต่อไม่ได้")

    return FineResult(False, max_steps, last_err, "ครบรอบสูงสุด")


def _load_scale_for_pitch(pitch_deg: float, path: str = "pixel_scale.json") -> dict:
    """โหลด pixel_scale.json แล้วเลือก entry ที่วัดไว้ที่ pitch ใกล้เคียงที่สุด

    ★ ค่าที่วัดไว้ (deg_per_px_x/mm_per_px_y) ใช้ได้เฉพาะ pitch ที่วัดตอนนั้น
    เท่านั้น (มุมกล้องเปลี่ยนตามความเอียงของมือ) — เจอจริงจากการทดสอบ (2026-09)
    ว่าเอาค่าที่วัดตอน pitch=0 ไปใช้ตอน pitch=45 แล้วเฟสละเอียดวิ่งผิดทิศ
    ไฟล์จึงเก็บหลาย pitch พร้อมกัน (ดู tools/measure_pixel_scale.py) เลือกอัน
    ใกล้สุดตอนใช้งานจริง
    """
    import json

    with open(path, encoding="utf-8") as f:
        data = json.load(f)

    entries = data.get("entries")
    if not entries:
        raise ValueError(
            f"{path} ยังเป็นฟอร์แมตเก่า (pitch เดียว) — รัน "
            f"tools/measure_pixel_scale.py ใหม่ (ใส่ --pitch ให้ตรงกับที่ "
            f"coarse_locate() เลือกจริง) ก่อนใช้ fine.py"
        )

    best_key = min(entries, key=lambda k: abs(float(k) - pitch_deg))
    best = entries[best_key]
    if abs(float(best_key) - pitch_deg) > 15.0:
        print(f"[fine] ⚠ ไม่มี pixel_scale ที่วัดไว้ใกล้ pitch={pitch_deg:+.0f}° เลย "
              f"(ใกล้สุดคือ {best_key}°, ห่าง {abs(float(best_key)-pitch_deg):.0f}°) "
              f"ผลอาจไม่แม่น — ควรวัดเพิ่มด้วย tools/measure_pixel_scale.py --pitch {pitch_deg:.0f}")
    return best


def _main():
    import argparse

    from camera import WristCamera
    from coarse import coarse_locate
    from valve_detector import load_model

    ap = argparse.ArgumentParser()
    ap.add_argument("--debug", action="store_true", help="พิมพ์ความคลาดเคลื่อนพิกเซลทุกรอบ")
    args = ap.parse_args()

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
        scale = _load_scale_for_pitch(coarse.pitch_deg)
        print(f"ใช้ pixel_scale ที่วัดไว้ที่ pitch={scale['measured_at_pitch']:+.0f}°")
        print("เฟสละเอียด: ไล่ตำแหน่ง...")
        res = fine_align(cam, session, arm, scale, debug=args.debug)
        print(f"ผล: converged={res.converged} steps={res.steps} "
              f"final_px_err={res.final_px_err:.1f} reason={res.reason}")
    finally:
        cam.close()
        arm.go_scan_pose()


if __name__ == "__main__":
    _main()
