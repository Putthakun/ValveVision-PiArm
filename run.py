# run.py — ร้อยทุกเฟสเข้าด้วยกันเป็น state machine (Task 11)
#
# SCAN     → coarse_locate() หาว่าจุ๊บอยู่โซนไหน แล้วพาแขนเข้าใกล้
# APPROACH → fine_align() ไล่ตำแหน่งด้วยกล้องจนจุ๊บตรงเป้า
# TOUCH    → เดินหน้าอีกนิดเดียวให้แตะ แล้ว **ถอยทันที**
# RECOVER  → retreat() แล้วกลับท่าสแกนเสมอ
#
# ★ ทุกทางที่ผิดพลาดต้องจบที่ "กลับท่าสแกน" ไม่ใช่ crash หรือค้าง (กฎข้อ 5 —
#   ห้ามทิ้งแขนค้างอยู่ในซอกล้อ) รวมถึงตอนโดน Ctrl+C ด้วย
#
# ★ TOUCH เดินหน้าแค่ TOUCH_PUSH_MM — ไม่ใช่ 12 ซม.ตามที่ PLAN.md เขียนไว้เดิม
#   ตัวเลข 12 ซม.นั้นมาจากตอนที่ coarse ยังถอยแขนไว้ 12 ซม. (STANDOFF_MM=120)
#   ซึ่งถอดออกไปแล้วเพราะทำให้หลายตำแหน่งเอื้อมไม่ถึง — ตอนนี้ coarse พาแขนไป
#   ถึงระยะสัมผัสตามเรขาคณิตเลย ถ้าเดินหน้าอีก 12 ซม.จะพุ่งทะลุเข้าล้อ
#   DESIGN.md หัวข้อ 7 เขียนกฎไว้อยู่แล้วว่า "ห้ามสั่งเกินตำแหน่งที่ตรวจจับได้
#   เกินประมาณ 5 มม." (กันเฟืองไหม้) จึงยึดตามนั้น

import time
from dataclasses import dataclass

from arm import Arm
from camera import BaseCamera
from coarse import coarse_locate
from fine import _load_scale_for_pitch, fine_align

TOUCH_PUSH_MM = 5.0        # ★ ตามกฎ DESIGN.md หัวข้อ 7 — ห้ามเกินนี้
TOUCH_SETTLE_SEC = 0.6     # รอให้ถึงจริงก่อนถอย (แต่ไม่ค้างนาน — กฎข้อ 5)


@dataclass
class RunOutcome:
    state: str            # สถานะสุดท้าย — ปกติคือ "RECOVER" เสมอ
    touched: bool         # ได้เดินหน้าไปแตะจริงไหม
    reason: str
    px_err: float = 0.0   # ความคลาดเคลื่อนพิกเซลตอนจบเฟสละเอียด


def run_once(cam: BaseCamera, session, arm: Arm, *,
             settle_sec: float | None = None, debug: bool = False) -> RunOutcome:
    """วิ่งครบหนึ่งรอบ: หา → เข้าใกล้ → ไล่ตำแหน่ง → แตะ → ถอยกลับท่าสแกน

    ★ ไม่ว่าจะจบทางไหน (สำเร็จ ล้มเหลว หรือโดนขัดจังหวะ) แขนต้องกลับท่าสแกน
      เสมอ — จัดการใน finally เพื่อให้ครอบคลุม KeyboardInterrupt ด้วย
    """
    state = "SCAN"
    touched = False
    reason = ""
    px_err = 0.0

    try:
        # ── SCAN ────────────────────────────────────────────────────────
        coarse = coarse_locate(cam, session, arm)
        if not coarse.ok:
            reason = f"เฟสหยาบไม่สำเร็จ: {coarse.reason}"
            return RunOutcome("RECOVER", False, reason)

        # ── APPROACH ────────────────────────────────────────────────────
        state = "APPROACH"
        scale = _load_scale_for_pitch(coarse.pitch_deg)
        kwargs = {"debug": debug}
        if settle_sec is not None:
            kwargs["settle_sec"] = settle_sec
        fine = fine_align(cam, session, arm, scale, **kwargs)
        px_err = fine.final_px_err
        if not fine.converged:
            # ★ ไม่ลู่เข้า = ไม่รู้ว่าจุ๊บอยู่ตรงไหนแน่ ห้ามดันหน้าไปแตะเด็ดขาด
            reason = f"เฟสละเอียดไม่ลู่เข้า: {fine.reason} (เหลือ {px_err:.0f}px)"
            return RunOutcome("RECOVER", False, reason, px_err)

        # ── TOUCH ───────────────────────────────────────────────────────
        state = "TOUCH"
        r, theta, z, pitch = arm.current()
        if arm.move_to(r + TOUCH_PUSH_MM, theta, z, pitch):
            touched = True
            reason = f"แตะแล้ว (ดันหน้า {TOUCH_PUSH_MM:.0f}mm, คลาดเคลื่อน {px_err:.0f}px)"
            time.sleep(TOUCH_SETTLE_SEC)
        else:
            reason = f"เอื้อมไปแตะไม่ถึง (ขาดอีก {TOUCH_PUSH_MM:.0f}mm, คลาดเคลื่อน {px_err:.0f}px)"

        return RunOutcome("RECOVER", touched, reason, px_err)

    finally:
        # ── RECOVER — ต้องทำเสมอ ไม่ว่าจะจบทางไหน รวมถึงตอนโดน Ctrl+C ────
        if touched:
            arm.retreat()          # ★ แตะแล้วถอยทันที ไม่ค้างดัน (กฎข้อ 5)
        arm.go_scan_pose()


def main(replay_dir: str | None = None) -> None:
    from camera import ReplayCamera, WristCamera
    from valve_detector import load_model

    arm = Arm()
    cam = ReplayCamera(replay_dir) if replay_dir else WristCamera()
    session = load_model()
    try:
        outcome = run_once(cam, session, arm, debug=True)
        print(f"\nผล: touched={outcome.touched} err={outcome.px_err:.1f}px — {outcome.reason}")
    finally:
        cam.close()


if __name__ == "__main__":
    import sys

    main(sys.argv[1] if len(sys.argv) > 1 else None)
