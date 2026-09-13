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
from coarse import CONFIRM_FRAMES_DEFAULT, _scan_from_pose, coarse_locate
from fine import _load_scale_for_pitch, fine_align
from geometry import valve_pose

TOUCH_PUSH_MM = 5.0        # ★ ตามกฎ DESIGN.md หัวข้อ 7 — ห้ามเกินนี้
TOUCH_SETTLE_SEC = 0.6     # รอให้ถึงจริงก่อนถอย (แต่ไม่ค้างนาน — กฎข้อ 5)

# ── โหมดวนยื่นหาจุ๊บ (ค่าเริ่มต้นของ `python run.py`) ──────────────────────
# แผน end-to-end ของเจ้าของโปรเจ็ค (2026-09-13): เปิด Pi → หาจุ๊บ → ยื่นไปใกล้ที่สุด
# ทุกตำแหน่งนาฬิกา → ค้าง → กลับท่าสแกน วนไปเรื่อยๆ ยังไม่สนความแม่น
# ข้ามเฟสละเอียด เพราะแขนตอนยื่นไกลยังโยก/ตก (ฐาน + J2) ค่า pixel_scale จึงวัดไม่นิ่ง
HOVER_STANDOFF_MM = 10.0   # สั่งหยุดก่อนถึงจุ๊บ — ค้างท่า 5 วิ ห้ามดันจุ๊บค้างไว้ (กฎข้อ 5)
HOVER_HOLD_SEC = 5.0
APPROACH_BELOW_MM = 20.0   # ลงต่ำกว่าเป้าก่อนแล้วยกขึ้นมา — วัดแล้วเป็นทิศที่แขนหยุดนิ่งที่สุด
# ★ J1 มีระยะคลอน แขนค้างเอียงขวาแบบสุ่ม — ทดสอบด้วยตา (2026-09-13 ที่ 12 นาฬิกา):
#   หมุนไปทางซ้าย (theta ลด) ก่อนแล้วหมุนกลับเข้าเป้า = ปลายตรงจุ๊บ · เข้าจากทางขวา = เยื้อง
#   เข้าจากทิศเดียวกันทุกครั้ง ฟันเฟืองจึงแนบด้านเดิมเสมอ (ไม่ใช่ค่าชดเชยตำแหน่ง)
#   ช่วงหมุนกลับเข้าเป้าต้องค่อยๆ ทีละก้าว — หมุนรวดเดียวแขนสะบัดเลยไปทางขวาจนเยื้องอีก
J1_APPROACH_FROM_LEFT_DEG = 4.0
J1_CREEP_STEP_DEG = 0.5
J1_CREEP_PAUSE_SEC = 0.08
LOOP_PAUSE_SEC = 2.0


def hover_once(cam: BaseCamera, session, arm: Arm) -> str:
    """หาจุ๊บ → ยื่นไปจุดที่ใกล้ที่สุดที่เอื้อมถึง → ค้าง → กลับท่าสแกนเสมอ คืนข้อความสรุปรอบ"""
    try:
        clock = _scan_from_pose(cam, session, arm, upper=False, confirm_frames=CONFIRM_FRAMES_DEFAULT)
        if clock is None:
            clock = _scan_from_pose(cam, session, arm, upper=True, confirm_frames=CONFIRM_FRAMES_DEFAULT)
        if clock is None:
            return "ไม่เจอจุ๊บ"

        r, theta, z = valve_pose(clock)
        target = arm.nearest_reachable(r - HOVER_STANDOFF_MM, theta, z)
        if target is None:
            return f"จุ๊บที่ {clock:.1f} นาฬิกา — เอื้อมไม่ถึงเลย แม้หดเข้ามา 70%"
        rr, tt, zz, pitch, short = target

        # ท่าเตรียม ไปไม่ได้ก็ไม่เป็นไร แค่ไม่ได้เข้าจากทิศที่นิ่งที่สุด
        arm.move_to(rr, tt - J1_APPROACH_FROM_LEFT_DEG, zz - APPROACH_BELOW_MM, pitch)
        time.sleep(0.8)
        steps = round(J1_APPROACH_FROM_LEFT_DEG / J1_CREEP_STEP_DEG)
        for i in range(1, steps + 1):
            arm.move_to(rr, tt - J1_APPROACH_FROM_LEFT_DEG + i * J1_CREEP_STEP_DEG, zz - APPROACH_BELOW_MM, pitch)
            time.sleep(J1_CREEP_PAUSE_SEC)
        time.sleep(0.5)
        if not arm.move_to(rr, tt, zz, pitch):
            return f"จุ๊บที่ {clock:.1f} นาฬิกา — สั่งไปจุดที่คำนวณไว้ไม่สำเร็จ"
        time.sleep(HOVER_HOLD_SEC)

        reach = "ถึงตามเรขาคณิต" if short == 0 else f"ขาด {short:.0f}mm (ยื่นได้แค่นี้)"
        return f"จุ๊บที่ {clock:.1f} นาฬิกา — ยื่นไป r={rr:.0f} z={zz:.0f} pitch={pitch:+.0f}° {reach}"
    finally:
        arm.go_scan_pose()


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


def main_once(replay_dir: str | None = None) -> None:
    """state machine เต็ม (มีเฟสละเอียด + แตะ) หนึ่งรอบ"""
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


def main() -> None:
    """วนยื่นหาจุ๊บไปเรื่อยๆ จนกด Ctrl+C (หรือ systemd สั่งหยุดด้วย SIGINT)"""
    from camera import WristCamera
    from valve_detector import load_model

    arm = Arm()
    if arm.simulate:
        print("[run] ⚠️ ต่อ servo ไม่ได้ — กำลังรันโหมดจำลอง แขนจะไม่ขยับจริง", flush=True)
    cam = WristCamera()
    session = load_model()
    n = 0
    try:
        while True:
            n += 1
            print(f"[รอบ {n}] {hover_once(cam, session, arm)}", flush=True)
            time.sleep(LOOP_PAUSE_SEC)
    except KeyboardInterrupt:
        print("\n[run] หยุดแล้ว", flush=True)
    finally:
        cam.close()
        arm.go_scan_pose()
        print("[run] แขนกลับท่าสแกนแล้ว", flush=True)


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="ValveVision — วนยื่นแขนหาจุ๊บ")
    ap.add_argument("--once", action="store_true", help="รัน state machine เต็ม (เฟสละเอียด + แตะ) หนึ่งรอบแทน")
    ap.add_argument("--replay", metavar="DIR", help="ใช้กับ --once: อ่านภาพจากโฟลเดอร์แทนกล้อง")
    args = ap.parse_args()
    if args.once:
        main_once(args.replay)
    else:
        main()
