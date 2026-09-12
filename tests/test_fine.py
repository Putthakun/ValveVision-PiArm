# tests/test_fine.py
from dataclasses import dataclass

import fine
from arm import Arm
from fine import fine_align
from geometry import valve_pose

SCALE = {
    "deg_per_px_x": -0.046901,
    "mm_per_px_y": -0.15777,
    "aim_from": "gripper_visible",
}


def _ready_arm() -> Arm:
    """Arm(simulate=True) เพิ่งสร้างใหม่ nudge() ไม่ได้ (ท่าเริ่มต้นไม่ใช่ผลจาก IK)
    ต้อง move_to() ไปท่าที่ IK แก้ได้ก่อน — เหมือนสถานการณ์จริงหลัง coarse_locate()
    """
    arm = Arm(simulate=True)
    r, theta, z = valve_pose(9)
    pitch = arm.best_pitch(r, theta, z)
    arm.move_to(r, theta, z, pitch)
    return arm


class FakeCam:
    """กล้องปลอม — grab() คืนภาพอะไรก็ได้ เพราะเทสนี้ปลอมการตรวจจับเอง (monkeypatch)"""

    def grab(self):
        return object()

    def close(self):
        pass


def _patch_detectors(monkeypatch, valve_track, gripper_xy=(640.0, 520.0)):
    """แทนที่ตัวตรวจจับจริงด้วยของปลอม — เทสนี้ต้องการคุมพิกเซลได้เป๊ะ ไม่ใช่ทดสอบ CV จริง

    valve_track : list ของตำแหน่งจุ๊บ (x, y) หรือ None — เรียกทีละตัวตามจำนวนรอบ
    """
    it = iter(valve_track)

    def fake_valve_px_in_frame(frame, session):
        return next(it, None)   # หมดลิสต์แล้วให้ถือว่าไม่เจอ แทนที่จะ StopIteration

    def fake_find_gripper_tip(frame):
        return gripper_xy

    monkeypatch.setattr(fine, "_valve_px_in_frame", fake_valve_px_in_frame)
    monkeypatch.setattr(fine, "_find_gripper_tip", fake_find_gripper_tip)


def test_ลู่เข้าเป้าแล้วต้องหยุด(monkeypatch):
    """จำลองว่าทุกครั้งที่ nudge ความคลาดเคลื่อนลดลงครึ่งหนึ่ง จนต่ำกว่า px_thresh

    ★ ค่าในแทร็กต้องไม่ตกในช่วง deadband (MIN_STEP_*) กลางทาง ไม่งั้นลูปจะหยุด
    ก่อนถึงเป้าอย่างถูกต้อง (ก้าวที่ต้องสั่งเล็กกว่าที่ servo ขยับจริงได้)
    """
    target = (640.0, 520.0)
    track = [(target[0] + 80, target[1]), (target[0] + 40, target[1]),
             (target[0] + 8, target[1])]
    _patch_detectors(monkeypatch, track, gripper_xy=target)

    res = fine_align(FakeCam(), None, _ready_arm(), SCALE, max_steps=8, px_thresh=12.0, settle_sec=0.0)

    assert res.converged is True
    assert res.reason == "เข้าเป้า"
    assert res.steps <= 4


def test_มองไม่เห็นวาล์วต้องหยุดทันที(monkeypatch):
    _patch_detectors(monkeypatch, [None])

    res = fine_align(FakeCam(), None, _ready_arm(), SCALE, max_steps=8, px_thresh=12.0, settle_sec=0.0)

    assert res.converged is False
    assert res.reason == "มองไม่เห็นวาล์ว"
    assert res.steps == 0


def test_ไม่ดีขึ้นต้องหยุด_ไม่วนไม่จบ(monkeypatch):
    """ความคลาดเคลื่อนไม่ลดลงเลย (ค้างที่ 80px) ต้องหยุดเอง ไม่วนจนครบ max_steps

    เจอจริง (2026-09-12): แขนทำซ้ำได้ ~18px พอ error ลงมาใกล้ระดับนั้นแล้ว
    การไล่ต่อมีแต่จะแกว่ง (ไล่ตามสัญญาณรบกวน) จนค่าแย่ลง — ต้องรู้จักหยุด
    """
    target = (640.0, 520.0)
    track = [(target[0] + 80, target[1])] * 20   # เผื่อไว้เกิน max_steps เยอะๆ
    _patch_detectors(monkeypatch, track, gripper_xy=target)

    res = fine_align(FakeCam(), None, _ready_arm(), SCALE, max_steps=20, px_thresh=12.0, settle_sec=0.0)

    assert res.converged is False
    assert res.reason == "นิ่งที่ค่าดีที่สุดแล้ว"
    assert res.steps < 20   # หยุดเองก่อนครบรอบ


def test_แกว่งออกแล้วต้องกลับไปท่าที่ดีที่สุด(monkeypatch):
    """error ดีขึ้นแล้วแย่ลง — ตอนจบแขนต้องกลับไปท่าตอนที่ดีที่สุด ไม่ใช่ท่าล่าสุด"""
    target = (640.0, 520.0)
    # 80 → 30 (ดีสุด) → แล้วแย่ลงเรื่อยๆ จนหยุด
    track = [(target[0] + 80, target[1]), (target[0] + 30, target[1]),
             (target[0] + 50, target[1]), (target[0] + 60, target[1]),
             (target[0] + 70, target[1]), (target[0] + 80, target[1])]
    _patch_detectors(monkeypatch, track, gripper_xy=target)

    arm = _ready_arm()
    res = fine_align(FakeCam(), None, arm, SCALE, max_steps=20, px_thresh=12.0, settle_sec=0.0)

    assert res.reason == "นิ่งที่ค่าดีที่สุดแล้ว"
    assert res.final_px_err == 30.0   # รายงานค่าที่ดีที่สุด ไม่ใช่ค่าล่าสุดที่แย่กว่า


def test_หาปลาย_gripper_สดไม่เจอต้องใช้ค่าสำรองแทน_ไม่ยกเลิกทั้งรอบ(monkeypatch):
    """เจอจริง (2026-09): เงายางทับติดกับก้ามคีบเป็นก้อนเดียว แยกรูปทรงไม่ออก
    ห้ามยกเลิกทั้งเฟสแค่เพราะเฟรมเดียวแยกไม่ออก ให้ใช้ค่าสำรองไปก่อน
    """
    target = fine.FALLBACK_GRIPPER_TIP_XY
    # จุ๊บเจอนิ่งทุกเฟรม (ใกล้เป้าสำรองพอที่จะเข้าเป้าได้เลย) — ปัญหาอยู่ที่หา
    # ปลาย gripper ไม่เจอเท่านั้น ให้ครบ RETRIES_PER_STEP ไว้กันหมดลิสต์ระหว่างลองซ้ำ
    it = iter([(target[0] + 8, target[1])] * fine.RETRIES_PER_STEP)
    monkeypatch.setattr(fine, "_valve_px_in_frame", lambda frame, session: next(it, None))
    monkeypatch.setattr(fine, "_find_gripper_tip", lambda frame: None)   # หาสดไม่เจอเสมอ

    res = fine_align(FakeCam(), None, _ready_arm(), SCALE, max_steps=8, px_thresh=12.0, settle_sec=0.0)

    assert res.converged is True
    assert res.reason == "เข้าเป้า"


def test_correct_y_False_ไม่แก้แกน_y_และไม่นับตอนเช็คเข้าเป้า(monkeypatch):
    """ใช้ตอนช่วยแมนวล — err_y ใหญ่แค่ไหนก็ไม่สน ขอแค่ err_x เข้าเป้า"""
    target = (640.0, 520.0)
    # err_y คงเดิมทุกรอบ (500px ค้างตลอด ถ้านับจะไม่มีวันเข้าเป้า) err_x ลู่เข้าปกติ
    track = [(target[0] + 80, target[1] + 500), (target[0] + 8, target[1] + 500)]
    _patch_detectors(monkeypatch, track, gripper_xy=target)

    res = fine_align(FakeCam(), None, _ready_arm(), SCALE, max_steps=8, px_thresh=12.0,
                      settle_sec=0.0, correct_x=True, correct_y=False)

    assert res.converged is True
    assert res.reason == "เข้าเป้า"


def test_มองไม่เห็นตอนเริ่มต้องเงยหาขึ้นก่อนยอมแพ้(monkeypatch):
    """เจอจริง (2026-09-12): แขนตกเพราะ J2 รับน้ำหนักไม่ไหวที่ระยะเอื้อมสุด กล้องมอง
    ต่ำกว่าจุ๊บ — ต้องลองขยับขึ้นทีละก้าวจนเห็น ไม่ใช่ยอมแพ้ตั้งแต่เฟรมแรก
    """
    target = (640.0, 520.0)
    # retry รอบแรกไม่เห็นเลย → หลังเงยขึ้น 1 ก้าว เห็นใกล้เป้าจนเข้าเป้าได้ทันที
    track = [None] * fine.RETRIES_PER_STEP + [(target[0] + 8, target[1])]
    _patch_detectors(monkeypatch, track, gripper_xy=target)

    arm = _ready_arm()
    z0 = arm.current()[2]
    res = fine_align(FakeCam(), None, arm, SCALE, max_steps=8, px_thresh=12.0, settle_sec=0.0)

    assert res.converged is True
    assert res.reason == "เข้าเป้า"
    assert arm.current()[2] > z0   # ต้องได้ขยับขึ้นจริง ไม่ใช่แค่ถ่ายซ้ำ


def test_arm_ขยับต่อไม่ได้ต้องหยุด(monkeypatch):
    target = (640.0, 520.0)
    _patch_detectors(monkeypatch, [(target[0] + 80, target[1])] * 3, gripper_xy=target)

    arm = Arm(simulate=True)   # ยังไม่ได้ move_to() ไปท่าที่ IK แก้ได้ — nudge() ต้องปฏิเสธ

    res = fine_align(FakeCam(), None, arm, SCALE, max_steps=8, px_thresh=12.0, settle_sec=0.0)

    assert res.converged is False
    assert res.reason == "แขนขยับต่อไม่ได้"
    assert res.steps == 0
