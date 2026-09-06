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
    """จำลองว่าทุกครั้งที่ nudge ความคลาดเคลื่อนลดลงครึ่งหนึ่ง จนต่ำกว่า px_thresh"""
    target = (640.0, 520.0)
    track = [(target[0] + 80, target[1]), (target[0] + 40, target[1]),
             (target[0] + 20, target[1]), (target[0] + 8, target[1])]
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


def test_ครบรอบสูงสุดต้องหยุด_ไม่วนไม่จบ(monkeypatch):
    """ความคลาดเคลื่อนไม่ลดลงเลย (ค้างที่ 80px) ต้องไม่วนเกิน max_steps"""
    target = (640.0, 520.0)
    track = [(target[0] + 80, target[1])] * 20   # เผื่อไว้เกิน max_steps เยอะๆ
    _patch_detectors(monkeypatch, track, gripper_xy=target)

    res = fine_align(FakeCam(), None, _ready_arm(), SCALE, max_steps=5, px_thresh=12.0, settle_sec=0.0)

    assert res.converged is False
    assert res.reason == "ครบรอบสูงสุด"
    assert res.steps == 5


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


def test_arm_ขยับต่อไม่ได้ต้องหยุด(monkeypatch):
    target = (640.0, 520.0)
    _patch_detectors(monkeypatch, [(target[0] + 80, target[1])] * 3, gripper_xy=target)

    arm = Arm(simulate=True)   # ยังไม่ได้ move_to() ไปท่าที่ IK แก้ได้ — nudge() ต้องปฏิเสธ

    res = fine_align(FakeCam(), None, arm, SCALE, max_steps=8, px_thresh=12.0, settle_sec=0.0)

    assert res.converged is False
    assert res.reason == "แขนขยับต่อไม่ได้"
    assert res.steps == 0
