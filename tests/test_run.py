# tests/test_run.py
import pytest

import run
from arm import Arm
from coarse import CoarseResult
from fine import FineResult
from geometry import valve_pose

SCALE = {"deg_per_px_x": -0.0469, "mm_per_px_y": -0.158, "aim_from": "calibrated",
         "aim_x": 510, "aim_y": 320}


class FakeCam:
    def grab(self):
        return object()

    def close(self):
        pass


def _ready_arm() -> Arm:
    """แขนที่อยู่ท่าซึ่ง IK แก้ได้แล้ว (เหมือนหลัง coarse_locate สำเร็จ)"""
    arm = Arm(simulate=True)
    r, theta, z = valve_pose(9)
    arm.move_to(r, theta, z, arm.best_pitch(r, theta, z))
    return arm


def _patch(monkeypatch, coarse_res, fine_res=None):
    monkeypatch.setattr(run, "coarse_locate", lambda *a, **k: coarse_res)
    monkeypatch.setattr(run, "fine_align", lambda *a, **k: fine_res)
    monkeypatch.setattr(run, "_load_scale_for_pitch", lambda *a, **k: SCALE)


def _coarse_ok() -> CoarseResult:
    r, theta, z = valve_pose(9)
    return CoarseResult(ok=True, r=r, theta_deg=theta, z=z, pitch_deg=0.0, reason="ok")


def test_ไม่เจอวาล์วต้องจบที่ท่าสแกน(monkeypatch):
    _patch(monkeypatch, CoarseResult(ok=False, reason="ไม่เจอวาล์ว"))
    arm = _ready_arm()

    outcome = run.run_once(FakeCam(), None, arm, settle_sec=0.0)

    assert outcome.state == "RECOVER"
    assert outcome.touched is False
    assert "ไม่เจอวาล์ว" in outcome.reason
    assert arm.at_scan_pose is True   # ★ ทุกทางต้องจบที่ท่าสแกน


def test_เอื้อมไม่ถึงต้องจบที่ท่าสแกน(monkeypatch):
    _patch(monkeypatch, CoarseResult(ok=False, reason="เอื้อมไม่ถึง"))
    arm = _ready_arm()

    outcome = run.run_once(FakeCam(), None, arm, settle_sec=0.0)

    assert outcome.touched is False
    assert "เอื้อมไม่ถึง" in outcome.reason
    assert arm.at_scan_pose is True


def test_เฟสละเอียดไม่ลู่เข้าต้องไม่แตะ_และจบที่ท่าสแกน(monkeypatch):
    _patch(monkeypatch, _coarse_ok(),
           FineResult(converged=False, steps=5, final_px_err=90.0, reason="ครบรอบสูงสุด"))
    arm = _ready_arm()

    outcome = run.run_once(FakeCam(), None, arm, settle_sec=0.0)

    assert outcome.touched is False   # ★ ไม่ลู่เข้า ห้ามดันหน้าไปแตะ
    assert "ครบรอบสูงสุด" in outcome.reason
    assert arm.at_scan_pose is True


def test_ลู่เข้าแล้วต้องแตะแล้วถอยทันที_จบที่ท่าสแกน(monkeypatch):
    _patch(monkeypatch, _coarse_ok(),
           FineResult(converged=True, steps=3, final_px_err=15.0, reason="เข้าเป้า"))
    arm = _ready_arm()

    outcome = run.run_once(FakeCam(), None, arm, settle_sec=0.0)

    assert outcome.touched is True
    assert outcome.state == "RECOVER"
    assert arm.at_scan_pose is True   # ★ แตะแล้วถอย ไม่ค้างอยู่ในซอกล้อ


def test_ดันหน้าตอนแตะต้องไม่เกินขีดที่กำหนดใน_DESIGN_md(monkeypatch):
    """★ กฎ DESIGN.md: ห้ามสั่งเกินตำแหน่งที่ตรวจจับได้เกิน ~5 มม. (กันเฟืองไหม้)

    เดิม PLAN.md เขียนว่าเดินหน้า 12 ซม. — ค่านั้นมาจากตอนยังมี standoff 12 ซม.
    ซึ่งถอดออกไปแล้ว ถ้าเดินหน้า 12 ซม.จริงจะพุ่งทะลุเข้าล้อ
    """
    assert run.TOUCH_PUSH_MM <= 5.0

    _patch(monkeypatch, _coarse_ok(),
           FineResult(converged=True, steps=3, final_px_err=15.0, reason="เข้าเป้า"))
    arm = _ready_arm()
    moves = []
    real_move_to = arm.move_to
    monkeypatch.setattr(arm, "move_to",
                        lambda r, th, z, p: (moves.append(r), real_move_to(r, th, z, p))[1])

    r_before = arm.current()[0]
    run.run_once(FakeCam(), None, arm, settle_sec=0.0)

    assert max(moves) <= r_before + run.TOUCH_PUSH_MM + 0.01


def test_ขัดจังหวะกลางคันต้องถอยก่อนออก(monkeypatch):
    """ดัก KeyboardInterrupt — ห้ามทิ้งแขนค้างอยู่ในซอกล้อ"""
    def boom(*a, **k):
        raise KeyboardInterrupt

    monkeypatch.setattr(run, "coarse_locate", boom)
    arm = _ready_arm()

    with pytest.raises(KeyboardInterrupt):
        run.run_once(FakeCam(), None, arm, settle_sec=0.0)

    assert arm.at_scan_pose is True
