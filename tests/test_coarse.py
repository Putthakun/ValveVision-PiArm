# tests/test_coarse.py
import cv2
import numpy as np

import coarse
from arm import Arm
from camera import ReplayCamera
from coarse import _pixel_angle_to_clock, _scan_from_pose, coarse_locate
from valve_detector import load_model


def test_ไม่เจอวาล์วต้องคืน_ok_False(tmp_path):
    cv2.imwrite(str(tmp_path / "000.jpg"), np.zeros((720, 1280, 3), np.uint8))
    res = coarse_locate(ReplayCamera(str(tmp_path)), load_model(), Arm(simulate=True))
    assert res.ok is False and res.reason == "ไม่เจอวาล์ว"


def test_มุม_12_นาฬิกาคือขึ้นบนจากดุม():
    # จุ๊บอยู่เหนือดุมพอดี (y น้อยกว่า) ต้องได้ 12 นาฬิกา
    clock = _pixel_angle_to_clock(hub_xy=(640, 400), valve_xy=(640, 200))
    assert abs(clock - 12.0) < 0.1


def test_มุม_6_นาฬิกาคือลงล่างจากดุม():
    clock = _pixel_angle_to_clock(hub_xy=(640, 400), valve_xy=(640, 600))
    assert abs(clock - 6.0) < 0.1


def test_มุม_3_และ_9_นาฬิกาอยู่คนละฝั่ง():
    """กับดักทิศแบบเดียวกับ geometry.valve_pose — ถ้าสลับ x จะได้ 3/9 กลับข้าง

    เคยลองกลับเครื่องหมาย dx ไปรอบหนึ่งจากคำบอกเล่าที่ยังไม่ได้ควบคุมตัวแปร
    (2026-08) แต่พอทดสอบจริงแบบมี debug print เทียบกับตำแหน่งที่ตั้งจริง (7 นาฬิกา)
    สูตรเดิม (ไม่กลับ dx) ตรงกว่ามาก จึงกลับมาใช้ค่าเดิม:
    จุ๊บทางขวาของดุมในภาพ (dx>0) คือ 3 นาฬิกา
    """
    c3 = _pixel_angle_to_clock(hub_xy=(640, 400), valve_xy=(840, 400))
    c9 = _pixel_angle_to_clock(hub_xy=(640, 400), valve_xy=(440, 400))
    assert abs(c3 - 3.0) < 0.1
    assert abs(c9 - 9.0) < 0.1


def test_ค่าที่เจอไม่นิ่งต้องไม่ถูกยืนยัน(monkeypatch):
    """เจอจริงจากแขนจริง (2026-09): ดุมล้อสลับไปเจอวัตถุคนละชิ้นกลางทาง
    (ค่ากระโดด 11.8 → 1.5 → 1.5) แต่ "เจอ" (ไม่ None) ครบ 3 เฟรมติดกันพอดี
    ระบบเดิมเลยยืนยันค่าผิด ต้องเช็คว่าค่านิ่งด้วย ไม่ใช่แค่เจอ
    """
    # เจอ 11.8 หนึ่งเฟรม สลับไปเจอวัตถุผิดที่ 1.5 สองเฟรมติด (แบบเดียวกับของจริง)
    # โค้ดเดิมจะยืนยัน 1.5 ผิดๆ ทันทีตรงนี้ (นับแค่ "เจอ" ครบ 3 ไม่สนว่าค่าตรงกันไหม)
    # ก่อนจะกลับมานิ่งจริงที่ 11.8 อีกครั้งตอนหลัง
    readings = iter([11.8, 1.5, 1.5, 11.8, 11.8, 11.8])
    monkeypatch.setattr(coarse, "_detect_once", lambda cam, session, pose_tag: next(readings))

    clock = _scan_from_pose(cam=None, session=None, arm=Arm(simulate=True), upper=False, confirm_frames=3)

    assert clock is not None
    assert abs(clock - 11.8) < 0.1   # ต้องได้ค่าที่นิ่งจริง ไม่ใช่ 1.5 ที่นิ่งปลอมแค่ 2 เฟรม
