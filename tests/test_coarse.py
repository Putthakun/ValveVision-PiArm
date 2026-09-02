# tests/test_coarse.py
import cv2
import numpy as np

from arm import Arm
from camera import ReplayCamera
from coarse import _pixel_angle_to_clock, coarse_locate
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
    """กับดักทิศแบบเดียวกับ geometry.valve_pose — ถ้าสลับ x จะได้ 3/9 กลับข้าง"""
    c3 = _pixel_angle_to_clock(hub_xy=(640, 400), valve_xy=(840, 400))
    c9 = _pixel_angle_to_clock(hub_xy=(640, 400), valve_xy=(440, 400))
    assert abs(c3 - 3.0) < 0.1
    assert abs(c9 - 9.0) < 0.1
