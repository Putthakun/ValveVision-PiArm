# tests/test_camera.py
import cv2
import numpy as np
from camera import ReplayCamera


def test_replay_อ่านภาพตามลำดับแล้ววนซ้ำ(tmp_path):
    for i in range(3):
        img = np.full((720, 1280, 3), i * 40, dtype=np.uint8)
        cv2.imwrite(str(tmp_path / f"{i:03d}.jpg"), img)
    cam = ReplayCamera(str(tmp_path))
    seen = [int(cam.grab()[0, 0, 0]) for _ in range(4)]
    assert seen[0] < seen[1] < seen[2]      # เรียงตามชื่อไฟล์
    assert seen[3] == seen[0]               # วนกลับมาเริ่มใหม่


def test_replay_คืน_None_เมื่อโฟลเดอร์ว่าง(tmp_path):
    assert ReplayCamera(str(tmp_path)).grab() is None
