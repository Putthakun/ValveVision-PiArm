# tests/test_arm.py
from arm import Arm


def test_ปฏิเสธเป้าที่เอื้อมไม่ถึง():
    arm = Arm(simulate=True)
    assert arm.move_to(r=900, theta_deg=90, z=100, pitch_deg=0) is False


def test_ปฏิเสธเป้าที่ยื่นเกินขีดสูงสุด():
    arm = Arm(simulate=True)
    assert arm.move_to(r=460, theta_deg=90, z=100, pitch_deg=0) is False


def test_เลือก_pitch_ที่เหลือระยะขยับมากสุด():
    arm = Arm(simulate=True)
    p = arm.best_pitch(r=350, theta_deg=90, z=100)
    assert p is not None and -60 <= p <= 60


def test_best_pitch_คืน_None_เมื่อไม่มี_pitch_ไหนทำได้():
    arm = Arm(simulate=True)
    assert arm.best_pitch(r=900, theta_deg=90, z=100) is None


def test_โหมดจำลองไม่ต้องใช้ฮาร์ดแวร์():
    arm = Arm(simulate=True)
    assert arm.move_to(r=350, theta_deg=90, z=100, pitch_deg=15) is True
    r, th, z, p = arm.current()
    assert abs(r - 350) < 1 and abs(z - 100) < 1
