# tests/test_geometry.py
import pytest

from geometry import valve_pose


@pytest.mark.parametrize("clock,r,th,z", [
    (5, 348, 102, -50), (6, 340, 90, -70), (7, 348, 78, -50),
    (8, 364, 69, 5),    (9, 372, 66, 80),
])
def test_ตรงกับตารางใน_DESIGN(clock, r, th, z):
    R, TH, Z = valve_pose(clock)
    assert abs(R - r) < 1 and abs(TH - th) < 1 and abs(Z - z) < 1


@pytest.mark.parametrize("clock,r,th,z", [
    (5.5, 342, 96.5, -65),
    (6.5, 342, 83.5, -65),
])
def test_รับมุมต่อเนื่องไม่ใช่แค่จำนวนเต็ม(clock, r, th, z):
    """ครึ่งชั่วโมงต้องคำนวณได้ด้วย เพราะแผนเก็บ dataset ใช้ 9 ตำแหน่ง (ทุกครึ่ง ชม.)

    ปักหมุดทั้งคู่ 5.5 กับ 6.5 ไว้ด้วยกัน เพราะเป็นคู่กระจกที่ r และ z เท่ากันเป๊ะ
    ต่างกันแค่ theta — เป็นจุดที่สูตรกลับทิศแล้วจับได้ยากที่สุด
    """
    R, TH, Z = valve_pose(clock)
    assert abs(R - r) < 1 and abs(TH - th) < 1 and abs(Z - z) < 1


def test_ทิศไม่กลับด้าน():
    """5 นาฬิกาต้องอยู่คนละข้างกับ 7 นาฬิกา — จับกับดัก (clock-6) แทน (6-clock)

    สองตำแหน่งนี้ให้ r กับ z เท่ากันเป๊ะ ต่างกันแค่ theta ถ้าสูตรกลับทิศ
    ตัวเลขทุกตัวจะยังดูถูกต้องแต่แขนกวาดผิดข้าง
    """
    r5, th5, z5 = valve_pose(5)
    r7, th7, z7 = valve_pose(7)

    assert abs(r5 - r7) < 0.01 and abs(z5 - z7) < 0.01   # เหมือนกันทุกอย่าง
    assert th5 > 90 > th7                                 # ยกเว้น theta ที่ต้องคนละข้าง


def test_ตำแหน่ง_6_นาฬิกาอยู่ตรงหน้าและต่ำสุด():
    r, th, z = valve_pose(6)
    assert abs(th - 90) < 0.01        # ตรงหน้าพอดี
    assert z < valve_pose(9)[2]       # ต่ำกว่า 9 นาฬิกา
    assert z < valve_pose(12)[2]      # ต่ำกว่า 12 นาฬิกา (จุดสูงสุด)
