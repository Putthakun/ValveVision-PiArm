# ValveVision-PiArm

ระบบควบคุมแขนกล 6-DOF บน Raspberry Pi 4 ผ่าน PCA9685
รองรับการส่งพิกัด x, y, z (mm) แล้วให้แขนเคลื่อนไปยังตำแหน่งนั้น

---

## Hardware

| ชิ้นส่วน | รุ่น / สเปค |
|----------|------------|
| SBC | Raspberry Pi 4 |
| Servo driver | PCA9685 (I2C, address 0x40) |
| Servo | × 6 ตัว, pulse range 500–2500 µs |

---

## โครงสร้างแขน

```
ปลาย gripper
    │← L4 = 180 mm  (J4 → ปลาย)
   [J4] wrist pitch
    │← L3 = 135 mm  (J3 → J4)
   [J3] elbow
    │← L2 =  50 mm  (J2 → J3)
   [J2] shoulder
    │← L1 =  10 mm  (J1 → J2)
   [J1] base (หมุนซ้าย-ขวา)
════════════ ฐาน
```

| Joint | Channel | INVERT | ZERO_OFFSET |
|-------|---------|--------|-------------|
| J1 Base | CH0 | False | 0 |
| J2 Shoulder | CH1 | True | 0 |
| J3 Elbow | CH2 | True | +7° |
| J4 Wrist pitch | CH3 | False | +7° |
| J5 Wrist roll | CH4 | False | 0 |
| J6 Gripper | CH5 | False | 0 |

---

## โครงสร้างไฟล์

```
ValveVision-PiArm/
├── config.py                  # ค่า link, channel, limits, offsets
├── ik_solver.py               # IK: แปลง (x,y,z) → joint angles
├── servo_controller.py        # ส่งคำสั่งไป servo ผ่าน PCA9685
├── step1_home.py              # ส่งทุก joint → 90° เพื่อจัด horn
├── step2_test_direction.py    # หา INVERT flag ทีละ joint
├── step3_calibrate_offsets.py # หา ZERO_OFFSET ทีละ joint
├── servo_scan_pulse.py        # ส่ง pulse ตรงๆ เพื่อ calibrate
├── test_ik_servo.py           # ทดสอบส่งพิกัด x,y,z → แขนจริง
└── requirements.txt
```

---

## ติดตั้ง

```bash
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

---

## Setup แขนใหม่ (ทำครั้งเดียว)

### 1 — จัด Horn
```bash
python step1_home.py
```
ส่งทุก joint → 90° แล้วขัน horn ให้แขนตั้งตรง

### 2 — หา INVERT
```bash
python step2_test_direction.py
```
ขยับทีละ joint แล้วตอบ y/n → print ค่า `INVERT` ให้คัดลอกใส่ `config.py`

### 3 — หา ZERO_OFFSET
```bash
python step3_calibrate_offsets.py
```
ปรับด้วย `+` `-` `>` `<` จนแขนตรง → print ค่า `ZERO_OFFSET` ให้คัดลอกใส่ `config.py`

---

## ใช้งาน

### ทดสอบส่งพิกัด
```bash
python test_ik_servo.py
```
```
>>> 300 0 150      # x=300mm, y=0, z=150mm
>>> 330 0 100
>>> home           # กลับ home
>>> q              # ออก
```

### Workspace (horizontal gripper)

สำหรับ y=0, gripper แนวนอน พิกัดที่ใช้งานได้:

| z (mm) | x range (mm) |
|--------|--------------|
| 100 | 290 – 340 |
| 150 | 215 – 300 |
| 170 | 170 – 270 |

> ถ้า IK คืน None = พิกัดอยู่นอก workspace

---

## IK ใน Code

```python
from ik_solver import solve_ik
from servo_controller import ServoController

arm = ServoController()
arm.move_to_home()

angles = solve_ik(300, 0, 150)   # x, y, z (mm)
if angles:
    arm.move_smooth(angles)
```

---

## Troubleshooting

| อาการ | วิธีแก้ |
|-------|--------|
| PCA9685 ไม่ตอบสนอง | `i2cdetect -y 1` ต้องเห็น `0x40` |
| Servo กระตุกรุนแรง | ตรวจ pulse range 500–2500 µs ใน `config.py` |
| IK คืน None | พิกัดนอก workspace หรือ joint limit เกิน |
| แขนเบี้ยวหลัง calibrate | ปรับ `ZERO_OFFSET` ใน `config.py` |
