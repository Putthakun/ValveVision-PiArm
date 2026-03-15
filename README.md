# ValveVision-PiArm

ระบบควบคุมแขนกล 6-DOF บน Raspberry Pi สำหรับยื่นปลาย gripper ไปยังพิกัด (x, y, z) ที่กำหนด
เป้าหมายหลัก: รับพิกัด valve จากกล้อง แล้วให้แขนกลยื่นไปถึงโดยอัตโนมัติ

---

## โครงสร้างโปรเจ็ค

```
ValveVision-PiArm/
├── main.py                  ← pipeline หลัก (รันตัวนี้ตอนใช้งานจริง)
├── config.py                ← ค่าคงที่ทั้งหมดของแขน
├── ik_solver.py             ← คำนวณ Inverse Kinematics
├── servo_controller.py      ← ควบคุม servo ผ่าน PCA9685
├── test_ik_servo.py         ← ทดสอบพิมพ์พิกัดแล้วแขนเคลื่อน
├── workspace_map.py         ← plot รูป workspace ของแขน
└── setup/                   ← script สำหรับ calibrate (ใช้ครั้งเดียวตอน setup)
    ├── step1_home.py
    ├── step2_test_direction.py
    ├── step3_calibrate_offsets.py
    ├── set_scan_pose.py
    └── servo_scan_pulse.py
```

---

## Hardware

| อุปกรณ์ | รายละเอียด |
|---------|-----------|
| Raspberry Pi 4 | รัน Python 3 |
| PCA9685 | Servo driver 16-channel (I2C) |
| Servo x6 | pulse range 500–2500 µs |
| แขนกล 6-DOF | โครงสร้าง 6 joint (J1–J6) |

### Link Lengths

```
ฐาน [J1]
  │  L1 = 10mm
 [J2]
  │  L2 = 50mm
 [J3]
  │  L3 = 135mm
 [J4]
  │  L4 = 180mm
 ปลาย gripper
```

---

## ไฟล์และหน้าที่

### `config.py` — ค่า configuration ทั้งหมด

เก็บค่าคงที่ทุกอย่างของระบบ แก้ที่นี่ที่เดียว ไฟล์อื่น import ไปใช้

| ตัวแปร | ความหมาย |
|--------|---------|
| `L1–L4` | ความยาว link แต่ละช่วง (mm) |
| `CHANNEL` | mapping joint → PCA9685 channel |
| `PULSE_MIN/MAX` | ขอบเขต pulse width ของ servo (µs) |
| `INVERT` | กลับทิศ servo (True/False) ต่อ joint |
| `ZERO_OFFSET` | offset เพื่อแก้ความคลาดเคลื่อนของ horn (องศา) |
| `LIMITS` | ขอบเขตมุมที่อนุญาต ต่อ joint (องศา) |
| `HOME` | ท่า home (ทุก joint = 90°) |
| `SCAN_POSE` | ท่าเตรียมพร้อม ขณะกล้อง detect |

---

### `ik_solver.py` — Inverse Kinematics

**ทฤษฎี: Geometric Inverse Kinematics**

IK คือการย้อนจาก "ต้องการให้ปลายแขนอยู่ที่ไหน" → "แต่ละ joint ต้องหมุนเท่าไหร่"

#### ขั้นตอนการคำนวณ

**1. J1 — หมุนฐาน (rotation around Z-axis)**
```
J1 = 90° + atan2(y, x)
```
J1 หมุนฐานให้แขนหันหน้าไปหา target เสมอ
atan2 ให้มุมในระนาบ X-Y

**2. แยกปัญหาเป็น 2D (ระนาบ r-z)**

เมื่อรู้ทิศแล้ว ปัญหากลายเป็น 2D:
```
r = sqrt(x² + y²)   ← ระยะแนวนอน
z                    ← ความสูง
```

**3. หาตำแหน่งข้อมือ (J4) — ถอย L4 ออกจากปลาย**

กำหนด `gripper_pitch` = มุม gripper จากแนวนอน (0° = แนวนอน)
```
r_wrist = r - L4 x cos(gripper_pitch)
z_wrist = z - L1 + L4 x sin(gripper_pitch)
```

**4. 2-Link IK สำหรับ J2, J3 → ข้อมือ**

ใช้ Law of Cosines หา J3 ก่อน:
```
d       = sqrt(r_w² + z_w²)
cos(a3) = (d² - L2² - L3²) / (2 x L2 x L3)
a3_rel  = acos(cos(a3))

phi = atan2(r_w, z_w)
psi = atan2(L3 x sin(a3), L2 + L3 x cos(a3))
a2  = phi - psi

J2 = 90° + a2      (องศา)
J3 = 90° + a3_rel
```

**5. J4 — ชดเชยให้ gripper อยู่ที่มุมที่ต้องการ**
```
J4 = 90° + (alpha4 - alpha3)
```

#### Workspace

```
sweet spot (ยืดหยุ่นที่สุด):
  z=100mm → x ≈ 290–340mm
  z=150mm → x ≈ 215–300mm
  z=170mm → x ≈ 170–270mm
```

#### ฟังก์ชัน

| ฟังก์ชัน | พฤติกรรม |
|---------|---------|
| `solve_ik(x, y, z)` | คืน dict หรือ None ถ้าไม่ถึง |
| `solve_ik_clamped(x, y, z)` | ยื่นสุด workspace ในทิศทางนั้น (ไม่มี None) |
| `fk(j2, j3, j4)` | Forward Kinematics สำหรับตรวจสอบ |

---

### `servo_controller.py` — ควบคุม Servo

**ทฤษฎี: PWM Servo Control + Logic Space Mapping**

Servo รับ PWM signal ความกว้าง 500–2500 µs แปลงเป็นมุม 0–180°
ระบบใช้ "logic angle" (space ของ IK) แยกจาก "servo angle" (physical)

#### การแปลง logic → servo

```
servo_angle = logic_angle           (INVERT=False)
servo_angle = 180 - logic_angle    (INVERT=True)
servo_angle += ZERO_OFFSET
servo_angle = clamp(0, 180)
```

**INVERT** — แก้ servo ที่ติดตั้งกลับด้าน
**ZERO_OFFSET** — แก้ horn ที่ขันไม่ตรงพอดี

#### Smooth Motion

Linear interpolation ระหว่างตำแหน่งปัจจุบัน → target
```
angle(t) = current + (target - current) x t     t: 0→1
```

| เมธอด | หน้าที่ |
|-------|--------|
| `set_joint(joint, angle)` | ส่ง angle ทันที |
| `move_smooth(target_dict)` | เคลื่อนทุก joint พร้อมกันแบบ smooth |
| `move_to_scan_pose()` | ไปท่า scan pose |
| `move_to_home()` | ไปท่า home |

---

### `main.py` — Pipeline หลัก

**Flow การทำงาน:**
```
scan pose → detect valve → solve IK → ยื่นแขน → scan pose → loop
```

**จุด swap สำหรับ integrate กล้อง:**
```python
def get_valve_position():
    # แทนด้วย camera detection ตรงนี้
    return x, y, z    # หรือ None ถ้าไม่เจอ
```

---

## ขั้นตอน Setup (ทำครั้งเดียว)

```bash
python setup/step1_home.py            # จัด Horn ที่ 90°
python setup/step2_test_direction.py  # หา INVERT flag
python setup/step3_calibrate_offsets.py  # หา ZERO_OFFSET
python setup/set_scan_pose.py         # จัดท่า scan pose
```

## การใช้งาน

```bash
python test_ik_servo.py   # ทดสอบพิมพ์พิกัดเอง
python main.py            # รัน pipeline จริง
python workspace_map.py   # ดู workspace
```
