# ValveVision-PiArm — คู่มือส่งต่อโปรเจ็ค

## ภาพรวม

ระบบแขนกล 6 แกน (6-DOF) บน Raspberry Pi 4 ที่ตรวจจับ **วาล์วเติมลมยาง (จุ๊บลม)**  
ด้วยกล้อง + YOLO แล้วสั่งให้แขนกลยื่นไปแตะหัววาล์วโดยอัตโนมัติ

```
กล้อง detect valve → pixel → (x, y, z) mm → IK คำนวณมุม → Servo ขยับแขนกล
```

---

## สถานะโปรเจ็คปัจจุบัน ✅

| งาน | สถานะ |
|-----|-------|
| Servo calibration (INVERT, ZERO_OFFSET) | ✅ Done |
| Camera calibration (CAM_X, CAM_Z, CAM_TILT, FOCAL_LENGTH) | ✅ Done |
| IK solver (solve_ik, solve_ik_clamped, fk) | ✅ Done |
| YOLO model (best_v2.onnx) | ✅ Done |
| 3-Phase reach pipeline (approach → servo → advance) | ✅ Done |
| ทดสอบตำแหน่ง 7–8 นาฬิกา | ✅ ทำงานได้ |
| ตำแหน่ง 9–10 นาฬิกา | ❌ เกิน workspace (ดูหัวข้อ Limitation) |

---

## โครงสร้างแขนกล

```
[J6 - Gripper]
      |  L4 = 175 mm
[J4 - Wrist Pitch]
      |  L3 = 140 mm
[J3 - Elbow]
      |  L2 = 105 mm
[J2 - Shoulder]
      |  L1 = 10 mm
[J1 - Base (หมุนซ้าย-ขวา)]
```

**Workspace ที่ใช้งานได้จริง** (valve ตำแหน่ง 5–8 นาฬิกา):

| z (mm) | r สูงสุดที่แขนถึง |
|--------|----------------|
| 85 mm  | 407 mm |
| 141 mm | 390 mm |
| 170 mm | 360 mm |

---

## โครงสร้างไฟล์

```
ValveVision-PiArm/
├── config.py           ← ★ ค่าตั้งค่าทั้งหมด แก้ที่นี่ที่เดียว
├── main.py             ← รัน pipeline อัตโนมัติ
├── ik_solver.py        ← คำนวณ Inverse Kinematics
├── servo_controller.py ← ควบคุม Servo ผ่าน PCA9685
├── valve_detector.py   ← โหลดโมเดล YOLO + detect
├── test_ik_servo.py    ← ทดสอบ manual พิมพ์พิกัด
├── camera_preview.py   ← ดูภาพกล้อง + detection preview
├── models/
│   └── best_v2.onnx    ← YOLO model ตัวล่าสุด
├── results/
│   ├── detection_test.csv      ← บันทึกผลทดสอบ detection
│   └── reach_accuracy_test.csv ← บันทึกผลทดสอบความแม่นยำ
└── setup/
    ├── step1_home.py              ← จัด Horn ที่ 90°
    ├── step2_test_direction.py    ← หาทิศหมุน INVERT
    ├── step3_calibrate_offsets.py ← หา ZERO_OFFSET
    └── set_scan_pose.py           ← กำหนดท่า scan
```

---

## ค่า config.py ปัจจุบัน (calibrated แล้ว)

```python
# ── Link Lengths ──────────────────────────────────────────
L1 = 10     # mm  (base → shoulder)
L2 = 105    # mm  (shoulder → elbow)
L3 = 140    # mm  (elbow → wrist)
L4 = 175    # mm  (wrist → gripper tip)

# ── Servo Channels (PCA9685) ──────────────────────────────
J1=0, J2=1, J3=2, J4=3, J5=4, J6=5

# ── Calibration ───────────────────────────────────────────
INVERT      = {J1:True, J2:True, J3:False, J4:False, J5:False, J6:False}
ZERO_OFFSET = {J1:0, J2:9, J3:4, J4:-3, J5:0, J6:0}

# ── Camera Model ──────────────────────────────────────────
CAM_X        = 260    # mm — กล้องห่าง J1 แนวนอน
CAM_Z        = 290    # mm — กล้องสูงกว่า J1
CAM_TILT     = 8      # องศา — กล้องก้มลงจากแนวนอน
FOCAL_LENGTH = 233    # px
VALVE_REAL_MM = 12    # mm — ขนาด valve จริง

# ── Target Offset (ชดเชย error ของกล้อง) ─────────────────
TARGET_OFFSET_X =   5   # mm
TARGET_OFFSET_Y = -36   # mm
TARGET_OFFSET_Z = -62   # mm
```

---

## วิธีรัน

### 1. ดูกล้อง + detection (ไม่ขยับแขน)
```bash
python camera_preview.py
```

### 2. ทดสอบแขนด้วยพิกัด manual
```bash
python test_ik_servo.py
# พิมพ์: x y z  เช่น  390 -100 85
# พิมพ์: home   เพื่อกลับท่าพัก
# พิมพ์: q      เพื่อออก
```

### 3. รันระบบอัตโนมัติ (detect + ขยับแขน)
```bash
python main.py
```

**Flow ของ main.py:**
```
1. แขนไปท่า Scan Pose → กล้องมองเห็น valve
2. Detect 3 เฟรม → ยืนยัน valve
3. แปลง pixel → (x, y, z) mm → บวก TARGET_OFFSET
4. Phase 1: Approach — ยื่นไปหยุดก่อน valve 80mm
5. Phase 2: Visual Servo — ปรับ y, z ละเอียด (ถ้ากล้องยังเห็น)
6. Phase 3: Advance — ยื่นไกลสุดเท่าที่ IK ทำได้
7. รอจน valve หาย → กลับ Scan Pose → วนซ้ำ
```

---

## ถ้าประกอบแขนกลใหม่ — ต้อง Calibrate ใหม่

ทำตามลำดับ:

```bash
# Step 1: จัด Horn ที่ตำแหน่ง 90° แล้วขันสกรู
python setup/step1_home.py

# Step 2: ทดสอบทิศหมุน → แก้ INVERT ใน config.py
python setup/step2_test_direction.py

# Step 3: ปรับ offset ทีละข้อต่อ → แก้ ZERO_OFFSET ใน config.py
python setup/step3_calibrate_offsets.py

# Step 4: ปรับท่า scan pose → บันทึกอัตโนมัติ
python setup/set_scan_pose.py
```

จากนั้น calibrate camera โดยวัดค่าจริงและปรับ:
- `CAM_X` — ระยะกล้องจาก J1 แนวนอน
- `CAM_Z` — ความสูงกล้องจาก J1
- `CAM_TILT` — มุมกล้องก้ม (วัดด้วย level app บนมือถือ)

ปรับ TARGET_OFFSET โดยทดสอบด้วย `test_ik_servo.py`:
1. hardcode ไป valve จริงแล้ววัดค่า detect ได้
2. `TARGET_OFFSET = ค่าจริง - ค่า detect`

---

## Workspace Limitation ที่ต้องรู้

**ระบบนี้ทำงานได้เฉพาะ valve ที่ตำแหน่ง 5–8 นาฬิกา เท่านั้น**

ที่ตำแหน่ง 9–10 นาฬิกา valve อยู่สูง (z > 200mm) แขนยื่นได้ไม่ถึง  
เพราะ max reach = `L4 + √((L2+L3)² - z_adj²)` ลดลงเมื่อ z สูงขึ้น

**แนวทางแก้ไขในอนาคต:**
- เพิ่มความยาว L2+L3 (ต้องสร้างแขนใหม่)
- ติดกล้องบนแขนแทนกล้องตรึง (Eye-in-Hand)

---

## การติดตั้ง

### 1. สร้าง Virtual Environment
```bash
python3 -m venv venv
source venv/bin/activate
```

### 2. ติดตั้ง Library ทั้งหมด
```bash
pip install -r requirements.txt
```

### 3. รายละเอียด Library แต่ละตัว

| Library | ใช้ทำอะไร | ติดตั้งแยก |
|---------|----------|-----------|
| `numpy` | คำนวณเมทริกซ์, ตัวเลข | `pip install numpy` |
| `opencv-python-headless` | อ่านภาพกล้อง, resize, แสดงผล | `pip install opencv-python-headless` |
| `pillow` | จัดการไฟล์ภาพ (PNG/JPG) | `pip install pillow` |
| `onnxruntime` | รัน YOLO model (.onnx) | `pip install onnxruntime` |
| `scipy` | ไม่ได้ใช้หลัก (optional) | `pip install scipy` |
| `adafruit-blinka` | ให้ Raspberry Pi ใช้ CircuitPython library | `pip install adafruit-blinka` |
| `adafruit-circuitpython-pca9685` | ควบคุม PCA9685 Servo Driver ผ่าน I2C | `pip install adafruit-circuitpython-pca9685` |

### 4. เปิดใช้ I2C บน Raspberry Pi (ทำครั้งเดียว)
```bash
sudo raspi-config
# → Interface Options → I2C → Enable
sudo reboot
```

ตรวจสอบว่า PCA9685 ต่ออยู่ถูกต้อง:
```bash
sudo i2cdetect -y 1
# ต้องเห็น address 0x40 ในตาราง
```

### 5. ตั้งค่า Camera Stream (ถ้าใช้ HTTP stream)
```bash
# ถ้าใช้ Pi Camera ผ่าน mjpg-streamer หรือ camera_preview.py
export CAMERA_SOURCE=http
export CAMERA_URL=http://localhost:8080/?action=stream
```

---

## Hardware

| อุปกรณ์ | รุ่น/สเปค |
|---------|----------|
| Raspberry Pi | Pi 4 |
| Servo Driver | PCA9685 (I2C address 0x40) |
| Servo | Standard RC, pulse 500–2500 µs |
| กล้อง | USB webcam / Pi Camera (ผ่าน http stream) |
| Power | Servo ต้องการ 5V/3A แยกจาก Pi |

---

## ไฟล์ผลการทดสอบ

กรอกผลจริงลงใน `results/`:
- `detection_test.csv` — ผล detect valve แต่ละชนิด/แสง
- `reach_accuracy_test.csv` — ผลความแม่นยำ (cmd vs fk vs จริง)

FK (Forward Kinematics) ใช้ตรวจสอบตำแหน่งจริงของปลายแขน:
```python
from ik_solver import fk
r, z = fk(j2_deg, j3_deg, j4_deg)
```
