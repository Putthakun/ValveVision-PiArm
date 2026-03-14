# ValveVision-PiArm

ระบบตรวจจับวาล์ว (valve) ด้วย YOLOv8 ONNX บน Raspberry Pi 4
พร้อมควบคุมแขนกล 6-DOF ผ่าน PCA9685

---

## Hardware

| ชิ้นส่วน | รุ่น / สเปค |
|----------|------------|
| SBC | Raspberry Pi 4 |
| Camera | Camera Module V3 (ติดด้านบน มองลงหา valve) |
| Servo driver | PCA9685 (I2C, address 0x40) |
| Servo | RDS3115 MG × 6 ตัว |
| Model | YOLOv8 ONNX, 1 class (valve), input 640×640 |

---

## โครงสร้างไฟล์

```
ValveVision-PiArm/
├── models/
│   └── Valve_detection_model.onnx   # YOLOv8 export
├── captures/                        # ภาพที่บันทึกเมื่อ detect ได้
├── docs/
│   └── home_position_setup.md       # วิธีประกอบ servo + จัด HOME
├── infer_cam_onnx.py                # Main loop: camera + detection + arm
├── infer_image_onnx.py              # ทดสอบ model กับภาพนิ่ง
├── home_position.py                 # ArmController (PCA9685)
├── servo_scan_pulse.py              # Calibrate servo ทีละตัว
├── test_camera.py                   # ทดสอบมุมกล้อง (headless / no GUI)
└── requirements.txt
```

---

## ติดตั้ง

```bash
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
# picamera2 ติดมากับ Pi OS แล้ว ถ้าไม่มีให้รัน:
# sudo apt install python3-picamera2
```

---

## Servo Calibration (วัดจริง)

> เรียงข้อต่อจากล่างขึ้นบน

| CH | Joint | MIN (µs) | MID (µs) | MAX (µs) | HOME (µs) | หมายเหตุ |
|----|-------|----------|----------|----------|-----------|---------|
| 0 | Base | 600 | 1500 | 2400 | **1500** | |
| 1 | Shoulder | 1200 | 1500 | 2400 | **2000** | |
| 2 | Elbow | 600 | 1500 | 2400 | **600** | |
| 3 | Wrist | 600 | 1500 | 2400 | **2500** | |
| 4 | Rotate | 600 | 1500 | 2400 | **1500** | |
| 5 | Grip | 1200 | 1500 | 1700 | **1700** | 1200=เปิด, 1700=ปิด |

---

## รันแต่ละไฟล์

### ทดสอบมุมกล้อง — `test_camera.py`

> สำหรับ remote ผ่าน VSCode ที่ไม่มี GUI

```bash
# ถ่ายภาพนิ่ง → บันทึก camera_test.jpg (เปิดดูใน VSCode ได้เลย)
python test_camera.py

# Live stream ที่ port 8080
python test_camera.py --stream
# เปิด browser บน host → http://<IP_ของ_PI>:8080

# หา IP
hostname -I
```

### ทดสอบ model กับภาพนิ่ง — `infer_image_onnx.py`

```bash
python infer_image_onnx.py
# บันทึกผลเป็น output.jpg
```

| ค่า | ค่าเริ่มต้น |
|-----|------------|
| `CONF_TH` | 0.20 |
| `IOU_TH` | 0.45 |
| `IMAGE_PATH` | `test_images/IMG_1138.JPG` |

### Real-time detection + arm — `infer_cam_onnx.py`

```bash
python infer_cam_onnx.py
```

| ค่า | ค่าเริ่มต้น | ความหมาย |
|-----|------------|----------|
| `CONF_THRES` | 0.30 | confidence threshold |
| `IOU_THRES` | 0.45 | NMS IoU threshold |
| `SAVE_COOLDOWN` | 3 วินาที | ความถี่บันทึกภาพ |

**Logic:**

```
detect valve  →  บันทึกภาพไว้ใน captures/
valve หาย    →  arm กลับ HOME position
```

### Calibrate servo — `servo_scan_pulse.py`

```bash
# แก้ CHANNEL = 0..5 ใน code ก่อนรัน
python servo_scan_pulse.py
```

| ปุ่ม | pulse | ผล |
|------|-------|----|
| `1` | 600 µs | MIN |
| `2` | 1500 µs | MID (90°) ← ใช้จัด horn |
| `3` | 2500 µs | MAX |
| `r` | 0 | relax |
| `q` | — | ออก |

รายละเอียดการประกอบ → [docs/home_position_setup.md](docs/home_position_setup.md)

---

## Pipeline (`infer_cam_onnx.py`)

```
Camera (640×480 RGB)
    │
    ▼
Letterbox → 640×640 + gray padding
    │
    ▼
ONNX Inference  →  output (1, 5, 8400)
    │               transpose → (8400, 5)
    ▼
Parse: cx, cy, w, h, conf
    │
    ▼
Filter conf > 0.30  →  NMS (cv2.dnn.NMSBoxes)
    │
    ├─ found  →  วาด bbox, บันทึก captures/
    └─ lost   →  arm.move_home()
```

---

## Troubleshooting

| อาการ | วิธีแก้ |
|-------|--------|
| `ImportError: picamera2` | `sudo apt install python3-picamera2` |
| กล้องไม่ติด | ทดสอบ `libcamera-hello` |
| PCA9685 ไม่ตอบสนอง | `i2cdetect -y 1` ต้องเห็น `0x40` |
| Servo กระตุกรุนแรง | ตรวจ pulse ไม่เกิน 500–2500 µs |
| Model ไม่โหลด | ตรวจ `models/Valve_detection_model.onnx` มีอยู่จริง |
