# config.py — ตั้งค่าแขนกล 6-DOF

# ── Link Lengths (mm) วัดจาก pivot ถึง pivot ──────────────────────────────
L1 = 10    # J1 → J2  (ความสูง base ถึง shoulder)
L2 = 105    # J2 → J3  (shoulder ถึง elbow)
L3 = 140   # J3 → J4  (elbow ถึง wrist)
L4 = 175   # J4 → ปลาย gripper

# ── Channel Map (PCA9685) ──────────────────────────────────────────────────
CHANNEL = {
    'J1': 0,   # Base     (หมุนซ้าย-ขวา)
    'J2': 1,   # Shoulder (ยก-กด)
    'J3': 2,   # Elbow
    'J4': 3,   # Wrist pitch
    'J5': 4,   # Wrist roll
    'J6': 5,   # Gripper
}

# ── Pulse Range (µs) ───────────────────────────────────────────────────────
PULSE_MIN = 500
PULSE_MAX = 2500

# ── Invert — กรอกหลังรัน step2_test_direction.py ─────────────────────────
# False = เพิ่ม angle → servo หมุนตามเข็ม
# True  = เพิ่ม angle → servo หมุนทวนเข็ม (พลิก)
INVERT = {
    'J1': True,
    'J2': True,
    'J3': False,
    'J4': False,
    'J5': False,
    'J6': False,
}

# ── Zero Offset (องศา) — ปรับหลัง step3_calibrate ────────────────────────
# ใช้แก้ถ้า servo อยู่ที่ 90° แล้วแขนไม่ตั้งตรงพอดี
ZERO_OFFSET = {
    'J1': 0, 'J2': 9, 'J3': 4,
    'J4': -3, 'J5': 0, 'J6': 0,
}

# ── Joint Limits (องศา) — ปรับหลังทดสอบว่าแขนชนอะไรบ้าง ─────────────────
LIMITS = {
    'J1': (0,   180),
    'J2': (0,   180),
    'J3': (0,   180),
    'J4': (0,   180),
    'J5': (0,   180),
    'J6': (0,   180),
}

# ── Home Pose (องศา) — ทุก joint ตั้งตรง ─────────────────────────────────
HOME = {
    'J1': 90,
    'J2': 90,
    'J3': 90,
    'J4': 90,
    'J5': 90,
    'J6': 90,
}


# ── Scan Pose (องศา) — ท่าเตรียมพร้อม ขณะกล้อง detect ──────────────────────
SCAN_POSE = {
    'J1': 90.0,
    'J2': 50.0,
    'J3': 180.0,
    'J4': 180.0,
    'J5': 90.0,
    'J6': 90.0,
}

# ── Workspace Reference ────────────────────────────────────────────────────
# z=100mm → x ≈ 290-340mm
# z=150mm → x ≈ 215-300mm
# z=170mm → x ≈ 170-270mm

# ── Target Offset (mm) — ชดเชย error ที่สังเกตได้จากการทดสอบ ──────────────
# บวก = ยื่นออก / ขึ้นบน / ขวา   ลบ = หด / ลงล่าง / ซ้าย
TARGET_OFFSET_X =   5    # ยื่นออก / หดเข้า  (390 - 385)
TARGET_OFFSET_Y = -36    # ซ้าย(-) / ขวา(+)  (-100 - -82, -8 extra)
TARGET_OFFSET_Z = -62    # ขึ้น(+) / ลง(-)   (ปรับหลัง CAM_TILT=8)

# ── Gravity Droop Compensation ────────────────────────────────────────────
# ยิ่งแขนยื่นออกไกล (r ใหญ่) เซอร์โวรับน้ำหนักไม่ไหว → z หล่นลง
# Z_DROOP_COMP = จำนวน mm ที่ต้องบวก z เพิ่มต่อ mm ของ r
# วิธีหาค่า: สั่งไป r=300 วัด z จริงที่หล่น → หาร 300
# เช่น หล่น 15mm ที่ r=300 → Z_DROOP_COMP = 15/300 = 0.05
Z_DROOP_COMP = 0.0

# ── Camera Calibration ─────────────────────────────────────────────────────
CAM_X          = 260    # mm — กล้องห่าง J1 แนวนอน
CAM_Y          = 0      # mm — กล้องอยู่แนวเดียวกับแขน
CAM_Z          = 290    # mm — กล้องสูงกว่า J1
CAM_TILT       = 8      # องศา — กล้องเอียงลงจากแนวนอน
FOCAL_LENGTH   = 233    # px — recalibrated จาก actual position (x=300)
VALVE_REAL_MM  = 12     # mm — ขนาด Schrader valve จริง
IMAGE_W        = 1280   # px
IMAGE_H        = 720    # px
