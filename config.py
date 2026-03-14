# config.py — ตั้งค่าแขนกล 6-DOF

# ── Link Lengths (mm) วัดจาก pivot ถึง pivot ──────────────────────────────
L1 = 10    # J1 → J2  (ความสูง base ถึง shoulder)
L2 = 50    # J2 → J3  (shoulder ถึง elbow)
L3 = 135   # J3 → J4  (elbow ถึง wrist)
L4 = 180   # J4 → ปลาย gripper

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
    'J1': False,
    'J2': True,
    'J3': True,
    'J4': False,
    'J5': False,
    'J6': False,
}

# ── Zero Offset (องศา) — ปรับหลัง step3_calibrate ────────────────────────
# ใช้แก้ถ้า servo อยู่ที่ 90° แล้วแขนไม่ตั้งตรงพอดี
ZERO_OFFSET = {
    'J1': 0, 'J2': 0, 'J3': 7,
    'J4': 7, 'J5': 0, 'J6': 0,
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


#workspace
#z=100  → x ≈ 290-340mm
#z=150  → x ≈ 215-300mm
#z=170  → x ≈ 170-270mm
