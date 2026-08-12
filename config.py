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

# ── Joint Limits (องศา) — วัดจริงจากแขนด้วย tools/find_joint_limits.py ────
# (Task 1, rebuild 2026-08) หักระยะปลอดภัย 10° เข้ามาจากจุดที่ชนจริงแล้ว
# J5, J6 ไม่ได้ใช้งาน ยังไม่ได้วัด คงค่าเดิมไว้ (0,180)
LIMITS = {
    'J1': (10,   170),
    'J2': (70,   170),
    'J3': (63,   170),
    'J4': (12,   170),
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

