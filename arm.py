# arm.py — ชั้นเดียวที่คุมแขนได้ (ความปลอดภัย + เลือก pitch + โหมดจำลอง)
#
# ทุกคำสั่งไปยัง servo ต้องผ่านไฟล์นี้ ห้ามมีทางลัด (กฎข้อ 4 ของ CLAUDE.md)
#
# ระบบพิกัดที่ใช้: ทรงกระบอก (r, theta_deg, z, pitch_deg) จากแกน J1
#   theta_deg มีความหมายตรงกับมุม J1 (logic angle) เป๊ะ — 90° คือ "ตรงหน้า" (ท่ากลาง)
#   pitch_deg คือ gripper_pitch ใน ik_solver (0 = แนวนอน)

import math

from config import LIMITS, HOME, SCAN_POSE
from ik_solver import solve_ik, fk

R_MAX_COMMAND = 420        # มม. — กันสั่งเลยขอบ workspace แม้ solve_ik จะหาคำตอบได้ (โซนขอบเปราะบาง)
MAX_DEG_PER_SEC = 60       # องศา/วินาที — จำกัดความเร็วการเคลื่อนที่ของ joint ที่ขยับมากที่สุด
RETREAT_DISTANCE_MM = 30   # ถอยจากตำแหน่งปัจจุบันกี่ มม. ตอน retreat() — แตะแล้วถอยทันที ไม่ค้าง (กฎข้อ 5)
STEP_DELAY_SEC = 0.02      # ช่วงเวลาระหว่างก้าวตอนเคลื่อนแบบ smooth
PITCH_SCAN_STEP_DEG = 5
PITCH_SCAN_MIN_DEG = -60
PITCH_SCAN_MAX_DEG = 60


def _polar_to_xy(r: float, theta_deg: float) -> tuple[float, float]:
    """แปลง (r, theta_deg) → (x, y) โดย theta_deg มีความหมายตรงกับมุม J1 เป๊ะ"""
    angle_rad = math.radians(theta_deg - 90.0)
    return r * math.cos(angle_rad), r * math.sin(angle_rad)


def _margin_to_limits(angles: dict) -> float:
    """ระยะเหลือน้อยที่สุดจากขีดจำกัดของทุก joint (ยิ่งมากยิ่งปลอดภัย)"""
    return min(min(v - LIMITS[j][0], LIMITS[j][1] - v) for j, v in angles.items())


class Arm:
    def __init__(self, simulate: bool | None = None):
        self._servo = None

        if simulate is None:
            try:
                from servo_controller import ServoController
                self._servo = ServoController()
                self.simulate = False
            except Exception as e:
                print(f"[Arm] ไม่พบฮาร์ดแวร์จริง เข้าโหมดจำลองอัตโนมัติ ({e})")
                self.simulate = True
        else:
            self.simulate = simulate
            if not simulate:
                from servo_controller import ServoController
                self._servo = ServoController()

        # ตำแหน่งข้อต่อจำลอง ใช้เฉพาะตอน self.simulate=True เพื่อคำนวณความเร็วและพิมพ์สถานะ
        self._sim_joints = dict(HOME)

        # ท่าเริ่มต้น: ประมาณจาก SCAN_POSE ด้วย forward kinematics
        r0, z0 = fk(SCAN_POSE['J2'], SCAN_POSE['J3'], SCAN_POSE['J4'])
        self._pose = {
            'r': r0,
            'theta_deg': SCAN_POSE['J1'],
            'z': z0,
            'pitch_deg': 0.0,
        }

    # ─── เคลื่อนที่ ──────────────────────────────────────────────────────
    def move_to(self, r: float, theta_deg: float, z: float, pitch_deg: float) -> bool:
        """สั่งไปที่ (r, theta_deg, z, pitch_deg) คืน False ถ้าถูกปฏิเสธ ห้ามโยน exception"""
        if r > R_MAX_COMMAND:
            print(f"[Arm] ปฏิเสธ: r={r:.1f} เกิน R_MAX_COMMAND={R_MAX_COMMAND}")
            return False

        x, y = _polar_to_xy(r, theta_deg)
        angles = solve_ik(x, y, z, pitch_deg)
        if angles is None:
            return False

        self._move_joints(angles)
        self._pose = {'r': r, 'theta_deg': theta_deg, 'z': z, 'pitch_deg': pitch_deg}
        return True

    def nudge(self, d_theta_deg: float, d_z: float) -> bool:
        """ขยับจากท่าปัจจุบันทีละนิด (ใช้ในเฟสละเอียด) — ไม่แตะ pitch"""
        r, theta_deg, z, pitch_deg = self.current()
        return self.move_to(r, theta_deg + d_theta_deg, z + d_z, pitch_deg)

    # ─── เลือก pitch ────────────────────────────────────────────────────
    def best_pitch(self, r: float, theta_deg: float, z: float) -> float | None:
        """วน pitch หาค่าที่ทำให้ joint ที่คับที่สุดยังเหลือระยะขยับมากที่สุด คืน None ถ้าไม่มี pitch ไหนทำได้"""
        if r > R_MAX_COMMAND:
            return None

        x, y = _polar_to_xy(r, theta_deg)
        best_margin = None
        best_p = None
        for p in range(PITCH_SCAN_MIN_DEG, PITCH_SCAN_MAX_DEG + 1, PITCH_SCAN_STEP_DEG):
            angles = solve_ik(x, y, z, float(p))
            if angles is None:
                continue
            margin = _margin_to_limits(angles)
            if best_margin is None or margin > best_margin:
                best_margin, best_p = margin, float(p)

        return best_p

    # ─── สถานะ ──────────────────────────────────────────────────────────
    def current(self) -> tuple[float, float, float, float]:
        p = self._pose
        return p['r'], p['theta_deg'], p['z'], p['pitch_deg']

    # ─── ท่าสำเร็จรูป ────────────────────────────────────────────────────
    def go_scan_pose(self) -> None:
        if self.simulate:
            print(f"[Arm][จำลอง] จะสั่งไปท่าสแกน SCAN_POSE = {SCAN_POSE}")
            self._sim_joints.update(SCAN_POSE)
        else:
            self._servo.move_to_scan_pose()

        r0, z0 = fk(SCAN_POSE['J2'], SCAN_POSE['J3'], SCAN_POSE['J4'])
        self._pose = {'r': r0, 'theta_deg': SCAN_POSE['J1'], 'z': z0, 'pitch_deg': 0.0}

    def retreat(self) -> None:
        """แตะแล้วถอยทันที — ถอย r เข้าหาฐาน ไม่ค้างดันของแข็ง (กฎข้อ 5)"""
        r, theta_deg, z, pitch_deg = self.current()
        new_r = max(0.0, r - RETREAT_DISTANCE_MM)
        if not self.move_to(new_r, theta_deg, z, pitch_deg):
            print("[Arm] ถอยตรงๆ ไม่สำเร็จ กลับท่าสแกนแทนเพื่อความปลอดภัย")
            self.go_scan_pose()

    # ─── ภายใน ──────────────────────────────────────────────────────────
    def _move_joints(self, target: dict) -> None:
        """เคลื่อนไปยัง target (dict มุม joint แบบ logic) โดยจำกัดความเร็วไม่เกิน MAX_DEG_PER_SEC"""
        if self.simulate:
            current = {j: self._sim_joints.get(j, 90.0) for j in target}
        else:
            current = self._servo.get_angles(target.keys())

        max_delta = max(abs(target[j] - current[j]) for j in target)
        duration = max_delta / MAX_DEG_PER_SEC
        steps = max(1, round(duration / STEP_DELAY_SEC))

        if self.simulate:
            rounded = {j: round(v, 1) for j, v in target.items()}
            print(f"[Arm][จำลอง] จะสั่ง joints → {rounded} "
                  f"(เดลต้าสูงสุด {max_delta:.1f}°, ~{duration:.2f} วิ, {steps} ก้าว)")
            self._sim_joints.update(target)
        else:
            self._servo.move_smooth(target, steps=steps, delay=STEP_DELAY_SEC)
