# servo_controller.py
import time
from adafruit_servokit import ServoKit
from config import INVERT, ZERO_OFFSET, CHANNEL, LIMITS, HOME

PULSE_MIN = 500   # µs
PULSE_MAX = 2500  # µs




class ServoController:

    def __init__(self):
        self.kit = ServoKit(channels=16)
        for ch in range(6):
            self.kit.servo[ch].set_pulse_width_range(PULSE_MIN, PULSE_MAX)

    # ─── แปลง logic → servo angle ──────────────────────────────────────
    def _to_servo(self, joint: str, logic: float) -> float:
        """แปลง logic angle (IK space) → servo angle โดยใช้ INVERT + ZERO_OFFSET"""
        angle = (180 - logic) if INVERT[joint] else logic
        angle += ZERO_OFFSET[joint]
        return max(0.0, min(180.0, angle))

    # ─── ส่งทันที ────────────────────────────────────────────────────────
    def set_joint(self, joint: str, logic: float):
        """ส่ง logic angle ไป servo ทันที (ไม่ smooth)"""
        servo_angle = self._to_servo(joint, logic)
        self.kit.servo[CHANNEL[joint]].angle = servo_angle

    # ─── เคลื่อนแบบ smooth ──────────────────────────────────────────────
    def move_smooth(self, target: dict, steps: int = 60, delay: float = 0.02,
                    settle: float = 0.3):
        """
        เคลื่อนทุก joint พร้อมกันจากตำแหน่งปัจจุบัน → target
        target  : {'J1': logic_angle, ...}
        settle  : รอหลัง step สุดท้าย (วิ) ให้ servo ถึงเป้าจริง
        """
        # อ่านตำแหน่งปัจจุบันจาก servo (แปลงกลับ)
        current = {}
        for joint in target:
            ch = CHANNEL[joint]
            servo_now = self.kit.servo[ch].angle or self._to_servo(joint, 90)
            # แปลง servo → logic (ย้อน INVERT + ZERO_OFFSET)
            logic_now = servo_now - ZERO_OFFSET[joint]
            if INVERT[joint]:
                logic_now = 180 - logic_now
            current[joint] = logic_now

        for step in range(1, steps + 1):
            t = step / steps
            for joint, goal in target.items():
                interp = current[joint] + (goal - current[joint]) * t
                self.set_joint(joint, interp)
            time.sleep(delay)

        if settle > 0:
            time.sleep(settle)  # รอให้ servo ถึงตำแหน่งจริงก่อนวัด

    # ─── ท่าสำเร็จรูป ────────────────────────────────────────────────────
    def move_to_home(self, steps: int = 60, delay: float = 0.02):
        self.move_smooth(HOME, steps=steps, delay=delay)
