# main.py — pipeline หลัก ValveVision-PiArm
#
# Flow:
#   scan pose → detect valve → ยื่นแขน → scan pose → loop

from servo_controller import ServoController
from ik_solver import solve_ik, solve_ik_clamped


# ══════════════════════════════════════════════════════
#  DETECTION — swap ฟังก์ชันนี้เป็น real camera ทีหลัง
# ══════════════════════════════════════════════════════

def get_valve_position() -> tuple[float, float, float] | None:
    """
    คืนพิกัด (x, y, z) ของ valve ในหน่วย mm จากแกน J1
    คืน None ถ้า detect ไม่เจอ

    ตอนนี้: hardcode
    ทีหลัง: แทนด้วย output จากกล้อง
    """
    # ── hardcode ทดสอบ ──────────────────────────────
    x = 300.0
    y = 0.0
    z = 150.0
    # ────────────────────────────────────────────────
    return x, y, z


# ══════════════════════════════════════════════════════
#  MAIN LOOP
# ══════════════════════════════════════════════════════

def main():
    arm = ServoController()

    print("ValveVision-PiArm เริ่มทำงาน")
    print("กด Ctrl+C เพื่อหยุด\n")

    arm.move_to_scan_pose()
    print("พร้อม — อยู่ที่ scan pose\n")

    while True:
        input("กด Enter เพื่อ detect และยื่นแขน...")

        # ── 1. detect ──────────────────────────────
        result = get_valve_position()
        if result is None:
            print("  [detect] ไม่พบ valve — กลับ scan pose\n")
            arm.move_to_scan_pose()
            continue

        x, y, z = result
        print(f"  [detect] valve ที่ ({x:.0f}, {y:.0f}, {z:.0f}) mm")

        # ── 2. คำนวณ IK ────────────────────────────
        angles = solve_ik(x, y, z)
        if angles:
            mode = "exact"
        else:
            angles = solve_ik_clamped(x, y, z)
            mode = "clamped"

        print(f"  [IK {mode}] J1={angles['J1']:.1f}° J2={angles['J2']:.1f}° "
              f"J3={angles['J3']:.1f}° J4={angles['J4']:.1f}°")

        # ── 3. ยื่นแขน ─────────────────────────────
        arm.move_smooth(angles)
        print("  [arm] ถึงเป้าแล้ว")

        # ── 4. รอสักครู่ แล้วกลับ scan pose ────────
        input("  กด Enter เพื่อกลับ scan pose...")
        arm.move_to_scan_pose()
        print("  [arm] กลับ scan pose\n")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nหยุดการทำงาน")
