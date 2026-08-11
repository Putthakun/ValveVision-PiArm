# setup/calibrate_target_offset.py — หา TARGET_OFFSET_X/Y/Z จากการวัดจริงหลายจุด
#
# หลักการ:
#   1. กล้อง detect จุ๊บลม → ได้ค่า detected (x, y, z) ดิบ (ยังไม่บวก offset)
#   2. ผู้ใช้ jog แขนทีละ mm จนปลาย gripper แตะหัวจุ๊บพอดี → ได้ค่า actual
#   3. offset = actual - detected  → เก็บหลายจุด (5-8 นาฬิกา) แล้วเฉลี่ย
#
# ก่อนรัน:
#   - รัน camera_preview.py ค้างไว้ (สคริปต์นี้ดึงภาพผ่าน http://localhost:8080)
#   - stop valvevision service (sudo systemctl stop valvevision)
#   - วางล้อที่ตำแหน่งใช้งานจริง + ทำเครื่องหมายพื้นไว้
#
# รัน (ต้องรันจาก terminal จริง เพราะใช้การกดคีย์):
#   python setup/calibrate_target_offset.py

import math
import os
import re
import sys
import termios
import time
import tty
from datetime import date

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# บังคับใช้กล้องผ่าน camera_preview ก่อน import main (main อ่าน env ตอน import)
os.environ.setdefault("CAMERA_SOURCE", "http")
os.environ.setdefault("CAMERA_URL", "http://localhost:8080/snapshot")

import main as vision                      # ใช้ pipeline detect เดียวกับระบบจริง
from servo_controller import ServoController
from ik_solver import solve_ik, solve_ik_clamped
from config import TARGET_OFFSET_X, TARGET_OFFSET_Y, TARGET_OFFSET_Z

N_DETECT_FRAMES = 8      # เฟรมที่ใช้เฉลี่ยค่า detect ต่อจุด
APPROACH_PULL   = 60     # mm — เริ่ม jog จากจุดที่ถอยห่าง valve เท่านี้
CSV_PATH        = "results/offset_calibration.csv"
CONFIG_PATH     = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config.py")


def getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def detect_average() -> tuple[float, float, float] | None:
    """detect หลายเฟรมแล้วเฉลี่ย คืน (x, y, z) ดิบ หรือ None"""
    hits = []
    for i in range(N_DETECT_FRAMES * 3):
        pos, err = vision.get_valve_position()
        if err is not None:
            print(f"  [cam] {err} — camera_preview รันอยู่หรือเปล่า?")
            return None
        if pos is not None:
            hits.append(pos)
            print(f"  detect {len(hits)}/{N_DETECT_FRAMES}: ({pos[0]:.0f}, {pos[1]:.0f}, {pos[2]:.0f})")
            if len(hits) >= N_DETECT_FRAMES:
                break
        time.sleep(0.15)

    if len(hits) < N_DETECT_FRAMES:
        print(f"  เจอแค่ {len(hits)} เฟรม — ขยับ valve ให้กล้องเห็นชัดๆ แล้วลองใหม่")
        return None

    xs = [p[0] for p in hits]
    ys = [p[1] for p in hits]
    zs = [p[2] for p in hits]
    n  = len(hits)
    x, y, z = sum(xs) / n, sum(ys) / n, sum(zs) / n
    sx = max(xs) - min(xs)
    sy = max(ys) - min(ys)
    sz = max(zs) - min(zs)
    print(f"  เฉลี่ย: ({x:.1f}, {y:.1f}, {z:.1f})  spread: ({sx:.0f}, {sy:.0f}, {sz:.0f}) mm")
    if sx > 40:
        print(f"  ⚠️ ค่า x แกว่ง {sx:.0f}mm (depth จาก bbox ไม่นิ่ง) — offset x ที่ได้จะหยาบ")
    return x, y, z


def jog_to_touch(arm: ServoController, x: float, y: float, z: float) -> tuple[float, float, float] | None:
    """ให้ผู้ใช้ jog แขนจนแตะจุ๊บ คืนพิกัดสุดท้าย หรือ None ถ้ายกเลิก"""
    step = 2.0
    print(f"""
  ── jog mode ──────────────────────────────────
   w/s : x ยื่นออก / ถอยเข้า      a/d : y ซ้าย / ขวา
   r/f : z ขึ้น / ลง              [/]/\\ : step 1 / 5 / 10 mm
   t   : ปลายแตะจุ๊บแล้ว → บันทึกจุดนี้
   h   : กลับ scan pose           q : ยกเลิกจุดนี้
  ──────────────────────────────────────────────""")
    while True:
        print(f"\r  ({x:7.1f}, {y:7.1f}, {z:7.1f}) step={step:.0f}mm   ", end="", flush=True)
        c = getch()
        nx, ny, nz = x, y, z
        if c == "w":   nx += step
        elif c == "s": nx -= step
        elif c == "a": ny -= step
        elif c == "d": ny += step
        elif c == "r": nz += step
        elif c == "f": nz -= step
        elif c == "[":  step = 1.0; continue
        elif c == "]":  step = 5.0; continue
        elif c == "\\": step = 10.0; continue
        elif c == "t":
            print()
            return x, y, z
        elif c == "h":
            arm.move_to_scan_pose()
            continue
        elif c in ("q", "\x03"):
            print()
            return None
        else:
            continue

        angles = solve_ik(nx, ny, nz)
        if angles is None:
            print(f"\n  ✗ ({nx:.0f}, {ny:.0f}, {nz:.0f}) เกิน workspace")
            continue
        arm.move_smooth(angles, steps=8, delay=0.01, settle=0.05)
        x, y, z = nx, ny, nz


def write_config(ox: int, oy: int, oz: int, n: int) -> None:
    with open(CONFIG_PATH) as f:
        src = f.read()
    tag = f"# calibrated {date.today().isoformat()} (n={n})"
    for name, val in (("TARGET_OFFSET_X", ox), ("TARGET_OFFSET_Y", oy), ("TARGET_OFFSET_Z", oz)):
        src = re.sub(rf"^{name}\s*=.*$", f"{name} = {val:>4}   {tag}", src, count=1, flags=re.M)
    with open(CONFIG_PATH, "w") as f:
        f.write(src)
    print(f"เขียน {CONFIG_PATH} แล้ว")


def main_cal():
    print("=== Calibrate TARGET_OFFSET ===\n")
    pos, err = vision.get_valve_position()
    if err is not None:
        print(f"ต่อกล้องไม่ได้: {err}")
        print("รัน python camera_preview.py ค้างไว้ก่อน แล้วค่อยรันสคริปต์นี้")
        sys.exit(1)

    arm = ServoController()
    print("[arm] warm-up: ไป scan pose...")
    arm.move_to_scan_pose()
    time.sleep(0.5)

    records = []   # (detected, actual, offset)
    while True:
        print(f"\n── จุดที่ {len(records) + 1} ─────────────────────────────")
        print("หมุนล้อให้จุ๊บอยู่ตำแหน่งที่ต้องการ (5-8 นาฬิกา) แล้วกด Enter")
        print("(พิมพ์ q แล้ว Enter เพื่อจบและสรุปผล)")
        if input("> ").strip().lower() == "q":
            break

        det = detect_average()
        if det is None:
            continue
        dx, dy, dz = det

        # เริ่มจากจุด detect + offset เดิม แล้วถอยออก APPROACH_PULL
        gx = dx + TARGET_OFFSET_X
        gy = dy + TARGET_OFFSET_Y
        gz = dz + TARGET_OFFSET_Z
        r  = math.hypot(gx, gy)
        sc = max(0.0, r - APPROACH_PULL) / r if r > 1e-6 else 0.0
        ax, ay = gx * sc, gy * sc
        print(f"  [arm] ไปจุดเริ่ม ({ax:.0f}, {ay:.0f}, {gz:.0f}) — แล้ว jog เข้าหาจุ๊บ")
        arm.move_smooth(solve_ik_clamped(ax, ay, gz))

        actual = jog_to_touch(arm, ax, ay, gz)
        arm.move_to_scan_pose()
        if actual is None:
            print("  ยกเลิกจุดนี้")
            continue

        off = (actual[0] - dx, actual[1] - dy, actual[2] - dz)
        records.append((det, actual, off))
        print(f"  ✓ detected ({dx:.0f}, {dy:.0f}, {dz:.0f}) → actual "
              f"({actual[0]:.0f}, {actual[1]:.0f}, {actual[2]:.0f})"
              f"  offset ({off[0]:+.0f}, {off[1]:+.0f}, {off[2]:+.0f})")

    if not records:
        print("\nไม่มีข้อมูล — จบ")
        return

    n  = len(records)
    ox = sum(r[2][0] for r in records) / n
    oy = sum(r[2][1] for r in records) / n
    oz = sum(r[2][2] for r in records) / n

    print(f"\n=== สรุปจาก {n} จุด ===")
    print(f"{'จุด':>4} {'detected':>22} {'actual':>22} {'offset':>18}")
    for i, (d, a, o) in enumerate(records, 1):
        print(f"{i:>4} ({d[0]:6.0f},{d[1]:6.0f},{d[2]:6.0f})"
              f"    ({a[0]:6.0f},{a[1]:6.0f},{a[2]:6.0f})"
              f"    ({o[0]:+5.0f},{o[1]:+5.0f},{o[2]:+5.0f})")
    spread_x = max(r[2][0] for r in records) - min(r[2][0] for r in records)
    spread_y = max(r[2][1] for r in records) - min(r[2][1] for r in records)
    spread_z = max(r[2][2] for r in records) - min(r[2][2] for r in records)
    print(f"\nofffset เฉลี่ย: X={ox:+.0f}  Y={oy:+.0f}  Z={oz:+.0f}")
    print(f"spread ระหว่างจุด: X={spread_x:.0f}  Y={spread_y:.0f}  Z={spread_z:.0f} mm")
    if max(spread_x, spread_y, spread_z) > 25:
        print("⚠️ spread สูง — error ไม่คงที่ตามตำแหน่ง อาจต้อง calibrate กล้อง (CAM_X/Z/TILT) เพิ่ม")

    os.makedirs("results", exist_ok=True)
    new_file = not os.path.exists(CSV_PATH)
    with open(CSV_PATH, "a") as f:
        if new_file:
            f.write("date,det_x,det_y,det_z,act_x,act_y,act_z,off_x,off_y,off_z\n")
        for d, a, o in records:
            f.write(f"{date.today().isoformat()},{d[0]:.1f},{d[1]:.1f},{d[2]:.1f},"
                    f"{a[0]:.1f},{a[1]:.1f},{a[2]:.1f},{o[0]:.1f},{o[1]:.1f},{o[2]:.1f}\n")
    print(f"บันทึกข้อมูลดิบที่ {CSV_PATH}")

    ans = input(f"\nเขียนค่าใหม่ลง config.py เลยไหม? (X={ox:+.0f} Y={oy:+.0f} Z={oz:+.0f}) [y/N]: ")
    if ans.strip().lower() == "y":
        write_config(round(ox), round(oy), round(oz), n)
        print("เสร็จ — ทดสอบด้วย: sudo systemctl start valvevision")
    else:
        print("ไม่เขียน — แก้เองใน config.py ได้ตามค่าด้านบน")


if __name__ == "__main__":
    try:
        main_cal()
    except KeyboardInterrupt:
        print("\nยกเลิก")
