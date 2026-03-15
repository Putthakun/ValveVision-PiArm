# ik_solver.py
import math
from config import L1, L2, L3, L4, LIMITS


def solve_ik(x: float, y: float, z: float, gripper_pitch: float = 0) -> dict | None:
    """
    แปลงพิกัด (x, y, z) → องศาทุก joint

    x, y, z       : พิกัดปลาย gripper (mm) จากแกน J1
    gripper_pitch : มุม gripper จากแนวนอน (องศา)
        0   = แนวนอน (default)
        90  = ชี้ลง

    คืน dict {'J1','J2','J3','J4','J5','J6'} หรือ None
    """
    try:
        # ── J1: หมุนฐานให้หันหน้าหา target ────────────────────────────
        j1 = 90 + math.degrees(math.atan2(y, x))

        r     = math.sqrt(x**2 + y**2)  # ระยะแนวนอนจากแกน J1
        z_adj = z - L1                   # ความสูงจาก J2

        # ── หาตำแหน่งข้อมือ (J4) โดยถอย L4 ออกจากปลาย ─────────────────
        # alpha4 = มุมของ L4 จากแนวดิ่ง (0=ชี้ขึ้น, π/2=แนวนอน, π=ชี้ลง)
        beta   = math.radians(gripper_pitch)
        alpha4 = math.pi / 2 + beta

        r_w = r     - L4 * math.cos(beta)   # = r - L4*sin(alpha4)
        z_w = z_adj + L4 * math.sin(beta)   # ถ้าแนวนอน: z_w = z_adj

        # ── 2-link IK: J2, J3 → ข้อมือ ─────────────────────────────────
        d = math.sqrt(r_w**2 + z_w**2)

        if d > L2 + L3:
            print(f"[IK] นอก reach: {d:.1f} > {L2+L3:.1f}")
            return None
        if d < abs(L2 - L3):
            print(f"[IK] ใกล้เกิน: {d:.1f} < {abs(L2-L3):.1f}")
            return None

        cos_a3 = (d**2 - L2**2 - L3**2) / (2 * L2 * L3)
        cos_a3 = max(-1.0, min(1.0, cos_a3))
        a3_rel = math.acos(cos_a3)          # มุมพับที่ J3 (relative)

        phi = math.atan2(r_w, z_w)          # มุมจากแนวดิ่งไปยัง target
        psi = math.atan2(L3 * math.sin(a3_rel), L2 + L3 * math.cos(a3_rel))
        a2  = phi - psi                      # มุมของ L2 จากแนวดิ่ง

        j2 = math.degrees(math.pi / 2 + a2)
        j3 = math.degrees(a3_rel) + 90

        # ── J4: ชดเชยให้ gripper อยู่ที่ gripper_pitch ─────────────────
        alpha3 = a2 + a3_rel                 # มุมของ L3 จากแนวดิ่ง
        j4     = math.degrees(math.pi / 2 + alpha4 - alpha3)

        j5, j6 = 90, 90

        angles = {'J1': j1, 'J2': j2, 'J3': j3,
                  'J4': j4, 'J5': j5, 'J6': j6}

        for joint, angle in angles.items():
            lo, hi = LIMITS[joint]
            if not (lo <= angle <= hi):
                print(f"[IK] {joint}={angle:.1f}° เกิน [{lo},{hi}]")
                return None

        return angles

    except Exception as e:
        print(f"[IK] error: {e}")
        return None


def solve_ik_clamped(x: float, y: float, z: float, gripper_pitch: float = 0) -> dict:
    """
    เหมือน solve_ik แต่ถ้า target อยู่นอก workspace จะยื่นสุดขีดในทิศทางนั้นแทน
    ไม่มีทาง return None — J1 ถูกทิศเสมอ ระยะถึงแค่ไหนเอาแค่นั้น
    """
    # ── J1: หมุนฐานถูกทิศเสมอ ────────────────────────────────────────
    j1 = 90 + math.degrees(math.atan2(y, x))

    r     = math.sqrt(x**2 + y**2)
    z_adj = z - L1

    beta   = math.radians(gripper_pitch)
    alpha4 = math.pi / 2 + beta

    r_w = r     - L4 * math.cos(beta)
    z_w = z_adj + L4 * math.sin(beta)

    # ── clamp ระยะข้อมือให้อยู่ใน [d_min, d_max] ─────────────────────
    d     = math.sqrt(r_w**2 + z_w**2)
    d_min = math.sqrt(L2**2 + L3**2)   # ~144mm — ข้อจำกัด J3 ≤ 180°
    d_max = L2 + L3                     # 185mm  — แขนเหยียดสุด

    if d > 1e-6:
        if d > d_max:
            r_w, z_w = r_w * d_max / d, z_w * d_max / d
        elif d < d_min:
            r_w, z_w = r_w * d_min / d, z_w * d_min / d
    else:
        # target ตรงแกน J1 พอดี — ยื่นตรงขึ้นบน
        r_w, z_w = 0.0, d_min

    d_c = math.sqrt(r_w**2 + z_w**2)

    # ── 2-link IK (guaranteed solution หลัง clamp) ───────────────────
    cos_a3 = (d_c**2 - L2**2 - L3**2) / (2 * L2 * L3)
    cos_a3 = max(-1.0, min(1.0, cos_a3))
    a3_rel = math.acos(cos_a3)

    phi = math.atan2(r_w, z_w)
    psi = math.atan2(L3 * math.sin(a3_rel), L2 + L3 * math.cos(a3_rel))
    a2  = phi - psi

    j2 = math.degrees(math.pi / 2 + a2)
    j3 = math.degrees(a3_rel) + 90

    alpha3 = a2 + a3_rel
    j4     = math.degrees(math.pi / 2 + alpha4 - alpha3)

    # ── clamp J2, J4 ถึงขีดจำกัด servo ──────────────────────────────
    for joint, val in [('J2', j2), ('J4', j4)]:
        lo, hi = LIMITS[joint]
        if not (lo <= val <= hi):
            pass  # clamp ด้านล่าง

    j2 = max(LIMITS['J2'][0], min(LIMITS['J2'][1], j2))
    j4 = max(LIMITS['J4'][0], min(LIMITS['J4'][1], j4))

    return {'J1': j1, 'J2': j2, 'J3': j3, 'J4': j4, 'J5': 90, 'J6': 90}


def fk(j2_deg: float, j3_deg: float, j4_deg: float) -> tuple[float, float]:
    """FK ย้อนกลับ: joint angles → (r, z) ของปลาย gripper"""
    a2     = math.radians(j2_deg) - math.pi / 2
    a3_rel = math.radians(j3_deg) - math.pi / 2
    a4_rel = math.radians(j4_deg) - math.pi / 2
    a3     = a2 + a3_rel
    a4     = a3 + a4_rel

    r = (L2 * math.sin(a2) +
         L3 * math.sin(a3) +
         L4 * math.sin(a4))
    z = (L1 +
         L2 * math.cos(a2) +
         L3 * math.cos(a3) +
         L4 * math.cos(a4))

    return round(r, 1), round(z, 1)


if __name__ == '__main__':
    # targets ปกติ + targets นอก workspace เพื่อทดสอบ clamped
    targets = [
        (330,   0, 100),   # ✅ ใน workspace
        (300,   0, 150),   # ✅ ใน workspace
        (250,   0, 100),   # ❌ ใกล้เกิน
        (200,   0, 500),   # ❌ สูงเกิน (ล้อรถใหญ่)
        (400,   0, 150),   # ❌ ไกลเกิน
        (280,  80, 130),   # ✅ ใน workspace มี y
        (280, -80, 600),   # ❌ นอก workspace มี y
    ]

    print(f"\n{'─'*80}")
    print(f"{'Target':>18} | {'mode':>8} | {'J1':>6} {'J2':>6} {'J3':>6} {'J4':>6}")
    print(f"{'─'*80}")

    for tx, ty, tz in targets:
        strict  = solve_ik(tx, ty, tz)
        clamped = solve_ik_clamped(tx, ty, tz)

        tag = "✅ exact " if strict else "⚠️ clamp"
        r = strict if strict else clamped
        print(f"({tx:4},{ty:4},{tz:4}) | {tag} | "
              f"{r['J1']:6.1f} {r['J2']:6.1f} {r['J3']:6.1f} {r['J4']:6.1f}")
