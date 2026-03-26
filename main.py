# main.py — pipeline หลัก ValveVision-PiArm
#
# Flow:
#   scan pose → detect valve → ยื่นแขน → scan pose → loop

import math
import urllib.request
import numpy as np
import cv2

from servo_controller import ServoController
from ik_solver import solve_ik, solve_ik_clamped
from config import (CAM_X, CAM_Y, CAM_Z, CAM_TILT,
                    FOCAL_LENGTH, VALVE_REAL_MM, IMAGE_W, IMAGE_H)
from valve_detector import load_model, preprocess, postprocess, CONF_THRESH

CAMERA_URL = "http://localhost:8080/snapshot"

# โหลด model ครั้งเดียวตอนเริ่ม
print("โหลด ONNX model...")
_session, _inp_name, _out_name = load_model()
print("โหลด model สำเร็จ")


# ══════════════════════════════════════════════════════
#  PIXEL → XYZ
# ══════════════════════════════════════════════════════

def pixel_to_xyz(u, v, bbox_w_px):
    """แปลง pixel center + bbox width → (x, y, z) mm relative to J1"""
    tilt = math.radians(CAM_TILT)
    cx   = IMAGE_W / 2
    cy   = IMAGE_H / 2

    # depth จากขนาด valve
    depth = (FOCAL_LENGTH * VALVE_REAL_MM) / bbox_w_px

    # offset pixel จาก center image
    dx = (u - cx) * depth / FOCAL_LENGTH   # ซ้าย-ขวา
    dy = (v - cy) * depth / FOCAL_LENGTH   # บน-ลง (pixel y ลงล่าง)

    # camera space → world space (J1 origin)
    x = CAM_X + depth * math.cos(tilt) - dy * math.sin(tilt)
    y = CAM_Y + dx
    z = CAM_Z - depth * math.sin(tilt) - dy * math.cos(tilt)

    return round(x, 1), round(y, 1), round(z, 1)


# ══════════════════════════════════════════════════════
#  DETECTION
# ══════════════════════════════════════════════════════

def get_valve_position() -> tuple[float, float, float] | None:
    """
    ดึง snapshot จาก camera_preview → detect valve → คืน (x, y, z) mm
    คืน None ถ้าไม่เจอ
    """
    try:
        with urllib.request.urlopen(CAMERA_URL, timeout=3) as resp:
            data = np.frombuffer(resp.read(), dtype=np.uint8)
        img = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if img is None:
            return None
    except Exception as e:
        print(f"  [cam] เชื่อมต่อกล้องไม่ได้: {e}")
        return None

    orig_h, orig_w = img.shape[:2]
    blob, scale, pad_left, pad_top = preprocess(img)
    out  = _session.run([_out_name], {_inp_name: blob})
    dets = postprocess(out[0], orig_w, orig_h, scale, pad_left, pad_top)

    if not dets:
        return None

    # เลือก detection ที่ confidence สูงสุด
    best = max(dets, key=lambda d: d[4])
    x1, y1, x2, y2, conf, _ = best

    if conf < CONF_THRESH:
        return None

    u      = (x1 + x2) / 2
    v      = (y1 + y2) / 2
    bbox_w = x2 - x1

    return pixel_to_xyz(u, v, bbox_w)


# ══════════════════════════════════════════════════════
#  MAIN LOOP
# ══════════════════════════════════════════════════════

CONFIRM_FRAMES = 3   # เฟรมติดกันก่อนยื่นแขน / ก่อนกลับ scan pose

def main():
    arm = ServoController()

    print("ValveVision-PiArm เริ่มทำงาน")
    print("กด Ctrl+C เพื่อหยุด\n")

    arm.move_to_scan_pose()
    print("พร้อม — อยู่ที่ scan pose\n")

    hit_buf  = []   # เก็บ (x,y,z) เฟรมที่เจอ valve
    miss_cnt = 0    # นับเฟรมติดกันที่ไม่เจอ
    at_scan  = True # ตอนนี้แขนอยู่ที่ scan pose ไหม

    while True:
        pos = get_valve_position()

        if pos is not None:
            miss_cnt = 0
            hit_buf.append(pos)
            print(f"  [detect] เจอ valve {pos} — hit {len(hit_buf)}/{CONFIRM_FRAMES}")

            if len(hit_buf) >= CONFIRM_FRAMES:
                # เฉลี่ย 3 เฟรม กัน noise
                x = sum(p[0] for p in hit_buf) / len(hit_buf)
                y = sum(p[1] for p in hit_buf) / len(hit_buf)
                z = sum(p[2] for p in hit_buf) / len(hit_buf)
                hit_buf.clear()

                print(f"  [detect] confirm valve ที่ ({x:.0f}, {y:.0f}, {z:.0f}) mm")

                angles = solve_ik(x, y, z)
                if angles:
                    mode = "exact"
                else:
                    angles = solve_ik_clamped(x, y, z)
                    mode = "clamped"

                print(f"  [IK {mode}] J1={angles['J1']:.1f}° J2={angles['J2']:.1f}° "
                      f"J3={angles['J3']:.1f}° J4={angles['J4']:.1f}°")

                arm.move_smooth(angles)
                print("  [arm] ถึงเป้าแล้ว")
                at_scan = False

        else:
            hit_buf.clear()
            miss_cnt += 1
            print(f"  [detect] ไม่เจอ valve — miss {miss_cnt}/{CONFIRM_FRAMES}")

            if miss_cnt >= CONFIRM_FRAMES and not at_scan:
                arm.move_to_scan_pose()
                print("  [arm] กลับ scan pose\n")
                at_scan  = True
                miss_cnt = 0


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nหยุดการทำงาน")
