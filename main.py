# main.py — pipeline หลัก ValveVision-PiArm
#
# Flow:
#   scan pose → detect valve → ยื่นแขน → scan pose → loop

import math
import os
import sys
import time
import urllib.request
import numpy as np
import cv2

from servo_controller import ServoController
from ik_solver import solve_ik, solve_ik_clamped
from config import (CAM_X, CAM_Y, CAM_Z, CAM_TILT,
                    FOCAL_LENGTH, VALVE_REAL_MM, IMAGE_W, IMAGE_H,
                    TARGET_OFFSET_X, TARGET_OFFSET_Y, TARGET_OFFSET_Z,
                    Z_DROOP_COMP)
from valve_detector import load_model, preprocess, postprocess, CONF_THRESH, CLASS_NAMES

try:
    from picamera2 import Picamera2
except Exception:
    Picamera2 = None

TARGET_CLASS = CLASS_NAMES.index("joob")   # class_id ของจุ๊บลม

CAMERA_SOURCE = os.getenv("CAMERA_SOURCE", "auto").lower()  # auto | picamera2 | local | http
CAMERA_DEVICE = os.getenv("CAMERA_DEVICE", "0")
CAMERA_URL = os.getenv("CAMERA_URL", "http://localhost:8080/snapshot")
CAMERA_TIMEOUT_SEC = float(os.getenv("CAMERA_TIMEOUT_SEC", "6"))
LOOP_INTERVAL_SEC = 0.10
CAM_ERROR_LOG_EVERY = 10
CAM_ERROR_BACKOFF_SEC = 0.50
CAMERA_STARTUP_WAIT_SEC = float(os.getenv("CAMERA_STARTUP_WAIT_SEC", "20"))
CAMERA_REQUIRE_READY = os.getenv("CAMERA_REQUIRE_READY", "1") == "1"
CAMERA_READ_RETRIES = int(os.getenv("CAMERA_READ_RETRIES", "3"))
CAMERA_WARMUP_FRAMES = int(os.getenv("CAMERA_WARMUP_FRAMES", "3"))
CAMERA_FRAME_WIDTH = int(os.getenv("CAMERA_FRAME_WIDTH", "1280"))
CAMERA_FRAME_HEIGHT = int(os.getenv("CAMERA_FRAME_HEIGHT", "720"))
CAMERA_FPS = int(os.getenv("CAMERA_FPS", "30"))

_cap: cv2.VideoCapture | None = None
_picam = None
_active_source: str | None = None
_picamera2_locked = False

# โหลด model ครั้งเดียวตอนเริ่ม
print("โหลด ONNX model...")
_session, _inp_name, _out_name = load_model()
print("โหลด model สำเร็จ")
print(f"CAMERA_SOURCE={CAMERA_SOURCE}")
if CAMERA_SOURCE == "http":
    print(f"CAMERA_URL={CAMERA_URL}")
elif CAMERA_SOURCE == "picamera2":
    print("CAMERA_BACKEND=picamera2")
elif CAMERA_SOURCE == "auto":
    print(f"CAMERA_AUTO_FALLBACK=http({CAMERA_URL})")
else:
    print(f"CAMERA_DEVICE={CAMERA_DEVICE}")


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

def get_valve_position() -> tuple[tuple[float, float, float] | None, str | None]:
    """
    ดึงภาพจากกล้อง (local/http) → detect valve
    คืน (ตำแหน่ง, ข้อผิดพลาดกล้อง)
    - (x, y, z), None   : detect สำเร็จ
    - None, None        : กล้องปกติ แต่ไม่เจอวัตถุ
    - None, "..."       : กล้องมีปัญหา (timeout/เชื่อมต่อไม่ได้)
    """
    img, cam_err = _fetch_snapshot()
    if cam_err is not None:
        return None, cam_err

    orig_h, orig_w = img.shape[:2]
    blob, scale, pad_left, pad_top = preprocess(img)
    out  = _session.run([_out_name], {_inp_name: blob})
    dets = postprocess(out[0], orig_w, orig_h, scale, pad_left, pad_top)

    if not dets:
        return None, None

    # กรองเฉพาะ class จุ๊บลม (joob) แล้วเลือก confidence สูงสุด
    joob_dets = [d for d in dets if d[5] == TARGET_CLASS]
    if not joob_dets:
        return None, None

    best = max(joob_dets, key=lambda d: d[4])
    x1, y1, x2, y2, conf, _ = best

    if conf < CONF_THRESH:
        return None, None

    u      = (x1 + x2) / 2
    v      = (y1 + y2) / 2
    bbox_w = x2 - x1

    return pixel_to_xyz(u, v, bbox_w), None


def _detect_joob_raw() -> tuple[float, float, float, float] | None:
    """
    ดึงภาพ 1 เฟรมแล้ว detect จุ๊บลม
    คืน (u, v, bbox_w, conf) ในหน่วย pixel หรือ None ถ้าไม่เจอ
    ใช้สำหรับ visual servo loop ที่ต้องการพิกัด pixel โดยตรง
    """
    img, cam_err = _fetch_snapshot()
    if cam_err is not None or img is None:
        return None
    orig_h, orig_w = img.shape[:2]
    blob, scale, pad_left, pad_top = preprocess(img)
    out  = _session.run([_out_name], {_inp_name: blob})
    dets = postprocess(out[0], orig_w, orig_h, scale, pad_left, pad_top)
    joob = [d for d in dets if d[5] == TARGET_CLASS and d[4] >= CONF_THRESH]
    if not joob:
        return None
    x1, y1, x2, y2, conf, _ = max(joob, key=lambda d: d[4])
    return (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, conf


def find_max_reach(x: float, y: float, z: float) -> tuple[float, float, float]:
    """
    Binary search หา r สูงสุดตาม direction (x, y) ที่ IK ยังทำได้ที่ z นั้น
    z_commanded รวม droop compensation ด้วย — ผลลัพธ์คือตำแหน่งจริงที่ไกลสุด
    คืน (x_max, y_max, z_comp) หรือ (x, y, z) ถ้าทั้งหมดเกิน workspace
    """
    import io, contextlib

    r_total = math.sqrt(x**2 + y**2)
    if r_total < 1e-6:
        return x, y, z
    cos_t = x / r_total
    sin_t = y / r_total

    # z < 185mm → ring workspace (r เล็กๆ J3 เกิน 180°) → start near target
    # z ≥ 185mm → r=0 reachable → start from 0
    r_lo = max(0.0, r_total - 100.0) if z < 185.0 else 0.0
    r_hi = 420.0
    best: tuple[float, float, float] | None = None

    sink = io.StringIO()
    for _ in range(25):   # precision < 420 / 2^25 < 0.02 mm
        r = (r_lo + r_hi) / 2
        xr = r * cos_t
        yr = r * sin_t
        zr = z + Z_DROOP_COMP * r
        with contextlib.redirect_stdout(sink):
            ok = solve_ik(xr, yr, zr)
        if ok is not None:
            best = (xr, yr, zr)
            r_lo = r
        else:
            r_hi = r

    if best is None:
        print(f"  [reach] z={z:.0f}mm เกิน workspace ทุก r — ใช้ clamped IK แทน")
        return x, y, z
    print(f"  [reach] max r={r_lo:.1f}mm → ({best[0]:.0f}, {best[1]:.0f}, {best[2]:.0f}) mm")
    return best


def visual_servo(arm, x: float, y: float, z: float) -> tuple[float, float]:
    """
    ปรับเฉพาะ y, z (ซ้าย-ขวา / สูง-ต่ำ) — x คงที่เพื่อป้องกันแขนชนล้อ
    คืน (y_new, z_new)
    """
    cx, cy = IMAGE_W / 2, IMAGE_H / 2
    prev_err = float('inf')

    for step in range(SERVO_MAX_STEPS):
        raw = _detect_joob_raw()
        if raw is None:
            print(f"  [servo] step {step+1}/{SERVO_MAX_STEPS}: ไม่เห็น valve — หยุด")
            break

        u, v, bbox_w, conf = raw
        if conf < 0.40:
            print(f"  [servo] step {step+1}/{SERVO_MAX_STEPS}: conf={conf:.2f} ต่ำเกิน — หยุด")
            break

        err = math.sqrt((u - cx) ** 2 + (v - cy) ** 2)
        print(f"  [servo] step {step+1}/{SERVO_MAX_STEPS}: "
              f"pixel_err={err:.1f}px  conf={conf:.2f}  "
              f"offset=({u-cx:+.0f}, {v-cy:+.0f})px")

        if err < SERVO_PIX_THRESH:
            print(f"  [servo] converged ({err:.1f}px < {SERVO_PIX_THRESH}px)")
            break

        if prev_err - err < 3.0:
            print(f"  [servo] หยุด — error ไม่ลดลง ({prev_err:.1f} → {err:.1f}px)")
            break

        prev_err = err

        # คำนวณ y, z จริงของ valve จากกล้อง (absolute — ไม่ใช่ relative)
        # กล้องอยู่กับที่ → pixel_to_xyz ให้ค่า world y, z ของ valve โดยตรง
        _, y, z = pixel_to_xyz(u, v, bbox_w)
        y += TARGET_OFFSET_Y
        z += TARGET_OFFSET_Z
        arm.move_smooth(solve_ik_clamped(x, y, z),
                        steps=30, delay=0.015, settle=0.2)

    return y, z


def _fetch_snapshot() -> tuple[np.ndarray | None, str | None]:
    """ดึงภาพ 1 เฟรมจากกล้อง"""
    global _active_source

    if CAMERA_SOURCE == "auto":
        if _active_source is None:
            # ลอง picamera2 ก่อน (เหมาะกับ Pi Camera)
            if not _picamera2_locked:
                frame, err = _fetch_picamera2_frame()
                if err is None:
                    _active_source = "picamera2"
                    print("[cam] active source = picamera2")
                    return frame, None
            else:
                err = "picamera2_locked_by_other_process"

            # ถ้าไม่สำเร็จ fallback ไป http เพื่อใช้งานร่วมกับ camera_preview ได้
            frame, err_http = _fetch_http_snapshot()
            if err_http is None:
                _active_source = "http"
                print("[cam] active source = http (fallback from picamera2)")
                return frame, None

            return None, f"auto_failed picamera2={err} http={err_http}"

        if _active_source == "picamera2":
            return _fetch_picamera2_frame()
        if _active_source == "http":
            return _fetch_http_snapshot()
        if _active_source == "local":
            return _fetch_local_frame()
        return None, f"auto_unknown_active_source:{_active_source}"

    if CAMERA_SOURCE == "http":
        return _fetch_http_snapshot()

    if CAMERA_SOURCE == "picamera2":
        return _fetch_picamera2_frame()

    if CAMERA_SOURCE == "local":
        return _fetch_local_frame()

    return None, f"unsupported_camera_source:{CAMERA_SOURCE}"


def _fetch_http_snapshot() -> tuple[np.ndarray | None, str | None]:
    try:
        with urllib.request.urlopen(CAMERA_URL, timeout=CAMERA_TIMEOUT_SEC) as resp:
            data = np.frombuffer(resp.read(), dtype=np.uint8)
        img = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if img is None:
            return None, "decode_failed"
        return img, None
    except Exception as e:
        return None, str(e)


def _camera_device_value() -> int | str:
    return int(CAMERA_DEVICE) if CAMERA_DEVICE.isdigit() else CAMERA_DEVICE


def _open_local_camera() -> tuple[cv2.VideoCapture | None, str | None]:
    dev = _camera_device_value()
    if isinstance(dev, int):
        dev_path = f"/dev/video{dev}"
        if not os.path.exists(dev_path):
            return None, f"device_not_found:{dev_path}"

    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(dev)
        if not cap.isOpened():
            return None, f"open_failed device={dev}"

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    for _ in range(max(CAMERA_WARMUP_FRAMES, 0)):
        cap.grab()

    return cap, None


def _fetch_local_frame() -> tuple[np.ndarray | None, str | None]:
    global _cap

    if _cap is None:
        _cap, err = _open_local_camera()
        if err is not None:
            return None, err

    for _ in range(max(CAMERA_READ_RETRIES, 1)):
        ok, frame = _cap.read()
        if ok and frame is not None and frame.size > 0:
            return frame, None
        time.sleep(0.03)

    _cap.release()
    _cap = None
    return None, "read_failed"


def _open_picamera2() -> tuple[object | None, str | None]:
    if Picamera2 is None:
        return None, "picamera2_not_installed"

    try:
        picam = Picamera2()
        config = picam.create_video_configuration(
            main={"size": (CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT)},
            controls={"FrameRate": CAMERA_FPS},
        )
        picam.configure(config)
        picam.start()
        # ให้ camera pipeline อุ่นก่อนอ่านภาพจริง
        time.sleep(0.2)
        return picam, None
    except Exception as e:
        return None, f"picamera2_open_failed:{e}"


def _fetch_picamera2_frame() -> tuple[np.ndarray | None, str | None]:
    global _picam
    global _picamera2_locked

    if _picam is None:
        _picam, err = _open_picamera2()
        if err is not None:
            if "Pipeline handler in use by another process" in err or "__init__ sequence did not complete" in err:
                _picamera2_locked = True
            return None, err

    try:
        frame = _picam.capture_array()
        if frame is None or frame.size == 0:
            return None, "picamera2_empty_frame"

        # picamera2 ให้ RGB โดยปกติ แต่ OpenCV pipeline ใช้ BGR
        if len(frame.shape) == 3 and frame.shape[2] == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame, None
    except Exception as e:
        try:
            _picam.stop()
            _picam.close()
        except Exception:
            pass
        _picam = None
        return None, f"picamera2_read_failed:{e}"


def _close_camera():
    global _cap
    global _picam
    global _active_source
    global _picamera2_locked

    if _cap is not None:
        _cap.release()
        _cap = None
    if _picam is not None:
        try:
            _picam.stop()
            _picam.close()
        except Exception:
            pass
        _picam = None
    _active_source = None
    _picamera2_locked = False


def wait_for_camera_ready() -> bool:
    """รอกล้องพร้อมใช้งานตอน startup เพื่อหลีกเลี่ยง error loop ตั้งแต่เริ่ม"""
    deadline = time.monotonic() + CAMERA_STARTUP_WAIT_SEC
    attempt = 0
    last_err = "unknown"

    while time.monotonic() < deadline:
        attempt += 1
        _, err = _fetch_snapshot()
        if err is None:
            print(f"[cam] กล้องพร้อมใช้งาน (attempt {attempt})")
            return True

        last_err = err
        if attempt == 1 or attempt % CAM_ERROR_LOG_EVERY == 0:
            print(f"[cam] รอกล้องพร้อม (attempt {attempt}): {err}")
        time.sleep(CAM_ERROR_BACKOFF_SEC)

    print(f"[cam] ไม่พร้อมหลังรอ {CAMERA_STARTUP_WAIT_SEC:.0f}s, last_error={last_err}")
    if CAMERA_SOURCE == "http":
        print("[cam] ตรวจ service stream และ CAMERA_URL ให้ถูกต้องก่อนเริ่มระบบ")
    elif CAMERA_SOURCE == "picamera2":
        print("[cam] ตรวจว่าเปิด camera โดย process อื่นอยู่หรือไม่ และสิทธิ์ libcamera ถูกต้อง")
    elif CAMERA_SOURCE == "auto":
        print("[cam] auto mode: ถ้า picamera2 ติด process อื่นอยู่ จะ fallback ไป http ให้เอง")
        print("[cam] ต้องมีอย่างใดอย่างหนึ่ง:")
        print("[cam] 1) ปิด process ที่ล็อกกล้อง เพื่อให้ main.py ใช้ picamera2 ได้")
        print("[cam] 2) รัน camera_preview.py แล้วให้ /snapshot ตอบได้ เพื่อใช้ทาง http")
    else:
        print("[cam] ตรวจ CAMERA_DEVICE, สายกล้อง และสิทธิ์เข้าถึง /dev/video*")
    return False


# ══════════════════════════════════════════════════════
#  MAIN LOOP
# ══════════════════════════════════════════════════════

CONFIRM_FRAMES   = 3     # เฟรมติดกันก่อนยื่นแขน / ก่อนกลับ scan pose
SERVO_MAX_STEPS  = 5     # จำนวน iteration สูงสุดของ visual servo
SERVO_PIX_THRESH = 10.0  # pixel error (px) ที่ถือว่า converge แล้ว
APPROACH_MM      = 80    # หยุดก่อนถึง valve (mm) เพื่อไม่ชนล้อ ปรับได้
HOLD_AT_TARGET_SEC = 7   # ค้างแขนไว้ที่เป้าหมายกี่วิ ก่อนเริ่มเช็คว่า valve หายไหม

def main():
    arm = ServoController()

    print("ValveVision-PiArm เริ่มทำงาน")
    print("กด Ctrl+C เพื่อหยุด\n")

    if not wait_for_camera_ready() and CAMERA_REQUIRE_READY:
        print("[system] จบการทำงานเพราะกล้องไม่พร้อม (CAMERA_REQUIRE_READY=1)")
        sys.exit(1)

    arm.move_to_scan_pose()
    print("พร้อม — อยู่ที่ scan pose\n")

    hit_buf     = []    # เก็บ (x,y,z) เฟรมที่เจอ valve
    miss_cnt    = 0     # นับเฟรมติดกันที่ไม่เจอ (ใช้ตอน at_scan=False)
    cam_err_cnt = 0     # นับกล้องล้มเหลวติดกัน
    at_scan     = True  # True = รอ detect / False = แขนอยู่ที่ target แล้ว (ล็อค)

    while True:
        pos, cam_err = get_valve_position()

        if cam_err is not None:
            cam_err_cnt += 1
            if cam_err_cnt == 1 or cam_err_cnt % CAM_ERROR_LOG_EVERY == 0:
                print(f"  [cam] เชื่อมต่อกล้องไม่ได้ ({cam_err_cnt}): {cam_err}")
            time.sleep(CAM_ERROR_BACKOFF_SEC)
            continue
        else:
            cam_err_cnt = 0

        if at_scan:
            # ── รอ detect ──────────────────────────────────────────────
            if pos is not None:
                hit_buf.append(pos)
                print(f"  [detect] เจอ valve {pos} — hit {len(hit_buf)}/{CONFIRM_FRAMES}")

                if len(hit_buf) >= CONFIRM_FRAMES:
                    x = sum(p[0] for p in hit_buf) / len(hit_buf)
                    y = sum(p[1] for p in hit_buf) / len(hit_buf)
                    z = sum(p[2] for p in hit_buf) / len(hit_buf)
                    hit_buf.clear()

                    print(f"  [detect] confirm valve ที่ ({x:.0f}, {y:.0f}, {z:.0f}) mm")
                    x += TARGET_OFFSET_X
                    y += TARGET_OFFSET_Y
                    z += TARGET_OFFSET_Z
                    if any([TARGET_OFFSET_X, TARGET_OFFSET_Y, TARGET_OFFSET_Z]):
                        print(f"  [offset] → ({x:.0f}, {y:.0f}, {z:.0f}) mm")

                    # Phase 1: approach — หยุดก่อนล้อ APPROACH_MM
                    r = math.sqrt(x**2 + y**2)
                    scale = max(0.0, r - APPROACH_MM) / r if r > 1e-6 else 0.0
                    x_app, y_app = x * scale, y * scale
                    y_valve, z_valve = y, z   # เก็บค่าจริงไว้ก่อน — fallback เมื่อ servo ไม่ได้ทำงาน
                    angles_app = solve_ik_clamped(x_app, y_app, z)
                    print(f"  [arm] Phase1 approach ({x_app:.0f}, {y_app:.0f}, {z:.0f}) mm")
                    arm.move_smooth(angles_app)

                    # Phase 2: visual servo — ปรับ y, z เท่านั้น (x คงที่)
                    # ส่ง y_valve, z_valve เป็น initial — ถ้า servo ไม่ทำงาน จะคืนค่าเหล่านี้กลับมา
                    print("  [arm] Phase2 visual servo (y, z only)")
                    y, z = visual_servo(arm, x_app, y_valve, z_valve)

                    # Phase 3: advance — หา max reach ที่ z ยังถูกต้อง แล้วเข้าหา
                    x3, y3, z3 = find_max_reach(x, y, z)
                    angles_final = solve_ik_clamped(x3, y3, z3)
                    print(f"  [arm] Phase3 advance → ({x3:.0f}, {y3:.0f}, {z3:.0f}) mm")
                    arm.move_smooth(angles_final, steps=30, delay=0.02, settle=0.3)
                    print(f"  [arm] ถึงเป้า — ค้างไว้ {HOLD_AT_TARGET_SEC:.0f}s ก่อนเช็ค valve หาย\n")
                    time.sleep(HOLD_AT_TARGET_SEC)
                    at_scan  = False   # ล็อค: ไม่รับ detection ใหม่จนกว่า valve จะหาย
                    miss_cnt = 0
            else:
                hit_buf.clear()

        else:
            # ── ล็อค: รอ valve หาย ────────────────────────────────────
            if pos is None:
                miss_cnt += 1
                print(f"  [detect] ไม่เจอ valve — miss {miss_cnt}/{CONFIRM_FRAMES}")
                if miss_cnt >= CONFIRM_FRAMES:
                    arm.move_to_scan_pose()
                    print("  [arm] กลับ scan pose\n")
                    at_scan  = True
                    miss_cnt = 0
            # ถ้า pos is not None ขณะ at_scan=False → ไม่ทำอะไร (ไม่สั่งแขน)

        time.sleep(LOOP_INTERVAL_SEC)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nหยุดการทำงาน")
    finally:
        _close_camera()
