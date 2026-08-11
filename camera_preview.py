#!/usr/bin/env python3
"""
camera_preview.py — ดูกล้องภาพรวม (overview camera, USB webcam) แบบ realtime ผ่าน browser
พร้อม real-time valve detection ด้วย ONNX model

กล้อง: Logitech Brio 100 (USB) ต่อผ่าน /dev/v4l/by-id/... — path คงที่ไม่ขึ้นกับลำดับ /dev/videoN
       (Camera Module 3 ย้ายไปติดที่ gripper แทนแล้ว ไม่ได้ใช้ในสคริปต์นี้)

รัน:  python3 camera_preview.py
เปิด: http://<TAILSCALE_IP>:8080/stream   — ภาพดิบ
      http://<TAILSCALE_IP>:8080/detect   — ภาพพร้อม detection
      http://<TAILSCALE_IP>:8080/snapshot — snapshot (ภาพดิบ)

Ctrl+C เพื่อหยุด
"""

import io
import os
import threading
import numpy as np
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer

from valve_detector import (load_model, preprocess, postprocess,
                             draw_detections, CONF_THRESH, IOU_THRESH, CLASS_NAMES)

# path คงที่ของ Brio 100 — ไม่เปลี่ยนแม้ /dev/videoN จะสลับลำดับหลัง reboot
CAMERA_DEVICE = os.getenv(
    "OVERVIEW_CAMERA_DEVICE",
    "/dev/v4l/by-id/usb-046d_Brio_100_2544APY4JYJ8-video-index0",
)
CAMERA_FRAME_WIDTH  = int(os.getenv("CAMERA_FRAME_WIDTH", "1280"))
CAMERA_FRAME_HEIGHT = int(os.getenv("CAMERA_FRAME_HEIGHT", "720"))
CAMERA_FPS          = int(os.getenv("CAMERA_FPS", "30"))


# ── Shared frame buffers ─────────────────────────────────────────────────────

class StreamingOutput(io.BufferedIOBase):
    """รับ JPEG bytes จาก picamera2 encoder"""
    def __init__(self):
        self.frame     = None
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class DetectionOutput:
    """เก็บ JPEG bytes หลัง overlay detection boxes"""
    def __init__(self):
        self.frame     = None
        self.condition = threading.Condition()

    def update(self, jpeg_bytes):
        with self.condition:
            self.frame = jpeg_bytes
            self.condition.notify_all()

    def wait_frame(self):
        with self.condition:
            self.condition.wait()
            return self.frame


raw_output = StreamingOutput()
det_output = DetectionOutput()

# latest detections shared between threads
_det_lock  = threading.Lock()
_last_dets = []

def get_last_dets():
    with _det_lock:
        return list(_last_dets)

def set_last_dets(dets):
    with _det_lock:
        global _last_dets
        _last_dets = dets


# ── Detection worker thread ──────────────────────────────────────────────────

def detection_worker():
    """รัน inference อย่างเดียว อัปเดต last_dets ไม่ต้อง encode JPEG"""
    print("[detect] โหลด ONNX model...")
    try:
        session, input_name, output_name = load_model()
        print("[detect] โหลดสำเร็จ — เริ่ม inference loop")
    except Exception as e:
        print(f"[detect] โหลด model ไม่ได้: {e}")
        return

    DEBOUNCE_N  = 3   # ต้องเห็น N frame ติดกันก่อนเปลี่ยนสถานะ
    stable      = 0   # สถานะที่ stable แล้ว
    streak      = 0   # นับ frame ติดกันที่เห็น raw == candidate
    candidate   = 0

    while True:
        with raw_output.condition:
            raw_output.condition.wait()
            jpeg_bytes = raw_output.frame

        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            continue

        orig_h, orig_w = img.shape[:2]
        blob, scale, pad_left, pad_top = preprocess(img)
        out  = session.run([output_name], {input_name: blob})
        dets = postprocess(out[0], orig_w, orig_h, scale, pad_left, pad_top)
        set_last_dets(dets)

        raw = 1 if dets else 0

        # debounce
        if raw == candidate:
            streak += 1
        else:
            candidate = raw
            streak    = 1

        if streak >= DEBOUNCE_N:
            stable = candidate

        print(f"[detect] {stable}", flush=True)


def overlay_worker():
    """รัน 30fps — วาด last_dets ลงทุก frame แล้ว push ไป det_output"""
    while True:
        with raw_output.condition:
            raw_output.condition.wait()
            jpeg_bytes = raw_output.frame

        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            continue

        dets = get_last_dets()
        img  = draw_detections(img, dets)

        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            det_output.update(buf.tobytes())


# ── USB camera capture thread ────────────────────────────────────────────────

def capture_worker(device):
    """อ่านเฟรมจากกล้อง USB ต่อเนื่อง → encode JPEG → push เข้า raw_output
    เขียนผ่าน raw_output.write() เหมือน interface เดิมของ picamera2 encoder"""
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"เปิดกล้อง {device} ไม่ได้")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print(f"[cam] เปิดกล้อง USB สำเร็จ: {device}")
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            raw_output.write(buf.tobytes())


# ── HTTP handler ─────────────────────────────────────────────────────────────

class StreamingHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # ปิด log รก

    def _mjpeg_stream(self, get_frame_fn):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()
        try:
            while True:
                frame = get_frame_fn()
                self.wfile.write(b"--FRAME\r\n")
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", len(frame))
                self.end_headers()
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except Exception:
            pass

    def do_GET(self):
        if self.path == "/":
            self.send_response(301)
            self.send_header("Location", "/detect")
            self.end_headers()

        elif self.path == "/stream":
            # ภาพดิบจาก camera
            def get_raw():
                with raw_output.condition:
                    raw_output.condition.wait()
                    return raw_output.frame
            self._mjpeg_stream(get_raw)

        elif self.path == "/detect":
            # ภาพพร้อม detection overlay
            self._mjpeg_stream(det_output.wait_frame)

        elif self.path == "/snapshot":
            with raw_output.condition:
                raw_output.condition.wait()
                frame = raw_output.frame
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", len(frame))
            self.end_headers()
            self.wfile.write(frame)

        else:
            self.send_error(404)


# ── Main ─────────────────────────────────────────────────────────────────────

PORT = 8080

# เริ่ม capture + detection + overlay threads
cap_thread = threading.Thread(target=capture_worker, args=(CAMERA_DEVICE,), daemon=True)
det_thread = threading.Thread(target=detection_worker, daemon=True)
ovl_thread = threading.Thread(target=overlay_worker,   daemon=True)
cap_thread.start()
det_thread.start()
ovl_thread.start()

server = ThreadingHTTPServer(("0.0.0.0", PORT), StreamingHandler)

try:
    print(f"Camera streaming เริ่มแล้ว (source: {CAMERA_DEVICE})")
    print(f"  ภาพดิบ:    http://<TAILSCALE_IP>:{PORT}/stream")
    print(f"  Detection: http://<TAILSCALE_IP>:{PORT}/detect")
    print(f"  Snapshot:  http://<TAILSCALE_IP>:{PORT}/snapshot")
    print("Ctrl+C เพื่อหยุด\n")
    server.serve_forever()
except KeyboardInterrupt:
    print("\nหยุด streaming")
