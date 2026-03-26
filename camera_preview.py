#!/usr/bin/env python3
"""
camera_preview.py — ดู Camera Module 3 แบบ realtime ผ่าน browser
พร้อม real-time valve detection ด้วย ONNX model

รัน:  python3 camera_preview.py
เปิด: http://<TAILSCALE_IP>:8080/stream   — ภาพดิบ
      http://<TAILSCALE_IP>:8080/detect   — ภาพพร้อม detection
      http://<TAILSCALE_IP>:8080/snapshot — snapshot (ภาพดิบ)

Ctrl+C เพื่อหยุด
"""

import io
import threading
import numpy as np
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

from valve_detector import (load_model, preprocess, postprocess,
                             draw_detections, CONF_THRESH)


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


# ── ONNX inference helpers ───────────────────────────────────────────────────

def load_model():
    sess_opts = ort.SessionOptions()
    sess_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    providers = ["CPUExecutionProvider"]
    session = ort.InferenceSession(MODEL_PATH, sess_options=sess_opts, providers=providers)
    input_name  = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    return session, input_name, output_name


def letterbox(img_bgr, size=640):
    """Resize with padding (letterbox) รักษา aspect ratio เหมือนตอน train"""
    h, w = img_bgr.shape[:2]
    scale = size / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(img_bgr, (new_w, new_h))
    # padding ให้ครบ size x size
    pad_top  = (size - new_h) // 2
    pad_left = (size - new_w) // 2
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[pad_top:pad_top+new_h, pad_left:pad_left+new_w] = resized
    return canvas, scale, pad_left, pad_top


def preprocess(img_bgr):
    """BGR numpy → float32 blob (1,3,H,W) normalized 0-1 พร้อม letterbox"""
    lb, scale, pad_left, pad_top = letterbox(img_bgr, INPUT_SIZE)
    img = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    return np.expand_dims(img, 0), scale, pad_left, pad_top


def postprocess(output, orig_w, orig_h, scale, pad_left, pad_top):
    """
    YOLOv8 output shape: (1, 4+nc, 8400)
    Returns list of (x1,y1,x2,y2, conf, class_id) in pixel coords of original image
    """
    preds = output[0]               # (4+nc, 8400)
    preds = preds.T                 # (8400, 4+nc)

    boxes_xywh  = preds[:, :4]
    class_scores = preds[:, 4:]

    class_ids = np.argmax(class_scores, axis=1)
    confs     = class_scores[np.arange(len(class_scores)), class_ids]

    mask  = confs > CONF_THRESH
    boxes_xywh  = boxes_xywh[mask]
    confs        = confs[mask]
    class_ids    = class_ids[mask]

    if len(boxes_xywh) == 0:
        return []

    # cx,cy,w,h อยู่ใน letterbox space → แปลงกลับเป็น pixel coords ของภาพต้นฉบับ
    cx, cy, w, h = boxes_xywh[:, 0], boxes_xywh[:, 1], boxes_xywh[:, 2], boxes_xywh[:, 3]
    x1 = ((cx - w / 2 - pad_left) / scale).astype(int)
    y1 = ((cy - h / 2 - pad_top)  / scale).astype(int)
    x2 = ((cx + w / 2 - pad_left) / scale).astype(int)
    y2 = ((cy + h / 2 - pad_top)  / scale).astype(int)

    x1 = np.clip(x1, 0, orig_w)
    y1 = np.clip(y1, 0, orig_h)
    x2 = np.clip(x2, 0, orig_w)
    y2 = np.clip(y2, 0, orig_h)

    xyxy = np.stack([x1, y1, x2, y2], axis=1).tolist()

    # NMS per class
    results = []
    for cid in np.unique(class_ids):
        idx = np.where(class_ids == cid)[0]
        sub_boxes = np.array([xyxy[i] for i in idx], dtype=np.float32)
        sub_confs = confs[idx].tolist()
        keep = cv2.dnn.NMSBoxes(
            sub_boxes.tolist(), sub_confs, CONF_THRESH, IOU_THRESH
        )
        for k in (keep.flatten() if len(keep) else []):
            results.append((*[int(v) for v in sub_boxes[k]], float(sub_confs[k]), int(cid)))

    return results   # [(x1,y1,x2,y2,conf,class_id), ...]


def draw_detections(img_bgr, detections):
    for x1, y1, x2, y2, conf, cid in detections:
        label = f"{CLASS_NAMES[cid] if cid < len(CLASS_NAMES) else cid} {conf:.2f}"
        cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(img_bgr, (x1, y1 - th - 6), (x1 + tw, y1), (0, 255, 0), -1)
        cv2.putText(img_bgr, label, (x1, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    return img_bgr


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

picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (1280, 720)},
    controls={"FrameRate": 30},
)
picam2.configure(config)
picam2.start_recording(MJPEGEncoder(), FileOutput(raw_output))

# เริ่ม detection + overlay threads
det_thread = threading.Thread(target=detection_worker, daemon=True)
ovl_thread = threading.Thread(target=overlay_worker,   daemon=True)
det_thread.start()
ovl_thread.start()

server = HTTPServer(("0.0.0.0", PORT), StreamingHandler)

try:
    import socket
    hostname = socket.gethostname()
    print(f"Camera streaming เริ่มแล้ว")
    print(f"  ภาพดิบ:    http://<TAILSCALE_IP>:{PORT}/stream")
    print(f"  Detection: http://<TAILSCALE_IP>:{PORT}/detect")
    print(f"  Snapshot:  http://<TAILSCALE_IP>:{PORT}/snapshot")
    print("Ctrl+C เพื่อหยุด\n")
    server.serve_forever()
except KeyboardInterrupt:
    print("\nหยุด streaming")
finally:
    picam2.stop_recording()
    picam2.close()
