#!/usr/bin/env python3
# tools/preview_detect.py — ดูภาพสดจากกล้องที่มือ พร้อมกรอบที่โมเดลตรวจจับได้
#
# ใช้ดูว่าโมเดลทำงานจริงแค่ไหน — หมุนล้อไปเรื่อยๆ แล้วดูว่ามันตามจุ๊บทันไหม
#
# รัน:  python3 tools/preview_detect.py
# เปิด: http://<TAILSCALE_IP>:8082/
#
# ปรับ conf ได้จากคอมมานด์ไลน์:  --conf 0.25
#
# ★ ใช้กล้องพร้อมกับ preview_cameras.py หรือ collect_dataset.py ไม่ได้
#   ต้องปิดตัวอื่นก่อน:  pkill -f preview_cameras.py

import argparse
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2

import valve_detector as vd
from camera import WristCamera

PORT = 8082
JPEG_QUALITY = 80

_lock = threading.Lock()
_frame = None          # เฟรมล่าสุดจากกล้อง (BGR)
_dets = []             # ผลตรวจจับล่าสุด
_stats = {'infer_ms': 0.0, 'fps': 0.0, 'n': 0}


def capture_worker():
    """อ่านกล้องต่อเนื่อง — แยกเธรดเพราะ inference ช้ากว่ากล้องมาก"""
    global _frame
    cam = WristCamera()
    for _ in range(15):          # warm-up ให้ auto-exposure ปรับตัว
        cam.grab()
        time.sleep(0.1)
    print('[cam] เปิดกล้องที่มือสำเร็จ')
    while True:
        f = cam.grab()
        if f is not None:
            with _lock:
                _frame = f


def detect_worker():
    """รันโมเดลบนเฟรมล่าสุดวนไป — ได้ ~2 fps บน Pi ซึ่งพอสำหรับดูด้วยตา"""
    global _dets
    sess, inp, out = vd.load_model()
    print(f'[detect] โหลดโมเดลแล้ว · CONF_THRESH={vd.CONF_THRESH}')
    while True:
        with _lock:
            f = None if _frame is None else _frame.copy()
        if f is None:
            time.sleep(0.05)
            continue
        t = time.time()
        h, w = f.shape[:2]
        blob, sc, pl, pt = vd.preprocess(f)
        d = vd.postprocess(sess.run([out], {inp: blob})[0], w, h, sc, pl, pt)
        dt = time.time() - t
        with _lock:
            _dets = d
            _stats['infer_ms'] = dt * 1000
            _stats['fps'] = 1.0 / dt if dt else 0.0
            _stats['n'] = len(d)


PAGE = """<!doctype html>
<title>ValveVision — โมเดลตรวจจับจุ๊บ</title>
<style>
 body{background:#111;color:#eee;font-family:sans-serif;margin:0;padding:16px}
 h1{font-size:18px;margin:0 0 4px}
 p{color:#888;font-size:13px;margin:0 0 12px}
 img{width:100%;max-width:1280px;border:1px solid #444;display:block}
</style>
<h1>โมเดลตรวจจับจุ๊บ — ภาพสดจากกล้องที่มือ</h1>
<p>กรอบเขียว = ที่โมเดลคิดว่าเป็นจุ๊บ · ตัวเลขคือความมั่นใจ · มุมซ้ายบนบอกเวลาที่ใช้ต่อเฟรม</p>
<img src="/stream.mjpg">
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == '/':
            body = PAGE.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', len(body))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path != '/stream.mjpg':
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
        self.end_headers()
        try:
            while True:
                with _lock:
                    f = None if _frame is None else _frame.copy()
                    d = list(_dets)
                    st = dict(_stats)
                if f is None:
                    time.sleep(0.05)
                    continue

                vis = vd.draw_detections(f, d)
                label = f"{st['infer_ms']:.0f} ms ({st['fps']:.1f} fps) · เจอ {st['n']} กล่อง"
                cv2.putText(vis, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 255) if st['n'] else (0, 140, 255), 2)

                ok, buf = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if not ok:
                    continue
                jpg = buf.tobytes()
                self.wfile.write(b'--FRAME\r\n')
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', len(jpg))
                self.end_headers()
                self.wfile.write(jpg)
                self.wfile.write(b'\r\n')
                time.sleep(0.05)
        except Exception:
            pass          # ผู้ชมปิดหน้าเว็บ — เรื่องปกติ


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--conf', type=float, help='ทับค่า CONF_THRESH')
    args = ap.parse_args()
    if args.conf is not None:
        vd.CONF_THRESH = args.conf

    threading.Thread(target=capture_worker, daemon=True).start()
    threading.Thread(target=detect_worker, daemon=True).start()

    server = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    print(f'\nเปิดดูได้ที่  http://<TAILSCALE_IP>:{PORT}/')
    print('Ctrl+C เพื่อหยุด\n')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nหยุดสตรีม')


if __name__ == '__main__':
    main()
