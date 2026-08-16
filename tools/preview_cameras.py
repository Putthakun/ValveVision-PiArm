#!/usr/bin/env python3
# tools/preview_cameras.py — ดูภาพสดจากกล้องทั้ง 2 ตัวพร้อมกันผ่าน browser
#
# ใช้ตอนจัดมุมกล้อง (ทั้งกล้องภาพรวมที่ตรึงกับที่ และกล้องที่มือ)
# มีกากบาทกลางภาพช่วยเล็ง และบอกความสว่างเฉลี่ยให้ดูว่าแสงพอไหม
#
# รัน:  python3 tools/preview_cameras.py
# เปิด: http://<TAILSCALE_IP>:8081/          ← ดูทั้ง 2 กล้องข้างกัน
#       http://<TAILSCALE_IP>:8081/overview  ← กล้องภาพรวมเต็มจอ
#       http://<TAILSCALE_IP>:8081/wrist     ← กล้องที่มือเต็มจอ
#
# Ctrl+C เพื่อหยุด

import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from camera import OverviewCamera, WristCamera

PORT = 8081
JPEG_QUALITY = 80


class CameraFeed:
    """อ่านภาพจากกล้องตัวหนึ่งต่อเนื่องในเธรดของตัวเอง แล้วเก็บ JPEG ล่าสุดไว้ให้ HTTP ดึงไปใช้

    แยกเธรดเพราะถ้าปล่อยให้ HTTP อ่านกล้องเอง ผู้ชมที่เน็ตช้าจะทำให้การอ่านกล้องช้าตามไปด้วย
    """

    def __init__(self, name: str, open_fn):
        self.name = name
        self._open_fn = open_fn
        self._jpeg = None
        self._condition = threading.Condition()
        self.error = None
        self.brightness = 0.0

        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        try:
            cam = self._open_fn()
        except Exception as e:
            self.error = str(e)
            print(f"[{self.name}] เปิดกล้องไม่ได้: {e}")
            return

        print(f"[{self.name}] เปิดกล้องสำเร็จ")
        while True:
            frame = cam.grab()
            if frame is None:
                time.sleep(0.05)
                continue

            self.brightness = float(frame.mean())
            self._draw_crosshair(frame)

            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                with self._condition:
                    self._jpeg = buf.tobytes()
                    self._condition.notify_all()

    def _draw_crosshair(self, frame):
        """วาดกากบาทกลางภาพ + กรอบพื้นที่กลาง ช่วยเล็งตอนจัดมุมกล้อง"""
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        color = (0, 255, 255)

        cv2.line(frame, (cx - 30, cy), (cx + 30, cy), color, 1)
        cv2.line(frame, (cx, cy - 30), (cx, cy + 30), color, 1)
        cv2.rectangle(frame, (w // 4, h // 4), (w * 3 // 4, h * 3 // 4), color, 1)
        cv2.putText(frame, f"{self.name}  brightness={self.brightness:.0f}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    def wait_jpeg(self):
        with self._condition:
            self._condition.wait(timeout=5.0)
            return self._jpeg


feeds = {}

PAGE_HTML = """<!doctype html>
<title>ValveVision — ดูกล้องสด</title>
<style>
  body { background:#111; color:#eee; font-family:sans-serif; margin:0; padding:16px; }
  h1 { font-size:18px; margin:0 0 12px; }
  .row { display:flex; gap:12px; flex-wrap:wrap; }
  .cam { flex:1 1 480px; }
  .cam h2 { font-size:14px; margin:0 0 6px; font-weight:normal; color:#9cf; }
  img { width:100%; border:1px solid #444; display:block; }
  .note { color:#888; font-size:12px; margin-top:12px; }
</style>
<h1>ValveVision — ดูกล้องสดทั้ง 2 ตัว</h1>
<div class="row">
  <div class="cam">
    <h2>กล้องภาพรวม (Brio USB · ตรึงกับที่ · เฟสหยาบ)</h2>
    <img src="/overview.mjpg">
  </div>
  <div class="cam">
    <h2>กล้องที่มือ (Camera Module 3 · บนแขน · เฟสละเอียด)</h2>
    <img src="/wrist.mjpg">
  </div>
</div>
<p class="note">กากบาทเหลือง = กลางภาพ · กรอบเหลือง = พื้นที่กลาง 50% · ดูเต็มจอได้ที่ /overview และ /wrist</p>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # ปิด log รก

    def _stream(self, feed: CameraFeed):
        if feed.error:
            self.send_error(503, f"camera unavailable: {feed.error}")
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()
        try:
            while True:
                jpeg = feed.wait_jpeg()
                if jpeg is None:
                    continue
                self.wfile.write(b"--FRAME\r\n")
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", len(jpeg))
                self.end_headers()
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except Exception:
            pass  # ผู้ชมปิดหน้าเว็บ — เรื่องปกติ

    def _single_page(self, title: str, src: str):
        html = (f"<!doctype html><title>{title}</title>"
                f"<body style='background:#111;margin:0'>"
                f"<img src='{src}' style='width:100%'>")
        body = html.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", len(body))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/":
            body = PAGE_HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", len(body))
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/overview.mjpg":
            self._stream(feeds["overview"])
        elif self.path == "/wrist.mjpg":
            self._stream(feeds["wrist"])

        elif self.path == "/overview":
            self._single_page("กล้องภาพรวม", "/overview.mjpg")
        elif self.path == "/wrist":
            self._single_page("กล้องที่มือ", "/wrist.mjpg")

        else:
            self.send_error(404)


def main():
    feeds["overview"] = CameraFeed("overview", OverviewCamera)
    feeds["wrist"] = CameraFeed("wrist", WristCamera)

    time.sleep(2)  # ให้กล้อง warm-up และ auto-exposure ปรับตัวก่อน

    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"\nเปิดดูได้ที่  http://<TAILSCALE_IP>:{PORT}/")
    print(f"  กล้องภาพรวมเต็มจอ: http://<TAILSCALE_IP>:{PORT}/overview")
    print(f"  กล้องที่มือเต็มจอ:  http://<TAILSCALE_IP>:{PORT}/wrist")
    print("Ctrl+C เพื่อหยุด\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nหยุดสตรีม")


if __name__ == "__main__":
    main()
