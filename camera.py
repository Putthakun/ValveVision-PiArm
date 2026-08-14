# camera.py — กล้อง 3 แบบ หน้าตาเรียกใช้เหมือนกัน (BaseCamera.grab() -> BGR ndarray | None)
#
# กล้องภาพรวม (OverviewCamera) กับกล้องที่มือ (WristCamera) คนละหน้าที่กันเด็ดขาด — ดู CONTEXT.md
# ReplayCamera คือหัวใจของโหมดจำลอง อ่านภาพที่บันทึกไว้แทนกล้องจริง ทำให้ทดสอบ pipeline ได้โดยไม่ต้องมีฮาร์ดแวร์

import glob
import os

import cv2
import numpy as np

# path คงที่ของ Brio 100 — ไม่เปลี่ยนแม้ /dev/videoN จะสลับลำดับหลัง reboot
OVERVIEW_CAMERA_DEVICE = os.getenv(
    "OVERVIEW_CAMERA_DEVICE",
    "/dev/v4l/by-id/usb-046d_Brio_100_2544APY4JYJ8-video-index0",
)
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720


class BaseCamera:
    def grab(self) -> np.ndarray | None:
        """คืนภาพล่าสุดเป็น BGR ndarray หรือ None ถ้าอ่านไม่ได้"""
        raise NotImplementedError

    def close(self) -> None:
        pass


class OverviewCamera(BaseCamera):
    """กล้องภาพรวม — Logitech Brio 100 (USB) ตรึงกับที่ ใช้ในเฟสหยาบ"""

    def __init__(self, device: str = OVERVIEW_CAMERA_DEVICE):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"เปิดกล้องภาพรวมไม่ได้: {device}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def grab(self) -> np.ndarray | None:
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self) -> None:
        self.cap.release()


class WristCamera(BaseCamera):
    """กล้องที่มือ — Pi Camera Module 3 ติดหลัง gripper ใช้ในเฟสละเอียด (eye-in-hand)

    import picamera2 ในเมธอดนี้ ไม่ใช่ระดับโมดูล เพื่อให้ import camera.py
    สำเร็จได้แม้ในเครื่องที่ไม่มี picamera2 (กฎข้อ 7 ของ CLAUDE.md)
    """

    def __init__(self, size: tuple[int, int] = (FRAME_WIDTH, FRAME_HEIGHT)):
        from picamera2 import Picamera2

        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": size, "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()

    def grab(self) -> np.ndarray | None:
        try:
            frame = self.picam2.capture_array()
        except Exception:
            return None
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    def close(self) -> None:
        self.picam2.stop()


class ReplayCamera(BaseCamera):
    """อ่านภาพจากโฟลเดอร์แทนกล้องจริง วนซ้ำเมื่อหมด — หัวใจของโหมดจำลอง"""

    def __init__(self, replay_dir: str):
        self._paths = sorted(
            glob.glob(os.path.join(replay_dir, "*.jpg"))
            + glob.glob(os.path.join(replay_dir, "*.png"))
        )
        self._i = 0

    def grab(self) -> np.ndarray | None:
        if not self._paths:
            return None
        path = self._paths[self._i % len(self._paths)]
        self._i += 1
        return cv2.imread(path)


def open_cameras(replay_dir: str | None = None) -> tuple[BaseCamera, BaseCamera]:
    """เปิดกล้องคู่ (ภาพรวม, ที่มือ) — คืน ReplayCamera สองตัวถ้าระบุ replay_dir ไม่งั้นคืนกล้องจริง"""
    if replay_dir is not None:
        overview_dir = os.path.join(replay_dir, "overview")
        wrist_dir = os.path.join(replay_dir, "wrist")
        return ReplayCamera(overview_dir), ReplayCamera(wrist_dir)

    return OverviewCamera(), WristCamera()
