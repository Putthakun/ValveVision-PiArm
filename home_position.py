import time
import board
import busio
from adafruit_pca9685 import PCA9685

# ============ CONFIG ============
FREQ = 50

BASE = 0
SHOULDER = 1
ELBOW = 2
WRIST = 3
ROTATE = 4
GRIP = 5

HOME_POSE = {
    BASE: 1500,
    SHOULDER: 2000,
    ELBOW: 600,
    WRIST: 2500,
    ROTATE: 1500,
    GRIP: 1700,
}

# ================================

def us_to_duty_u16(us: float, freq: float) -> int:
    period_us = 1_000_000 / freq
    duty = int((us / period_us) * 65535)
    return max(0, min(65535, duty))

def set_pulse_us(pca, ch, us):
    pca.channels[ch].duty_cycle = us_to_duty_u16(us, pca.frequency)

def relax_all(pca):
    for ch in HOME_POSE.keys():
        pca.channels[ch].duty_cycle = 0

if __name__ == "__main__":
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = FREQ

    print("=== SET HOME POSITION ===")

    try:
        for ch, pulse in HOME_POSE.items():
            print(f"CH{ch} -> {pulse}us")
            set_pulse_us(pca, ch, pulse)
            time.sleep(0.1)  # หน่วงนิด ป้องกันกระชากไฟพร้อมกัน

        print("✅ HOME position set")

    except KeyboardInterrupt:
        pass
    finally:
        # ถ้าต้องการให้ค้างตำแหน่งไว้ ไม่ต้อง relax
        # relax_all(pca)
        pca.deinit()