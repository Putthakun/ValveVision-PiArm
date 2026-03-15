import sys
import time
import termios
import tty

import board
import busio
from adafruit_pca9685 import PCA9685

# ============ CONFIG ============
CHANNEL = 1
FREQ = 50

PULSE_MIN = 600
PULSE_MID = 1500
PULSE_MAX = 2500


# ================================
def us_to_duty_u16(us: float, freq: float) -> int:
    period_us = 1_000_000 / freq
    return int((us / period_us) * 65535)

def set_pulse_us(pca, ch, us):
    pca.channels[ch].duty_cycle = us_to_duty_u16(us, pca.frequency)

def relax(pca, ch):
    pca.channels[ch].duty_cycle = 0

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

if __name__ == "__main__":
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = FREQ

    print("=== SERVO QUICK TEST ===")
    print("1 = MIN (400us)")
    print("2 = MID (1500us)")
    print("3 = MAX (2600us)")
    print("r = relax")
    print("q = quit")
    print("------------------------")

    try:
        while True:
            key = getch()

            if key == "1":
                print("-> MIN 400us")
                set_pulse_us(pca, CHANNEL, PULSE_MIN)

            elif key == "2":
                print("-> MID 1500us")
                set_pulse_us(pca, CHANNEL, PULSE_MID)

            elif key == "3":
                print("-> MAX 2600us")
                set_pulse_us(pca, CHANNEL, PULSE_MAX)

            elif key == "r":
                print("-> RELAX")
                relax(pca, CHANNEL)

            elif key == "q":
                break

            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        print("\nExit.")
        relax(pca, CHANNEL)
        pca.deinit()