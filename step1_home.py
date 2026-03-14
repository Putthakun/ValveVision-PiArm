# step1_home.py — ส่งทุก joint ไป 90° เพื่อจัด Horn
# รันก่อนขันน็อต horn ทุกตัว
#
# วิธีใช้:
#   python step1_home.py

from adafruit_servokit import ServoKit
from config import CHANNEL, PULSE_MIN, PULSE_MAX

kit = ServoKit(channels=16)

for ch in CHANNEL.values():
    kit.servo[ch].set_pulse_width_range(PULSE_MIN, PULSE_MAX)

print("ส่งทุก joint → 90°")
for name, ch in CHANNEL.items():
    kit.servo[ch].angle = 90
    print(f"  {name} (CH{ch}) = 90°")

print()
print("ตอนนี้แขนควรอยู่ท่าตั้งตรง")
print("ถ้า joint ไหนไม่ตรง → ถอด horn แล้วจัดใหม่ที่ตำแหน่งนี้")
print("เสร็จแล้วรัน step2_test_direction.py")
