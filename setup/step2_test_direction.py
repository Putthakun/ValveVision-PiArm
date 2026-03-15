# step2_test_direction.py
# ทดสอบทิศทาง servo ทีละตัว -- หา INVERT flag
# รันแล้วดูแขน แล้วตอบ y/n
import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
for ch in range(6):
    kit.servo[ch].set_pulse_width_range(500, 2500)

joints = ['J1 Base', 'J2 Shoulder', 'J3 Elbow', 'J4 Wrist Pitch', 'J5 Wrist Roll', 'J6 Gripper']
results = {}

print("=" * 50)
print("STEP 2 -- Test Servo Direction")
print("แต่ละ joint จะขยับจาก 90° → 110° → 90°")
print("ดูว่าขยับไปทิศที่ถูกต้องไหม")
print("=" * 50)

# กลับ home ก่อน
print("\nกลับ Home (90° ทุกตัว)...")
for ch in range(6):
    kit.servo[ch].angle = 90
time.sleep(2)

for ch, name in enumerate(joints):
    input(f"\n[CH{ch} {name}] กด Enter เพื่อเริ่มทดสอบ...")

    print(f"  90° → 110° (เพิ่มองศา)")
    kit.servo[ch].angle = 110
    time.sleep(1.5)
    print(f"  กลับ 90°")
    kit.servo[ch].angle = 90
    time.sleep(1)

    ans = input(f"  ขยับถูกทิศ (positive direction) ไหม? (y=ถูก / n=กลับหัว): ").strip().lower()
    results[f'J{ch+1}'] = False if ans == 'y' else True

print("\n" + "=" * 50)
print("ผลลัพธ์ INVERT -- คัดลอกไปใส่ใน config.py:")
print("=" * 50)
print("INVERT = {")
for j, v in results.items():
    print(f"    '{j}': {v},")
print("}")
