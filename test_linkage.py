# test_linkage.py — เช็คว่า J3 ขยับตาม J2 หรือเปล่า (parallel linkage test)
#
# วิธีใช้:
#   python test_linkage.py
#
# สังเกต J3 (elbow link) ว่าขยับตามหรือนิ่ง

from servo_controller import ServoController
import time

arm = ServoController()

print("=== ทดสอบ Parallel Linkage ===\n")

print("Step 1: กลับ Home (J2=90, J3=90)...")
arm.move_to_home()
time.sleep(1)
print("  → อยู่ที่ home แล้ว\n")

print("Step 2: ขยับ J2 → 45° เท่านั้น (J3 ยังคง 90°)")
print("  สังเกต J3 (elbow) ว่าขยับตามหรือนิ่ง...")
arm.move_smooth({'J1': 90, 'J2': 45, 'J3': 90, 'J4': 90, 'J5': 90, 'J6': 90})
time.sleep(2)

answer = input("\n  J3 ขยับตาม J2 หรือเปล่า? (y=ขยับตาม / n=นิ่ง): ").strip().lower()

if answer == 'y':
    print("\n→ แขนมี PARALLEL LINKAGE")
    print("   IK model ต้องแก้ใหม่ให้รองรับ linkage compensation")
else:
    print("\n→ แขนเป็น DIRECT DRIVE")
    print("   แก้ SCALE factor ได้เลย")

print("\nกลับ Home...")
arm.move_to_home()
print("เสร็จ")
