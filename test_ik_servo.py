# test_ik_servo.py — ทดสอบส่งพิกัด x,y,z ไปแขนจริง
#
# วิธีใช้:
#   python test_ik_servo.py
#   พิมพ์พิกัด เช่น  300 0 150  แล้วกด Enter
#   พิมพ์  home  เพื่อกลับ home
#   พิมพ์  q     เพื่อออก

from servo_controller import ServoController
from ik_solver import solve_ik

arm = ServoController()

print("กลับ Home ก่อน...")
arm.move_to_home()
print("พร้อม\n")

print("พิมพ์พิกัด:  x y z   (mm)  เช่น  300 0 150")
print("             home          กลับ home")
print("             q             ออก")
print()

while True:
    cmd = input(">>> ").strip()

    if cmd.lower() == 'q':
        break

    if cmd.lower() == 'home':
        arm.move_to_home()
        print("  → home")
        continue

    parts = cmd.split()
    if len(parts) != 3:
        print("  ใส่ 3 ค่า: x y z")
        continue

    try:
        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
    except ValueError:
        print("  ค่าต้องเป็นตัวเลข")
        continue

    angles = solve_ik(x, y, z)
    if angles is None:
        print("  → IK หา solution ไม่ได้ (นอก workspace)")
        continue

    print(f"  J1={angles['J1']:.1f}  J2={angles['J2']:.1f}  "
          f"J3={angles['J3']:.1f}  J4={angles['J4']:.1f}")
    arm.move_smooth(angles)
    print("  → เสร็จ วัดระยะปลาย gripper จากเป้าจริงว่าห่างกี่ mm")

arm.move_to_home()
print("home แล้ว ออก")
