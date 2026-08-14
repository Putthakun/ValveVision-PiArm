# tools/test_model_at_range.py — ถ่ายภาพจากกล้องที่มือที่ระยะต่างๆ เพื่อทดสอบว่าโมเดลเดิมเจอวาล์วที่ระยะใกล้ไหม (Task 5)
#
# วิธีใช้:
#   1. วัดระยะจากหน้าเลนส์กล้องที่มือถึงก้านจุ๊บด้วยไม้บรรทัด ให้ตรงกับ <ระยะ> ที่ระบุ
#   2. รัน: python3 tools/test_model_at_range.py <ระยะเป็นซม.>
#      เช่น: python3 tools/test_model_at_range.py 12
#   3. สคริปต์จะถ่าย 5 ภาพ เว้นช่วงให้ขยับกล้อง/มุมเล็กน้อยระหว่างภาพ บันทึกลง data/range_test/<ระยะ>cm/
#   4. ทำซ้ำสำหรับระยะ 12, 15, 20, 30, 40 ซม. (ตาม Task 5 Step 2)

import sys
import os
import time

import cv2

from camera import WristCamera

N_SHOTS = 5
WARMUP_FRAMES = 15
DELAY_BETWEEN_SHOTS_SEC = 1.5


def main():
    if len(sys.argv) != 2:
        print("วิธีใช้: python3 tools/test_model_at_range.py <ระยะเป็นซม.>")
        sys.exit(1)

    distance_cm = sys.argv[1]
    out_dir = os.path.join("data", "range_test", f"{distance_cm}cm")
    os.makedirs(out_dir, exist_ok=True)

    print(f"เปิดกล้องที่มือ ... (ระยะที่จะถ่าย: {distance_cm} ซม. — ต้องวัดจากหน้าเลนส์ถึงก้านจุ๊บให้ตรงแล้ว)")
    cam = WristCamera()

    print(f"warm-up {WARMUP_FRAMES} เฟรม ...")
    for _ in range(WARMUP_FRAMES):
        cam.grab()
        time.sleep(0.1)

    for i in range(N_SHOTS):
        frame = cam.grab()
        if frame is None:
            print(f"  [{i+1}/{N_SHOTS}] อ่านภาพไม่ได้ ข้าม")
            continue
        path = os.path.join(out_dir, f"{i:02d}.jpg")
        cv2.imwrite(path, frame)
        print(f"  [{i+1}/{N_SHOTS}] บันทึก {path} (brightness={frame.mean():.1f})")
        if i < N_SHOTS - 1:
            print(f"  ขยับมุม/ระยะเล็กน้อย แล้วรอ {DELAY_BETWEEN_SHOTS_SEC} วิ ...")
            time.sleep(DELAY_BETWEEN_SHOTS_SEC)

    cam.close()
    print(f"เสร็จ — เก็บภาพระยะ {distance_cm} ซม. ครบ {N_SHOTS} ภาพที่ {out_dir}")


if __name__ == "__main__":
    main()
