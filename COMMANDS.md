# คำสั่งที่ใช้บ่อย

รันบน Pi ทั้งหมด (`venv/bin/python3` คือ interpreter ที่มีทั้ง cv2/onnxruntime และ
`adafruit_servokit`/`picamera2` — ดู [CLAUDE.md](./CLAUDE.md) หัวข้อสภาพแวดล้อม)

## ก่อนเปิดกล้องตัวไหนก็ตาม

โปรเซสที่จับกล้องค้างไว้ (`preview_cameras.py`, `preview_detect.py`,
`test_coarse_live.py`) ต้องปิดก่อน ไม่งั้นจะเจอ `Device or resource busy`:

```bash
pkill -f preview_cameras.py
pkill -f preview_detect.py
pkill -f test_coarse_live.py
```

## เทส

```bash
venv/bin/python3 -m pytest tests/ -q          # รันเทสทั้งหมด
venv/bin/python3 -m pytest tests/test_coarse.py -q   # รันเฉพาะไฟล์เดียว
```

## ดูภาพสดผ่านเบราว์เซอร์

```bash
venv/bin/python3 tools/preview_cameras.py     # http://<Tailscale IP>:8081/wrist — ดูภาพดิบ
venv/bin/python3 tools/preview_detect.py      # http://<Tailscale IP>:8082/ — ดูพร้อมกรอบที่โมเดลจับได้
```

## ทดสอบเฟสหยาบ (Task 9) บนแขนจริง

```bash
venv/bin/python3 tools/test_coarse_live.py
```
หมุนล้อไปตำแหน่งที่ต้องการ พิมพ์เลขนาฬิกาแล้ว Enter ดู log บรรทัด `[coarse]`
(hub/valve/dx/dy/clock) เทียบกับตำแหน่งจริงที่ตั้ง ภาพที่ถ่ายเก็บไว้ที่
`data/coarse_test/` ภาพ debug ตอนหาไม่เจอเก็บที่ `/tmp/coarse_debug_*.jpg`

## ปรับท่าสแกน (SCAN_POSE / SCAN_POSE_UPPER)

```bash
venv/bin/python3 tools/adjust_scan_pose.py            # ท่า A
venv/bin/python3 tools/adjust_scan_pose.py --upper    # ท่า B
```
กด `a/d`=J1, `w/x`=J2, `e/c`=J3, `[`/`]`=ขนาดก้าว 1°/5°, `p`=บันทึกลง `config.py`, `q`=ออก

## ไล่หามุมที่ servo เริ่มฝืด/สตอล

```bash
venv/bin/python3 tools/probe_j2_stall.py
```
ใช้เวลาสงสัยว่า servo ยกไม่ขึ้น/ค้างกลางทาง (ดู DESIGN.md หัวข้อ 7)

## เครื่องมืออื่นจาก Task ก่อนหน้า

```bash
venv/bin/python3 tools/find_joint_limits.py          # หาขีดจำกัดจริงของแต่ละ joint (Task 1)
venv/bin/python3 tools/test_reach_all_clock.py       # เช็คว่าแขนไปถึงทุกตำแหน่งนาฬิกาไหม (Task 5B)
venv/bin/python3 tools/collect_dataset.py --clock 6  # เก็บภาพ dataset ที่ตำแหน่ง 6 นาฬิกา (Task 6/12)
venv/bin/python3 tools/test_model.py                 # วัด accuracy โมเดลบนภาพที่เก็บไว้ (Task 7)
venv/bin/python3 tools/measure_pixel_scale.py        # วัดอัตราส่วนพิกเซล→มม. สำหรับเฟสละเอียด (Task 8)
```

## Git

```bash
git status --short
git add -A && git commit -m "..."
git push origin main
```
