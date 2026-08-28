#!/usr/bin/env bash
# tools/fetch_colab_result.sh — ดึงผลการเทรนจาก Colab เข้ามาใน repo (รันบน Mac)
#
# Colab -> Google Drive -> เบราว์เซอร์โหลดลง ~/Downloads -> สคริปต์นี้ย้ายเข้า repo
#
# วิธีใช้:
#   bash tools/fetch_colab_result.sh                    # หาไฟล์ zip ล่าสุดใน ~/Downloads เอง
#   bash tools/fetch_colab_result.sh path/to/file.zip   # ระบุไฟล์เอง
#   bash tools/fetch_colab_result.sh --install          # แถมติดตั้ง .onnx ทับโมเดลปัจจุบัน
#
# ผลลัพธ์แตกไว้ที่ training/<วันเวลา>/ (ไม่ commit — อยู่ใน .gitignore)

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL=0
ZIP=""

for a in "$@"; do
  case "$a" in
    --install) INSTALL=1 ;;
    *)         ZIP="$a" ;;
  esac
done

# ── หาไฟล์ zip ──────────────────────────────────────────────────────────
if [ -z "$ZIP" ]; then
  ZIP=$(/bin/ls -t "$HOME"/Downloads/valve_result*.zip 2>/dev/null | head -1 || true)
fi

if [ -z "$ZIP" ] || [ ! -f "$ZIP" ]; then
  echo "หาไฟล์ valve_result.zip ไม่เจอใน ~/Downloads"
  echo
  echo "โหลดมาก่อน แล้วค่อยรันสคริปต์นี้ใหม่:"
  echo "  1. เปิด https://drive.google.com  ->  MyDrive  ->  ValveVision"
  echo "  2. โหลด valve_result.zip"
  echo "  3. bash tools/fetch_colab_result.sh"
  exit 1
fi

echo "เจอไฟล์: $ZIP"
echo "         $(du -h "$ZIP" | cut -f1) · แก้ไขล่าสุด $(date -r "$ZIP" '+%Y-%m-%d %H:%M')"

# ── แตกไฟล์ ─────────────────────────────────────────────────────────────
DEST="$REPO/training/$(date -r "$ZIP" '+%Y%m%d_%H%M')"
mkdir -p "$DEST"
unzip -q -o "$ZIP" -d "$DEST"
echo
echo "แตกไว้ที่: ${DEST#$REPO/}"
echo
/bin/ls -lh "$DEST" | tail -n +2 | awk '{printf "   %-46s %s\n", $9, $5}'

# ── โชว์รายงาน ──────────────────────────────────────────────────────────
if [ -f "$DEST/report.txt" ]; then
  echo
  echo "════════════════════ report.txt ════════════════════"
  cat "$DEST/report.txt"
  echo "════════════════════════════════════════════════════"
fi

# ── ติดตั้งโมเดลลง models/ ──────────────────────────────────────────────
ONNX="$DEST/best.onnx"
TARGET="$REPO/models/Valve_detection_model.onnx"

if [ ! -f "$ONNX" ]; then
  echo
  echo "!! ไม่มี best.onnx ในซิป — เซลล์ export ONNX บน Colab น่าจะยังไม่ได้รัน"
  exit 0
fi

if [ "$INSTALL" -eq 0 ]; then
  echo
  echo "โมเดลใหม่อยู่ที่ ${ONNX#$REPO/}"
  echo "ยังไม่ได้ติดตั้งทับของเดิม — ดูตัวเลขในรายงานให้ผ่านเกณฑ์ก่อน แล้วค่อยสั่ง:"
  echo "  bash tools/fetch_colab_result.sh --install"
  exit 0
fi

if [ -f "$TARGET" ]; then
  BAK="$REPO/models/Valve_detection_model.onnx.bak-$(date '+%Y%m%d_%H%M%S')"
  cp "$TARGET" "$BAK"
  echo
  echo "สำรองของเดิมไว้ที่ ${BAK#$REPO/}"
fi

cp "$ONNX" "$TARGET"
echo "ติดตั้งแล้ว -> models/Valve_detection_model.onnx ($(du -h "$TARGET" | cut -f1))"
echo
echo "เหลืออีก 2 อย่างก่อนใช้บน Pi:"
echo "  1. แก้ CONF_THRESH ใน valve_detector.py:13 เป็นค่าที่รายงานบอก (ตอนนี้ 0.10)"
echo "  2. git add models/Valve_detection_model.onnx valve_detector.py && git commit && git push"
echo "     แล้ว ssh ไป Pi สั่ง git pull"
