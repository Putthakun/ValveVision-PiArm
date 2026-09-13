#!/bin/bash
# ติดตั้ง service ให้เปิด Pi แล้ว run.py เริ่มเอง — ต้องรันด้วย sudo
# ⚠️ แขนจะเริ่มขยับทันทีหลังติดตั้งเสร็จ ~10 วินาที
set -e
cd "$(dirname "$0")"
cp valvevision.service /etc/systemd/system/valvevision.service
systemctl daemon-reload
systemctl enable --now valvevision.service
echo "ติดตั้งแล้ว — แขนจะเริ่มขยับใน ~10 วินาที"
echo "  ดู log สด:   journalctl -u valvevision -f"
echo "  หยุด:        sudo systemctl stop valvevision"
echo "  ปิดไม่ให้เริ่มตอนบูต: sudo systemctl disable valvevision"
