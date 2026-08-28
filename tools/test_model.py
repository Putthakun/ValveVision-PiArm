#!/usr/bin/env python3
# tools/test_model.py — วัดว่าโมเดลเจอจุ๊บได้แค่ไหนบนภาพที่เก็บไว้จริง
#
# วิธีใช้:
#   python3 tools/test_model.py                 # สุ่มรอบละ 6 ใบ (~3 นาที)
#   python3 tools/test_model.py --all           # ทุกใบ 1,260 ใบ (~11 นาที)
#   python3 tools/test_model.py --conf 0.25     # ลองค่า conf อื่น
#   python3 tools/test_model.py --save-bad out/ # เก็บภาพที่หาไม่เจอไว้ดู
#
# ⚠️ อ่านตัวเลขจากเครื่องมือนี้ให้ระวัง 2 อย่าง — มันไม่ใช่ผลวัดที่เอาลงเล่มได้
#
#  1. ภาพส่วนใหญ่ถูกใช้เทรนไปแล้ว ตัวเลขจึงสวยเกินจริง
#  2. ★ ภาพพวกนี้ไม่มี label — สคริปต์เดาว่าภาพชนิด near/scan ต้องมีจุ๊บเสมอ
#     ซึ่งไม่จริง บางตำแหน่ง (เช่น 10, 14) เล็งพลาดจนจุ๊บไม่อยู่ในเฟรม
#     "หาไม่เจอ" ตรงนั้นจึงเป็นความผิดของการเก็บภาพ ไม่ใช่ของโมเดล
#
#  → ใช้เครื่องมือนี้เพื่อ "หาว่าพังตรงไหน" แล้วเปิดภาพดูเสมอว่าเป็นเพราะอะไร
#    ผลวัดจริงต้องมาจาก validation set ที่มี label (คำนวณในโน้ตบุ๊ก train_colab)

import argparse
import os
import random
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import glob

import valve_detector as vd

RAW = 'data/raw'
CLOCKS = [6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]
LIGHTS = ['day', 'room', 'dim', 'torch']
# ชนิดที่ "ต้องเจอจุ๊บ" กับชนิดที่ "ต้องไม่เจอ"
POSITIVE = ('near', 'nearblur', 'scanA', 'scanB')
NEGATIVE = ('neg',)


def kind_of(path: str) -> str:
    return os.path.basename(path).rsplit('_', 1)[-1][:-4]


def clock_of(path: str) -> str:
    return os.path.basename(os.path.dirname(path)).split('_')[0]


def light_of(path: str) -> str:
    return os.path.basename(os.path.dirname(path)).split('_')[1]


def detect(sess, inp, out, path):
    img = cv2.imread(path)
    if img is None:
        return None, None
    h, w = img.shape[:2]
    blob, sc, pl, pt = vd.preprocess(img)
    dets = vd.postprocess(sess.run([out], {inp: blob})[0], w, h, sc, pl, pt)
    return dets, img


def pct(a, b):
    return 100.0 * a / b if b else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--all', action='store_true', help='ทุกใบ (ช้า) แทนการสุ่ม')
    ap.add_argument('--per-round', type=int, default=6, help='สุ่มกี่ใบต่อรอบ (ค่าเริ่มต้น 6)')
    ap.add_argument('--conf', type=float, default=None, help='ทับค่า CONF_THRESH')
    ap.add_argument('--save-bad', metavar='DIR', help='เก็บภาพที่หาไม่เจอไว้ที่นี่')
    args = ap.parse_args()

    if args.conf is not None:
        vd.CONF_THRESH = args.conf

    files = sorted(glob.glob(f'{RAW}/*/*.jpg'))
    if not args.all:
        by_round = defaultdict(list)
        for f in files:
            by_round[os.path.dirname(f)].append(f)
        rng = random.Random(0)          # seed คงที่ ให้รันซ้ำได้ผลเดิม
        files = [f for fs in by_round.values() for f in rng.sample(fs, min(args.per_round, len(fs)))]
        files.sort()

    print(f'โมเดล: {vd.MODEL_PATH}')
    print(f'CONF_THRESH = {vd.CONF_THRESH}')
    print(f'ทดสอบ {len(files)} ใบ' + ('' if args.all else f' (สุ่มรอบละ {args.per_round})'))
    print('⚠️ ภาพไม่มี label — "หาไม่เจอ" อาจเป็นเพราะจุ๊บไม่อยู่ในเฟรมตั้งแต่แรก')
    print('   และภาพส่วนใหญ่ถูกใช้เทรนแล้ว ตัวเลขนี้ใช้หาจุดที่พังเท่านั้น ไม่ใช่ผลวัด')
    print('=' * 60, flush=True)

    sess, inp, out = vd.load_model()
    if args.save_bad:
        os.makedirs(args.save_bad, exist_ok=True)

    stat = defaultdict(lambda: [0, 0])       # key -> [เจอ, ทั้งหมด]
    fp = [0, 0]                              # ภาพลบ: [เจอผิด, ทั้งหมด]
    bad = []

    for n, f in enumerate(files, 1):
        dets, img = detect(sess, inp, out, f)
        if dets is None:
            continue
        k, c, l = kind_of(f), clock_of(f), light_of(f)
        found = len(dets) > 0

        if k in NEGATIVE:
            fp[1] += 1
            if found:
                fp[0] += 1
                bad.append((f, dets, img, 'เจอผิดบนภาพลบ'))
        else:
            for key in (('kind', k), ('clock', c), ('light', l), ('all', 'all')):
                stat[key][1] += 1
                if found:
                    stat[key][0] += 1
            if not found:
                bad.append((f, dets, img, 'หาไม่เจอ'))

        if n % 50 == 0:
            print(f'  ...{n}/{len(files)}', flush=True)

    ok, tot = stat[('all', 'all')]
    print(f'\nภาพที่ควรเจอจุ๊บ: {ok}/{tot} = {pct(ok, tot):.1f}%')
    print(f'ภาพลบที่เจอผิด : {fp[0]}/{fp[1]} = {pct(fp[0], fp[1]):.1f}%  (ยิ่งต่ำยิ่งดี)')

    for grp, title in (('kind', 'แยกตามชนิดภาพ'), ('light', 'แยกตามสภาพแสง'), ('clock', 'แยกตามตำแหน่งนาฬิกา')):
        print(f'\n{title}')
        rows = sorted((k[1], v) for k, v in stat.items() if k[0] == grp)
        if grp == 'clock':
            rows.sort(key=lambda r: int(r[0]))
        for name, (a, b) in rows:
            disp = name
            if grp == 'clock' and int(name) > 12:
                disp = f'{name}(={int(name)-12})'
            bar = '█' * int(pct(a, b) / 5)
            print(f'  {disp:>10} {a:>4}/{b:<4} {pct(a,b):5.1f}%  {bar}')

    if args.save_bad and bad:
        for f, dets, img, why in bad:
            vis = vd.draw_detections(img.copy(), dets)
            cv2.putText(vis, why, (10, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imwrite(os.path.join(args.save_bad, os.path.basename(f)), vis)
        print(f'\nเก็บภาพที่มีปัญหา {len(bad)} ใบไว้ที่ {args.save_bad}/')


if __name__ == '__main__':
    main()
