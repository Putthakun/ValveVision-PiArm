"""
valve_detector.py — ONNX inference helpers (ไม่มี camera init)
import ได้จากทั้ง camera_preview.py และ main.py
"""

import os
import numpy as np
import cv2
import onnxruntime as ort

MODEL_PATH  = os.path.join(os.path.dirname(__file__), "models", "best.onnx")
INPUT_SIZE  = 640
CONF_THRESH = 0.10
IOU_THRESH  = 0.45
CLASS_NAMES = ["valve"]


def load_model():
    sess_opts = ort.SessionOptions()
    sess_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    session    = ort.InferenceSession(MODEL_PATH, sess_options=sess_opts,
                                      providers=["CPUExecutionProvider"])
    input_name  = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    return session, input_name, output_name


def letterbox(img_bgr, size=640):
    h, w   = img_bgr.shape[:2]
    scale  = size / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(img_bgr, (new_w, new_h))
    pad_top  = (size - new_h) // 2
    pad_left = (size - new_w) // 2
    canvas   = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[pad_top:pad_top+new_h, pad_left:pad_left+new_w] = resized
    return canvas, scale, pad_left, pad_top


def preprocess(img_bgr):
    lb, scale, pad_left, pad_top = letterbox(img_bgr, INPUT_SIZE)
    img = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    return np.expand_dims(img, 0), scale, pad_left, pad_top


def postprocess(output, orig_w, orig_h, scale, pad_left, pad_top):
    preds = output[0].T   # (8400, 4+nc)

    boxes_xywh   = preds[:, :4]
    class_scores = preds[:, 4:]
    class_ids    = np.argmax(class_scores, axis=1)
    confs        = class_scores[np.arange(len(class_scores)), class_ids]

    mask        = confs > CONF_THRESH
    boxes_xywh  = boxes_xywh[mask]
    confs       = confs[mask]
    class_ids   = class_ids[mask]

    if len(boxes_xywh) == 0:
        return []

    cx, cy, w, h = boxes_xywh[:, 0], boxes_xywh[:, 1], boxes_xywh[:, 2], boxes_xywh[:, 3]
    x1 = np.clip(((cx - w / 2 - pad_left) / scale).astype(int), 0, orig_w)
    y1 = np.clip(((cy - h / 2 - pad_top)  / scale).astype(int), 0, orig_h)
    x2 = np.clip(((cx + w / 2 - pad_left) / scale).astype(int), 0, orig_w)
    y2 = np.clip(((cy + h / 2 - pad_top)  / scale).astype(int), 0, orig_h)

    xyxy    = np.stack([x1, y1, x2, y2], axis=1).tolist()
    results = []
    for cid in np.unique(class_ids):
        idx       = np.where(class_ids == cid)[0]
        sub_boxes = np.array([xyxy[i] for i in idx], dtype=np.float32)
        sub_confs = confs[idx].tolist()
        keep      = cv2.dnn.NMSBoxes(sub_boxes.tolist(), sub_confs, CONF_THRESH, IOU_THRESH)
        for k in (keep.flatten() if len(keep) else []):
            results.append((*[int(v) for v in sub_boxes[k]], float(sub_confs[k]), int(cid)))

    return results   # [(x1,y1,x2,y2,conf,class_id), ...]


def draw_detections(img_bgr, detections):
    for x1, y1, x2, y2, conf, cid in detections:
        label = f"{CLASS_NAMES[cid] if cid < len(CLASS_NAMES) else cid} {conf:.2f}"
        cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(img_bgr, (x1, y1 - th - 6), (x1 + tw, y1), (0, 255, 0), -1)
        cv2.putText(img_bgr, label, (x1, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    return img_bgr
