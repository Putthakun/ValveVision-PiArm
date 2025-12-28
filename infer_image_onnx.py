import cv2
import numpy as np
import onnxruntime as ort
from pathlib import Path

MODEL_PATH = "models/Valve_detection_model.onnx"
IMAGE_PATH = "test_images/IMG_1125.jpg"

CONF_TH = 0.20
IOU_TH = 0.45
IMG_SIZE = 640  # ตอน export ใช้ 640 ก็ให้ตรงกัน

def letterbox(im, new_shape=640, color=(114, 114, 114)):
    h, w = im.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / h, new_shape[1] / w)
    nh, nw = int(round(h * r)), int(round(w * r))

    im_resized = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top = (new_shape[0] - nh) // 2
    bottom = new_shape[0] - nh - top
    left = (new_shape[1] - nw) // 2
    right = new_shape[1] - nw - left
    im_padded = cv2.copyMakeBorder(im_resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)

    return im_padded, r, left, top

def nms(boxes, scores, iou_th=0.45):
    # boxes: (N, 4) x1y1x2y2
    idxs = scores.argsort()[::-1]
    keep = []
    while idxs.size > 0:
        i = idxs[0]
        keep.append(i)
        if idxs.size == 1:
            break
        rest = idxs[1:]

        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h

        area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        area_r = (boxes[rest, 2] - boxes[rest, 0]) * (boxes[rest, 3] - boxes[rest, 1])
        union = area_i + area_r - inter + 1e-6
        iou = inter / union

        idxs = rest[iou < iou_th]
    return keep

def main():
    assert Path(MODEL_PATH).exists(), f"Model not found: {MODEL_PATH}"
    assert Path(IMAGE_PATH).exists(), f"Image not found: {IMAGE_PATH}"

    img0 = cv2.imread(IMAGE_PATH)
    h0, w0 = img0.shape[:2]

    img, r, padw, padh = letterbox(img0, IMG_SIZE)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    x = img_rgb.transpose(2, 0, 1).astype(np.float32) / 255.0
    x = np.expand_dims(x, 0)

    sess = ort.InferenceSession(MODEL_PATH, providers=["CPUExecutionProvider"])
    input_name = sess.get_inputs()[0].name
    out = sess.run(None, {input_name: x})[0]

    # อธิบาย: export ของ YOLOv8 ONNX บางแบบจะออก shape (1, 84, 8400) หรือ (1, 8400, 84)
    # เราจะทำให้เป็น (N, C) ก่อน
    if out.shape[1] < out.shape[2]:
        out = out.transpose(0, 2, 1)  # (1, 8400, 84)

    pred = out[0]  # (N, C)
    # YOLOv8: [cx, cy, w, h, obj?, class...] หรือ [cx,cy,w,h, class...]
    # Ultralytics export ใหม่ๆ มักไม่มี obj แยก จะเป็น 4 + nc
    nc = pred.shape[1] - 4
    boxes = pred[:, :4]
    cls_scores = pred[:, 4:] if nc > 1 else pred[:, 4:5]

    scores = cls_scores.max(axis=1)
    cls_ids = cls_scores.argmax(axis=1)

    # filter by conf
    m = scores >= CONF_TH
    boxes = boxes[m]
    scores = scores[m]
    cls_ids = cls_ids[m]

    if boxes.shape[0] == 0:
        print("No detections")
        cv2.imwrite("output.jpg", img0)
        return

    # convert cxcywh -> xyxy on padded image
    xyxy = np.zeros_like(boxes)
    xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2

    keep = nms(xyxy, scores, IOU_TH)
    xyxy = xyxy[keep]
    scores = scores[keep]
    cls_ids = cls_ids[keep]

    # scale back to original image
    for (x1, y1, x2, y2), sc in zip(xyxy, scores):
        x1 = (x1 - padw) / r
        y1 = (y1 - padh) / r
        x2 = (x2 - padw) / r
        y2 = (y2 - padh) / r

        x1 = int(np.clip(x1, 0, w0 - 1))
        y1 = int(np.clip(y1, 0, h0 - 1))
        x2 = int(np.clip(x2, 0, w0 - 1))
        y2 = int(np.clip(y2, 0, h0 - 1))

        cv2.rectangle(img0, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img0, f"valve {sc:.2f}", (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imwrite("output.jpg", img0)
    print("Saved -> output.jpg")

if __name__ == "__main__":
    main()
