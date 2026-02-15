import cv2
import numpy as np
from pathlib import Path

MODEL_PATH = "models/Valve_detection_model.onnx"
IMAGE_PATH = "test_images/IMG_1138.JPG"

CONF_TH = 0.20
IOU_TH = 0.45
IMG_SIZE = 640

def letterbox(im, new_shape=640, color=(114,114,114)):
    h, w = im.shape[:2]
    r = min(new_shape / h, new_shape / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    im_resized = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top = (new_shape - nh) // 2
    bottom = new_shape - nh - top
    left = (new_shape - nw) // 2
    right = new_shape - nw - left
    im_padded = cv2.copyMakeBorder(im_resized, top, bottom, left, right,
                                   cv2.BORDER_CONSTANT, value=color)
    return im_padded, r, left, top

def nms(boxes, scores, iou_th=0.45):
    idxs = scores.argsort()[::-1]
    keep = []
    while idxs.size:
        i = idxs[0]
        keep.append(i)
        if idxs.size == 1:
            break
        rest = idxs[1:]

        xx1 = np.maximum(boxes[i,0], boxes[rest,0])
        yy1 = np.maximum(boxes[i,1], boxes[rest,1])
        xx2 = np.minimum(boxes[i,2], boxes[rest,2])
        yy2 = np.minimum(boxes[i,3], boxes[rest,3])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h

        area_i = (boxes[i,2]-boxes[i,0]) * (boxes[i,3]-boxes[i,1])
        area_r = (boxes[rest,2]-boxes[rest,0]) * (boxes[rest,3]-boxes[rest,1])
        iou = inter / (area_i + area_r - inter + 1e-6)

        idxs = rest[iou < iou_th]
    return keep

def main():
    assert Path(MODEL_PATH).exists(), f"Model not found: {MODEL_PATH}"
    assert Path(IMAGE_PATH).exists(), f"Image not found: {IMAGE_PATH}"

    img0 = cv2.imread(IMAGE_PATH)
    h0, w0 = img0.shape[:2]

    img, r, padw, padh = letterbox(img0, IMG_SIZE)
    blob = cv2.dnn.blobFromImage(img, 1/255.0, (IMG_SIZE, IMG_SIZE), swapRB=True, crop=False)

    net = cv2.dnn.readNetFromONNX(MODEL_PATH)
    net.setInput(blob)
    out = net.forward()  # มักได้ shape (1, 5, 8400) สำหรับ class เดียว

    # normalize -> (N, C)
    if out.ndim == 3 and out.shape[1] < out.shape[2]:
        out = out.transpose(0, 2, 1)  # (1, 8400, 5)
    pred = out[0]

    boxes = pred[:, :4]          # cx,cy,w,h
    cls_scores = pred[:, 4:5]    # class เดียว = valve
    scores = cls_scores[:, 0]

    m = scores >= CONF_TH
    boxes, scores = boxes[m], scores[m]

    if len(scores) == 0:
        print("No detections")
        cv2.imwrite("output.jpg", img0)
        return

    xyxy = np.zeros_like(boxes)
    xyxy[:,0] = boxes[:,0] - boxes[:,2]/2
    xyxy[:,1] = boxes[:,1] - boxes[:,3]/2
    xyxy[:,2] = boxes[:,0] + boxes[:,2]/2
    xyxy[:,3] = boxes[:,1] + boxes[:,3]/2

    keep = nms(xyxy, scores, IOU_TH)
    xyxy, scores = xyxy[keep], scores[keep]

    for (x1,y1,x2,y2), sc in zip(xyxy, scores):
        x1 = int(np.clip((x1 - padw)/r, 0, w0-1))
        y1 = int(np.clip((y1 - padh)/r, 0, h0-1))
        x2 = int(np.clip((x2 - padw)/r, 0, w0-1))
        y2 = int(np.clip((y2 - padh)/r, 0, h0-1))

        cv2.rectangle(img0, (x1,y1), (x2,y2), (0,255,0), 2)
        cv2.putText(img0, f"valve {sc:.2f}", (x1, max(0,y1-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    out_path = Path("output.jpg").resolve()
    cv2.imwrite(str(out_path), img0)
    print("Saved ->", out_path)


if __name__ == "__main__":
    main()
