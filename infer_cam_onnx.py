import cv2
import numpy as np
import time
from pathlib import Path
from picamera2 import Picamera2
import onnxruntime as ort

# ================= CONFIG =================
MODEL_PATH = "models/Valve_detection_model.onnx"
IMG_SIZE = 640
CONF_THRES = 0.4
SAVE_DIR = "captures"
# ==========================================

Path(SAVE_DIR).mkdir(exist_ok=True)

# Load ONNX model
print("Loading ONNX model...")
session = ort.InferenceSession(MODEL_PATH, providers=["CPUExecutionProvider"])

input_name = session.get_inputs()[0].name
output_name = session.get_outputs()[0].name

# Start camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 640)})
picam2.configure(config)
picam2.start()

time.sleep(2)

print("Running ONNX detect...")

while True:
    frame = picam2.capture_array()

    img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0

    # CHW format (สำคัญมากสำหรับ ONNX YOLO)
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)

    outputs = session.run([output_name], {input_name: img})
    output = outputs[0][0]

    found = False

    for det in output:
        conf = det[4]
        if conf > CONF_THRES:
            found = True
            x, y, w, h = det[:4]

            x1 = int((x - w/2) * frame.shape[1])
            y1 = int((y - h/2) * frame.shape[0])
            x2 = int((x + w/2) * frame.shape[1])
            y2 = int((y + h/2) * frame.shape[0])

            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.putText(frame, f"{conf:.2f}", (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    print("Detected:", found)
