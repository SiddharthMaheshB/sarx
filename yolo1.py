#!/usr/bin/env python3

import sys
import time
from pathlib import Path
import cv2
import torch
from picamera2 import Picamera2

YOLOV5_DIR = Path("/home/drone/Desktop/yolov5")   # yolov5 repo
WEIGHTS = "/home/drone/Desktop/final/weights_only.pt"
MODEL_YAML = "/home/drone/Desktop/yolov5/models/yolov5n.yaml"

IMG_SIZE = 320
CONF_THRES = 0.25
IOU_THRES = 0.45

sys.path.append(str(YOLOV5_DIR))

from models.yolo import Model
from utils.general import non_max_suppression, scale_boxes, check_yaml
from utils.torch_utils import select_device

device = select_device("cpu")

cfg = check_yaml(MODEL_YAML)
model = Model(cfg, ch=3, nc=1)   # nc=1 ? person
state_dict = torch.load(WEIGHTS, map_location="cpu")
model.load_state_dict(state_dict, strict=True)
model.to(device).eval()

names = ["person"]

print("? YOLOv5 model loaded from state_dict (CPU)")

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (640, 480)}
)
picam2.configure(config)
picam2.start()

print("?? Picamera started. Press 'q' to quit.")

while True:
    frame_rgb = picam2.capture_array()              # RGB
    frame_bgr = frame_rgb

    start = time.time()

    img = cv2.resize(frame_rgb, (IMG_SIZE, IMG_SIZE))
    img = img.transpose(2, 0, 1)                     # HWC ? CHW
    img = torch.from_numpy(img).to(device)
    img = img.float() / 255.0
    img = img.unsqueeze(0)

    with torch.no_grad():
        pred = model(img)

    pred = non_max_suppression(
        pred,
        conf_thres=CONF_THRES,
        iou_thres=IOU_THRES
    )

    for det in pred:
        if len(det):
            det[:, :4] = scale_boxes(
                img.shape[2:], det[:, :4], frame_bgr.shape
            ).round()

            for *xyxy, conf, cls in det:
                x1, y1, x2, y2 = map(int, xyxy)
                label = f"{names[int(cls)]} {conf:.2f}"

                cv2.rectangle(
                    frame_bgr,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )
                cv2.putText(
                    frame_bgr,
                    label,
                    (x1, y1 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

    inference_ms = (time.time() - start) * 1000
    fps = 1000 / inference_ms if inference_ms > 0 else 0

    cv2.putText(
        frame_bgr,
        f"PT CPU | {inference_ms:.1f} ms | {fps:.1f} FPS",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2
    )

    # Resize output display to 640x640
    frame_display = cv2.resize(frame_bgr, (640, 640))
    
    cv2.imshow("YOLOv5 PyTorch (Raspberry Pi)", frame_display)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

picam2.stop()
cv2.destroyAllWindows()
print("?? Stopped cleanly")
