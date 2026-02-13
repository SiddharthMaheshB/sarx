#!/usr/bin/env python3
"""
Computer Vision module for SARX system.
Handles YOLO model loading and inference.
"""

import sys
import cv2
import torch
import numpy as np
from pathlib import Path

# Import config
from config import (
    YOLOV5_DIR, WEIGHTS_PT, MODEL_YAML, FALLBACK_MODEL,
    CONF_THRES, IOU_THRES, IMG_SIZE, MODEL_NAMES
)

# Global model state
model = None
USE_PYTORCH = False
names = MODEL_NAMES.copy()


def load_model():
    """
    Load YOLOv5 model using PyTorch (optimized for CPU).
    Falls back to Ultralytics if PyTorch model not available.
    
    Returns:
        bool: True if model loaded successfully, False otherwise
    """
    global model, USE_PYTORCH, names
    
    # Try PyTorch direct loading first (optimized for Pi)
    if YOLOV5_DIR.exists() and WEIGHTS_PT.exists() and MODEL_YAML.exists():
        try:
            sys.path.insert(0, str(YOLOV5_DIR))
            from models.yolo import Model
            from utils.general import check_yaml
            from utils.torch_utils import select_device
            
            print("[MODEL] Loading YOLOv5 PyTorch model (optimized)...")
            device = select_device("cpu")
            cfg = check_yaml(str(MODEL_YAML))
            model = Model(cfg, ch=3, nc=1)
            state_dict = torch.load(str(WEIGHTS_PT), map_location="cpu")
            model.load_state_dict(state_dict, strict=True)
            model.to(device).eval()
            USE_PYTORCH = True
            names = MODEL_NAMES
            print("[MODEL] ✓ YOLOv5 PyTorch model loaded (CPU optimized)")
            return True
        except Exception as e:
            print(f"[MODEL] PyTorch loading failed: {e}")
            print("[MODEL] Falling back to Ultralytics YOLO...")
    
    # Fallback to Ultralytics
    try:
        print("couldnt load")
        # from ultralytics import YOLO
        # print(f"[MODEL] Loading Ultralytics YOLO: {FALLBACK_MODEL}")
        # model = YOLO(FALLBACK_MODEL)
        # USE_PYTORCH = False
        # names = model.names
        # print("[MODEL] ✓ Ultralytics YOLO loaded")
        # return True
        return False
    except Exception as e:
        print(f"[ERROR] Failed to load any model: {e}")
        return False


def infer_pytorch(frame, model_obj):
    """
    Run PyTorch YOLOv5 inference (optimized preprocessing).
    
    Args:
        frame: Input image (BGR format, any size)
        model_obj: PyTorch YOLOv5 model
        
    Returns:
        torch.Tensor: Detections [N, 6] with format (x1, y1, x2, y2, conf, cls)
    """
    orig_h, orig_w = frame.shape[:2]
    
    # Resize and preprocess
    img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
    img = img.transpose(2, 0, 1)  # HWC -> CHW
    img = torch.from_numpy(img).to(torch.device("cpu"))
    img = img.float() / 255.0
    img = img.unsqueeze(0)
    
    # Inference
    with torch.no_grad():
        pred = model_obj(img)
    
    # Post-process
    from utils.general import non_max_suppression, scale_boxes
    pred = non_max_suppression(pred, conf_thres=CONF_THRES, iou_thres=IOU_THRES)
    
    # Scale back to original frame dimensions
    det = pred[0] if len(pred) > 0 else torch.tensor([], device="cpu")
    if len(det) > 0:
        det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], (orig_h, orig_w)).round()
    
    return det


def draw_results(img, result, model_obj, use_pytorch=False):
    """
    Draw detection boxes on image (supports both PyTorch and Ultralytics).
    
    Args:
        img: Image to draw on (modified in-place)
        result: Detection results (tensor or Ultralytics result)
        model_obj: Model object (for getting class names)
        use_pytorch: Whether using PyTorch format
    """
    h, w = img.shape[:2]
    
    if use_pytorch:
        # PyTorch format: tensor [N, 6] (x1, y1, x2, y2, conf, cls)
        if result is not None and isinstance(result, torch.Tensor) and len(result) > 0:
            try:
                for *xyxy, conf, cls in result:
                    x1, y1, x2, y2 = map(float, xyxy)
                    x1_clamped = max(0, min(int(x1), w - 1))
                    y1_clamped = max(0, min(int(y1), h - 1))
                    x2_clamped = max(0, min(int(x2), w - 1))
                    y2_clamped = max(0, min(int(y2), h - 1))
                    
                    if x2_clamped > x1_clamped and y2_clamped > y1_clamped:
                        label = f"{names[int(cls)]} {conf:.2f}"
                        color = (0, 255, 0) if names[int(cls)].lower() == "person" else (255, 0, 0)
                        cv2.rectangle(img, (x1_clamped, y1_clamped), (x2_clamped, y2_clamped), color, 2)
                        cv2.putText(img, label, (x1_clamped, max(10, y1_clamped-6)), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            except Exception as e:
                print(f"[WARNING] Error drawing PyTorch results: {e}")
    else:
        # Ultralytics format (placeholder - not implemented)
        print("Ultralytics drawing not implemented")


def get_person_info(result, model_obj, orig_w, orig_h, conf_thresh=0.3, use_pytorch=False):
    """
    Get largest person detection info: area ratio and center offset from image center.
    
    Args:
        result: Detection results
        model_obj: Model object
        orig_w: Original image width
        orig_h: Original image height
        conf_thresh: Confidence threshold
        use_pytorch: Whether using PyTorch format
        
    Returns:
        tuple: (area_ratio, center_x_offset, center_y_offset, found)
            - area_ratio: Detection area / image area
            - center_x_offset: Normalized offset from center (-1 to 1)
            - center_y_offset: Normalized offset from center (-1 to 1)
            - found: Whether person was detected
    """
    if use_pytorch:
        # PyTorch format: tensor [N, 6] (x1, y1, x2, y2, conf, cls)
        if result is None or not isinstance(result, torch.Tensor) or len(result) == 0:
            return 0.0, 0.0, 0.0, False
        
        max_area = 0.0
        max_center_x = 0.0
        max_center_y = 0.0
        found = False
        
        for *xyxy, conf, cls in result:
            x1, y1, x2, y2 = map(float, xyxy)
            
            # Check if person
            if int(cls) == 0 and conf >= conf_thresh:  # Class 0 is person
                w = max(0.0, x2 - x1)
                h = max(0.0, y2 - y1)
                area = w * h
                
                if area > max_area:
                    max_area = area
                    center_x = (x1 + x2) / 2.0
                    center_y = (y1 + y2) / 2.0
                    max_center_x = (center_x - orig_w / 2.0) / (orig_w / 2.0)
                    max_center_y = (center_y - orig_h / 2.0) / (orig_h / 2.0)
                    found = True
        
        area_ratio = max_area / (orig_w * orig_h) if orig_w * orig_h > 0 else 0.0
        return area_ratio, max_center_x, max_center_y, found
    
    else:
        # Ultralytics format (placeholder)
        if result is None:
            return 0.0, 0.0, 0.0, False
        print("Ultralytics person detection not implemented")
        return 0.0, 0.0, 0.0, False


def main():
    """Test CV module by loading model and running inference on a test image"""
    print("="*60)
    print("CV MODULE TEST")
    print("="*60)
    
    # Test model loading
    print("\n[TEST] Loading model...")
    if load_model():
        print("✓ Model loaded successfully")
        print(f"  Using PyTorch: {USE_PYTORCH}")
        print(f"  Model names: {names}")
    else:
        print("✗ Model loading failed")
        return
    
    # Test inference on a blank image
    print("\n[TEST] Testing inference on blank image...")
    test_img = np.zeros((640, 480, 3), dtype=np.uint8)
    
    if USE_PYTORCH:
        result = infer_pytorch(test_img, model)
        print(f"✓ Inference complete")
        print(f"  Detections: {len(result) if result is not None else 0}")
        
        # Test person info extraction
        area, cx, cy, found = get_person_info(result, model, 480, 640, use_pytorch=True)
        print(f"  Person found: {found}")
        if found:
            print(f"  Area ratio: {area:.3f}")
            print(f"  Center offset: ({cx:.2f}, {cy:.2f})")
    
    print("\n" + "="*60)


if __name__ == "__main__":
    main()
