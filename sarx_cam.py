#!/usr/bin/env python3
"""
sarx_camera_test.py

Camera-only testing version of SARX system.
This allows testing the detection logic and state machine without connecting to a drone.
Hold the cameras and move them according to the printed directions.

Optimizations applied:
- PyTorch direct model loading (yolo1.py approach) for CPU optimization
- Manual preprocessing with normalized float tensors
- Direct non_max_suppression instead of wrapper
- Proper tensor device management
- Fallback to Ultralytics YOLO if PyTorch model not available

Usage: python3 sarx_camera_test.py
"""

import sys
import time
import cv2
import torch
import numpy as np
from datetime import datetime
from pathlib import Path
from picamera2 import Picamera2

# ============ Configuration ============
CAM_NATIVE_SIZE = (3280, 2464)  # IMX219 full resolution (max FOV)
CAM_PREVIEW_SIZE = (1280, 720)  # Use a high-res preview for best FOV and speed
INFER_SIZE = 320
DISPLAY_WINDOW = "SARX Camera Test"
FPS_AVG_ALPHA = 0.9
INFER_SIZE = 320
DISPLAY_WINDOW = "SARX Camera Test"
FPS_AVG_ALPHA = 0.9

# Model paths (try Pi paths first, then fallback)
YOLOV5_DIR = Path("/home/drone/Desktop/yolov5")
WEIGHTS_PT = Path("/home/drone/Desktop/final/weights_only.pt")
MODEL_YAML = Path("/home/drone/Desktop/yolov5/models/yolov5n.yaml")
FALLBACK_MODEL = "yolo11n.pt"

# Inference settings
IMG_SIZE = 320
CONF_THRES = 0.25
IOU_THRES = 0.45
USE_PYTORCH = False
model = None
names = ["person"]

# Detection thresholds
PERSON_CONF_THR = 0.3
PERSON_AREA_THRESHOLD_FRONT = 0.15  # Front camera threshold to trigger approach
PERSON_AREA_THRESHOLD_BOTTOM = 0.4  # Bottom camera threshold for centered position

# Movement parameters (for printing only)
APPROACH_SPEED = 0.8  # m/s
YAW_RATE = 30.0  # deg/s

# State timing
DESCEND_TIME = 6.0
DROP_TIME = 2.0
RETURN_TIME = 5.0
CENTERING_TIMEOUT = 15.0  # Max time in CENTERING state before timeout


# ============ State Machine States ============
class State:
    SEARCHING = "SEARCHING"
    APPROACHING = "APPROACHING"
    CENTERING = "CENTERING"
    DESCENDING = "DESCENDING"
    DROPPING = "DROPPING"
    RETURNING = "RETURNING"


# ============ Simulated Drone Controller ============
class SimulatedDroneController:
    """Simulated drone controller that prints directions instead of moving"""
    
    def __init__(self):
        self.checkpoint_saved = False
        self.checkpoint_time = None
        
    def save_checkpoint(self):
        """Simulate saving checkpoint"""
        self.checkpoint_saved = True
        self.checkpoint_time = time.time()
        print("\n" + "="*60)
        print("📍 [CHECKPOINT] Position saved!")
        print("="*60)
        return True
    
    def move_forward(self, speed=APPROACH_SPEED, duration=0.5):
        """Print forward movement instruction"""
        distance = speed * duration
        print(f"➡️  [MOVE] Move camera FORWARD {distance:.2f}m (speed: {speed:.1f}m/s)")
    
    def move_with_yaw(self, forward=0.0, right=0.0, down=0.0, yaw_rate=0.0, duration=0.5):
        """Print movement and yaw instructions"""
        directions = []
        
        if forward != 0:
            dir_str = "FORWARD" if forward > 0 else "BACKWARD"
            directions.append(f"{dir_str} {abs(forward*duration):.2f}m")
        
        if right != 0:
            dir_str = "RIGHT" if right > 0 else "LEFT"
            directions.append(f"{dir_str} {abs(right*duration):.2f}m")
        
        if down != 0:
            dir_str = "DOWN" if down > 0 else "UP"
            directions.append(f"{dir_str} {abs(down*duration):.2f}m")
        
        if yaw_rate != 0:
            dir_str = "CLOCKWISE" if yaw_rate > 0 else "COUNTER-CLOCKWISE"
            directions.append(f"YAW {dir_str} {abs(yaw_rate):.1f}°/s")
        
        if directions:
            print(f"🎯 [MOVE] {' + '.join(directions)}")
    
    def yaw_towards(self, yaw_rate_deg=YAW_RATE, duration=0.5):
        """Print yaw instruction"""
        dir_str = "RIGHT" if yaw_rate_deg > 0 else "LEFT"
        print(f"🔄 [YAW] Turn camera {dir_str} at {abs(yaw_rate_deg):.1f}°/s")
    
    def descend_to_delivery_height(self):
        """Print descent instruction"""
        print("\n" + "="*60)
        print("⬇️  [DESCEND] Lower camera to delivery height (1.5m from ground)")
        print("="*60)
    
    def return_to_checkpoint(self):
        """Print return instruction"""
        print("\n" + "="*60)
        print("🔙 [RETURN] Move camera back to original checkpoint position")
        print("    - First: Raise to original height")
        print("    - Then: Move backward to starting point")
        print("="*60)


# ============ Model Loading ============
def load_model():
    """
    Load YOLOv5 model using PyTorch (optimized for CPU).
    Falls back to Ultralytics if PyTorch model not available.
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
            names = ["person"]
            print("[MODEL] ✅ YOLOv5 PyTorch model loaded (CPU optimized)")
            return True
        except Exception as e:
            print(f"[MODEL] ⚠️  PyTorch loading failed: {e}")
            print("[MODEL] Falling back to Ultralytics YOLO...")
    
    # Fallback to Ultralytics
    try:
        from ultralytics import YOLO
        print(f"[MODEL] Loading Ultralytics YOLO: {FALLBACK_MODEL}")
        model = YOLO(FALLBACK_MODEL)
        USE_PYTORCH = False
        names = model.names
        print("[MODEL] ✅ Ultralytics YOLO loaded")
        return True
    except Exception as e:
        print(f"[ERROR] Failed to load any model: {e}")
        return False


def infer_pytorch(frame, model):
    """
    Run PyTorch YOLOv5 inference (optimized preprocessing).
    Returns detections tensor [N, 6] with format (x1, y1, x2, y2, conf, cls) or empty tensor.
    Input frame should be original size, will be resized internally.
    """
    orig_h, orig_w = frame.shape[:2]  # Original frame dimensions
    
    # Resize and preprocess
    img = cv2.resize(frame, (IMG_SIZE, IMG_SIZE))
    img = img.transpose(2, 0, 1)  # HWC -> CHW
    img = torch.from_numpy(img).to(torch.device("cpu"))
    img = img.float() / 255.0
    img = img.unsqueeze(0)
    
    # Inference
    with torch.no_grad():
        pred = model(img)
    
    # Post-process
    from utils.general import non_max_suppression, scale_boxes
    pred = non_max_suppression(pred, conf_thres=CONF_THRES, iou_thres=IOU_THRES)
    
    # Scale back to ORIGINAL frame dimensions (not resized 320x320)
    det = pred[0] if len(pred) > 0 else torch.tensor([], device="cpu")
    if len(det) > 0:
        det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], (orig_h, orig_w)).round()
    
    return det


def infer_ultralytics(frame, model):
    """Run Ultralytics YOLO inference and return results."""
    results = model(frame, imgsz=IMG_SIZE, verbose=False)
    return results


# ============ Detection Functions ============
def draw_results(img, result, use_pytorch=False):
    """Draw detection boxes on image (supports both PyTorch and Ultralytics)"""
    h, w = img.shape[:2]  # Get image dimensions for clamping
    
    if use_pytorch:
        # PyTorch format: tensor [N, 6] (x1, y1, x2, y2, conf, cls)
        if result is not None and isinstance(result, torch.Tensor) and len(result) > 0:
            try:
                for *xyxy, conf, cls in result:
                    x1, y1, x2, y2 = map(float, xyxy)
                        
                    # Clamp ONLY for drawing - don't modify original values
                    x1_clamped = max(0, min(int(x1), w - 1))
                    y1_clamped = max(0, min(int(y1), h - 1))
                    x2_clamped = max(0, min(int(x2), w - 1))
                    y2_clamped = max(0, min(int(y2), h - 1))
                    
                    # Only draw if box has valid dimensions
                    if x2_clamped > x1_clamped and y2_clamped > y1_clamped:
                        label = f"{names[int(cls)]} {conf:.2f}"
                        color = (0, 255, 0) if names[int(cls)].lower() == "person" else (255, 0, 0)
                        cv2.rectangle(img, (x1_clamped, y1_clamped), (x2_clamped, y2_clamped), color, 2)
                        cv2.putText(img, label, (x1_clamped, max(10, y1_clamped-6)), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            except Exception as e:
                print(f"[WARNING] Error drawing PyTorch results: {e}")
    else:
        # Ultralytics format
        try:
            boxes = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            clsids = result.boxes.cls.cpu().numpy().astype(int)
        except Exception:
            boxes = np.array(result.boxes.xyxy).astype(np.float32)
            confs = np.array(result.boxes.conf).astype(np.float32)
            clsids = np.array(result.boxes.cls).astype(np.int32)
        
        for (x1, y1, x2, y2), c, cid in zip(boxes, confs, clsids):
            try:
                label_name = result.names[int(cid)] if hasattr(result, 'names') else names[int(cid)]
            except Exception:
                label_name = str(int(cid))
            
            # Clamp ONLY for drawing - keep original values for calculations
            x1_clamped = max(0, min(int(x1), w - 1))
            y1_clamped = max(0, min(int(y1), h - 1))
            x2_clamped = max(0, min(int(x2), w - 1))
            y2_clamped = max(0, min(int(y2), h - 1))
            
            # Only draw if box has valid dimensions
            if x2_clamped > x1_clamped and y2_clamped > y1_clamped:
                label = f"{label_name} {c:.2f}"
                color = (0, 255, 0) if label_name.lower() == "person" else (255, 0, 0)
                cv2.rectangle(img, (x1_clamped, y1_clamped), (x2_clamped, y2_clamped), color, 2)
                cv2.putText(img, label, (x1_clamped, max(10, y1_clamped-6)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


def get_person_info(result, use_pytorch=False, orig_w=640, orig_h=480, conf_thresh=PERSON_CONF_THR):
    """
    Get largest person detection info: area ratio and center offset from image center.
    Supports both PyTorch and Ultralytics result formats.
    Returns: (area_ratio, center_x_offset, center_y_offset, found)
    """
    if use_pytorch:
        # PyTorch format: tensor [N, 6] (x1, y1, x2, y2, conf, cls)
        if result is None or not isinstance(result, torch.Tensor) or len(result) == 0:
            return 0.0, 0.0, 0.0, False
        
        max_area = 0.0
        max_center_x = 0.0
        max_center_y = 0.0
        found = False
        
        try:
            for *xyxy, conf, cls in result:
                if conf < conf_thresh or int(cls) != 0:  # Only person class
                    continue
                
                x1, y1, x2, y2 = map(float, xyxy)
                
                # Skip boxes that are completely outside image bounds
                if x2 < 0 or y2 < 0 or x1 > orig_w or y1 > orig_h:
                    continue
                
                # Clamp to valid bounds for area calculation
                x1_valid = max(0.0, min(float(x1), orig_w))
                y1_valid = max(0.0, min(float(y1), orig_h))
                x2_valid = max(0.0, min(float(x2), orig_w))
                y2_valid = max(0.0, min(float(y2), orig_h))
                
                w = max(0.0, x2_valid - x1_valid)
                h = max(0.0, y2_valid - y1_valid)
                area = w * h
                
                if area > max_area:
                    max_area = area
                    # Use clamped coordinates for center calculation
                    center_x = (x1_valid + x2_valid) / 2.0
                    center_y = (y1_valid + y2_valid) / 2.0
                    max_center_x = (center_x - orig_w / 2.0) / (orig_w / 2.0)
                    max_center_y = (center_y - orig_h / 2.0) / (orig_h / 2.0)
                    found = True
        except Exception as e:
            print(f"[WARNING] Error processing PyTorch detections: {e}")
            return 0.0, 0.0, 0.0, False
    else:
        # Ultralytics format - same bounds checking
        try:
            boxes = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            clsids = result.boxes.cls.cpu().numpy().astype(int)
        except Exception:
            boxes = np.array(result.boxes.xyxy).astype(np.float32)
            confs = np.array(result.boxes.conf).astype(np.float32)
            clsids = np.array(result.boxes.cls).astype(np.int32)
        
        if boxes.size == 0:
            return 0.0, 0.0, 0.0, False
        
        max_area = 0.0
        max_center_x = 0.0
        max_center_y = 0.0
        found = False
        
        for (x1, y1, x2, y2), conf, cid in zip(boxes, confs, clsids):
            # Check if person
            is_person = False
            try:
                name = result.names[int(cid)].lower() if hasattr(result, 'names') else names[int(cid)].lower()
                if name == "person":
                    is_person = True
            except Exception:
                if int(cid) == 0:  # COCO person class
                    is_person = True
            
            if not is_person or conf < conf_thresh:
                continue
            
            # Skip boxes that are completely outside image bounds
            if x2 < 0 or y2 < 0 or x1 > orig_w or y1 > orig_h:
                continue
            
            # Clamp to valid bounds for area calculation
            x1_valid = max(0.0, min(float(x1), orig_w))
            y1_valid = max(0.0, min(float(y1), orig_h))
            x2_valid = max(0.0, min(float(x2), orig_w))
            y2_valid = max(0.0, min(float(y2), orig_h))
            
            w = max(0.0, x2_valid - x1_valid)
            h = max(0.0, y2_valid - y1_valid)
            area = w * h
            
            if area > max_area:
                max_area = area
                # Use clamped coordinates for center calculation
                center_x = (x1_valid + x2_valid) / 2.0
                center_y = (y1_valid + y2_valid) / 2.0
                max_center_x = (center_x - orig_w / 2.0) / (orig_w / 2.0)
                max_center_y = (center_y - orig_h / 2.0) / (orig_h / 2.0)
                found = True
    
    area_ratio = max_area / (orig_w * orig_h) if orig_w * orig_h > 0 else 0.0
    return area_ratio, max_center_x, max_center_y, found


def print_state_change(old_state, new_state):
    """Print state transition with visual separator"""
    print("\n" + "="*60)
    print(f"🔄 STATE TRANSITION: {old_state} → {new_state}")
    print("="*60 + "\n")


# ============ Main Program ============
def main():
    print("="*60)
    print("🎥 SARX CAMERA TEST MODE")
    print("="*60)
    print("\nThis is a simulation mode for testing camera detection.")
    print("Hold the cameras and move them according to instructions.")
    print("Press 'q' to quit.\n")
    
    # Load model
    if not load_model():
        print("[ERROR] Failed to load model. Exiting.")
        return
    
    # Initialize cameras for max FOV
    print("[CAMERA] Initializing IMX219 cameras for max FOV...")
    cam0 = Picamera2(0)
    cam1 = Picamera2(1)
    # Use preview size that matches wide FOV and is fast enough for inference
    cfg0 = cam0.create_preview_configuration(main={"size": CAM_PREVIEW_SIZE, "format": "BGR888"})
    cfg1 = cam1.create_preview_configuration(main={"size": CAM_PREVIEW_SIZE, "format": "BGR888"})
    cam0.configure(cfg0)
    cam1.configure(cfg1)
    cam0.start()
    cam1.start()
    time.sleep(1)
    print(f"[CAMERA]  Cameras ready at {CAM_PREVIEW_SIZE} (IMX219 wide FOV)\n")
    
    # Initialize simulated drone controller
    drone = SimulatedDroneController()
    
    # State machine
    current_state = State.SEARCHING
    fps = 0.0
    last_time = time.time()
    
    # State timing for simulated operations
    state_start_time = time.time()
    
    print("="*60)
    print(" STARTING DETECTION LOOP")
    print("="*60)
    print(f"Initial State: {current_state}\n")
    
    try:
        while True:
            # Capture frames from both cameras
            frame0_rgb = cam0.capture_array()
            frame1_rgb = cam1.capture_array()
            
            frame0 = cv2.cvtColor(frame0_rgb, cv2.COLOR_RGB2BGR)  # Bottom
            frame1 = cv2.cvtColor(frame1_rgb, cv2.COLOR_RGB2BGR)  # Front
            
            vis0 = frame0.copy()
            vis1 = frame1.copy()
            
            # Get frame dimensions (should match preview size for max FOV)
            h0, w0 = frame0.shape[:2]
            h1, w1 = frame1.shape[:2]
            # If you want to use the full sensor, you can use capture_image() with CAM_NATIVE_SIZE, but preview is faster for real-time
            
            # Run inference (PyTorch optimized or Ultralytics)
            # Pass original full-size frames, only resize for model input, display uncropped
            if USE_PYTORCH:
                # For PyTorch, infer_pytorch resizes internally, so just pass the full frame
                result0 = infer_pytorch(frame0, model)
                result1 = infer_pytorch(frame1, model)
                draw_results(vis0, result0, use_pytorch=True)
                draw_results(vis1, result1, use_pytorch=True)
            else:
                # For Ultralytics, resize for model input but display uncropped
                h0, w0 = frame0.shape[:2]
                h1, w1 = frame1.shape[:2]
                # Resize to model input size, but keep original for display
                f0_in = cv2.resize(frame0, (INFER_SIZE, INFER_SIZE))
                f1_in = cv2.resize(frame1, (INFER_SIZE, INFER_SIZE))
                results = model([f0_in, f1_in], imgsz=INFER_SIZE, verbose=False)
                # The results boxes are in the resized image coordinates, so we need to scale them back to original size for drawing
                def scale_boxes_back(result, orig_w, orig_h):
                    # Scale detection boxes from model input size back to original image size
                    try:
                        boxes = result.boxes.xyxy.cpu().numpy()
                        scale_x = orig_w / INFER_SIZE
                        scale_y = orig_h / INFER_SIZE
                        boxes[:, [0, 2]] *= scale_x
                        boxes[:, [1, 3]] *= scale_y
                        result.boxes.xyxy = type(result.boxes.xyxy)(boxes)
                    except Exception:
                        pass
                if len(results) >= 1:
                    scale_boxes_back(results[0], w0, h0)
                    draw_results(vis0, results[0], use_pytorch=False)
                if len(results) >= 2:
                    scale_boxes_back(results[1], w1, h1)
                    draw_results(vis1, results[1], use_pytorch=False)
                result0 = results[0] if len(results) >= 1 else None
                result1 = results[1] if len(results) >= 2 else None
            
            # Get detection info (use original frame dimensions)
            
            if USE_PYTORCH:
                area_bottom, cx_bottom, cy_bottom, found_bottom = get_person_info(result0, use_pytorch=True, orig_w=w0, orig_h=h0)
                area_front, cx_front, cy_front, found_front = get_person_info(result1, use_pytorch=True, orig_w=w1, orig_h=h1)
            else:
                area_bottom, cx_bottom, cy_bottom, found_bottom = get_person_info(result0, use_pytorch=False, orig_w=w0, orig_h=h0)
                area_front, cx_front, cy_front, found_front = get_person_info(result1, use_pytorch=False, orig_w=w1, orig_h=h1)
            
            # ========== State Machine Logic ==========
            old_state = current_state
            
            if current_state == State.SEARCHING:
                # Looking for human in front camera
                if found_front and area_front > PERSON_AREA_THRESHOLD_FRONT:
                    print(f"\n🎯 [DETECTION] Human detected in FRONT camera!")
                    print(f"   Area: {area_front:.3f} (threshold: {PERSON_AREA_THRESHOLD_FRONT})")
                    print(f"   Position: ({cx_front:.2f}, {cy_front:.2f})")
                    if drone.save_checkpoint():
                        current_state = State.APPROACHING
                        state_start_time = time.time()
            
            elif current_state == State.APPROACHING:
                # Approach the human using front camera
                if found_front:
                    # Yaw towards human (based on horizontal offset)
                    yaw_rate = cx_front * YAW_RATE  # Proportional control
                    
                    # Move forward while adjusting yaw
                    drone.move_with_yaw(
                        forward=APPROACH_SPEED,
                        right=0.0,
                        down=0.0,
                        yaw_rate=yaw_rate,
                        duration=0.3
                    )
                    
                    # Check if human visible in bottom camera
                    if found_bottom and area_bottom > 0.05:
                        print(f"\n [DETECTION] Human now visible in BOTTOM camera!")
                        print(f"   Area: {area_bottom:.3f}")
                        current_state = State.CENTERING
                        state_start_time = time.time()
                else:
                    # Lost target, return to searching
                    print("\n⚠️  [WARNING] Lost target in front camera!")
                    print("   Returning to search mode...")
                    current_state = State.SEARCHING
                    state_start_time = time.time()
            
            elif current_state == State.CENTERING:
                # Use bottom camera to center over person
                elapsed = time.time() - state_start_time
                
                if elapsed > CENTERING_TIMEOUT:
                    # Timeout - centering took too long
                    print(f"\n⏱️  [TIMEOUT] Centering timeout after {elapsed:.1f}s!")
                    print("   Returning to APPROACHING state...")
                    current_state = State.APPROACHING
                    state_start_time = time.time()
                
                elif found_bottom:
                    # Proportional control to center over person
                    forward_vel = -cy_bottom * 0.5  # Negative because +y is down in image
                    right_vel = cx_bottom * 0.5
                    
                    # Check if centered (within threshold)
                    if abs(cx_bottom) < 0.15 and abs(cy_bottom) < 0.15:
                        # Person is centered horizontally and vertically
                        if area_bottom > PERSON_AREA_THRESHOLD_BOTTOM:
                            # Ready to descend and deliver
                            print(f"\n✅ [CENTERED] Person is centered in bottom camera!")
                            print(f"   Area: {area_bottom:.3f} (threshold: {PERSON_AREA_THRESHOLD_BOTTOM})")
                            print(f"   Position offset: ({cx_bottom:.2f}, {cy_bottom:.2f})")
                            print(f"   Time to center: {elapsed:.1f}s")
                            current_state = State.DESCENDING
                            state_start_time = time.time()
                        else:
                            # Centered but too far away - descend slowly to get closer
                            if elapsed % 1 < 0.1:  # Log every second
                                print(f"📐 [CENTERING] Centered position reached, descending to target size...")
                                print(f"   Current area: {area_bottom:.3f} → Target: {PERSON_AREA_THRESHOLD_BOTTOM:.3f}")
                            drone.move_with_yaw(
                                forward=0.0,
                                right=0.0,
                                down=0.3,  # Descend faster when centered
                                yaw_rate=0.0,
                                duration=0.3
                            )
                    else:
                        # Not centered - adjust position
                        # Log movement commands
                        directions = []
                        if abs(right_vel) > 0.05:
                            directions.append(f"RIGHT {right_vel:.2f}" if right_vel > 0 else f"LEFT {abs(right_vel):.2f}")
                        if abs(forward_vel) > 0.05:
                            directions.append(f"{'FORWARD' if forward_vel > 0 else 'BACKWARD'} {abs(forward_vel):.2f}")
                        
                        if directions or elapsed % 2 < 0.1:  # Log periodically
                            print(f"🎯 [CENTERING] Adjusting position (offset: x={cx_bottom:.2f}, y={cy_bottom:.2f})")
                            if directions:
                                print(f"   Moving: {', '.join(directions)}")
                        
                        drone.move_with_yaw(
                            forward=forward_vel,
                            right=right_vel,
                            down=0.0,
                            yaw_rate=0.0,
                            duration=0.3
                        )
                else:
                    # Lost bottom view - try to recover by moving upward
                    if elapsed % 1 < 0.1:  # Log every second
                        print(f"⚠️  [WARNING] Lost target in bottom camera!")
                        print(f"   Attempting recovery by ascending...")
                    
                    # Move upward to regain view
                    drone.move_with_yaw(
                        forward=0.0,
                        right=0.0,
                        down=-0.3,  # Move up (negative down = up)
                        yaw_rate=0.0,
                        duration=0.3
                    )
                    
                    # If lost for too long, abort centering
                    if elapsed > CENTERING_TIMEOUT / 2:
                        print(f"\n❌ [ABORT] Lost target in bottom camera for {elapsed:.1f}s")
                        print("   Returning to APPROACHING state...")
                        current_state = State.APPROACHING
                        state_start_time = time.time()
            
            elif current_state == State.DESCENDING:
                # Simulate descent time
                elapsed = time.time() - state_start_time
                if elapsed < DESCEND_TIME:
                    remaining = DESCEND_TIME - elapsed
                    print(f"⬇️  [DESCENDING] Lowering to delivery height... ({remaining:.1f}s remaining)")
                    time.sleep(1)  # Print every second
                else:
                    drone.descend_to_delivery_height()
                    current_state = State.DROPPING
                    state_start_time = time.time()
            
            elif current_state == State.DROPPING:
                # Simulate drop time
                elapsed = time.time() - state_start_time
                if elapsed < DROP_TIME:
                    print("📦 [DROPPING] Releasing payload...")
                    time.sleep(1)
                else:
                    print("\n" + "="*60)
                    print("✅ [PAYLOAD] Payload dropped successfully!")
                    print("="*60)
                    current_state = State.RETURNING
                    state_start_time = time.time()
            
            elif current_state == State.RETURNING:
                # Simulate return time
                elapsed = time.time() - state_start_time
                if elapsed < RETURN_TIME:
                    remaining = RETURN_TIME - elapsed
                    print(f"🔙 [RETURNING] Moving back to checkpoint... ({remaining:.1f}s remaining)")
                    time.sleep(1)
                else:
                    drone.return_to_checkpoint()
                    print("\n" + "="*60)
                    print("✅ [COMPLETE] Delivery mission complete!")
                    print("   Resuming search for next target...")
                    print("="*60)
                    current_state = State.SEARCHING
                    state_start_time = time.time()
            
            # Print state transition if changed
            if old_state != current_state:
                print_state_change(old_state, current_state)
            
            # ========== Visualization ==========
            # Add state info to display
            state_text = f"State: {current_state}"
            state_color = {
                State.SEARCHING: (100, 100, 100),
                State.APPROACHING: (0, 255, 255),
                State.CENTERING: (255, 165, 0),
                State.DESCENDING: (255, 255, 0),
                State.DROPPING: (0, 255, 0),
                State.RETURNING: (255, 0, 255)
            }.get(current_state, (255, 255, 255))
            
            cv2.putText(vis0, state_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
            cv2.putText(vis1, state_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)
            
            # Add camera labels
            cv2.putText(vis0, "BOTTOM CAM", (10, h0-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(vis1, "FRONT CAM", (10, h1-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Add detection info
            if found_bottom:
                bottom_text = f"Person: {area_bottom:.3f} ({cx_bottom:.2f}, {cy_bottom:.2f})"
                cv2.putText(vis0, bottom_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                cv2.putText(vis0, "No person detected", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            if found_front:
                front_text = f"Person: {area_front:.3f} ({cx_front:.2f}, {cy_front:.2f})"
                cv2.putText(vis1, front_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                cv2.putText(vis1, "No person detected", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Draw crosshair on bottom camera for centering reference
            if current_state in [State.CENTERING, State.DESCENDING]:
                center_x = w0 // 2
                center_y = h0 // 2
                cv2.line(vis0, (center_x - 30, center_y), (center_x + 30, center_y), (0, 255, 0), 2)
                cv2.line(vis0, (center_x, center_y - 30), (center_x, center_y + 30), (0, 255, 0), 2)
                cv2.circle(vis0, (center_x, center_y), 50, (0, 255, 0), 2)
            
            # Combine views for display, keep aspect ratio and wide FOV
            target_h = 480  # Higher display height for wide FOV
            vis0_resized = cv2.resize(vis0, (int(w0 * target_h / h0), target_h))
            vis1_resized = cv2.resize(vis1, (int(w1 * target_h / h1), target_h))
            combined = np.hstack((vis0_resized, vis1_resized))
            
            # Calculate and display FPS
            now = time.time()
            loop_time = now - last_time
            last_time = now
            cur_fps = 1.0 / loop_time if loop_time > 0 else 0.0
            fps = FPS_AVG_ALPHA * fps + (1 - FPS_AVG_ALPHA) * cur_fps
            
            cv2.putText(combined, f"FPS: {fps:.1f}", (10, combined.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Display
            cv2.imshow(DISPLAY_WINDOW, combined)
            
            # Check for quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n[SYSTEM] Quit requested")
                break
    
    except KeyboardInterrupt:
        print("\n[SYSTEM] Interrupted by user")
    
    finally:
        print("\n[CLEANUP] Shutting down...")
        
        # Stop cameras
        cam0.stop()
        cam1.stop()
        cv2.destroyAllWindows()
        
        print("[CLEANUP] ✅ Complete. Goodbye!\n")


if __name__ == "__main__":
    main()
