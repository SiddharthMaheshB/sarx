#!/usr/bin/env python3
"""
sarx_camera_test.py

Camera-only testing version of SARX system.
This allows testing the detection logic and state machine without connecting to a drone.
Hold the cameras and move them according to the printed directions.

Usage: python3 sarx_camera_test.py
"""

import time
import cv2
import numpy as np
from datetime import datetime
from picamera2 import Picamera2
from ultralytics import YOLO

# ============ Configuration ============
CAM_SIZE = (640, 480)
INFER_SIZE = 320
MODEL_PATH = "yolo11n.pt"
DISPLAY_WINDOW = "SARX Camera Test"
FPS_AVG_ALPHA = 0.9

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


# ============ Detection Functions (copied from sarx.py) ============
def draw_results(img, result, model):
    """Draw detection boxes on image"""
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
            label_name = model.names[int(cid)]
        except Exception:
            label_name = str(int(cid))
        
        label = f"{label_name} {c:.2f}"
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        
        # Highlight person detections
        color = (0, 255, 0) if label_name.lower() == "person" else (255, 0, 0)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        cv2.putText(img, label, (x1, max(10, y1-6)), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


def get_person_info(result, model, orig_w, orig_h, conf_thresh=PERSON_CONF_THR):
    """
    Get largest person detection info: area ratio and center offset from image center
    Returns: (area_ratio, center_x_offset, center_y_offset, found)
    """
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
    
    sx = orig_w / float(INFER_SIZE)
    sy = orig_h / float(INFER_SIZE)
    
    max_area = 0.0
    max_center_x = 0.0
    max_center_y = 0.0
    found = False
    
    for (x1, y1, x2, y2), conf, cid in zip(boxes, confs, clsids):
        # Check if person
        is_person = False
        try:
            name = model.names[int(cid)].lower()
            if name == "person":
                is_person = True
        except Exception:
            if int(cid) == 0:  # COCO person class
                is_person = True
        
        if not is_person or conf < conf_thresh:
            continue
        
        # Scale to original image size
        x1o = max(0.0, x1 * sx)
        y1o = max(0.0, y1 * sy)
        x2o = min(orig_w, x2 * sx)
        y2o = min(orig_h, y2 * sy)
        
        w = max(0.0, x2o - x1o)
        h = max(0.0, y2o - y1o)
        area = w * h
        
        if area > max_area:
            max_area = area
            # Calculate center of bounding box
            center_x = (x1o + x2o) / 2.0
            center_y = (y1o + y2o) / 2.0
            # Offset from image center (normalized -1 to 1)
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
    
    # Initialize cameras
    print("[CAMERA] Initializing cameras...")
    cam0 = Picamera2(0)  # Bottom camera (overhead view)
    cam1 = Picamera2(1)  # Front camera (forward view)
    
    cfg0 = cam0.create_preview_configuration(main={"format": "BGR888", "size": CAM_SIZE})
    cfg1 = cam1.create_preview_configuration(main={"format": "BGR888", "size": CAM_SIZE})
    cam0.configure(cfg0)
    cam1.configure(cfg1)
    cam0.start()
    cam1.start()
    time.sleep(1)
    print("[CAMERA] ✅ Cameras ready\n")
    
    # Load YOLO model
    print(f"[YOLO] Loading model: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    print("[YOLO] ✅ Model loaded\n")
    
    # Initialize simulated drone controller
    drone = SimulatedDroneController()
    
    # State machine
    current_state = State.SEARCHING
    fps = 0.0
    last_time = time.time()
    
    # State timing for simulated operations
    state_start_time = time.time()
    
    print("="*60)
    print("🚀 STARTING DETECTION LOOP")
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
            
            # Resize for inference
            f0_in = cv2.resize(frame0, (INFER_SIZE, INFER_SIZE))
            f1_in = cv2.resize(frame1, (INFER_SIZE, INFER_SIZE))
            
            # Run YOLO inference on both cameras
            results = model([f0_in, f1_in], imgsz=INFER_SIZE, verbose=False)
            
            # Draw results
            if len(results) >= 1:
                draw_results(vis0, results[0], model)
            if len(results) >= 2:
                draw_results(vis1, results[1], model)
            
            # Get detection info
            h0, w0 = frame0.shape[:2]
            h1, w1 = frame1.shape[:2]
            
            area_bottom, cx_bottom, cy_bottom, found_bottom = get_person_info(
                results[0] if len(results) >= 1 else None, model, w0, h0
            )
            area_front, cx_front, cy_front, found_front = get_person_info(
                results[1] if len(results) >= 2 else None, model, w1, h1
            )
            
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
                        print(f"\n👁️  [DETECTION] Human now visible in BOTTOM camera!")
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
                if found_bottom:
                    # Proportional control to center over person
                    forward_vel = -cy_bottom * 0.5  # Negative because +y is down in image
                    right_vel = cx_bottom * 0.5
                    
                    # Check if centered (within threshold)
                    if abs(cx_bottom) < 0.15 and abs(cy_bottom) < 0.15:
                        if area_bottom > PERSON_AREA_THRESHOLD_BOTTOM:
                            print(f"\n✅ [CENTERED] Person is centered in bottom camera!")
                            print(f"   Area: {area_bottom:.3f} (threshold: {PERSON_AREA_THRESHOLD_BOTTOM})")
                            print(f"   Offset: ({cx_bottom:.2f}, {cy_bottom:.2f})")
                            current_state = State.DESCENDING
                            state_start_time = time.time()
                        else:
                            # Need to descend to get better view
                            print(f"📐 [CENTERING] Centered but too high, descending...")
                            drone.move_with_yaw(
                                forward=forward_vel,
                                right=right_vel,
                                down=0.2,  # Descend slowly
                                yaw_rate=0.0,
                                duration=0.3
                            )
                    else:
                        # Move to center
                        print(f"🎯 [CENTERING] Adjusting position (offset: {cx_bottom:.2f}, {cy_bottom:.2f})")
                        drone.move_with_yaw(
                            forward=forward_vel,
                            right=right_vel,
                            down=0.0,
                            yaw_rate=0.0,
                            duration=0.3
                        )
                else:
                    # Lost bottom view, try to reacquire
                    print("⚠️  [WARNING] Lost bottom view, adjusting...")
            
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
            
            # Combine views
            target_h = 360
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
