#!/usr/bin/env python3
"""
human_detection_delivery.py

Advanced human detection and delivery system combining dual cameras and MAVLink control.

Workflow:
1. Continuously run YOLO model detecting humans on front camera
2. When human detected:
   - Save current GPS position as checkpoint
   - Yaw towards the human and move forward
   - Once human visible in lower camera (overhead view), use it for precise navigation
   - Navigate exactly to the top of the person
   - Reduce altitude to 25 units (delivery height)
   - Drop payload
   - Return to checkpoint at previous altitude
3. Continue monitoring with YOLO

Requirements: python3, picamera2, opencv-python, ultralytics, numpy, mavsdk, gpiozero
"""

import time
import cv2
import numpy as np
import asyncio
import threading
import math
from datetime import datetime
from picamera2 import Picamera2
from ultralytics import YOLO

try:
    from mavsdk import System
    from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw
    from mavsdk.telemetry import Position
    MAVSDK_AVAILABLE = True
except Exception as e:
    print(f"[ERROR] mavsdk import failed: {e}")
    MAVSDK_AVAILABLE = False
    exit(1)

try:
    from gpiozero import Servo
    SERVO_AVAILABLE = True
except Exception as e:
    print(f"[WARN] gpiozero import failed: {e}")
    SERVO_AVAILABLE = False

# ============ Configuration ============
CAM_SIZE = (640, 480)
INFER_SIZE = 320
MODEL_PATH = "yolo11n.pt"
DISPLAY_WINDOW = "Human Detection Delivery"
FPS_AVG_ALPHA = 0.9

# Detection thresholds
PERSON_CONF_THR = 0.3
PERSON_AREA_THRESHOLD_FRONT = 0.15  # Front camera threshold to trigger approach
PERSON_AREA_THRESHOLD_BOTTOM = 0.4  # Bottom camera threshold for centered position

# MAV settings
SYSTEM_ADDRESS = "serial:///dev/ttyACM0:115200"
TAKEOFF_ALT = -5.0  # NED down negative = up 5m
DELIVERY_ALT = -1.5  # NED down negative = 1.5m (25 in relative terms)
APPROACH_SPEED = 0.8  # m/s
YAW_RATE = 30.0  # deg/s

# Servo/drop settings
SERVO_PIN = 18
SERVO_INIT_MAX = True
DROP_HOLD_SECONDS = 1.0
RESET_SERVO_AFTER_DROP = True


# ============ State Machine States ============
class State:
    SEARCHING = "SEARCHING"
    APPROACHING = "APPROACHING"
    CENTERING = "CENTERING"
    DESCENDING = "DESCENDING"
    DROPPING = "DROPPING"
    RETURNING = "RETURNING"


# ============ DroneController Class ============
class DroneController:
    """Handles all drone operations in a separate async thread"""
    
    def __init__(self, system_address=SYSTEM_ADDRESS):
        if not MAVSDK_AVAILABLE:
            raise RuntimeError("mavsdk not available")
        
        self.system_address = system_address
        self.loop = None
        self.thread = None
        self._stop_event = threading.Event()
        self.ready_event = threading.Event()
        self.drone = System()
        self._connected = False
        
        # State tracking
        self.current_position = None
        self.checkpoint_position = None
        self.checkpoint_altitude = None
        self.current_altitude = None
        
    def start(self):
        """Start the drone controller in a separate thread"""
        self.thread = threading.Thread(target=self._thread_main, daemon=True)
        self.thread.start()
        ok = self.ready_event.wait(timeout=30)
        if not ok:
            print("[WARN] DroneController not ready within 30s. Check connection.")
        else:
            print("[INFO] DroneController ready")
    
    def _thread_main(self):
        """Main thread loop for async operations"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.create_task(self._connect_and_setup())
        self.loop.create_task(self._position_monitor())
        try:
            self.loop.run_forever()
        finally:
            pending = asyncio.all_tasks(loop=self.loop)
            for t in pending:
                t.cancel()
            try:
                self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            except Exception:
                pass
            self.loop.close()
    
    async def _connect_and_setup(self):
        """Connect to drone and perform initial setup"""
        try:
            print(f"[DRONE] Connecting to: {self.system_address}...")
            await self.drone.connect(system_address=self.system_address)
            
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    print("[DRONE] Connected")
                    break
            
            print("[DRONE] Arming...")
            await self.drone.action.arm()
            
            print("[DRONE] Taking off...")
            await self.drone.action.takeoff()
            await asyncio.sleep(8)
            
            # Switch to offboard mode
            print("[DRONE] Switching to offboard mode...")
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, TAKEOFF_ALT, 0.0)
            )
            await self.drone.offboard.start()
            print("[DRONE] Offboard started, holding position")
            
            self._connected = True
            self.ready_event.set()
            
        except Exception as e:
            print(f"[DRONE] Setup error: {e}")
            self.ready_event.set()
    
    async def _position_monitor(self):
        """Continuously monitor drone position and altitude"""
        await asyncio.sleep(5)  # Wait for connection
        try:
            async for position in self.drone.telemetry.position():
                self.current_position = position
                self.current_altitude = -position.relative_altitude_m  # Convert to NED
                await asyncio.sleep(0.1)
        except Exception as e:
            print(f"[DRONE] Position monitor error: {e}")
    
    def save_checkpoint(self):
        """Save current position as checkpoint for return"""
        if self.current_position:
            self.checkpoint_position = {
                'lat': self.current_position.latitude_deg,
                'lon': self.current_position.longitude_deg,
                'alt': self.current_position.absolute_altitude_m
            }
            self.checkpoint_altitude = self.current_altitude
            print(f"[CHECKPOINT] Saved at lat={self.checkpoint_position['lat']:.6f}, "
                  f"lon={self.checkpoint_position['lon']:.6f}, "
                  f"alt={self.checkpoint_altitude:.2f}")
            return True
        return False
    
    def move_forward(self, speed=APPROACH_SPEED, duration=0.5):
        """Move forward at specified speed"""
        if not self.loop or not self._connected:
            print("[DRONE] Move requested but drone not ready")
            return
        fut = asyncio.run_coroutine_threadsafe(
            self._do_velocity(speed, 0.0, 0.0, 0.0, duration), 
            self.loop
        )
    
    def move_with_yaw(self, forward=0.0, right=0.0, down=0.0, yaw_rate=0.0, duration=0.5):
        """Move with velocity in body frame and yaw rate"""
        if not self.loop or not self._connected:
            print("[DRONE] Move requested but drone not ready")
            return
        fut = asyncio.run_coroutine_threadsafe(
            self._do_velocity(forward, right, down, yaw_rate, duration),
            self.loop
        )
    
    async def _do_velocity(self, forward, right, down, yaw_rate, duration):
        """Execute velocity command"""
        try:
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(forward, right, down, yaw_rate)
            )
            await asyncio.sleep(duration)
            # Stop after duration
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
        except Exception as e:
            print(f"[DRONE] Velocity command error: {e}")
    
    def yaw_towards(self, yaw_rate_deg=YAW_RATE, duration=0.5):
        """Yaw towards target"""
        if not self.loop or not self._connected:
            return
        fut = asyncio.run_coroutine_threadsafe(
            self._do_yaw(yaw_rate_deg, duration),
            self.loop
        )
    
    async def _do_yaw(self, yaw_rate, duration):
        """Execute yaw command"""
        try:
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, yaw_rate)
            )
            await asyncio.sleep(duration)
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
        except Exception as e:
            print(f"[DRONE] Yaw command error: {e}")
    
    def descend_to_delivery_height(self):
        """Descend to delivery altitude"""
        if not self.loop or not self._connected:
            return
        fut = asyncio.run_coroutine_threadsafe(
            self._do_descend(),
            self.loop
        )
    
    async def _do_descend(self):
        """Gradually descend to delivery altitude"""
        try:
            print(f"[DRONE] Descending to delivery altitude {DELIVERY_ALT}m...")
            # Use position control for precise altitude
            current_pos = self.current_position
            if current_pos:
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(0.0, 0.0, DELIVERY_ALT, 0.0)
                )
                # Wait for descent
                await asyncio.sleep(5)
        except Exception as e:
            print(f"[DRONE] Descend error: {e}")
    
    def return_to_checkpoint(self):
        """Return to saved checkpoint position"""
        if not self.loop or not self._connected:
            return
        fut = asyncio.run_coroutine_threadsafe(
            self._do_return_to_checkpoint(),
            self.loop
        )
        try:
            fut.result(timeout=30)
        except Exception as e:
            print(f"[DRONE] Return to checkpoint error: {e}")
    
    async def _do_return_to_checkpoint(self):
        """Navigate back to checkpoint"""
        if not self.checkpoint_position or not self.checkpoint_altitude:
            print("[DRONE] No checkpoint saved, cannot return")
            return
        
        try:
            print("[DRONE] Returning to checkpoint...")
            
            # First, ascend to original altitude
            print(f"[DRONE] Ascending to original altitude {self.checkpoint_altitude}m...")
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, self.checkpoint_altitude, 0.0)
            )
            await asyncio.sleep(5)
            
            # Use simple backward movement (approximation)
            # In production, use GPS-based navigation
            print("[DRONE] Moving back to checkpoint (approximate)...")
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(-APPROACH_SPEED, 0.0, 0.0, 0.0)
            )
            await asyncio.sleep(5)  # Adjust based on distance traveled
            
            # Stop
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
            
            print("[DRONE] Returned to checkpoint area")
            
        except Exception as e:
            print(f"[DRONE] Return navigation error: {e}")
    
    def stop_and_land(self):
        """Stop offboard mode and land"""
        if not self.loop:
            return
        fut = asyncio.run_coroutine_threadsafe(self._stop_and_land_coro(), self.loop)
        try:
            fut.result(timeout=20)
        except Exception as e:
            print(f"[DRONE] Stop and land error: {e}")
        
        def _stop_loop():
            self.loop.stop()
        self.loop.call_soon_threadsafe(_stop_loop)
        if self.thread:
            self.thread.join(timeout=5)
    
    async def _stop_and_land_coro(self):
        """Stop offboard and land"""
        try:
            print("[DRONE] Stopping offboard...")
            await self.drone.offboard.stop()
        except Exception as e:
            print(f"[DRONE] Offboard stop error: {e}")
        
        try:
            print("[DRONE] Landing...")
            await self.drone.action.land()
        except Exception as e:
            print(f"[DRONE] Landing error: {e}")


# ============ Servo Control ============
def init_servo():
    """Initialize servo"""
    if not SERVO_AVAILABLE:
        print("[SERVO] gpiozero not available")
        return None
    
    try:
        servo = Servo(SERVO_PIN)
        print(f"[SERVO] Initialized on pin {SERVO_PIN}")
        if SERVO_INIT_MAX:
            servo.max()
            print("[SERVO] Set to max() (closed position)")
        return servo
    except Exception as e:
        print(f"[SERVO] Initialization failed: {e}")
        return None


def drop_payload(servo):
    """Drop payload using servo"""
    if servo is None:
        print("[PAYLOAD] Drop called but servo not available")
        return
    
    try:
        print("[PAYLOAD] Dropping payload...")
        servo.min()  # Open servo to drop
        time.sleep(DROP_HOLD_SECONDS)
        
        if RESET_SERVO_AFTER_DROP:
            servo.max()  # Close servo
            print("[PAYLOAD] Payload dropped, servo reset")
        else:
            print("[PAYLOAD] Payload dropped")
    except Exception as e:
        print(f"[PAYLOAD] Drop error: {e}")


# ============ Detection Functions ============
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


# ============ Main Program ============
def main():
    print("=" * 60)
    print("SARX")
    print("=" * 60)
    
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
    print("[CAMERA] Cameras ready")
    
    # Load YOLO model
    print(f"[YOLO] Loading model: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    print("[YOLO] Model loaded")
    
    # Initialize servo
    servo = init_servo()
    
    # Initialize drone controller
    print("[DRONE] Initializing drone controller...")
    drone = DroneController(system_address=SYSTEM_ADDRESS)
    drone.start()
    
    # State machine
    current_state = State.SEARCHING
    fps = 0.0
    last_time = time.time()
    
    print("\n[SYSTEM] Starting main detection loop...")
    print("Press 'q' to quit\n")
    
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
            if current_state == State.SEARCHING:
                # Looking for human in front camera
                if found_front and area_front > PERSON_AREA_THRESHOLD_FRONT:
                    print(f"\n[STATE] Human detected! Area: {area_front:.3f}")
                    if drone.save_checkpoint():
                        current_state = State.APPROACHING
                        print("[STATE] -> APPROACHING")
            
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
                        print(f"[STATE] Human visible in bottom camera")
                        current_state = State.CENTERING
                        print("[STATE] -> CENTERING")
                else:
                    # Lost target, return to searching
                    print("[STATE] Lost target, returning to search")
                    current_state = State.SEARCHING
            
            elif current_state == State.CENTERING:
                # Use bottom camera to center over person
                if found_bottom:
                    # Proportional control to center over person
                    forward_vel = -cy_bottom * 0.5  # Negative because +y is down in image
                    right_vel = cx_bottom * 0.5
                    
                    # Check if centered (within threshold)
                    if abs(cx_bottom) < 0.15 and abs(cy_bottom) < 0.15:
                        if area_bottom > PERSON_AREA_THRESHOLD_BOTTOM:
                            print("[STATE] Centered over person")
                            current_state = State.DESCENDING
                            print("[STATE] -> DESCENDING")
                        else:
                            # Need to descend to get better view
                            drone.move_with_yaw(
                                forward=forward_vel,
                                right=right_vel,
                                down=0.2,  # Descend slowly
                                yaw_rate=0.0,
                                duration=0.3
                            )
                    else:
                        # Move to center
                        drone.move_with_yaw(
                            forward=forward_vel,
                            right=right_vel,
                            down=0.0,
                            yaw_rate=0.0,
                            duration=0.3
                        )
                else:
                    # Lost bottom view, try to reacquire
                    print("[STATE] Lost bottom view, adjusting...")
                    time.sleep(0.5)
            
            elif current_state == State.DESCENDING:
                # Descend to delivery height
                print("[STATE] Descending to delivery height...")
                drone.descend_to_delivery_height()
                time.sleep(6)  # Wait for descent
                current_state = State.DROPPING
                print("[STATE] -> DROPPING")
            
            elif current_state == State.DROPPING:
                # Drop payload
                print("[STATE] Dropping payload...")
                drop_payload(servo)
                time.sleep(2)
                current_state = State.RETURNING
                print("[STATE] -> RETURNING")
            
            elif current_state == State.RETURNING:
                # Return to checkpoint
                print("[STATE] Returning to checkpoint...")
                drone.return_to_checkpoint()
                time.sleep(2)
                current_state = State.SEARCHING
                print("[STATE] -> SEARCHING\n")
            
            # ========== Visualization ==========
            # Add state info to display
            state_text = f"State: {current_state}"
            cv2.putText(vis0, state_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(vis1, state_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Add detection info
            if found_bottom:
                bottom_text = f"Bottom: {area_bottom:.3f} ({cx_bottom:.2f}, {cy_bottom:.2f})"
                cv2.putText(vis0, bottom_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            if found_front:
                front_text = f"Front: {area_front:.3f} ({cx_front:.2f}, {cy_front:.2f})"
                cv2.putText(vis1, front_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
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
        
        # Stop drone
        if drone:
            print("[CLEANUP] Landing drone...")
            try:
                drone.stop_and_land()
            except Exception as e:
                print(f"[CLEANUP] Drone stop error: {e}")
        
        # Stop cameras
        cam0.stop()
        cam1.stop()
        cv2.destroyAllWindows()
        
        print("[CLEANUP] Complete. Goodbye!")


if __name__ == "__main__":
    main()
