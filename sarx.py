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
MODEL_PATH = "best.pt"
DISPLAY_WINDOW = "Human Detection Delivery"
FPS_AVG_ALPHA = 0.9

# Detection thresholds
PERSON_CONF_THR = 0.3
PERSON_AREA_THRESHOLD_FRONT = 0.15  # Front camera threshold to trigger approach
PERSON_AREA_THRESHOLD_BOTTOM = 0.4  # Bottom camera threshold for centered position

# MAV settings
SYSTEM_ADDRESS = "serial:///dev/ttyACM0:115200"
TAKEOFF_ALT = -15.24  # NED down negative = up 15.24m (50ft)
DELIVERY_ALT = -6  # NED down negative = up 6m (20ft)
SEARCH_SPEED = 1.0  # m/s
APPROACH_SPEED = 0.8  # m/s
YAW_RATE = 30.0  # deg/s

# Servo/drop settings
SERVO_PIN = 18
SERVO_INIT_MAX = True
DROP_HOLD_SECONDS = 1.0
RESET_SERVO_AFTER_DROP = True

# State timing and failsafes
APPROACHING_TIMEOUT = 20.0  # Max 20 seconds in APPROACHING
CENTERING_TIMEOUT = 15.0  # Max 15 seconds in CENTERING
CENTERING_LOST_TIMEOUT = 7.5  # Timeout to recover if bottom cam lost
DESCENT_RATE = 1.0  # m/s descent speed
RETURN_RATE = 1.0  # m/s return speed


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
        self.altitude_m = 0.0  # Altitude in meters (positive = up)
        self.checkpoint_altitude_m = 0.0  # Checkpoint altitude in meters
        
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
                self.current_altitude = -position.relative_altitude_m  # NED (negative down)
                self.altitude_m = abs(position.relative_altitude_m)  # Positive altitude (positive = up)
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
            self.checkpoint_altitude_m = abs(self.current_altitude)
            print(f"\n" + "="*60)
            print(f"📍 [CHECKPOINT] Position saved!")
            print(f"   Latitude: {self.checkpoint_position['lat']:.6f}")
            print(f"   Longitude: {self.checkpoint_position['lon']:.6f}")
            print(f"   Altitude: {self.checkpoint_altitude_m:.2f} m ({self.checkpoint_altitude_m*3.28084:.2f} ft)")
            print("="*60)
            return True
        print("[WARNING] No position available for checkpoint")
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
            ##### DO WE NEED THIS???
            # we need this.
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
        """Navigate back to checkpoint using GPS-based navigation"""
        if not self.checkpoint_position or not self.checkpoint_altitude:
            print("[DRONE] No checkpoint saved, cannot return")
            return
        
        try:
            print("[DRONE] Returning to checkpoint using GPS navigation...")
            print(f"   Target: {self.checkpoint_position['lat']:.6f}, {self.checkpoint_position['lon']:.6f}")
            print(f"   Target altitude: {self.checkpoint_altitude_m:.2f}m")
            
            # Stop offboard mode to use action.goto_location
            print("[DRONE] Stopping offboard mode for GPS navigation...")
            await self.drone.offboard.stop()
            await asyncio.sleep(0.5)
            
            # Use GPS-based goto_location for precise navigation
            # goto_location uses absolute altitude (MSL)
            await self.drone.action.goto_location(
                latitude_deg=self.checkpoint_position['lat'],
                longitude_deg=self.checkpoint_position['lon'],
                absolute_altitude_m=self.checkpoint_position['alt'],
                yaw_deg=0.0  # Maintain current heading
            )
            
            print("[DRONE] GPS navigation command sent, monitoring progress...")
            
            # Monitor distance to checkpoint
            start_time = time.time()
            arrived = False
            
            while True:
                if self.current_position:
                    # Calculate distance to checkpoint
                    distance = self._calculate_gps_distance(
                        self.current_position.latitude_deg,
                        self.current_position.longitude_deg,
                        self.checkpoint_position['lat'],
                        self.checkpoint_position['lon']
                    )
                    
                    altitude_diff = abs(self.current_position.absolute_altitude_m - 
                                       self.checkpoint_position['alt'])
                    
                    # Check if arrived (within 2 meters horizontally and 1 meter vertically)
                    if distance < 2.0 and altitude_diff < 1.0:
                        arrived = True
                        print(f"[DRONE] ✅ Arrived at checkpoint!")
                        print(f"   Distance: {distance:.2f}m, Alt diff: {altitude_diff:.2f}m")
                        break
                    
                    # Log progress every 2 seconds
                    if int(time.time() - start_time) % 2 == 0:
                        print(f"[DRONE] Returning... Distance: {distance:.1f}m, Alt diff: {altitude_diff:.1f}m")
                
                await asyncio.sleep(0.5)
            
            # Resume offboard mode for continued operation
            print("[DRONE] Resuming offboard mode...")
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, self.checkpoint_altitude, 0.0)
            )
            await self.drone.offboard.start()
            await asyncio.sleep(1)
            
            print("[DRONE] Returned to checkpoint, offboard mode active")
            
        except Exception as e:
            print(f"[DRONE] Return navigation error: {e}")
            # Try to restart offboard mode on error
            try:
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(0.0, 0.0, self.checkpoint_altitude, 0.0)
                )
                await self.drone.offboard.start()
            except Exception as e2:
                print(f"[DRONE] Failed to restart offboard: {e2}")
    
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
    
    def get_altitude(self):
        """Get current altitude in meters"""
        return self.altitude_m
    
    def is_ready(self):
        """Check if drone is connected and ready"""
        return self._connected
    
    def _calculate_gps_distance(self, lat1, lon1, lat2, lon2):
        """
        Calculate distance between two GPS coordinates using Haversine formula
        Returns distance in meters
        """
        R = 6371000  # Earth's radius in meters
        
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi/2)**2 + \
            math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        distance = R * c
        return distance
    
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
        print("[PAYLOAD] ⚠️  Drop called but servo not available")
        return False
    
    try:
        print("\n" + "="*60)
        print("📦 [PAYLOAD] Releasing package...")
        servo.min()  # Open servo to drop
        time.sleep(DROP_HOLD_SECONDS)
        
        if RESET_SERVO_AFTER_DROP:
            servo.max()  # Close servo
            print("✅ [PAYLOAD] Package dropped successfully, servo reset")
        else:
            print("✅ [PAYLOAD] Package dropped")
        print("="*60)
        return True
    except Exception as e:
        print(f"❌ [PAYLOAD] Drop error: {e}")
        print("="*60)
        return False


# ============ Detection Functions ============
def print_state_change(old_state, new_state):
    """Print state transition with visual separator"""
    print("\n" + "="*60)
    print(f"🔄 STATE TRANSITION: {old_state} → {new_state}")
    print("="*60 + "\n")


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
    state_start_time = time.time()
    
    print("\n" + "="*60)
    print("🚀 STARTING DETECTION LOOP")
    print("="*60)
    print(f"Initial State: {current_state}")
    print(f"Drone Ready: {drone.is_ready()}\n")
    
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
            elapsed = time.time() - state_start_time
            
            # Failsafe: Check drone connection
            if not drone.is_ready():
                print("❌ [FAILSAFE] Drone connection lost! Landing immediately...")
                drone.stop_and_land()
                break
            
            if current_state == State.SEARCHING:
                # Looking for human in front camera
                                
                if found_front and area_front > PERSON_AREA_THRESHOLD_FRONT:
                    print(f"\n🎯 [DETECTION] Human detected in FRONT camera!")
                    print(f"   Area ratio: {area_front:.3f} (threshold: {PERSON_AREA_THRESHOLD_FRONT})")
                    print(f"   Position offset: ({cx_front:.2f}, {cy_front:.2f})")
                    if drone.save_checkpoint():
                        current_state = State.APPROACHING
                        state_start_time = time.time()
                elif elapsed % 5 < 0.1:  # Log every 5 seconds
                    print(f"🔍 [SEARCHING] Scanning for target... ({elapsed:.0f}s elapsed)")
            
            elif current_state == State.APPROACHING:
                # Approach the human using front camera
                # Timeout protection: return to search after 20 seconds
                if elapsed > APPROACHING_TIMEOUT:
                    print(f"\n⏱️  [TIMEOUT] Approaching timeout after {elapsed:.1f}s!")
                    print("   Returning to search mode...")
                    current_state = State.SEARCHING
                    state_start_time = time.time()
                
                # CHECK FOR BOTTOM CAMERA FIRST - transition to CENTERING if detected
                elif found_bottom:
                    print(f"\n👁️  [TRANSITION] Human detected in BOTTOM camera!")
                    print(f"   Bottom area: {area_bottom:.3f}")
                    if found_front:
                        print(f"   Front area: {area_front:.3f}")
                    print(f"   Switching to CENTERING mode")
                    current_state = State.CENTERING
                    state_start_time = time.time()
                
                # Otherwise, continue approaching using front camera if available
                elif found_front:
                    # Yaw towards human first to face them
                    if abs(cx_front) > 0.1:  # If not centered horizontally
                        yaw_rate = cx_front * YAW_RATE  # Proportional yaw control
                        drone.yaw_towards(yaw_rate_deg=yaw_rate, duration=0.3)
                        if elapsed % 2 < 0.1:  # Log every 2 seconds
                            print(f"🔄 [APPROACHING] Yawing to face person (offset: {cx_front:.2f})")
                    else:
                        # Facing person, now move forward
                        drone.move_forward(speed=APPROACH_SPEED, duration=0.3)
                        if elapsed % 2 < 0.1:
                            print(f"➡️  [APPROACHING] Moving forward toward person (area: {area_front:.3f})")
                else:
                    # Lost target in front camera
                    if elapsed % 1 < 0.1:  # Log every second
                        print(f"⚠️  [WARNING] Lost target in front camera! ({elapsed:.1f}s elapsed)")
                    # Stay in approaching a bit longer before giving up
                    if elapsed > APPROACHING_TIMEOUT / 2:
                        print("\n❌ [ABORT] Lost target for too long, returning to SEARCHING")
                        current_state = State.SEARCHING
                        state_start_time = time.time()
            
            elif current_state == State.CENTERING:
                # Use bottom camera to center over person
                # Timeout protection: return to approach after 15 seconds
                if elapsed > CENTERING_TIMEOUT:
                    print(f"\n⏱️  [TIMEOUT] Centering timeout after {elapsed:.1f}s!")
                    print("   Returning to APPROACHING state...")
                    current_state = State.APPROACHING
                    state_start_time = time.time()
                
                elif found_bottom:
                    # Check if centered (within thresholds)
                    if abs(cx_bottom) < 0.10 and abs(cy_bottom) < 0.10:
                        # Person is centered horizontally and vertically
                        if area_bottom >= PERSON_AREA_THRESHOLD_BOTTOM:
                            # Ready to descend and deliver
                            print(f"\n✅ [CENTERED] Person centered in bottom camera!")
                            print(f"   Area: {area_bottom:.3f} (threshold: {PERSON_AREA_THRESHOLD_BOTTOM})")
                            print(f"   Position offset: ({cx_bottom:.2f}, {cy_bottom:.2f})")
                            print(f"   Time to center: {elapsed:.1f}s")
                            print(f"   Switching to DESCENDING mode")
                            current_state = State.DESCENDING
                            state_start_time = time.time()
                        else:
                            # Centered but too far away - descend slowly to get closer
                            if elapsed % 1 < 0.1:  # Log every second
                                print(f"📐 [CENTERING] Centered, descending to target size...")
                                print(f"   Current area: {area_bottom:.3f} → Target: {PERSON_AREA_THRESHOLD_BOTTOM:.3f}")
                            drone.move_with_yaw(forward=0.0, right=0.0, down=0.5, yaw_rate=0.0, duration=0.3)
                    else:
                        # Not centered - adjust position using proportional control
                        forward_vel = -cy_bottom * 0.5  # Negative because +y is down
                        right_vel = cx_bottom * 0.5
                        
                        if elapsed % 2 < 0.1:  # Log movement every 2 seconds
                            print(f"🎯 [CENTERING] Adjusting position (x_off: {cx_bottom:.2f}, y_off: {cy_bottom:.2f}, area: {area_bottom:.3f})")
                        
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
                        print(f"⚠️  [WARNING] Lost target in bottom camera! ({elapsed:.1f}s)")
                    
                    # Move upward to regain view
                    drone.move_with_yaw(forward=0.0, right=0.0, down=-0.3, yaw_rate=0.0, duration=0.3)
                    
                    # If lost for too long, abort centering
                    if elapsed > CENTERING_TIMEOUT / 2:
                        print(f"\n❌ [ABORT] Lost target for {elapsed:.1f}s, returning to APPROACHING")
                        current_state = State.APPROACHING
                        state_start_time = time.time()
            
            elif current_state == State.DESCENDING:
                # Descend to delivery altitude (20 feet / 6.1 meters)
                descent_distance = drone.checkpoint_altitude_m - abs(DELIVERY_ALT)
                descend_time = descent_distance / DESCENT_RATE
                
                if elapsed < descend_time:
                    remaining_time = descend_time - elapsed
                    if elapsed % 1 < 0.1:  # Log every second
                        print(f"⬇️  [DESCENDING] Lowering to delivery height")
                        print(f"   Current: {drone.get_altitude():.2f}m → Target: {abs(DELIVERY_ALT):.2f}m")
                        print(f"   Time remaining: {remaining_time:.1f}s")
                    
                    # Descend at 1 m/s
                    drone.move_with_yaw(forward=0.0, right=0.0, down=DESCENT_RATE, yaw_rate=0.0, duration=0.3)
                else:
                    # Reached delivery altitude
                    print(f"\n✅ [READY] Delivery altitude reached ({abs(DELIVERY_ALT):.1f}m)")
                    current_state = State.DROPPING
                    state_start_time = time.time()
            
            elif current_state == State.DROPPING:
                # Release payload
                if elapsed < DROP_TIME:
                    if elapsed < 0.1:  # Drop on entry to state
                        drop_payload(servo)
                else:
                    current_state = State.RETURNING
                    state_start_time = time.time()
            
            elif current_state == State.RETURNING:
                # Return to checkpoint altitude
                altitude_diff = abs(drone.checkpoint_altitude_m - drone.get_altitude())
                return_time = altitude_diff / RETURN_RATE
                
                if elapsed < return_time:
                    remaining_time = return_time - elapsed
                    if elapsed % 1 < 0.1:  # Log every second
                        print(f"🔙 [RETURNING] Moving back to checkpoint")
                        print(f"   Current: {drone.get_altitude():.2f}m → Checkpoint: {drone.checkpoint_altitude_m:.2f}m")
                        print(f"   Time remaining: {remaining_time:.1f}s")
                    
                    # Return to checkpoint altitude
                    if drone.get_altitude() < drone.checkpoint_altitude_m:
                        drone.move_with_yaw(forward=0.0, right=0.0, down=-RETURN_RATE, yaw_rate=0.0, duration=0.3)  # Up
                    else:
                        drone.move_with_yaw(forward=0.0, right=0.0, down=RETURN_RATE, yaw_rate=0.0, duration=0.3)  # Down
                else:
                    print("\n" + "="*60)
                    print("✅ [COMPLETE] Delivery mission complete!")
                    print(f"   Final altitude: {drone.get_altitude():.2f}m")
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
