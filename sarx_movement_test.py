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
    print("SARX MOVEMENT TEST")
    print("=" * 60)
    
    # Initialize drone controller
    print("[DRONE] Initializing drone controller...")
    drone = DroneController(system_address=SYSTEM_ADDRESS)
    drone.start()
    
    # Wait for drone to be ready
    while not drone.is_ready():
        print("[DRONE] Waiting for drone to be ready...")
        time.sleep(1)
    
    print("\n[DRONE] Drone ready! Starting movement sequence...\n")
    
    try:
        # 1. Go to altitude of 10m (from 15.24m, descend 5.24m)
        print("📍 Step 1: Adjusting to 10m altitude...")
        current_alt = drone.get_altitude()
        print(f"   Current altitude: {current_alt:.2f}m")
        target_alt = 10.0
        alt_diff = current_alt - target_alt
        
        if abs(alt_diff) > 0.5:  # Only adjust if difference is significant
            duration = abs(alt_diff) / 1.0  # 1 m/s rate
            down_velocity = 1.0 if alt_diff > 0 else -1.0  # Positive down = descend in NED
            drone.move_with_yaw(forward=0.0, right=0.0, down=down_velocity, yaw_rate=0.0, duration=duration)
            time.sleep(duration + 1)
        print(f"✅ Altitude adjusted to ~10m (actual: {drone.get_altitude():.2f}m)\n")
        
        # 2. Set checkpoint
        print("📍 Step 2: Setting checkpoint...")
        drone.save_checkpoint()
        time.sleep(1)
        
        # 3. Move right 3m
        print("📍 Step 3: Moving right 3m...")
        duration = 3.0 / APPROACH_SPEED  # distance / speed
        drone.move_with_yaw(forward=0.0, right=APPROACH_SPEED, down=0.0, yaw_rate=0.0, duration=duration)
        time.sleep(duration + 1)
        print(f"✅ Moved right (altitude: {drone.get_altitude():.2f}m)\n")
        
        # 4. Move forward 3m
        print("📍 Step 4: Moving forward 3m...")
        duration = 3.0 / APPROACH_SPEED  # distance / speed
        drone.move_with_yaw(forward=APPROACH_SPEED, right=0.0, down=0.0, yaw_rate=0.0, duration=duration)
        time.sleep(duration + 1)
        print(f"✅ Moved forward (altitude: {drone.get_altitude():.2f}m)\n")
        
        # 5. Increase altitude by 3m (to 13m)
        print("📍 Step 5: Increasing altitude by 3m...")
        duration = 3.0 / 1.0  # 3m at 1 m/s
        drone.move_with_yaw(forward=0.0, right=0.0, down=-1.0, yaw_rate=0.0, duration=duration)  # Negative down = ascend
        time.sleep(duration + 1)
        print(f"✅ Altitude increased (altitude: {drone.get_altitude():.2f}m)\n")
        
        # 6. Return to checkpoint
        print("📍 Step 6: Returning to checkpoint...")
        drone.return_to_checkpoint()
        time.sleep(2)
        print(f"✅ Returned to checkpoint (altitude: {drone.get_altitude():.2f}m)\n")
        
        # 7. Yaw 360 degrees
        print("📍 Step 7: Yawing 360 degrees...")
        duration = 360.0 / YAW_RATE  # 360 degrees at YAW_RATE deg/s
        drone.yaw_towards(yaw_rate_deg=YAW_RATE, duration=duration)
        time.sleep(duration + 1)
        print("✅ Completed 360 degree yaw\n")
        
        # 8. Land
        print("📍 Step 8: Landing...")
        drone.stop_and_land()
        
        print("\n" + "="*60)
        print("✅ Movement sequence complete!")
        print("="*60)
    
    except KeyboardInterrupt:
        print("\n[SYSTEM] Interrupted by user")
        drone.stop_and_land()
    
    except Exception as e:
        print(f"\n❌ [ERROR] {e}")
        print("[CLEANUP] Landing drone...")
        drone.stop_and_land()
    
    finally:
        print("[CLEANUP] Complete. Goodbye!")


if __name__ == "__main__":
    main()
