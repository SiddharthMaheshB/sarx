#!/usr/bin/env python3
"""
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

import sys
import time
import cv2
import torch
import numpy as np
import asyncio
import threading
import math
from datetime import datetime
from pathlib import Path
from picamera2 import Picamera2
from pymavlink import mavutil
import custom_survey as cs
try:
    from mavsdk import System
    from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw
    from mavsdk.telemetry import Position
    MAVSDK_AVAILABLE = True
except Exception as e:
    print(f"[ERROR] mavsdk import failed: {e}")
    MAVSDK_AVAILABLE = False
    exit(1)

# Servo control via pymavlink - no separate import check needed
SERVO_AVAILABLE = True
tagged_locations = []
CAM_NATIVE_SIZE = (3280, 2464)  # IMX219 full resolution (max FOV)
CAM_PREVIEW_SIZE = (1280, 720)  # Use a high-res preview for best FOV and speed
INFER_SIZE = 320
IMG_SIZE = 320
DISPLAY_WINDOW = "Human Detection Delivery"
FPS_AVG_ALPHA = 0.9

# Model paths (try Pi paths first, then fallback)
YOLOV5_DIR = Path("/home/drone/Desktop/yolov5")
WEIGHTS_PT = Path("/home/drone/Desktop/final/weights_only.pt")
MODEL_YAML = Path("/home/drone/Desktop/yolov5/models/yolov5n.yaml")
FALLBACK_MODEL = "yolo11n.pt"

# Inference settings
CONF_THRES = 0.25
IOU_THRES = 0.45
USE_PYTORCH = False
model = None
names = ["person"]

# Detection thresholds
PERSON_CONF_THR = 0.3
PERSON_AREA_THRESHOLD_FRONT = 0.15  # Front camera threshold to trigger approach
PERSON_AREA_THRESHOLD_BOTTOM = 0.4  # Bottom camera threshold for centered position

# MAV settings
SYSTEM_ADDRESS = "serial:///dev/ttyACM0:115200"
TAKEOFF_ALT = -15.24  # NED down negative = up 15.24m (50ft)
SURVEY_ALT = -12.19  # NED down negative = up 12.19m (40ft) for survey
DELIVERY_ALT = -6  # NED down negative = up 6m (20ft)
SEARCH_SPEED = 1.0  # m/s
APPROACH_SPEED = 0.8  # m/s
YAW_RATE = 30.0  # deg/s

# Survey path settings
PLAN_FILE = "/home/drone/Desktop/kml/mission.plan"
DEFAULT_SEPARATION_M = 15.0  # meters between survey lines
WAYPOINT_RADIUS = 2.0  # meters - GPS accuracy radius
WAYPOINT_SPACING = 5.0  # meters between waypoints

# Servo/drop settings (pymavlink)
SERVO_NUMBERS = [9, 10, 11, 12, 13]  # 5 servos on Pixhawk
SERVO_CLOSE_PWM = 900
SERVO_OPEN_PWM = 1400
DROP_HOLD_SECONDS = 1.0
current_servo_index = 0  # Tracks which servo to use next (0-4)

# State timing and failsafes
APPROACHING_TIMEOUT = 20.0  # Max 20 seconds in APPROACHING
CENTERING_TIMEOUT = 15.0  # Max 15 seconds in CENTERING
CENTERING_LOST_TIMEOUT = 7.5  # Timeout to recover if bottom cam lost
DESCENT_RATE = 1.0  # m/s descent speed
RETURN_RATE = 1.0  # m/s return speed
DETECTION_COOLDOWN = 10.0  # Seconds to ignore detections after returning from delivery


# ============ State Machine States ============
class State:
    SEARCHING = "SEARCHING"
    APPROACHING = "APPROACHING"
    CENTERING = "CENTERING"
    DESCENDING = "DESCENDING"
    DROPPING = "DROPPING"
    RETURNING = "RETURNING"


# ============ Model Loading (from sarx_camera_test.py) ============
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
            print("[MODEL] ✓ YOLOv5 PyTorch model loaded (CPU optimized)")
            return True
        except Exception as e:
            print(f"[MODEL] PyTorch loading failed: {e}")
            print("[MODEL] Falling back to Ultralytics YOLO...")
    
    # Fallback to Ultralytics
    try:
        print("couldnt load")
    #     from ultralytics import YOLO
    #     print(f"[MODEL] Loading Ultralytics YOLO: {FALLBACK_MODEL}")
    #     model = YOLO(FALLBACK_MODEL)
    #     USE_PYTORCH = False
    #     names = model.names
    #     print("[MODEL] ✓ Ultralytics YOLO loaded")
    #     return True
    except Exception as e:
        print(f"[ERROR] Failed to load any model: {e}")
        return False


def infer_pytorch(frame, model):
    """
    Run PyTorch YOLOv5 inference (optimized preprocessing).
    Returns detections tensor [N, 6] with format (x1, y1, x2, y2, conf, cls).
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
        pred = model(img)
    
    # Post-process
    from utils.general import non_max_suppression, scale_boxes
    pred = non_max_suppression(pred, conf_thres=CONF_THRES, iou_thres=IOU_THRES)
    
    # Scale back to original frame dimensions
    det = pred[0] if len(pred) > 0 else torch.tensor([], device="cpu")
    if len(det) > 0:
        det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], (orig_h, orig_w)).round()
    
    return det


# def infer_ultralytics(frame, model):
#     """Run Ultralytics YOLO inference and return results."""
#     results = model(frame, imgsz=IMG_SIZE, verbose=False)
#     return results


# ============ Waypoint Generator ============

class WaypointGenerator:
    """Converts Shapely LineString paths to navigable GPS waypoints"""
    
    def __init__(self, path_m, lat0, lon0, m_per_deg_lat, m_per_deg_lon):
        """
        Initialize waypoint generator with path in local meters.
        
        Args:
            path_m: Shapely LineString in local meters (East, North)
            lat0, lon0: Reference GPS point for coordinate conversion
            m_per_deg_lat, m_per_deg_lon: Meter-per-degree conversion factors
        """
        self.path_m = path_m
        self.lat0 = lat0
        self.lon0 = lon0
        self.m_per_deg_lat = m_per_deg_lat
        self.m_per_deg_lon = m_per_deg_lon
        self.waypoints_gps = []
        self.waypoints_local = []
    
    def generate_waypoints(self, spacing_m=5.0, altitude_m=None):
        """
        Generate waypoints along path at specified spacing.
        
        Args:
            spacing_m: Distance between waypoints in meters
            altitude_m: Altitude for waypoints (defaults to survey altitude)
            
        Returns:
            List of (lat, lon, alt_m) tuples in GPS coordinates
        """
        if self.path_m is None or self.path_m.is_empty:
            print("ERROR: No valid path to generate waypoints")
            return []
        
        if altitude_m is None:
            altitude_m = abs(SURVEY_ALT)
        
        total_length = self.path_m.length
        num_points = max(2, int(total_length / spacing_m) + 1)
        
        print(f"  Generating {num_points} waypoints along {total_length:.1f}m path at {altitude_m:.1f}m altitude")
        
        for i in range(num_points):
            # Interpolate point along path (normalized = 0.0 to 1.0)
            fraction = i / (num_points - 1) if num_points > 1 else 0.0
            point = self.path_m.interpolate(fraction, normalized=True)
            
            x_m = point.x  # East in meters
            y_m = point.y  # North in meters
            
            # Convert local meters to lat/lon offset
            dlat_deg = y_m / self.m_per_deg_lat
            dlon_deg = x_m / self.m_per_deg_lon
            
            lat = self.lat0 + dlat_deg
            lon = self.lon0 + dlon_deg
            
            self.waypoints_local.append((x_m, y_m))
            self.waypoints_gps.append((lat, lon, altitude_m))
        
        return self.waypoints_gps
    
    def get_local_waypoints(self):
        """Return waypoints in local meters (for visualization)"""
        return self.waypoints_local
    
    def get_gps_waypoints(self):
        """Return waypoints in GPS coordinates"""
        return self.waypoints_gps


# ============ Path Planning Functions ============

def load_survey_polygon():
    """
    Load polygon from mission.plan file.
    
    Returns:
        tuple: (poly_m, lat0, lon0, m_per_deg_lat, m_per_deg_lon) or (None, None, None, None, None) on error
    """
    print("\n[SURVEY] Loading polygon from mission.plan...")
    try:
        poly_m, (lat0, lon0, m_per_deg_lat, m_per_deg_lon) = cs.load_polygon_from_plan_in_meters(
            PLAN_FILE
        )
        print(f"  Polygon loaded: {poly_m.area:.1f} m²")
        print(f"  Reference: lat0={lat0:.6f}, lon0={lon0:.6f}")
        return poly_m, lat0, lon0, m_per_deg_lat, m_per_deg_lon
    except Exception as e:
        print(f"ERROR loading polygon: {e}")
        return None, None, None, None, None


def generate_survey_paths(poly_m, separation_m):
    """
    Generate survey paths for two polygon halves.
    
    Args:
        poly_m: Shapely Polygon in meters
        separation_m: Separation distance between survey lines
        
    Returns:
        tuple: (path1, path2, angle_half1, angle_half2) or (None, None, None, None) on error
    """
    print("\n[SURVEY] Generating survey paths...")
    try:
        # Split polygon into two halves
        poly1_m, poly2_m, cut_line = cs.compute_equal_area_split(poly_m, angle_rad=0.0)
        
        if poly1_m is None or poly2_m is None:
            print("ERROR: Failed to split polygon into two regions")
            print("  Polygon may be too small or irregularly shaped")
            return None, None, None, None
        
        # Find best angles for each half
        print("  Finding optimal angles for each region...")
        angle_half1, path1, _, _, _ = cs.find_best_angle_for_region(
            poly1_m, separation_m, (0, 0), angle_step_deg=1.0
        )
        angle_half2, path2, _, _, _ = cs.find_best_angle_for_region(
            poly2_m, separation_m, (0, 0), angle_step_deg=1.0
        )
        
        # Validate path generation
        if angle_half1 is None or path1 is None:
            print("ERROR: Could not generate survey path for first half")
            print(f"  Separation {separation_m}m may be too large for region size")
            minx, miny, maxx, maxy = poly_m.bounds
            min_dim = min(maxx - minx, maxy - miny)
            print(f"  Try reducing separation to <{min_dim/2:.1f}m")
            return None, None, None, None
        
        if angle_half2 is None or path2 is None:
            print("ERROR: Could not generate survey path for second half")
            print(f"  Separation {separation_m}m may be too large for region size")
            minx, miny, maxx, maxy = poly_m.bounds
            min_dim = min(maxx - minx, maxy - miny)
            print(f"  Try reducing separation to <{min_dim/2:.1f}m")
            return None, None, None, None
        
        if path1.is_empty or path2.is_empty:
            print("ERROR: Generated paths are empty")
            print("  Check polygon size and separation distance")
            return None, None, None, None
        
        print(f"  ✓ Half 1: angle={angle_half1:.1f} deg, path length={path1.length:.1f}m")
        print(f"  ✓ Half 2: angle={angle_half2:.1f} deg, path length={path2.length:.1f}m")
        
        return path1, path2, angle_half1, angle_half2
        
    except Exception as e:
        print(f"ERROR generating survey paths: {e}")
        import traceback
        traceback.print_exc()
        return None, None, None, None


def generate_waypoints_from_paths(path1, path2, lat0, lon0, m_per_deg_lat, m_per_deg_lon):
    """
    Generate GPS waypoints from survey paths.
    
    Args:
        path1, path2: Shapely LineString paths in meters
        lat0, lon0: Reference GPS coordinates
        m_per_deg_lat, m_per_deg_lon: Meter-per-degree conversion factors
        
    Returns:
        tuple: (waypoints1, waypoints2, all_waypoints, total_distance, est_time_min) or None on error
    """
    print("\n[SURVEY] Converting paths to waypoints...")
    
    try:
        # Create waypoint generators for both paths
        gen1 = WaypointGenerator(path1, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
        gen2 = WaypointGenerator(path2, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
        
        # Use relative altitude for now - will be converted to absolute later
        survey_altitude_relative = abs(SURVEY_ALT)  # 12.19m relative altitude
        
        # Generate waypoints for both paths
        waypoints1 = gen1.generate_waypoints(spacing_m=WAYPOINT_SPACING, altitude_m=survey_altitude_relative)
        waypoints2 = gen2.generate_waypoints(spacing_m=WAYPOINT_SPACING, altitude_m=survey_altitude_relative)
        
        # Validate waypoint generation
        if not waypoints1 or not waypoints2:
            print("ERROR: Failed to generate waypoints from paths")
            return None
        
        print(f"  ✓ Path 1: {len(waypoints1)} waypoints")
        print(f"  ✓ Path 2: {len(waypoints2)} waypoints")
        
        # Combine waypoints (first half, then second half)
        all_waypoints = waypoints1 + waypoints2
        print(f"  ✓ Total waypoints: {len(all_waypoints)}")
        
        # Estimate mission time
        total_distance = path1.length + path2.length
        est_time_min = (total_distance / SEARCH_SPEED) / 60
        print(f"  Estimated mission time: {est_time_min:.1f} minutes")
        print(f"  Survey altitude: {abs(SURVEY_ALT):.1f}m")
        
        return waypoints1, waypoints2, all_waypoints, total_distance, est_time_min
        
    except Exception as e:
        print(f"ERROR generating waypoints: {e}")
        import traceback
        traceback.print_exc()
        return None


def generate_survey_waypoints(separation_m=DEFAULT_SEPARATION_M):
    """
    Generate complete survey path waypoints (wrapper function).
    
    Args:
        separation_m: Distance between survey lines
        
    Returns:
        list: GPS waypoints [(lat, lon, alt), ...] or None on error
    """
    # Load polygon
    poly_m, lat0, lon0, m_per_deg_lat, m_per_deg_lon = load_survey_polygon()
    if poly_m is None:
        return None
    
    # Generate paths
    path1, path2, angle_half1, angle_half2 = generate_survey_paths(poly_m, separation_m)
    if path1 is None or path2 is None:
        return None
    
    # Generate waypoints
    result = generate_waypoints_from_paths(path1, path2, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
    if result is None:
        return None
    
    waypoints1, waypoints2, all_waypoints, total_distance, est_time_min = result
    return all_waypoints


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
        
        await self.drone.telemetry.set_rate_position(5)
        await self.drone.telemetry.set_rate_gps_info(1)
        
        try:
            async for position in self.drone.telemetry.position():
                self.current_position = position
                self.current_altitude = -position.relative_altitude_m  # NED (negative down)
                self.altitude_m = abs(position.relative_altitude_m)  # Positive altitude (positive = up)
                print("[POSITION] Lat: ", position.latitude_deg, " Long: ", position.longitude_deg, " Alt: ", position.absolute_altitude_m)
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
    
    def navigate_to_waypoint(self, lat, lon, alt, tolerance_m=WAYPOINT_RADIUS):
        """Navigate to GPS waypoint and wait for arrival"""
        if not self.loop or not self._connected:
            print("[DRONE] Navigation requested but drone not ready")
            return False
        
        fut = asyncio.run_coroutine_threadsafe(
            self._do_navigate_waypoint(lat, lon, alt, tolerance_m),
            self.loop
        )
        try:
            return fut.result(timeout=120)
        except Exception as e:
            print(f"[DRONE] Navigation error: {e}")
            return False
    
    async def _do_navigate_waypoint(self, lat, lon, alt, tolerance_m):
        """Execute GPS waypoint navigation"""
        try:
            # Stop offboard mode to use action.goto_location
            await self.drone.offboard.stop()
            await asyncio.sleep(0.5)
            
            # Send goto command
            await self.drone.action.goto_location(
                latitude_deg=lat,
                longitude_deg=lon,
                absolute_altitude_m=alt,
                yaw_deg=0.0
            )
            
            # Monitor arrival
            arrived = False
            last_log_time = 0
            
            while not arrived:
                if self.current_position:
                    dist = self._calculate_gps_distance(
                        self.current_position.latitude_deg,
                        self.current_position.longitude_deg,
                        lat, lon
                    )
                    alt_diff = abs(self.current_position.absolute_altitude_m - alt)
                    
                    # Check if arrived
                    if dist < tolerance_m and alt_diff < 1.0:
                        arrived = True
                        break
                    
                    # Log progress every 5 seconds
                    current_time = time.time()
                    if current_time - last_log_time >= 5:
                        print(f"   [NAV] Distance: {dist:.1f}m, Alt: {self.altitude_m:.1f}m")
                        last_log_time = current_time
                
                await asyncio.sleep(0.5)
            
            # Resume offboard mode
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, TAKEOFF_ALT, 0.0)
            )
            await self.drone.offboard.start()
            await asyncio.sleep(0.5)
            
            return arrived
            
        except Exception as e:
            print(f"[DRONE] GPS navigation error: {e}")
            try:
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(0.0, 0.0, TAKEOFF_ALT, 0.0)
                )
                await self.drone.offboard.start()
            except:
                pass
            return False
    
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
    """Initialize servo connection via pymavlink"""
    global current_servo_index
    
    try:
        master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        master.wait_heartbeat()
        print("[SERVO] Connected to Pixhawk for servo control")
        
        # Initialize all servos to closed position
        for servo_num in SERVO_NUMBERS:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                servo_num,
                SERVO_CLOSE_PWM,
                0, 0, 0, 0, 0
            )
            time.sleep(0.1)
        
        print(f"[SERVO] Initialized {len(SERVO_NUMBERS)} servos (closed)")
        current_servo_index = 0  # Reset counter
        return master
    except Exception as e:
        print(f"[SERVO] Initialization failed: {e}")
        return None


def drop_payload(master):
    """Drop payload using servo with counter tracking"""
    global current_servo_index
    
    if master is None:
        print("[PAYLOAD] ⚠️  Drop called but servo not available")
        return False
    
    if current_servo_index >= len(SERVO_NUMBERS):
        print("[PAYLOAD] ⚠️  All servos have been used (5/5)")
        return False
    
    try:
        servo_num = SERVO_NUMBERS[current_servo_index]
        print("\n" + "="*60)
        print(f"📦 [PAYLOAD] Releasing package from servo {servo_num} ({current_servo_index + 1}/5)...")
        
        # Open servo to drop payload
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_num,
            SERVO_OPEN_PWM,  # Open position (1400)
            0, 0, 0, 0, 0
        )
        
        time.sleep(DROP_HOLD_SECONDS)
        print(f"✅ [PAYLOAD] Servo {servo_num} opened and remains open")
        print("="*60)
        
        # Increment counter for next human
        current_servo_index += 1
        if current_servo_index < len(SERVO_NUMBERS):
            print(f"[SERVO] Next drop will use servo {SERVO_NUMBERS[current_servo_index]}")
        else:
            print("[SERVO] All servos used - no more drops available")
        
        return True
    except Exception as e:
        print(f"❌ [PAYLOAD] Drop error: {e}")
        print("="*60)
        return False


# ============ Detection Functions ============
def is_in_exclusion_zone(drone, exclusion_radius_m=5.0):
    """
    Check if current drone position is within exclusion radius of any tagged location.
    Returns: (bool, distance) - True if in exclusion zone, and distance to nearest tagged location
    """
    if not drone.current_position or not tagged_locations:
        return False, float('inf')
    
    current_lat = drone.current_position.latitude_deg
    current_lon = drone.current_position.longitude_deg
    
    min_distance = float('inf')
    
    for tagged_loc in tagged_locations:
        distance = drone._calculate_gps_distance(
            current_lat, current_lon,
            tagged_loc['lat'], tagged_loc['lon']
        )
        
        if distance < min_distance:
            min_distance = distance
        
        if distance < exclusion_radius_m:
            return True, distance
    
    return False, min_distance


def print_state_change(old_state, new_state):
    """Print state transition with visual separator"""
    print("\n" + "="*60)
    print(f"🔄 STATE TRANSITION: {old_state} → {new_state}")
    print("="*60 + "\n")


def start_bottom_camera(cam0):
    """Start the bottom camera"""
    try:
        cam0.start()
        print("[CAMERA] ✅ Bottom camera started")
        return True
    except Exception as e:
        print(f"[CAMERA] ⚠️  Error starting bottom camera: {e}")
        return False


def stop_bottom_camera(cam0):
    """Stop the bottom camera to save resources"""
    try:
        cam0.stop()
        print("[CAMERA] Bottom camera stopped (idle)")
        return True
    except Exception as e:
        print(f"[CAMERA] ⚠️  Error stopping bottom camera: {e}")
        return False


def draw_results(img, result, model, use_pytorch=False):
    """Draw detection boxes on image (supports both PyTorch and Ultralytics)"""
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
        # Ultralytics format
        print("something wrong here")
        # try:
        #     boxes = result.boxes.xyxy.cpu().numpy()
        #     confs = result.boxes.conf.cpu().numpy()
        #     clsids = result.boxes.cls.cpu().numpy().astype(int)
        # except Exception:
        #     boxes = np.array(result.boxes.xyxy).astype(np.float32)
        #     confs = np.array(result.boxes.conf).astype(np.float32)
        #     clsids = np.array(result.boxes.cls).astype(np.int32)
        
        # for (x1, y1, x2, y2), c, cid in zip(boxes, confs, clsids):
        #     try:
        #         label_name = model.names[int(cid)]
        #     except Exception:
        #         label_name = str(int(cid))
            
        #     label = f"{label_name} {c:.2f}"
        #     x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
        #     color = (0, 255, 0) if label_name.lower() == "person" else (255, 0, 0)
        #     cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        #     cv2.putText(img, label, (x1, max(10, y1-6)), 
        #                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)



def get_person_info(result, model, orig_w, orig_h, conf_thresh=PERSON_CONF_THR, use_pytorch=False):
    """
    Get largest person detection info: area ratio and center offset from image center
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
        # Ultralytics format
        if result is None:
            return 0.0, 0.0, 0.0, False
        print("something wrong here")
        # try:
        #     boxes = result.boxes.xyxy.cpu().numpy()
        #     confs = result.boxes.conf.cpu().numpy()
        #     clsids = result.boxes.cls.cpu().numpy().astype(int)
        # except Exception:
        #     boxes = np.array(result.boxes.xyxy).astype(np.float32)
        #     confs = np.array(result.boxes.conf).astype(np.float32)
        #     clsids = np.array(result.boxes.cls).astype(np.int32)
        
        # if boxes.size == 0:
        #     return 0.0, 0.0, 0.0, False
        
        # max_area = 0.0
        # max_center_x = 0.0
        # max_center_y = 0.0
        # found = False
        
        # for (x1, y1, x2, y2), conf, cid in zip(boxes, confs, clsids):
        #     # Check if person
        #     is_person = False
        #     try:
        #         name = model.names[int(cid)].lower()
        #         if name == "person":
        #             is_person = True
        #     except Exception:
        #         if int(cid) == 0:  # COCO person class
        #             is_person = True
            
        #     if not is_person or conf < conf_thresh:
        #         continue
            
        #     w = max(0.0, x2 - x1)
        #     h = max(0.0, y2 - y1)
        #     area = w * h
            
        #     if area > max_area:
        #         max_area = area
        #         center_x = (x1 + x2) / 2.0
        #         center_y = (y1 + y2) / 2.0
        #         max_center_x = (center_x - orig_w / 2.0) / (orig_w / 2.0)
        #         max_center_y = (center_y - orig_h / 2.0) / (orig_h / 2.0)
        #         found = True
        
        # area_ratio = max_area / (orig_w * orig_h) if orig_w * orig_h > 0 else 0.0
        # return area_ratio, max_center_x, max_center_y, found



# ============ Main Program ============
def main():
    print("=" * 60)
    print("SARX")
    print("=" * 60)
    
    # Initialize cameras for max FOV
    print("[CAMERA] Initializing IMX219 cameras for max FOV...")
    cam0 = Picamera2(0)  # Bottom camera (started only during APPROACHING)
    cam1 = Picamera2(1)  # Front camera (always active during search)
    cfg0 = cam0.create_preview_configuration(main={"size": CAM_PREVIEW_SIZE, "format": "BGR888"})
    cfg1 = cam1.create_preview_configuration(main={"size": CAM_PREVIEW_SIZE, "format": "BGR888"})
    cam0.configure(cfg0)
    cam1.configure(cfg1)
    # Start only front camera initially - bottom camera will start on APPROACHING
    cam1.start()
    time.sleep(0.5)
    cam0_active = False
    print(f"[CAMERA]  Front camera ready at {CAM_PREVIEW_SIZE} (IMX219 wide FOV)")
    print(f"[CAMERA] Bottom camera initialized (will start on APPROACHING)")
    
    # Load YOLO model (PyTorch optimized or Ultralytics fallback)
    if not load_model():
        print("[ERROR] Failed to load model, exiting")
        return
    
    # Initialize servo
    servo = init_servo()
    
    # Initialize drone controller
    print("[DRONE] Initializing drone controller...")
    drone = DroneController(system_address=SYSTEM_ADDRESS)
    drone.start()
    
    # Generate survey waypoints
    print("\n[SURVEY] Generating survey path...")
    survey_waypoints = generate_survey_waypoints(separation_m=DEFAULT_SEPARATION_M)
    
    if survey_waypoints and len(survey_waypoints) > 0:
        print(f"[SURVEY] ✓ Survey path ready with {len(survey_waypoints)} waypoints")
        # Wait for drone position to convert to absolute altitude
        time.sleep(3)
        home_alt_msl = drone.current_position.absolute_altitude_m - drone.current_position.relative_altitude_m if drone.current_position else 0
        if home_alt_msl > 0:
            survey_waypoints = [(lat, lon, home_alt_msl + alt) for lat, lon, alt in survey_waypoints]
            print(f"[SURVEY] ✓ Converted to absolute altitude (Home MSL: {home_alt_msl:.1f}m)")
    else:
        print("[SURVEY]   No survey path available, drone will hover in place")
        survey_waypoints = None
    
    # State machine
    current_state = State.SEARCHING
    current_waypoint_idx = 0
    last_return_time = 0  # Track when we last returned from delivery
    fps = 0.0
    last_time = time.time()
    state_start_time = time.time()
    
    print("\n" + "="*60)
    print("STARTING DETECTION LOOP")
    print("="*60)
    print(f"Initial State: {current_state}")
    print(f"Drone Ready: {drone.is_ready()}")
    print(f"Model: {'PyTorch (CPU optimized)' if USE_PYTORCH else 'Ultralytics'}\n")
    
    global tagged_locations
    try:
        while True:
            # Capture frames from both cameras
            frame0_rgb = cam0.capture_array()
            frame1_rgb = cam1.capture_array()
            
            frame0 = cv2.cvtColor(frame0_rgb, cv2.COLOR_RGB2BGR)  # Bottom
            frame1 = cv2.cvtColor(frame1_rgb, cv2.COLOR_RGB2BGR)  # Front
            
            vis0 = frame0.copy()
            vis1 = frame1.copy()
            
            # Run inference on both cameras (PyTorch or Ultralytics)
            # Pass original full-size frames, only resize for model input, display uncropped
            if USE_PYTORCH:
                det0 = infer_pytorch(frame0, model)
                det1 = infer_pytorch(frame1, model)
                results = [det0, det1]
                draw_results(vis0, det0, model, use_pytorch=True)
                draw_results(vis1, det1, model, use_pytorch=True)
            else:
                h0, w0 = frame0.shape[:2]
                h1, w1 = frame1.shape[:2]
                f0_in = cv2.resize(frame0, (IMG_SIZE, IMG_SIZE))
                f1_in = cv2.resize(frame1, (IMG_SIZE, IMG_SIZE))
                results = model([f0_in, f1_in], imgsz=IMG_SIZE, verbose=False)
                def scale_boxes_back(result, orig_w, orig_h):
                    try:
                        boxes = result.boxes.xyxy.cpu().numpy()
                        scale_x = orig_w / IMG_SIZE
                        scale_y = orig_h / IMG_SIZE
                        boxes[:, [0, 2]] *= scale_x
                        boxes[:, [1, 3]] *= scale_y
                        result.boxes.xyxy = type(result.boxes.xyxy)(boxes)
                    except Exception:
                        pass
                if len(results) >= 1:
                    scale_boxes_back(results[0], w0, h0)
                    draw_results(vis0, results[0], model, use_pytorch=False)
                if len(results) >= 2:
                    scale_boxes_back(results[1], w1, h1)
                    draw_results(vis1, results[1], model, use_pytorch=False)
            
            # Get detection info
            h0, w0 = frame0.shape[:2]
            h1, w1 = frame1.shape[:2]
            
            area_bottom, cx_bottom, cy_bottom, found_bottom = get_person_info(
                results[0] if len(results) >= 1 else None, model, w0, h0, use_pytorch=USE_PYTORCH
            )
            area_front, cx_front, cy_front, found_front = get_person_info(
                results[1] if len(results) >= 2 else None, model, w1, h1, use_pytorch=USE_PYTORCH
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
                # Navigate through survey waypoints while looking for humans
                
                # Check if human detected - interrupt waypoint navigation
                # Apply cooldown period after returning from delivery to avoid re-detecting same person
                time_since_return = time.time() - last_return_time
                
                if found_front and area_front > PERSON_AREA_THRESHOLD_FRONT:
                    if time_since_return > DETECTION_COOLDOWN:
                        print(f"\n [DETECTION] Human detected in FRONT camera!")
                        print(f"   Area ratio: {area_front:.3f} (threshold: {PERSON_AREA_THRESHOLD_FRONT})")
                        print(f"   Position offset: ({cx_front:.2f}, {cy_front:.2f})")
                        if drone.save_checkpoint():
                            # Start bottom camera for precise approach
                            if not cam0_active:
                                start_bottom_camera(cam0)
                                cam0_active = True
                                time.sleep(0.5)  # Give camera time to start
                            current_state = State.APPROACHING
                            state_start_time = time.time()
                    else:   
                        # Still in cooldown period - ignore detection
                        if elapsed % 5 < 0.1:  # Log occasionally
                            print(f"[COOLDOWN] Detection ignored - {DETECTION_COOLDOWN - time_since_return:.1f}s remaining")
                
                # Navigate to next waypoint if survey path available
                elif survey_waypoints and len(survey_waypoints) > 0:
                    if current_waypoint_idx < len(survey_waypoints):
                        lat, lon, alt = survey_waypoints[current_waypoint_idx]
                        
                        # Check if close enough to current waypoint
                        if drone.current_position:
                            dist = drone._calculate_gps_distance(
                                drone.current_position.latitude_deg,
                                drone.current_position.longitude_deg,
                                lat, lon
                            )
                            
                            if dist < WAYPOINT_RADIUS:
                                # Reached waypoint, move to next
                                current_waypoint_idx += 1
                                print(f"[SURVEY] ✓ Waypoint {current_waypoint_idx}/{len(survey_waypoints)} reached")
                                
                                if current_waypoint_idx >= len(survey_waypoints):
                                    print("[SURVEY] ✓ Survey complete! Restarting from beginning...")
                                    current_waypoint_idx = 0
                            else:
                                # Navigate to current waypoint
                                if elapsed % 10 < 0.1:  # Print status every 10 seconds
                                    print(f"[SURVEY] Waypoint {current_waypoint_idx+1}/{len(survey_waypoints)} - Distance: {dist:.1f}m")
                                
                                # Use goto_location for waypoint navigation
                                drone.navigate_to_waypoint(lat, lon, alt)
                    else:
                        # Restart survey
                        current_waypoint_idx = 0
                
                else:
                    # No survey path, just hover and search
                    if elapsed % 5 < 0.1:
                        print(f"🔍 [SEARCHING] Hovering and scanning for humans... ({elapsed:.0f}s)")
            
            elif current_state == State.APPROACHING:
                # Approach the human using front camera
                # Check exclusion zone - abort if within 5m of previously tagged location
                in_exclusion, dist_to_tagged = is_in_exclusion_zone(drone, exclusion_radius_m=5.0)
                if in_exclusion:
                    print(f"\n⛔ [EXCLUSION ZONE] Within {dist_to_tagged:.2f}m of previously tagged location!")
                    print("   Aborting approach and returning to checkpoint...")
                    if cam0_active:
                        stop_bottom_camera(cam0)
                        cam0_active = False
                    # Return to checkpoint without tagging
                    drone.return_to_checkpoint()
                    last_return_time = time.time()
                    current_state = State.SEARCHING
                    state_start_time = time.time()
                
                # Timeout protection: return to search after 20 seconds
                elif elapsed > APPROACHING_TIMEOUT:
                    print(f"\n [TIMEOUT] Approaching timeout after {elapsed:.1f}s!")
                    print("   Returning to search mode...")
                    if cam0_active:
                        stop_bottom_camera(cam0)
                        cam0_active = False
                    current_state = State.SEARCHING
                    state_start_time = time.time()
                
                # CHECK FOR BOTTOM CAMERA FIRST - transition to CENTERING if detected
                elif found_bottom:
                    print(f"\n [TRANSITION] Human detected in BOTTOM camera!")
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
                            print(f" [APPROACHING] Yawing to face person (offset: {cx_front:.2f})")
                    else:
                        # Facing person, now move forward
                        drone.move_forward(speed=APPROACH_SPEED, duration=0.3)
                        if elapsed % 2 < 0.1:
                            print(f"  [APPROACHING] Moving forward toward person (area: {area_front:.3f})")
                else:
                    # Lost target in front camera
                    if elapsed % 1 < 0.1:  # Log every second
                        print(f"⚠️  [WARNING] Lost target in front camera! ({elapsed:.1f}s elapsed)")
                    # Stay in approaching a bit longer before giving up
                    if elapsed > APPROACHING_TIMEOUT / 2:
                        print("\n❌ [ABORT] Lost target for too long, returning to SEARCHING")
                        if cam0_active:
                            stop_bottom_camera(cam0)
                            cam0_active = False
                        current_state = State.SEARCHING
                        state_start_time = time.time()
            
            elif current_state == State.CENTERING:
                # Use bottom camera to center over person
                # Check exclusion zone - abort if within 5m of previously tagged location
                in_exclusion, dist_to_tagged = is_in_exclusion_zone(drone, exclusion_radius_m=5.0)
                if in_exclusion:
                    print(f"\n⛔ [EXCLUSION ZONE] Within {dist_to_tagged:.2f}m of previously tagged location!")
                    print("   Aborting centering and returning to checkpoint...")
                    if cam0_active:
                        stop_bottom_camera(cam0)
                        cam0_active = False
                    # Return to checkpoint without tagging
                    drone.return_to_checkpoint()
                    last_return_time = time.time()
                    current_state = State.SEARCHING
                    state_start_time = time.time()
                
                # Timeout protection: return to approach after 15 seconds
                elif elapsed > CENTERING_TIMEOUT:
                    print(f"\n⏱️  [TIMEOUT] Centering timeout after {elapsed:.1f}s!")
                    print("   Returning to APPROACHING state...")
                    if cam0_active:
                        stop_bottom_camera(cam0)
                        cam0_active = False
                    current_state = State.APPROACHING
                    state_start_time = time.time()
                
                elif found_bottom:
                    # Check if centered (within thresholds)
                    if abs(cx_bottom) < 0.10 and abs(cy_bottom) < 0.10:
                        # Person is centered horizontally and vertically
                        if area_bottom >= PERSON_AREA_THRESHOLD_BOTTOM:
                            # Save current GPS location to global list before descending
                            if drone.current_position:
                                tagged_locations.append({
                                    'lat': drone.current_position.latitude_deg,
                                    'lon': drone.current_position.longitude_deg,
                                    'alt': drone.current_position.absolute_altitude_m,
                                    'timestamp': time.time()
                                })
                                print(f"[CENTERED] GPS location saved: {tagged_locations[-1]}")
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
                        if cam0_active:
                            stop_bottom_camera(cam0)
                            cam0_active = False
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
                    # Stop bottom camera to save resources during search
                    if cam0_active:
                        stop_bottom_camera(cam0)
                        cam0_active = False
                    last_return_time = time.time()  # Update cooldown timer
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
        try:
            if cam0_active:
                cam0.stop()
        except Exception:
            pass
        try:
            cam1.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        
        print("[CLEANUP] Complete. Goodbye!")


if __name__ == "__main__":
    main()
