#!/usr/bin/env python3
"""
kml_path.py

Integrated drone survey navigation using:
- Survey path generation from completesurvey.py
- Raspberry Pi drone control from sarx.py patterns
- Mission planning from mission.plan
- Body-frame velocity movement commands

Workflow:
1. Load mission.plan and extract polygon boundary
2. Generate optimal survey path for two polygon halves
3. Convert path to navigable waypoints (GPS coords)
4. Use sarx.py's DroneController for Raspberry Pi movement
5. Navigate via GPS with body-frame velocity capability
6. Return to takeoff point on completion

Requirements: python3, shapely, numpy, mavsdk, picamera2, ultralytics
"""

import sys
import os
import json
import math
import time
import asyncio
import threading
from datetime import datetime

# Import survey path generation
try:
    import custom_survey as cs
    from shapely.geometry import Polygon, LineString
    from shapely import affinity
except ImportError as e:
    print(f"ERROR: Missing survey modules: {e}")
    sys.exit(1)

# Import drone control (from sarx.py pattern)
try:
    from mavsdk import System
    from mavsdk.offboard import PositionNedYaw, VelocityBodyYawspeed
    from mavsdk.telemetry import Position
    MAVSDK_AVAILABLE = True
except ImportError:
    MAVSDK_AVAILABLE = False

import numpy as np

# ============ Configuration ============
PLAN_FILE = "mission.plan"
SYSTEM_ADDRESS = "serial:///dev/ttyACM0:115200"

# Survey parameters (NED convention: negative = up)
TAKEOFF_ALT = -15.24  # NED: -15.24m = 15.24m altitude (50ft)
SURVEY_ALT = -12.19  # NED: -12.19m = 12.19m altitude (40ft) - reduced for better imagery
SEARCH_SPEED = 1.0  # m/s for waypoint navigation
APPROACH_SPEED = 0.8  # m/s for precision movement
YAW_RATE = 30.0  # deg/s
DESCENT_RATE = 1.0  # m/s for altitude changes
RETURN_RATE = 1.0  # m/s for return journey
altitude_m = abs(SURVEY_ALT)  # Positive altitude for GPS
# Waypoint detection
WAYPOINT_RADIUS = 2.0  # me ters - GPS accuracy radius
WAYPOINT_TIMEOUT = 120  # seconds per waypoint before abort

# Defaults
DEFAULT_SEPARATION_M = 15.0  # meters
FEET_PER_METER = 3.280839895


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


# ============ Raspberry Pi Drone Controller (from sarx.py) ============

class DroneController:
    """
    Drone controller with body-frame velocity commands (Raspberry Pi compatible).
    Based on sarx.py patterns for MAVLink control with offboard mode.
    """
    
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
        self.altitude_m = 0.0  # Positive = up
        self.checkpoint_altitude_m = 0.0
    
    def start(self):
        """Start the drone controller in a separate async thread"""
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
                # Periodic position logging for debugging
                # print(f"[POSITION] Lat: {position.latitude_deg:.6f}, Lon: {position.longitude_deg:.6f}, Alt: {position.absolute_altitude_m:.1f}m")
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
            print(f"[CHECKPOINT] Position saved at {self.checkpoint_altitude_m:.2f}m")
            print(f"   Lat: {self.checkpoint_position['lat']:.6f}")
            print(f"   Lon: {self.checkpoint_position['lon']:.6f}")
            print("="*60)
            return True
        print("[WARNING] No position available for checkpoint")
        return False
    
    def move_with_velocity(self, forward=0.0, right=0.0, down=0.0, yaw_rate=0.0, duration=0.5):
        """
        Move drone using body-frame velocity commands (Raspberry Pi style).
        
        Args:
            forward: m/s forward (positive = forward)
            right: m/s to the right (positive = right)
            down: m/s downward (positive = down)
            yaw_rate: deg/s rotation (positive = right)
            duration: seconds to apply command
        """
        if not self.loop or not self._connected:
            print("[DRONE] Movement requested but drone not ready")
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
    
    def navigate_gps_waypoint(self, lat, lon, alt, tolerance_m=WAYPOINT_RADIUS, timeout_s=WAYPOINT_TIMEOUT):
        """
        Navigate to a GPS waypoint using goto_location.
        
        Args:
            lat, lon: Target GPS coordinates
            alt: Target altitude in meters (absolute MSL)
            tolerance_m: Acceptance radius in meters
            timeout_s: Timeout in seconds
            
        Returns:
            True if waypoint reached, False on timeout/error
        """
        if not self.loop or not self._connected:
            print("[DRONE] Navigation requested but drone not ready")
            return False
        
        fut = asyncio.run_coroutine_threadsafe(
            self._do_navigate_gps(lat, lon, alt, tolerance_m, timeout_s),
            self.loop
        )
        try:
            return fut.result(timeout=timeout_s + 5)
        except Exception as e:
            print(f"[DRONE] Navigation error: {e}")
            return False
    
    async def _do_navigate_gps(self, lat, lon, alt, tolerance_m, timeout_s):
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
            last_log_time = 0
            arrived = False
            
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
                        print(f"   [ARRIVED] Distance: {dist:.2f}m, Alt diff: {alt_diff:.2f}m")
                        arrived = True
                        break
                    
                    # Log progress every 5 seconds
                    current_time = time.time()
                    if current_time - last_log_time >= 5:
                        print(f"   [NAV] Distance: {dist:.1f}m, Alt: {self.altitude_m:.1f}m (target: {alt:.1f}m MSL)")
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
            # Try to resume offboard on error
            try:
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(0.0, 0.0, TAKEOFF_ALT, 0.0)
                )
                await self.drone.offboard.start()
            except:
                pass
            return False
    
    def return_to_checkpoint(self):
        """Return to saved checkpoint position using GPS navigation"""
        if not self.checkpoint_position or not self.checkpoint_altitude:
            print("[DRONE] No checkpoint saved, cannot return")
            return False
        
        if not self.loop or not self._connected:
            return False
        
        fut = asyncio.run_coroutine_threadsafe(
            self._do_return_to_checkpoint(),
            self.loop
        )
        try:
            return fut.result(timeout=300)
        except Exception as e:
            print(f"[DRONE] Return error: {e}")
            return False
    
    async def _do_return_to_checkpoint(self):
        """Navigate back to checkpoint using GPS-based navigation"""
        if not self.checkpoint_position:
            return False
        
        try:
            print("\n[DRONE] Returning to checkpoint using GPS navigation...")
            print(f"   Target: {self.checkpoint_position['lat']:.6f}, {self.checkpoint_position['lon']:.6f}")
            print(f"   Target altitude: {self.checkpoint_altitude_m:.2f}m")
            
            # Stop offboard mode to use action.goto_location
            print("[DRONE] Stopping offboard mode for GPS navigation...")
            await self.drone.offboard.stop()
            await asyncio.sleep(0.5)
            
            # Use GPS-based goto_location for precise navigation
            await self.drone.action.goto_location(
                latitude_deg=self.checkpoint_position['lat'],
                longitude_deg=self.checkpoint_position['lon'],
                absolute_altitude_m=self.checkpoint_position['alt'],
                yaw_deg=0.0
            )
            
            print("[DRONE] GPS navigation command sent, monitoring progress...")
            
            # Monitor distance to checkpoint
            start_time = time.time()
            arrived = False
            last_log_time = 0
            
            while time.time() - start_time < 300:
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
                    if distance < 1.0 and altitude_diff < 1.0:
                        arrived = True
                        print(f"[DRONE] ✅ Arrived at checkpoint!")
                        print(f"   Distance: {distance:.2f}m, Alt diff: {altitude_diff:.2f}m")
                        break
                    
                    # Log progress every 5 seconds
                    current_time = time.time()
                    if current_time - last_log_time >= 5:
                        print(f"[DRONE] Returning... Distance: {distance:.1f}m, Alt diff: {altitude_diff:.1f}m")
                        last_log_time = current_time
                
                await asyncio.sleep(0.5)
            
            # Resume offboard mode for continued operation
            print("[DRONE] Resuming offboard mode...")
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, self.checkpoint_altitude, 0.0)
            )
            await self.drone.offboard.start()
            await asyncio.sleep(1)
            
            print("[DRONE] Returned to checkpoint, offboard mode active")
            return arrived
            
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
            return False
    
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
    
    def get_altitude(self):
        """Get current altitude in meters (relative)"""
        return self.altitude_m
    
    def get_absolute_altitude(self):
        """Get current absolute altitude in meters (MSL)"""
        if self.current_position:
            return self.current_position.absolute_altitude_m
        return 0.0
    
    def get_home_altitude_msl(self):
        """Get home/ground altitude in MSL (for converting relative to absolute)"""
        if self.current_position:
            return self.current_position.absolute_altitude_m - self.current_position.relative_altitude_m
        return 0.0
    
    def is_ready(self):
        """Check if drone is ready"""
        return self._connected
    
    @staticmethod
    def _calculate_gps_distance(lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS points using Haversine formula"""
        R = 6371000  # Earth radius in meters
        
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_phi/2)**2 +
             math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c


# ============ Path Planning ============

def load_polygon():
    """
    Load polygon from mission.plan file.
    
    Returns:
        tuple: (poly_m, lat0, lon0, m_per_deg_lat, m_per_deg_lon) or (None, None, None, None, None) on error
    """
    print("\n[STEP 1] Loading polygon from mission.plan...")
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


def get_survey_parameters(poly_m):
    """
    Get survey parameters from user with validation.
    
    Args:
        poly_m: Shapely Polygon in meters
        
    Returns:
        float: separation distance in meters, or None on error/cancel
    """
    print("\n[STEP 2] Survey configuration...")
    minx, miny, maxx, maxy = poly_m.bounds
    min_dim = min(maxx - minx, maxy - miny)
    recommended_max = min_dim * 0.8
    
    print(f"  Polygon dimensions: {maxx-minx:.1f}m x {maxy-miny:.1f}m")
    print(f"  Recommended max separation: {recommended_max:.1f}m")
    
    # Input validation with retries
    separation_m = DEFAULT_SEPARATION_M
    max_attempts = 3
    for attempt in range(max_attempts):
        try:
            user_sep = input(f"Enter separation distance in meters [{separation_m:.1f}] (or press Enter for default): ").strip()
            
            # Empty input = use default
            if not user_sep:
                print(f"  Using default separation: {separation_m:.1f}m")
                break
            
            # Parse and validate
            sep_value = float(user_sep)
            
            if sep_value <= 0:
                print(f"  ERROR: Separation must be positive (attempt {attempt+1}/{max_attempts})")
                continue
            
            if sep_value >= min_dim:
                print(f"  ERROR: Separation {sep_value:.1f}m too large for polygon {min_dim:.1f}m (attempt {attempt+1}/{max_attempts})")
                continue
            
            if sep_value > recommended_max:
                confirm = input(f"  WARNING: {sep_value:.1f}m exceeds recommended {recommended_max:.1f}m. Continue? (y/n): ")
                if confirm.lower() != 'y':
                    continue
            
            separation_m = sep_value
            print(f"  ✓ Using separation: {separation_m:.1f}m")
            break
            
        except ValueError:
            print(f"  ERROR: Invalid number format (attempt {attempt+1}/{max_attempts})")
            if attempt == max_attempts - 1:
                print(f"  Using default separation: {separation_m:.1f}m")
        except KeyboardInterrupt:
            print("\n[ABORT] Mission cancelled by user")
            return None
        except Exception as e:
            print(f"  ERROR: {e}")
            if attempt == max_attempts - 1:
                print(f"  Using default separation: {separation_m:.1f}m")
    
    return separation_m


def generate_survey_paths(poly_m, separation_m):
    """
    Generate survey paths for two polygon halves.
    
    Args:
        poly_m: Shapely Polygon in meters
        separation_m: Separation distance between survey lines
        
    Returns:
        tuple: (path1, path2, angle_half1, angle_half2) or (None, None, None, None) on error
    """
    print("\n[STEP 3] Generating survey paths...")
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
    print("\n[STEP 4] Converting paths to waypoints...")
    
    try:
        # Create waypoint generators for both paths
        gen1 = WaypointGenerator(path1, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
        gen2 = WaypointGenerator(path2, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
        
        # Use relative altitude for now - will be converted to absolute later
        survey_altitude_relative = abs(SURVEY_ALT)  # 12.19m relative altitude
        
        # Generate waypoints for both paths
        waypoints1 = gen1.generate_waypoints(spacing_m=5.0, altitude_m=survey_altitude_relative)
        waypoints2 = gen2.generate_waypoints(spacing_m=5.0, altitude_m=survey_altitude_relative)
        
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
        print(f"  Survey altitude: {altitude_m:.1f}m ({altitude_m*FEET_PER_METER:.1f}ft)")
        
        return waypoints1, waypoints2, all_waypoints, total_distance, est_time_min
        
    except Exception as e:
        print(f"ERROR generating waypoints: {e}")
        import traceback
        traceback.print_exc()
        return None

# ============ Main Execution ============

def main():
    """Main program: Load survey path and fly drone using sarx.py patterns"""
    
    # Check if mavsdk is available
    if not MAVSDK_AVAILABLE:
        print("ERROR: mavsdk module not available")
        print("Install with: pip install mavsdk")
        return
    
    print("\n" + "="*60)
    print("KML PATH - RASPBERRY PI DRONE SURVEY")
    print("="*60)
    
    # Step 1: Load polygon from mission.plan
    poly_m, lat0, lon0, m_per_deg_lat, m_per_deg_lon = load_polygon()
    if poly_m is None:
        return
    
    # Step 2: Get survey parameters from user
    separation_m = get_survey_parameters(poly_m)
    if separation_m is None:
        return
    
    # Step 3: Generate survey paths
    path1, path2, angle_half1, angle_half2 = generate_survey_paths(poly_m, separation_m)
    if path1 is None or path2 is None:
        return
    
    # Step 4: Generate waypoints from paths
    waypoint_result = generate_waypoints_from_paths(path1, path2, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
    if waypoint_result is None:
        return
    
    waypoints1, waypoints2, all_waypoints, total_distance, est_time_min = waypoint_result
    
    # Mission confirmation prompt
    print("\n" + "="*60)
    print("MISSION PLAN SUMMARY")
    print("="*60)
    print(f"  Total waypoints: {len(all_waypoints)}")
    print(all_waypoints)
    print(f"  Total distance: {total_distance:.1f}m")
    print(f"  Estimated time: {est_time_min:.1f} minutes")
    print(f"  Survey altitude: {altitude_m:.1f}m")
    print(f"  Separation: {separation_m:.1f}m")
    print("="*60)
    
    try:
        confirm = input("\nProceed with mission? (yes/no): ").strip().lower()
        if confirm not in ['yes', 'y']:
            print("[ABORT] Mission cancelled by user")
            return
    except KeyboardInterrupt:
        print("\n[ABORT] Mission cancelled by user")
        return
    
    # Step 5: Initialize drone controller (sarx.py style)
    print("\n[STEP 5] Initializing drone controller (Raspberry Pi)...")
    # 
    try:
        drone = DroneController(system_address=SYSTEM_ADDRESS)
        drone.start()
        # 
        # Wait for drone to be ready with timeout
        print("  Waiting for drone connection...")
        wait_time = 0
        max_wait = 30
        while not drone.is_ready() and wait_time < max_wait:
            time.sleep(1)
            wait_time += 1
            if wait_time % 5 == 0:
                print(f"  Still waiting... ({wait_time}s/{max_wait}s)")
        # 
        if not drone.is_ready():
            print("\nERROR: Drone not ready after 30 seconds")
            print("  Check:")
            print("  - Pixhawk is powered on")
            print("  - Serial connection /dev/ttyACM0")
            print("  - MAVLink communication working")
            print("  - No other programs using the serial port")
            return
        # 
        print("  ✓ Drone controller ready")
        # 
        # Convert waypoint altitudes from relative to absolute MSL
        print("\n  Converting waypoint altitudes to absolute MSL...")
        time.sleep(2)  # Wait for position data
        home_alt_msl = drone.get_home_altitude_msl()
        # 
        if home_alt_msl > 0:
            # Convert all waypoints to absolute altitude
            waypoints1_abs = [(lat, lon, home_alt_msl + alt) for lat, lon, alt in waypoints1]
            waypoints2_abs = [(lat, lon, home_alt_msl + alt) for lat, lon, alt in waypoints2]
            all_waypoints_abs = waypoints1_abs + waypoints2_abs
            # 
            print(f"  ✓ Converted to absolute altitude (Home MSL: {home_alt_msl:.1f}m)")
            print(f"     Survey altitude: {survey_altitude_relative:.1f}m relative = {home_alt_msl + survey_altitude_relative:.1f}m MSL")
            # 
            # Update waypoint references
            waypoints1 = waypoints1_abs
            waypoints2 = waypoints2_abs
            all_waypoints = all_waypoints_abs
        else:
            print("  WARNING: Could not get home altitude, using relative altitudes")
        # 
    except Exception as e:
        print(f"\nERROR initializing drone controller: {e}")
        import traceback
        traceback.print_exc()
        return

    # Step 6: Save checkpoint and start navigation
    print("\n[STEP 6] Starting mission...")
    time.sleep(2)
    
    # Try to save checkpoint with retries
    checkpoint_saved = False
    for attempt in range(5):
        if drone.save_checkpoint():
            checkpoint_saved = True
            break
        print(f"  WARNING: Checkpoint save attempt {attempt+1}/5 failed, retrying...")
        time.sleep(1)
    
    if not checkpoint_saved:
        print("\nERROR: Could not save checkpoint position after 5 attempts")
        print("  GPS may not have lock yet")
        print("  Landing for safety...")
        try:
            drone.stop_and_land()
        except:
            pass
        return
    
    print("Starting mission")
    # # Final confirmation before takeoff
    # print("\n" + "="*60)
    # print("⚠️  FINAL SAFETY CHECK ⚠️")
    # print("="*60)
    # print("  Drone is ready to begin autonomous mission")
    # print(f"  {len(all_waypoints)} waypoints will be flown")
    # print("  Press Ctrl+C at any time to abort")
    # print("="*60)
    
    # try:
    #     final_confirm = input("\nType 'START' to begin mission: ").strip().upper()
    #     if final_confirm != 'START':
    #         print("[ABORT] Mission cancelled - landing drone")
    #         try:
    #             drone.stop_and_land()
    #         except:
    #             pass
    #         return
    # except KeyboardInterrupt:
    #     print("\n[ABORT] Mission cancelled by user - landing drone")
    #     try:
    #         drone.stop_and_land()
    #     except:
    #         pass
    #     return
    
    # Step 7: Fly first half survey path
    print("\n[STEP 7a] Flying FIRST HALF survey path...")
    print(f"[INFO] {len(waypoints1)} waypoints on first half")
    success_half1 = True
    failed_waypoints_half1 = 0
    max_failures = 3  # Allow 3 failures before aborting
    
    # Safety altitude threshold
    MIN_SAFE_ALTITUDE = 5.0  # meters
    
    try:
        for idx, (lat, lon, alt) in enumerate(waypoints1):
            # ALTITUDE FAILSAFE: Check if drone is too low
            current_alt = drone.get_altitude()
            if current_alt < MIN_SAFE_ALTITUDE:
                print(f"\n\n⚠️  CRITICAL ALTITUDE FAILSAFE TRIGGERED!")
                print(f"  Current altitude: {current_alt:.2f}m < {MIN_SAFE_ALTITUDE}m threshold")
                print(f"  EMERGENCY LANDING for safety...")
                try:
                    drone.stop_and_land()
                except:
                    pass
                success_half1 = False
                print("\n❌ Mission aborted due to low altitude")
                return
            
            print(f"\n[WAYPOINT {idx+1}/{len(waypoints1)} - HALF 1] (Alt: {current_alt:.1f}m)")
            print(f"  Target: {lat:.6f}, {lon:.6f} at {alt:.1f}m")
            
            if not drone.navigate_gps_waypoint(lat, lon, alt):
                print(f"  ⚠️  WARNING: Failed to reach waypoint {idx+1}")
                failed_waypoints_half1 += 1
                
                if failed_waypoints_half1 >= max_failures:
                    print(f"\n❌ ABORT: {failed_waypoints_half1} waypoints failed - too many failures")
                    print("  Returning to home for safety...")
                    success_half1 = False
                    break
            else:
                print(f"  ✓ Waypoint {idx+1} reached")
            
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("\n\n[EMERGENCY ABORT] Mission cancelled by user!")
        print("  Returning to home immediately...")
        success_half1 = False
    
    # Return to checkpoint and land
    print("\n[RETURN] Returning to checkpoint...")
    drone.return_to_checkpoint()
    time.sleep(2)
    
    print("\n[LANDING] Landing drone...")
    drone.stop_and_land()
    
    print("\n✓ Mission complete - drone has landed")
    return
    
    # # Step 8: Fly second half survey path
    # print("\n[STEP 8a] Flying SECOND HALF survey path...")
    # print(f"[INFO] {len(waypoints2)} waypoints on second half")
    # success_half2 = True
    # failed_waypoints_half2 = 0
    
    # try:
    #     for idx, (lat, lon, alt) in enumerate(waypoints2):
    #         # ALTITUDE FAILSAFE: Check if drone is too low
    #         current_alt = drone.get_altitude()
    #         if current_alt < MIN_SAFE_ALTITUDE:
    #             print(f"\n\n⚠️  CRITICAL ALTITUDE FAILSAFE TRIGGERED!")
    #             print(f"  Current altitude: {current_alt:.2f}m < {MIN_SAFE_ALTITUDE}m threshold")
    #             print(f"  EMERGENCY LANDING for safety...")
    #             try:
    #                 drone.stop_and_land()
    #             except:
    #                 pass
    #             success_half2 = False
    #             print("\n❌ Mission aborted due to low altitude")
    #             return
            
    #         print(f"\n[WAYPOINT {idx+1}/{len(waypoints2)} - HALF 2] (Alt: {current_alt:.1f}m)")
    #         print(f"  Target: {lat:.6f}, {lon:.6f} at {alt:.1f}m")
            
    #         if not drone.navigate_gps_waypoint(lat, lon, alt):
    #             print(f"  ⚠️  WARNING: Failed to reach waypoint {idx+1}")
    #             failed_waypoints_half2 += 1
                
    #             if failed_waypoints_half2 >= max_failures:
    #                 print(f"\n❌ ABORT: {failed_waypoints_half2} waypoints failed - too many failures")
    #                 print("  Returning to home for safety...")
    #                 success_half2 = False
    #                 break
    #         else:
    #             print(f"  ✓ Waypoint {idx+1} reached")
            
    #         time.sleep(0.5)
    
    # except KeyboardInterrupt:
    #     print("\n\n[EMERGENCY ABORT] Mission cancelled by user!")
    #     print("  Returning to home immediately...")
    #     success_half2 = False
    
    # if success_half2:
    #     print("\n✓ Second half survey completed successfully")
    # else:
    #     print("\nWARNING: Second half survey did not complete successfully")
    
    # # Return to checkpoint after second half
    # print("\n[STEP 8b] Returning to home after second half...")
    # drone.return_to_checkpoint()
    # time.sleep(2)
    
    # # Step 9: Land
    # print("\n[STEP 9] Landing drone...")
    # time.sleep(2)
    
    # try:
    #     drone.stop_and_land()
    # except Exception as e:
    #     print(f"ERROR during landing: {e}")
    #     print("Manual intervention may be required")
    
    # # Final mission report
    # print("\n" + "="*60)
    # print("MISSION COMPLETE - FINAL REPORT")
    # print("="*60)
    # print(f"  Half 1 Status: {'✓ SUCCESS' if success_half1 else '✗ INCOMPLETE'}")
    # print(f"    Waypoints: {len(waypoints1)}")
    # print(f"    Failed: {failed_waypoints_half1}")
    # print(f"  Half 2 Status: {'✓ SUCCESS' if success_half2 else '✗ INCOMPLETE'}")
    # print(f"    Waypoints: {len(waypoints2)}")
    # print(f"    Failed: {failed_waypoints_half2}")
    # print(f"  Total waypoints: {len(all_waypoints)}")
    # print(f"  Total failures: {failed_waypoints_half1 + failed_waypoints_half2}")
    # print(f"  Overall status: {'✓ MISSION SUCCESS' if success_half1 and success_half2 else '⚠️  MISSION INCOMPLETE'}")
    # print("="*60 + "\n")


if __name__ == "__main__":
    main()
