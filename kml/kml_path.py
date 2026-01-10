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

# Waypoint detection
WAYPOINT_RADIUS = 2.0  # meters - GPS accuracy radius
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
                    print("[DRONE] Connected to flight controller")
                    break
            
            print("[DRONE] Arming...")
            await self.drone.action.arm()
            
            print("[DRONE] Taking off to {:.1f}m...".format(abs(TAKEOFF_ALT)))
            await self.drone.action.takeoff()
            await asyncio.sleep(8)
            
            # Switch to offboard mode (body-frame velocity control)
            print("[DRONE] Switching to offboard mode (body-frame velocity)...")
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, TAKEOFF_ALT, 0.0)
            )
            await self.drone.offboard.start()
            print("[DRONE] Offboard mode active - ready for body-frame commands")
            
            self._connected = True
            self.ready_event.set()
            
        except Exception as e:
            print(f"[DRONE] Setup error: {e}")
            self.ready_event.set()
    
    async def _position_monitor(self):
        """Continuously monitor drone position and altitude"""
        await asyncio.sleep(5)
        try:
            async for position in self.drone.telemetry.position():
                self.current_position = position
                self.current_altitude = -position.relative_altitude_m  # NED (negative down)
                self.altitude_m = abs(position.relative_altitude_m)  # Positive altitude
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
            # Stop offboard for goto_location
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
            start_time = time.time()
            while time.time() - start_time < timeout_s:
                if self.current_position:
                    dist = self._calculate_gps_distance(
                        self.current_position.latitude_deg,
                        self.current_position.longitude_deg,
                        lat, lon
                    )
                    alt_diff = abs(self.current_position.absolute_altitude_m - alt)
                    
                    if dist < tolerance_m and alt_diff < 1.0:
                        print(f"   [ARRIVED] Distance: {dist:.1f}m, Alt: {alt_diff:.1f}m")
                        # Resume offboard
                        await self.drone.offboard.set_position_ned(
                            PositionNedYaw(0.0, 0.0, TAKEOFF_ALT, 0.0)
                        )
                        await self.drone.offboard.start()
                        return True
                    
                    if int(time.time() - start_time) % 5 == 0:
                        print(f"   [PROGRESS] Distance: {dist:.1f}m, Alt: {self.altitude_m:.1f}m")
                
                await asyncio.sleep(1)
            
            return False
            
        except Exception as e:
            print(f"[DRONE] GPS navigation error: {e}")
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
        """Navigate back to checkpoint"""
        if not self.checkpoint_position:
            return False
        
        try:
            print(f"\n[DRONE] Returning to checkpoint...")
            print(f"   Target altitude: {self.checkpoint_altitude_m:.2f}m")
            
            # Stop offboard for goto_location
            await self.drone.offboard.stop()
            await asyncio.sleep(0.5)
            
            # Use GPS-based goto_location
            await self.drone.action.goto_location(
                latitude_deg=self.checkpoint_position['lat'],
                longitude_deg=self.checkpoint_position['lon'],
                absolute_altitude_m=self.checkpoint_position['alt'],
                yaw_deg=0.0
            )
            
            # Monitor arrival
            start_time = time.time()
            while time.time() - start_time < 300:
                if self.current_position:
                    dist = self._calculate_gps_distance(
                        self.current_position.latitude_deg,
                        self.current_position.longitude_deg,
                        self.checkpoint_position['lat'],
                        self.checkpoint_position['lon']
                    )
                    
                    if dist < WAYPOINT_RADIUS:
                        print("[DRONE] Returned to checkpoint")
                        # Resume offboard
                        await self.drone.offboard.set_position_ned(
                            PositionNedYaw(0.0, 0.0, self.checkpoint_altitude, 0.0)
                        )
                        await self.drone.offboard.start()
                        return True
                    
                    if int(time.time() - start_time) % 10 == 0:
                        print(f"[RETURN] Distance: {dist:.1f}m")
                
                await asyncio.sleep(1)
            
            return False
            
        except Exception as e:
            print(f"[DRONE] Return navigation error: {e}")
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
        """Get current altitude in meters"""
        return self.altitude_m
    
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
    print("\n[STEP 1] Loading polygon from mission.plan...")
    try:
        poly_m, (lat0, lon0, m_per_deg_lat, m_per_deg_lon) = cs.load_polygon_from_plan_in_meters(
            PLAN_FILE
        )
        print(f"  Polygon loaded: {poly_m.area:.1f} m²")
        print(f"  Reference: lat0={lat0:.6f}, lon0={lon0:.6f}")
    except Exception as e:
        print(f"ERROR loading polygon: {e}")
        return
    
    # Step 2: Get survey parameters from user
    print("\n[STEP 2] Survey configuration...")
    minx, miny, maxx, maxy = poly_m.bounds
    min_dim = min(maxx - minx, maxy - miny)
    recommended_max = min_dim * 0.8
    
    print(f"  Polygon dimensions: {maxx-minx:.1f}m x {maxy-miny:.1f}m")
    print(f"  Recommended max separation: {recommended_max:.1f}m")
    
    separation_m = DEFAULT_SEPARATION_M
    try:
        user_sep = input(f"Enter separation distance in meters [{separation_m:.1f}]: ").strip()
        if user_sep:
            separation_m = float(user_sep)
            if separation_m <= 0 or separation_m >= min_dim:
                print(f"  Invalid separation, using default {DEFAULT_SEPARATION_M}m")
                separation_m = DEFAULT_SEPARATION_M
    except ValueError:
        print(f"  Invalid input, using default {DEFAULT_SEPARATION_M}m")
    
    # Step 3: Generate survey paths
    print("\n[STEP 3] Generating survey paths...")
    try:
        # Split polygon into two halves
        poly1_m, poly2_m, cut_line = cs.compute_equal_area_split(poly_m, angle_rad=0.0)
        
        # Find best angles for each half
        print("  Finding optimal angles for each region...")
        angle_half1, path1, _, _, _ = cs.find_best_angle_for_region(
            poly1_m, separation_m, (0, 0), angle_step_deg=1.0
        )
        angle_half2, path2, _, _, _ = cs.find_best_angle_for_region(
            poly2_m, separation_m, (0, 0), angle_step_deg=1.0
        )
        
        if angle_half1 is None or angle_half2 is None:
            print("ERROR: Could not generate survey paths")
            print(f"  Separation {separation_m}m may be too large for polygon size")
            return
        
        print(f"  Half 1: angle={angle_half1:.1f} deg, path length={path1.length:.1f}m")
        print(f"  Half 2: angle={angle_half2:.1f} deg, path length={path2.length:.1f}m")
        
    except Exception as e:
        print(f"ERROR generating survey paths: {e}")
        return
    
    # Step 4: Generate waypoints from paths
    print("\n[STEP 4] Converting paths to waypoints...")
    
    # Create waypoint generators for both paths
    gen1 = WaypointGenerator(path1, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
    gen2 = WaypointGenerator(path2, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
    
    # Use survey altitude (40ft / 12.19m) for better imagery
    altitude_m = abs(SURVEY_ALT)
    
    waypoints1 = gen1.generate_waypoints(spacing_m=5.0, altitude_m=altitude_m)
    waypoints2 = gen2.generate_waypoints(spacing_m=5.0, altitude_m=altitude_m)
    
    print(f"  Path 1: {len(waypoints1)} waypoints")
    print(f"  Path 2: {len(waypoints2)} waypoints")
    
    # Combine waypoints (first half, then second half)
    all_waypoints = waypoints1 + waypoints2
    print(f"  Total waypoints: {len(all_waypoints)}")
    
    # Step 5: Initialize drone controller (sarx.py style)
    print("\n[STEP 5] Initializing drone controller (Raspberry Pi)...")
    drone = DroneController(system_address=SYSTEM_ADDRESS)
    drone.start()
    
    if not drone.is_ready():
        print("ERROR: Drone not ready. Check connection and try again.")
        return

    # Step 6: Save checkpoint and start navigation
    print("\n[STEP 6] Starting mission...")
    time.sleep(2)
    
    if not drone.save_checkpoint():
        print("ERROR: Could not save checkpoint position")
        drone.stop_and_land()
        return
    
    # Step 7: Fly first half survey path
    print("\n[STEP 7a] Flying FIRST HALF survey path...")
    print(f"[INFO] {len(waypoints1)} waypoints on first half")
    success_half1 = True
    for idx, (lat, lon, alt) in enumerate(waypoints1):
        print(f"\n[WAYPOINT {idx+1}/{len(waypoints1)} - HALF 1]")
        print(f"  Target: {lat:.6f}, {lon:.6f} at {alt:.1f}m")
        if not drone.navigate_gps_waypoint(lat, lon, alt):
            print(f"  WARNING: Failed to reach waypoint {idx+1}")
            success_half1 = False
            break
        time.sleep(0.5)
    
    if success_half1:
        print("\n✓ First half survey completed successfully")
    else:
        print("\nWARNING: First half survey did not complete successfully")
    
    # Return to checkpoint after first half
    print("\n[STEP 7b] Returning to home after first half...")
    drone.return_to_checkpoint()
    time.sleep(2)
    
    # Step 8: Fly second half survey path
    print("\n[STEP 8a] Flying SECOND HALF survey path...")
    print(f"[INFO] {len(waypoints2)} waypoints on second half")
    success_half2 = True
    for idx, (lat, lon, alt) in enumerate(waypoints2):
        print(f"\n[WAYPOINT {idx+1}/{len(waypoints2)} - HALF 2]")
        print(f"  Target: {lat:.6f}, {lon:.6f} at {alt:.1f}m")
        if not drone.navigate_gps_waypoint(lat, lon, alt):
            print(f"  WARNING: Failed to reach waypoint {idx+1}")
            success_half2 = False
            break
        time.sleep(0.5)
    
    if success_half2:
        print("\n✓ Second half survey completed successfully")
    else:
        print("\nWARNING: Second half survey did not complete successfully")
    
    # Return to checkpoint after second half
    print("\n[STEP 8b] Returning to home after second half...")
    drone.return_to_checkpoint()
    time.sleep(2)
    
    # Step 9: Land
    print("\n[STEP 9] Landing drone...")
    time.sleep(2)
    drone.stop_and_land()
    
    print("\n" + "="*60)
    print("MISSION COMPLETE")
    print(f"Half 1: {'SUCCESS' if success_half1 else 'INCOMPLETE'}")
    print(f"Half 2: {'SUCCESS' if success_half2 else 'INCOMPLETE'}")
    print("="*60 + "\n")


if __name__ == "__main__":
    main()
