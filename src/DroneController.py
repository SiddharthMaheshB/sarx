#!/usr/bin/env python3
"""
Drone controller module for SARX system.
Handles all drone operations via MAVSDK in a separate async thread.
"""

import asyncio
import threading
import time
import math

try:
    from mavsdk import System
    from mavsdk.offboard import VelocityBodyYawspeed, PositionNedYaw
    from mavsdk.telemetry import Position
    MAVSDK_AVAILABLE = True
except Exception as e:
    print(f"[ERROR] mavsdk import failed: {e}")
    MAVSDK_AVAILABLE = False

from config import (
    SYSTEM_ADDRESS, TAKEOFF_ALT, SURVEY_ALT, DELIVERY_ALT,
    APPROACH_SPEED, YAW_RATE, WAYPOINT_RADIUS, DESCENT_RATE, RETURN_RATE
)


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
        Calculate distance between two GPS coordinates using Haversine formula.
        Returns distance in meters.
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


def main():
    """Test drone controller"""
    print("="*60)
    print("DRONE CONTROLLER TEST")
    print("="*60)
    
    if not MAVSDK_AVAILABLE:
        print("✗ MAVSDK not available - cannot test")
        return
    
    print("\n[TEST] Creating DroneController...")
    print(f"  Address: {SYSTEM_ADDRESS}")
    
    try:
        drone = DroneController(system_address=SYSTEM_ADDRESS)
        print("✓ DroneController created")
        
        print("\n[TEST] Starting controller...")
        drone.start()
        
        if drone.is_ready():
            print("✓ Drone connected and ready")
            print(f"  Altitude: {drone.get_altitude():.2f}m")
            
            # Wait a bit to collect position
            time.sleep(3)
            
            if drone.current_position:
                print(f"  Position: ({drone.current_position.latitude_deg:.6f}, "
                      f"{drone.current_position.longitude_deg:.6f})")
            
            # Test checkpoint save
            print("\n[TEST] Saving checkpoint...")
            if drone.save_checkpoint():
                print("✓ Checkpoint saved")
        else:
            print("✗ Drone not ready")
        
        # Clean shutdown
        print("\n[TEST] Shutting down...")
        drone.stop_and_land()
        print("✓ Shutdown complete")
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
    
    print("\n" + "="*60)


if __name__ == "__main__":
    main()
