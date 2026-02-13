#!/usr/bin/env python3
"""
Main program for SARX system.
Advanced human detection and delivery system combining dual cameras and MAVLink control.

Workflow:
1. Continuously run YOLO model detecting humans on front camera
2. When human detected:
   - Save current GPS position as checkpoint
   - Yaw towards the human and move forward
   - Once human visible in lower camera (overhead view), use it for precise navigation
   - Navigate exactly to the top of the person
   - Reduce altitude to delivery height
   - Drop payload
   - Return to checkpoint at previous altitude
3. Continue monitoring with YOLO
"""

import cv2
import time
import asyncio
import numpy as np

# Import modules
from config import (
    CAM_PREVIEW_SIZE, DISPLAY_WINDOW, FPS_AVG_ALPHA,
    PERSON_AREA_THRESHOLD_FRONT, PERSON_AREA_THRESHOLD_BOTTOM,
    DETECTION_COOLDOWN, APPROACHING_TIMEOUT, CENTERING_TIMEOUT,
    DESCENT_RATE, RETURN_RATE, DROP_HOLD_SECONDS
)
from cv import load_model, infer_pytorch, draw_results, get_person_info, USE_PYTORCH, model
from DroneController import DroneController
from servo import init_servo, drop_payload
from detection import (
    init_cameras, init_video_writers, start_bottom_camera, stop_bottom_camera,
    is_in_exclusion_zone, tag_location, draw_crosshair, add_status_overlay,
    combine_camera_views
)
from path_planning import generate_survey_waypoints, rtl_at_height


# ============ State Machine States ============
class State:
    SEARCHING = "SEARCHING"
    APPROACHING = "APPROACHING"
    CENTERING = "CENTERING"
    DESCENDING = "DESCENDING"
    DROPPING = "DROPPING"
    RETURNING = "RETURNING"


def print_state_change(old_state, new_state):
    """Print state transition with visual separator"""
    print("\n" + "="*60)
    print(f"🔄 STATE TRANSITION: {old_state} → {new_state}")
    print("="*60 + "\n")


def main():
    """Main program entry point"""
    print("=" * 60)
    print("SARX - Search and Rescue eXtended")
    print("=" * 60)
    
    # Initialize cameras
    cam0, cam1 = init_cameras()
    cam0_active = True
    
    # Initialize video writers for recording
    front_writer, back_writer, front_path, back_path = init_video_writers()
    
    # Load YOLO model
    if not load_model():
        print("[ERROR] Failed to load model, exiting")
        return
    
    # Initialize servo
    servo = init_servo()
    
    # Initialize drone controller
    print("[DRONE] Initializing drone controller...")
    drone = DroneController()
    drone.start()
    
    # Generate survey waypoints
    print("\n[SURVEY] Generating survey path...")
    
    # Uncomment to enable survey waypoints from mission.plan
    survey_waypoints = generate_survey_waypoints()
    
    # Use manual waypoints for testing
    # survey_waypoints = [
    #     (28.416159, 77.525385, 10),
    #     (28.416092, 77.525227, 10),
    #     (28.416159, 77.525385, 10)
    # ]
    
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
    
    try:
        while True:
            # Capture frames from both cameras
            frame0_rgb = cam0.capture_array()
            frame1_rgb = cam1.capture_array()
            
            frame0 = cv2.cvtColor(frame0_rgb, cv2.COLOR_RGB2BGR)  # Bottom
            frame1 = cv2.cvtColor(frame1_rgb, cv2.COLOR_RGB2BGR)  # Front
            
            vis0 = frame0.copy()
            vis1 = frame1.copy()
            
            # Run inference on both cameras
            if USE_PYTORCH:
                det0 = infer_pytorch(frame0, model)
                det1 = infer_pytorch(frame1, model)
                draw_results(vis0, det0, model, use_pytorch=True)
                draw_results(vis1, det1, model, use_pytorch=True)
                results = [det0, det1]
            else:
                print("[ERROR] Ultralytics not implemented")
                break
            
            # Get detection info
            h0, w0 = frame0.shape[:2]
            h1, w1 = frame1.shape[:2]
            
            area_bottom, cx_bottom, cy_bottom, found_bottom = get_person_info(
                results[0], model, w0, h0, use_pytorch=USE_PYTORCH
            )
            area_front, cx_front, cy_front, found_front = get_person_info(
                results[1], model, w1, h1, use_pytorch=USE_PYTORCH
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
                time_since_return = time.time() - last_return_time
                
                if found_front and area_front > PERSON_AREA_THRESHOLD_FRONT:
                    if time_since_return > DETECTION_COOLDOWN:
                        print(f"\n🔍 [DETECTION] Human detected in FRONT camera!")
                        print(f"   Area ratio: {area_front:.3f} (threshold: {PERSON_AREA_THRESHOLD_FRONT})")
                        print(f"   Position offset: ({cx_front:.2f}, {cy_front:.2f})")
                        if drone.save_checkpoint():
                            current_state = State.APPROACHING
                            state_start_time = time.time()
                            if not cam0_active:
                                start_bottom_camera(cam0)
                                cam0_active = True
                    else:
                        if elapsed % 5 < 0.1:
                            print(f"⏳ [COOLDOWN] Ignoring detection ({time_since_return:.0f}s/{DETECTION_COOLDOWN}s)")
                
                # Navigate to next waypoint if survey path available
                elif survey_waypoints and len(survey_waypoints) > 0:
                    if current_waypoint_idx < len(survey_waypoints):
                        lat, lon, alt = survey_waypoints[current_waypoint_idx]
                        
                        if drone.current_position:
                            dist = drone._calculate_gps_distance(
                                drone.current_position.latitude_deg,
                                drone.current_position.longitude_deg,
                                lat, lon
                            )
                            
                            if dist > 2.0:  # Not at waypoint yet
                                if elapsed % 10 < 0.1:
                                    print(f"[SEARCHING] Navigating to waypoint {current_waypoint_idx+1}/{len(survey_waypoints)} ({dist:.1f}m)")
                                # Navigate in background
                                if elapsed % 5 < 0.1:
                                    drone.navigate_to_waypoint(lat, lon, alt)
                            else:
                                # Reached waypoint
                                print(f"✅ [WAYPOINT] Reached waypoint {current_waypoint_idx+1}")
                                current_waypoint_idx += 1
                    else:
                        # Restart survey
                        current_waypoint_idx = 0
                
                else:
                    # No survey path, return to RTL
                    if elapsed % 30 < 0.1:
                        print("[SEARCHING] No survey path - initiating RTL")
                        asyncio.run_coroutine_threadsafe(
                            rtl_at_height(drone.drone),
                            drone.loop
                        )
            
            elif current_state == State.APPROACHING:
                # Check exclusion zone
                in_exclusion, dist_to_tagged = is_in_exclusion_zone(drone)
                if in_exclusion:
                    print(f"\n⛔ [EXCLUSION ZONE] Within {dist_to_tagged:.2f}m of previously tagged location!")
                    print("   Aborting approach and returning to checkpoint...")
                    if cam0_active:
                        stop_bottom_camera(cam0)
                        cam0_active = False
                    drone.return_to_checkpoint()
                    last_return_time = time.time()
                    current_state = State.SEARCHING
                    state_start_time = time.time()
                
                # Timeout protection
                elif elapsed > APPROACHING_TIMEOUT:
                    print(f"\n⏱️  [TIMEOUT] Approaching timeout after {elapsed:.1f}s!")
                    print("   Returning to search mode...")
                    if cam0_active:
                        stop_bottom_camera(cam0)
                        cam0_active = False
                    current_state = State.SEARCHING
                    state_start_time = time.time()
                
                # Check for bottom camera detection
                elif found_bottom:
                    print(f"\n👁️  [TRANSITION] Human detected in BOTTOM camera!")
                    print(f"   Bottom area: {area_bottom:.3f}")
                    print(f"   Switching to CENTERING mode")
                    current_state = State.CENTERING
                    state_start_time = time.time()
                
                # Continue approaching using front camera
                elif found_front:
                    if abs(cx_front) > 0.1:
                        yaw_rate = cx_front * 30.0
                        drone.yaw_towards(yaw_rate_deg=yaw_rate, duration=0.3)
                        if elapsed % 2 < 0.1:
                            print(f"🔄 [APPROACHING] Yawing towards target ({cx_front:.2f})")
                    else:
                        drone.move_forward(speed=0.2, duration=0.3)
                        if elapsed % 2 < 0.1:
                            print(f"➡️  [APPROACHING] Moving forward")
                
                else:
                    if elapsed % 1 < 0.1:
                        print(f"⚠️  [WARNING] Lost target in front camera! ({elapsed:.1f}s elapsed)")
            
            elif current_state == State.CENTERING:
                # Check exclusion zone
                in_exclusion, dist_to_tagged = is_in_exclusion_zone(drone)
                if in_exclusion:
                    print(f"\n⛔ [EXCLUSION ZONE] Within {dist_to_tagged:.2f}m of previously tagged location!")
                    if cam0_active:
                        stop_bottom_camera(cam0)
                        cam0_active = False
                    drone.return_to_checkpoint()
                    last_return_time = time.time()
                    current_state = State.SEARCHING
                    state_start_time = time.time()
                
                # Timeout protection
                elif elapsed > CENTERING_TIMEOUT:
                    print(f"\n⏱️  [TIMEOUT] Centering timeout after {elapsed:.1f}s!")
                    if cam0_active:
                        stop_bottom_camera(cam0)
                        cam0_active = False
                    current_state = State.APPROACHING
                    state_start_time = time.time()
                
                elif found_bottom:
                    if abs(cx_bottom) < 0.10 and abs(cy_bottom) < 0.10:
                        if area_bottom >= PERSON_AREA_THRESHOLD_BOTTOM:
                            print(f"\n🎯 [CENTERED] Person centered and close enough!")
                            print(f"   Area: {area_bottom:.3f}, Offset: ({cx_bottom:.2f}, {cy_bottom:.2f})")
                            print(f"   Tagging location and proceeding to descent...")
                            tag_location(drone)
                            current_state = State.DESCENDING
                            state_start_time = time.time()
                        else:
                            drone.move_forward(speed=0.2, duration=0.3)
                            if elapsed % 2 < 0.1:
                                print(f"➡️  [CENTERING] Moving closer (area: {area_bottom:.3f})")
                    else:
                        forward_vel = -cy_bottom * 0.5
                        right_vel = cx_bottom * 0.5
                        drone.move_with_yaw(forward=forward_vel, right=right_vel, down=0.0, yaw_rate=0.0, duration=0.3)
                        if elapsed % 2 < 0.1:
                            print(f"🎯 [CENTERING] Adjusting position ({cx_bottom:.2f}, {cy_bottom:.2f})")
                
                else:
                    if elapsed % 1 < 0.1:
                        print(f"⚠️  [WARNING] Lost bottom view! ({elapsed:.1f}s)")
                    drone.move_with_yaw(forward=0.0, right=0.0, down=-0.3, yaw_rate=0.0, duration=0.3)
            
            elif current_state == State.DESCENDING:
                descent_distance = drone.checkpoint_altitude_m - 6.0
                descend_time = descent_distance / DESCENT_RATE
                
                if elapsed < descend_time:
                    if elapsed % 1 < 0.1:
                        remaining_time = descend_time - elapsed
                        print(f"⬇️  [DESCENDING] Descending... ({remaining_time:.1f}s remaining)")
                    drone.move_with_yaw(forward=0.0, right=0.0, down=DESCENT_RATE, yaw_rate=0.0, duration=0.3)
                else:
                    print(f"\n✅ [READY] Delivery altitude reached")
                    current_state = State.DROPPING
                    state_start_time = time.time()
            
            elif current_state == State.DROPPING:
                if elapsed < DROP_HOLD_SECONDS:
                    if elapsed < 0.1:
                        drop_payload(servo)
                else:
                    current_state = State.RETURNING
                    state_start_time = time.time()
            
            elif current_state == State.RETURNING:
                altitude_diff = abs(drone.checkpoint_altitude_m - drone.get_altitude())
                return_time = altitude_diff / RETURN_RATE
                
                if elapsed < return_time:
                    if elapsed % 1 < 0.1:
                        remaining_time = return_time - elapsed
                        print(f"⬆️  [RETURNING] Ascending... ({remaining_time:.1f}s remaining)")
                    
                    if drone.get_altitude() < drone.checkpoint_altitude_m:
                        drone.move_with_yaw(forward=0.0, right=0.0, down=-RETURN_RATE, yaw_rate=0.0, duration=0.3)
                    else:
                        drone.move_with_yaw(forward=0.0, right=0.0, down=RETURN_RATE, yaw_rate=0.0, duration=0.3)
                else:
                    print("\n" + "="*60)
                    print("✅ [COMPLETE] Delivery mission complete!")
                    print(f"   Final altitude: {drone.get_altitude():.2f}m")
                    print("   Resuming search for next target...")
                    print("="*60)
                    if cam0_active:
                        stop_bottom_camera(cam0)
                        cam0_active = False
                    last_return_time = time.time()
                    current_state = State.SEARCHING
                    state_start_time = time.time()
            
            # Print state transition if changed
            if old_state != current_state:
                print_state_change(old_state, current_state)
            
            # ========== Visualization ==========
            # Add status overlays
            add_status_overlay(vis0, current_state, drone.get_altitude(), 
                             found_bottom, area_bottom, (cx_bottom, cy_bottom))
            add_status_overlay(vis1, current_state, drone.get_altitude(),
                             found_front, area_front, (cx_front, cy_front))
            
            # Draw crosshair on bottom camera during centering
            if current_state in [State.CENTERING, State.DESCENDING]:
                draw_crosshair(vis0)
            
            # Record frames
            if back_writer.isOpened():
                back_writer.write(vis0)
            if front_writer.isOpened():
                front_writer.write(vis1)
            
            # Combine and display views
            combined = combine_camera_views(vis0, vis1, target_h=360)
            
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
        
        # Release video writers
        try:
            if front_writer.isOpened():
                front_writer.release()
                print(f"[RECORDING] ✓ Front footage saved: {front_path}")
        except Exception as e:
            print(f"[CLEANUP] Front writer error: {e}")
        try:
            if back_writer.isOpened():
                back_writer.release()
                print(f"[RECORDING] ✓ Back footage saved: {back_path}")
        except Exception as e:
            print(f"[CLEANUP] Back writer error: {e}")
        
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
