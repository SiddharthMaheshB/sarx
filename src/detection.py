#!/usr/bin/env python3
"""
Detection and camera management module for SARX system.
Handles camera initialization, exclusion zones, and drawing utilities.
"""

import time
import cv2
import numpy as np
from picamera2 import Picamera2
from datetime import datetime
from pathlib import Path

from config import (
    CAM_PREVIEW_SIZE, RECORDING_DIR, RECORDING_FPS, EXCLUSION_RADIUS_M
)

# Global list of tagged locations
tagged_locations = []


def init_cameras():
    """
    Initialize both IMX219 cameras for max FOV.
    
    Returns:
        tuple: (cam0, cam1) - Bottom and Front cameras
    """
    print("[CAMERA] Initializing IMX219 cameras for max FOV...")
    
    cam0 = Picamera2(1)  # Bottom camera
    cam1 = Picamera2(0)  # Front camera
    
    cfg0 = cam0.create_preview_configuration(main={"size": CAM_PREVIEW_SIZE, "format": "BGR888"})
    cfg1 = cam1.create_preview_configuration(main={"size": CAM_PREVIEW_SIZE, "format": "BGR888"})
    
    cam0.configure(cfg0)
    cam1.configure(cfg1)
    
    # Start both cameras
    cam0.start()
    cam1.start()
    time.sleep(0.5)
    
    print(f"[CAMERA] ✓ Bottom camera ready at {CAM_PREVIEW_SIZE} (IMX219 wide FOV)")
    print(f"[CAMERA] ✓ Front camera ready at {CAM_PREVIEW_SIZE} (IMX219 wide FOV)")
    
    return cam0, cam1


def init_video_writers(cam_size=CAM_PREVIEW_SIZE):
    """
    Initialize video writers for recording both camera feeds.
    
    Args:
        cam_size: Camera resolution tuple (width, height)
        
    Returns:
        tuple: (front_writer, back_writer, front_path, back_path)
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    RECORDING_DIR.mkdir(exist_ok=True)
    
    front_footage_path = RECORDING_DIR / f"front_footage_{timestamp}.avi"
    back_footage_path = RECORDING_DIR / f"back_footage_{timestamp}.avi"
    
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    front_writer = cv2.VideoWriter(str(front_footage_path), fourcc, RECORDING_FPS, cam_size)
    back_writer = cv2.VideoWriter(str(back_footage_path), fourcc, RECORDING_FPS, cam_size)
    
    print(f"[RECORDING] Front footage: {front_footage_path}")
    print(f"[RECORDING] Back footage: {back_footage_path}")
    
    return front_writer, back_writer, str(front_footage_path), str(back_footage_path)


def start_bottom_camera(cam0):
    """
    Start the bottom camera.
    
    Args:
        cam0: Bottom camera object
        
    Returns:
        bool: True if successful
    """
    try:
        cam0.start()
        print("[CAMERA] ✅ Bottom camera started")
        return True
    except Exception as e:
        print(f"[CAMERA] ⚠️  Error starting bottom camera: {e}")
        return False


def stop_bottom_camera(cam0):
    """
    Stop the bottom camera to save resources.
    
    Args:
        cam0: Bottom camera object
        
    Returns:
        bool: True if successful
    """
    try:
        cam0.stop()
        print("[CAMERA] Bottom camera stopped (idle)")
        return True
    except Exception as e:
        print(f"[CAMERA] ⚠️  Error stopping bottom camera: {e}")
        return False


def is_in_exclusion_zone(drone, exclusion_radius_m=EXCLUSION_RADIUS_M):
    """
    Check if current drone position is within exclusion radius of any tagged location.
    
    Args:
        drone: DroneController object
        exclusion_radius_m: Exclusion radius in meters
        
    Returns:
        tuple: (bool, distance) - True if in exclusion zone, and distance to nearest tagged location
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


def tag_location(drone):
    """
    Tag current location as visited (add to exclusion zone).
    
    Args:
        drone: DroneController object
        
    Returns:
        bool: True if location tagged successfully
    """
    if not drone.current_position:
        print("[TAG] Warning: No position available to tag")
        return False
    
    location = {
        'lat': drone.current_position.latitude_deg,
        'lon': drone.current_position.longitude_deg,
        'alt': drone.current_position.absolute_altitude_m,
        'timestamp': time.time()
    }
    
    tagged_locations.append(location)
    
    print(f"\n🏷️  [TAG] Location marked (Total: {len(tagged_locations)})")
    print(f"   Coordinates: ({location['lat']:.6f}, {location['lon']:.6f})")
    print(f"   Altitude: {location['alt']:.1f}m")
    
    return True


def draw_crosshair(img, color=(0, 255, 0)):
    """
    Draw crosshair in center of image for centering reference.
    
    Args:
        img: Image to draw on (modified in-place)
        color: BGR color tuple
    """
    h, w = img.shape[:2]
    center_x = w // 2
    center_y = h // 2
    
    # Draw crosshair lines
    cv2.line(img, (center_x - 30, center_y), (center_x + 30, center_y), color, 2)
    cv2.line(img, (center_x, center_y - 30), (center_x, center_y + 30), color, 2)
    
    # Draw circle
    cv2.circle(img, (center_x, center_y), 50, color, 2)


def add_status_overlay(img, state, altitude_m, found_person=False, area=0.0, offset=(0.0, 0.0)):
    """
    Add status information overlay to image.
    
    Args:
        img: Image to draw on (modified in-place)
        state: Current state string
        altitude_m: Current altitude in meters
        found_person: Whether person is detected
        area: Detection area ratio
        offset: (cx, cy) normalized offsets
    """
    # State and altitude
    cv2.putText(img, f"State: {state}", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(img, f"Alt: {altitude_m:.1f}m", (10, 60),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    # Detection info
    if found_person:
        cx, cy = offset
        det_text = f"Person: {area:.3f} ({cx:.2f}, {cy:.2f})"
        cv2.putText(img, det_text, (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)


def combine_camera_views(img0, img1, target_h=360):
    """
    Combine two camera views side by side.
    
    Args:
        img0: Bottom camera image
        img1: Front camera image
        target_h: Target height for combined view
        
    Returns:
        np.ndarray: Combined image
    """
    h0, w0 = img0.shape[:2]
    h1, w1 = img1.shape[:2]
    
    # Resize both to same height
    img0_resized = cv2.resize(img0, (int(w0 * target_h / h0), target_h))
    img1_resized = cv2.resize(img1, (int(w1 * target_h / h1), target_h))
    
    # Stack horizontally
    combined = np.hstack((img0_resized, img1_resized))
    
    return combined


def main():
    """Test detection and camera module"""
    print("="*60)
    print("DETECTION & CAMERA MODULE TEST")
    print("="*60)
    
    print("\n[TEST] Initializing cameras...")
    try:
        cam0, cam1 = init_cameras()
        print("✓ Cameras initialized")
    except Exception as e:
        print(f"✗ Camera initialization failed: {e}")
        return
    
    print("\n[TEST] Initializing video writers...")
    try:
        front_writer, back_writer, front_path, back_path = init_video_writers()
        print("✓ Video writers initialized")
    except Exception as e:
        print(f"✗ Video writer initialization failed: {e}")
        cam0.stop()
        cam1.stop()
        return
    
    print("\n[TEST] Capturing and displaying frames for 5 seconds...")
    print("  Press 'q' to quit early")
    
    start_time = time.time()
    frame_count = 0
    
    try:
        while time.time() - start_time < 5.0:
            # Capture frames
            frame0 = cam0.capture_array()
            frame1 = cam1.capture_array()
            
            # Convert RGB to BGR
            frame0_bgr = cv2.cvtColor(frame0, cv2.COLOR_RGB2BGR)
            frame1_bgr = cv2.cvtColor(frame1, cv2.COLOR_RGB2BGR)
            
            # Add overlays
            draw_crosshair(frame0_bgr)
            add_status_overlay(frame0_bgr, "TEST", 10.0)
            add_status_overlay(frame1_bgr, "TEST", 10.0)
            
            # Record frames
            back_writer.write(frame0_bgr)
            front_writer.write(frame1_bgr)
            
            # Combine and display
            combined = combine_camera_views(frame0_bgr, frame1_bgr)
            cv2.imshow("Camera Test", combined)
            
            frame_count += 1
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        duration = time.time() - start_time
        fps = frame_count / duration
        
        print(f"\n✓ Test complete")
        print(f"  Frames captured: {frame_count}")
        print(f"  Duration: {duration:.1f}s")
        print(f"  Average FPS: {fps:.1f}")
        
    except KeyboardInterrupt:
        print("\n✓ Test interrupted")
    
    finally:
        # Cleanup
        print("\n[CLEANUP] Releasing resources...")
        cam0.stop()
        cam1.stop()
        front_writer.release()
        back_writer.release()
        cv2.destroyAllWindows()
        print("✓ Cleanup complete")
    
    print("\n" + "="*60)


if __name__ == "__main__":
    main()
