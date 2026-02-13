#!/usr/bin/env python3
"""
Configuration settings for SARX system.
All constants and configuration parameters in one place.
"""

from pathlib import Path

# ============ Camera Settings ============
CAM_NATIVE_SIZE = (3280, 2464)  # IMX219 full resolution (max FOV)
CAM_PREVIEW_SIZE = (1280, 720)  # Use a high-res preview for best FOV and speed
INFER_SIZE = 320
IMG_SIZE = 320
DISPLAY_WINDOW = "Human Detection Delivery"
FPS_AVG_ALPHA = 0.9

# ============ Model Settings ============
# Model paths (try Pi paths first, then fallback)
YOLOV5_DIR = Path("/home/drone/Desktop/yolov5")
WEIGHTS_PT = Path("/home/drone/Desktop/final/weights_only.pt")
MODEL_YAML = Path("/home/drone/Desktop/yolov5/models/yolov5n.yaml")
FALLBACK_MODEL = "yolo11n.pt"

# Inference settings
CONF_THRES = 0.25
IOU_THRES = 0.45
MODEL_NAMES = ["person"]

# ============ Detection Thresholds ============
PERSON_CONF_THR = 0.3
PERSON_AREA_THRESHOLD_FRONT = 0.15  # Front camera threshold to trigger approach
PERSON_AREA_THRESHOLD_BOTTOM = 0.4  # Bottom camera threshold for centered position

# ============ MAV Settings ============
SYSTEM_ADDRESS = "serial:///dev/ttyACM0:115200"
TAKEOFF_ALT = -15.24  # NED down negative = up 15.24m (50ft)
SURVEY_ALT = -12.19  # NED down negative = up 12.19m (40ft) for survey
DELIVERY_ALT = -6  # NED down negative = up 6m (20ft)
SEARCH_SPEED = 0.5  # m/s
APPROACH_SPEED = 0.2  # m/s
YAW_RATE = 30.0  # deg/s

# ============ Survey Path Settings ============
PLAN_FILE = "/home/drone/Desktop/kml/mission.plan"
DEFAULT_SEPARATION_M = 15.0  # meters between survey lines
WAYPOINT_RADIUS = 2.0  # meters - GPS accuracy radius
WAYPOINT_SPACING = 5.0  # meters between waypoints

# ============ Servo/Drop Settings ============
SERVO_NUMBERS = [9, 10, 11, 12, 13]  # 5 servos on Pixhawk
SERVO_CLOSE_PWM = 900
SERVO_OPEN_PWM = 1400
DROP_HOLD_SECONDS = 1.0

# ============ State Timing and Failsafes ============
APPROACHING_TIMEOUT = 20.0  # Max 20 seconds in APPROACHING
CENTERING_TIMEOUT = 15.0  # Max 15 seconds in CENTERING
CENTERING_LOST_TIMEOUT = 7.5  # Timeout to recover if bottom cam lost
DESCENT_RATE = 1.0  # m/s descent speed
RETURN_RATE = 1.0  # m/s return speed
DETECTION_COOLDOWN = 10.0  # Seconds to ignore detections after returning from delivery
RTL_ALTITUDE_M = 20.0  # desired RTL height (meters AGL)

# ============ Recording Settings ============
RECORDING_DIR = Path("/home/drone/Desktop/recordings")
RECORDING_FPS = 20.0

# ============ Exclusion Zone ============
EXCLUSION_RADIUS_M = 5.0  # Meters - don't re-approach within this radius of tagged locations


def main():
    """Test configuration by printing all settings"""
    print("="*60)
    print("SARX CONFIGURATION SETTINGS")
    print("="*60)
    print(f"\n[CAMERA]")
    print(f"  Native Size: {CAM_NATIVE_SIZE}")
    print(f"  Preview Size: {CAM_PREVIEW_SIZE}")
    print(f"  Inference Size: {IMG_SIZE}")
    
    print(f"\n[MODEL]")
    print(f"  YOLOv5 Dir: {YOLOV5_DIR}")
    print(f"  Weights: {WEIGHTS_PT}")
    print(f"  Fallback: {FALLBACK_MODEL}")
    
    print(f"\n[DETECTION]")
    print(f"  Front Threshold: {PERSON_AREA_THRESHOLD_FRONT}")
    print(f"  Bottom Threshold: {PERSON_AREA_THRESHOLD_BOTTOM}")
    
    print(f"\n[MAV]")
    print(f"  System Address: {SYSTEM_ADDRESS}")
    print(f"  Takeoff Alt: {TAKEOFF_ALT}m (NED)")
    print(f"  Survey Alt: {SURVEY_ALT}m (NED)")
    print(f"  Delivery Alt: {DELIVERY_ALT}m (NED)")
    
    print(f"\n[SERVO]")
    print(f"  Servo Numbers: {SERVO_NUMBERS}")
    print(f"  Close PWM: {SERVO_CLOSE_PWM}")
    print(f"  Open PWM: {SERVO_OPEN_PWM}")
    
    print("="*60)


if __name__ == "__main__":
    main()
