# SARX - Refactored Source Code

## Directory Structure

```
src/
├── config.py           # Configuration settings and constants
├── cv.py              # Computer vision (YOLO model loading and inference)
├── waypoints.py       # Waypoint generation from paths
├── path_planning.py   # Survey path planning and polygon loading
├── DroneController.py # Drone control via MAVSDK
├── servo.py           # Servo control for payload drops
├── detection.py       # Camera management and detection utilities
├── main.py            # Main program with state machine
└── README.md          # This file
```

## Module Overview

### config.py
Contains all configuration constants:
- Camera settings (resolution, FPS)
- Model paths and inference settings
- Detection thresholds
- MAV/drone settings (altitudes, speeds)
- Survey path settings
- Servo PWM values
- State machine timeouts

**Test**: `python config.py` - Prints all configuration values

### cv.py
Computer vision module for YOLO detection:
- `load_model()` - Load YOLOv5 PyTorch model (optimized for Raspberry Pi)
- `infer_pytorch()` - Run inference on frame
- `draw_results()` - Draw detection boxes on image
- `get_person_info()` - Extract person detection info (area, center offset)

**Test**: `python cv.py` - Loads model and tests inference

### waypoints.py
GPS waypoint generation:
- `WaypointGenerator` class - Converts Shapely paths to GPS coordinates
- `generate_waypoints()` - Generate evenly-spaced waypoints along path
- Handles coordinate transformations (local meters ↔ GPS)

**Test**: `python waypoints.py` - Creates test path and generates waypoints

### path_planning.py
Survey path planning:
- `load_survey_polygon()` - Load polygon from mission.plan
- `generate_survey_paths()` - Split polygon and generate optimal survey paths
- `generate_waypoints_from_paths()` - Convert paths to GPS waypoints
- `rtl_at_height()` - Return to launch at specified altitude

**Test**: `python path_planning.py` - Loads polygon and generates survey path

### DroneController.py
Drone control via MAVSDK:
- `DroneController` class - Manages all drone operations in async thread
- Connection, arming, takeoff, offboard mode
- Position monitoring and checkpoint saving
- Movement commands (forward, yaw, velocity)
- GPS waypoint navigation
- Return to checkpoint

**Test**: `python DroneController.py` - Connects to drone and tests basic operations

**Note**: Requires actual drone connection to test

### servo.py
Servo control for payload drops:
- `init_servo()` - Initialize pymavlink connection and servos
- `drop_payload()` - Drop payload from next available servo
- Tracks which servos have been used (5 servos total)

**Test**: `python servo.py` - Initializes servos (will trigger real servo!)

**Warning**: Test will actually trigger servo movement if Pixhawk connected

### detection.py
Camera and detection utilities:
- `init_cameras()` - Initialize dual Picamera2 cameras
- `init_video_writers()` - Setup video recording
- `start_bottom_camera()` / `stop_bottom_camera()` - Manage bottom camera
- `is_in_exclusion_zone()` - Check if near previously tagged location
- `tag_location()` - Mark location as visited
- `draw_crosshair()` - Draw centering reference
- `add_status_overlay()` - Add status info to display
- `combine_camera_views()` - Combine front/bottom camera views

**Test**: `python detection.py` - Captures and displays camera feeds for 5 seconds

**Note**: Requires Raspberry Pi with cameras connected

### main.py
Main program with state machine:
- Coordinates all modules
- State machine: SEARCHING → APPROACHING → CENTERING → DESCENDING → DROPPING → RETURNING
- Human detection and tracking
- Autonomous navigation and delivery
- Video recording

**Run**: `python main.py` - Starts full SARX system

## Usage

### Running the Full System

```bash
cd src/
python main.py
```

### Testing Individual Modules

Each module has a `main()` function for testing:

```bash
# Test configuration
python config.py

# Test computer vision
python cv.py

# Test waypoint generation
python waypoints.py

# Test path planning
python path_planning.py

# Test drone controller (requires drone)
python DroneController.py

# Test servo (WARNING: will trigger servo)
python servo.py

# Test cameras (requires Pi with cameras)
python detection.py
```

## Dependencies

```bash
# Core
pip install opencv-python numpy torch

# Camera (Raspberry Pi)
pip install picamera2

# Drone
pip install mavsdk pymavlink

# Path planning (requires completesurvey module)
# Located at /home/drone/Desktop/kml/completesurvey.py

# Geometry
pip install shapely
```

## Configuration

Edit `config.py` to change:
- Camera resolution and settings
- Model paths
- Detection thresholds (how close to approach, when to trigger)
- Flight parameters (altitudes, speeds)
- Survey line spacing
- Servo PWM values

## State Machine Flow

```
SEARCHING
  ↓ (human detected in front camera)
APPROACHING
  ↓ (human visible in bottom camera)
CENTERING
  ↓ (centered over human)
DESCENDING
  ↓ (reached delivery altitude)
DROPPING
  ↓ (payload dropped)
RETURNING
  ↓ (returned to checkpoint altitude)
SEARCHING (loop)
```

## Safety Features

1. **Exclusion Zones**: Won't approach within 5m of previously tagged locations
2. **Timeouts**: Auto-abort if states take too long
3. **Cooldown**: 10s cooldown after delivery before detecting next person
4. **Connection Check**: Lands immediately if drone connection lost

## Recording

All flights are automatically recorded:
- Front camera: `/home/drone/Desktop/recordings/front_footage_TIMESTAMP.avi`
- Bottom camera: `/home/drone/Desktop/recordings/back_footage_TIMESTAMP.avi`

## Troubleshooting

### Model won't load
- Check paths in `config.py` (YOLOV5_DIR, WEIGHTS_PT, MODEL_YAML)
- Ensure YOLOv5 repository is cloned to expected location

### Cameras not working
- Run `libcamera-hello` to test cameras
- Check camera numbers in `detection.py` (0 and 1)

### Drone won't connect
- Check SYSTEM_ADDRESS in `config.py`
- Verify Pixhawk connected at `/dev/ttyACM0`
- Test with `python DroneController.py`

### Survey path generation fails
- Check PLAN_FILE exists
- Verify polygon is valid (not too small or irregular)
- Try increasing DEFAULT_SEPARATION_M

## License

MIT License
