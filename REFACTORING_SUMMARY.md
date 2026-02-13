# SARX Code Refactoring Summary

## Overview

The SARX codebase has been refactored from a single large file (`sarx_test.py`) into a modular structure under the `src/` directory. This improves maintainability, testability, and code organization.

## New Structure

```
sarx/
├── src/
│   ├── __init__.py         # Package initialization
│   ├── config.py           # Configuration constants
│   ├── cv.py               # Computer vision module
│   ├── waypoints.py        # Waypoint generation
│   ├── path_planning.py    # Survey path planning
│   ├── DroneController.py  # Drone control
│   ├── servo.py            # Servo control
│   ├── detection.py        # Camera & detection utilities
│   ├── main.py             # Main program
│   ├── requirements.txt    # Dependencies
│   └── README.md           # Module documentation
├── sarx_test.py            # Original monolithic file (kept for reference)
└── REFACTORING_SUMMARY.md  # This file
```

## Module Breakdown

### 1. config.py
**Purpose**: Central configuration file for all constants

**Contents**:
- Camera settings (resolution, FPS, display)
- Model paths and inference parameters
- Detection thresholds
- MAV/drone settings (altitudes, speeds, connection)
- Survey path parameters
- Servo PWM values
- State machine timeouts and failsafes

**Benefits**:
- Single place to modify settings
- Easy to create different configurations (dev, prod, test)
- No hardcoded values scattered throughout code

**Testing**: `python config.py` prints all configuration values

---

### 2. cv.py
**Purpose**: Computer vision and YOLO inference

**Contents**:
- `load_model()` - PyTorch YOLOv5 model loading (CPU optimized)
- `infer_pytorch()` - Run inference on frames
- `draw_results()` - Draw bounding boxes on images
- `get_person_info()` - Extract person detection data (area, center offset)

**Benefits**:
- Isolates CV code from drone/camera logic
- Easy to swap YOLO versions or try different models
- Can test inference independently

**Testing**: `python cv.py` loads model and tests on blank image

---

### 3. waypoints.py
**Purpose**: GPS waypoint generation from survey paths

**Contents**:
- `WaypointGenerator` class - Converts Shapely LineStrings to GPS coordinates
- Coordinate transformation (local meters ↔ lat/lon)
- Waypoint spacing and interpolation

**Benefits**:
- Reusable for any path planning application
- Clean separation from survey logic
- Easy to test coordinate transformations

**Testing**: `python waypoints.py` generates waypoints from test path

---

### 4. path_planning.py
**Purpose**: Survey path generation and mission planning

**Contents**:
- `load_survey_polygon()` - Load mission.plan file
- `generate_survey_paths()` - Split polygon and generate optimal coverage paths
- `generate_waypoints_from_paths()` - Convert paths to GPS waypoints
- `rtl_at_height()` - Return to launch at specified altitude

**Benefits**:
- Complex path planning isolated from main logic
- Can be used for mission planning without flying
- Easy to visualize and debug paths

**Testing**: `python path_planning.py` generates full survey from mission.plan

---

### 5. DroneController.py
**Purpose**: All drone operations via MAVSDK

**Contents**:
- `DroneController` class with async thread management
- Connection, arming, takeoff, offboard mode
- Position monitoring and GPS tracking
- Movement commands (velocity, yaw, forward)
- GPS waypoint navigation
- Checkpoint save/return functionality
- Haversine distance calculation

**Benefits**:
- All drone logic in one place
- Async operations properly isolated
- Easy to mock for testing without hardware
- Reusable for other drone projects

**Testing**: `python DroneController.py` connects and tests basic operations (requires drone)

---

### 6. servo.py
**Purpose**: Payload drop servo control

**Contents**:
- `init_servo()` - pymavlink servo initialization
- `drop_payload()` - Drop from next available servo
- Servo usage tracking (5 servos max)
- PWM control utilities

**Benefits**:
- Servo logic isolated from main state machine
- Easy to test servo sequences
- Tracks usage to prevent overdrop

**Testing**: `python servo.py` initializes and tests servo (⚠️ triggers real servo!)

---

### 7. detection.py
**Purpose**: Camera management and detection utilities

**Contents**:
- `init_cameras()` - Initialize dual Picamera2 cameras
- `init_video_writers()` - Setup recording
- `start_bottom_camera()` / `stop_bottom_camera()` - Manage bottom camera lifecycle
- `is_in_exclusion_zone()` - Check proximity to tagged locations
- `tag_location()` - Mark visited locations
- `draw_crosshair()` - Centering reference overlay
- `add_status_overlay()` - Status information display
- `combine_camera_views()` - Side-by-side camera display

**Benefits**:
- Camera operations separate from CV inference
- Exclusion zone logic isolated and testable
- Display utilities reusable
- Easy to add new camera features

**Testing**: `python detection.py` captures and displays camera feeds for 5 seconds

---

### 8. main.py
**Purpose**: Main program and state machine

**Contents**:
- State machine (`SEARCHING` → `APPROACHING` → `CENTERING` → `DESCENDING` → `DROPPING` → `RETURNING`)
- Integration of all modules
- Main loop with frame processing
- State transition logic
- Failsafes and timeouts
- Cleanup and shutdown

**Benefits**:
- Clear high-level program flow
- State machine easy to understand and modify
- All components integrated cleanly
- Proper error handling and cleanup

**Running**: `python main.py` starts full SARX system

---

## Key Improvements

### 1. Modularity
- Each module has a single, clear responsibility
- Modules can be tested independently
- Easy to modify one part without affecting others

### 2. Testability
- Every module has a `main()` function for testing
- Mock-able interfaces for hardware components
- Can test logic without drone/cameras

### 3. Maintainability
- Configuration in one place
- Clear separation of concerns
- Easy to find and fix bugs
- Better code organization

### 4. Reusability
- Modules can be used in other projects
- `WaypointGenerator` can handle any path
- `DroneController` is project-agnostic
- CV module easily swappable

### 5. Documentation
- Each module has clear docstrings
- README explains structure and usage
- Easy onboarding for new developers

## Migration Guide

### From Old Code
```python
# Old: Everything in one file
python sarx_test.py
```

### To New Code
```python
# New: Run from src directory
cd src/
python main.py
```

### Importing Modules
```python
# Import from package
from src import DroneController, load_model
from src.waypoints import WaypointGenerator
from src.config import TAKEOFF_ALT, SURVEY_ALT

# Create instances
drone = DroneController()
gen = WaypointGenerator(path, lat0, lon0, m_lat, m_lon)
```

### Running Tests
```bash
# Test individual modules
cd src/

python config.py           # Print configuration
python cv.py              # Test model loading
python waypoints.py       # Test waypoint generation
python path_planning.py   # Test survey planning
python DroneController.py # Test drone (needs hardware)
python servo.py           # Test servo (needs hardware)
python detection.py       # Test cameras (needs Pi)
```

## Configuration Changes

All configuration is now in `src/config.py`. To customize:

```python
# Edit src/config.py

# Change detection thresholds
PERSON_AREA_THRESHOLD_FRONT = 0.15  # Increase to require closer approach
PERSON_AREA_THRESHOLD_BOTTOM = 0.4  # Increase for tighter centering

# Change flight parameters
TAKEOFF_ALT = -15.24        # Takeoff altitude (NED)
SURVEY_ALT = -12.19         # Survey altitude
DELIVERY_ALT = -6           # Delivery altitude
APPROACH_SPEED = 0.2        # Approach speed (m/s)

# Change timeouts
APPROACHING_TIMEOUT = 20.0  # Max seconds in APPROACHING state
CENTERING_TIMEOUT = 15.0    # Max seconds in CENTERING state
DETECTION_COOLDOWN = 10.0   # Cooldown after delivery
```

## Dependencies

Install all dependencies:
```bash
cd src/
pip install -r requirements.txt
```

## File Size Comparison

| File | Original | Refactored | Change |
|------|----------|------------|--------|
| Total | 1712 lines | 8 files (~1800 lines) | +88 lines |
| But... | 1 monolithic file | Modular, testable, documented | Much better! |

## Future Enhancements

The modular structure makes it easy to add:

1. **Multiple detection models**: Swap in `cv.py`
2. **Different drones**: Implement `DroneController` interface
3. **Additional sensors**: Add module in `src/`
4. **Web interface**: Create `web.py` module
5. **Logging system**: Add `logger.py` module
6. **Configuration files**: Support `.yaml` or `.json` config
7. **Unit tests**: Create `tests/` directory
8. **Simulation mode**: Mock hardware for testing

## Notes

- Original `sarx_test.py` is kept for reference
- All functionality is preserved in refactored code
- Each module is self-documented
- Testing is built into every module
- Configuration is centralized and easy to modify

## Support

For questions or issues:
1. Check module README: `src/README.md`
2. Run module tests to isolate issues
3. Review configuration in `src/config.py`
4. Check original code in `sarx_test.py` for reference
