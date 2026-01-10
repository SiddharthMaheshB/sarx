# KML Path Drone Navigation System

## Overview

`kml_path.py` is an integrated system that combines survey path generation with autonomous drone navigation. It:

1. **Loads mission boundaries** from `mission.plan`
2. **Generates optimal survey paths** using `completesurvey.py` algorithms
3. **Converts paths to GPS waypoints** with precise lat/lon coordinates
4. **Autonomously navigates the drone** through the complete survey path
5. **Returns to takeoff point** on completion

## Architecture

```
┌─────────────────────────────────────────────┐
│         kml_path.py (Main Controller)       │
└──────────────┬──────────────────────────────┘
               │
     ┌─────────┼──────────┐
     │         │          │
     v         v          v
┌─────────┐ ┌──────────┐ ┌──────────────┐
│ custom_ │ │complete_ │ │ Drone Path   │
│ survey  │ │ survey   │ │ Controller   │
└─────────┘ └──────────┘ └──────────────┘
     │         │              │
     └─────────┼──────────────┘
               │
        ┌──────v──────┐
        │ mission.plan│
        └─────────────┘
```

## Key Components

### WaypointGenerator
Converts Shapely LineString survey paths into navigable GPS waypoints:
- Takes local metric coordinates from path
- Converts to GPS lat/lon using reference point
- Creates waypoint list with consistent spacing
- Handles path interpolation

**Methods:**
- `generate_waypoints(spacing_m=5.0)` - Create evenly-spaced waypoints
- `get_gps_waypoints()` - Return GPS coordinates
- `get_local_waypoints()` - Return local meter coordinates

### DronePathController
Enhanced drone controller with waypoint navigation:
- Manages drone connection and setup
- Monitors position continuously
- Navigates through waypoint sequences
- Handles checkpoint saving and returns
- Provides failsafe abort capability

**Key Methods:**
- `navigate_waypoints(waypoints_gps)` - Execute survey path
- `save_checkpoint()` - Save return position
- `return_to_checkpoint()` - Autonomous return
- `stop_and_land()` - Safe shutdown

## Workflow

### Step 1: Load Polygon
```python
poly_m, (lat0, lon0, m_per_deg_lat, m_per_deg_lon) = cs.load_polygon_from_plan_in_meters(
    PLAN_FILE
)
```
- Reads mission.plan geoFence
- Converts to local meter coordinates
- Provides reference point for conversions

### Step 2: Configure Survey
- User specifies line spacing (separation_m)
- System validates against polygon size
- Default: 15 meters

### Step 3: Generate Paths
```python
poly1_m, poly2_m, cut_line = cs.compute_equal_area_split(poly_m)
angle1, path1, ... = cs.find_best_angle_for_region(poly1_m, separation_m, ...)
```
- Splits polygon into two equal-area halves
- Finds optimal line orientation for each half
- Minimizes total flight distance

### Step 4: Create Waypoints
```python
gen1 = WaypointGenerator(path1, lat0, lon0, m_per_deg_lat, m_per_deg_lon)
waypoints1 = gen1.generate_waypoints(spacing_m=5.0)
```
- Interpolates points along path at 5m spacing
- Converts each point from meters to GPS coordinates
- Maintains takeoff altitude for entire survey

### Step 5: Drone Initialization
- Connects to drone via MAVLink
- Arms and takes off
- Switches to offboard mode
- Ready for waypoint navigation

### Step 6: Waypoint Navigation
```python
drone.navigate_waypoints(all_waypoints)
```
- Sends goto_location command for each waypoint
- Monitors arrival (within 2m)
- Moves to next waypoint on arrival
- Timeout protection per waypoint (2 minutes)

### Step 7: Return & Land
- Navigates back to checkpoint (takeoff point)
- Lands safely
- Closes all connections

## Usage

### Basic Usage
```bash
python kml_path.py
```

When prompted:
```
Enter separation distance in meters [15.0]: 15
```

### Advanced Configuration

Edit the Configuration section in `kml_path.py`:

```python
# Survey parameters
TAKEOFF_ALT = -15.24  # NED: negative = up
SURVEY_CRUISE_SPEED = 8.0  # m/s
WAYPOINT_RADIUS = 2.0  # meters acceptance

# Flight parameters
YAW_RATE = 30.0  # deg/s
APPROACH_SPEED = 1.0  # m/s

# Defaults
DEFAULT_SEPARATION_M = 15.0  # meters
```

### Customizing Waypoint Spacing

In `main()`, change waypoint generation:
```python
waypoints1 = gen1.generate_waypoints(spacing_m=3.0)  # 3m spacing instead of 5m
```

Smaller spacing = more waypoints = finer path tracking
Larger spacing = fewer waypoints = faster execution

## Output Information

The system provides detailed logging:

```
============================================================
KML PATH DRONE NAVIGATION
============================================================

[STEP 1] Loading polygon from mission.plan...
  Polygon loaded: 876.2 m²
  Reference: lat0=17.522304, lon0=78.367444

[STEP 2] Survey configuration...
  Polygon dimensions: 58.9m x 31.0m
  Recommended max separation: 24.7m

[STEP 3] Generating survey paths...
  Finding optimal angles for each region...
  Half 1: angle=45.0°, path length=187.3m
  Half 2: angle=135.0°, path length=192.1m

[STEP 4] Converting paths to waypoints...
  Path 1: 38 waypoints
  Path 2: 39 waypoints
  Total waypoints: 77

[STEP 5] Initializing drone controller...
[DRONE] Connected
[DRONE] Offboard started

[STEP 6] Starting mission...
CHECKPOINT saved at altitude 15.24m

[STEP 7] Flying waypoint path...
[WAYPOINT 1/77]
  Target: 17.522310, 78.367450 at 15.24m
  [PROGRESS] Distance: 45.2m, Alt: 15.2m
  [ARRIVED] Distance: 1.8m, Alt diff: 0.1m
...
```

## Safety Features

1. **Connection Monitoring** - Verifies drone connection before each mission
2. **Timeout Protection** - 2-minute timeout per waypoint
3. **Waypoint Acceptance Radius** - 2m acceptance before moving to next point
4. **Checkpoint Saving** - Automatic return-to-home capability
5. **Failsafe Abort** - Can be interrupted with Ctrl+C (lands drone)
6. **Validation** - Checks separation distance against polygon size

## GPS Coordinate Conversion

The system uses equirectangular approximation for local-to-GPS conversion:

```
x (meters) = dlon * m_per_deg_lon
y (meters) = dlat * m_per_deg_lat

lat = lat0 + (y / m_per_deg_lat)
lon = lon0 + (x / m_per_deg_lon)
```

This is accurate to within 0.1% for small survey areas (<5 km²).

## Troubleshooting

### "Drone not ready"
- Check serial connection to flight controller
- Verify SYSTEM_ADDRESS is correct
- Check if ArduPilot/PX4 is running

### "Could not generate survey paths"
- Separation distance is too large for polygon
- Try smaller separation (e.g., 10m instead of 15m)

### Drone doesn't move
- Verify offboard mode is active
- Check MAVLink protocol version
- Confirm GPS lock before mission

### Waypoints skipped
- Check waypoint spacing (default 5m)
- Increase WAYPOINT_RADIUS if terrain is rough
- Reduce cruise speed if drone struggles

## Integration with sarx.py

`kml_path.py` can be integrated with `sarx.py` for:
- **Integrated detection + navigation** - Fly survey path while running YOLO detection
- **Checkpoint management** - Use same checkpoint system
- **Camera streaming** - Display progress on both cameras

Example integration:
```python
from kml_path import DronePathController
from sarx import main as sarx_main

# Run survey path navigation
drone = DronePathController()
drone.navigate_waypoints(waypoints)

# Switch to human detection mode
sarx_main()
```

## File Dependencies

```
kml_path.py (main program)
├── custom_survey.py (survey path algorithms)
├── mission.plan (polygon definition)
└── Shapely/NumPy (geometry and math)
```

Ensure all files are in the same directory or in Python path.

## Performance Metrics

For example mission (15m separation, ~60m² polygon):
- **Waypoint generation**: < 1 second
- **Path length**: ~380 meters total
- **Estimated flight time**: 5-8 minutes at 8 m/s
- **Waypoint count**: 75-80 points
- **Memory usage**: < 50 MB

## Future Enhancements

- [ ] Real-time path adjustment based on GPS drift
- [ ] Wind estimation and compensation
- [ ] Automatic obstacle avoidance
- [ ] Integration with camera triggers for aerial imaging
- [ ] Waypoint optimization using genetic algorithms
- [ ] Multi-drone coordination support

## License

Same as parent SARX project

## Author Notes

This system bridges survey planning and autonomous flight execution, providing
a complete turnkey solution for autonomous area coverage missions.
