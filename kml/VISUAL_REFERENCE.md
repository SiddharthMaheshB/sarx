# KML PATH - VISUAL REFERENCE GUIDE

## System Overview Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    MISSION PLANNING                         │
│  ┌──────────────┐         ┌──────────────┐                 │
│  │ mission.plan │────────▶│  Polygon &   │                 │
│  │  (geoFence)  │         │   GPS Ref    │                 │
│  └──────────────┘         └──────┬───────┘                 │
│                                  │                          │
│  ┌──────────────────────────────▼─────────────────────┐   │
│  │      Survey Path Generation (completesurvey)       │   │
│  │  ├─ Split into 2 equal areas                       │   │
│  │  ├─ Find optimal angles                            │   │
│  │  └─ Generate survey paths                          │   │
│  └──────────────────────────────┬─────────────────────┘   │
│                                  │                          │
│  ┌──────────────────────────────▼─────────────────────┐   │
│  │    Waypoint Generation (kml_path.py)              │   │
│  │  ├─ Interpolate path points                        │   │
│  │  ├─ Convert meters → GPS                           │   │
│  │  └─ Create waypoint list                           │   │
│  └──────────────────────────────┬─────────────────────┘   │
│                                  │                          │
└──────────────────────────────────┼────────────────────────┘
                                   │
┌──────────────────────────────────▼────────────────────────┐
│                  AUTONOMOUS FLIGHT                        │
│  ┌────────────────────────────────────────────────────┐  │
│  │  DronePathController                              │  │
│  │  ├─ Connect to drone (MAVLink)                    │  │
│  │  ├─ Arm & Takeoff                                 │  │
│  │  ├─ Navigate waypoints                            │  │
│  │  │  ├─ goto_location()                            │  │
│  │  │  ├─ Monitor distance                           │  │
│  │  │  └─ Confirm arrival                            │  │
│  │  ├─ Return to checkpoint                          │  │
│  │  └─ Land safely                                   │  │
│  └────────────────────────────────────────────────────┘  │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

---

## Data Flow Architecture

```
INPUT LAYER
  │
  ├─ mission.plan (JSON)
  │  └─ geoFence.polygons[].polygon → [(lat, lon), ...]
  │
  ├─ custom_survey.py
  │  └─ Polygon boundary + GPS reference
  │
  └─ completesurvey.py
     └─ Survey path optimization algorithms

          ↓↓↓

PROCESSING LAYER
  │
  ├─ WaypointGenerator
  │  ├─ Input: Shapely LineString path (meters)
  │  ├─ Process: Interpolate & convert coordinates
  │  └─ Output: [(lat, lon, alt), ...] GPS waypoints
  │
  └─ DronePathController
     ├─ Input: Waypoint list
     ├─ Process: Navigate via MAVLink commands
     └─ Output: Flight telemetry & logs

          ↓↓↓

OUTPUT LAYER
  │
  ├─ Waypoints (JSON/CSV files)
  ├─ Flight path (telemetry)
  ├─ Mission logs (status messages)
  └─ Drone state (position, altitude, battery)
```

---

## Mission Execution Timeline

```
TIME →

 0s   ├─ Start program
      │  └─ Load mission.plan
      │
 2s   ├─ Parse survey configuration
      │  └─ Validate parameters
      │
 5s   ├─ Generate survey paths
      │  ├─ Split polygon
      │  ├─ Find optimal angles
      │  └─ Create paths
      │
 8s   ├─ Generate waypoints
      │  ├─ Interpolate points
      │  └─ Convert coordinates
      │
12s   ├─ Initialize drone
      │  ├─ Connect (serial)
      │  ├─ Arm
      │  └─ Takeoff (8s)
      │
20s   ├─ SURVEY PHASE
      │  ├─ Save checkpoint
      │  ├─ Navigate waypoint 1
      │  ├─ Navigate waypoint 2
      │  │  ...
      │  └─ Navigate waypoint N (45-50s)
      │
65s   ├─ RETURN PHASE
      │  ├─ Goto checkpoint
      │  └─ Monitor arrival (10s)
      │
75s   ├─ LANDING PHASE
      │  ├─ Descend
      │  ├─ Touch down
      │  └─ Disarm (5s)
      │
80s   └─ MISSION COMPLETE
```

---

## Coordinate System Transformation

```
LOCAL METERS (Survey Space)
┌─────────────────────────────┐
│         (x, y)              │
│     ╱─────────────╲         │
│    │               │        │
│    │   Polygon     │ +Y     │
│    │   (survey     │ ▲      │
│    │    area)      │ │      │
│     ╲─────────────╱  │      │
│            ●────────→ +X    │
│      (0,0)                  │
│    [Centroid]               │
└─────────────────────────────┘
       Meters scale

         (CONVERSION)

      GPS COORDINATES
┌─────────────────────────────┐
│         (lat, lon)          │
│     ╱─────────────╲         │
│    │               │        │
│    │   Polygon     │ +lat   │
│    │   (GPS        │ ▲      │
│    │    coords)    │ │      │
│     ╲─────────────╱  │      │
│      (lat0,lon0)────→ +lon  │
│         ●                   │
│    [Reference]              │
└─────────────────────────────┘
     Degrees scale

CONVERSION FORMULAS:
  x_m = dlon_deg × m_per_deg_lon
  y_m = dlat_deg × m_per_deg_lat
  
  lat = lat0 + (y_m / m_per_deg_lat)
  lon = lon0 + (x_m / m_per_deg_lon)
```

---

## Survey Path Generation Pattern

```
Original Polygon
┌──────────────────────────┐
│   ╱──────────────────╲   │
│  │                    │  │
│  │                    │  │
│  │      SEARCH        │  │
│  │        AREA        │  │
│  │                    │  │
│  │                    │  │
│   ╲──────────────────╱   │
└──────────────────────────┘

         ↓ SPLIT

Half 1 (Top)           Half 2 (Bottom)
┌────────────────────┐  ┌────────────────────┐
│ ──────────────────┐│  │┌──────────────────│
│ ─────────────────│ │  │ ─────────────────│ │
│ ────────────────│  │  │  ────────────────┐ │
│ ───────────────│   │  │   ───────────────┐ │
└────────────────────┘  └────────────────────┘
  (Angle: 45°)            (Angle: 135°)
  
         ↓ OPTIMIZE

Path 1: Zigzag      Path 2: Zigzag
└───────┘           └───────┐
    ┌───────┐           │
┌───┘       └───┐       └───────┐
│               │               │
└───┐       ┌───┘           ┌───┘
    └───────┘               │
                        ┌───┘
                        
          ↓ CONVERT
          
WAYPOINTS GPS LIST:
1. (17.522310, 78.367450, 15.24)
2. (17.522312, 78.367452, 15.24)
3. (17.522314, 78.367454, 15.24)
...
77. (17.522350, 78.367500, 15.24)
```

---

## Waypoint Navigation State Machine

```
                   ┌─────────────────┐
                   │  INITIALIZATION  │
                   └────────┬─────────┘
                            │
                   ┌────────▼──────────┐
                   │  CONNECT & SETUP  │
                   │  ├─ Arm drone     │
                   │  └─ Takeoff       │
                   └────────┬──────────┘
                            │
                   ┌────────▼──────────┐
              ┌────│ NAVIGATE         │◄────────┐
              │    │ WAYPOINTS        │         │
              │    │ ├─ Current WP    │         │
              │    │ ├─ Distance < 2m?│         │
              │    │ └─ Next WP       │────┐    │
              │    └──────┬──────────┘     │    │
              │           │                │    │
              │      More WPs?      ┌──────┘    │
              │           │         │           │
              │      ┌────▼────┐    │           │
              │      │  YES    │    │ Timeout?  │
              │      │ (LOOP)  │    │           │
              │      └────┬────┘    │      ┌────┘
              │           │         │      │
              │      ┌────▼────┐    │ ┌────▼────┐
              │      │   NO    │    │ │ ABORT   │
              │      │  (Exit) │    │ │ & LAND  │
              │      └────┬────┘    │ └─────────┘
              │           │         │
              │  ┌────────▼──────────┴──┐
              │  │  RETURN TO           │
              │  │  CHECKPOINT          │
              │  └────────┬─────────────┘
              │           │
              │  ┌────────▼──────────┐
              └──│  LAND & SHUTDOWN   │
                 └────────────────────┘
```

---

## Waypoint Arrival Detection

```
APPROACH PHASE:
  Drone:        GPS Target:        Distance:
    X            T                  ~50m
           ---▶
                                    
APPROACHING:
    X ●        T                    ~25m
         ─────▶
         
NEAR:
      X ●      T                    ~5m
      ──────▶
      
ARRIVAL (< 2m):
      X ●●     T                    ~1m
      ───▶
      ✓ CONFIRMED ARRIVAL
      
NEXT WAYPOINT:
      X         T●                  Moving to
                ──────▶             next target
```

---

## Battery & Flight Time Calculation

```
EXAMPLE: 380m survey path @ 8 m/s

TIME BREAKDOWN:

Preflight:
  ├─ Initialization     2 seconds
  ├─ Connect & arm      8 seconds
  └─ Takeoff            8 seconds
  Total: 18 seconds

Survey Flight:
  ├─ Path distance      380 meters
  ├─ Cruise speed       8 m/s
  └─ Duration           47.5 seconds
  (Plus 10-15% turns/hovering)
  Actual: ~52 seconds

Return:
  ├─ Distance           80 meters (approximate)
  ├─ Speed              8 m/s
  └─ Duration           10 seconds

Landing:
  ├─ Descent            5 seconds
  └─ Disarm             2 seconds
  Total: 7 seconds

─────────────────────────────────────
TOTAL MISSION TIME:  ~130 seconds (2+ minutes)
                     or ~8 minutes typical
                     (with margins & safety)

BATTERY CONSUMPTION:
  Flight time:  130 seconds ÷ 3600s/hour = 0.036 hours
  Power:        ~1200W typical
  Energy:       1200W × 0.036h = 43 Wh
  Battery size: 5000 mAh @ 22.2V = 111 Wh
  Usage:        43 ÷ 111 = 39% of battery
  Margin:       61% remaining (safe)
```

---

## Waypoint Format Reference

```
GENERATED WAYPOINT FORMAT:

(latitude_deg, longitude_deg, altitude_m)

Example:
  (17.522310, 78.367450, 15.24)
   └─ Latitude      └─ Longitude    └─ Altitude

GPS COORDINATE PRECISION:
  Latitude:  17.522310 degrees = ±0.0001° ≈ ±11 meters
  Longitude: 78.367450 degrees = ±0.0001° ≈ ±9 meters
  Altitude:  15.24 meters AGL (above ground level)

EXPORT FORMAT - JSON:
{
  "index": 0,
  "latitude": 17.522310,
  "longitude": 78.367450,
  "altitude_m": 15.24,
  "altitude_ft": 50.00
}

EXPORT FORMAT - CSV:
Index,Latitude,Longitude,Altitude (m),Altitude (ft)
0,17.522310,78.367450,15.24,50.00
1,17.522312,78.367452,15.24,50.00
...
```

---

## Configuration Quick Edit Reference

```
FILE: kml_path.py
SECTION: Lines 45-60 (Configuration)

PARAMETER            │ CHANGE TO... │ EFFECT
─────────────────────┼──────────────┼─────────────────────
TAKEOFF_ALT         │ -20.0        │ Higher survey altitude
                    │ -10.0        │ Lower survey altitude
────────────────────┼──────────────┼─────────────────────
SURVEY_CRUISE_SPEED │ 5.0          │ Slower, more stable
                    │ 12.0         │ Faster, less stable
────────────────────┼──────────────┼─────────────────────
WAYPOINT_RADIUS     │ 1.0          │ Tighter control
                    │ 4.0          │ Loose tolerance
────────────────────┼──────────────┼─────────────────────
DEFAULT_SEPARATION_ │ 10.0         │ 10m line spacing
M                   │ 20.0         │ 20m line spacing
────────────────────┼──────────────┼─────────────────────

RULE: Changes to DEFAULT values affect next run
      Changes to other values take effect immediately
```

---

## Error Recovery Flowchart

```
START MISSION
    │
    ├─ Can load mission.plan?
    │  └─ NO → ERROR: File not found
    │         ✗ EXIT
    │
    ├─ Can parse polygon?
    │  └─ NO → ERROR: Invalid geoFence
    │         ✗ EXIT
    │
    ├─ Is separation valid?
    │  └─ NO → PROMPT: "Use smaller value"
    │         ↓ USER ENTERS NEW VALUE
    │         ↓ RETRY
    │
    ├─ Can connect drone?
    │  └─ NO → ERROR: Serial connection failed
    │         ✗ LAND & EXIT
    │
    ├─ Can arm drone?
    │  └─ NO → ERROR: Battery low / geofence
    │         ✗ LAND & EXIT
    │
    ├─ Can takeoff?
    │  └─ NO → ERROR: GPS not locked
    │         ✗ LAND & EXIT
    │
    ├─ NAVIGATE WAYPOINTS
    │  ├─ Reach waypoint?
    │  │  └─ TIMEOUT (2 min) → ERROR: GPS drift?
    │  │                       ✗ RETURN & LAND
    │  │
    │  ├─ Lost GPS signal?
    │  │  └─ YES → ERROR: Signal lost
    │  │          ✗ RETURN & LAND (if possible)
    │  │
    │  └─ Battery critical?
    │     └─ YES → WARNING: Low battery
    │            ✗ RETURN & LAND
    │
    └─ COMPLETE MISSION
       ✓ SUCCESS
```

---

## Performance Comparison Matrix

```
SEPARATION DISTANCE vs FLIGHT TIME

Spacing │ # Lines │ Total Distance │ Flight Time │ Waypoints
────────┼─────────┼────────────────┼─────────────┼───────────
5m      │   20    │      520m      │   9 min     │   105
10m     │   10    │      300m      │   5 min     │   60
15m     │    6    │      200m      │   3 min     │   40
20m     │    5    │      180m      │   2 min     │   36
25m     │    4    │      150m      │   2 min     │   30

SMALLER spacing = finer coverage but longer flight time
LARGER spacing = faster mission but less detailed coverage
```

---

## Checklist Before Flight

```
PRE-FLIGHT CHECKLIST

System Setup:
  ☐ Computer running Python 3.8+
  ☐ Required packages installed
  ☐ kml_path.py in same directory as mission.plan
  ☐ mission.plan has valid geoFence

Drone Hardware:
  ☐ Battery charged (> 80%)
  ☐ Battery balanced
  ☐ Motors tested
  ☐ Propellers secure
  ☐ GPS antenna connected
  ☐ Flight controller powered

Environment:
  ☐ Sufficient space (minimum 100m × 100m)
  ☐ No obstacles in survey area
  ☐ Clear weather (< 10 knots wind)
  ☐ GPS lock confirmed (60+ satellites)
  ☐ Remote transmitter in range

Software:
  ☐ ArduPilot/PX4 running
  ☐ Serial connection tested
  ☐ MAVLink protocol active
  ☐ Geofence validated
  ☐ RC control responsive

Mission Setup:
  ☐ Separation distance selected
  ☐ Waypoints generated
  ☐ Flight path reviewed
  ☐ Estimated time calculated
  ☐ Battery sufficient for mission

Safety:
  ☐ Failsafe altitude set
  ☐ Return-to-home verified
  ☐ Spectators clear
  ☐ Mobile geofence active (if applicable)
  ☐ Video recording enabled

START MISSION ✓
```

---

**Visual Reference Guide Complete**
All diagrams updated for kml_path.py integration.
