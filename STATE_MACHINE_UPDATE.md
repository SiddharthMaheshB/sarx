# State Machine Implementation Update

## Overview
Comprehensive state machine for autonomous drone delivery system with altitude tracking, timeouts, and failsafes.

## Complete Workflow

### 1. **SEARCHING State**
- **Objective:** Scan for person in front camera
- **Trigger Condition:** Person detected in front camera with area > 0.15 (15% of image)
- **Actions:**
  - Logs search progress every 5 seconds
  - Saves checkpoint at 10m altitude when detection occurs
  - Transitions to APPROACHING
- **Timeout:** 60 seconds (resets search if no detection)
- **Failsafe:** Auto-reset after timeout

---

### 2. **APPROACHING State**
- **Objective:** Move toward detected person until visible in bottom camera
- **Trigger Conditions:**
  - Person visible in front camera
  - Person transitions to bottom camera (area > 0.05)
- **Actions:**
  - Yaw-first centering: Rotates to face person (if offset > 0.1)
  - Forward movement: Advances toward person at 0.8 m/s
  - Logs movement every 2 seconds
  - Transitions to CENTERING when bottom camera detects person
- **Timeout:** 20 seconds (returns to SEARCHING if no bottom detection)
- **Failsafes:**
  - Lost target recovery: Returns to SEARCHING after 10 seconds without detection
  - Timeout abort: Returns to SEARCHING after 20 seconds

---

### 3. **CENTERING State**
- **Objective:** Center person in bottom camera and prepare for delivery
- **Trigger Conditions:**
  - Person visible in bottom camera
  - Horizontal AND vertical offset < 0.10 (centered)
  - Area > 0.4 (within proper distance)
- **Actions:**
  - Proportional control: Adjusts position based on center offset
  - Auto-descend: Descends when centered but area < 0.4 (get closer)
  - Logs adjustments every 2 seconds
  - Transitions to DESCENDING when fully centered and at proper distance
- **Timeout:** 15 seconds (returns to APPROACHING if centering takes too long)
- **Failsafes:**
  - Lost target recovery: Ascends to regain bottom view (-0.3 m/s)
  - Timeout abort: Returns to APPROACHING after 7.5 seconds without target

---

### 4. **DESCENDING State**
- **Objective:** Lower drone to 20 feet delivery altitude
- **Descent Profile:**
  - Initial altitude: 10m (checkpoint altitude)
  - Target altitude: 20 feet (6.1m)
  - Descent rate: 1.0 m/s
  - Duration: ~3.9 seconds
- **Actions:**
  - Maintains constant descent rate
  - Tracks altitude in real-time
  - Logs altitude and time remaining every second
  - Transitions to DROPPING when reaching delivery altitude
- **Safety:**
  - Altitude floor: Never descends below DELIVERY_ALTITUDE_M
  - Continuous monitoring: Tracks current_altitude throughout

---

### 5. **DROPPING State**
- **Objective:** Release package at delivery location
- **Duration:** 2 seconds
- **Actions:**
  - Simulates payload release
  - Logs drop progress every 0.5 seconds
  - Transitions to RETURNING after DROP_TIME expires
- **Safety:**
  - Fixed duration: No early transitions
  - Altitude: Maintains delivery altitude (20 ft)

---

### 6. **RETURNING State**
- **Objective:** Return to checkpoint altitude
- **Altitude Profile:**
  - Start: 20 feet (6.1m)
  - Target: 10m (checkpoint altitude)
  - Return rate: 1.0 m/s
  - Duration: ~3.9 seconds
- **Actions:**
  - Ascending/descending based on altitude difference
  - Tracks altitude in real-time
  - Logs return progress every second
  - Completes mission and returns to SEARCHING
- **Safety:**
  - Altitude correction: Moves up if below checkpoint, down if above
  - Continuous tracking: Ensures accurate altitude restoration

---

## Configuration Constants

```python
# Altitude Parameters
DELIVERY_ALTITUDE_FEET = 20.0      # 20 feet delivery target
DELIVERY_ALTITUDE_M = 6.1          # ~6.1 meters
INITIAL_ALTITUDE_M = 0.0           # Will be set at checkpoint

# State Timeouts
SEARCHING_TIMEOUT = 60.0           # Max 60 seconds in SEARCHING
APPROACHING_TIMEOUT = 20.0         # Max 20 seconds in APPROACHING
CENTERING_TIMEOUT = 15.0           # Max 15 seconds in CENTERING

# Movement Parameters
APPROACH_SPEED = 0.8               # m/s forward movement
YAW_RATE = 30.0                    # deg/s rotation
DESCENT_RATE = 1.0                 # m/s descent/return speed

# Drop & Timing
DROP_TIME = 2.0                    # 2 seconds package release
RETURN_TIME = 5.0                  # 5 seconds return simulation

# Detection Thresholds
PERSON_AREA_THRESHOLD_FRONT = 0.15 # 15% of front image
PERSON_AREA_THRESHOLD_BOTTOM = 0.4 # 40% of bottom image
```

---

## Enhanced SimulatedDroneController

### New Attributes
- `checkpoint_altitude`: Altitude saved during detection (10m)
- `current_altitude`: Real-time altitude tracking throughout mission

### Enhanced Methods

#### `save_checkpoint(altitude=10.0)`
- Saves GPS position and altitude
- Prints altitude in both meters and feet
- Sets `current_altitude` to checkpoint value

#### `move_with_yaw(..., down=0.0, ...)`
- Updates `current_altitude` during vertical movement
- Enforces altitude floor (never below DELIVERY_ALTITUDE_M)
- Displays altitude info during descent/ascent

#### `descend_to_delivery_height()`
- Sets `current_altitude = DELIVERY_ALTITUDE_M`
- Prints delivery altitude confirmation

#### `return_to_checkpoint()`
- Restores `current_altitude = checkpoint_altitude`
- Prints return-to-checkpoint message with target altitude

---

## Failsafe Summary

| State | Timeout | Recovery |
|-------|---------|----------|
| SEARCHING | 60s | Reset search |
| APPROACHING | 20s | Return to SEARCHING |
| CENTERING | 15s | Return to APPROACHING |
| DESCENDING | N/A | Continuous monitoring |
| DROPPING | 2s | Fixed duration |
| RETURNING | N/A | Altitude-based completion |

---

## Logging Features

### Real-time Status Updates
- Every 1-5 seconds depending on state
- Shows altitude in both m and ft
- Displays time remaining and progress

### State Transitions
- `print_state_change()` called for each transition
- Logs reason for transition
- Shows elapsed time in previous state

### Detection Events
- Area ratio and position offsets logged
- Threshold comparisons printed
- Waypoint information displayed

---

## Testing Recommendations

1. **Simulation Mode:** Run with simulated drone controller (current setup)
2. **Camera Testing:** Verify dual-camera detection during each state
3. **Timeout Validation:** Trigger timeouts to verify recovery behavior
4. **Altitude Tracking:** Confirm altitude values displayed correctly
5. **State Transitions:** Validate all 6 state transitions occur at proper conditions

---

## Example Console Output

```
🔍 [SEARCHING] Scanning for target... (0s elapsed)
🔍 [SEARCHING] Scanning for target... (5s elapsed)

🎯 [DETECTION] Human detected in FRONT camera!
   Area ratio: 0.182 (threshold: 0.150)
   Position offset: (0.05, -0.02)

📍 [CHECKPOINT] Position saved!
   Altitude: 10.00 m (32.81 ft)

[STATE] SEARCHING → APPROACHING (elapsed: 7.2s)

🔄 [APPROACHING] Yawing to face person (offset: 0.05)
➡️  [APPROACHING] Moving forward toward person (area: 0.195)

👁️  [TRANSITION] Human now visible in BOTTOM camera!
   Bottom area: 0.073 | Front area: 0.198
   Switching to CENTERING mode

🎯 [CENTERING] Adjusting position (x_off: 0.08, y_off: -0.05, area: 0.092)
📐 [CENTERING] Centered, descending to target size...

✅ [CENTERED] Person centered in bottom camera!
   Area: 0.412 (threshold: 0.400)
   Position offset: (0.02, 0.01)
   Time to center: 6.3s
   Switching to DESCENDING mode

⬇️  [DESCENDING] Lowering to delivery height
   Current: 10.00m (32.81ft) → Target: 6.10m (20.00ft)
   Time remaining: 3.9s

✅ [READY] Delivery altitude reached (20 ft)

📦 [DROPPING] Releasing payload... (1.5s remaining)

✅ [SUCCESS] Payload dropped successfully!
   Drop altitude: 20 ft (6.10m)

🔙 [RETURNING] Moving back to checkpoint
   Current: 6.10m (20.00ft) → Checkpoint: 10.00m (32.81ft)
   Time remaining: 3.9s

✅ [COMPLETE] Delivery mission complete!
   Final altitude: 10.00m (32.81ft)
   Resuming search for next target...
```

---

## Next Steps

1. **Real Drone Integration:** Replace `SimulatedDroneController` with actual drone API calls
2. **GPS Position Tracking:** Add lat/lon to checkpoint and return navigation
3. **Sensor Feedback:** Integrate altitude sensor readings for real-time validation
4. **Emergency Landing:** Add failsafe for lost GPS/detection
5. **Performance Optimization:** Reduce logging frequency if needed for production

---

