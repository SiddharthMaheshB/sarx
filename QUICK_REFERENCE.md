# SARX Quick Reference Guide

## System Overview

**SARX** = Autonomous Drone Delivery System
- **Components:** Drone + Raspberry Pi + Cameras + Servo
- **Purpose:** Detect human, navigate, and drop package
- **Status:** Production Ready (v2.0)

---

## Files & Locations

| File | Purpose | Status |
|------|---------|--------|
| `sarx.py` | **Production code** (with drone) | ✅ Updated |
| `sarx_camera_test.py` | Test/simulation code | ✅ Reference |
| `SARX_ARCHITECTURE.md` | Full documentation | ✅ Created |
| `STATE_MACHINE_UPDATE.md` | State machine details | ✅ Created |
| `SARX_IMPLEMENTATION_SUMMARY.md` | This implementation | ✅ Created |

---

## 6-State Machine

```
SEARCHING
   ↓ person detected (area > 15%)
APPROACHING
   ↓ person in bottom camera
CENTERING
   ↓ centered + large enough (area > 40%)
DESCENDING
   ↓ reached 20 feet
DROPPING
   ↓ drop complete (2s)
RETURNING
   ↓ checkpoint altitude reached
SEARCHING (repeat)
```

---

## Key Configuration

### Timeouts
```python
SEARCHING_TIMEOUT = 60.0    # Reset search
APPROACHING_TIMEOUT = 20.0  # Return to search
CENTERING_TIMEOUT = 15.0    # Return to approach
```

### Altitudes
```python
TAKEOFF_ALT = -15.24   # 50 feet (NED coordinate)
DELIVERY_ALT = -6      # 20 feet (NED coordinate)
```

### Detection Thresholds
```python
PERSON_AREA_THRESHOLD_FRONT = 0.15   # 15% to start approach
PERSON_AREA_THRESHOLD_BOTTOM = 0.4   # 40% to start descend
```

---

## State Logic Summary

### SEARCHING (Timeout: 60s)
- **Wait for:** Person in front camera (area > 15%)
- **Do:** Save checkpoint (GPS + altitude)
- **Go to:** APPROACHING

### APPROACHING (Timeout: 20s)
- **Look for:** Bottom camera detection (priority)
- **If bottom found:** Go to CENTERING immediately
- **Else if front found:** Yaw to face, move forward
- **Else:** Timeout abort back to SEARCHING

### CENTERING (Timeout: 15s)
- **Goal:** Center person in bottom camera
- **If centered + large (area ≥ 40%):** Go to DESCENDING
- **If centered but small:** Descend slowly (get closer)
- **If not centered:** Apply proportional control to move
- **If lost target:** Ascend to regain view

### DESCENDING (Altitude-based)
- **Goal:** Reach 20 feet altitude
- **Rate:** 1.0 m/s descent
- **Time:** ~3.9 seconds (from 10m to 6.1m)
- **Monitor:** Real-time altitude tracking
- **Go to:** DROPPING

### DROPPING (Duration: 2s)
- **Action:** Activate servo (release package)
- **Hold:** 1.0 second
- **Reset:** Servo returns to closed position
- **Go to:** RETURNING

### RETURNING (Altitude-based)
- **Goal:** Return to checkpoint altitude (10m)
- **Rate:** 1.0 m/s ascent
- **Time:** ~3.9 seconds
- **Direction:** Adaptive (up/down based on altitude difference)
- **Go to:** SEARCHING (mission repeat)

---

## Failsafes

### Always Active
```
if drone NOT connected:
    Land immediately
```

### Per State
| State | Trigger | Action |
|-------|---------|--------|
| SEARCHING | > 60s no detection | Reset |
| APPROACHING | > 20s in state | Back to SEARCHING |
| APPROACHING | > 10s lost target | Back to SEARCHING |
| CENTERING | > 15s in state | Back to APPROACHING |
| CENTERING | > 7.5s lost target | Ascend, then abort |
| DESCENDING | Altitude < 20ft | Enforce floor |
| RETURNING | Altitude diff | Adapt direction |

---

## Movement Commands

### Body Frame Velocity
```python
drone.move_with_yaw(
    forward=0.8,      # m/s (0 = stop)
    right=0.5,        # m/s (positive = right)
    down=0.3,         # m/s (positive = down/descend)
    yaw_rate=30.0,    # deg/s (positive = clockwise)
    duration=0.3      # seconds
)
```

### Proportional Control
```python
# Center offset: -1.0 to +1.0
forward_vel = -cy_bottom * 0.5
right_vel = cx_bottom * 0.5

# Example: Person at (0.2, -0.1) offset
# → Move right 0.1 m/s, forward 0.05 m/s
```

---

## Altitude Tracking

### Real-time Monitoring
```python
current_alt = drone.get_altitude()  # Returns meters (positive = up)
```

### Checkpoint Storage
```python
# Saved when person detected
checkpoint_altitude_m = 10.0  # Always saved at ~10m

# Used for return
if current_alt < checkpoint_alt:
    ascend()
else:
    descend()
```

### NED Coordinate System
```python
# Drone uses NED (North-East-Down)
# Positive = down, Negative = up
NED_up_10m = -10.0    # Stored in drone.current_altitude
meters_up_10m = 10.0  # Stored in drone.altitude_m
```

---

## Servo Drop Mechanism

### Activation
```python
drop_payload(servo)  # Called during DROPPING state

# What happens:
servo.min()           # Open grip → release package
time.sleep(1.0)       # Hold open
servo.max()           # Close grip → reset
```

### Configuration
```python
SERVO_PIN = 18           # GPIO pin
DROP_HOLD_SECONDS = 1.0  # Hold duration
RESET_SERVO_AFTER_DROP = True  # Auto-reset
```

---

## Logging Output Examples

### Detection
```
🎯 [DETECTION] Human detected in FRONT camera!
   Area ratio: 0.182 (threshold: 0.150)
   Position offset: (0.05, -0.02)
```

### Checkpoint
```
📍 [CHECKPOINT] Position saved!
   Latitude: 37.123456
   Longitude: -122.654321
   Altitude: 10.00 m (32.81 ft)
```

### State Transition
```
============================================================
🔄 STATE TRANSITION: SEARCHING → APPROACHING
============================================================
```

### Centering
```
🎯 [CENTERING] Adjusting position
   x_off: 0.08, y_off: -0.05, area: 0.092
```

### Descent
```
⬇️  [DESCENDING] Lowering to delivery height
   Current: 10.00m → Target: 6.10m
   Time remaining: 3.9s
```

### Delivery Complete
```
✅ [SUCCESS] Payload dropped successfully!
   Drop altitude: 20 ft (6.10m)

✅ [COMPLETE] Delivery mission complete!
   Final altitude: 10.00m
   Resuming search for next target...
```

---

## Commands

### Run Production System
```bash
python sarx.py
```
(Connects to drone, starts deliveries)

### Run Test/Simulation
```bash
python sarx_camera_test.py
```
(No drone, prints commands instead)

### Exit
```
Press 'q' during execution
→ Lands drone and exits cleanly
```

---

## Troubleshooting Quick Fixes

### Drone won't connect
```bash
# Check serial port
ls /dev/ttyACM*
# Verify: SYSTEM_ADDRESS = "serial:///dev/ttyACM0:115200"
```

### Detection not working
```bash
# Run test version to debug
python sarx_camera_test.py
# Adjust thresholds if needed
PERSON_AREA_THRESHOLD_FRONT = 0.15  # Lower = more sensitive
```

### Servo not activating
```bash
# Test manually
python -c "from gpiozero import Servo; s = Servo(18); s.min(); print('Open')"
# Verify GPIO pin and power supply
```

### Timeouts triggering too often
```bash
# Increase timeouts in config:
SEARCHING_TIMEOUT = 120.0  # Instead of 60
APPROACHING_TIMEOUT = 30.0  # Instead of 20
```

### Altitude not tracking
```bash
# Check telemetry stream:
# Should print altitude every 100ms
# Verify drone sends position data
```

---

## Performance Tips

1. **Faster detection:** Use sarx_camera_test.py (PyTorch optimized)
2. **Better centering:** Adjust proportional control gains (0.5 multiplier)
3. **Smooth descent:** Increase descent rate if needed (currently 1.0 m/s)
4. **Reliable servo:** Test drop mechanism before flights

---

## Checklists

### Pre-Flight
- [ ] Drone battery: 100%
- [ ] Drone GPS: Lock acquired
- [ ] Cameras: Focus adjusted, clear view
- [ ] Servo: Test activation works
- [ ] Serial port: Connected and verified
- [ ] Model file: Available
- [ ] Airspace: Clear, geofence active

### During Flight
- [ ] State transitions: Observe in console
- [ ] Altitude tracking: Verify in real-time
- [ ] Detection status: Check bounding boxes
- [ ] FPS: > 10 FPS (performance OK)
- [ ] Failsafe messages: None should appear

### Post-Flight
- [ ] Servo reset: In closed position
- [ ] Battery: Safe to store
- [ ] Cameras: No damage
- [ ] Logs: Review for issues

---

## API Reference

### DroneController Methods
```python
drone.save_checkpoint()              # Save position + altitude
drone.get_altitude()                 # Get current altitude (m)
drone.is_ready()                     # Check connection
drone.move_forward(speed, duration)  # Forward movement
drone.move_with_yaw(f, r, d, y, dur) # Combined movement
drone.yaw_towards(rate, duration)    # Pure yaw
drone.descend_to_delivery_height()   # Descend command
drone.return_to_checkpoint()         # Return command
drone.stop_and_land()                # Emergency land
```

### Utility Functions
```python
get_person_info(result, model, w, h)     # Extract detection info
draw_results(img, result, model)         # Draw bounding boxes
print_state_change(old, new)             # Print transition
drop_payload(servo)                      # Release package
```

---

## Next Steps

1. **Review** SARX_ARCHITECTURE.md for full details
2. **Test** with sarx_camera_test.py first
3. **Configure** sarx.py for your drone setup
4. **Verify** all pre-flight checks
5. **Deploy** for autonomous delivery operations

---

**Version:** 2.0 Production Ready
**Last Updated:** January 9, 2026
**Status:** ✅ Ready for Deployment

