#!/usr/bin/env python3
"""
Servo control module for SARX system.
Handles payload drop via pymavlink servo commands.
"""

import time
from pymavlink import mavutil

from config import (
    SERVO_NUMBERS, SERVO_CLOSE_PWM, SERVO_OPEN_PWM, DROP_HOLD_SECONDS
)

# Global servo state
current_servo_index = 0  # Tracks which servo to use next (0-4)


def init_servo():
    """
    Initialize servo connection via pymavlink.
    
    Returns:
        mavutil connection object or None on failure
    """
    global current_servo_index
    
    try:
        master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        master.wait_heartbeat()
        print("[SERVO] Connected to Pixhawk for servo control")
        
        # Initialize all servos to closed position
        for servo_num in SERVO_NUMBERS:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                servo_num,
                SERVO_CLOSE_PWM,
                0, 0, 0, 0, 0
            )
            time.sleep(0.1)
        
        print(f"[SERVO] Initialized {len(SERVO_NUMBERS)} servos (closed)")
        current_servo_index = 0  # Reset counter
        return master
    except Exception as e:
        print(f"[SERVO] Initialization failed: {e}")
        return None


def drop_payload(master):
    """
    Drop payload using servo with counter tracking.
    
    Args:
        master: pymavlink connection object
        
    Returns:
        bool: True if drop successful, False otherwise
    """
    global current_servo_index
    
    if master is None:
        print("[PAYLOAD] ⚠️  Drop called but servo not available")
        return False
    
    if current_servo_index >= len(SERVO_NUMBERS):
        print("[PAYLOAD] ⚠️  All servos have been used (5/5)")
        return False
    
    try:
        servo_num = SERVO_NUMBERS[current_servo_index]
        print("\n" + "="*60)
        print(f"📦 [PAYLOAD] Releasing package from servo {servo_num} ({current_servo_index + 1}/5)...")
        
        # Open servo to drop payload
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_num,
            SERVO_OPEN_PWM,  # Open position (1400)
            0, 0, 0, 0, 0
        )
        
        time.sleep(DROP_HOLD_SECONDS)
        print(f"✅ [PAYLOAD] Servo {servo_num} opened and remains open")
        print("="*60)
        
        # Increment counter for next human
        current_servo_index += 1
        if current_servo_index < len(SERVO_NUMBERS):
            print(f"[SERVO] Next drop will use servo {SERVO_NUMBERS[current_servo_index]}")
        else:
            print("[SERVO] All servos used - no more drops available")
        
        return True
    except Exception as e:
        print(f"❌ [PAYLOAD] Drop error: {e}")
        print("="*60)
        return False


def get_remaining_drops():
    """
    Get number of remaining drops available.
    
    Returns:
        int: Number of drops remaining
    """
    return max(0, len(SERVO_NUMBERS) - current_servo_index)


def reset_servo_counter():
    """Reset servo counter to 0 (for testing)"""
    global current_servo_index
    current_servo_index = 0
    print(f"[SERVO] Counter reset to 0")


def main():
    """Test servo control"""
    print("="*60)
    print("SERVO CONTROL TEST")
    print("="*60)
    
    print(f"\n[CONFIG]")
    print(f"  Servo Numbers: {SERVO_NUMBERS}")
    print(f"  Close PWM: {SERVO_CLOSE_PWM}")
    print(f"  Open PWM: {SERVO_OPEN_PWM}")
    print(f"  Drop Hold Time: {DROP_HOLD_SECONDS}s")
    
    print("\n[TEST] Initializing servo connection...")
    master = init_servo()
    
    if master is None:
        print("✗ Failed to connect to servo")
        print("  Make sure Pixhawk is connected at /dev/ttyACM0")
        return
    
    print("✓ Servo initialized")
    print(f"  Remaining drops: {get_remaining_drops()}")
    
    # Test drop (WARNING: This will actually trigger servo!)
    print("\n[WARNING] This test will trigger servo drop!")
    print("  Press Ctrl+C to cancel, or wait 3 seconds to proceed...")
    try:
        time.sleep(3)
        
        print("\n[TEST] Triggering test drop...")
        if drop_payload(master):
            print("✓ Drop successful")
            print(f"  Remaining drops: {get_remaining_drops()}")
        else:
            print("✗ Drop failed")
        
        # Reset for next test
        print("\n[TEST] Resetting counter...")
        reset_servo_counter()
        print(f"  Remaining drops: {get_remaining_drops()}")
        
    except KeyboardInterrupt:
        print("\n✓ Test cancelled")
    
    print("\n" + "="*60)


if __name__ == "__main__":
    main()
