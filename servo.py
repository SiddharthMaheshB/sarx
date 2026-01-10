from pymavlink import mavutil
import time

# Connect to Pixhawk
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()

print("Connected to Pixhawk")

def set_servo(servo_number, pwm):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_number,
        pwm,
        0, 0, 0, 0, 0
    )
time.sleep(1)
for i in range(9,14):
	set_servo(i, 900)
	time.sleep(2)
