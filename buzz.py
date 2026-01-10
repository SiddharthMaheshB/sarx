# save as play_tune_pymavlink.py
from pymavlink import mavutil
import time

# set the device & baud you actually use
SERIAL = "/dev/ttyACM0"      # change to your port (or /dev/ttyAMA0)
BAUD = 115200                 # change to your telemetry baud

# connect
master = mavutil.mavlink_connection(SERIAL, baud=BAUD)
print("Waiting for heartbeat from autopilot...")
master.wait_heartbeat(timeout=10)
print("Heartbeat received from system %u component %u" % (master.target_system, master.target_component))

# MML / tune string example (classic MFT / QBasic style used by many systems)
mml = "MFT220L8O6CECECECECECECECECECECECECECECECECECECECECECECECECECECECECECECECE"   # pick your tune; try small simple tune first

# encode as bytes and send PLAY_TUNE (v1)
try:
    master.mav.play_tune_send(master.target_system, master.target_component, mml.encode('utf-8'))
    print("PLAY_TUNE message sent:", mml)
except Exception as e:
    print("Error sending PLAY_TUNE:", e)

# keep program alive a short while so message gets through
time.sleep(1)
