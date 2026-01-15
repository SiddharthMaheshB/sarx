#!/usr/bin/env python3
from pymavlink import mavutil
import time
from datetime import datetime

# ---------------- USER CONFIG ---------------- #
MAVLINK_PORT = "/dev/ttyACM0"
BAUD = 115200

TRIGGER_RC_CHANNEL = 6
SWITCH_HIGH_THRESHOLD = 1600

SERVO_OUTPUTS = [9, 10, 11, 12, 13]   # AUX1..AUX5

LOCK_PWM = 900
RELEASE_PWM = 1400
PULSE_TIME_SEC = 1

LOG_FILE = "servo_release_log.txt"

# MML tune string (edit to taste). Keep short when testing.
TUNE_MML = "MFT220L8O6CECECECE"   # example tune; change if you want
# -------------------------------------------- #


def set_servo(master, servo_output, pwm):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_output,
        pwm,
        0, 0, 0, 0, 0
    )


def play_tune(master, mml):
    """
    Send PLAY_TUNE to autopilot. mml should be a short ASCII string.
    """
    try:
        # pymavlink may accept a bytes or string; use bytes for safety
        master.mav.play_tune_send(master.target_system, master.target_component, mml.encode('utf-8'))
        print(f"[TUNE] PLAY_TUNE sent: {mml}")
    except Exception as e:
        print("[WARN] Failed to send PLAY_TUNE:", e)


def request_message_interval(master, message_id, freq_hz):
    interval_us = int(1e6 / freq_hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0, 0, 0, 0, 0
    )


def get_rc_channel_value(msg, ch):
    field = f"chan{ch}_raw"
    return getattr(msg, field, None)


def append_to_log(line):
    with open(LOG_FILE, "a") as f:
        f.write(line + "\n")


def format_gps(gps):
    if gps["lat"] is None or gps["lon"] is None:
        return "GPS:NOT_READY"
    if gps["alt"] is None:
        return f"GPS:{gps['lat']:.7f},{gps['lon']:.7f}"
    else:
        return f"GPS:{gps['lat']:.7f},{gps['lon']:.7f},ALT:{gps['alt']:.2f}m"


def main():
    print("[INFO] Connecting MAVLink...")
    master = mavutil.mavlink_connection(MAVLINK_PORT, baud=BAUD)

    print("[INFO] Waiting heartbeat...")
    master.wait_heartbeat()
    print(f"[INFO] Heartbeat OK (sys={master.target_system}, comp={master.target_component})")

    # Request important streams (ArduPilot often requires explicit requests)
    print("[INFO] Requesting streams...")
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 10)         # 10 Hz
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5) # 5 Hz
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5)         # 5 Hz

    last_switch_state = 0
    servo_index = 0
    last_rc_val = None

    last_gps = {"lat": None, "lon": None, "alt": None}

    print("[INFO] Ready. Toggle RC switch to HIGH to release next servo.")
    print(f"[INFO] Logging to: {LOG_FILE}")
    append_to_log(f"\n--- Script start {datetime.now().isoformat()} ---")

    while True:
        msg = master.recv_match(blocking=True, timeout=2)
        if not msg:
            continue

        msg_type = msg.get_type()

        # ---- keep last GPS updated ----
        if msg_type == "GLOBAL_POSITION_INT":
            # lat/lon are int32 degrees*1e7, alt is mm
            if getattr(msg, "lat", 0) != 0 and getattr(msg, "lon", 0) != 0:
                last_gps["lat"] = msg.lat / 1e7
                last_gps["lon"] = msg.lon / 1e7
                last_gps["alt"] = msg.alt / 1000.0

        elif msg_type == "GPS_RAW_INT":
            if getattr(msg, "lat", 0) != 0 and getattr(msg, "lon", 0) != 0:
                last_gps["lat"] = msg.lat / 1e7
                last_gps["lon"] = msg.lon / 1e7
                if getattr(msg, "alt", 0) != 0:
                    last_gps["alt"] = msg.alt / 1000.0

        # ---- RC handling ----
        if msg_type != "RC_CHANNELS":
            continue

        rc_val = get_rc_channel_value(msg, TRIGGER_RC_CHANNEL)
        if rc_val is None or rc_val == 0:
            continue

        if rc_val != last_rc_val:
            print(f"[RC] CH{TRIGGER_RC_CHANNEL} PWM = {rc_val}")
            last_rc_val = rc_val

        switch_state = 1 if rc_val >= SWITCH_HIGH_THRESHOLD else 0

        # Rising edge LOW -> HIGH triggers release & tune
        if switch_state == 1 and last_switch_state == 0:
            now = datetime.now().isoformat(timespec="seconds")

            if servo_index >= len(SERVO_OUTPUTS):
                print("[DONE] All servos already released.")
                log_line = f"{now} | DONE | RC{TRIGGER_RC_CHANNEL}:{rc_val} | {format_gps(last_gps)}"
                append_to_log(log_line)

            else:
                servo_out = SERVO_OUTPUTS[servo_index]
                servo_index += 1

                print(f"[ACTION] Releasing servo output {servo_out} ({servo_index}/{len(SERVO_OUTPUTS)})")
                print(f"[GPS] {format_gps(last_gps)}")

                # send release pwm
                set_servo(master, servo_out, RELEASE_PWM)

                # play tune immediately after issuing release (non-blocking send)
                play_tune(master, TUNE_MML)

                # wait the pulse time (tune plays on autopilot buzzer)
                time.sleep(PULSE_TIME_SEC)

                # return to lock pwm
                set_servo(master, servo_out, LOCK_PWM)

                print(f"[OK] Servo {servo_out} released and tune played.")

                # log the event
                log_line = (
                    f"{now} | SERVO:{servo_out} | RC{TRIGGER_RC_CHANNEL}:{rc_val} | "
                    f"LOCK:{LOCK_PWM} RELEASE:{RELEASE_PWM} | {format_gps(last_gps)} | TUNE:{TUNE_MML}"
                )
                append_to_log(log_line)

        last_switch_state = switch_state
        time.sleep(0.01)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Exiting.")
