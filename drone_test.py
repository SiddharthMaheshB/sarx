import asyncio
import time
import cv2
import numpy as np
from threading import Thread

from picamera2 import Picamera2

from mavsdk import System
from mavsdk.offboard import ActuatorControl


SYSTEM_ADDRESS = "serial:///dev/ttyACM0:115200"

CAMERA_INDEX = 0           # 0 = bottom, 1 = front (adjust)
CAM_SIZE = (640, 480)

MOTOR_TEST_POWER = 0.6   # VERY LOW (0.1-0.2 recommended)
MOTOR_TEST_TIME = 1.0      # seconds per motor




class CameraViewer:
    def __init__(self):
        self.running = True
        self.cam = Picamera2(CAMERA_INDEX)

        cfg = self.cam.create_preview_configuration(
            main={"format": "BGR888", "size": CAM_SIZE}
        )
        self.cam.configure(cfg)
        self.cam.start()

    def run(self):
        last = time.time()
        fps = 0.0

        while self.running:
            frame = self.cam.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            now = time.time()
            dt = now - last
            last = now
            fps = 0.9 * fps + 0.1 * (1.0 / dt if dt > 0 else 0)

            cv2.putText(
                frame,
                f"FPS: {fps:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.imshow("Camera Test", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.running = False
                break

        self.cam.stop()
        cv2.destroyAllWindows()


# ---------------- MOTOR TEST ----------------
async def motor_test():
    drone = System()
    print("[DRONE] Connecting...")
    await drone.connect(system_address=SYSTEM_ADDRESS)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[DRONE] Connected")
            break

    print("[DRONE] Arming (NO TAKEOFF)")
    await drone.action.arm()

    print("[DRONE] Starting offboard actuator control")
    await drone.offboard.start()

    # 4 motors for quad
    for motor in range(1):
        print(f"[TEST] Spinning motor {motor + 1}")

        controls = [0.0] * 8
        controls[motor] = MOTOR_TEST_POWER

        await drone.offboard.set_actuator_control(
            ActuatorControl(group=0, controls=controls)
        )

        await asyncio.sleep(MOTOR_TEST_TIME)

        # Stop all motors
        await drone.offboard.set_actuator_control(
            ActuatorControl(group=0, controls=[0.0] * 8)
        )

        await asyncio.sleep(1.0)

    print("[TEST] Motor test complete")

    print("[DRONE] Disarming")
    await drone.action.disarm()

    await drone.offboard.stop()


if __name__ == "__main__":
    cam_viewer = CameraViewer()
    cam_thread = Thread(target=cam_viewer.run, daemon=True)
    cam_thread.start()

    try:
        asyncio.run(motor_test())
    except KeyboardInterrupt:
        print("Interrupted")

    cam_viewer.running = False
    cam_thread.join()
