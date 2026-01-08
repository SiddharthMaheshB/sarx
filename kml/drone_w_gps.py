import asyncio
import math
import time
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import custom_survey as cs

PLAN_FILE = "generated_polygons/polygon_16_sides.plan"

WAYPOINT_THRESHOLD = 1.0          # meters
GPS_UPDATE_INTERVAL = 0.5         # seconds
WAYPOINT_TIMEOUT = 60.0           # seconds per waypoint
LANDING_TIMEOUT = 60.0            # seconds

CRUISE_ALTITUDE = -2.0             # meters (NED)
FIXED_YAW = 0.0                    # radians



def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


async def get_current_position(drone):
    async for pos in drone.telemetry.position_velocity_ned():
        return pos.position.north_m, pos.position.east_m, pos.position.down_m
    return None, None, None


def extract_waypoints(path):
    return list(path.coords)


# ---------------- FLIGHT LOGIC ----------------

async def follow_survey_path(drone, waypoints):

    print("[DRONE] Arming")
    await drone.action.arm()

    # --- Get current position to use as origin offset ---
    north0, east0, down0 = await get_current_position(drone)
    if north0 is None:
        print("[ERROR] Unable to read initial position")
        await drone.action.disarm()
        return

    print(f"[DRONE] Origin set at N={north0:.2f}, E={east0:.2f}")

    # --- Send initial setpoint BEFORE offboard start ---
    await drone.offboard.set_position_ned(
        PositionNedYaw(north0, east0, CRUISE_ALTITUDE, FIXED_YAW)
    )

    print("[DRONE] Starting offboard mode")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[ERROR] Offboard start failed: {e._result.result}")
        await drone.action.disarm()
        return

    # --- Waypoint loop ---
    for idx, (dx, dy) in enumerate(waypoints):
        target_n = north0 + dx
        target_e = east0 + dy

        print(f"[DRONE] Waypoint {idx+1}: N={target_n:.2f}, E={target_e:.2f}")

        await drone.offboard.set_position_ned(
            PositionNedYaw(target_n, target_e, CRUISE_ALTITUDE, FIXED_YAW)
        )

        start_time = time.time()

        while True:
            cn, ce, _ = await get_current_position(drone)
            if cn is None:
                await asyncio.sleep(GPS_UPDATE_INTERVAL)
                continue

            dist = calculate_distance(cn, ce, target_n, target_e)
            print(f"[DRONE] Distance: {dist:.2f} m")

            if dist <= WAYPOINT_THRESHOLD:
                print(f"[DRONE] Waypoint {idx+1} reached")
                break

            if time.time() - start_time > WAYPOINT_TIMEOUT:
                print(f"[WARN] Waypoint {idx+1} timeout — continuing")
                break

            await asyncio.sleep(GPS_UPDATE_INTERVAL)

    # ---------------- LANDING SEQUENCE ----------------

    print("[DRONE] Survey complete — stopping offboard")
    await drone.offboard.stop()

    print("[DRONE] Landing")
    await drone.action.land()

    landing_start = time.time()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("[DRONE] Landed successfully")
            break
        if time.time() - landing_start > LANDING_TIMEOUT:
            print("[WARN] Landing timeout")
            break
        await asyncio.sleep(1)

    print("[DRONE] Disarming")
    await drone.action.disarm()

    print("[DRONE] Mission complete — safe shutdown")


# ---------------- MAIN ----------------

async def main():

    poly_m, _ = cs.load_polygon_from_plan_in_meters(PLAN_FILE)
    poly1_m, _, _ = cs.compute_equal_area_split(poly_m, angle_rad=0.0)

    _, path1, _, _, _ = cs.find_best_angle_for_region(
        poly1_m,
        separation_m=10,
        to_point=(0, 0),
        angle_step_deg=1.0
    )

    waypoints = extract_waypoints(path1)

    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")

    await follow_survey_path(drone, waypoints)


if __name__ == "__main__":
    asyncio.run(main())
