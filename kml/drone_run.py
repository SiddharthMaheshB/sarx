import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import custom_survey as cs

PLAN_FILE = "generated_polygons/polygon_16_sides.plan"

async def follow_survey_path(drone, waypoints):
    print("[DRONE] Arming")
    await drone.action.arm()

    print("[DRONE] Starting offboard mode")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard start failed: {e._result.result}")
        await drone.action.disarm()
        return
    
    for idx, (x, y) in enumerate(waypoints):
        print(f"[DRONE] Moving to waypoint {idx+1}: x={x:.2f}, y={y:.2f}")
        await drone.offboard.set_position_ned(PositionNedYaw(x, y, -2.0, 0))  # -2.0m altitude, 0 yaw
        await asyncio.sleep(3)  # Adjust time as needed for your drone's speed

    print("[DRONE] Stopping offboard mode")
    await drone.offboard.stop()
    print("[DRONE] Disarming")
    await drone.action.disarm()

def extract_waypoints(path):
    # path is a Shapely LineString
    return list(path.coords)

async def main():
    # Load polygon and survey path
    poly_m, _ = cs.load_polygon_from_plan_in_meters(PLAN_FILE)
    poly1_m, poly2_m, _ = cs.compute_equal_area_split(poly_m, angle_rad=0.0)
    # Use one of the survey paths, e.g., for half 1
    angle_half1, path1, _, _, _ = cs.find_best_angle_for_region(
        poly1_m, separation_m=10, to_point=(0, 0), angle_step_deg=1.0
    )
    waypoints = extract_waypoints(path1)

    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    await follow_survey_path(drone, waypoints)

if __name__ == "__main__":
    asyncio.run(main())