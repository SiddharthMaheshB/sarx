import asyncio
from mavsdk import System

async def test_gps():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:115200")
    
    await drone.telemetry.set_rate_position(5)
    await drone.telemetry.set_rate_gps_info(1)
    # await drone.telemetry.set_rate_altitude(10)
    print("[TEST] Reading GPS position for 10 seconds...")
    count = 0
    async for health in drone.telemetry.health():
        print(f"GPS info: {health}")
        break
        await asyncio.sleep(1)

    async for position in drone.telemetry.position():
        print(f"GPS info: {position}")
        count += 1
        if count >= 10:
            break

    async for health in drone.telemetry.health():
        print(f"GPS info: {health}")
        if health.is_global_position_ok:
            break
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(test_gps())