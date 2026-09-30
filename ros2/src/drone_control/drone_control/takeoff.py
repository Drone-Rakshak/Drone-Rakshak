import asyncio
from mavsdk import System
from mavsdk.action import ActionError


async def run():
    drone = System()

    print("Connecting to PX4...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected!")
            break

    print("Waiting for vehicle to be ready...")

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("✅ Vehicle is ready!")
            break

    print("Arming...")

    try:
        await drone.action.arm()
        print("✅ Drone Armed Successfully!")

        print("Setting takeoff altitude...")
        await drone.action.set_takeoff_altitude(3.0)

        print("Taking off...")
        await drone.action.takeoff()

        print("Hovering...")
        await asyncio.sleep(5)

        print("Landing...")
        await drone.action.land()

        print("Waiting for landing...")
        await asyncio.sleep(10)

        print("Mission Complete!")
    except ActionError as e:
        print(f"❌ Arming Failed: {e}")
        return

    # Check armed status
    async for armed in drone.telemetry.armed():
        print(f"Armed Status: {armed}")
        break

    print("\nWaiting 5 seconds before exit...")
    await asyncio.sleep(5)


def main():
    asyncio.run(run())


if __name__ == "__main__":
    main()
