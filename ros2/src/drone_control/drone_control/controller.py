import asyncio

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError


class DroneController:

    def __init__(self):
        self.drone = System()

    async def connect(self):
        print("Connecting to PX4...")

        await self.drone.connect(
            system_address="udpin://0.0.0.0:14540"
        )

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("✅ Connected")
                break

    async def wait_until_ready(self):
        print("Waiting for GPS...")

        async for health in self.drone.telemetry.health():
            if (
                health.is_global_position_ok
                and health.is_home_position_ok
            ):
                print("✅ GPS Ready")
                break

    async def arm(self):
        print("Arming...")
        await self.drone.action.arm()

    async def takeoff(self, altitude=3):
        print(f"Taking off to {altitude} m...")

        await self.drone.action.set_takeoff_altitude(altitude)
        await self.drone.action.takeoff()

        # Wait until the drone reaches altitude
        await asyncio.sleep(8)

    async def start_offboard(self):
        print("Sending initial setpoint...")

        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(
                0.0,
                0.0,
                0.0,
                0.0
            )
        )

        await asyncio.sleep(0.2)

        print("Starting Offboard...")

        try:
            await self.drone.offboard.start()

        except OffboardError as e:
            print(f"❌ Offboard start failed: {e}")
            await self.drone.action.disarm()
            raise

    async def fly_velocity(
        self,
        north,
        east,
        down,
        yaw,
        seconds
    ):
        print(
            f"Velocity N:{north} "
            f"E:{east} "
            f"D:{down} "
            f"Yaw:{yaw}"
        )

        for _ in range(int(seconds * 20)):
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(
                    north,
                    east,
                    down,
                    yaw
                )
            )

            await asyncio.sleep(0.05)

    # -------------------------
    # Motion Library
    # -------------------------

    async def move_forward(self, speed=1.0, seconds=5):
        await self.fly_velocity(speed, 0.0, 0.0, 0.0, seconds)

    async def move_backward(self, speed=1.0, seconds=5):
        await self.fly_velocity(-speed, 0.0, 0.0, 0.0, seconds)

    async def move_right(self, speed=1.0, seconds=5):
        await self.fly_velocity(0.0, speed, 0.0, 0.0, seconds)

    async def move_left(self, speed=1.0, seconds=5):
        await self.fly_velocity(0.0, -speed, 0.0, 0.0, seconds)

    async def move_up(self, speed=0.5, seconds=3):
        await self.fly_velocity(0.0, 0.0, -speed, 0.0, seconds)

    async def move_down(self, speed=0.5, seconds=3):
        await self.fly_velocity(0.0, 0.0, speed, 0.0, seconds)

    async def rotate_right(self, yaw_rate=30.0, seconds=3):
        await self.fly_velocity(0.0, 0.0, 0.0, yaw_rate, seconds)

    async def rotate_left(self, yaw_rate=30.0, seconds=3):
        await self.fly_velocity(0.0, 0.0, 0.0, -yaw_rate, seconds)

    async def hover(self, seconds=3):
        print("Hovering...")

        for _ in range(int(seconds * 20)):
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(
                    0.0,
                    0.0,
                    0.0,
                    0.0
                )
            )

            await asyncio.sleep(0.05)

    async def land(self):
        print("Stopping Offboard...")

        try:
            await self.drone.offboard.stop()
        except Exception:
            pass

        print("Landing...")

        await self.drone.action.land()

        await asyncio.sleep(10)

        print("✅ Mission Complete")
