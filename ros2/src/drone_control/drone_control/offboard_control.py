import asyncio

from drone_control.controller import DroneController
from drone_control.mission_manager import MissionManager

from drone_control.commands.move_forward import MoveForward
from drone_control.commands.move_right import MoveRight
from drone_control.commands.hover import Hover
from drone_control.commands.return_home import ReturnHome

from drone_control import config


async def run():

    controller = DroneController()

    mission = MissionManager(controller)

    # Connect to PX4
    await controller.connect()

    # Wait until GPS and Home Position are ready
    await controller.wait_until_ready()

    # Arm
    await controller.arm()

    # Takeoff
    await controller.takeoff(
        config.TAKEOFF_ALTITUDE
    )

    # Switch to Offboard mode
    await controller.start_offboard()

    # ----------------------------
    # Mission
    # ----------------------------

    mission.add(
        MoveForward(
            speed=1.0,
            seconds=5
        )
    )

    mission.add(
        Hover(
            seconds=3
        )
    )

    mission.add(
        MoveRight(
            speed=1.0,
            seconds=4
        )
    )

    mission.add(
        Hover(
            seconds=3
        )
    )

    mission.add(
        ReturnHome()
    )

    # Execute Mission
    await mission.execute()

    # Land
    await controller.land()


def main():

    asyncio.run(run())


if __name__ == "__main__":
    main()
