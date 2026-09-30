from drone_control.commands.base_command import MissionCommand


class MoveRight(MissionCommand):

    def __init__(self,
                 speed=1.0,
                 seconds=5):

        self.speed = speed
        self.seconds = seconds

    async def execute(self,
                      controller):

        print(
            f"Right "
            f"{self.seconds}s"
        )

        await controller.move_right(
            self.speed,
            self.seconds
        )
