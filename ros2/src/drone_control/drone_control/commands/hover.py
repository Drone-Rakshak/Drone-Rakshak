from drone_control.commands.base_command import MissionCommand


class Hover(MissionCommand):

    def __init__(self,
                 seconds=3):

        self.seconds = seconds

    async def execute(self,
                      controller):

        print(
            f"Hover "
            f"{self.seconds}s"
        )

        await controller.hover(
            self.seconds
        )
