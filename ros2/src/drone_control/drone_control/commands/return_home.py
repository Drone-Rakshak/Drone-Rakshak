from drone_control.commands.base_command import MissionCommand


class ReturnHome(MissionCommand):

    async def execute(self, controller):

        print("Returning Home...")

        # For now
        await controller.hover(2)
