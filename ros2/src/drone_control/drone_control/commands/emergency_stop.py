from drone_control.commands.base_command import MissionCommand


class EmergencyStop(MissionCommand):

    async def execute(self, controller):

        print("EMERGENCY STOP!")

        await controller.hover(1)

        await controller.land()
