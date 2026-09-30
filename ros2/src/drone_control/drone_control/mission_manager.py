from drone_control.logger import MissionLogger


class MissionManager:

    def __init__(self, controller):

        self.controller = controller

        self.commands = []

        self.logger = MissionLogger()

    def add(self, command):

        self.commands.append(command)

    async def execute(self):

        for command in self.commands:

            self.logger.log(
                command.__class__.__name__
            )

            try:

                await command.execute(
                    self.controller
                )

            except Exception as e:

                self.logger.log(str(e))

                raise
