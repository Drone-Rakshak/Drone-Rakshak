from drone_control.controller import DroneController
from drone_control import config


class MotionLibrary:

    def __init__(self, controller: DroneController):
        self.controller = controller


    async def forward(self):
        await self.controller.move_forward(
            config.DEFAULT_SPEED,
            config.MOVE_TIME
        )


    async def backward(self):
        await self.controller.move_backward(
            config.DEFAULT_SPEED,
            config.MOVE_TIME
        )


    async def left(self):
        await self.controller.move_left(
            config.DEFAULT_SPEED,
            config.MOVE_TIME
        )


    async def right(self):
        await self.controller.move_right(
            config.DEFAULT_SPEED,
            config.MOVE_TIME
        )


    async def up(self):
        await self.controller.move_up(
            0.5,
            3
        )


    async def down(self):
        await self.controller.move_down(
            0.5,
            3
        )


    async def hover(self):
        await self.controller.hover(
            config.DEFAULT_HOVER_TIME
        )


    async def rotate_left(self):
        await self.controller.rotate_left(
            config.YAW_SPEED,
            3
        )


    async def rotate_right(self):
        await self.controller.rotate_right(
            config.YAW_SPEED,
            3
        )
