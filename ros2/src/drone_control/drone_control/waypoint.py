class Waypoint:

    def __init__(
        self,
        action,
        seconds=0,
        speed=None
    ):

        self.action = action
        self.seconds = seconds
        self.speed = speed

    def __str__(self):

        return (
            f"{self.action} "
            f"{self.seconds}s "
            f"{self.speed}"
        )
