from datetime import datetime


class MissionLogger:

    def log(self, message):

        print(
            f"[{datetime.now().strftime('%H:%M:%S')}] {message}"
        )
